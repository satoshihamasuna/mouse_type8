"""Validate turn slip candidates on captured 1 ms firmware inputs and optical tracks."""
import argparse
import json
from pathlib import Path
import subprocess

import matplotlib
matplotlib.use("Agg")
from matplotlib import pyplot as plt
import numpy as np
import pandas as pd

from review_turn1600_A_r180_onboard_slip_html_20261119 import (
    ROOT, SOURCE_K, COLORS, digest, image, interpolate_valid, load_case, read, table,
)

CORE_FRACTION = .02
SIGMA = 7.
RIGHT_PULSE = "Encoder_GetProperty_Right().sp_pulse"
LEFT_PULSE = "Encoder_GetProperty_Left().sp_pulse"
INPUTS = ["irq.period_us", "irq.total_us", "ego.velo", "ego.accel", "ego.rad_velo",
          "ego.horizon_accel.get", "ideal.velo", "ego.radian", RIGHT_PULSE, LEFT_PULSE]


def input_rows(record, case):
    run = Path(record["source_trial"]).parent
    log = pd.read_csv(run/"saved_1ms.csv")
    stream = pd.read_csv(run/"motion.csv")
    motion = read(run/"motion.settings.json")
    duty = motion["suction"]["duty"]
    assert duty == 650 and motion["suction"]["enabled"]
    dt = log["irq.period_us"].median()*1e-6
    start = motion["log"]["motion_start_time_ms"]
    # Retained 1 ms data starts with pre-run. Only the quiet lead-in is at 5 ms.
    before = stream.loc[(stream.time_ms < start+case["log_ms"][0]-.6) & (stream["motion.pattern"] == 28)]
    assert len(before) >= 65
    pre_dt = before.time_ms.diff().div(1000).fillna(.005).to_numpy()
    assert np.all((pre_dt >= .004) & (pre_dt <= .006))
    pre = before[INPUTS].to_numpy()
    quiet = (abs(before["ideal.velo"]) < .01) & (abs(before["ideal.rad_velo"]) < .05)
    pre_rows = np.column_stack([pre, pre_dt, np.full(len(pre),duty),before["motion.pattern"],
                                np.zeros(len(pre)),quiet,np.zeros((len(pre),3))])

    rate = log["ideal.rad_velo"].to_numpy()
    pattern = log["motion.pattern"].to_numpy()
    first = np.flatnonzero((pattern==5) & (rate<-.001))[0]
    last = case["end"]
    assert first+115 < last < first+130 and rate[last] == 0.
    turning = log.cnt.between(first,last)
    motion_start = log.cnt == 0
    starts = log.loc[motion_start,"cnt"]
    assert len(starts) == 1 and starts.iloc[0] == 0
    retained = log[INPUTS].copy()
    # The tracker runs in preprocess. Command and ego.radian from postprocess of
    # the preceding tick match the state at the start of its current tick.
    retained["ideal.velo"] = log["ideal.velo"].shift(fill_value=0)
    retained["ego.radian"] = log["ego.radian"].shift(fill_value=0)
    retained = retained.fillna(-1)
    rows = np.column_stack([retained,np.full(len(log),dt),np.full(len(log),duty),pattern,turning,
                            (abs(retained["ideal.velo"])<.01) & (abs(log["ideal.rad_velo"].shift(fill_value=0))<.05),
                            motion_start,np.zeros(len(log)),np.ones(len(log))])
    actual = np.vstack([pre_rows,rows])
    nominal = actual.copy()
    nominal[-len(rows):,-2] = turning

    # Check two identities on the stored firmware outputs independently of
    # the replay's inference of the rotation state and stationary calibration.
    beta = log["ego.turn_slip_theta"]
    v = log["ego.velo"]
    gyro = log["ego.rad_velo"]
    core = (pattern==5) & (abs(rate)>CORE_FRACTION*max(abs(rate)))
    legacy_beta = (1000*beta.shift()-gyro)/(1000+SOURCE_K/v)
    beta_residual = (beta-legacy_beta)[core]
    encoder_mm = (log[RIGHT_PULSE]-log[LEFT_PULSE])*15.02*np.pi/(2*172)
    legacy_lateral = encoder_mm*beta
    ekf_lateral = v/(log["irq.period_us"]/1000)*np.tan(beta)
    y = log["ego.horizon_velo.get"]
    rmse = lambda x: float(np.sqrt(np.mean((y[core]-x[core])**2)))
    bad = log.loc[turning & (log["irq.total_us"]>=.85*log["irq.period_us"]),
                  ["cnt","irq.total_us","irq.period_us","irq.logging_us"]]
    first_bad = bad.iloc[0].to_dict()
    first_bad["cnt"] = int(first_bad["cnt"])
    stat = dict(first_turn_cnt=int(first),last_turn_cnt=int(last),turn_count=int(turning.sum()),
                pre_roll_rows_5ms=len(before),first_invalid_irq=first_bad,invalid_irq_count=len(bad),
                core_legacy_beta_max_residual_rad=float(abs(beta_residual).max()),
                core_lateral_velocity_rmse_legacy_m_s=rmse(legacy_lateral),
                core_lateral_velocity_rmse_ekf_m_s=rmse(ekf_lateral),
                stationary_gyro_mad_rad_s=case["stats"]["sync"]["gyro_diagnostic"]["stationary_gyro_mad_rad_s"])
    return actual, nominal, turning.to_numpy(), stat


def replay(exe, input_file, directory, name, k, std, count, legacy=False):
    target = directory / (name+".csv")
    subprocess.run([str(exe),str(input_file),str(target),str(k),str(std),str(int(legacy))],
                   cwd=directory, check=True, timeout=20)
    return pd.read_csv(target).tail(count).reset_index(drop=True)


def score(case, replayed, turning):
    positions = replayed[["left_mm","forward_mm"]].to_numpy()*[-1,1]
    video = case["video"]
    tv = case["video_ms"]/1000
    tl = case["log_ms"]/1000
    inside = (tv >= case["first"]/1000) & (tv <= case["last"]/1000)
    valid = np.isfinite(video).all(axis=1)
    v_end = case["stats"]["timed_video_lend_right_forward_mm"]
    rmse = []
    for shift in (-.005,0.,.005):
        compared = interpolate_valid(tl, positions, tv+shift)
        both = inside & valid & np.isfinite(compared).all(axis=1)
        assert both.sum() >= 20
        error = np.linalg.norm(video[both]-compared[both],axis=1)
        rmse.append(float(np.sqrt(np.mean(error**2))))
    beta = interpolate_valid(tl, replayed[["beta_rad"]].to_numpy(), tv)[:,0]
    core = case["windows"][.020].frame.core.to_numpy(bool)
    assert np.isfinite(beta[core]).all()
    applied = np.interp(tv[core],tl,replayed.applied.to_numpy()) > .99
    summary = dict(turn_rmse_mm=rmse[1],turn_rmse_plus_minus_5ms_mm=[min(rmse),max(rmse)],
                   turn_rmse_by_time_shift_mm=dict(zip(("-5","0","+5"),rmse)),
                   apex_mm=float(positions[:,1].max()),
                   Lend_end_right_forward_mm=positions[-1].tolist(),
                   Lend_error_mm=float(np.linalg.norm(positions[-1]-v_end)) if v_end is not None else None,
                   replayed_core_beta_deg=float(np.degrees(np.median(beta[core][applied]))) if applied.any() else None,
                   applied_core_video_frames=int(applied.sum()),
                   applied_cycles=int(replayed.loc[turning,"applied"].sum()),
                   reasons=replayed.loc[turning,"reason"].value_counts().to_dict(),
                   updates=int(replayed.loc[turning,"updates"].max()),
                   rejected=int(replayed.loc[turning,"rejected"].max()),
                   bias_ready=bool(replayed.loc[turning,"bias_ready"].any()),
                   sigma_beta_max_deg=float(np.degrees(replayed.loc[turning,"sigma_beta_rad"].max())),
                   nis_max=float(replayed.loc[turning,"nis"].max()))
    return positions, summary


def figure(cases):
    fig,axes=plt.subplots(1,3,figsize=(10.8,4.8),sharex=True,sharey=True,layout="constrained")
    for i,(c,ax) in enumerate(zip(cases,axes),1):
        source=c["source"]
        keep=(source["video_ms"]>=-15)&(source["video_ms"]<=source["log_ms"][-1]+18)
        ax.plot(*source["video"][keep].T,color=COLORS[0],lw=2,label="Video")
        ax.plot(*c["tracks"]["recorded_current"].T,color="#5a6670",lw=1.7,label="Recorded timing")
        ax.plot(*c["tracks"]["stable_current"].T,color=COLORS[1],lw=1.6,label="Stable timing, current K")
        ax.plot(*c["tracks"]["stable_heldout"].T,color=COLORS[2],lw=1.6,label="Stable timing, held-out K")
        ax.set(xlabel="Right [mm]",title=f"Run {i}",xlim=(-8,103),ylim=(-5,194),aspect="equal")
        ax.grid(alpha=.2)
    axes[0].set_ylabel("Forward from start [mm]")
    axes[-1].legend(loc="lower center",fontsize=7)
    return image(fig,"3本の動画、記録周期、および周期安定を仮定した再生の位置比較")


def html(cases, last_results):
    timing=table(["試行","最初に不適用になったログ点","処理時間 / 周期 µs","うちログ処理 µs","採用周期 現行 / 候補"],
        [[i,c["first_invalid_irq"]["cnt"],f'{c["first_invalid_irq"]["irq.total_us"]:.0f} / {c["first_invalid_irq"]["irq.period_us"]:.1f}',
          f'{c["first_invalid_irq"]["irq.logging_us"]:.1f}',
          f'{c["methods"]["recorded_current"]["applied_cycles"]} / {c["methods"]["recorded_heldout"]["applied_cycles"]}（全{c["turn_count"]}周期）']
         for i,c in enumerate(last_results,1)])
    rows=[]
    for i,c in enumerate(last_results,1):
        m=c["methods"]
        for name,label in (("recorded_current","記録周期・現行"),("stable_current","周期安定・現行"),
                           ("stable_heldout","周期安定・別2本のK"),("stable_heldout_std6","同・観測標準偏差6"),
                           ("stable_heldout_std8","同・観測標準偏差8")):
            a=m[name]
            early,late=a["turn_rmse_plus_minus_5ms_mm"]
            k=SOURCE_K if name.endswith("current") else c["K_heldout_m_s2"]
            rows.append([i,label,f'{k:.2f}',f'{a["turn_rmse_mm"]:.2f}',f"{early:.2f}～{late:.2f}",
                         f'{a["apex_mm"]:.2f}',f'{a["Lend_error_mm"]:.2f}' if a["Lend_error_mm"] is not None else "欠測",
                         f'{a["replayed_core_beta_deg"]:.2f}' if a["replayed_core_beta_deg"] is not None else "適用なし",
                         f'{a["applied_cycles"]}/{c["turn_count"]}',f'{a["updates"]}/{a["rejected"]}'])
    metric=table(["試行","再生条件","K m/s²","旋回中RMSE mm","同期±5 msでの範囲 mm","頂点 mm","Lend誤差 mm","旋回中のEKF β °","採用周期","観測更新 / 棄却"],rows)
    return f"""<!doctype html>
<html lang="ja"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>機体A 右180°の滑り係数候補と因果再生</title><style>
* {{box-sizing:border-box}} body {{margin:0;background:#edf3f5;color:#192734;font:16px/1.65 system-ui,sans-serif}}
main {{max-width:1160px;margin:auto;padding:24px 16px}} section {{background:white;padding:clamp(14px,3vw,29px);margin-bottom:18px;border:1px solid #cad8df;border-radius:12px}}
h1,h2 {{margin:0 0 .5em;color:#122f41;line-height:1.3}} h1 {{font-size:clamp(1.6rem,4vw,2.15rem)}} h2 {{font-size:1.35rem}}
p {{max-width:84ch}} img {{width:100%;height:auto}} figure {{margin:20px 0}} figcaption {{font-size:.9rem;color:#42576a}}
.scroll {{overflow:auto}} table {{width:100%;border-collapse:collapse;font-variant-numeric:tabular-nums;font-size:.9rem}}
td,th {{white-space:nowrap;text-align:right;padding:8px 12px;border-bottom:1px solid #d9e4e9}} th {{background:#e7f0f4}}
a {{color:#075f73}}
</style></head><body><main>
<section><h1>機体A 右180°の滑り係数候補</h1>
<p>turn1600、半径−45 mm、補正＋2.0°、吸引 duty 650。現行 K 254.91 m/s²、3本を合わせた横加速度モデルの候補 K 191.34 m/s²。位置の比較は試行を除外して求めた K を使用。</p></section>
<section><h2>記録した周期での因果再生</h2>
{timing}
<p>各ログ点の割り込み時間は直前の割り込みを表す。現行条件は処理時間が周期の85%未満。3本とも旋回の初期にこの条件を外れ、残りの旋回でEKFが中断された。記録の横速度も従来式と一致した。</p></section>
<section><h2>周期が安定した場合の感度</h2>
<p>比較のため、旋回の入力だけを記録周期の中央値で与えてタイミングを有効とした。K以外の条件は揃え、横加速度の観測標準偏差6・7・8 m/s²の感度も確認した。</p>
<figure>{figure(cases)}<figcaption>進入相対座標。記録周期の灰色線は従来位置積分とほぼ重なる。各時刻と原点は4方式で共通。動画の欠測は途切れとして表示。</figcaption></figure>
<p>基準の時刻合わせでは3本とも現行Kの位置RMSEが小さかった。別2本で求めたKを使うと動画のβに近づき、頂点をさらに先に推定した。時刻合わせをずらすと位置誤差の大小が変わる試行もある。観測標準偏差を6～8 m/s²で変えた場合、候補Kでの旋回中RMSEの差は最大0.02 mm未満だった。</p>
{metric}
<p>自己位置の再生には実C++のTrackerとPoseFramesを使用。静止校正は5 msの配信ログ、走行中は保存済み1 msログ。旋回開始は指令角速度から復元。周期安定条件は診断用の仮定で、実機での割り込み時間の改善や走行を確認した結果とは区別する。</p>
<p>動画は登録済みマーカー中点と姿勢–ジャイロ照合を使用。復路の欠測と時刻合わせにより、Lend終端の誤差は参考値。Kと観測雑音の採用を判断するには、安定周期で再度検証する。</p>
<p><a href="report.json">計算結果・入力ハッシュ</a> · <a href="../corr2p0_radius45_repeatability_03/compare.html">3走行の元比較</a></p>
</section></main></body></html>"""


def main():
    arg=argparse.ArgumentParser(description=__doc__)
    arg.add_argument("comparison",type=Path)
    arg.add_argument("output",type=Path)
    args=arg.parse_args()
    comparison=args.comparison.resolve()
    out=args.output.resolve()
    exe=out/"replay.exe"
    previous=read(comparison/"onboard_slip_report.json")
    geometry=read(comparison/"video_comparison.json")
    cases=[]
    result=[]
    for actual,prior in zip(geometry["comparisons"],previous["cases"]):
        assert actual["label"]==prior["label"]
        source=load_case(actual,comparison)
        measured,nominal,turning,summary=input_rows(actual,source)
        k=prior["heldout_force"][1]["k_other_runs_m_s2"]
        d=out/prior["label"]
        d.mkdir(exist_ok=True)
        observed_input,stabilized_input=(d/f"{mode}.txt" for mode in ("recorded_timing","stable_timing"))
        np.savetxt(observed_input,measured,fmt="%.9g")
        np.savetxt(stabilized_input,nominal,fmt="%.9g")
        options=[("legacy",observed_input,0,SIGMA,True),
                 ("recorded_current",observed_input,0,SIGMA,False),
                 ("recorded_heldout",observed_input,k,SIGMA,False),
                 ("stable_current",stabilized_input,0,SIGMA,False),
                 ("stable_heldout",stabilized_input,k,SIGMA,False),
                 ("stable_heldout_std6",stabilized_input,k,6.,False),
                 ("stable_heldout_std8",stabilized_input,k,8.,False)]
        statistics={}
        tracks={}
        for name,inputs,coefficient,sigma,legacy in options:
            estimates=replay(exe,inputs,d,name,coefficient,sigma,len(turning),legacy)
            tracks[name],statistics[name]=score(source,estimates,turning)
        for mode in ("recorded_current","recorded_heldout"):
            s=statistics[mode]
            assert s["applied_cycles"] <= 1 and s["applied_core_video_frames"] == 0
            assert np.max(np.linalg.norm(tracks["legacy"]-tracks[mode],axis=1)) < .2
        observed_error=np.linalg.norm(tracks["legacy"]-source["onboard"],axis=1)
        summary["fallback_vs_logged_pose_rmse_mm"]=float(np.sqrt(np.mean(observed_error**2)))
        result.append(dict(label=prior["label"],source_trial=actual["source_trial"],K_heldout_m_s2=k,
                           **summary,methods=statistics))
        cases.append(dict(source=source,tracks=tracks))
    inputs=[comparison/"onboard_slip_report.json",comparison/"video_comparison.json",
            ROOT/"Params/turn_pose_param.h",ROOT/"Task/Inc/turn_pose_ekf.h",ROOT/"Module/Inc/pose_frames.h",
            ROOT/"Module/Src/interrupt.cpp",ROOT/"Module/Inc/interrupt_timing.h",ROOT/"Module/Src/log_data.cpp",
            Path(__file__).resolve(),ROOT/"tmp/replay_r180_candidate_20261119.cpp"]
    report=dict(mouse="A",motion="long_r180",speed_mm_s=1600,duty=650,radius_mm=-45.,
                current_k_m_s2=SOURCE_K,force_candidate_k_m_s2=previous["pooled_force"]["0.02"]["k_m_s2"],
                holdout="A different K is trained on the other two runs for each replay",
                use_as_firmware_parameter=False,results=result,
                assumptions=["5 ms pre-roll for stationary calibration; PWM setpoint assumed stable",
                             "No inherited heading with wall EKF shadow; turning state inferred from command",
                             "Same optical origin/time transform for every replay; no endpoint fit",
                             "Stable timing is a counterfactual, using the median recorded period during rotation.",
                             "Observation-noise sensitivity tests 6/7/8 m/s², with the source gyro/bias noise priors."],
                input_sha256={str(p):digest(p) for p in inputs},
                decision="Keep the force-model candidate separate from firmware adoption. Re-establish sustained eligibility and compare against optical pose before selecting a coefficient.")
    (out/"report.json").write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+"\n",encoding="utf-8")
    (out/"compare.html").write_text(html(cases,result),encoding="utf-8")
    print(out/"compare.html")


if __name__=="__main__":
    main()
