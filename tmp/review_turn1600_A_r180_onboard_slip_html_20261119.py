"""Compare onboard position and lateral slip with registered video for the three A R180 trials."""
import argparse
import base64
from html import escape
from io import BytesIO
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
from matplotlib import pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools/analysis"))
from turn_slip_conditions import force_fit, observe
from turn_trajectory_overlay import digest, interpolate_valid

SOURCE_K = 254.90725  # Params/turn_pose_param.h, Mouse A with suction
WINDOWS = (.015, .020, .025)
SIGNATURE = ["ideal.velo", "ego.velo", "ideal.rad_velo", "ego.rad_velo", "V_r", "V_l"]
COLORS = ["#076c7b", "#ad4823", "#685690"]


def read(path):
    return json.loads(path.read_text(encoding="utf-8"))


def rotation(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s], [s, c]])


def onboard_pose(log, settings):
    """Transform the three logged local frames, estimating only their reset ticks."""
    p = log[["ego.turn_x", "ego.turn_y"]].to_numpy() * [-1, 1]
    pattern = log["motion.pattern"].to_numpy()
    yaw = log["ego.radian"].to_numpy()
    rate = log["ideal.rad_velo"].to_numpy()
    pre = np.flatnonzero(pattern == 1)
    turn = np.flatnonzero(pattern == 5)
    pauses = np.flatnonzero(pattern == 30)
    assert np.all(np.isfinite(p)) and np.all(np.diff(log.cnt) == 1)
    assert list(pauses) == [pre[-1] + 1, turn[-1] + 1]
    assert turn[0] == pauses[0] + 1 and np.allclose(p[pauses], 0)
    end = np.flatnonzero((np.arange(len(log)) > np.argmax(abs(rate))) & (rate == 0))[0]
    assert np.allclose(p[end], 0) and pattern[end] == 5 and yaw[end] == 0
    travel = log["ego.length"].to_numpy()
    enter_yaw = yaw[pre[-1]]
    origin_turn = p[pre[-1]] + rotation(enter_yaw) @ [0, travel[pauses[0]]]
    curve = p[pauses[0]:end] @ rotation(enter_yaw).T + origin_turn
    exit_yaw = enter_yaw + yaw[end - 1]
    origin_exit = curve[-1] + (curve[-1] - curve[-2])
    exit_segment = p[end:turn[-1]+1] @ rotation(exit_yaw).T + origin_exit
    last_step = settings["parameters"]["lend"] + travel[pauses[1]] - travel[turn[-1]]
    assert 0 <= last_step <= 3 and 0 <= travel[pauses[0]] <= 3
    exit_end = exit_segment[-1] + rotation(exit_yaw + yaw[turn[-1]]) @ [0, last_step]
    joined = np.vstack([p[pre], curve, exit_segment, exit_end])
    assert len(joined) == len(log) and np.linalg.norm(origin_exit - curve[-1]) < 3
    return joined, end


def logged_k(log):
    # Invert the ideal slip-angle recurrence before conversion to binary16.
    rate = log["ideal.rad_velo"].to_numpy()[1:]
    speed = log["ideal.velo"].to_numpy()[1:]
    beta = log["ideal.turn_slip_theta"].to_numpy()
    core = (abs(rate) > .5 * max(abs(rate))) & (abs(beta[1:]) > .04)
    previous, current = beta[:-1][core], beta[1:][core]
    values = (1000 * (previous - current) - rate[core]) * speed[core] * (1 + previous**2 / 2) / current
    return float(np.median(values))


def load_case(record, comparison_dir):
    trial = Path(record["source_trial"])
    run = trial.parent
    full = run / "full"
    log_file, stream_file = run / "saved_1ms.csv", run / "motion.csv"
    settings_file, motion_settings = run / "saved_1ms.settings.json", run / "motion.settings.json"
    pose_file, geometry_file, report_file = (
        full / name for name in ("apparent_floor_marker_pose.csv", "body_geometry_used.json", "report.json")
    )
    optical_file = comparison_dir / (record["label"] + "_relative_trajectory.csv")
    settings = read(settings_file)
    report = read(report_file)
    motion = read(motion_settings)
    log, stream, pose, optical = map(pd.read_csv, (log_file, stream_file, pose_file, optical_file))
    geometry = read(geometry_file)
    assert np.array_equal(optical.time_video_s, pose.time_video_s)
    origin = motion["log"]["motion_start_time_ms"]
    matches = log.merge(stream[["time_ms"] + SIGNATURE], on=SIGNATURE)
    assert len(matches) >= 25 and matches.cnt.is_unique
    period, offset = np.polyfit(matches.cnt, matches.time_ms, 1)
    assert .95 < period < 1.05
    log_ms = log.cnt.to_numpy() * period + offset - origin
    onboard, end = onboard_pose(log, settings)
    windows = {w: observe(stream, motion, pose, geometry, report, w) for w in WINDOWS}
    observation = windows[.020]
    f = observation.frame
    video_ms = f.log_time_s.to_numpy() * 1000
    valid = optical.detected.to_numpy(bool)
    video = optical[["right_mm", "forward_mm"]].to_numpy()
    video[~valid] = np.nan
    comparison = interpolate_valid(log_ms / 1000, onboard, video_ms / 1000)
    distance = np.linalg.norm(video - comparison, axis=1)
    core = f.core.to_numpy(bool)
    first = observation.diagnostics["turn_first_s"] * 1000
    last = observation.diagnostics["turn_last_s"] * 1000
    rotation_frames = (video_ms >= first) & (video_ms <= last)
    turn = (video_ms >= first) & (video_ms <= last) & np.isfinite(distance)
    assert turn.sum() >= 10
    video_lend = interpolate_valid(video_ms[valid] / 1000, video[valid], [log_ms[-1] / 1000])[0]
    lend_shift = interpolate_valid(video_ms[valid] / 1000, video[valid], (log_ms[-1] + np.array([-5,5])) / 1000)
    if np.isfinite(np.vstack([video_lend,lend_shift])).all():
        lend_result, lend_range = video_lend.tolist(), sorted(lend_shift[:,1].tolist())
    else:
        lend_result, lend_range = None, None

    k = logged_k(log)
    assert abs(k - SOURCE_K) < 4
    hand = observation.diagnostics["frame_handedness_vs_left_positive_gyro"]
    video_beta = f.beta_rad.to_numpy() * hand
    log_beta = log["ego.turn_slip_theta"].to_numpy()
    ideal_beta = log["ideal.turn_slip_theta"].to_numpy()
    aligned_beta = np.interp(video_ms[core], log_ms, log_beta)
    stats = dict(
        label=record["label"], logged_k_m_s2=k,
        rotation_deg=record["entry_to_exit_path_rotation_deg"],
        return_spacing_mm=record["exit_line_right_at_forward45_mm"],
        apex_video_mm=record["forward_apex_mm"], apex_onboard_mm=float(onboard[:end,1].max()),
        motion_model_rmse_mm=record["encoder_model_motion_rmse_mm"],
        combined_relative_review_passed=record["relative_review_eligible"],
        optical_rate=record["motion_observation_rate"],
        optical_turn_detected_frames=int((rotation_frames & valid).sum()),
        optical_turn_all_frames=int(rotation_frames.sum()),
        onboard_video_turn_rmse_mm=float(np.sqrt(np.mean(distance[turn]**2))),
        onboard_video_turn_common_frames=int(turn.sum()), timed_video_lend_right_forward_mm=lend_result,
        onboard_lend_right_forward_mm=onboard[-1].tolist(), video_lend_forward_plus_minus_5ms_mm=lend_range,
        slip_video_core_deg=float(np.degrees(np.median(video_beta[core]))),
        slip_onboard_core_deg=float(np.degrees(np.median(aligned_beta))),
        slip_video_lateral_forward_pct=float(100*np.median(np.tan(video_beta[core]))),
        slip_onboard_lateral_forward_pct=float(100*np.median(np.tan(aligned_beta))),
        force={str(w):obs.diagnostics["force"] for w,obs in windows.items()},
        sync=dict(gyro_diagnostic=observation.diagnostics, log_period_ms=float(period),
                  match_count=len(matches), label="Shared optical-heading alignment"),
        source_sha256={str(p):digest(p) for p in (trial, log_file, stream_file, settings_file, motion_settings,
                                                 pose_file, geometry_file, report_file, optical_file)})
    return dict(stats=stats, log_ms=log_ms, video_ms=video_ms, onboard=onboard, end=end, video=video,
                video_beta=video_beta, log_beta=log_beta, ideal_beta=ideal_beta, first=first,last=last,
                distance=distance, windows=windows)


def predict_force(cases):
    """Compare the current coefficient with coefficients fitted on other trials."""
    def fit_inputs(cases_to_fit, window):
        return pd.concat([c["windows"][window].frame for c in cases_to_fit], ignore_index=True)

    for i, case in enumerate(cases):
        evaluated = []
        for w in WINDOWS:
            train = fit_inputs([c for j,c in enumerate(cases) if i != j], w)
            fitted = force_fit(train.averaged_beta_rad,train.imu_left_m_s2,train.fit)
            test = case["windows"][w].frame
            beta, ay = (test.loc[test.fit,k].to_numpy() for k in ("averaged_beta_rad","imu_left_m_s2"))
            assert len(ay) >= 8 and np.isfinite([beta,ay]).all()
            rmse = lambda k: float(np.sqrt(np.mean((-k*beta-ay)**2)))
            evaluated.append(dict(window_ms=w*1000,samples=len(ay),k_other_runs_m_s2=fitted["k_m_s2"],
                                  current_rmse_m_s2=rmse(SOURCE_K),heldout_rmse_m_s2=rmse(fitted["k_m_s2"])))
        case["stats"]["heldout_force"] = evaluated
    pooled = {}
    for w in WINDOWS:
        frame = fit_inputs(cases, w)
        pooled[str(w)] = force_fit(frame.averaged_beta_rad,frame.imu_left_m_s2,frame.fit)
    return pooled


def image(fig, alt):
    output = BytesIO()
    fig.savefig(output, format="svg", bbox_inches="tight")
    plt.close(fig)
    return f'<img alt="{escape(alt)}" src="data:image/svg+xml;base64,{base64.b64encode(output.getvalue()).decode()}">'


def render_traces(cases):
    fig, axes = plt.subplots(1, len(cases), figsize=(10.8, 5), sharex=True, sharey=True, layout="constrained")
    for i, (ax,case) in enumerate(zip(axes,cases),1):
        log, video, end = (case[k] for k in ("onboard","video","end"))
        keep = (case["video_ms"] >= -30) & (case["video_ms"] <= case["log_ms"][-1]+45)
        ax.plot(*video[keep].T,color=COLORS[0],lw=2.1,label="Video marker")
        ax.plot(*log[:end].T,color=COLORS[1],lw=1.8,label="Onboard pose")
        ax.plot(*log[end-1:].T,color=COLORS[1],lw=1.5,ls="--",label="Joined Lend")
        ax.scatter(*log[-1],c=COLORS[1],s=27,marker="s",zorder=3)
        v_end = case["stats"]["timed_video_lend_right_forward_mm"]
        if v_end is not None:
            ax.scatter(*v_end,c=COLORS[0],s=24,zorder=3)
        ax.set(xlabel="Right [mm]",title=f"Run {i}",xlim=(-8,102),ylim=(-8,194),aspect="equal")
        ax.grid(alpha=.23)
    axes[0].set_ylabel("Forward from start [mm]")
    axes[-1].legend(loc="lower center",fontsize=8)
    return image(fig,"試行ごとの動画と内部位置の軌跡")


def render_slip(cases):
    fig, axes = plt.subplots(len(cases), 2, figsize=(11,7.4), layout="constrained",sharex="col")
    for i,(case,row) in enumerate(zip(cases,axes),1):
        mcu, video = case["log_ms"], case["video_ms"]
        keep = (video >= case["first"]-25) & (video <= case["last"]+25)
        vk = (mcu >= case["first"]-25) & (mcu <= case["last"]+25)
        plots = [(video[keep],np.degrees(case["video_beta"][keep]),"Video + yaw reference"),
                 (mcu[vk],np.degrees(case["log_beta"][vk]),"Onboard estimate"),
                 (mcu[vk],np.degrees(case["ideal_beta"][vk]),"Onboard target model")]
        for (t,beta,name),color in zip(plots,COLORS):
            row[0].plot(t,beta,lw=1.8,color=color,label=name)
        row[0].axvspan(case["first"],case["last"],color="grey",alpha=.10)
        frame=case["windows"][.020].frame
        b,ay=(frame.loc[frame.fit,k].to_numpy() for k in ("averaged_beta_rad","imu_left_m_s2"))
        ordered=np.argsort(b)
        candidate=case["stats"]["heldout_force"][1]["k_other_runs_m_s2"]
        row[1].scatter(np.degrees(b),ay,s=14,color=COLORS[0],label="Video beta / measured IMU")
        row[1].plot(np.degrees(b[ordered]),-SOURCE_K*b[ordered],color=COLORS[1],label="Current K")
        row[1].plot(np.degrees(b[ordered]),-candidate*b[ordered],color=COLORS[2],label="K from other runs")
        row[0].set_ylabel(f"Run {i} | beta [deg]")
        row[1].set_ylabel("Lateral accel. [m/s²]")
        for ax in row:
            ax.grid(alpha=.20)
    axes[-1,0].set_xlabel("MCU time from motion onset [ms]")
    axes[-1,1].set_xlabel("20-tick averaged video beta [deg]")
    axes[0,0].legend(loc="upper left",fontsize=7.7)
    axes[0,1].legend(loc="best",fontsize=7.7)
    return image(fig,"動画の横滑り角、内部モデル、横加速度の比較")


def table(head,rows):
    header = "".join(f"<th scope='col'>{escape(h)}</th>" for h in head)
    body = "".join("<tr>"+"".join(f"<td>{escape(str(cell))}</td>" for cell in row)+"</tr>" for row in rows)
    return f"<div class='scroll'><table><thead><tr>{header}</tr></thead><tbody>{body}</tbody></table></div>"


def render_html(cases, pooled):
    stats = [c["stats"] for c in cases]
    paths = table(["試行","旋回角 °","折返し幅 mm","動画の頂点 mm","内部の頂点 mm","内部位置・動画の旋回中RMSE mm","動画・速度積分の照合 mm"],
        [[i,f'{s["rotation_deg"]:.2f}',f'{s["return_spacing_mm"]:.2f}',f'{s["apex_video_mm"]:.2f}',f'{s["apex_onboard_mm"]:.2f}',
          f'{s["onboard_video_turn_rmse_mm"]:.2f}',f'{s["motion_model_rmse_mm"]:.2f} / '+("基準内" if s["combined_relative_review_passed"] else "要確認")]
         for i,s in enumerate(stats,1)])
    quality = table(["試行","評価窓のマーカー観測率","旋回中の有効フレーム"],
        [[i,f'{s["optical_rate"]:.1%}',f'{s["optical_turn_detected_frames"]}/{s["optical_turn_all_frames"]}']
         for i,s in enumerate(stats,1)])
    slip = table(["試行","動画 β °","内部 β °","横/前速度比 動画 / 内部","K 逆算 m/s²","K 別2本で推定 m/s²","横加速度RMSE 現行 → 検証 m/s²"],
        [[i,f'{s["slip_video_core_deg"]:.2f}',f'{s["slip_onboard_core_deg"]:.2f}',
          f'{s["slip_video_lateral_forward_pct"]:.1f}% / {s["slip_onboard_lateral_forward_pct"]:.1f}%',
          f'{s["logged_k_m_s2"]:.2f}',f'{s["heldout_force"][1]["k_other_runs_m_s2"]:.2f}',
          f'{s["heldout_force"][1]["current_rmse_m_s2"]:.2f} → {s["heldout_force"][1]["heldout_rmse_m_s2"]:.2f}']
         for i,s in enumerate(stats,1)])
    lend = table(["試行","内部 Lend 終端 前方 mm","動画 Lend 終端 前方 mm","動画の±5 ms感度 mm"],
        [[i,f'{s["onboard_lend_right_forward_mm"][1]:.1f}',
          f'{s["timed_video_lend_right_forward_mm"][1]:.1f}' if s["timed_video_lend_right_forward_mm"] else "欠測",
          "～".join(f"{x:.1f}" for x in s["video_lend_forward_plus_minus_5ms_mm"]) if s["video_lend_forward_plus_minus_5ms_mm"] else "欠測"]
         for i,s in enumerate(stats,1)])
    coef = pooled[str(.020)]["k_m_s2"]
    k_bounds = [p["k_m_s2"] for p in pooled.values()]
    return f"""<!doctype html>
<html lang="ja"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>機体A turn1600 右180°の比較</title><style>
* {{box-sizing:border-box}} body {{margin:0;color:#192734;background:#edf3f5;font:16px/1.65 system-ui,sans-serif}}
main {{max-width:1160px;margin:auto;padding:28px 16px 44px}} header,section {{background:white;border:1px solid #cbd8df;border-radius:12px;padding:clamp(14px,3vw,30px);margin-bottom:18px}}
h1,h2,h3 {{line-height:1.26; margin-top:0;color:#112f42}} h1 {{font-size:clamp(1.55rem,4vw,2.35rem)}} h2 {{font-size:1.4rem}}
p {{max-width:84ch}} img {{width:100%;height:auto;display:block}} figure {{margin:20px 0}} figcaption {{font-size:.88rem;color:#435568}}
.scroll {{overflow-x:auto}} table {{border-collapse:collapse;width:100%;font-size:.9rem;font-variant-numeric:tabular-nums}}
th,td {{padding:9px 12px;text-align:right;border-bottom:1px solid #dbe4e9;white-space:nowrap}} th {{background:#e7f0f3}}
summary {{cursor:pointer;font-weight:650}} details {{margin-top:20px}} a {{color:#075f73}}
</style></head><body><main>
<header><h1>機体A turn1600 右180°の比較</h1>
<p>3走行とも 1.6 m/s、半径 −45 mm、補正 +2.0°、Lstart 13.12 mm、Lend 27.38 mm、吸引 duty 650。</p>
<p>動画・横加速度から求めた滑り係数 K は <strong>{coef:.1f} m/s²</strong>。動画の微分窓を15～25 msに変えたときの範囲は {min(k_bounds):.1f}～{max(k_bounds):.1f} m/s²。</p></header>
<section><h2>動画と機体内部の自己位置</h2>
<p>進入直線基準。原点をそろえ、尺度は固定。目標は旋回角180°、折返し幅90 mm、頂点185 mm。</p>
{paths}<figure>{render_traces(cases)}<figcaption>内部位置は1 msの ego.turn_x/y。方向を統一し、ログ原点のリセット点を区間ごとに接続。破線はジャイロ角でつないだLend区間。丸と四角は推定したLend終端。</figcaption></figure>
<details><summary>動画の観測率と照合誤差</summary>{quality}
<p>全評価窓のマーカー観測率は95%以上が基準。3本目は復路側に欠測あり。速度・ジャイロ積分と動画の照合基準は8.0 mm。3本とも照合誤差が基準を超えており、位置差は診断値。</p></details>
<details><summary>Lend終端の時刻感度</summary>{lend}
<p>動画側は全試行で同じ手順の姿勢・ジャイロ照合を使用。±5 msの時刻ずれに対する範囲を表示。接続点の1周期分は走行距離または隣接点で補間した。</p></details></section>
<section><h2>横滑り角と内部モデルの滑り係数</h2>
<p>β は進行方向と機体の向きの差。横/前速度比は tan(β) × 100。代表値は目標角速度がピークの50%を超えた区間の中央値。</p>
{slip}<figure>{render_slip(cases)}<figcaption>右の横加速度比較は20 msの微分半窓を使用。動画のβをファームの20点平均に合わせ、a<sub>y</sub> = −Kβ の残差で比較。</figcaption></figure>
<p>現行 K は機体A・吸引時の254.91 m/s²。各試行の「別2本」は、その試行を除いて推定した K を用いた横加速度の検証。K の採用には制御器での位置推定・軌道の再検証が必要。</p>
<details><summary>入力と評価の範囲</summary>
<p>動画は各フレームでArUco登録したマーカー中点。車軸中心との対応は近似。内部の位置は機体が採用した推定で、EKFの周期ごとの採否は記録外。βの取得には動画とジャイロを併用。独立した時刻同期は未取得。</p>
<p><a href="video_comparison.json">動画基準の3本比較</a> · <a href="onboard_slip_report.json">比較値・入力ハッシュ</a></p>
</details></section></main></body></html>"""


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("comparison_dir",type=Path)
    args=parser.parse_args()
    target=args.comparison_dir.resolve()
    source=read(target/"video_comparison.json")
    assert source["repeat_count"] == 3 and source["identical_device_settings_verified"]
    cases=[load_case(record,target) for record in source["comparisons"]]
    pooled=predict_force(cases)
    report=dict(mouse="A",motion="long_r180",speed_mm_s=1600,radius_mm=-45,correction_deg=2,
                source_k=SOURCE_K,pooled_force=pooled,cases=[c["stats"] for c in cases],
                source_script_sha256=digest(Path(__file__)),
                source_comparison_sha256=digest(target/"video_comparison.json"),
                fitted_parameter_scope="Effective lateral-force coefficient for this geometry and duty. Offline force validation only.")
    page=render_html(cases,pooled)
    with (target/"onboard_slip_report.json").open("w",encoding="utf-8") as output:
        json.dump(report,output,indent=2,allow_nan=False)
        output.write("\n")
    with (target/"compare.html").open("w",encoding="utf-8") as output:
        output.write(page)
    print(f'{target / "compare.html"}')


if __name__ == "__main__":
    main()
