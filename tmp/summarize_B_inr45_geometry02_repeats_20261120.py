"""同一設定のB1600 inR45を集計する。機器操作・採用値への反映は行わない。"""
from pathlib import Path
import argparse
import hashlib
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr45_20261120_v1'
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--count',type=int,choices=(2,3),required=True)
args=parser.parse_args()
OUT=BASE/f'geometry02_{args.count}run_review_v1';OUT.mkdir(exist_ok=False)
read=lambda p:json.loads(p.read_text(encoding='utf-8'))
cases=[('geometry02','B_INR45_1600_geometry02_green_20261120_006'),
    ('geometry02_repeat_01','B_INR45_1600_geometry02_repeat01_green_20261120_007'),
    ('geometry02_repeat_02','B_INR45_1600_geometry02_repeat02_green_20261120_008')][:args.count]
rows=[];reference_settings=None
plt.rcParams['font.family']='Yu Gothic'
fig,axes=plt.subplots(1,2,figsize=(11,7),layout='constrained')
for number,(stage,identifier) in enumerate(cases,1):
    run=BASE/stage/'local_execution'/('run_'+identifier)
    settings=read(run/'saved_1ms.settings.json');geom=read(run/'midpoint_raw_wheel_clock_v3/midpoint_review.json')
    track=read(run/'tracking_1ms_review_v3/verification.json')
    operator=read(run/'operator_after_run.json')
    assert geom['independent_optical_geometry_eligible'] and operator['both_powered_off'] and operator['no_contact']
    selected={key:settings[key] for key in ('parameters','pid','feedforward','suction','turn_count')}
    if reference_settings is None:reference_settings=selected
    assert selected==reference_settings
    row=dict(number=number,stage=stage,request_id=identifier,source_trial=str(run/'trial.json'),settings=settings,
        exit_angle_deg=geom['exit_path_direction_change_deg'],Lend_right_forward_mm=geom['midpoint_Lend_right_forward_mm'],
        Lend_residual_mm=geom['midpoint_Lend_residual_mm'],Lend_residual_norm_mm=float(np.linalg.norm(geom['midpoint_Lend_residual_mm'])),
        required_motion_quality=geom['independent_clock_required_motion_quality'],clock=geom['clock'],
        late_stationary=geom['late_stationary'],tracking=track,operator=operator,
        original_summary=read(run/'full/summary.json'),
        source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (
            run/'trial.json',run/'saved_1ms.csv',run/'saved_1ms.settings.json',run/'midpoint_raw_wheel_clock_v3/midpoint_review.json')})
    rows.append(row)
    pose=pd.read_csv(run/'midpoint_raw_wheel_clock_v3/midpoint_relative_trajectory.csv')
    xy=pose[['right_mm','forward_mm']].to_numpy();valid=pose.registered_midpoint_valid.to_numpy(bool)
    label=f"{number}本目：{row['exit_angle_deg']:.2f}°"
    for ax in axes:
        ax.plot(*np.where(valid[:,None],xy,np.nan).T,label=label,linewidth=1.2)
        ax.scatter(*row['Lend_right_forward_mm'],s=35)
angles=np.array([r['exit_angle_deg'] for r in rows])
points=np.array([r['Lend_right_forward_mm'] for r in rows])
for ax in axes:
    ax.plot([45,135],[180,270],'--',color='gray',label='目標出口線')
    ax.scatter(45,180,marker='x',s=70,c='black',label='Lend目標45/180mm')
    ax.set(xlabel='入口に対する右 [mm]',ylabel='入口に対する前 [mm]',aspect='equal')
    ax.grid(alpha=.3);ax.legend(fontsize=8)
axes[0].set(xlim=(-5,155),ylim=(-5,285),title=f'機体B1600 inR45：同一候補{args.count}本')
axes[1].set(xlim=(39,53),ylim=(173,187),title='Lend付近の拡大')
fig.savefig(OUT/'comparison.png',dpi=140);plt.close(fig)
summary=dict(same_candidate_runs=args.count,repeat_goal=3,remaining_repeats=3-args.count,
    exit_angle_mean_deg=float(angles.mean()),exit_angle_min_deg=float(angles.min()),exit_angle_max_deg=float(angles.max()),
    Lend_mean_right_forward_mm=points.mean(0).tolist(),Lend_mean_residual_mm=(points.mean(0)-[45,180]).tolist(),
    Lend_component_range_mm=np.ptp(points,axis=0).tolist(),individual_Lend_residual_norm_mm=[r['Lend_residual_norm_mm'] for r in rows],
    parameter_adoption_allowed=False,independent_absolute_accuracy_guaranteed=False)
with (OUT/'review.json').open('x',encoding='utf-8') as f:json.dump(dict(summary=summary,rows=rows,
    selected_settings=reference_settings,source_params_changed=False,firmware_written=False,
    limitations=['入口相対マーカー中点の反復。絶対車軸/yaw・盤面固定の配置/停止・連続接続は未検証。',
        '既存summaryのauto_stop=falseを保持し、撮影時停止・後半静止・操作者OFF/接触なしを別に記録。',
        '出口角の短い窓はexit_window_diagnostic_v1に診断を保存。既存の採否条件は変更していない。',
        'BのEKF係数は未同定。左旋回・ファーム反映は未確認。']),f,ensure_ascii=False,indent=2,allow_nan=False)
text=f'# 機体B turn1600 右in45：同一候補{args.count}本\n\n'
text+='Lstart7.8 mm、Lend34.5 mm、角度補正−1.5度、半径−55 mm、吸引650、角加減速FF0.000750、角速度Ki0.005。\n\n'
text+='| 試行 | 出口角 度 | Lend差 右/前 mm | Lend距離差 mm | 最低電圧 V |\n|---|---:|---|---:|---:|\n'
for row in rows:
    delta=row['Lend_residual_mm']
    text+=f"| {row['request_id']} | {row['exit_angle_deg']:.3f} | {delta[0]:+.3f}/{delta[1]:+.3f} | {row['Lend_residual_norm_mm']:.3f} | {row['tracking']['battery_min_v']:.3f} |\n"
text+=f"\n出口角範囲{angles.min():.3f}〜{angles.max():.3f}度、平均{angles.mean():.3f}度。Lend平均差は右{summary['Lend_mean_residual_mm'][0]:+.3f}/前{summary['Lend_mean_residual_mm'][1]:+.3f}mm。"
text+='入口相対の中点評価であり、盤面固定・絶対車軸・body yaw・連続接続の合格ではありません。採用は未確定。\n\n'
if args.count==2:
    text+='残り1本はgeometry02_repeat_02、実行ID008。最低電圧7.328Vのため、次の反復前に機体バッテリー充電を依頼。現在は両方OFF・接触なし確認済み、充電完了は未確認。'
    text+='充電完了後に元225/45mm・前緑下向きへ概略配置し、両方ON・手離しを新しく確認して一回だけ取得します。\n'
(OUT/'README.md').write_text(text,encoding='utf-8')
state=read(BASE/'resume_state.json')
state.update(state=f'GEOMETRY02_{args.count}_RUNS_REVIEWED',same_candidate_runs=args.count,
    remaining_same_candidate_repeats=3-args.count,physical_power_state='BOTH_OFF_USER_CONFIRMED',
    execution_pending=False,execution_commands_sent=args.count+3,operator_ready=False,
    stage=str(BASE/cases[-1][0]),request_id=cases[-1][1],request_sent=True,
    latest_aggregate_review=str(OUT/'review.json'),latest_review=str(BASE/cases[-1][0]/'local_execution'/('run_'+cases[-1][1])/'midpoint_raw_wheel_clock_v3/midpoint_review.json'))
if args.count==2:
    next_stage=BASE/'geometry02_repeat_02';plan=read(next_stage/'placement_plan.json')
    state.update(next_stage=str(next_stage),next_settings=str(next_stage/'candidate.settings.json'),
        next_request_id=plan['request_id'],next_request_sent=False,next_operator_ready=False,
        charging_requested=True,charging_completion_confirmed=False,
        next_action='機体バッテリー充電後、元225/45mm・前緑下向きへ概略配置し、干渉なし・両方ON・手離しを確認して反復008を一回だけ取得。')
(BASE/'resume_state.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
with (BASE/'README.md').open('a',encoding='utf-8') as f:f.write(f'\n同一候補{args.count}本の集計は{OUT.name}/README.md。採用未確定、ファーム/Params変更なし。\n')
print(json.dumps(summary,ensure_ascii=False),flush=True)
