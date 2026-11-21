"""B右V90の非対称FFを保ち、適格な入口相対動画解から長さ候補を準備する。実機操作なし。"""
from pathlib import Path
from datetime import datetime,timezone
import json
import shutil
import sys
import numpy as np
import pandas as pd

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_rv90_20261120_v1'
OLD=BASE/'geometry_ff_acc0p0008_dec0p0007_Ls3p3_Le18p9_c0p0_01'
NEW=BASE/'geometry_ff_acc0p0008_dec0p0007_Ls6p0_Le17p4_c0p0_01'
RID='B_RV90_1600_geom6p0_17p4_c0p0_ffA0p0008D0p0007_20261121_001'
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
def save(p,d):
    with p.open('x',encoding='utf-8') as f:json.dump(d,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')

run=next((OLD/'local_execution').glob('run_*'))
m=read(run/'midpoint_raw_wheel_clock_v2/midpoint_review.json')
track=read(run/'tracking_1ms_review_v3/verification.json')
static=read(next((OLD/'local_execution').glob('prepare_*'))/'static_check.json')
assert m['independent_optical_geometry_eligible']
norm=float(np.linalg.norm(m['midpoint_Lend_residual_mm']))
points=pd.read_csv(run/'full/relative_marker_midpoint.csv')
xy=points[['center_dx_mm','center_dy_mm']].to_numpy();directions={}
for name in ('entry','exit'):
    lo,hi=m[f'{name}_window_pts_s'];mask=points.detected&(points.time_video_s>=lo)&(points.time_video_s<=hi)
    a=xy[mask];d=np.linalg.svd(a-a.mean(0),full_matrices=False)[2][0]
    if d@(a[-1]-a[0])<0:d=-d
    directions[name+'_board_direction_deg']=float(np.degrees(np.arctan2(d[1],d[0]))%360)
review=dict(created_at=datetime.now(timezone.utc).isoformat(),mouse='B',motion='r_v90',preset_mm_s=1600,
    source_trial=str(run/'trial.json'),geometry_review=str(run/'midpoint_raw_wheel_clock_v2/midpoint_review.json'),
    independent_optical_geometry_quality_passed=True,
    relative_geometry_goal_passed=bool(norm<=3 and abs(m['exit_path_error_from90_deg'])<=.5),
    entry_relative_exit_deg=m['exit_path_direction_change_deg'],Lend_residual_mm=m['midpoint_Lend_residual_mm'],Lend_residual_norm_mm=norm,
    **directions,static_apparent_midpoint_board_mm=static['center_board_mm'],static_apparent_heading_deg=static['heading_deg'],
    requested_length_solution=m['length_only_candidate_at_current_angle'],
    control_comparison=str(BASE/'asymmetric_ff01_tracking_comparison.json'),
    original_summary_preserved=True,geometry_goal_tolerances_unchanged=True,
    additional_3mm_1degree_gate_applied=False,operator_approximate_placement_requested=True,
    board_fixed_connection_passed=False,absolute_axle_or_body_yaw_calibrated=False,EKF_B_calibrated=False,
    parameter_adoption_allowed=False,source_params_changed=False,firmware_write_performed=False)
save(run/'relative_geometry_review.json',review)

solution=m['length_only_candidate_at_current_angle']
assert abs(solution['request_Lstart_mm']-6.)<.05 and abs(solution['request_Lend_mm']-17.4)<.05
assert not NEW.exists();NEW.mkdir()
for name in ('baseline.settings.json','config.json','static_pose_after_settle.py','static_5s_revision.json'):
    shutil.copyfile(OLD/name,NEW/name)
candidate=read(OLD/'candidate.settings.json')
candidate['parameters'].update(lstart=6.,lend=17.4)
candidate['source']='B右V90・加速FF0.000800/減速FF0.000700で末尾RMS4.970rad/s、出口89.457度、Lend残差右+1.525/前-2.691mm。適格な入口raw wheel時計の動画長さ解6.005/17.375mmを6.0/17.4mmに丸めて比較。FF/PID/角度補正は維持。'
save(NEW/'candidate.settings.json',candidate)
old_id=read(OLD/'comparison_plan.json')['request_id']
wrapper=(OLD/'run_recorded_once.py').read_text(encoding='utf-8')
assert wrapper.count(old_id)==1;wrapper=wrapper.replace(old_id,RID)
compile(wrapper,str(NEW/'run_recorded_once.py'),'exec')
(NEW/'run_recorded_once.py').write_text(wrapper,encoding='utf-8',newline='\n')
plan=read(OLD/'comparison_plan.json')
plan.update(request_id=RID,independent_attempt=1,parameters=candidate['parameters'],
    changes=dict(lstart_mm=[3.3,6.],lend_mm=[18.9,17.4]),
    source_trial=str(run/'trial.json'),source_review=str(run/'relative_geometry_review.json'),
    source_comparison=str(BASE/'asymmetric_ff01_tracking_comparison.json'),
    source_reviews=[str(run/'midpoint_raw_wheel_clock_v2/midpoint_review.json')],
    original_length_solution=solution,reason=candidate['source'],
    settings_identical_to=None,power_state='BOTH_OFF_USER_CONFIRMED',operator_ready=False,
    execution_commands_sent=0,hardware_operations=0,prospective_setup_gate_applied=False,
    solution_scope='現在の出口方向を保つ局所近似の入口相対マーカー中点解。絶対車軸値ではなく、次の実機測定で検証する。',
    retained_limitations=['FF非対称の制御比較はまだ1本。末尾RMSは低下したが振れは残る。',
        '元summaryのauto_stop_confirmed falseは保持し、全telemetry・固定後半静止窓・操作者の停止確認を別に記録。',
        '設置はユーザー指定の概略位置。盤面固定差と相対軌道を区別し、停止点の余走を長さ解に含めない。'],
    acceptance='従来の光学品質基準、Lend3mm/出口90±0.5度を維持し、同時に旋回中と末尾のRMS・逆方向ピーク・飽和を確認。',
    source_params_changed=False,firmware_write_performed=False)
save(NEW/'comparison_plan.json',plan)
sys.path.insert(0,str(ROOT/'tools'))
from pixel_camera_tuning_mcp import McpTuningService
payload=McpTuningService(NEW/'offline_validation',{}).prepare(str(NEW/'config.json'),str(NEW/'candidate.settings.json'),plan['placement'],25.,None)
save(NEW/'offline_validation.json',dict(state='VALIDATED_NOT_EXECUTED',payload=payload,hardware_operations=0))
shutil.copyfile(BASE/'asymmetric_ff01_tracking_comparison.json',run/'analysis_sources/asymmetric_ff01_tracking_comparison.json')
shutil.copyfile(Path(__file__),run/'analysis_sources'/Path(__file__).name)
state=read(BASE/'resume_state.json')
state.update(state='ASYMMETRIC_FF_LENGTH6P0_17P4_PREPARED_BOTH_OFF',next_stage=str(NEW),next_request_id=RID,
    next_request_sent=False,execution_pending=False,operator_ready=False,power_state='BOTH_OFF_USER_CONFIRMED',
    same_candidate_quality_valid_runs=1,same_candidate_goal_passed_runs=int(review['relative_geometry_goal_passed']),same_candidate_valid_runs=1,
    current_review=str(run/'relative_geometry_review.json'),current_feedforward=track['feedforward'],
    offline_analysis_sessions=[],next_action='新規準備確認後、元の静止品質検査通過時だけ非対称FFを保ち長さ6.0/17.4mmを1回測定。')
(BASE/'resume_state.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(folder=str(NEW),request_id=RID,previous_exit_deg=review['entry_relative_exit_deg'],
    previous_Lend_norm_mm=norm,changes=plan['changes'],hardware_operations=0),ensure_ascii=False))
