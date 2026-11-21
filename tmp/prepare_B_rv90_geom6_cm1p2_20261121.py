"""同一長さ/FFの角度補正0/-2度を補間し、-1.2度を比較する。実機操作なし。"""
from pathlib import Path
from datetime import datetime,timezone
import json,shutil,sys,hashlib
import numpy as np
import pandas as pd

ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_rv90_20261120_v1'
OLD=BASE/'geometry_ff_acc0p0008_dec0p0007_Ls6p0_Le17p4_cm2p0_01'
NEW=BASE/'geometry_ff_acc0p0008_dec0p0007_Ls6p0_Le17p4_cm1p2_01'
RID='B_RV90_1600_geom6p0_17p4_cm1p2_ffA0p0008D0p0007_20261121_001'
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
def save(p,d):
    with p.open('x',encoding='utf-8') as f:json.dump(d,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')

run=next((OLD/'local_execution').glob('run_*'));m=read(run/'midpoint_raw_wheel_clock_v2/midpoint_review.json')
assert m['independent_optical_geometry_eligible']
static=read(next((OLD/'local_execution').glob('prepare_*'))/'static_check.json');track=read(run/'tracking_1ms_review_v3/verification.json')
points=pd.read_csv(run/'full/relative_marker_midpoint.csv');xy=points[['center_dx_mm','center_dy_mm']].to_numpy();directions={}
for name in ('entry','exit'):
    lo,hi=m[f'{name}_window_pts_s'];mask=points.detected&(points.time_video_s>=lo)&(points.time_video_s<=hi)
    a=xy[mask];d=np.linalg.svd(a-a.mean(0),full_matrices=False)[2][0]
    if d@(a[-1]-a[0])<0:d=-d
    directions[name+'_board_direction_deg']=float(np.degrees(np.arctan2(d[1],d[0]))%360)
norm=float(np.linalg.norm(m['midpoint_Lend_residual_mm']))
review=dict(created_at=datetime.now(timezone.utc).isoformat(),mouse='B',motion='r_v90',preset_mm_s=1600,
    source_trial=str(run/'trial.json'),geometry_review=str(run/'midpoint_raw_wheel_clock_v2/midpoint_review.json'),
    independent_optical_geometry_quality_passed=True,relative_geometry_goal_passed=bool(norm<=3 and abs(m['exit_path_error_from90_deg'])<=.5),
    entry_relative_exit_deg=m['exit_path_direction_change_deg'],Lend_residual_mm=m['midpoint_Lend_residual_mm'],Lend_residual_norm_mm=norm,**directions,
    static_apparent_midpoint_board_mm=static['center_board_mm'],static_apparent_heading_deg=static['heading_deg'],
    original_summary=m['original_summary'],tracking=track['tracking'],battery_min_v=track['battery_min_v'],
    original_summary_preserved=True,additional_3mm_1degree_gate_applied=False,operator_approximate_placement_requested=True,
    board_fixed_connection_passed=False,absolute_axle_or_body_yaw_calibrated=False,EKF_B_calibrated=False,
    source_params_changed=False,firmware_write_performed=False,parameter_adoption_allowed=False)
save(run/'relative_geometry_review.json',review)
before_stage=BASE/'geometry_ff_acc0p0008_dec0p0007_Ls6p0_Le17p4_c0p0_02';before_run=next((before_stage/'local_execution').glob('run_*'))
before=read(before_run/'midpoint_raw_wheel_clock_v2/midpoint_review.json');assert before['independent_optical_geometry_eligible']
a0=before['exit_path_direction_change_deg'];a1=m['exit_path_direction_change_deg'];slope=(a1-a0)/-2.
correction=(90.-a0)/slope;chosen=-1.2
assert -2.<correction<0. and abs(correction-chosen)<.1
interpolation=dict(source_trials=[str(before_run/'trial.json'),str(run/'trial.json')],
    corrections_deg=[0.,-2.],exit_angles_deg=[a0,a1],slope_exit_per_correction_deg=slope,
    interpolated_correction_deg=correction,chosen_correction_deg=chosen,predicted_exit_deg=a0+slope*chosen,
    same_length_ff_pid=True,only_two_measured_points=True,response_linearity_unverified=True,
    trial_validation_required=True,battery_min_v=[read(before_run/'tracking_1ms_review_v3/verification.json')['battery_min_v'],track['battery_min_v']])
save(BASE/'geom6p0_17p4_correction_bracket.json',interpolation)
assert not NEW.exists();NEW.mkdir()
for name in ('baseline.settings.json','config.json','static_pose_after_settle.py','static_5s_revision.json'):shutil.copyfile(OLD/name,NEW/name)
candidate=read(OLD/'candidate.settings.json');candidate['parameters']['correction']=chosen
candidate['source']='B右V90・長さ6.0/17.4mm・FF0.000800/0.000700・同PIDの有効2本で補正0度が87.672度、-2度が91.683度。線形補間の-1.161度を比較用-1.20度に丸め、位置/角度/末尾を再確認する。'
save(NEW/'candidate.settings.json',candidate)
plan=read(OLD/'comparison_plan.json');old_id=plan['request_id'];wrapper=(OLD/'run_recorded_once.py').read_text(encoding='utf-8');assert wrapper.count(old_id)==1
wrapper=wrapper.replace(old_id,RID);compile(wrapper,str(NEW/'run_recorded_once.py'),'exec');(NEW/'run_recorded_once.py').write_text(wrapper,encoding='utf-8',newline='\n')
plan.update(request_id=RID,parameters=candidate['parameters'],changes=dict(correction_deg=[-2.,chosen]),
    source_trial=str(run/'trial.json'),source_review=str(run/'relative_geometry_review.json'),source_reviews=[str(before_run/'midpoint_raw_wheel_clock_v2/midpoint_review.json'),str(run/'midpoint_raw_wheel_clock_v2/midpoint_review.json')],
    source_comparison=str(BASE/'geom6p0_17p4_correction_bracket.json'),reason=candidate['source'],
    operator_ready=False,power_state='BOTH_OFF_USER_CONFIRMED',execution_commands_sent=0,hardware_operations=0,
    logical_degree=-90.,physical_degree_candidate=-91.2,interpolation=interpolation,
    assumed_exit_response_per_correction_deg=slope,predicted_exit_under_unit_response_deg=interpolation['predicted_exit_deg'],
    response_is_unverified_hypothesis=True,source_params_changed=False,firmware_write_performed=False)
save(NEW/'comparison_plan.json',plan)
sys.path.insert(0,str(ROOT/'tools'));from pixel_camera_tuning_mcp import McpTuningService
payload=McpTuningService(NEW/'offline_validation',{}).prepare(str(NEW/'config.json'),str(NEW/'candidate.settings.json'),plan['placement'],25.,None)
save(NEW/'offline_validation.json',dict(state='VALIDATED_NOT_EXECUTED',payload=payload,hardware_operations=0))
shutil.copyfile(BASE/'geom6p0_17p4_correction_bracket.json',run/'analysis_sources/geom6p0_17p4_correction_bracket.json');shutil.copyfile(Path(__file__),run/'analysis_sources'/Path(__file__).name)
snapshot=read(run/'analysis_sources/module_hashes_during_analysis.json');changed=[n for n,d in snapshot['modules'].items() if hashlib.sha256((ROOT/n).read_bytes()).hexdigest()!=d['sha256']]
save(run/'analysis_sources/module_snapshot_postcheck.json',dict(changed_since_snapshot=changed,checked_after_analysis=True))
state=read(BASE/'resume_state.json');state.update(state='GEOM6P0_17P4_CM1P2_PREPARED_BOTH_OFF',next_stage=str(NEW),next_request_id=RID,
    next_request_sent=False,execution_pending=False,operator_ready=False,power_state='BOTH_OFF_USER_CONFIRMED',
    same_candidate_quality_valid_runs=1,same_candidate_goal_passed_runs=int(review['relative_geometry_goal_passed']),same_candidate_valid_runs=1,
    current_review=str(run/'relative_geometry_review.json'),offline_analysis_sessions=[],
    next_action='新規準備確認後、長さ/FF/PIDを保ち補間候補-1.20度を1回測定。採用値のソース反映は確認後。')
(BASE/'resume_state.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(folder=str(NEW),request_id=RID,previous_exit_deg=a1,previous_Lend_norm_mm=norm,interpolation=interpolation,hardware_operations=0),ensure_ascii=False))
