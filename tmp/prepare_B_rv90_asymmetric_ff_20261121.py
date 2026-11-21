"""B右V90のFF0.000800の2本を保存し、減速FFだけを戻す比較を準備する。機器操作なし。"""
from pathlib import Path
from datetime import datetime,timezone
import hashlib
import json
import shutil
import sys
import numpy as np
import pandas as pd

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_rv90_20261120_v1'
OLD=BASE/'geometry_ff0p0008_Ls3p3_Le18p9_c0p0_02'
NEW=BASE/'geometry_ff_acc0p0008_dec0p0007_Ls3p3_Le18p9_c0p0_01'
RID='B_RV90_1600_geom3p3_18p9_c0p0_ffA0p0008D0p0007_20261121_001'
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
def save(p,d):
    with p.open('x',encoding='utf-8') as f:json.dump(d,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')

rows=[]
for i in (1,2):
    stage=BASE/f'geometry_ff0p0008_Ls3p3_Le18p9_c0p0_{i:02d}'
    run=next((stage/'local_execution').glob('run_*'))
    m=read(run/'midpoint_raw_wheel_clock_v2/midpoint_review.json')
    static=read(next((stage/'local_execution').glob('prepare_*'))/'static_check.json')
    track=read(run/'tracking_1ms_review_v3/verification.json')
    assert m['independent_optical_geometry_eligible']
    points=pd.read_csv(run/'full/relative_marker_midpoint.csv')
    xy=points[['center_dx_mm','center_dy_mm']].to_numpy()
    directions={}
    for name in ('entry','exit'):
        lo,hi=m[f'{name}_window_pts_s'];mask=points.detected&(points.time_video_s>=lo)&(points.time_video_s<=hi)
        a=xy[mask];d=np.linalg.svd(a-a.mean(0),full_matrices=False)[2][0]
        if d@(a[-1]-a[0])<0:d=-d
        directions[name+'_board_direction_deg']=float(np.degrees(np.arctan2(d[1],d[0]))%360)
    norm=float(np.linalg.norm(m['midpoint_Lend_residual_mm']))
    rows.append(dict(trial=str(run/'trial.json'),geometry_review=str(run/'midpoint_raw_wheel_clock_v2/midpoint_review.json'),
        independent_optical_geometry_quality_passed=True,
        relative_geometry_goal_passed=bool(norm<=3 and abs(m['exit_path_error_from90_deg'])<=.5),
        entry_relative_exit_deg=m['exit_path_direction_change_deg'],Lend_residual_norm_mm=norm,
        Lend_residual_mm=m['midpoint_Lend_residual_mm'],**directions,
        static_apparent_midpoint_board_mm=static['center_board_mm'],static_apparent_heading_deg=static['heading_deg'],
        original_summary=m['original_summary'],late_stationary=m['late_stationary'],
        turn_tracking_rms_rad_s=track['tracking']['tracking_rms_rad_s'],post_tracking_rms_rad_s=track['tracking']['post_rms_rad_s'],
        battery_min_v=track['battery_min_v'],saturation_pct=track['tracking']['saturation_pct']))

comparison=dict(created_at=datetime.now(timezone.utc).isoformat(),mouse='B',motion='r_v90',preset_mm_s=1600,
    settings_identical=True,all_trials_retained=True,trials=rows,geometry_goal_passed_runs=sum(r['relative_geometry_goal_passed'] for r in rows),
    total_quality_valid_runs=2,exit_angle_spread_deg=abs(rows[0]['entry_relative_exit_deg']-rows[1]['entry_relative_exit_deg']),
    tracking_comparison=str(BASE/'ff0p0008_two_run_tracking_comparison.json'),
    conclusion='角加減速FF0.000800で旋回中の追従RMSは低下したが、末尾の振れは改善せず出口角度のばらつきも残る。加速FFを保ち減速FFだけ0.000700へ戻して比較する。',
    causality_proven=False,battery_or_surface_confounding_excluded=False,
    original_summary_failures_preserved=True,independent_late_stationary_window_unchanged=True,
    video_heading_median_filter_frames=5,high_frequency_raw_yaw_validation_available=False,
    absolute_axle_or_body_yaw_calibrated=False,EKF_B_calibrated=False,
    source_params_changed=False,firmware_write_performed=False,parameter_adoption_allowed=False)
save(BASE/'ff0p0008_two_run_geometry_review.json',comparison)
run=next((OLD/'local_execution').glob('run_*'))
save(run/'relative_geometry_review.json',dict(**rows[-1],comparison=str(BASE/'ff0p0008_two_run_geometry_review.json'),
    original_summary_preserved=True,additional_3mm_1degree_gate_applied=False,
    operator_approximate_placement_requested=True,geometry_goal_tolerances_unchanged=True,
    board_fixed_connection_passed=False,source_params_changed=False))

assert not NEW.exists();NEW.mkdir()
for name in ('baseline.settings.json','config.json','static_pose_after_settle.py','static_5s_revision.json'):
    shutil.copyfile(OLD/name,NEW/name)
candidate=read(OLD/'candidate.settings.json');assert candidate['feedforward']['ff_om_decel']==.0008
candidate['feedforward']['ff_om_decel']=.0007
candidate['source']='B右V90・角加減速FF0.000800の2本で旋回追従RMS3.660/3.940rad/s、末尾RMS6.173/6.621rad/s。加速FF0.000800を維持し減速FFだけ0.000700へ下げ、旋回末尾の逆方向の振れと幾何を比較する。長さ/補正/PID/並進FFは維持。'
save(NEW/'candidate.settings.json',candidate)
old_id=read(OLD/'comparison_plan.json')['request_id']
wrapper=(OLD/'run_recorded_once.py').read_text(encoding='utf-8')
assert wrapper.count(old_id)==1;wrapper=wrapper.replace(old_id,RID)
old_guard="previous = dict(expected['feedforward'], ff_om_accel=0.000600, ff_om_decel=0.000600)"
assert wrapper.count(old_guard)==1
wrapper=wrapper.replace(old_guard,"previous = dict(expected['feedforward'], ff_om_accel=0.000800, ff_om_decel=0.000800)")
compile(wrapper,str(NEW/'run_recorded_once.py'),'exec')
(NEW/'run_recorded_once.py').write_text(wrapper,encoding='utf-8',newline='\n')
plan=read(OLD/'comparison_plan.json')
plan.update(request_id=RID,independent_attempt=1,feedforward=candidate['feedforward'],
    changes=dict(ff_om_decel=[.0008,.0007]),source_trial=str(run/'trial.json'),
    source_comparison=str(BASE/'ff0p0008_two_run_geometry_review.json'),
    source_reviews=[r['geometry_review'] for r in rows],reason=candidate['source'],
    settings_identical_to=None,power_state='BOTH_OFF_USER_CONFIRMED',operator_ready=False,
    execution_commands_sent=0,hardware_operations=0,prospective_setup_gate_applied=False,
    acceptance='同じ1ms評価で旋回中と末尾のRMS、残留PID、逆方向ピーク、飽和を比較する。元の光学品質基準とLend3mm/出口90±0.5度を維持し、変更後の幾何を測定。設置はユーザー指定の概略配置で入口相対評価。',
    source_params_changed=False,firmware_write_performed=False)
plan.pop('voltage_screen',None)
save(NEW/'comparison_plan.json',plan)
sys.path.insert(0,str(ROOT/'tools'))
from pixel_camera_tuning_mcp import McpTuningService
payload=McpTuningService(NEW/'offline_validation',{}).prepare(str(NEW/'config.json'),str(NEW/'candidate.settings.json'),plan['placement'],25.,None)
save(NEW/'offline_validation.json',dict(state='VALIDATED_NOT_EXECUTED',payload=payload,hardware_operations=0))
for name in ('ff0p0008_two_run_tracking_comparison.json','ff0p0008_two_run_geometry_review.json'):
    shutil.copyfile(BASE/name,run/'analysis_sources'/name)
shutil.copyfile(Path(__file__),run/'analysis_sources'/Path(__file__).name)
state=read(BASE/'resume_state.json')
state.update(state='ASYMMETRIC_FF_CANDIDATE_PREPARED_BOTH_OFF',next_stage=str(NEW),next_request_id=RID,
    next_request_sent=False,execution_pending=False,operator_ready=False,power_state='BOTH_OFF_USER_CONFIRMED',
    same_candidate_quality_valid_runs=2,same_candidate_goal_passed_runs=1,same_candidate_valid_runs=2,
    current_review=str(run/'relative_geometry_review.json'),
    next_action='新規準備確認後、従来の静止品質検査を通過した場合だけ加速FF0.000800/減速FF0.000700を1回測定。',
    previous_exit_angles_deg=[r['entry_relative_exit_deg'] for r in rows])
(BASE/'resume_state.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(folder=str(NEW),request_id=RID,exit_angles_deg=state['previous_exit_angles_deg'],
    changed=dict(ff_om_decel=[.0008,.0007]),hardware_operations=0),ensure_ascii=False))
