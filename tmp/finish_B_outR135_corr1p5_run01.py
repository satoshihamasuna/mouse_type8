from pathlib import Path
from datetime import datetime, timezone
import json, hashlib, copy, importlib.util, sys
import numpy as np
import pandas as pd

ROOT=Path.cwd()
BASE=ROOT/'tools/turn_analysis/20261114_B_turn500_seed_from_A'
sys.path[:0]=[str(ROOT/'tools'), str(ROOT/'tools/analysis'), str(BASE)]
from video_marker_pose import board_to_pose_transform, transform_points, log_elapsed_seconds
from prepare_outR135_candidate_fast import validate_files

def load(p): return json.loads(Path(p).read_text(encoding='utf-8'))
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p,d):
    with Path(p).open('x',encoding='utf-8',newline='\n') as f:
        json.dump(d,f,ensure_ascii=False,indent=2,allow_nan=False); f.write('\n')

meta_path=BASE/'next_session_B_outR135_corr1p5_Lstart33p7_v1.json'
meta=load(meta_path); session=Path(meta['session']); run=session/('run_'+meta['capture_request_id'])
review_dir=BASE/'outR135_corr1p5_Lstart33p7_run01_review'
review=load(review_dir/'review.json'); dense=load(BASE/'outR135_corr1p5_Lstart33p7_run01_review_dense/result.json')
job=load(session/'jobs'/f"{meta['capture_request_id']}.json")
assert job['state']=='DONE' and job['result']['full_returncode']==0 and job.get('error') is None
assert load(session/'jobs/close_after_B_outR135_corr1p5_Lstart33p7_run01.json')['state']=='DONE'
assert not (session/'service.lock').exists()
assert review['status']=='VALID_SINGLE_TRIAL_REVIEW' and dense['state']=='PASSED'
assert review['summary']['motion_observation_rate']>=.95
assert job['result']['capture']['execution_resend_count']==0
power=session/'power_off_confirmation.json'
save(power,dict(recorded_at=datetime.now(timezone.utc).isoformat(),question_call_id='call_d9d948bc1b9b4fd3bf49352f11498f94',
    user_reply='接触なし・両方OFFにしました',hardware_power='BOTH_OFF_USER_CONFIRMED',operator_reported_no_hole_or_wall_contact=True))
fv=review_dir/'final_validity.json'
save(fv,dict(state='VALID_RELATIVE_SINGLE_TRIAL',full_trajectory_review_passed=True,dense_static_confirmation_passed=True,
    relative_endpoint_error_right_forward_mm=[review['endpoint_error_right_mm'],review['endpoint_error_forward_mm']],
    exit_travel_error_deg=review['exit_travel_error_deg'],motion_observation_rate=review['summary']['motion_observation_rate'],
    same_settings_valid_trials=1,repeatability_verified=False,parameter_adoption_allowed=False,hardware_power='BOTH_OFF_USER_CONFIRMED',
    operator_reported_no_hole_or_wall_contact=True,board_fixed_goal_verified=False,evaluation_frame=meta['evaluation_frame'],
    input_sha256={str(p):sha(p) for p in [review_dir/'review.json',BASE/'outR135_corr1p5_Lstart33p7_run01_review_dense/result.json',power]}))

report=load(run/'full/report.json'); trial=load(run/'trial.json'); plan=load(meta['plan'])
reference=load(run/'marker_reference_used.json'); inv=np.linalg.inv(board_to_pose_transform(reference))
pose=pd.read_csv(run/'full/apparent_floor_marker_pose.csv'); log=pd.read_csv(run/'motion.csv'); t=log_elapsed_seconds(log)
start,end=[(trial['log_quality'][k]-float(log.time_ms.iloc[0]))%2**32/1000 for k in ['motion_start_time_ms','motion_end_time_ms']]
pt=pose.time_video_s.to_numpy()-report['log_time_zero_at_video_s']
good=(pose.marker_detected_raw & pose.calibration_valid).to_numpy(bool)
xy=transform_points(pose[['center_x_mm','center_y_mm']].to_numpy(float),inv)
om=log['ideal.rad_velo'].to_numpy(); idx=np.flatnonzero(abs(om)>=.02*abs(om).max())
def angle(mask):
    p=xy[mask]; assert len(p)>=15
    d=np.linalg.svd(p-p.mean(axis=0),full_matrices=False)[2][0]
    if d@(p[-1]-p[0])<0: d=-d
    assert np.ptp(p@d)>=30
    return float(np.degrees(np.arctan2(d[1],d[0]))%360)
entry=angle(good&(pt>=start)&(pt<t[idx[0]]-.015)&(np.interp(pt,t,log['ideal.velo'])>.25))
exit=angle(good&(pt>t[idx[-1]]+.03)&(pt<=end-.02)&(np.interp(pt,t,log['ideal.velo'])>.25))
pre=good&(pt>=start-.35)&(pt<=start-.15); post=good&(pt>=end+.2)&(pt<=end+.6)
assert pre.sum()>=20 and post.sum()>=20
before=np.median(xy[pre],axis=0); after=np.median(xy[post],axis=0)
save(review_dir/'board_fixed_diagnostic.json',dict(evaluation_frame='board_fixed_apparent_floor_midpoint_diagnostic_only',
    planned_start_board_xy_mm=plan['initial_position_board_xy_mm'],observed_start_board_xy_mm=before.tolist(),
    planned_stop_board_xy_mm=plan['planned_stop_board_xy_mm'],observed_stop_board_xy_mm=after.tolist(),
    stop_error_xy_mm=(after-np.array(plan['planned_stop_board_xy_mm'])).tolist(),entry_angle_board_deg=entry,
    entry_error_from45_deg=(entry-45+180)%360-180,exit_angle_board_deg=exit,exit_error_from180_deg=(exit-180+180)%360-180,
    coordinates_aligned_to_robot=False,absolute_axle_or_height_corrected_position_verified=False))
meta.update(state='VALID_RELATIVE_TRIAL_POWER_OFF_CLOSED',execution_commands_sent=1,execution_resends=0,
    hardware_power='BOTH_OFF_USER_CONFIRMED',operator_ready='RECONFIRM_REQUIRED_FOR_NEXT_RUN',resume_allowed=False,
    recording_id=trial['recording_id'],trial=str(run/'trial.json'),final_validity=str(fv),service_closed=True)
meta_path.write_text(json.dumps(meta,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

new_session=ROOT/'tools/logs/pixel_camera/tuning_cli_B_outR135_corr1p5_Lstart33p7_repeat02_20261115_prepared'
new_plan=BASE/'B_outR135_corr1p5_Lstart33p7_repeat02_plan.json'
new_meta=BASE/'next_session_B_outR135_corr1p5_Lstart33p7_repeat02_v1.json'
new_helper=BASE/'preflight_outR135_corr1p5_Lstart33p7_repeat02_zoom2.py'
new_session.mkdir(exist_ok=False)
plan.update(state='UNEXECUTED_SAME_SETTINGS_REPEATABILITY_TRIAL',baseline_actual_settings=str(run/'motion.settings.json'),
    baseline_final_validity=str(fv),baseline_actual_settings_sha256=sha(run/'motion.settings.json'),baseline_final_validity_sha256=sha(fv),
    changes_from_last_trial={},comparison_reason='Same settings second trial; first measured R+0.493/F-0.758 mm, exit+0.611deg. Do not adjust below unknown repeat spread.',
    candidate_bundled_compensation=False)
plan.pop('next_candidate_model',None)
save(new_plan,plan)
new=copy.deepcopy(meta)
for k in ['recording_id','trial','final_validity','service_pid','service_closed']: new.pop(k,None)
new.update(state='OFFLINE_PREPARED_AWAITING_FRESH_OPERATOR_READY',session=str(new_session),plan=str(new_plan),
    capture_request_id='B_outR135_corr1p5_Lstart33p7_repeat02',operator_ready='PENDING_FRESH_CONFIRMATION',
    hardware_power='BOTH_OFF_USER_CONFIRMED',execution_commands_sent=0,execution_resends=0,preflight_ready=False,
    resume_allowed=True,operator_hole_clearance_pending=True,preflight_helper=str(new_helper),
    preparation_validation=str(BASE/'outR135_corr1p5_Lstart33p7_repeat02_preparation_validation.json'))
for key in ['settings_source','preset_readback','settings']: new[key]=str(new_session/Path(meta[key]).name)
oldhelper=Path(meta['preflight_helper'])
new_helper.write_text(oldhelper.read_text(encoding='utf-8').replace(meta_path.name,new_meta.name),encoding='utf-8',newline='\n')
validate_files(new)
save(new_meta,new)
# Same frozen detector/nominal gates, regressed against the previous real static image offline.
spec=importlib.util.spec_from_file_location('new_preflight',new_helper); module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
import cv2
config=load(new['config']); frame=cv2.imread(str(session/'static_preflight_frame.png')); assert frame is not None
check=module.inspect_frame(frame,load(config['reference']),load(config['geometry']),plan['expected_candidate_parameters'],plan)
assert check['ready'] and check['physical_hole_clearance_verified_by_model'] is False
save(new['preparation_validation'],dict(state='PASSED_OFFLINE_METADATA_AND_REAL_FRAME_REGRESSION',hardware_commands_sent=0,
    unchanged_geometry_from_prior_trial=True,real_frame_ready=check['ready'],
    input_sha256={str(p):sha(p) for p in [new_plan,new_meta,new_helper,session/'static_preflight_frame.png',fv]}))
save(review_dir/'next_repeat.json',dict(meta=str(new_meta),same_settings_valid_trials=1,values_changed=False))
(review_dir/'README.md').write_text('機体B・右135°出口・500プリセット。半径−32.3 mm、Lstart33.7 mm、Lend46.07 mm、補正＋1.5°、旋回P0.10。\n\n入口方向に対する相対誤差は横＋0.493 mm・前後−0.758 mm、出口＋0.611°。全フレーム観測95.95%、固定288フレームの静止照合100%で通過。速報の品質不足も元資料として保持。接触なし・両方OFFを操作者が確認。1本のみのため再現性は未確認、正式採用・盤面固定目標・連続接続は未確認。次は値を変えず同じ候補2本目。\n',encoding='utf-8')
print(json.dumps(dict(next_meta=str(new_meta),board_stop_error=(after-np.array(plan['planned_stop_board_xy_mm'])).tolist(),entry=entry,exit=exit),ensure_ascii=False))
