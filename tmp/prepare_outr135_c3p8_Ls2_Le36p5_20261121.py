"""A1800右135出口の完全な10ms/保存1ms試行から、角度と長さの次候補を準備する。"""
from pathlib import Path
from datetime import datetime, timezone
import ast
import copy
import hashlib
import json
import re
import shutil
import numpy as np
import pandas as pd

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_outr135_20261120_v1'
SOURCE=BASE/'c2p1_Ls0_Le37_stream10_01'
RID='A_OUTR135_1800_c2p1_Ls0_Le37_s10_20261121_001'
RUN=SOURCE/'local_execution'/('run_'+RID)
REVIEW=BASE/'parallel_analysis'/RID/'midpoint_raw_wheel_clock_v1/midpoint_review.json'
OUT=BASE/'c3p8_Ls2_Le36p5_preparation_20261121_v1'
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def save(path,value):
    with path.open('x',encoding='utf-8') as file:
        json.dump(value,file,ensure_ascii=False,indent=2,allow_nan=False);file.write('\n')


review=read(REVIEW);operator=read(SOURCE/'local_execution/operator_results'/f'{RID}.json')
assert operator['both_powered_off'] and not operator['contact_detected']
assert review['independent_optical_geometry_eligible'] and review['clock']['passed']
prior=read(SOURCE/'candidate.settings.json')
assert prior['parameters']['lstart']==0 and prior['parameters']['lend']==37 and prior['parameters']['correction']==2.1
assert read(SOURCE/'config.json')['stream_period_ms']==10
OUT.mkdir(exist_ok=False)
table=(ROOT/'Params/turn_table.h').read_text(encoding='utf-8')
wave=np.fromstring(re.search(r'accel_table\[1001\]\s*=\s*\{([^}]+)\}',table)[1],sep=',')
integral=float(re.search(r'accel_Integral\s*=\s*([.\d]+)',table)[1])


def endpoint(correction):
    dt=np.deg2rad(135-correction)/(1800/41*integral)/1000
    theta=np.r_[0,np.cumsum((wave[:-1]+wave[1:])/2*(1800/41)*dt)]
    middle=(theta[:-1]+theta[1:])/2
    return np.array([np.sum(1800*np.sin(middle)*dt),np.sum(1800*np.cos(middle)*dt)])+37*np.array([np.sin(theta[-1]),np.cos(theta[-1])])


angle_delta=endpoint(3.8)-endpoint(2.1)
angle=review['exit_path_direction_change_deg']-1.7
direction=np.array([np.sin(np.deg2rad(angle)),np.cos(np.deg2rad(angle))])
target=np.array(review['target_midpoint_Lend_right_forward_mm'])
measured=np.array(review['midpoint_Lend_right_forward_mm'])
adjusted=measured+angle_delta
delta_le=(target[0]-adjusted[0])/direction[0]
delta_ls=target[1]-adjusted[1]-delta_le*direction[1]
predicted=adjusted+[0,2.]-.5*direction
stages=[]
for index in (1,2,3):
    stage=BASE/f'c3p8_Ls2_Le36p5_stream10_{index:02}'
    stage.mkdir(exist_ok=False)
    candidate=copy.deepcopy(prior)
    candidate['source']='A1800 out_r135 stream10 trial001 complete video+1ms analysis; provisional coupled angle/length refinement.'
    candidate['parameters'].update(lstart=2.,lend=36.5,correction=3.8)
    save(stage/'candidate.settings.json',candidate)
    for name in ('body_geometry_A_out135_yellow_s20.json','static_pose_after_settle.py','static_settle_policy.json'):
        shutil.copyfile(SOURCE/name,stage/name)
    config=read(SOURCE/'config.json');config['geometry']=str(stage/'body_geometry_A_out135_yellow_s20.json')
    save(stage/'config.json',config)
    identifier=f'A_OUTR135_1800_c3p8_Ls2_Le36p5_s10_20261121_{index:03}'
    wrapper=(SOURCE/'run_recorded_once.py').read_text(encoding='utf-8-sig')
    assert wrapper.count(RID)==1;wrapper=wrapper.replace(RID,identifier);ast.parse(wrapper)
    with (stage/'run_recorded_once.py').open('x',encoding='utf-8') as file:file.write(wrapper)
    plan=read(SOURCE/'placement_plan.json')
    plan.update(state='NEXT_ANGLE_LENGTH_CANDIDATE_UNEXECUTED_CURRENT_STATE_REQUIRED',request_id=identifier,next_request_id=identifier,
        candidate=str(stage/'candidate.settings.json'),config=str(stage/'config.json'),candidate_requested_parameters=candidate['parameters'],
        operator_ready=False,execution_pending=False,next_candidate_executed=False,current_placement_confirmed=False,
        current_clearance_confirmed=False,current_hands_clear_confirmed=False,both_powered_off_confirmed=True,
        power_state='Both_OFF_current_operator_confirmed_after_stream10_Le37_001',source_params_changed=False,
        source_review=str(REVIEW),guide=str(BASE/'telemetry_period_recovery_20261121_v1/start_placement_photo_accepted_20261121.png'),
        last_failure_review=None,physical_placement_changed=False,battery_charge_requested=False,
        battery_charge_status='Prior retained minimum7.730V; re-read before next execution')
    save(stage/'placement_plan.json',plan);stages.append(dict(stage=str(stage),trial_id=identifier))

# 盤面に対する入口/出口方向も、相対評価と区別した診断として保存する。
pose=pd.read_csv(RUN/'full/relative_marker_midpoint.csv')
headings={}
for label,key in (('entry','entry_window_pts_s'),('exit','exit_window_pts_s')):
    lo,hi=review[key];mask=pose.detected.to_numpy(bool)&pose.time_video_s.between(lo,hi).to_numpy()
    xy=pose.loc[mask,['center_dx_mm','center_dy_mm']].to_numpy()
    d=np.linalg.svd(xy-xy.mean(0),full_matrices=False)[2][0]
    if d@(xy[-1]-xy[0])<0:d=-d
    headings[label+'_path_heading_in_registered_axes_deg']=float(np.degrees(np.arctan2(d[1],d[0]))%360)
save(OUT/'candidate_basis.json',dict(mouse='A',motion='out_r135',preset_mm_s=1800,source_review=str(REVIEW),
    source_operator=operator,prior_requested_parameters=prior['parameters'],next_requested_parameters=candidate['parameters'],stages=stages,
    current_measured_exit_angle_deg=review['exit_path_direction_change_deg'],current_Lend_right_forward_mm=measured.tolist(),
    current_Lend_residual_mm=(measured-target).tolist(),angle_only_nominal_delta_mm=angle_delta.tolist(),
    coupled_unrounded_lengths=dict(lstart=delta_ls,lend=37+delta_le),predicted_angle_deg=angle,
    predicted_Lend_right_forward_mm=predicted.tolist(),predicted_residual_mm=(predicted-target).tolist(),predictions_are_not_measurements=True,
    rationale='入口方向に対する回転136.665度へ補正を+1.7度増やして3.8度へ。角度変更後の局所解Ls2.079/Le36.586mmを2.0/36.5mmへ丸める。半径41、PID/FF/吸引650、連続10ms/保持1msは維持。次走行で再確認。',
    relative_motion_quality=review['independent_clock_required_motion_quality'],clock=review['clock'],
    board_frame_path_diagnostics=headings,board_frame_diagnostic_note='Registered path direction only; not absolute axle/body yaw. Initial placement tilt is removed only for the relative turn-shape evaluation.',
    original_summary_preserved=review['original_summary'],original_trajectory_model_rms_mm=review['original_trajectory_model_rms_mm'],
    independent_late_stationary=review['late_stationary'],trial_stop_evidence=read(RUN/'trial.json')['recording_stop']['log_stop_evidence'],
    parameter_adoption_allowed=False,firmware_changed=False,hardware_operations=0,
    ekf_parameters_changed=False,ekf_status='Current out135 unsupported by firmware eligibility; body-yaw/IMU-marker geometry not calibrated, no new K/noise adoption.',
    calibration_condition_note='Prior low-battery35.2mm trial and current fully charged37mm trial differ in voltage and stream cadence; changes are not attributed to one factor.',
    input_sha256={str(p):sha(p) for p in (REVIEW,RUN/'saved_1ms.csv',RUN/'saved_1ms.settings.json',ROOT/'Params/turn_table.h',Path(__file__))}))
root=BASE/'preparation.json';shutil.copyfile(root,OUT/'before_preparation.json')
plan=read(BASE/'c3p8_Ls2_Le36p5_stream10_01/placement_plan.json')
plan.update(updated_at=datetime.now(timezone.utc).isoformat(),last_request_id=RID,last_request_executed=True,
    last_successful_capture_id=RID,last_saved_log_status='saved',last_contact_detected=False,
    camera_health_report=str(OUT/'phone_health_before_on.json'))
root.write_text(json.dumps(plan,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(prepared=True,parameters=candidate['parameters'],predicted_angle_deg=angle,
    predicted_residual_mm=(predicted-target).tolist(),board_path_diagnostics=headings,hardware_operations=0),ensure_ascii=False))
