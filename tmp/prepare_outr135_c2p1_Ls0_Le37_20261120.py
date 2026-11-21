"""A1800右135出口の実走003から、Lend37mmの未実行候補を準備する。"""
from pathlib import Path
from datetime import datetime, timezone
import ast
import copy
import hashlib
import json
import shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_outr135_20261120_v1'
SOURCE=BASE/'c2p1_Ls0_Le35p2_03'
RID='A_OUTR135_1800_c2p1_Ls0_Le35p2_20261120_003'
RUN=SOURCE/'local_execution'/('run_'+RID)
REVIEW=BASE/'parallel_analysis'/RID/'midpoint_raw_wheel_clock_v1/midpoint_review.json'
OUT=BASE/'c2p1_Ls0_Le37_preparation_v1'
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def save(path,value):
    with path.open('x',encoding='utf-8') as file:
        json.dump(value,file,ensure_ascii=False,indent=2,allow_nan=False);file.write('\n')


review=read(REVIEW)
operator=read(SOURCE/'local_execution/operator_results'/f'{RID}.json')
assert operator['both_powered_off'] and not operator['contact_detected']
assert review['independent_optical_geometry_eligible'] and review['clock']['passed']
assert abs(review['exit_path_error_from135_deg'])<.1
prior=read(SOURCE/'candidate.settings.json')
assert prior['parameters']['lstart']==0 and prior['parameters']['lend']==35.2
assert prior['parameters']['correction']==2.1
OUT.mkdir(exist_ok=False)
stages=[]
for index in (1,2,3):
    stage=BASE/f'c2p1_Ls0_Le37_{index:02}'
    stage.mkdir(exist_ok=False)
    candidate=copy.deepcopy(prior)
    candidate['source']='A1800 out_r135 successful c2p1/Ls0/Le35.2 trial003 video and raw-wheel clock; Lend-only provisional refinement.'
    candidate['parameters']['lend']=37.
    save(stage/'candidate.settings.json',candidate)
    for name in ('body_geometry_A_out135_yellow_s20.json','static_pose_after_settle.py','static_settle_policy.json'):
        shutil.copyfile(SOURCE/name,stage/name)
    config=read(SOURCE/'config.json');config['geometry']=str(stage/'body_geometry_A_out135_yellow_s20.json')
    save(stage/'config.json',config)
    identifier=f'A_OUTR135_1800_c2p1_Ls0_Le37_20261120_{index:03}'
    wrapper=(SOURCE/'run_recorded_once.py').read_text(encoding='utf-8-sig')
    assert wrapper.count(RID)==1;wrapper=wrapper.replace(RID,identifier);ast.parse(wrapper)
    with (stage/'run_recorded_once.py').open('x',encoding='utf-8') as file:file.write(wrapper)
    plan=read(SOURCE/'placement_plan.json')
    plan.update(state='LEN37_CANDIDATE_READY_CHARGE_AND_CURRENT_STATE_REQUIRED',request_id=identifier,next_request_id=identifier,
        candidate=str(stage/'candidate.settings.json'),config=str(stage/'config.json'),
        candidate_requested_parameters=candidate['parameters'],source_review=str(REVIEW),
        guide=str(OUT/'start_placement_guide.png'),next_candidate_executed=False,execution_pending=False,
        operator_ready=False,current_placement_confirmed=False,current_clearance_confirmed=False,current_hands_clear_confirmed=False,
        power_state='Both_OFF_current_operator_confirmed_after_c2p1_Le35p2_003',both_powered_off_confirmed=True,
        source_params_changed=False,battery_charge_requested=True,
        battery_charge_reason='Prior start7033mV near device7000mV start limit; retained under-load minimum6.543V.',
        recovery_order='Camera static capture/transfer/analysis first; then BLE connect/settings/readback/one capture/disconnect')
    save(stage/'placement_plan.json',plan);stages.append(dict(stage=str(stage),trial_id=identifier))

angle=review['exit_path_direction_change_deg']
direction=np.array([np.sin(np.deg2rad(angle)),np.cos(np.deg2rad(angle))])
measured=np.array(review['midpoint_Lend_right_forward_mm'])
target=np.array(review['target_midpoint_Lend_right_forward_mm'])
predicted=measured+1.8*direction
save(OUT/'candidate_basis.json',dict(mouse='A',motion='out_r135',preset_mm_s=1800,
    source_review=str(REVIEW),source_operator=operator,prior_requested_parameters=prior['parameters'],
    next_requested_parameters=candidate['parameters'],stages=stages,
    measured_exit_angle_deg=angle,measured_Lend_right_forward_mm=measured.tolist(),
    measured_Lend_residual_mm=(measured-target).tolist(),
    length_only_solution=review['length_only_candidate_at_current_angle'],
    Lstart_delta_from_exit_line_mm=review['Lstart_delta_from_exit_line_mm'],
    predicted_Lend_right_forward_mm=predicted.tolist(),predicted_residual_mm=(predicted-target).tolist(),
    predictions_are_not_measurements=True,
    rationale='出口134.961度を維持。入口時計からのLe局所解37.028mmを37.0mmへ丸める。Ls解は-0.516mmだが出口線はLs差-0.053mmで一致しており、許容下限Ls0を維持。半径/PID/FFは維持して実走で再確認。',
    observation_rate=review['independent_clock_required_motion_quality']['observation_rate'],
    original_summary_preserved=review['original_summary'],
    late_stationary=review['late_stationary'],
    trial_stop_evidence=read(RUN/'trial.json')['recording_stop']['log_stop_evidence'],
    measurement_scope='Height-corrected relative marker midpoint in measured incoming axes; absolute board connection/axle/body yaw not calibrated.',
    parameter_adoption_allowed=False,firmware_changed=False,hardware_operations=0,
    ekf_parameters_changed=False,ekf_calibration_status='Unavailable: absolute body yaw and IMU-marker transform uncalibrated; out135 excluded by current firmware eligibility.',
    current_power_state='Both_OFF_confirmed',charging_status='requested_not_confirmed',
    input_sha256={str(p):sha(p) for p in (REVIEW,SOURCE/'candidate.settings.json',RUN/'trial.json',Path(__file__))}))

plt.rcParams['font.family']='Yu Gothic'
fig,ax=plt.subplots(figsize=(6.5,7),layout='constrained')
ax.set(xlim=(-8,368),ylim=(368,-8),aspect='equal',xlabel='盤面右 [mm]',ylabel='盤面下 [mm]')
ax.set_xticks(np.arange(0,361,45));ax.set_yticks(np.arange(0,361,45));ax.grid(alpha=.35)
for x in range(0,361,90):ax.scatter([x]*5,range(0,361,90),s=17,color='#3e474b')
ax.scatter(180,135,c='black',marker='+',s=140)
h=np.deg2rad(45);u=np.array([np.cos(h),np.sin(h)]);front=np.array([180,135])+25*u;rear=np.array([180,135])-25*u
ax.plot([rear[0],front[0]],[rear[1],front[1]],c='black')
ax.scatter(*front,c='red',s=65,label='前：赤・右下向き');ax.scatter(*rear,c='gold',edgecolors='black',s=65,label='後：黄')
ax.annotate('開始中点 (180,135) mm',xy=(180,135),xytext=(15,75),arrowprops=dict(arrowstyle='->'))
ax.scatter(225,315,marker='x',s=65,label='Lend格子目標 (225,315)')
ax.scatter(135,315,marker='+',s=85,label='停止格子目標 (135,315)')
ax.set_title('機体A / 右135°出口・1800 mm/s\nLstart0 / Lend37.0 mm / 補正＋2.1°')
ax.text(.02,.02,'開始位置は大まかに確認。配置案内のみ。\n穴・全接地点・車幅・旋回と停止の空きを実物で確認。',
    transform=ax.transAxes,fontsize=9,bbox=dict(facecolor='white',alpha=.9,edgecolor='lightgray'))
ax.legend(loc='upper right',fontsize=8);fig.savefig(OUT/'start_placement_guide.png',dpi=150);plt.close(fig)
root=BASE/'preparation.json';shutil.copyfile(root,OUT/'before_preparation.json')
plan=read(BASE/'c2p1_Ls0_Le37_01/placement_plan.json')
plan.update(updated_at=datetime.now(timezone.utc).isoformat(),last_request_id=RID,last_request_executed=True,
    last_saved_log_status='saved',last_contact_detected=False,last_successful_capture_id=RID,
    camera_health_report=str(OUT/'phone_health_before_on.json'))
root.write_text(json.dumps(plan,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(prepared=True,parameters=candidate['parameters'],predicted_residual_mm=(predicted-target).tolist(),hardware_operations=0),ensure_ascii=False))
