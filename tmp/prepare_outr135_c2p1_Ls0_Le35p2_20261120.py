"""A1800右135出口の初回から次候補を準備する。実機操作は行わない。"""
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
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT/'tools/logs/pixel_camera/turn1800_A_outr135_20261120_v1'
SOURCE = BASE/'baseline01'
RUN = SOURCE/'local_execution/run_A_OUTR135_1800_baseline_20261120_001'
VALIDATION = BASE/'resume_20261120_v1/yellow_s20_validation_v1'
REVIEW = VALIDATION/'midpoint_raw_wheel_clock_v1/midpoint_review.json'
OUT = BASE/'c2p1_Ls0_Le35p2_preparation_v1'
read = lambda path: json.loads(path.read_text(encoding='utf-8-sig'))
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def save(path, value):
    with path.open('x', encoding='utf-8') as file:
        json.dump(value, file, ensure_ascii=False, indent=2, allow_nan=False)
        file.write('\n')


review = read(REVIEW)
operator = read(SOURCE/'local_execution/operator_results/A_OUTR135_1800_baseline_20261120_001.json')
assert review['independent_optical_geometry_eligible'] and review['clock']['passed']
assert operator['both_powered_off'] and not operator['contact_detected']
original = read(SOURCE/'candidate.settings.json')
assert original['parameters']['r_min'] == -41 and original['parameters']['correction'] == 0
OUT.mkdir(exist_ok=False)

# 同じ実フレームで旧検出が変わる量を監査する。補間点は一切観測に数えない。
old = pd.read_csv(RUN/'full/apparent_floor_marker_pose.csv')
new = pd.read_csv(VALIDATION/'full/apparent_floor_marker_pose.csv')
joined = old[old.marker_detected_raw & old.calibration_valid].merge(
    new[new.marker_detected_raw & new.calibration_valid], on='frame', suffixes=('_old', '_new'))
deltas = {}
for colour in ('blue', 'yellow'):
    delta = np.hypot(joined[colour+'_x_mm_new']-joined[colour+'_x_mm_old'],
        joined[colour+'_y_mm_new']-joined[colour+'_y_mm_old'])
    deltas[colour] = dict(compared_actual_pairs=len(delta), rms_mm=float(np.sqrt(np.mean(delta**2))),
        max_mm=float(delta.max()), p95_mm=float(delta.quantile(.95)))
save(OUT/'detector_validation.json', dict(
    change='Only rear yellow minimum saturation30->20; red/H/V/opening/area/span unchanged',
    compared_existing_raw_observations=deltas,
    recovered_actual_colour_pairs=9,
    motion_observation_rate=review['independent_clock_required_motion_quality']['observation_rate'],
    per_frame_aruco_ids=[2,3], raw_pair_observation_threshold=.95,
    visual_review='Nine-frame recovered_colour_components.png reviewed: actual pale rear marker, not central yellow PCB.',
    inferred_observations=0, original_full_and_config_preserved=True,
    geometry=str(VALIDATION/'body_geometry_A_out135_yellow_s20.json'),
    source_review=str(REVIEW), hardware_operations=0))

table = (ROOT/'Params/turn_table.h').read_text(encoding='utf-8')
wave = np.fromstring(re.search(r'accel_table\[1001\]\s*=\s*\{([^}]+)\}', table)[1], sep=',')
integral = float(re.search(r'accel_Integral\s*=\s*([.\d]+)', table)[1])


def curve(correction):
    dt = np.deg2rad(135-correction)/(1800/41*integral)/1000
    theta = np.r_[0, np.cumsum((wave[:-1]+wave[1:])/2*(1800/41)*dt)]
    middle = (theta[:-1]+theta[1:])/2
    return np.c_[np.r_[0,np.cumsum(1800*np.sin(middle)*dt)],
        np.r_[0,np.cumsum(1800*np.cos(middle)*dt)]], np.array([np.sin(theta[-1]),np.cos(theta[-1])])


old_curve, old_direction = curve(0.)
new_curve, new_direction = curve(2.1)
delta_angle = new_curve[-1]+46.65*new_direction-old_curve[-1]-46.65*old_direction
predicted_angle = review['exit_path_direction_change_deg']-2.1
exit_direction = np.array([np.sin(np.deg2rad(predicted_angle)),np.cos(np.deg2rad(predicted_angle))])
measured = np.array(review['midpoint_Lend_right_forward_mm'])
target = np.array(review['target_midpoint_Lend_right_forward_mm'])
with_angle = measured+delta_angle
delta_le = (target[0]-with_angle[0])/exit_direction[0]
delta_ls = target[1]-with_angle[1]-delta_le*exit_direction[1]
unconstrained = dict(lstart=15.61+delta_ls, lend=46.65+delta_le)
assert -.9 < unconstrained['lstart'] < -.7
predicted = with_angle + np.array([0.,-15.61])+(35.2-46.65)*exit_direction
stages = []
for index in (1,2,3):
    stage = BASE/f'c2p1_Ls0_Le35p2_{index:02}'
    stage.mkdir(exist_ok=False)
    candidate = copy.deepcopy(original)
    candidate['source'] = 'A1800 out_r135 baseline001 actual video/raw-wheel analysis; provisional next candidate, not adopted.'
    candidate['parameters'].update(lstart=0., lend=35.2, correction=2.1)
    save(stage/'candidate.settings.json', candidate)
    shutil.copyfile(VALIDATION/'body_geometry_A_out135_yellow_s20.json', stage/'body_geometry_A_out135_yellow_s20.json')
    config = read(SOURCE/'config.json')
    config['geometry'] = str(stage/'body_geometry_A_out135_yellow_s20.json')
    save(stage/'config.json', config)
    shutil.copyfile(SOURCE/'static_pose_after_settle.py', stage/'static_pose_after_settle.py')
    policy = read(SOURCE/'static_settle_policy.json')
    policy.update(position_tolerance_mm_unchanged=10., heading_tolerance_deg_unchanged=5.,
        current_policy='Approximate placement per user; registered static raw-pair95% and full-frame motion95% remain required.')
    save(stage/'static_settle_policy.json', policy)
    identifier = f'A_OUTR135_1800_c2p1_Ls0_Le35p2_20261120_{index:03}'
    wrapper = (SOURCE/'run_recorded_once.py').read_text(encoding='utf-8-sig')
    assert wrapper.count('A_OUTR135_1800_baseline_20261120_001') == 1
    wrapper = wrapper.replace('A_OUTR135_1800_baseline_20261120_001',identifier)
    ast.parse(wrapper)
    with (stage/'run_recorded_once.py').open('x',encoding='utf-8') as file:file.write(wrapper)
    plan = read(SOURCE/'placement_plan.json')
    plan.update(state='PREPARED_UNEXECUTED_CURRENT_OPERATOR_STATE_REQUIRED',
        request_id=identifier,next_request_id=identifier,last_request_id=None,
        candidate=str(stage/'candidate.settings.json'), config=str(stage/'config.json'),
        source_review=str(REVIEW),candidate_requested_parameters=candidate['parameters'],
        guide=str(OUT/'start_placement_guide.png'),source_params_changed=False,
        execution_pending=False,next_candidate_executed=False,operator_ready=False,
        current_placement_confirmed=False,current_clearance_confirmed=False,current_hands_clear_confirmed=False,
        power_state='Both_OFF_operator_confirmed_after_baseline001',
        historical_placement_only=True,clearance_model_verified=False)
    save(stage/'placement_plan.json',plan)
    stages.append(dict(stage=str(stage),trial_id=identifier))

save(OUT/'candidate_basis.json',dict(mouse='A',motion='out_r135',preset_mm_s=1800,
    source_review=str(REVIEW),baseline_parameters=original['parameters'],
    next_requested_parameters=candidate['parameters'],stages=stages,
    measured_baseline_exit_angle_deg=review['exit_path_direction_change_deg'],
    measured_baseline_Lend_right_forward_mm=measured.tolist(),
    angle_only_nominal_delta_mm=delta_angle.tolist(),unconstrained_length_solution=unconstrained,
    predicted_exit_angle_deg=predicted_angle,predicted_Lend_right_forward_mm=predicted.tolist(),
    predicted_Lend_residual_mm=(predicted-target).tolist(),predictions_are_not_measurements=True,
    rationale='約2.15度の回り過ぎへ補正+2.1度。角度変更を見込んだ局所解Ls-0.809/Le35.223mmのうち、Lstartは許容下限0、Lend35.2へ丸める。半径41/PID/FFは維持。残差予測約0.8mmは実走で再確認。',
    evaluation_scope='高さ補正相対マーカー中点・実測入口方向。盤面固定接続・絶対車軸/機軸は未校正。',
    original_summary_preserved=review['original_summary'],parameter_adoption_allowed=False,
    ekf=dict(mode_evidence=str(SOURCE/'local_execution/runtime_modes.json'),
        mode='control',current_out135_eligibility=False,parameters_changed=False,
        calibration_status='Unavailable: absolute body yaw/IMU-marker transform uncalibrated; single trial and no independent validation. No new K/noise claim.'),
    hardware_operations=0,firmware_parameters_changed=False,current_power_state='Both_OFF_confirmed',
    input_sha256={str(path):sha(path) for path in (REVIEW,SOURCE/'candidate.settings.json',
        VALIDATION/'body_geometry_A_out135_yellow_s20.json',ROOT/'Params/turn_table.h',Path(__file__))}))

# 同じ成功配置を使う。線は全車幅や穴との干渉の保証ではない。
pre = 90*np.sqrt(2)
lend = np.array([0,pre])+new_curve[-1]+35.2*new_direction
local = np.vstack(([0,0],[0,pre],new_curve+[0,pre],lend,lend+90*new_direction))
h = np.deg2rad(45);basis = np.array([[-np.sin(h),np.cos(h)],[np.cos(h),np.sin(h)]])
board = local@basis.T+[180,135]
plt.rcParams['font.family']='Yu Gothic'
fig,ax=plt.subplots(figsize=(7,7.5),layout='constrained')
ax.set(xlim=(-8,368),ylim=(368,-8),aspect='equal',xlabel='盤面右 [mm]',ylabel='盤面下 [mm]')
ax.set_xticks(np.arange(0,361,45));ax.set_yticks(np.arange(0,361,45));ax.grid(alpha=.35)
for x in range(0,361,90):ax.scatter([x]*5,range(0,361,90),s=17,color='#3e474b')
ax.plot(*board.T,label='次候補の名目中点軌跡（未実走）')
ax.scatter(180,135,c='black',marker='+',s=130)
front=np.array([180,135])+25*np.array([np.cos(h),np.sin(h)])
rear=np.array([180,135])-25*np.array([np.cos(h),np.sin(h)])
ax.plot([rear[0],front[0]],[rear[1],front[1]],c='black')
ax.scatter(*front,c='red',s=65,label='前：赤・右下向き');ax.scatter(*rear,c='gold',edgecolors='black',s=65,label='後：黄')
ax.annotate('開始中点 (180,135) mm',xy=(180,135),xytext=(20,70),arrowprops=dict(arrowstyle='->'))
ax.scatter(225,315,marker='x',s=65,label='Lend格子目標 (225,315)')
ax.scatter(135,315,marker='+',s=85,label='停止格子目標 (135,315)')
ax.set_title('機体A / 1800 mm/s / 右135°出口・次候補\n半径41 / Lstart0 / Lend35.2 mm / 補正＋2.1°')
ax.text(.02,.02,'初期配置は大まかに確認。図は名目中点のみ。\n穴・全接地点・車幅・旋回と停止の空きを実物で確認。',
    transform=ax.transAxes,fontsize=9,bbox=dict(facecolor='white',alpha=.9,edgecolor='lightgray'))
ax.legend(loc='upper right',fontsize=8);fig.savefig(OUT/'start_placement_guide.png',dpi=150);plt.close(fig)

root_plan = BASE/'preparation.json'
shutil.copyfile(root_plan,OUT/'before_preparation.json')
plan=read(BASE/'c2p1_Ls0_Le35p2_01/placement_plan.json')
plan.update(updated_at=datetime.now(timezone.utc).isoformat(),last_request_id='A_OUTR135_1800_baseline_20261120_001',
    last_request_executed=True,last_saved_log_status='saved',both_powered_off_confirmed=True,
    state='NEXT_CANDIDATE_PREPARED_AWAIT_CURRENT_PLACEMENT',source_review=str(REVIEW))
root_plan.write_text(json.dumps(plan,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(prepared=True,next_parameters=candidate['parameters'],stages=stages,
    predicted_residual_mm=(predicted-target).tolist(),hardware_operations=0),ensure_ascii=False))
