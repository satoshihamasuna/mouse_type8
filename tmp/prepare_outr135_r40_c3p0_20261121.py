"""A1800 out135の再現した曲がり不足から次の局所候補を準備する。実機操作なし。"""
from pathlib import Path
from datetime import datetime, timezone
import ast
import copy
import hashlib
import json
import re
import shutil
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tools/logs/pixel_camera/turn1800_A_outr135_20261120_v1'
OUT = BASE / 'r40_c3p0_Ls0p5_Le35_preparation_20261121_v1'
SOURCE = BASE / 'c3p8_Ls2_Le36p5_stream10_01'
read = lambda p: json.loads(p.read_text(encoding='utf-8-sig'))

def save(path, value):
    with path.open('x', encoding='utf-8') as f:
        json.dump(value, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.write('\n')

first_path = BASE / 'parallel_analysis/A_OUTR135_1800_c3p8_Ls2_Le36p5_s10_20261121_001/midpoint_actual_motion_v3/midpoint_review.json'
first = read(first_path)
second_dir = BASE / 'c3p8_Ls2_Le36p5_stream10_02/video_only_diagnostic_v1'
second = read(second_dir / 'direction_diagnostic_v2.json')
first_opt = read(BASE / 'c3p8_first_trial_review_20261121_v1/optical_direction_method_comparison_v1/direction_diagnostic_v2.json')
previous_path = BASE / 'parallel_analysis/A_OUTR135_1800_c2p1_Ls0_Le37_s10_20261121_001/midpoint_actual_motion_v3/midpoint_review.json'
previous = read(previous_path)
assert first['independent_optical_geometry_eligible'] and first['clock']['passed']
assert not second['parameter_adoption_allowed'] and second['required_raw_registered_observation_rate'] >= .95
angles = lambda v: np.array([e['angle_deg'] for e in v['direction_window_sensitivity']
    if e.get('entry', {}).get('passed') and e.get('exit', {}).get('passed')])
a, b = angles(first_opt), angles(second)
assert len(a) >= 3 and len(b) >= 3 and np.ptp(a) < .3 and np.ptp(b) < .3
angle_mean = float(np.mean([np.median(a), np.median(b)]))
slope = (angle_mean-previous['exit_path_direction_change_deg'])/(3.8-2.1)
secant_correction = 3.8+(135-angle_mean)/slope
table = (ROOT / 'Params/turn_table.h').read_text(encoding='utf-8')
wave = np.fromstring(re.search(r'accel_table\[1001\]\s*=\s*\{([^}]+)\}', table)[1], sep=',')
integral = float(re.search(r'accel_Integral\s*=\s*([.\d]+)', table)[1])

def direction(angle):
    return np.array([np.sin(np.deg2rad(angle)), np.cos(np.deg2rad(angle))])

def arc(radius, angle):
    dt = np.deg2rad(angle)/(1800/radius*integral)/1000
    theta = np.r_[0, np.cumsum((wave[:-1]+wave[1:])/2*(1800/radius)*dt)]
    middle = (theta[:-1]+theta[1:])/2
    return np.array([np.sum(1800*np.sin(middle)*dt), np.sum(1800*np.cos(middle)*dt)])

old_angle = first['exit_path_direction_change_deg']
measured = np.array(first['midpoint_Lend_right_forward_mm'])
target = np.array(first['target_midpoint_Lend_right_forward_mm'])
nominal_angle_radius_delta = arc(40,135)-arc(41,old_angle)+36.5*(direction(135)-direction(old_angle))
adjusted = measured+nominal_angle_radius_delta
delta_le = (target[0]-adjusted[0])/direction(135)[0]
delta_ls = target[1]-adjusted[1]-delta_le*direction(135)[1]
predicted = adjusted + [0, .5-2] + (35-36.5)*direction(135)
OUT.mkdir(exist_ok=False)
second_rid = 'A_OUTR135_1800_c3p8_Ls2_Le36p5_s10_20261121_002'
second_stage = BASE / 'c3p8_Ls2_Le36p5_stream10_02'
operator_path = second_stage / 'local_execution/operator_results' / f'{second_rid}.json'
operator = read(operator_path)
assert operator['both_powered_off'] and operator['stopped'] and not operator['contact_detected']
stages = []
for index in (1,2,3):
    stage = BASE / f'r40_c3p0_Ls0p5_Le35_stream10_{index:02}'
    stage.mkdir(exist_ok=False)
    candidate = copy.deepcopy(read(SOURCE / 'candidate.settings.json'))
    candidate['source'] = 'Provisional A1800 out135 local angle/radius/length candidate; two repeat optical directions, first complete1ms only. Not adopted.'
    candidate['parameters'].update(r_min=-40., lstart=.5, lend=35., correction=3.)
    save(stage / 'candidate.settings.json', candidate)
    for name in ('body_geometry_A_out135_yellow_s20.json','static_pose_after_settle.py','static_settle_policy.json'):
        shutil.copyfile(SOURCE/name, stage/name)
    config = read(SOURCE / 'config.json')
    config['geometry'] = str(stage / 'body_geometry_A_out135_yellow_s20.json')
    save(stage / 'config.json', config)
    rid = f'A_OUTR135_1800_r40_c3p0_Ls0p5_Le35_s10_20261121_{index:03}'
    wrapper = (SOURCE / 'run_recorded_once.py').read_text(encoding='utf-8-sig')
    old_rid = 'A_OUTR135_1800_c3p8_Ls2_Le36p5_s10_20261121_001'
    assert wrapper.count(old_rid) == 1
    wrapper = wrapper.replace(old_rid,rid)
    ast.parse(wrapper)
    with (stage / 'run_recorded_once.py').open('x', encoding='utf-8') as f:
        f.write(wrapper)
    plan = read(SOURCE / 'placement_plan.json')
    plan.update(state='LOCAL_RADIUS_ANGLE_LENGTH_CANDIDATE_UNEXECUTED',request_id=rid,next_request_id=rid,
        candidate=str(stage/'candidate.settings.json'),config=str(stage/'config.json'),candidate_requested_parameters=candidate['parameters'],
        operator_ready=False,execution_pending=False,next_candidate_executed=False,current_placement_confirmed=False,
        current_clearance_confirmed=False,current_hands_clear_confirmed=False,current_physical_clearance_verified=False,
        both_powered_off_confirmed=True,power_state='Both_OFF_operator_confirmed_after_second_c3p8_trial',
        source_params_changed=False,source_review=str(OUT/'candidate_basis.json'),last_request_id=second_rid,
        last_request_executed=True,last_saved_log_status='failed',last_contact_detected=False,
        camera_health_report=str(BASE/'c3p8_second_trial_review_20261121_v1/phone_health_before_on.json'),
        updated_at=datetime.now(timezone.utc).isoformat())
    save(stage/'placement_plan.json',plan)
    stages.append(dict(stage=str(stage),trial_id=rid))
save(OUT/'candidate_basis.json',dict(mouse='A',motion='out_r135',preset_mm_s=1800,
    first_complete_review=str(first_path),second_video_only_review=str(second_dir/'direction_diagnostic_v2.json'),
    second_trial_retained1ms_available=False,second_trial_original_state='INCOMPLETE_LOG',operator=operator,
    first_optical_angle_range_deg=[float(a.min()),float(a.max())],second_optical_angle_range_deg=[float(b.min()),float(b.max())],
    repeat_optical_angle_medians_deg=[float(np.median(a)),float(np.median(b))],
    prior_c2p1_angle_deg=previous['exit_path_direction_change_deg'],empirical_angle_response_deg_per_correction_deg=float(slope),
    secant_correction_candidate_deg=float(secant_correction),rounded_correction_deg=3.,
    radius_mm=40.,unrounded_local_lengths_mm=dict(lstart=float(2+delta_ls),lend=float(36.5+delta_le)),
    rounded_lengths_mm=dict(lstart=.5,lend=35.),predicted_Lend_right_forward_mm=predicted.tolist(),
    predicted_residual_mm=(predicted-target).tolist(),predictions_are_not_measurements=True,
    rationale='同設定の相対出口方向が133.3/133.5度に再現。補正secant約2.96度を3.0度へ。半径41の局所長さ解はLstart負となるため、半径40で局所解Ls0.487/Le34.864mmを0.5/35へ丸め、実測する。',
    limitations=['角度secantは独立した制御同定ではなく、電圧と過去の長さ差も含む。',
        '半径と角度の位置差分は既存波形の局所予測。実測形状との偏りがあり、次走行で確認する。',
        '2本目は動画のみ。Lend位置・制御追従・保持1msを回収済みとは扱わない。',
        '入口に合わせた相対中点評価。絶対車軸・機軸yaw・盤面固定接続は未校正。'],
    parameter_adoption_allowed=False,firmware_changed=False,hardware_operations=0,
    ekf_parameters_changed=False,ekf_status='out135 not eligible in current firmware; no identified K/noise adoption',stages=stages,
    input_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in
        (first_path,second_dir/'direction_diagnostic_v2.json',previous_path,operator_path,ROOT/'Params/turn_table.h',Path(__file__))}))
shutil.copyfile(BASE/'preparation.json',OUT/'before_preparation.json')
plan=read(BASE/'r40_c3p0_Ls0p5_Le35_stream10_01/placement_plan.json')
plan['last_successful_capture_id']='A_OUTR135_1800_c3p8_Ls2_Le36p5_s10_20261121_001'
(BASE/'preparation.json').write_text(json.dumps(plan,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(prepared=True,parameters=candidate['parameters'],angle_repeat_medians=[float(np.median(a)),float(np.median(b))],
    secant_correction_deg=float(secant_correction),predicted_residual_mm=(predicted-target).tolist(),hardware_operations=0)))
