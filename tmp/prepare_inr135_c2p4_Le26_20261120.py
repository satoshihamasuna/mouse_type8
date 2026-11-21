"""同一候補3本から、A1800右135入口の角度・Lend候補を準備する。実機操作なし。"""
from pathlib import Path
import ast
import copy
import hashlib
import json
import re
import shutil
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tools/logs/pixel_camera/turn1800_A_inr135_20261120_v1'
SOURCE = BASE / 'c3p5_Ls2p5_Le24_03'
REVIEW = BASE / 'Ls2p5_Le24_c3p5_three_review_v1/review.json'
OUT = BASE / 'lengths_Ls2p5_Le26_c2p4_preparation_v1'
read = lambda p: json.loads(p.read_text(encoding='utf-8-sig'))


def save(path, value):
    with path.open('x', encoding='utf-8', newline='\n') as file:
        json.dump(value, file, ensure_ascii=False, indent=2, allow_nan=False)
        file.write('\n')


review = read(REVIEW)
assert review['trial_count'] == 3 and review['all_relative_motion_quality_passed']
assert review['both_powered_off_confirmed_after_third']
original = read(SOURCE / 'candidate.settings.json')
assert original['parameters']['lstart'] == 2.5 and original['parameters']['lend'] == 24.
assert original['parameters']['correction'] == 3.5
assert round(3.5 + review['mean_exit_angle_deg'] - 135, 1) == 2.4
OUT.mkdir(exist_ok=False)
stages = []
for index in (1, 2, 3):
    stage = BASE / f'c2p4_Ls2p5_Le26_{index:02}'
    stage.mkdir(exist_ok=False)
    candidate = copy.deepcopy(original)
    candidate['parameters'].update(lend=26., correction=2.4)
    save(stage / 'candidate.settings.json', candidate)
    for name in ('config.json', 'static_pose_after_settle.py', 'static_settle_policy.json'):
        shutil.copyfile(SOURCE / name, stage / name)
    identifier = f'A_INR135_1800_c2p4_Ls2p5_Le26_req_20261120_{index:03}'
    code = (SOURCE / 'run_recorded_once.py').read_text(encoding='utf-8-sig')
    old = 'A_INR135_1800_c3p5_Ls2p5_Le24_req_20261120_003'
    assert code.count(old) == 1 and 'position_tolerance_mm=10., heading_tolerance_deg=5.' in code
    code = code.replace(old, identifier)
    ast.parse(code)
    with (stage / 'run_recorded_once.py').open('x', encoding='utf-8', newline='\n') as file:
        file.write(code)
    plan = read(SOURCE / 'placement_plan.json')
    plan.update(next_request_id=identifier, request_id=identifier,
        candidate=str(stage / 'candidate.settings.json'), config=str(stage / 'config.json'),
        source_review=str(REVIEW), candidate_requested_parameters=candidate['parameters'],
        guide=str(OUT / 'start_placement_guide.png'), next_candidate_executed=False,
        execution_pending=False, operator_ready=False, current_placement_confirmed=False,
        current_clearance_confirmed=False, current_hands_clear_confirmed=False,
        current_physical_clearance_verified=False,
        power_state='Both_OFF_confirmed_after_c3p5_Ls2p5_Le24_003', source_params_changed=False)
    save(stage / 'placement_plan.json', plan)
    stages.append(dict(stage=str(stage), trial_id=identifier))

text = (ROOT / 'Params/turn_table.h').read_text(encoding='utf-8')
wave = np.fromstring(re.search(r'accel_table\[1001\]\s*=\s*\{([^}]+)\}', text)[1], sep=',')
integral = float(re.search(r'accel_Integral\s*=\s*([.\d]+)', text)[1])


def nominal_endpoint(correction, lend):
    duration = np.deg2rad(135 - correction) / (45 * integral)
    time = np.linspace(0, duration, 1001)
    omega = 45 * wave
    theta = np.r_[0., np.cumsum((omega[:-1] + omega[1:]) / 2 * np.diff(time))]
    middle = (theta[:-1] + theta[1:]) / 2
    curve = np.array([np.sum(1800 * np.sin(middle) * np.diff(time)),
        92.5 + np.sum(1800 * np.cos(middle) * np.diff(time))])
    return curve + lend * np.array([np.sin(theta[-1]), np.cos(theta[-1])])


angle_delta = nominal_endpoint(2.4, 24.) - nominal_endpoint(3.5, 24.)
predicted_angle = review['mean_exit_angle_deg'] + 1.1
direction = np.array([np.sin(np.deg2rad(predicted_angle)), np.cos(np.deg2rad(predicted_angle))])
predicted = np.array(review['mean_Lend_right_forward_mm']) + angle_delta + 2 * direction
save(OUT / 'candidate_basis.json', dict(mouse='A', motion='in_r135', preset_mm_s=1800,
    input_review=str(REVIEW), prior_requested_parameters=original['parameters'],
    next_requested_parameters=candidate['parameters'], stages=stages,
    measured_mean_exit_angle_deg=review['mean_exit_angle_deg'],
    measured_exit_angle_range_deg=review['exit_angle_range_deg'],
    measured_mean_Lend_residual_mm=review['mean_Lend_residual_right_forward_mm'],
    predicted_angle_deg=predicted_angle, nominal_angle_only_endpoint_delta_mm=angle_delta.tolist(),
    predicted_Lend_right_forward_mm=predicted.tolist(), predicted_residual_mm=(predicted-[90., 135.]).tolist(),
    predictions_are_not_measurements=True,
    rationale='3本共通の角度不足約1.12度に対して補正を+3.5から+2.4度へ減らす。平均右方向不足約1.49mmにはLendを2mm伸ばす。角度変更とLend延長に伴う前方向変位を見込み、Lstart2.5mmは維持する。',
    evaluation_scope='入口直線へ合わせた相対中点。理想角度応答と固定半径を仮定した局所候補であり、実走で再確認する。',
    initial_placement_policy='User requested approximate check: 10mm / 5deg; per-frame ArUco and observation quality retained.',
    source_params_changed=False, hardware_operations=0, parameter_adoption_allowed=False,
    current_power_state='Both_OFF_confirmed', current_operator_ready=False,
    source_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in
        (REVIEW, SOURCE / 'candidate.settings.json', ROOT / 'Params/turn_table.h', Path(__file__))}))
code = (ROOT / 'tmp/guide_inr135_lengths_20261120.py').read_text(encoding='utf-8-sig')
for before, after in {
    "out = base / 'lengths_Ls2p5_Le24_preparation_v1'": "out = base / 'lengths_Ls2p5_Le26_c2p4_preparation_v1'",
    "base / 'c3p5_Ls2p5_Le24_01/candidate.settings.json'": "base / 'c2p4_Ls2p5_Le26_01/candidate.settings.json'",
    'Lend24.0 mm / 補正＋3.5°': 'Lend26.0 mm / 補正＋2.4°',
}.items():
    assert code.count(before) == 1, before
    code = code.replace(before, after)
ast.parse(code)
with (ROOT / 'tmp/guide_inr135_c2p4_Le26_20261120.py').open('x', encoding='utf-8', newline='\n') as file:
    file.write(code)
print(json.dumps(dict(prepared=True, next_parameters=candidate['parameters'], predicted_residual_mm=(predicted-[90., 135.]).tolist(), hardware_operations=0), ensure_ascii=False))
