"""A1800右135入口の完全ログ002から、次の長さ候補を新規準備する。実機操作なし。"""
from pathlib import Path
import ast
import copy
import hashlib
import json
import shutil

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tools/logs/pixel_camera/turn1800_A_inr135_20261120_v1'
SOURCE = BASE / 'c3p5_Ls7p85_02'
REVIEW = BASE / 'parallel_analysis/A_INR135_1800_c3p5_Ls7p85_req_20261120_002/midpoint_raw_wheel_clock_v1/midpoint_review.json'
OUT = BASE / 'lengths_Ls2p5_Le24_preparation_v1'
read = lambda p: json.loads(p.read_text(encoding='utf-8-sig'))


def save(path, value):
    with path.open('x', encoding='utf-8', newline='\n') as file:
        json.dump(value, file, ensure_ascii=False, indent=2, allow_nan=False)
        file.write('\n')


review = read(REVIEW)
assert review['independent_optical_geometry_eligible'] is True
assert review['clock']['passed'] is True
assert abs(review['exit_path_error_from135_deg']) < .5
assert read(SOURCE / 'local_execution/operator_results/A_INR135_1800_c3p5_Ls7p85_req_20261120_002.json')['both_powered_off']
original = read(SOURCE / 'candidate.settings.json')
assert original['parameters']['lstart'] == 7.85 and original['parameters']['lend'] == 34.81
assert original['parameters']['correction'] == 3.5
OUT.mkdir(exist_ok=False)
stages = []
for index in (1, 2, 3):
    stage = BASE / f'c3p5_Ls2p5_Le24_{index:02}'
    stage.mkdir(exist_ok=False)
    candidate = copy.deepcopy(original)
    candidate['parameters'].update(lstart=2.5, lend=24.)
    save(stage / 'candidate.settings.json', candidate)
    for name in ('config.json', 'static_pose_after_settle.py', 'static_settle_policy.json'):
        shutil.copyfile(SOURCE / name, stage / name)
    identifier = f'A_INR135_1800_c3p5_Ls2p5_Le24_req_20261120_{index:03}'
    code = (SOURCE / 'run_recorded_once.py').read_text(encoding='utf-8-sig')
    old = 'A_INR135_1800_c3p5_Ls7p85_req_20261120_002'
    assert code.count(old) == 1
    code = code.replace(old, identifier)
    ast.parse(code)
    with (stage / 'run_recorded_once.py').open('x', encoding='utf-8', newline='\n') as file:
        file.write(code)
    plan = read(SOURCE / 'placement_plan.json')
    plan.update(next_request_id=identifier, request_id=identifier,
        candidate=str(stage / 'candidate.settings.json'), config=str(stage / 'config.json'),
        next_candidate_executed=False, execution_pending=False, operator_ready=False,
        current_placement_confirmed=False, current_clearance_confirmed=False,
        current_hands_clear_confirmed=False, current_physical_clearance_verified=False,
        power_state='Both_OFF_confirmed_after_c3p5_Ls7p85_002',
        source_review=str(REVIEW), guide=str(OUT / 'start_placement_guide.png'),
        candidate_requested_parameters=candidate['parameters'],
        historical_placement_only=True, source_params_changed=False,
        workflow='今回の配置・両方ON・手離しを確認し、静止照合後に新しい1回のみ。採用は独立反復と目標残差を確認してから。')
    plan['static_sampling']['acceptance_version'] = 5
    save(stage / 'placement_plan.json', plan)
    stages.append(dict(stage=str(stage), trial_id=identifier))

predicted = review['midpoint_Lend_right_forward_mm']
save(OUT / 'candidate_basis.json', dict(mouse='A', motion='in_r135', preset_mm_s=1800,
    input_review=str(REVIEW), measured_Lend_right_forward_mm=predicted,
    observed_exit_angle_deg=review['exit_path_direction_change_deg'],
    current_requested_parameters=original['parameters'], current_actual_parameters=review['current_parameters'],
    length_only_solution_from_actual_readback=review['length_only_candidate_at_current_angle'],
    next_requested_parameters=candidate['parameters'], stages=stages,
    rationale='独立入口時計で得た局所解Ls2.252/Le23.849mmを、要求Ls2.5/Le24.0mmへ丸める。半径・角度補正・PID・FFは元要求値を維持する。',
    full_summary_preserved=review['original_summary'],
    evaluation_scope='相対マーカー中点・実測入口方向。盤面固定接続・絶対車軸位置は未検証。',
    source_params_changed=False, hardware_operations=0, next_candidate_executed=False,
    source_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in
        (REVIEW, SOURCE / 'candidate.settings.json', ROOT / 'Params/Param_A/turn_1800/turn_1800.cpp', Path(__file__))}))

guide_source = (ROOT / 'tmp/guide_inr135_resume_20261120.py').read_text(encoding='utf-8-sig')
for before, after in {
    "out = base / 'resume_20261120_inr135_v1'": "out = base / 'lengths_Ls2p5_Le24_preparation_v1'",
    "base / 'c3p5_Ls7p85_01/candidate.settings.json'": "base / 'c3p5_Ls2p5_Le24_01/candidate.settings.json'",
    'Lstart7.85 / Lend34.81': 'Lstart2.5 / Lend24.0',
}.items():
    assert guide_source.count(before) == 1, before
    guide_source = guide_source.replace(before, after)
ast.parse(guide_source)
guide_path = ROOT / 'tmp/guide_inr135_lengths_20261120.py'
with guide_path.open('x', encoding='utf-8', newline='\n') as file:
    file.write(guide_source)
print(json.dumps(dict(prepared=True, stages=stages, hardware_operations=0), ensure_ascii=False))
