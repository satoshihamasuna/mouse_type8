"""オフライン準備検証と光学比較の要点を新規記録する。"""
from pathlib import Path
import ast
import hashlib
import json

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tools/logs/pixel_camera/turn1800_A_r180_20261120_v1'
STAGE = BASE / 'finish02_01'
SOURCE = BASE / 'corr3p5_r49_Le34p5_02'


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


candidate = read(STAGE / 'candidate.settings.json')
original = read(SOURCE / 'candidate.settings.json')
assert candidate['parameters'] == dict(original['parameters'], r_min=-48.0, lend=31.5)
for name in ('pid', 'feedforward', 'suction', 'mouse', 'motion_name', 'preset_speed_mm_s', 'turn_count'):
    assert candidate[name] == original[name], name
assert read(STAGE / 'config.json') == read(SOURCE / 'config.json')
wrapper = ast.parse((STAGE / 'run_recorded_once.py').read_text(encoding='utf-8'))
prepare = [node for node in ast.walk(wrapper) if isinstance(node, ast.Call)
           and isinstance(node.func, ast.Attribute) and node.func.attr == 'prepare']
assert len(prepare) == 1
assert ast.literal_eval(prepare[0].args[-2]) is None
assert ast.literal_eval(prepare[0].args[-1]) == 20.0
optical = read(BASE / 'parallel_finish/candidate_two_optical_clock_v1/report.json')['results']
comparison = read(BASE / 'parallel_finish/candidate_two_relative_v1/video_comparison.json')
report = dict(mouse='A', motion='long_r180', stage=str(STAGE),
    parameters=candidate['parameters'], source_stage=str(SOURCE),
    offline_validation=dict(
        command='C:/Users/sato1/Documents/git/Debug_camera/.venv/Scripts/python.exe ' + str(STAGE / 'run_recorded_once.py'),
        operator_ready_argument=False, returncode=0,
        stdout='VALIDATED; no connection or robot command sent', robot_commands_sent=0,
        source_wrapper_preserved_except_trial_id=True, post_straight_accel_m_s2=20.0,
        identical_pid_ff_suction_and_capture_settings=True),
    input_analysis_python='D:/venv/venv3-11/Scripts/python.exe',
    optical_metrics=[dict(label=r['label'], angle_deg=r['optical_path_angle_deg'],
        return_spacing_forward45_mm=r['optical_return_spacing_at_forward45_mm'],
        Lend_right_forward_mm=r['optical_Lend_right_forward_mm'],
        apex_forward_mm=r['optical_apex_forward_mm'], independent_optical_geometry_eligible=r['independent_optical_geometry_eligible'],
        original_encoder_path_review_eligible=r['original_encoder_path_review_eligible'],
        encoder_motion_rms_mm=r['encoder_path_reconstruction_motion_rms_mm']) for r in optical],
    mean=dict(angle_deg=sum(r['optical_path_angle_deg'] for r in optical)/2,
        return_spacing_forward45_mm=sum(r['optical_return_spacing_at_forward45_mm'] for r in optical)/2,
        Lend_forward_mm=sum(r['optical_Lend_right_forward_mm'][1] for r in optical)/2),
    observation_rates=[c['motion_observation_rate'] for c in comparison['comparisons']],
    return_spacing_forward45_extrapolated=[not c['exit_reference45_inside_observed_extent'] for c in comparison['comparisons']],
    geometric_response_identified=False,
    trial03_excluded='full/summary.json欠落かつrequired_motion_window不合格。入力へ混在させていない。',
    parameter_adoption_allowed=False, fixed_board_acceptance=False, body_axle_absolute_pose_verified=False,
    source_params_changed=False,
    file_sha256={name:hashlib.sha256((STAGE/name).read_bytes()).hexdigest()
                 for name in ('candidate.settings.json', 'config.json', 'placement_plan.json', 'run_recorded_once.py', 'static_pose_after_settle.py')})
path = BASE / 'parallel_finish/finish02_preparation_review.json'
with path.open('x', encoding='utf-8') as file:
    json.dump(report, file, ensure_ascii=False, indent=2, allow_nan=False)
    file.write('\n')
print(json.dumps(dict(review=str(path), mean=report['mean'], observation_rates=report['observation_rates'],
    return_spacing_forward45_extrapolated=report['return_spacing_forward45_extrapolated']), ensure_ascii=False))
