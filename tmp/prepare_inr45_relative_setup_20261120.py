"""未実行の再現性確認stageへ、ユーザー指定の相対入口評価を適用する。"""
from pathlib import Path
from hashlib import sha256
from types import SimpleNamespace
import ast
import json

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tools/logs/pixel_camera/turn1800_A_inr45_20261120_v1'
assets = BASE / 'refine_Ls1_Le39p3_assets'
helper = '''def relative_setup_acceptance(original, original_path):
    checked = dict(original)
    reasons = [reason for reason in original['failure_reasons']
               if reason not in ('position_tolerance_exceeded', 'heading_tolerance_exceeded')]
    if original.get('max_position_error_mm', float('inf')) > 10.:
        reasons.append('coarse_position_tolerance_exceeded')
    if original.get('representative_heading_error_deg', float('inf')) > 5.:
        reasons.append('coarse_heading_tolerance_exceeded')
    checked.update(passed=not reasons, failure_reasons=reasons,
        original_strict_failure_reasons=original['failure_reasons'],
        original_static_result=str(original_path),
        user_authorized_relative_entry_alignment=True,
        coarse_setup_limits=dict(position_mm=10., heading_deg=5.),
        stability_limits_unchanged=True,
        evaluation_frame='Measured entry origin/direction from this recorded run; no fixed-board accuracy claim.')
    return checked


'''
ending = '''    original = tuning.read_json(result)
    checked = relative_setup_acceptance(original, result)
    tuning.save_json(folder/'relative_setup_check.json', checked)
    if checked['passed'] is not True:
        raise RuntimeError('Video quality or coarse setup check failed: ' + str(result))
    return checked
'''
for index in (2, 3):
    stage = BASE / f'refine_Ls1_Le39p3_{index:02d}'
    assert not list((stage / 'local_execution/jobs').glob('*.json'))
    script = stage / 'run_recorded_once.py'
    source = script.read_text(encoding='utf-8')
    before = sha256(script.read_bytes()).hexdigest()
    (stage / 'strict_setup_wrapper_source.txt').write_text(source, encoding='utf-8')
    start = source.index('    checked = tuning.read_json(result)')
    end = source.index('\n\n\ntuning.static_check', start)
    source = source[:start] + ending + source[end:]
    anchor = source.index('def dense_static_check(')
    source = source[:anchor] + helper + source[anchor:]
    ast.parse(source)
    script.write_text(source, encoding='utf-8', newline='\n')
    with (stage / 'relative_setup_input_validation.json').open('x', encoding='utf-8') as stream:
        json.dump(dict(user_requested_rough_initial_placement=True,
            user_requested_video_entry_alignment=True, strict_source_sha256=before,
            current_source_sha256=sha256(script.read_bytes()).hexdigest(),
            coarse_setup_limits=dict(position_mm=10., heading_deg=5.),
            original_observation_and_stability_limits_retained=True,
            executed=False), stream, ensure_ascii=False, indent=2)

namespace = {}
exec(compile(ast.parse(helper), '<relative setup policy>', 'exec'), namespace)
check = namespace['relative_setup_acceptance']
cases = [
    (dict(failure_reasons=['heading_tolerance_exceeded'], max_position_error_mm=3., representative_heading_error_deg=3.), True),
    (dict(failure_reasons=['position_tolerance_exceeded'], max_position_error_mm=128., representative_heading_error_deg=1.), False),
    (dict(failure_reasons=['heading_tolerance_exceeded'], max_position_error_mm=3., representative_heading_error_deg=180.), False),
    (dict(failure_reasons=['heading_jitter_rms_exceeded'], max_position_error_mm=3., representative_heading_error_deg=1.), False),
    (dict(failure_reasons=['insufficient_observation_fraction'], max_position_error_mm=3., representative_heading_error_deg=1.), False),
]
for original, expected in cases:
    assert check(original, 'original_static_check.json')['passed'] is expected

tree = ast.parse((BASE/'refine_Ls1_Le39p3_02/run_recorded_once.py').read_text(encoding='utf-8'))
node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'RecordedDevice')
mode_cases = []
for initial, fail_restore in [
        ({'wall_ekf':'shadow', 'wall_edge':'legacy', 'turn_ekf':'control'}, False),
        ({'wall_ekf':'control', 'wall_edge':'control', 'turn_ekf':'shadow'}, False),
        ({'wall_ekf':'control', 'wall_edge':'legacy', 'turn_ekf':'control'}, True)]:
    saved = {}
    class FakeBase:
        def __init__(self):
            self.folder = Path('fake')
            self.base_calls = 0
        def read_settings(self, shell, motion, preset=None):
            self.base_calls += 1
            return 'settings_read'
    class FakeShell:
        def __init__(self):
            self.modes, self.sent = dict(initial), []
        def query(self, command, predicate):
            if command == 'debug camera show':
                line = 'DEBUG_CAMERA_CAP protocol:1'
            else:
                name = command.split()[0]
                line = name.upper() + ' mode:' + self.modes[name] + ' active:0'
            assert predicate(line)
            return line
        def send(self, command):
            name, mode = command.split()
            self.sent.append(command)
            if not fail_restore:
                self.modes[name] = mode
    namespace = dict(TuningService=FakeBase,
        tuning=SimpleNamespace(save_json=lambda p, value: saved.update({str(p): value})))
    exec(compile(ast.Module(body=[node], type_ignores=[]), '<runtime mode check>', 'exec'), namespace)
    device, shell = namespace['RecordedDevice'](), FakeShell()
    rejected = False
    try:
        assert device.read_settings(shell, 'in_r45') == 'settings_read'
        previous = list(shell.sent)
        assert device.read_settings(shell, 'in_r45') == 'settings_read' and previous == shell.sent
    except RuntimeError:
        rejected = True
    assert rejected is fail_restore
    if fail_restore:
        assert device.base_calls == 0
    else:
        assert saved[str(device.folder/'runtime_mode_restore.json')]['verified']
    mode_cases.append(dict(initial=initial, simulated_restore_failure=fail_restore,
        rejected_before_settings_read=rejected, commands=shell.sent, passed=True))
with (assets / 'relative_setup_and_mode_offline_checks.json').open('x', encoding='utf-8') as stream:
    json.dump(dict(setup_cases=5, mode_cases=mode_cases, all_passed=True, hardware_used=False),
        stream, ensure_ascii=False, indent=2)
print('Five setup cases and three runtime mode cases passed offline; executed stage01 unchanged.')
