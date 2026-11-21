"""A1800 in_r45の微調整候補をオフラインで準備する。実機接続なし。"""
from pathlib import Path
from datetime import datetime, timezone
from hashlib import sha256
import ast
import json
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tools/logs/pixel_camera/turn1800_A_inr45_20261120_v1'
REPORT = BASE / 'single_turn_candidate_3run_review_v1/review.json'
source_stage = BASE / 'repeat03_resume01'
run = source_stage / 'local_execution/run_A_INR45_1800_repeat03_resume_20261120_001'
assets = BASE / 'refine_Ls1_Le39p3_assets'
assert not assets.exists()
command = [sys.executable, str(ROOT / 'tmp/mouse_tuning_prepare_candidate_20261120.py'),
    '--base', str(source_stage), '--source-run', str(run),
    '--settings', str(BASE / 'single_turn_candidate_3run_review_v1/candidate.settings.json'),
    '--name', 'refine_Ls1_Le39p3_', '--id-prefix', 'A_INR45_1800_Ls1_Le39p3',
    '--count', '3', '--date', '20261120', '--lstart', '1.0', '--lend', '39.3', '--basis', str(REPORT)]
result = subprocess.run(command, capture_output=True, text=True)
assert result.returncode == 0, result.stderr
prepared = json.loads(result.stdout)
source_config = json.loads((source_stage / 'config.json').read_text(encoding='utf-8'))
assets.mkdir()
reference = assets / 'marker_reference_A_snapshot.json'
geometry = assets / 'body_geometry_A_yellow_s20_validated_snapshot.json'
shutil.copy2(source_config['reference'], reference)
shutil.copy2(BASE / 'rear_yellow_s20_validation_v1/body_geometry_A_yellow_s20_candidate.json', geometry)

replacement = '''class RecordedDevice(TuningService):
    def read_settings(self, shell, motion, preset=None):
        if not getattr(self, 'modes_recorded', False):
            expected = {'wall_ekf show': 'shadow', 'wall_edge show': 'legacy', 'turn_ekf show': 'control'}
            before, modes, restore_commands = {}, {}, []
            for command, prefix in (('debug camera show', 'DEBUG_CAMERA_CAP '),
                                    ('wall_ekf show', 'WALL_EKF '),
                                    ('wall_edge show', 'WALL_EDGE '),
                                    ('turn_ekf show', 'TURN_EKF ')):
                response = shell.query(command, lambda line, p=prefix: line.startswith(p))
                before[command] = response
                wanted = expected.get(command)
                if wanted and ('mode:' + wanted) not in response.split():
                    restore = command.split()[0] + ' ' + wanted
                    shell.send(restore)
                    restore_commands.append(restore)
                    response = shell.query(command, lambda line, p=prefix: line.startswith(p))
                    if ('mode:' + wanted) not in response.split():
                        raise RuntimeError('Runtime mode readback mismatch: ' + command)
                modes[command] = response
            tuning.save_json(self.folder/'runtime_modes_before_restore.json', before)
            tuning.save_json(self.folder/'runtime_modes.json', modes)
            tuning.save_json(self.folder/'runtime_mode_restore.json', dict(expected=expected,
                commands=restore_commands, verified=True))
            self.modes_recorded = True
        return super().read_settings(shell, motion, preset)


'''
sys.path.insert(0, str(ROOT / 'tools'))
from pixel_camera_tuning_mcp import McpTuningService
from pixel_camera_tuning import setting_differences

for record in prepared['stages']:
    stage = Path(record['stage'])
    config_path = stage / 'config.json'
    config = json.loads(config_path.read_text(encoding='utf-8'))
    config.update(reference=str(reference), geometry=str(geometry))
    config_path.write_text(json.dumps(config, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    wrapper_path = stage / 'run_recorded_once.py'
    wrapper = wrapper_path.read_text(encoding='utf-8')
    start, end = wrapper.index('class RecordedDevice('), wrapper.index('class RecordedService(')
    wrapper = wrapper[:start] + replacement + wrapper[end:]
    ast.parse(wrapper)
    wrapper_path.write_text(wrapper, encoding='utf-8', newline='\n')
    plan_path = stage / 'placement_plan.json'
    plan = json.loads(plan_path.read_text(encoding='utf-8'))
    plan.update(operator_ready=False, execution_pending=False, current_placement_confirmed=False,
        current_clearance_confirmed=False, current_hands_clear_confirmed=False,
        current_physical_clearance_verified=False, power_state='OFF_REQUESTED_CURRENT_CONFIRMATION_PENDING',
        source_profile_sha256=sha256((ROOT/'Params/Param_A/turn_1800/turn_1800.cpp').read_bytes()).hexdigest(),
        current_source_hash_is_firmware_identity=False,
        expected_runtime_modes=dict(wall_ekf='shadow', wall_edge='legacy', turn_ekf='control'))
    plan_path.write_text(json.dumps(plan, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    service = McpTuningService(stage / 'local_execution', {})
    payload = service.prepare(str(config_path), str(stage/'candidate.settings.json'),
        record['placement'], record['post_diagonal_accel_m_s2'], record['post_straight_accel_m_s2'])
    original = json.loads((BASE/'single_turn_candidate_3run_review_v1/candidate.settings.json').read_text(encoding='utf-8'))
    differences = setting_differences(payload['settings'], original)
    assert set(differences) == {'parameters.lstart', 'parameters.lend'}, differences
    verification = dict(state='VALIDATED_NOT_EXECUTED', request_id=record['request_id'],
        changed_parameters=dict(lstart_mm=1.0, lend_mm=39.3), unchanged=['radius', 'angle_correction', 'PID', 'FF', 'suction'],
        geometry_change='Rear-yellow saturation only, 30 to20; independently validated on three saved trials.',
        snapshot_sha256={str(p): sha256(p.read_bytes()).hexdigest() for p in
            (config_path, wrapper_path, reference, geometry, stage/'candidate.settings.json')},
        physical_execution=False, robot_commands_sent=0)
    with (stage/'refinement_input_validation.json').open('x', encoding='utf-8') as stream:
        json.dump(verification, stream, ensure_ascii=False, indent=2)

summary = dict(prepared_at_utc=datetime.now(timezone.utc).isoformat(), mouse='A', motion='in_r45',
    preset_mm_s=1800, previous_parameters=dict(lstart_mm=2., lend_mm=40.),
    candidate_parameters=dict(lstart_mm=1., lend_mm=39.3, r_min_mm=-53., correction_deg=0.),
    basis_report=str(REPORT), planned_runs=3, each_requires_fresh_operator_confirmation=True,
    first_request_id=prepared['stages'][0]['request_id'], first_stage=prepared['stages'][0]['stage'],
    camera_health=str(BASE/'health_resume1859_01.json'), physical_execution=False,
    workflow='One candidate trial first; compare before further repeated trials. Same runtime modes as prior three runs.')
with (assets/'refinement_plan.json').open('x', encoding='utf-8') as stream:
    json.dump(summary, stream, ensure_ascii=False, indent=2)
print(json.dumps(summary, ensure_ascii=False), flush=True)
