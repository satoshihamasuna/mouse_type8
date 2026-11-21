"""既存の静止照合をBLE接続より先に行い、その同一要求で単体1回を取得する。"""
from pathlib import Path
from datetime import datetime, timezone
from hashlib import sha256
from contextlib import redirect_stdout, redirect_stderr
import argparse
import ast
import json
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--stage', type=Path, required=True)
parser.add_argument('--trial-id', required=True)
parser.add_argument('--operator-ready', action='store_true')
parser.add_argument('--preflight', type=Path, help='同じ未実行・未移動の現在配置で直前に取得した静止資料')
args = parser.parse_args()
stage = args.stage.resolve()
assert stage.is_relative_to(ROOT / 'tools/logs/pixel_camera/turn1800_A_inr45_20261120_v1')
wrapper = stage / 'run_recorded_once.py'
tree = ast.parse(wrapper.read_text(encoding='utf-8'))
cut = next(i for i, node in enumerate(tree.body) if isinstance(node, ast.Assign)
    and any(isinstance(target, ast.Name) and target.id == 'parser' for target in node.targets))
namespace = dict(__file__=str(wrapper), __name__='recording_procedure')
exec(compile(ast.Module(body=tree.body[:cut], type_ignores=[]), str(wrapper), 'exec'), namespace)
tuning = namespace['tuning']
service = namespace['RecordedService'](stage / 'local_execution', {})
placement = dict(x_mm=225., y_mm=45., heading_deg=90., position_tolerance_mm=5., heading_tolerance_deg=2.)
payload = service.prepare(str(stage/'config.json'), str(stage/'candidate.settings.json'), placement, 20., None)
if not args.operator_ready:
    print('VALIDATED: camera-first ordering; no device connection or robot command')
    raise SystemExit(0)

sys.path.insert(0, str(namespace['ROOT'] / 'tools'))
from pixel_camera_capture import camera_workspace
sys.path.insert(0, str(camera_workspace() / 'python'))
from pixel_camera import PixelCameraClient
identifier = args.trial_id
assert not (service.folder/'jobs'/(identifier+'.json')).exists()
preflight = service.folder / ('preflight_before_ble_' + identifier)
preflight.mkdir(exist_ok=False)
def semantic_digest(path):
    value = json.loads(Path(path).read_text(encoding='utf-8'))
    return sha256(json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(',', ':')).encode('utf-8')).hexdigest()

config_hashes = {key: semantic_digest(payload['config'][key])
    for key in ('reference', 'geometry')}
result = dict(state='PREPARING', execution_write_attempted=False, ble_connection_started=False)
with (service.folder/(identifier+'.stdout.txt')).open('x', encoding='utf-8') as log:
    with redirect_stdout(log), redirect_stderr(log):
        camera = None
        try:
            if args.preflight:
                source = args.preflight.resolve()
                assert source.is_relative_to(stage.parent)
                plan = tuning.read_json(source/'static_plan.json')
                quality = tuning.read_json(source/'static_window_quality.json')
                manifest = tuning.read_json(plan['camera_manifest'])
                age = (datetime.now(timezone.utc)-datetime.fromisoformat(manifest['started_at'])).total_seconds()
                assert quality['passed'] and 0 <= age <= 600
                assert plan['placement'] == placement
                assert all(semantic_digest(plan[key]) == digest for key, digest in config_hashes.items())
                assert manifest['camera_settings'] == payload['config']['camera_settings']
                assert sha256(Path(plan['video']).read_bytes()).hexdigest() == quality['video_sha256']
                original = tuning.read_json(source/'static_check.json')
                checked = namespace['relative_setup_acceptance'](original, source/'static_check.json')
                assert checked['passed']
                checked.update(current_unexecuted_setup_static_source=str(source),
                    source_static_age_s=age, source_video_sha256=quality['video_sha256'])
                tuning.save_json(preflight/'current_setup_reference.json', checked)
            else:
                # この時点ではBLE中継を起動しない。静止画像と盤面登録だけを検査する。
                camera = PixelCameraClient(connection='wifi', port=8765, auto_start=False)
                device = service.capture_service(payload['config'])
                checked = tuning.static_check(camera, device, preflight, placement)
        except Exception as error:
            result.update(state='FAILED_BEFORE_BLE', error=f'{type(error).__name__}: {error}')
            tuning.save_json(preflight/'procedure_result.json', result)
        finally:
            if camera is not None:
                camera.disconnect()
        if result['state'] != 'FAILED_BEFORE_BLE':
            assert checked['passed'] is True
            preflight_finished_at = datetime.now(timezone.utc).isoformat()

            def already_checked(camera_client, live_device, folder, actual_placement):
                # 過去試行ではなく、この要求で直前に測った同じ入力の照合を渡す。
                assert actual_placement == placement
                assert all(semantic_digest(live_device.config[key]) == digest
                    for key, digest in config_hashes.items())
                proof = dict(checked, preflight_before_ble=str(preflight),
                    preflight_finished_at=preflight_finished_at,
                    current_request_id=identifier, measurement_conducted_before_ble=True)
                tuning.save_json(folder/'camera_first_preflight_reference.json', proof)
                return proof

            tuning.static_check = already_checked
            payload['operator_ready'] = True
            result['ble_connection_started'] = True
            service.submit('capture', payload, identifier)
            while True:
                job = service.status(identifier)
                if job['state'] in ('DONE', 'FAILED'):
                    break
                time.sleep(.5)
            result = job.get('result', dict(state='FAILED', error=job.get('error')))
            tuning.save_json(preflight/'procedure_result.json', result)
            log.write(json.dumps(job, ensure_ascii=False, indent=2)+'\n')
print(json.dumps(dict(state=result['state'], error=result.get('error'),
    execution_write_attempted=result.get('execution_write_attempted'),
    trial_json=result.get('trial_json'), ble_disconnected=result.get('ble_disconnected'),
    preflight_before_ble=str(preflight)), ensure_ascii=False), flush=True)
raise SystemExit(0 if result['state'] == 'CAPTURED_REVIEW_REQUIRED' else 1)
