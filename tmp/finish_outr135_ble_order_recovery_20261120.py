"""走行前2回目の失敗・OFF確認・接続順序変更の検証と再開条件を保存する。"""
from pathlib import Path
from datetime import datetime, timezone
import difflib
import hashlib
import json
import shutil
import pandas as pd

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_outr135_20261120_v1'
OUT=BASE/'ble_preparation_order_recovery_v1'
STAGE=BASE/'c2p1_Ls0_Le35p2_02'
RID='A_OUTR135_1800_c2p1_Ls0_Le35p2_20261120_002'
NEXT=BASE/'c2p1_Ls0_Le35p2_03'
NEXT_ID='A_OUTR135_1800_c2p1_Ls0_Le35p2_20261120_003'
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
now=datetime.now(timezone.utc).isoformat()


def save(path,value):
    with path.open('x',encoding='utf-8') as file:
        json.dump(value,file,ensure_ascii=False,indent=2,allow_nan=False);file.write('\n')


job=read(STAGE/'local_execution/jobs'/f'{RID}.json')
assert job['result']['capture_started'] is False and job['result']['execution_write_attempted'] is False
assert job['result']['ble_disconnected'] and job['result']['static_check']['passed']
assert not (NEXT/'local_execution/jobs'/f'{NEXT_ID}.json').exists()
assert read(STAGE/'candidate.settings.json')==read(NEXT/'candidate.settings.json')
(STAGE/'local_execution/operator_results').mkdir(exist_ok=True)
operator=dict(trial_id=RID,confirmed_at=now,source='current_operator_reply',
    answer='動いていない、停止・両方OFF',ran=False,stopped=True,both_powered_off=True,
    contact_detected=None,execution_write_attempted=False)
save(STAGE/'local_execution/operator_results'/f'{RID}.json',operator)
diagnostics=[]
for number in (1,2):
    rid=f'A_OUTR135_1800_c2p1_Ls0_Le35p2_20261120_{number:03}'
    stage=BASE/f'c2p1_Ls0_Le35p2_{number:02}'
    transport=stage/'local_execution'/f'transport_{rid}'
    raw=pd.read_csv(transport/'telemetry.csv')
    diagnostics.append(dict(request_id=rid,job=read(stage/'local_execution/jobs'/f'{rid}.json'),
        operator=read(stage/'local_execution/operator_results'/f'{rid}.json'),
        telemetry_status=read(transport/'telemetry.status.json'),
        max_ideal_velocity_m_s=float(raw['ideal.velo'].abs().max()),
        min_battery_v=float(raw.Battery.min()),shell_bytes=(transport/'telemetry.shell.bin').stat().st_size,
        error_log=str(transport/'gateway.stderr.txt'),error_log_sha256=sha(transport/'gateway.stderr.txt')))

files=['tools/pixel_camera_tuning.py','tools/pixel_camera_tuning_mcp.py',
    'tools/tests/test_pixel_camera_tuning_mcp.py','tools/tests/test_camera_startup.py','tools/pixel_camera_README.md']
patch=[];sources={}
for name in files:
    path=ROOT/name;before=OUT/('before_'+path.name)
    patch.extend(difflib.unified_diff(before.read_text(encoding='utf-8').splitlines(True),
        path.read_text(encoding='utf-8').splitlines(True),fromfile='a/'+name,tofile='b/'+name))
    shutil.copyfile(path,OUT/('after_'+path.name))
    if path.suffix=='.py':compile(path.read_text(encoding='utf-8'),str(path),'exec')
    sources[name]=dict(before_sha256=sha(before),after_sha256=sha(path),
        uniform_crlf=b'\n' not in path.read_bytes().replace(b'\r\n',b''))
with (OUT/'patch_from_this_task.patch').open('x',encoding='utf-8') as file:file.write(''.join(patch))
save(OUT/'verification.json',dict(mouse='A',motion='out_r135',preset_mm_s=1800,
    state='SOFTWARE_WORKFLOW_VERIFIED_HARDWARE_RECOVERY_UNVERIFIED',
    failure_evidence=diagnostics,operator_current_state='Both_OFF_confirmed',
    diagnosis='Both links ended in Windows STATUS read cancellation during static transfer/analysis; no UART shell query or execute sent. Physical/radio/OS cause not identified.',
    change='LiveSession.prepare_camera can check Pixel without BLE/CSV. MCP performs static capture/analysis first and only then starts one BLE connection. No auto-reconnect or execution replay.',
    before_change_regression=dict(suite='test_pixel_camera_tuning_mcp.py',tests=7,failures=2,
        reason='New tests reject opening BLE before static inspection.'),
    validation=[dict(suite='test_pixel_camera_tuning_mcp.py',tests=7,passed=7),
        dict(suite='test_camera_startup.py',tests=4,passed=4),
        dict(suite='test_mouse_ble_transport.py',tests=36,passed=36),
        dict(suite='test_pixel_camera_tuning.py',tests=21,passed=21,
            environment_note='20 passed in sandbox; GUI Tk initialization failed there. GUI-only rerun outside sandbox passed without hardware access.')],
    test_total_passed=68,skips=0,syntax_validated=True,source_files=sources,
    turn_candidate_parameters_changed=False,firmware_changed=False,hardware_recovery_confirmed=False,
    next_request_id=NEXT_ID,next_candidate=str(NEXT/'candidate.settings.json'),
    request_execution_limit=1,current_operator_ready=False,hardware_operations_for_review=0))
root=BASE/'preparation.json'
shutil.copyfile(root,OUT/'before_root_preparation.json')
plan=read(NEXT/'placement_plan.json')
plan.update(state='CAMERA_FIRST_WORKFLOW_READY_NEW_CURRENT_STATE_REQUIRED',
    updated_at=now,last_request_id=RID,last_request_executed=False,
    last_saved_log_status='not_captured_preflight_failed',
    last_successful_capture_id='A_OUTR135_1800_baseline_20261120_001',
    last_failure_review=str(OUT/'verification.json'),both_powered_off_confirmed=True,
    power_state='Both_OFF_current_operator_confirmed_after_unexecuted_preflight002',
    execution_pending=False,operator_ready=False,current_placement_confirmed=False,
    current_clearance_confirmed=False,current_hands_clear_confirmed=False,
    next_request_id=NEXT_ID,request_id=NEXT_ID,next_candidate_executed=False,
    recovery_order='Camera static capture/transfer/analysis first; then BLE connect/settings/readback/one capture/disconnect',
    camera_health_report=str(OUT/'phone_health_before_next_on.json'))
root.write_text(json.dumps(plan,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(next_request_id=NEXT_ID,test_total_passed=68,hardware_operations=0),ensure_ascii=False))
