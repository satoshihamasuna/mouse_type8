"""走行前BLE失敗と操作者OFF確認を保存し、別IDの同一候補を準備する。実機操作なし。"""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json
import shutil
import pandas as pd

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_outr135_20261120_v1'
STAGE=BASE/'c2p1_Ls0_Le35p2_01'
RID='A_OUTR135_1800_c2p1_Ls0_Le35p2_20261120_001'
NEXT=BASE/'c2p1_Ls0_Le35p2_02'
NEXT_ID='A_OUTR135_1800_c2p1_Ls0_Le35p2_20261120_002'
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
now=datetime.now(timezone.utc).isoformat()
job=read(STAGE/'local_execution/jobs'/f'{RID}.json')
assert job['result']['capture_started'] is False
assert job['result']['execution_write_attempted'] is False
assert job['result']['ble_disconnected'] and job['result']['static_check']['passed']
assert read(STAGE/'candidate.settings.json')==read(NEXT/'candidate.settings.json')
assert not (NEXT/'local_execution/jobs'/f'{NEXT_ID}.json').exists()
OUT=BASE/'c2p1_Ls0_Le35p2_preparation_v1/preflight001_failure_review'
OUT.mkdir(exist_ok=False)


def save(path,value):
    with path.open('x',encoding='utf-8') as file:
        json.dump(value,file,ensure_ascii=False,indent=2,allow_nan=False);file.write('\n')


operator=dict(trial_id=RID,confirmed_at=now,source='current_operator_reply',
    answer='動いていない、停止・両方OFF',ran=False,stopped=True,both_powered_off=True,
    contact_detected=None,execution_write_attempted=False)
(STAGE/'local_execution/operator_results').mkdir(exist_ok=True)
save(STAGE/'local_execution/operator_results'/f'{RID}.json',operator)
transport=STAGE/'local_execution'/f'transport_{RID}'
raw=pd.read_csv(transport/'telemetry.csv')
status=read(transport/'telemetry.status.json')
save(OUT/'review.json',dict(mouse='A',motion='out_r135',preset_mm_s=1800,
    trial_id=RID,job=str(STAGE/'local_execution/jobs'/f'{RID}.json'),
    outcome='FAILED_PREFLIGHT_NO_EXECUTION',execution_write_attempted=False,capture_started=False,
    execution_resend_count=0,ble_disconnected=True,operator=operator,
    static_check=job['result']['static_check'],transport_status=status,
    telemetry_rows=len(raw),max_ideal_speed_m_s=float(raw['ideal.velo'].abs().max()),
    max_ideal_angular_speed_rad_s=float(raw['ideal.rad_velo'].abs().max()),
    error='OSError WinError -2147023673 while watch_health/read_gatt_char(STATUS)',
    cause='Read cancellation recorded in Windows BLE layer; physical/radio/OS origin not distinguished.',
    candidate_parameters_changed=False,firmware_parameters_changed=False,hardware_operations_for_review=0,
    next_candidate=str(NEXT/'candidate.settings.json'),next_request_id=NEXT_ID,
    next_execution_allowed_only_after_new_current_placement_confirmation=True,
    source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in
        (transport/'gateway.stderr.txt',STAGE/'local_execution/jobs'/f'{RID}.json',Path(__file__))}))
root=BASE/'preparation.json'
shutil.copyfile(root,OUT/'before_preparation.json')
plan=read(NEXT/'placement_plan.json')
plan.update(state='SAME_CANDIDATE_PREPARED_AFTER_PREFLIGHT_FAILURE_CURRENT_STATE_REQUIRED',
    updated_at=now,last_request_id=RID,last_request_executed=False,
    last_saved_log_status='not_captured_preflight_failed',
    last_successful_capture_id='A_OUTR135_1800_baseline_20261120_001',
    last_failure_review=str(OUT/'review.json'),both_powered_off_confirmed=True,
    power_state='Both_OFF_current_operator_confirmed_after_unexecuted_preflight001',
    execution_pending=False,operator_ready=False,current_placement_confirmed=False,
    current_clearance_confirmed=False,current_hands_clear_confirmed=False,
    next_request_id=NEXT_ID,request_id=NEXT_ID,next_candidate_executed=False)
root.write_text(json.dumps(plan,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(state=plan['state'],next_request_id=NEXT_ID,
    telemetry_rows=len(raw),max_ideal_speed_m_s=float(raw['ideal.velo'].abs().max()),
    hardware_operations=0),ensure_ascii=False))
