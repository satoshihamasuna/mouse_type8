"""実機用モジュールを起動せず、欠けた充電確認で停止するガードと資料を検証。"""
from pathlib import Path
from types import SimpleNamespace
from datetime import datetime,timezone
import ast
import hashlib
import json
import sys
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr45_20261120_v1'
STAGE=BASE/'geometry05_repeat02_correction_m1p2';read=lambda p:json.loads(Path(p).read_text(encoding='utf-8-sig'))
for filename,digest in read(STAGE/'offline_validation.json')['input_sha256'].items():
 assert hashlib.sha256(Path(filename).read_bytes()).hexdigest()==digest,filename
runner=STAGE/'run_recorded_once.py';tree=ast.parse(runner.read_text(encoding='utf-8'))
guards=[node for node in tree.body if isinstance(node,ast.If) and isinstance(node.test,ast.Attribute)
 and isinstance(node.test.value,ast.Name) and node.test.value.id=='args' and node.test.attr=='operator_ready']
assert len(guards)==1;guard=guards[0]
service_lines=[n.lineno for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='RecordedService']
assert service_lines and guard.end_lineno<min(service_lines) and not (STAGE/'charging_completion_confirmed.json').exists()
code=compile(ast.fix_missing_locations(ast.Module(body=[guard],type_ignores=[])),'isolated charging guard','exec')
try:
 exec(code,dict(args=SimpleNamespace(operator_ready=True,trial_id='B_INR45_1600_m1p2_repeat02_green_20261121_013'),OUT=STAGE))
except RuntimeError as error:message=str(error)
else:raise AssertionError('Missing charging confirmation did not stop the isolated guard')
assert '充電完了' in message
sys.path.insert(0,str(ROOT/'tools'))
from pixel_camera_archive import verify_archive,verify_sources
state=read(BASE/'resume_state.json');folder=Path(state['archive']['archive_folder']);manifest=verify_archive(folder)
verify_sources(manifest,Path(state['source_trial']))
assert state['physical_power_state']=='BOTH_OFF_USER_CONFIRMED' and not state['charging_completion_confirmed'] and not state['next_request_sent']
result=dict(verified_at_utc=datetime.now(timezone.utc).isoformat(),candidate_input_hashes_match=True,
 charge_guard_verified_in_isolation=True,guard_precedes_service_creation=True,missing_charge_confirmation_stops=True,
 isolated_error=message,hardware_runner_imported=False,hardware_commands_sent=0,ble_connections_started=0,
 source012_archive_and_originals_verified=True,original_source012_rejection_unchanged=True,request_sent=False,
 charging_completion_confirmed=False,physical_power_state='BOTH_OFF_USER_CONFIRMED',
 input_sha256={str(runner):hashlib.sha256(runner.read_bytes()).hexdigest(),str(Path(__file__)):hashlib.sha256(Path(__file__).read_bytes()).hexdigest()})
with (STAGE/'final_artifact_verification.json').open('x',encoding='utf-8') as f:json.dump(result,f,ensure_ascii=False,indent=2)
print(json.dumps(dict(charge_gate_verified=True,archive_verified=True,input_hashes_match=True,request_sent=False)))
