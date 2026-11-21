"""out45の入力整合とカメラ待機・B専用ポート空きだけを確認する。"""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json
import socket
import sys

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_outr45_20261121_v1'
read=lambda p:json.loads(Path(p).read_text(encoding='utf-8-sig'))
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
state_path=BASE/'resume_state.json';state=read(state_path)
STAGE=Path(state['next_stage']);identifier=state['next_request_id']
assert STAGE.resolve().is_relative_to(BASE.resolve()) and not state['next_request_sent']
assert state['physical_power_state']=='BOTH_OFF_USER_CONFIRMED' and state['execution_commands_sent']==0
config=read(STAGE/'config.json');candidate=read(STAGE/'candidate.settings.json')
charge=read(STAGE/'charging_completion_confirmed.json');health=read(STAGE/'camera_health_20261121.json')
assert candidate['mouse']==config['mouse']=='B' and candidate['motion_name']=='out_r45'
assert candidate['preset_speed_mm_s']==1600 and candidate['turn_count']==1
assert config['address']=='E1:7F:86:21:84:73' and config['ble_port']==8767
assert charge['request_id']==identifier and charge['charging_completed'] and charge['explicit_user_confirmation']
assert health['passed'] and health['robot_commands_sent']==0
assert not (STAGE/'local_execution/jobs'/f'{identifier}.json').exists()
assert not (STAGE/'local_execution'/f'{identifier}.stdout.txt').exists()
assert read(BASE/'guide_source.json')['planned_start_mm']==[90,45]
policy=read(BASE/'analysis_policy.json')
assert policy['minimum_observation_rate']==.95 and policy['per_frame_real_aruco_minimum']==2
helper_names=['review_B_out45_tracking_20261121.py','review_B_out45_midpoint_20261121.py',
 'postprocess_B_out45_20261121.py','record_B_out45_next_operator_ready_20261121.py',
 'checkpoint_B_out45_next_after_run_20261121.py']
paths=[STAGE/'config.json',STAGE/'candidate.settings.json',STAGE/'run_recorded_once.py',
 STAGE/'coarse_placement_check.py',STAGE/'charging_completion_confirmed.json',
 Path(config['reference']),Path(config['geometry']),BASE/'analysis_policy.json',STAGE/'placement_plan.json',
 BASE/'start_placement_guide.jpg']+[ROOT/'tmp'/name for name in helper_names]
for path in paths:
 assert path.is_file(),path
 if path.suffix=='.py':compile(path.read_text(encoding='utf-8-sig'),str(path),'exec')
validation=dict(checked_at_utc=datetime.now(timezone.utc).isoformat(),
 runner_input_validation='VALIDATED; no connection or robot command sent',
 helper_syntax_passed=True,input_sha256={str(path):sha(path) for path in paths},robot_commands_sent=0)
with (STAGE/'offline_validation.json').open('x',encoding='utf-8') as f:json.dump(validation,f,ensure_ascii=False,indent=2)
sys.path.insert(0,str(ROOT/'tools'))
import pixel_camera_tuning_cli
from pixel_camera_capture import camera_workspace
sys.path.insert(0,str(camera_workspace()/'python'))
from pixel_camera import PixelCameraClient
status=PixelCameraClient(connection='wifi',port=8765,auto_start=False).get_status()
device=status.get('device',{})
assert status.get('connected') and device.get('phase')=='IDLE'
assert not device.get('pending_count') and not device.get('owner')
with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as probe:
 if hasattr(socket,'SO_EXCLUSIVEADDRUSE'):probe.setsockopt(socket.SOL_SOCKET,socket.SO_EXCLUSIVEADDRUSE,1)
 probe.bind(('127.0.0.1',8767))
result=dict(checked_at_utc=datetime.now(timezone.utc).isoformat(),ready_for_new_operator_readiness=True,
 camera=dict(connected=True,device={k:device.get(k) for k in ('phase','pending_count','owner','zoom_ratio','torch','focus_lock')}),
 camera_health_passed=True,camera_battery_percent=health['battery']['level_percent'],
 ble_port=8767,ble_port_free=True,ble_address=config['address'],request_id=identifier,
 request_sent=False,operator_ready=False,physical_power_state='BOTH_OFF_USER_CONFIRMED',
 offline_runner_validation_passed=True,placement_guide_visually_reviewed=True,
 charging_explicitly_confirmed=True,robot_commands_sent=0,ble_connections_started=0,
 camera_recording_started=False,input_sha256=validation['input_sha256'])
path=STAGE/'preflight_read_only_20261121.json'
with path.open('x',encoding='utf-8') as f:json.dump(result,f,ensure_ascii=False,indent=2)
state.update(state='OUT45_BASELINE_READY_WAITING_CURRENT_PLACEMENT',latest_preparation=str(path),
 next_camera_health_report=str(STAGE/'camera_health_20261121.json'),
 readiness_question_pending=dict(request_id=identifier,answered=False,current_operator_ready_required=True,
 source='新しいout45図の概略90/45mm・前緑右下45度、干渉余裕、両方ON・手離し・一回可を確認する。'),
 next_action='配置図を案内し、今回の概略配置・干渉余裕・両方ON・手離し・一回可を確認後、基準一回のみ取得する。')
state_path.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(ready=True,request_sent=False,executions=0,camera_phase=device['phase'],
 camera_battery_percent=health['battery']['level_percent'],ble_address=config['address'],ble_port=8767),ensure_ascii=False))
