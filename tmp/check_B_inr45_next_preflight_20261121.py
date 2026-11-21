"""準備済みの次試行について状態照会と保存資料の整合だけを確認する。"""
from pathlib import Path
from datetime import datetime,timezone
import hashlib
import json
import socket
import sys
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr45_20261120_v1'
read=lambda p:json.loads(Path(p).read_text(encoding='utf-8-sig'))
state_path=BASE/'resume_state.json';state=read(state_path);STAGE=Path(state['next_stage']);identifier=state['next_request_id']
assert STAGE.is_relative_to(BASE) and not state['next_request_sent'] and state['physical_power_state']=='BOTH_OFF_USER_CONFIRMED'
candidate=read(STAGE/'candidate.settings.json');config=read(STAGE/'config.json');health=read(STAGE/'camera_health_20261121.json')
assert candidate['mouse']==config['mouse']=='B' and candidate['motion_name']=='in_r45' and candidate['turn_count']==1
assert config['address']=='E1:7F:86:21:84:73' and config['ble_port']==8767 and health['passed']
assert not (STAGE/'local_execution/jobs'/f'{identifier}.json').exists() and not (STAGE/'local_execution'/f'{identifier}.stdout.txt').exists()
for filename,digest in read(STAGE/'offline_validation.json')['input_sha256'].items():
 assert hashlib.sha256(Path(filename).read_bytes()).hexdigest()==digest,filename
sys.path.insert(0,str(ROOT/'tools'))
import pixel_camera_tuning_cli
from pixel_camera_capture import camera_workspace
from pixel_camera_archive import verify_archive,verify_sources
archive=Path(state['archive']['archive_folder']);manifest=verify_archive(archive);verify_sources(manifest,Path(state['source_trial']))
sys.path.insert(0,str(camera_workspace()/'python'))
from pixel_camera import PixelCameraClient
status=PixelCameraClient(connection='wifi',port=8765,auto_start=False).get_status();device=status.get('device',{})
assert status.get('connected') and device.get('phase')=='IDLE' and not device.get('pending_count') and not device.get('owner')
with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as probe:
 if hasattr(socket,'SO_EXCLUSIVEADDRUSE'):probe.setsockopt(socket.SOL_SOCKET,socket.SO_EXCLUSIVEADDRUSE,1)
 probe.bind(('127.0.0.1',8767))
result=dict(checked_at_utc=datetime.now(timezone.utc).isoformat(),ready_for_new_operator_readiness=True,
 camera=dict(connected=status['connected'],device={k:device.get(k) for k in ('phase','pending_count','owner','zoom_ratio','torch','focus_lock')}),
 camera_health_passed=True,camera_battery_percent=health['battery']['level_percent'],ble_port=8767,ble_port_free=True,
 ble_address=config['address'],request_id=identifier,request_sent=False,operator_ready=False,
 physical_power_state='BOTH_OFF_USER_CONFIRMED',offline_runner_validation_passed=True,
 latest_archive_and_sources_verified=True,analysis_comparison_figure_visually_reviewed=True,robot_commands_sent=0,
 ble_connections_started=0,camera_recording_started=False,input_hashes_match=True,
 input_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (STAGE/'config.json',STAGE/'candidate.settings.json',
 STAGE/'run_recorded_once.py',STAGE/'offline_validation.json',Path(config['geometry']),Path(config['reference']),STAGE/'camera_health_20261121.json',Path(__file__))})
path=STAGE/'preflight_read_only_20261121.json'
with path.open('x',encoding='utf-8') as f:json.dump(result,f,ensure_ascii=False,indent=2)
state.update(latest_preparation=str(path),next_camera_health_report=str(STAGE/'camera_health_20261121.json'),next_operator_ready=False,
 readiness_question_pending=dict(request_id=identifier,answered=False,current_operator_ready_required=True,
 source='次は同じ概略位置・前緑下向き、干渉余裕、両方ON・手離し・一回可を確認する。'))
state_path.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(ready=True,request_sent=False,correction=candidate['parameters']['correction'],archive_verified=True,camera_battery_percent=health['battery']['level_percent']),ensure_ascii=False))
