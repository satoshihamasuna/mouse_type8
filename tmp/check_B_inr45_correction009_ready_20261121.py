"""009は未送信のまま、カメラ状態とBポート空きを読む。BLE接続なし。"""
from pathlib import Path
from datetime import datetime,timezone
import hashlib
import json
import socket
import sys
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr45_20261120_v1'
STAGE=BASE/'geometry03_correction_m0p4';read=lambda p:json.loads(Path(p).read_text(encoding='utf-8'))
sys.path.insert(0,str(ROOT/'tools'))
import pixel_camera_tuning_cli
from pixel_camera_capture import camera_workspace
sys.path.insert(0,str(camera_workspace()/'python'))
from pixel_camera import PixelCameraClient
health=read(STAGE/'camera_health_20261121.json');assert health['passed']
status=PixelCameraClient(connection='wifi',port=8765,auto_start=False).get_status()
device=status.get('device',{});filtered=dict(connected=status.get('connected'),device_id=status.get('device_id'),
 device={key:device.get(key) for key in ('phase','pending_count','owner','recording_id','exposure_compensation_steps','zoom_ratio','torch','focus_lock')})
free=True;error=None
try:
 with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as probe:
  if hasattr(socket,'SO_EXCLUSIVEADDRUSE'):probe.setsockopt(socket.SOL_SOCKET,socket.SO_EXCLUSIVEADDRUSE,1)
  probe.bind(('127.0.0.1',8767))
except OSError as issue:free=False;error=str(issue)
config=read(STAGE/'config.json');candidate=read(STAGE/'candidate.settings.json');geometry=read(config['geometry']);reference=read(config['reference'])
assert config['mouse']==candidate['mouse']==geometry['mouse']=='B' and candidate['motion_name']=='in_r45' and candidate['turn_count']==1
assert config['address']=='E1:7F:86:21:84:73' and config['ble_port']==8767 and geometry['reference_id']==reference['reference_id']
assert geometry['front_colour']=='green' and geometry['rear_colour']=='red' and candidate['parameters']['correction']==-.4
assert geometry['analysis_profile_adoption_state']=='VALIDATED_OFFLINE_FOR_THIS_B_CAPTURE_CONDITION'
identifier='B_INR45_1600_correction_m0p4_green_20261121_009'
assert not (STAGE/'local_execution/jobs'/f'{identifier}.json').exists() and not (STAGE/'local_execution'/f'{identifier}.stdout.txt').exists()
ready=bool(status.get('connected') and device.get('phase')=='IDLE' and not device.get('pending_count') and not device.get('owner') and free)
result=dict(checked_at_utc=datetime.now(timezone.utc).isoformat(),ready_for_new_operator_readiness=ready,camera=filtered,
 camera_health_passed=health['passed'],ble_port=8767,ble_port_free=free,ble_port_error=error,ble_address=config['address'],
 request_id=identifier,request_sent=False,operator_ready=False,current_power_state='BOTH_OFF_USER_CONFIRMED',
 offline_runner_validation_passed=True,analysis_figure_visually_reviewed=True,new_detector_actual_frame_audit_visually_reviewed=True,
 ble_connections_started=0,robot_commands_sent=0,camera_configure_sent=False,camera_recording_started=False,
 input_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (STAGE/'config.json',STAGE/'candidate.settings.json',
 STAGE/'run_recorded_once.py',Path(config['geometry']),Path(config['reference']),STAGE/'camera_health_20261121.json',Path(__file__))})
with (STAGE/'preflight_read_only_20261121.json').open('x',encoding='utf-8') as f:json.dump(result,f,ensure_ascii=False,indent=2)
assert ready,result
state_path=BASE/'resume_state.json';state=read(state_path);assert state['next_request_id']==identifier and not state['next_request_sent']
state.update(next_camera_health_report=str(STAGE/'camera_health_20261121.json'),latest_preparation=str(STAGE/'preflight_read_only_20261121.json'),next_operator_ready=False)
state_path.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
(STAGE/'README.md').write_text('# 機体B1600 右in45：009の準備\n\n角度補正−1.5→−0.4度だけを比較。Lstart7.8/Lend34.5mm・速度1.6m/s・半径−55mm・PID/FF/吸引650は固定。新しい実行IDで一回のみ取得する。旧008は実行済みで再送しない。\n\n'
 '根拠と赤色域の実映像検証は ../red_hue160_validation_v1/README.md。新しいB専用の検証済み解析プロファイルを使用し、95%・実ArUco・補間上限を保つ。\n\n'
 'BLE E1:7F:86:21:84:73 / 8767、カメラ8765。使用中ポートへの接続は起動時に拒否。通常起動は設定検証のみで接続・set/exeなし。カメラ状態照会はIDLE、残量95%・USB給電中、Bポート空きを確認。\n\n'
 '開始位置は同じ中点225/45mm・前緑下向きへ概略配置。現在の穴・全接地点・幅・停止余裕を操作者が確認し、新規の両方ON/手離し/一回可の返答後に実行する。ON後は保存候補を復元して実機値を照合。終了後は両方OFFで解析する。\n\n'
 '本準備ではBLE接続・実機設定・走行・ファーム書込み0回。ホスト解析とオフライン起動検証を実施し、ファーム/Paramsは変更していない。実走確認は未実施。\n',encoding='utf-8')
print(json.dumps(dict(ready=ready,camera_idle=True,camera_battery_percent=health['battery']['level_percent'],ble_port_free=free,request_sent=False),ensure_ascii=False))
