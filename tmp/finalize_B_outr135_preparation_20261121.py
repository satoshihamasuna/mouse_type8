from pathlib import Path
from datetime import datetime, timezone
import json
root=Path(__file__).resolve().parents[1]
base=root/'tools/logs/pixel_camera/turn1600_B_outr135_20261121_resume_v1'
read=lambda path:json.loads(path.read_text(encoding='utf-8-sig'))
resources=read(base/'resource_snapshot.json')
camera=read(base/'camera_status.json');health=read(base/'camera_health.json')
assert not resources['active_processes']
assert not any(port['LocalPort']==8766 for port in resources['listening_ports'])
assert camera['status']['connected'] and camera['status']['device']['phase']=='IDLE'
assert health['passed'] and health['read_only'] and not health['recording_started']
record=dict(checked_at=datetime.now(timezone.utc).isoformat(),state='PREPARED_AWAITING_FRESH_OPERATOR_READINESS',
    mouse='B',motion='out_r135',preset_mm_s=1600,ble_address='E1:7F:86:21:84:73',
    parameters=dict(velo=1.6,r_min=-41.,lstart=5.3,lend=31.,degree=-135.,correction=1.,pre_accel=6.5),
    resource_snapshot=str(base/'resource_snapshot.json'),ble_port_free_at_check=True,
    other_capture_processes_at_check=0,resource_reservation_created=False,
    camera_status=str(base/'camera_status.json'),camera_connected=True,camera_phase='IDLE',
    camera_health=str(base/'camera_health.json'),camera_battery_percent=health['battery']['level_percent'],
    camera_temperature_c=health['battery']['temperature_c'],camera_charging=health['battery']['status_name'],
    input_validation_passed=True,robot_connections_attempted=0,robot_commands_sent=0,
    robot_voltage_verified=False,current_placement_confirmed=False,current_hands_clear_confirmed=False,
    next_step='今回の出口配置/穴・全接地点・車幅・旋回/停止余裕・両方ON・手離しを確認後、競合を再確認して新IDの1回測定へ進む。')
with (base/'preflight_review.json').open('x',encoding='utf-8') as stream:
    json.dump(record,stream,ensure_ascii=False,indent=2,allow_nan=False)
state=read(base/'resume_state.json')
state.update(state=record['state'],preflight_review=str(base/'preflight_review.json'),
    camera_health_passed=True,other_resource_use_at_last_check=False,robot_commands_sent=0,
    next_action='今回の出口180/45mm・緑前右下45度、干渉なし/両方ON/手離し確認後、競合を再確認して新ID20261121_001を1回実行。')
(base/'resume_state.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
with (base/'README.md').open('a',encoding='utf-8') as stream:
    stream.write('\n最終準備確認：別の取得処理は終了し8766番ポートの解放を確認。既存カメラはconnected/IDLE、電池96%・USB充電中・29.3℃、読み取り専用の確認が成功。新規撮影・カメラ設定変更・機体接続/設定/走行は0件。各1回の3試行を検証し、初回wrapperの通常起動もVALIDATED。次は今回の配置/干渉なし/ON/手離しの確認待ち。ポートを予約しているわけではないため、実行直前にも競合を再確認する。詳細preflight_review.json。\n')
print(json.dumps(dict(state=record['state'],B_address=record['ble_address'],camera_phase=record['camera_phase'],camera_battery_percent=record['camera_battery_percent'],robot_commands_sent=0),ensure_ascii=False))
