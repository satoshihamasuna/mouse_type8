"""実機照会中のBLE切断を記録。未送信の新IDを準備し、実機へ接続しない。"""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json
root=Path(__file__).resolve().parents[1]
base=root/'tools/logs/pixel_camera/turn1600_B_outr135_20261121_resume_v1'
stage=base/'geometry_c1p0_Ls5p3_Le31_02'
identifier='B_OUTR135_1600_c1p0_Ls5p3_Le31_green_20261121_002'
local=stage/'local_execution'
read=lambda path:json.loads(path.read_text(encoding='utf-8-sig'))
job=read(local/'jobs'/(identifier+'.json'))
events_path=local/('prepare_'+identifier)/'events.jsonl'
events=[json.loads(line) for line in events_path.read_text(encoding='utf-8').splitlines()]
commands=[event['command'] for event in events if event['event']=='shell_write_attempt']
assert commands==['shell','info battery','debug camera show','wall_ekf show','wall_edge show','turn_ekf show']
assert not (local/('run_'+identifier)).exists()
assert job['result']['state']=='FAILED' and job['result']['ble_disconnected']
assert job['request']['config']['address']=='E1:7F:86:21:84:73'
next_stage=base/'geometry_c1p0_Ls5p3_Le31_03'
next_id='B_OUTR135_1600_c1p0_Ls5p3_Le31_green_20261121_003'
assert not list((next_stage/'local_execution/jobs').glob('*.json'))
assert read(stage/'candidate.settings.json')==read(next_stage/'candidate.settings.json')
assert read(next_stage/'config.json')['address']=='E1:7F:86:21:84:73'
static=job['result']['static_check']
sources=[events_path,local/'jobs'/(identifier+'.json'),local/('transport_'+identifier)/'gateway.stderr.txt']
record=dict(recorded_at=datetime.now(timezone.utc).isoformat(),request_id=identifier,mouse='B',motion='out_r135',
    preset_mm_s=1600,state='FAILED_BEFORE_PARAMETER_CHANGE_AND_EXECUTION',ble_address='E1:7F:86:21:84:73',
    error=job['result']['error'],gateway_disconnected_callback_observed=True,automatic_replay=False,
    shell_commands_attempted=commands,parameter_write_commands=0,execution_write_commands=0,physical_run_count=0,
    static_check_passed=static['passed'],static_center_board_mm=static['center_board_mm'],
    static_heading_deg=static['heading_deg'],power_state='BOTH_OFF_USER_CONFIRMED',
    operator_reply='両方OFF・出口の配置そのまま',placement_maintained=True,
    source_sha256={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in sources},
    cause='BLEリンク切断を確認。表示ファイル置換失敗とは別。無線/機器/OSの根本原因は今回の記録だけで特定しない。',
    next_stage=str(next_stage),next_request_id=next_id,next_request_sent=False,
    successful_same_candidate_runs=1,spare_stages=[str(base/f'repeat_c1p0_Ls5p3_Le31_{number:02d}') for number in (1,2)])
with (base/'geometry002_ble_failure_review.json').open('x',encoding='utf-8') as stream:
    json.dump(record,stream,ensure_ascii=False,indent=2)
phase=read(stage/'phase.json')
phase.update(state=record['state'],request_id=identifier,next_candidate_executed=False,
    execution_pending=False,physical_run_count=0,review=str(base/'geometry002_ble_failure_review.json'))
(stage/'phase.json').write_text(json.dumps(phase,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
state=read(base/'resume_state.json')
state.update(state='GEOMETRY002_BLE_FAILED_BEFORE_RUN_BOTH_OFF_NEXT003_READY',power_state='BOTH_OFF_USER_CONFIRMED',
    operator_ready=False,current_placement_confirmed=True,current_hands_clear_confirmed=False,
    operator_power_off_confirmation='両方OFF・出口の配置そのまま',ble_connected=False,execution_pending=False,
    last_request_id=identifier,last_request_result='FAILED_BEFORE_PARAMETER_CHANGE_AND_EXECUTION',
    last_execution_write_attempted=False,execution_commands_sent=0,last_error=job['result']['error'],
    last_attempt_review=str(base/'geometry002_ble_failure_review.json'),successful_same_candidate_runs=1,
    next_stage=str(next_stage),next_request_id=next_id,next_request_sent=False,
    candidate=str(next_stage/'candidate.settings.json'),config=str(next_stage/'config.json'),
    next_action='出口配置を維持し、今回の両方ON/干渉なし/手離し確認後、競合を再確認して新ID003で同一候補の2本目を1回測る。消費済みID002は再送しない。')
(base/'resume_state.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
with (base/'README.md').open('a',encoding='utf-8') as stream:
    stream.write('\n同候補の要求002は開始配置5/5観測・ArUco登録を照合後、B用BLEで実機モード照会中に切断。shell/info battery/debug camera show/wall_ekf show/wall_edge show/turn_ekf showだけを試行し、設定変更0・走行指令0・実走0。gateway切断callbackとWinError10053を保存。原因を表示ファイル置換失敗と同一視しない。操作者が両方OFF・出口配置保持を確認。成功走行数は1本のまま。未送信の新ID003へ進む準備と、同条件の追加反復stage2件をオフライン準備。新しいON/手離し確認前に接続しない。詳細geometry002_ble_failure_review.json。\n')
print(json.dumps(dict(state=record['state'],execution_write_commands=0,next_request_id=next_id),ensure_ascii=False))
