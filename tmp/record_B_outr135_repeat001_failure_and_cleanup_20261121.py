"""未走行の失敗と終了処理修正の根拠を保存し、未消費IDへ引き継ぐ。"""
from pathlib import Path
from datetime import datetime, timezone
import difflib
import hashlib
import json
import shutil
root=Path(__file__).resolve().parents[1]
base=root/'tools/logs/pixel_camera/turn1600_B_outr135_20261121_resume_v1'
stage=base/'repeat_c1p0_Ls5p3_Le31_01'
identifier='B_OUTR135_1600_c1p0_Ls5p3_Le31_repeat_green_20261121_001'
local=stage/'local_execution';fix=base/'ble_cleanup_fix_v1'
read=lambda path:json.loads(path.read_text(encoding='utf-8-sig'))
sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
def save(path,value):
    with path.open('x',encoding='utf-8') as stream:
        json.dump(value,stream,ensure_ascii=False,indent=2,allow_nan=False);stream.write('\n')
job=read(local/'jobs'/(identifier+'.json'))
events_path=local/('prepare_'+identifier)/'events.jsonl'
events=[json.loads(line) for line in events_path.read_text(encoding='utf-8').splitlines()]
commands=[event['command'] for event in events if event['event']=='shell_write_attempt']
assert not (local/('run_'+identifier)).exists()
assert not any(' exe' in command for command in commands)
assert any(' preset 1600' in command for command in commands)
assert commands[-1].startswith('debug straight set ')
assert job['request']['config']['address']=='E1:7F:86:21:84:73'
assert job['result']['state']=='FAILED' and job['result']['ble_disconnected']
patches=[]
for relative,before_name,after_name in [
    ('tools/ble/mouse_ble_gateway.py','mouse_ble_gateway.before.py','mouse_ble_gateway.after.py'),
    ('tools/tests/test_mouse_ble_transport.py','test_mouse_ble_transport.before.py','test_mouse_ble_transport.after.py'),
    ('tools/ble/README.md','README.before.md','README.after.md')]:
    current=root/relative
    shutil.copyfile(current,fix/after_name)
    patches.extend(difflib.unified_diff((fix/before_name).read_text(encoding='utf-8-sig').splitlines(keepends=True),
        current.read_text(encoding='utf-8-sig').splitlines(keepends=True),fromfile=relative+' (before)',tofile=relative+' (after)'))
with (fix/'our_change.patch').open('x',encoding='utf-8') as stream:stream.writelines(patches)
fix_record=dict(recorded_at=datetime.now(timezone.utc).isoformat(),
    reproduced_test='GatewayCleanupTests.test_cancelled_writer_keeps_original_ble_disconnect_and_releases_tasks',
    before_fix='本来のRuntimeError: BLE disconnectedの処理中にactive_writer.wait_closed()がCancelledErrorを送り、監視task停止に到達しなかった。模擬BLEで同じ失敗を再現。',
    change='閉鎖待ち内部のキャンセルだけを結果として受け取り、外部タスク中止と未知のエラーは伝播。閉鎖待ちが失敗しても監視taskをfinallyで停止。',
    targeted_tests=dict(command='python -m unittest discover -s tools/tests -p test_mouse_ble_transport.py -k GatewayCleanupTests -v',passed=3,failed=0,skipped=0),
    transport_tests=dict(command='python -m unittest discover -s tools/tests -p test_mouse_ble_transport.py -v',passed=41,failed=0,skipped=0),
    verification_scope='模擬BLE/localhost。実機の切断根本原因の同定/解消と同一視しない。',
    automatic_reconnect_added=False,UART_execution_replay_added=False,firmware_changed=False,
    sources={relative:sha(root/relative) for relative in ('tools/ble/mouse_ble_gateway.py','tools/tests/test_mouse_ble_transport.py','tools/ble/README.md')})
save(fix/'verification.json',fix_record)
next_stage=base/'repeat_c1p0_Ls5p3_Le31_02'
next_id='B_OUTR135_1600_c1p0_Ls5p3_Le31_repeat_green_20261121_002'
assert not list((next_stage/'local_execution/jobs').glob('*.json'))
assert read(next_stage/'candidate.settings.json')==read(stage/'candidate.settings.json')
save(base/'repeat001_ble_failure_review.json',dict(recorded_at=fix_record['recorded_at'],request_id=identifier,
    mouse='B',motion='out_r135',preset_mm_s=1600,state='FAILED_DURING_SETTINGS_BEFORE_EXECUTION',
    ble_address=job['request']['config']['address'],error=job['result']['error'],shell_commands_attempted=commands,
    execution_commands_sent=0,physical_run_count=0,parameter_preset_attempted=True,
    straight_profile_write_attempted=True,partial_UART_command_possible=True,configuration_fully_verified=False,
    power_state='BOTH_OFF_USER_CONFIRMED',operator_reply='両方OFF・配置を動かした',current_placement_confirmed=False,
    successful_same_candidate_runs=2,cleanup_fix_verification=str(fix/'verification.json'),
    source_at_failure=str(fix/'mouse_ble_gateway.before.py'),next_stage=str(next_stage),next_request_id=next_id,
    next_request_sent=False,automatic_replay=False))
phase=read(stage/'phase.json')
phase.update(state='FAILED_DURING_SETTINGS_BEFORE_EXECUTION',next_candidate_executed=False,execution_pending=False,
    physical_run_count=0,review=str(base/'repeat001_ble_failure_review.json'))
(stage/'phase.json').write_text(json.dumps(phase,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
state=read(base/'resume_state.json')
state.update(state='REPEAT001_BLE_FAILED_BEFORE_RUN_CLEANUP_FIXED_BOTH_OFF_NEXT002_READY',
    power_state='BOTH_OFF_USER_CONFIRMED',operator_power_off_confirmation='両方OFF・配置を動かした',
    operator_ready=False,current_placement_confirmed=False,current_hands_clear_confirmed=False,
    execution_pending=False,ble_connected=False,last_execution_write_attempted=False,execution_commands_sent=0,
    last_request_id=identifier,last_request_result='FAILED_DURING_SETTINGS_BEFORE_EXECUTION',last_error=job['result']['error'],
    last_attempt_review=str(base/'repeat001_ble_failure_review.json'),cleanup_fix_verification=str(fix/'verification.json'),
    successful_same_candidate_runs=2,next_stage=str(next_stage),next_request_id=next_id,next_request_sent=False,
    candidate=str(next_stage/'candidate.settings.json'),config=str(next_stage/'config.json'),
    next_action='操作者が配置を移動済み。出口180/45mm・緑前右下45度へ戻し、今回の干渉なし/両方ON/手離し確認後、競合を再確認して新IDで3本目を1回測る。旧失敗要求を再送しない。')
(base/'resume_state.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
with (base/'README.md').open('a',encoding='utf-8') as stream:
    stream.write('\n3本目として要求したrepeat_001はBLE切断で走行前に終了。preset1600と後走直進setを試行しており、後者は途中まで届いた可能性がある。走行指令0/実走0、操作者は両方OFF・配置移動を確認。次は配置を開始点へ戻し、RAM設定を再取得/復元/照合する。成功走行は2本のまま。切断後のwriter閉鎖futureのCancelledErrorが後始末を中断する問題を模擬BLEで再現し修正。外部タスク中止と未知の例外は伝播、監視task停止とBLE終了を継続する。追加3回帰を含むtransport41テスト成功/skip0。元のBLE切断原因は未特定、実機で安定性を確認したとはしない。次の未消費IDはrepeat_green_20261121_002。詳細repeat001_ble_failure_review.jsonとble_cleanup_fix_v1/verification.json。\n')
print(json.dumps(dict(execution_commands_sent=0,transport_tests_passed=41,next_request_id=next_id,
    current_placement_confirmed=False,next_request_sent=False),ensure_ascii=False))
