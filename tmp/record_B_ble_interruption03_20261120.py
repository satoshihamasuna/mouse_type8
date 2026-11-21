"""設定中のBLE切断と、停止状態で行う次の通信診断計画を保存する。"""
from pathlib import Path
import hashlib
import json

import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT/'tools/logs/pixel_camera/turn1600_B_r90_20261120_v1'
PREP = BASE/'device_prepare_green_Lstart13p6_03'


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def write_new(path, value):
    with path.open('x',encoding='utf-8') as output:
        json.dump(value,output,ensure_ascii=False,indent=2)


result = read(PREP/'result.json')
assert result['state']=='PREPARE_FAILED' and result['execution_commands_sent']==0
assert result['ble_disconnected'] and not (BASE/'cli_session06').exists()
events = [json.loads(line) for line in (PREP/'events.jsonl').read_text(encoding='utf-8').splitlines()]
assert not any('exe' in row.get('command','') or 'debug camera arm' in row.get('command','') for row in events)
data = pd.read_csv(PREP/'transport/telemetry.csv')
moving = (data['ideal.velo'].abs()>.01) | (data['ideal.rad_velo'].abs()>.01)
assert not moving.any()
health = read(PREP/'transport/telemetry.status.json')
diagnosis = dict(mouse='B', motion='long_r90', state='PREPARE_FAILED_NO_EXECUTION',
    reserved_request_id='B_R90_1600_Lstart13p6_green_20261120_003',
    capture_request_submitted=False, execution_commands_sent=0, execution_resend_count=0,
    ram_turn_set_write_attempted=True, ram_turn_set_readback_completed=False,
    RAM_state_must_be_restored_and_queried_next_time=True,
    saved_settings_to_restore=str(BASE/'device_prepare_green_Lstart13p6_01/actual.settings.json'),
    gateway_error='RuntimeError: BLE disconnected; in-flight data will not be replayed',
    shell_error=result['error'], low_voltage_rejection=False,
    stream_quality=health, minimum_logged_battery_v=float(data.Battery.min()),
    last_logged_battery_v=float(data.Battery.iloc[-1]), commanded_motion_rows=0,
    root_cause_identified=False,
    PC_inspection=dict(related_python_processes_after_close=0, tuning_ports_listening_after_close=0,
        bluetooth_adapter_status='OK', matching_windows_Bluetooth_fault_events_found=False,
        inspection_scope='現在状態と切断時刻のSystemログ。過去の他クライアント接続やRF/電源の瞬断を否定する証拠ではない。'),
    operator_stopped_both_off=True, source_params_changed=False, firmware_write_performed=False,
    next_action='OFFで配線の緩み・他端末のHY0020接続を確認し、停止状態で既存BLE段階測定を行う。通信確認中は走行しない。',
    input_sha256={str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest() for path in
        [PREP/'result.json', PREP/'events.jsonl', PREP/'transport/gateway.stderr.txt', PREP/'transport/telemetry.csv']})
write_new(PREP/'unexecuted_prepare_review.json',diagnosis)
benchmark = ROOT/'tools/ble/mouse_ble_benchmark.py'
plan = dict(mouse='B', purpose='STATIONARY_LINK_DIAGNOSIS_ONLY',
    periods_ms=[20,5], seconds_per_period=45, expected_measurement_seconds=90,
    enter_shell=True, execution_commands_sent=0, reconnect_automatically=False,
    tool=str(benchmark), tool_sha256=hashlib.sha256(benchmark.read_bytes()).hexdigest(),
    arguments=['--address','C1:B5:60:87:91:73','--mouse','B','--enter-shell',
        '--periods','20,5','--duration','45','--output-root',str(BASE/'ble_link_diagnosis')],
    before_power_on='OFFで配線と他クライアント接続を確認。診断準備後に操作者の新規ON・停止・手離し返答を待つ。',
    after_test='結果保存・BLE切断後は両方OFF。通信診断の成功をターンの追加走行許可として扱わない。')
write_new(BASE/'ble_link_diagnosis_plan_v1.json',plan)
state = read(BASE/'resume_state.json')
state.update(state='BOTH_OFF_AFTER_SECOND_BLE_DROP_WAIT_WIRING_CHECK',
    power_state='BOTH_OFF_USER_CONFIRMED', robot_connected=False,
    execution_pending=False, pending_request_id=None, pending_request_sent=False,
    current_hands_clear_confirmed=False, current_placement_confirmed=True,
    last_attempt_execution_sent=False, last_attempt_quality_review=str(PREP/'unexecuted_prepare_review.json'),
    last_reserved_request_id='B_R90_1600_Lstart13p6_green_20261120_003',
    last_reserved_request_submitted=False, next_unsent_request_id='B_R90_1600_Lstart13p6_green_20261120_004',
    next_request_sent=False, same_setting_repeat_count=1,
    next_action='配線の緩みと他PC・スマホのHY0020接続について返答を待つ。次は20/5msの停止通信診断のみを準備済み。新規ON確認まで接続・set・exeを送らない。',
    next_diagnostic_plan=str(BASE/'ble_link_diagnosis_plan_v1.json'),
    operator_power_confirmation='2026-11-20 user confirmed stopped and both OFF after prepare003 failure')
(BASE/'resume_state.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(saved=True, actual_additional_runs=0, firmware_changed=False,
    both_off=True, next_diagnostic_prepared=True, root_cause_identified=False)))
