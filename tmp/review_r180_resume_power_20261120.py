"""R180再開時の未走行・低電圧と操作者OFF確認を保存する。実機接続なし。"""
from pathlib import Path
from datetime import datetime, timezone
import csv
import hashlib
import json
import statistics

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tools/logs/pixel_camera/turn1800_A_r180_20261120_v1'
STAGE = BASE / 'angle_c4p25_01'
SESSION = STAGE / 'local_execution'
IDENTIFIER = 'A_R180_1800_angle_c4p25_r48_Le31p5_20261120_001'


def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def save(path, value):
    with path.open('x', encoding='utf-8', newline='\n') as output:
        json.dump(value, output, ensure_ascii=False, indent=2, allow_nan=False)
        output.write('\n')


job_path = SESSION / 'jobs' / (IDENTIFIER + '.json')
job = read(job_path)
result = job['result']
assert job['state'] == 'DONE' and result['state'] == 'FAILED'
assert result['execution_write_attempted'] is False
assert result['capture_started'] is False and result['ble_disconnected'] is True
prepare = Path(result['preparation_folder'])
event_path = prepare / 'events.jsonl'
events = [json.loads(line) for line in event_path.read_text(encoding='utf-8').splitlines()]
commands = [item['command'] for item in events if item['event'] == 'shell_write_attempt']
assert commands == ['shell', 'info battery']
transport = SESSION / ('transport_' + IDENTIFIER)
csv_path = transport / 'telemetry.csv'
with csv_path.open(encoding='utf-8', newline='') as stream:
    rows = list(csv.DictReader(stream))
voltage = [float(row['Battery']) for row in rows]
assert rows and all(abs(float(row['ideal.velo'])) < .01 and
                    abs(float(row['ideal.rad_velo'])) < .01 for row in rows)
assert (prepare / 'shell.bin').stat().st_size == 0
report = dict(
    recorded_at_utc=datetime.now(timezone.utc).isoformat(), mouse='A', motion='long_r180',
    preset_mm_s=1800, request_id=IDENTIFIER, state='PRE_RUN_FAILED_WAITING_POWER_CHECK',
    execution_write_attempted=False, execution_commands_sent=0, settings_commands_sent=0,
    shell_commands_sent=commands, shell_response_bytes=0, ble_disconnected=True,
    operator_power_state='Both_OFF_confirmed',
    operator_confirmation='停止・両方OFFを確認', operator_contact_confirmation=None,
    static_check=result['static_check'],
    battery_telemetry_v=dict(samples=len(voltage), minimum=min(voltage),
        median=statistics.median(voltage), maximum=max(voltage)),
    battery_firmware_limit_v=7.0,
    cause='電圧約3.57VはAの2セル基準7.0V未満。Battery_LimiterVoltageの無限待機とシェル無応答が整合する。電池・配線の物理原因は未確認。',
    candidate_parameters=read(STAGE / 'candidate.settings.json')['parameters'],
    failed_id_must_not_be_reused=True,
    next_stage=str(BASE / 'angle_c4p25_02'),
    next_id='A_R180_1800_angle_c4p25_r48_Le31p5_20261120_002',
    next_operator_ready=False, next_execution_pending=False,
    resume_requirements=['機体電池の接続・充電状態を確認',
        '今回の配置・余裕・両方ON・手離しを新規確認',
        '新IDで静止照合・info battery・実機設定の読返し後に単体1回'],
    parameter_adoption_allowed=False, source_params_changed=False,
    source_sha256={str(path): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in [job_path, event_path, csv_path, ROOT / 'Peripheral/Src/battery.c',
                     STAGE / 'candidate.settings.json', STAGE / 'run_recorded_once.py']})
save(STAGE / 'resume_power_failure_review.json', report)
save(STAGE / 'operator_stop_resume_20261120.json', dict(
    recorded_at_utc=report['recorded_at_utc'], trial_id=IDENTIFIER,
    both_powered_off=True, stopped=True, operator_answer='停止・両方OFFを確認',
    execution_commands_sent=0, contact_detected=None))
phase = read(STAGE / 'phase.json')
phase.update(state=report['state'], execution_pending=False, operator_ready=False,
    request_consumed=True, execution_write_attempted=False, ble_disconnected=True,
    power_state='Both_OFF_confirmed',
    review=str(STAGE / 'resume_power_failure_review.json'),
    next_stage=report['next_stage'], next_request_id=report['next_id'])
(STAGE / 'phase.json').write_text(json.dumps(phase, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
print(json.dumps(dict(report=str(STAGE / 'resume_power_failure_review.json'),
    state=report['state'], battery=report['battery_telemetry_v'],
    execution_commands_sent=0), ensure_ascii=False))
