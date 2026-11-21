"""停止応答取得済みの002から保持ログだけを1回読み直す。走行・set・presetは送らない。"""
from pathlib import Path
import json
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from pixel_camera_tuning import LiveSession, setting_differences
from pixel_camera_tuning_cli import TuningService, load_config
from pixel_camera_turn_capture import save_retained_log

STAGE = ROOT / 'tools/logs/pixel_camera/turn1800_A_inr135_20261120_v1/c2p4_Ls2p5_Le26_02'
RUN = STAGE / 'local_execution/run_A_INR135_1800_c2p4_Ls2p5_Le26_req_20261120_002'
OUT = STAGE / 'retained_log_recovery_v1'
read = lambda p: json.loads(p.read_text(encoding='utf-8-sig'))
trial = read(RUN / 'trial.json')
assert trial['log_stop_evidence']['confirmed'] and trial['camera_run_start_confirmed']
assert b'DEBUG_CAMERA_RUN_DONE' in (RUN / 'pc.shell.bin').read_bytes()
events = [json.loads(line) for line in (RUN / 'events.jsonl').read_text(encoding='utf-8').splitlines()]
assert any(item['event'] == 'saved_log_requested' for item in events)
settings = read(RUN / 'motion.settings.json')
config = load_config(STAGE / 'config.json')
OUT.mkdir(exist_ok=False)
session = LiveSession(ROOT, OUT / 'transport')
service = TuningService(OUT, config, session)
report = dict(state='READING_RETAINED_LOG', original_trial=str(RUN / 'trial.json'),
    original_trial_state=trial['state'], execution_commands_sent=0, execution_resend_count=0,
    stopped_response_source=str(RUN / 'pc.shell.bin'), new_run_started=False)


class PriorStopAcknowledgement:
    def __init__(self, shell, event):
        self.shell, self.event, self.first = shell, event, True

    def __getattr__(self, name):
        return getattr(self.shell, name)

    def wait_line(self, predicate, timeout=8):
        if self.first:
            self.first = False
            assert predicate('DEBUG_CAMERA_RUN_DONE')
            self.event('original_recorded_stop_ack_used', source=str(RUN / 'pc.shell.bin'))
            return 'DEBUG_CAMERA_RUN_DONE'
        return self.shell.wait_line(predicate, timeout=timeout)

    def send(self, command):
        assert command in ('log mode0', 'disp log_bin'), command
        return self.shell.send(command)


try:
    session.prepare(config['address'])
    with service.shell(OUT) as shell:
        current = service.read_settings(shell, 'in_r135')
        differences = setting_differences(current, settings)
        if differences:
            raise RuntimeError('Device readback changed: ' + ','.join(differences))
        capability = shell.query('debug camera show', lambda line: line.startswith('DEBUG_CAMERA_CAP '))
        event_list = []

        def event(name, **fields):
            event_list.append(dict(event=name, **fields))
            print(name, flush=True)

        result = save_retained_log(PriorStopAcknowledgement(shell, event), OUT, current,
            capability, 'in_r135', event, timeout=20)
        report.update(state='RETRIEVED_PENDING_MATCH_VALIDATION', saved_log=result, events=event_list)
except Exception as error:
    report.update(state='RECOVERY_FAILED', error=f'{type(error).__name__}: {error}')
finally:
    session.close()
    report['ble_closed'] = True
    with (OUT / 'recovery.json').open('x', encoding='utf-8') as file:
        json.dump(report, file, ensure_ascii=False, indent=2)
print(json.dumps(report, ensure_ascii=False), flush=True)
raise SystemExit(0 if report['state'] == 'RETRIEVED_PENDING_MATCH_VALIDATION' else 1)
