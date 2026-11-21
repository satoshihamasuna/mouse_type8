"""走行未送信の接続停止と操作者の両方OFFを保存。機器操作なし。"""
from pathlib import Path
from datetime import datetime, timezone
import argparse
import json

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tools/logs/pixel_camera/turn1800_A_r180_20261120_v1'
p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--stage', type=Path, required=True)
p.add_argument('--trial-id', required=True)
p.add_argument('--next-stage', type=Path, required=True)
a = p.parse_args()
stage, next_stage = a.stage.resolve(), a.next_stage.resolve()
assert all(x.is_relative_to(BASE) for x in (stage, next_stage))
read = lambda x: json.loads(x.read_text(encoding='utf-8-sig'))
job = read(stage / 'local_execution/jobs' / (a.trial_id + '.json'))
r = job['result']
assert r['state'] == 'FAILED' and not r['execution_write_attempted']
assert not r['capture_started'] and r['ble_disconnected']
assert read(stage / 'candidate.settings.json') == read(next_stage / 'candidate.settings.json')
record = dict(recorded_at_utc=datetime.now(timezone.utc).isoformat(),
    state='PREFLIGHT_FAILED_NO_EXECUTION_BOTH_OFF_CONFIRMED',
    operator_answer='停止・両方OFFを確認', both_powered_off=True,
    capture_started=False, execution_write_attempted=False, physical_executions=0,
    execution_resend_count=0, ble_disconnected=True, error=r['error'],
    cause_identified=False, settings_changed=False, next_stage=str(next_stage),
    next_operator_ready=False, next_execution_pending=False)
with (stage / 'preflight_failure_off_confirmation.json').open('x', encoding='utf-8') as f:
    json.dump(record, f, ensure_ascii=False, indent=2)
for path in (stage / 'phase.json', BASE / 'preparation.json'):
    state = read(path)
    state.update(record)
    state.update(operator_ready=False, execution_pending=False,
        power_state='Both_OFF_confirmed', current_placement_confirmed=False,
        current_clearance_confirmed=False, current_hands_clear_confirmed=False,
        current_physical_clearance_verified=False, next_candidate_executed=False,
        candidate=str(next_stage / 'candidate.settings.json'),
        next_candidate=str(next_stage / 'candidate.settings.json'),
        config=str(next_stage / 'config.json'),
        next_request_id=read(next_stage / 'phase.json')['request_id'])
    path.write_text(json.dumps(state, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
print('Preflight failure and both-OFF confirmation saved; no hardware action.')
