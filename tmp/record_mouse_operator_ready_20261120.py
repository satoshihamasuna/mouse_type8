"""今回の操作者確認を新規JSONへ保存するだけ。接続・撮影・走行は行わない。"""
from pathlib import Path
from datetime import datetime, timezone
import argparse
import json

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--stage', type=Path, required=True)
parser.add_argument('--trial-id', required=True)
parser.add_argument('--motion', required=True)
parser.add_argument('--answer', required=True)
parser.add_argument('--health', type=Path, required=True)
args = parser.parse_args()
stage = args.stage.resolve()
assert stage.is_relative_to(ROOT / 'tools/logs/pixel_camera')
assert args.trial_id and all(c.isalnum() or c in '_-' for c in args.trial_id)
settings = json.loads((stage / 'candidate.settings.json').read_text(encoding='utf-8'))
assert settings['mouse'] == 'A' and settings['motion_name'] == args.motion
assert settings['preset_speed_mm_s'] == 1800 and settings['turn_count'] == 1
health = json.loads(args.health.read_text(encoding='utf-8'))
now = datetime.now(timezone.utc)
age = (now - datetime.fromisoformat(health['checked_at'])).total_seconds()
assert health['passed'] is True and -5 <= age <= 1800, '新しい端末電池確認が必要です'
for path in [stage / 'local_execution/jobs' / (args.trial_id + '.json'),
             stage / 'local_execution' / (args.trial_id + '.stdout.txt')]:
    assert not path.exists(), '消費済み要求IDは使用しません'
record = stage / ('operator_ready_' + args.trial_id + '.json')
value = dict(trial_id=args.trial_id, confirmed_at_utc=now.isoformat(), operator_answer=args.answer,
             placement_confirmed=True, clearance_confirmed=True, both_powered_on=True,
             hands_clear=True, authorized_executions=1, camera_health_report=str(args.health.resolve()),
             hardware_commands_sent=0)
with record.open('x', encoding='utf-8') as output:
    json.dump(value, output, ensure_ascii=False, indent=2)
prep = stage.parent / 'preparation.json'
state = json.loads(prep.read_text(encoding='utf-8'))
state.update(operator_ready=True, execution_pending=True, next_request_id=args.trial_id,
             current_placement_confirmed=True, current_clearance_confirmed=True,
             current_hands_clear_confirmed=True, current_physical_clearance_verified=True,
             both_powered_off_confirmed=False,
             power_state='Both_ON_hands_clear_confirmed', readiness_record=str(record))
prep.write_text(json.dumps(state, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
print('Fresh one-run readiness saved; no connection or robot command sent.')
