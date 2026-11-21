import json
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

base = Path('tools/logs/pixel_camera/turn1600_B_outr135_20261120_v1')
stage = base/'lengths_Ls6_Le27p9_c1p9_02'
identifier = 'B_OUTR135_1600_Ls6_Le27p9_c1p9_green_20261120_002'
evidence = dict(recorded_at=datetime.now(timezone.utc).isoformat(),
    user_reply='off2on完了，はいちそのまま',
    previous_reply='両方ON・手離し済み',
    placement_maintained=True, hands_clear_confirmed=True,
    previous_request='B_OUTR135_1600_Ls6_Le27p9_c1p9_green_20261120_001',
    previous_execution_write_attempted=False,
    new_request_id=identifier, automatic_replay=False)
with (base/'operator_ready_lengths002.json').open('x', encoding='utf-8') as stream:
    json.dump(evidence, stream, ensure_ascii=False, indent=2)
state_file = base/'resume_state.json'
state = json.loads(state_file.read_text(encoding='utf-8-sig'))
state.update(last_request_id=evidence['previous_request'],
    last_error='BLE disconnected during pre-run device check; no execution attempted',
    last_execution_write_attempted=False, execution_commands_sent=0,
    next_stage=str(stage.resolve()), next_request_id=identifier,
    next_request_sent=True, execution_pending=True,
    operator_ready_source='operator_ready_lengths002.json',
    power_state='BOTH_ON_USER_CONFIRMED', current_placement_confirmed=True,
    current_hands_clear_confirmed=True, state='LENGTHS6_27P9_002_ACTIVE',
    next_action='新IDの結果のみ確認。再送せず、終了後に両方OFFを依頼する。')
state_file.write_text(json.dumps(state, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
raise SystemExit(subprocess.call([sys.executable, str(stage/'run_recorded_once.py'), '--operator-ready']))
