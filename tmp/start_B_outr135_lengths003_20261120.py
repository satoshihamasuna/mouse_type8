import json
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

base = Path('tools/logs/pixel_camera/turn1600_B_outr135_20261120_v1')
stage = base/'lengths_Ls6_Le27p9_c1p9_03'
identifier = 'B_OUTR135_1600_Ls6_Le27p9_c1p9_green_20261120_003'
assert not (stage/'local_execution/jobs'/(identifier+'.json')).exists()
evidence = dict(recorded_at=datetime.now(timezone.utc).isoformat(),
    user_reply='両方ON・手離し済み',
    response_to='同じ右135°出口候補の2本目。両方OFFで中点(180,45)mm・緑前右下45°へ戻し、干渉なしを確認後、両方ON・手離し。',
    placement_confirmation_in_context=True, fresh_camera_placement_check_required=True,
    hands_clear_confirmed=True, new_request_id=identifier, automatic_replay=False)
with (base/'operator_ready_lengths003.json').open('x', encoding='utf-8') as stream:
    json.dump(evidence, stream, ensure_ascii=False, indent=2)
state_file = base/'resume_state.json'
state = json.loads(state_file.read_text(encoding='utf-8-sig'))
state.update(next_stage=str(stage.resolve()), next_request_id=identifier,
    next_request_sent=True, execution_pending=True, operator_ready=True,
    operator_ready_source='operator_ready_lengths003.json',
    power_state='BOTH_ON_USER_CONFIRMED', current_placement_confirmed=True,
    current_hands_clear_confirmed=True, state='LENGTHS6_27P9_003_ACTIVE',
    next_action='新IDの結果のみ確認。再送せず、終了後に両方OFFを依頼する。')
state_file.write_text(json.dumps(state, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
raise SystemExit(subprocess.call([sys.executable, str(stage/'run_recorded_once.py'), '--operator-ready']))
