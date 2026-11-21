import json
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

base=Path('tools/logs/pixel_camera/turn1600_B_outr135_20261121_resume_v1')
stage=base/'geometry_c1p0_Ls5p3_Le31_03'
identifier='B_OUTR135_1600_c1p0_Ls5p3_Le31_green_20261121_003'
assert not (stage/'local_execution/jobs'/(identifier+'.json')).exists()
config=json.loads((stage/'config.json').read_text(encoding='utf-8-sig'))
assert config['mouse']=='B' and config['address']=='E1:7F:86:21:84:73'
evidence=dict(recorded_at=datetime.now(timezone.utc).isoformat(),user_reply='配置準備完了，ONです', previous_hands_clear_reply='両方ON・干渉なし・手離し済み',
    response_to='出口180/45mm・緑前右下45度、今回の干渉なし確認後に両方ON・手離し。',
    placement_confirmation_in_context=True,fresh_camera_placement_check_required=True,
    hands_clear_confirmed=True,new_request_id=identifier,automatic_replay=False,
    resource_check_at='2026-11-20T15:56:01.5680593+00:00',other_acquisition_processes=0,
    ble_port_8766_free=True,ble_address=config['address'])
with (base/'operator_ready_geometry003.json').open('x',encoding='utf-8') as stream:
    json.dump(evidence,stream,ensure_ascii=False,indent=2)
state_file=base/'resume_state.json'
state=json.loads(state_file.read_text(encoding='utf-8-sig'))
state.update(state='GEOMETRY5P3_31_C1P0_003_ACTIVE',operator_ready=True,
    next_request_sent=True,execution_pending=True,current_placement_confirmed=True,
    current_hands_clear_confirmed=True,power_state='BOTH_ON_USER_CONFIRMED',
    operator_ready_source='operator_ready_geometry003.json',
    next_action='新IDの結果だけ照会し再送しない。保存/失敗後は両方OFFを案内。')
state_file.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
raise SystemExit(subprocess.call([sys.executable,str(stage/'run_recorded_once.py'),'--operator-ready']))


