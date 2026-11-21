"""操作者の今回のON・経路・手離し確認だけを保存する。機器操作なし。"""
from pathlib import Path
from datetime import datetime,timezone
import json

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_r90_20261120_v1'
ready=dict(at=datetime.now(timezone.utc).isoformat(),mouse='B',
    request_id='B_R90_1600_Lstart13p6_corr1p0_green_20261120_002',
    user_reply='両方ON・干渉なし・手を離した',both_on=True,hands_clear=True,
    route_clearance_confirmed=True,uart_wiring_confirmed=True,
    placement_mode='relative_to_measured_motion_entry_direction',exact_start_target_required=False,
    authorized_execution_count=1,execution_resend_count=0)
with (BASE/'operator_ready_green_corr1p0_relative02.json').open('x',encoding='utf-8') as output:
    json.dump(ready,output,ensure_ascii=False,indent=2)
path=BASE/'resume_state.json'
state=json.loads(path.read_text(encoding='utf-8'))
state.update(state='CURRENT_ON_READY_PREPARE_AND_ONE_CAPTURE',power_state='BOTH_ON_USER_CONFIRMED',
    operator_ready_source=str(BASE/'operator_ready_green_corr1p0_relative02.json'),
    current_placement_confirmed=True,current_hands_clear_confirmed=True,
    next_action='Restore verified settings and execute current unique request once; no resend.',
    execution_pending=False,next_request_sent=False)
path.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
