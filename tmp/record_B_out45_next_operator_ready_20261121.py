"""現在の返答を、指定された未使用IDの一回限りの準備確認として記録する。"""
from pathlib import Path
from datetime import datetime,timezone
import hashlib
import json
import sys
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_outr45_20261121_v1'
read=lambda p:json.loads(Path(p).read_text(encoding='utf-8-sig'))
state_path=BASE/'resume_state.json';state=read(state_path);identifier=sys.argv[1];STAGE=Path(state['next_stage'])
assert STAGE.is_relative_to(BASE) and state['next_request_id']==identifier and not state['next_request_sent']
assert state['readiness_question_pending']['request_id']==identifier and not state['readiness_question_pending']['answered']
assert not (STAGE/'local_execution/jobs'/f'{identifier}.json').exists() and not (STAGE/'local_execution'/f'{identifier}.stdout.txt').exists()
for filename,digest in read(STAGE/'offline_validation.json')['input_sha256'].items():
 assert hashlib.sha256(Path(filename).read_bytes()).hexdigest()==digest,filename
config=read(STAGE/'config.json');candidate=read(STAGE/'candidate.settings.json')
assert candidate['mouse']==config['mouse']=='B' and config['address']=='E1:7F:86:21:84:73' and config['ble_port']==8767
assert candidate['motion_name']=='out_r45' and candidate['preset_speed_mm_s']==1600 and candidate['turn_count']==1
record=dict(confirmed_at_utc=datetime.now(timezone.utc).isoformat(),request_id=identifier,
 source='Current reply to explicit request for original coarse placement, clearance, both ON, hands clear and one run',
 user_reply=sys.argv[2],operator_ready=True,both_powered_on=True,
 hands_clear_and_one_run_confirmed_in_readiness_context=True,allowed_runs=1,request_sent=False,
 settings_json=str(STAGE/'candidate.settings.json'),ble_address=config['address'],ble_port=8767)
path=STAGE/f'operator_ready_{identifier}.json'
with path.open('x',encoding='utf-8') as f:json.dump(record,f,ensure_ascii=False,indent=2)
state.update(state='OPERATOR_READY_SINGLE_CAPTURE_PENDING',operator_ready=True,next_operator_ready=True,
 physical_power_state='BOTH_ON_USER_READY_CONFIRMED',current_physical_power_state_unconfirmed=False,operator_record=str(path),
 next_action=f'{identifier}を一回だけ撮影取得。送信後は再送しない。取得後は両方OFFを案内し保存データを解析する。')
state['readiness_question_pending'].update(answered=True,user_reply=record['user_reply'],answered_at_utc=record['confirmed_at_utc'])
state_path.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print('Current readiness recorded for '+identifier+'; no hardware command sent')
