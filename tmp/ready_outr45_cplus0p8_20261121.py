from datetime import datetime, timezone
from pathlib import Path
import json
import socket

ROOT=Path(__file__).resolve().parents[1]
STAGE=ROOT/'tools/logs/pixel_camera/turn1800_A_outr45_20261120_v1/joint_cplus0p8_Ls19p9_Le22p1_20261121_01'
ID='A_OUTR45_1800_cplus0p8_Ls19p9_Le22p1_20261121_001'
settings=json.loads((STAGE/'candidate.settings.json').read_text(encoding='utf-8'))
health=json.loads((STAGE/'camera_health_pre_run.json').read_text(encoding='utf-8'))
now=datetime.now(timezone.utc)
assert health['passed'] and (now-datetime.fromisoformat(health['checked_at'])).total_seconds()<300
assert settings['mouse']=='A' and settings['motion_name']=='out_r45' and settings['turn_count']==1
assert settings['parameters']['correction']==.8 and settings['parameters']['lstart']==19.9 and settings['parameters']['lend']==22.1
for path in (STAGE/'local_execution/jobs'/(ID+'.json'),STAGE/'local_execution'/(ID+'.stdout.txt')):
    assert not path.exists(),'A consumed request must not be replayed'
with socket.socket() as probe:
    probe.bind(('127.0.0.1',8766))
record=dict(confirmed_at_utc=now.isoformat(),request_id=ID,source='current_conversation_operator_replies',
    mouse='A',motion='out_r45',preset_mm_s=1800,correction_deg=.8,lstart_mm=19.9,lend_mm=22.1,
    placement_confirmed=True,clearance_confirmed=True,both_powered_on=True,hands_clear=True,
    operator_answers=['配置電源ON済み','手離し・走行範囲確認済み'],allowed_execution_count=1,
    automatic_execution_replay_allowed=False,ble_port_free_at_check=8766,camera_health_passed=True)
with (STAGE/('operator_ready_'+ID+'.json')).open('x',encoding='utf-8') as file:
    json.dump(record,file,ensure_ascii=False,indent=2);file.write('\n')
print('Current readiness saved; BLE port8766 free; no hardware commands sent.',flush=True)
