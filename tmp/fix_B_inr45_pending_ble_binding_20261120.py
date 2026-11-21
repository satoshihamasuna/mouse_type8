"""未送信のB反復008だけを新しい機体別BLE割当へ更新する。接続・走行はしない。"""
from pathlib import Path
from datetime import datetime,timezone
import hashlib
import json
import shutil

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr45_20261120_v1'
STAGE=BASE/'geometry02_repeat_02'
read=lambda p:json.loads(p.read_text(encoding='utf-8'))
def save(p,v):
    with p.open('x',encoding='utf-8') as f:json.dump(v,f,ensure_ascii=False,indent=2,allow_nan=False)
expected='E1:7F:86:21:84:73'
documentation=ROOT/'tools/ble/README.md'
assert '| B（新品） | `HY0020-MOUSE-B` | `'+expected+'` |' in documentation.read_text(encoding='utf-8')
state=read(BASE/'resume_state.json')
assert state['next_request_id']=='B_INR45_1600_geometry02_repeat02_green_20261120_008'
assert not state['next_request_sent']
assert not (STAGE/'local_execution/jobs'/f"{state['next_request_id']}.json").exists()
config=read(STAGE/'config.json');assert config['mouse']=='B'
before=config['address']
shutil.copyfile(STAGE/'config.json',STAGE/'config.before_ble_assignment_20261120.json')
config['address']=expected
(STAGE/'config.json').write_text(json.dumps(config,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
script=STAGE/'run_recorded_once.py'
shutil.copyfile(script,STAGE/'run_recorded_once.before_ble_assignment_20261120.py')
source=script.read_text(encoding='utf-8')
anchor='from pixel_camera_capture import record_trial\n'
assert source.count(anchor)==1
guard='''

# 2026-11-20の機体別割当。旧アドレスや別ポートへ自動接続しない。
EXPECTED_BLE_ADDRESS = 'E1:7F:86:21:84:73'
EXPECTED_BLE_PORT = 8766
config_binding = tuning.read_json(OUT/'config.json')
if config_binding.get('mouse') != 'B' or config_binding.get('address', '').upper() != EXPECTED_BLE_ADDRESS:
    raise RuntimeError('機体BのBLE割当と未送信試行の接続先が一致しません')

OriginalLiveSession = tuning.LiveSession


class MachineBBoundSession(OriginalLiveSession):
    def prepare(self, address, port=EXPECTED_BLE_PORT, progress=lambda message: None):
        if address.upper() != EXPECTED_BLE_ADDRESS or port != EXPECTED_BLE_PORT:
            raise RuntimeError('機体BはE1:7F:86:21:84:73 / BLE8766へ固定します')
        return super().prepare(address, port=EXPECTED_BLE_PORT, progress=progress)


tuning.LiveSession = MachineBBoundSession
'''
source=source.replace(anchor,anchor+guard)
script.write_text(source,encoding='utf-8')
binding=dict(mouse='B',advertised_name='HY0020-MOUSE-B',address=expected,ble_port=8766,camera_port=8765,
    previous_pending_address=before,updated_at_utc=datetime.now(timezone.utc).isoformat(),
    source='tools/ble/README.md: 機体A/BのBLE接続先。ユーザーがBLE割当の注意を指定。',
    source_sha256=hashlib.sha256(documentation.read_bytes()).hexdigest(),
    pending_request_id=state['next_request_id'],request_sent=False,
    historical_configs_rewritten=False,hardware_connections_started=0,hardware_commands_sent=0)
save(STAGE/'ble_binding.json',binding)
state.update(ble_binding=binding,ble_assignment_checked=True,ble_binding_readback_pending=True,
    next_request_sent=False,next_operator_ready=False,execution_pending=False)
for item in state.get('repeat_stages',[]):
    if item['request_id'].endswith('_007'):item['request_sent']=True
(BASE/'resume_state.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
with (BASE/'README.md').open('a',encoding='utf-8') as f:
    f.write('\n2026-11-20の機体別BLE割当を再確認。AはC1:B5:60:87:91:73、BはE1:7F:86:21:84:73（HY0020-MOUSE-B）。'
        '未送信の反復008のconfigと起動処理をB新アドレス/BLE8766へ固定し、旧設定と起動処理を別名で保持。'
        '取得済みログ・試行のアドレスは取得時の記録として変更しない。カメラ8765、Bの接続・実機値照合は再開時に行う。'
        'この更新では接続・設定送信・走行は0件。充電完了・現在の設置確認は引き続き未確認。\n')
print(json.dumps(binding,ensure_ascii=False,indent=2))
