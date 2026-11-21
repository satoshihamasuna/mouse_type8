"""未実行のB右V90設定だけを新しいB用HY0020へ切り替える。実機操作なし。"""
from pathlib import Path
from datetime import datetime, timezone
import copy
import hashlib
import json
import sys

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT/'tools/logs/pixel_camera/turn1600_B_rv90_20261120_v1'
OLD_ADDRESS = 'C1:B5:60:87:91:73'
NEW_ADDRESS = 'E1:7F:86:21:84:73'
sys.path.insert(0,str(ROOT/'tools'))
from pixel_camera_tuning_mcp import McpTuningService


def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def save(path,value):
    with path.open('x',encoding='utf-8') as stream:
        json.dump(value,stream,ensure_ascii=False,indent=2,allow_nan=False)
        stream.write('\n')


changes=[]
for name in ('geometry_ff0p0006_Ls3p3_Le18p9_c1p3_01',
             'geometry_ff0p0006_Ls3p3_Le18p9_c1p3_02'):
    stage=BASE/name
    assert not list((stage/'local_execution/jobs').glob('*.json')), '実行要求済みの記録は変更しない'
    path=stage/'config.json'
    before=read(path)
    assert before['mouse']=='B' and before['address']==OLD_ADDRESS
    save(stage/'config.before_B_ble_address_revision.json',before)
    after=copy.deepcopy(before)
    after['address']=NEW_ADDRESS
    assert {key:value for key,value in before.items() if key!='address'}=={key:value for key,value in after.items() if key!='address'}
    settings_before=(stage/'candidate.settings.json').read_bytes()
    path.write_text(json.dumps(after,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    plan=read(stage/'comparison_plan.json')
    payload=McpTuningService(stage/'offline_validation_B_ble_address',{}).prepare(
        str(path),str(stage/'candidate.settings.json'),plan['placement'],25.,None)
    assert payload['config']['address']==NEW_ADDRESS
    assert (stage/'candidate.settings.json').read_bytes()==settings_before
    save(stage/'offline_validation.B_ble_address.json',dict(
        state='VALIDATED_NOT_EXECUTED',payload=payload,hardware_operations=0,
        target_advertising_name='HY0020-MOUSE-B',supersedes='offline_validation.json'))
    changes.append(dict(config=str(path),before_address=OLD_ADDRESS,after_address=NEW_ADDRESS,
        candidate_sha256=hashlib.sha256(settings_before).hexdigest(),settings_unchanged=True,
        commands_sent=0))

revision=dict(recorded_at=datetime.now(timezone.utc).isoformat(),
    reason='ユーザーからBLEアドレスの確認指定。現在の機体別割り当てへ未実行設定を切り替える。',
    source_documents=['tools/ble/README.md','firmware/hy0020/README.md'],
    observed_scan_command='tools/ble/ble_monitor.py --scan --timeout 5',
    observed_advertisements=[dict(address=NEW_ADDRESS,name='HY0020-MOUSE-B'),
                             dict(address=OLD_ADDRESS,name='HY0020-MOUSE-A')],
    scan_was_read_only=True,scan_connected=False,robot_commands_sent=0,
    changes=changes,historical_trial_configs_unchanged=True,
    historical_note='取得済み5本の接続先は旧アドレスとして保持。今回の機体別割り当てを過去の接続ログへ書き戻さない。取得時の機体は操作者確認・動画・実機設定を参照する。',
    operator_ready=False,physical_power_state='CURRENT_CONFIRMATION_REQUIRED')
save(BASE/'B_ble_address_revision_20261120.json',revision)

state=read(BASE/'resume_state.json')
state.update(state='B_BLE_ADDRESS_REVISED_C1P3_READY_CURRENT_SETUP_PENDING',
    config=str(BASE/'geometry_ff0p0006_Ls3p3_Le18p9_c1p3_01/config.json'),
    candidate=str(BASE/'geometry_ff0p0006_Ls3p3_Le18p9_c1p3_01/candidate.settings.json'),
    ble_address=NEW_ADDRESS,ble_advertising_name='HY0020-MOUSE-B',
    last_confirmed_power_state='BOTH_OFF_USER_CONFIRMED_AFTER_C1P0',
    power_state='CURRENT_CONFIRMATION_REQUIRED',operator_ready=False,
    current_placement_confirmed=False,current_clearance_confirmed=False,
    current_hands_clear_confirmed=False,execution_pending=False,next_request_sent=False,
    next_action='B用HY0020(E1:7F:86:21:84:73)と機体Bの接続・開始中点315/90mm・前緑左下135度・干渉なし・両方ON・手離しの今回確認後に、補正+1.30度を1回だけ比較する。',
    address_revision=str(BASE/'B_ble_address_revision_20261120.json'))
(BASE/'resume_state.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(state='VALIDATED_NOT_EXECUTED',address=NEW_ADDRESS,
    advertising_name='HY0020-MOUSE-B',configs_updated=len(changes),
    historical_trials_unchanged=True,robot_commands_sent=0),ensure_ascii=False))
