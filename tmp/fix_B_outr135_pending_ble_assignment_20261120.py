"""新しいA/B割当を未実行out135候補だけへ適用。実機への接続・送信なし。"""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json
import shutil
import sys

root=Path(__file__).resolve().parents[1]
base=root/'tools/logs/pixel_camera/turn1600_B_outr135_20261120_v1'
sys.path.insert(0,str(root/'tools'))
from pixel_camera_tuning_mcp import McpTuningService
read=lambda path:json.loads(path.read_text(encoding='utf-8-sig'))
def save(path,value):
    with path.open('x',encoding='utf-8') as stream:
        json.dump(value,stream,ensure_ascii=False,indent=2,allow_nan=False)
        stream.write('\n')
address='E1:7F:86:21:84:73'
source=root/'tools/ble/README.md'
assert '| B（新品） | `HY0020-MOUSE-B` | `E1:7F:86:21:84:73` |' in source.read_text(encoding='utf-8-sig')
stages=[base/f'geometry_c1p0_Ls5p3_Le31_{number:02d}' for number in (1,2,3)]
for stage in stages:
    assert not list((stage/'local_execution/jobs').glob('*.json'))
    assert not list((stage/'local_execution').glob('transport_*'))
    assert not (stage/'config_before_B_ble_assignment.json').exists()
rows=[]
for stage in stages:
    config_path=stage/'config.json'
    before=read(config_path)
    assert before['mouse']=='B'
    shutil.copyfile(config_path,stage/'config_before_B_ble_assignment.json')
    config=dict(before,address=address)
    config_path.write_text(json.dumps(config,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    plan=read(stage/'placement_plan.json')
    payload=McpTuningService(stage/'ble_assignment_offline_validation',{}).prepare(
        str(config_path),str(stage/'candidate.settings.json'),plan['placement'],None,20.)
    assert payload['config']['address']==address and payload['config']['mouse']=='B'
    evidence=dict(state='VALIDATED_NOT_EXECUTED',payload=payload,
        old_address=before['address'],new_address=address,advertisement_name='HY0020-MOUSE-B',
        assignment_source=str(source),supersedes_address_in='offline_validation.json',
        source_config_snapshot='config_before_B_ble_assignment.json',hardware_operations=0)
    save(stage/'offline_validation_B_ble_assignment.json',evidence)
    phase=read(stage/'phase.json')
    phase.update(ble_address=address,ble_advertisement_name='HY0020-MOUSE-B',
        ble_assignment_validation=str(stage/'offline_validation_B_ble_assignment.json'),
        operator_ready=False,execution_pending=False,next_candidate_executed=False)
    (stage/'phase.json').write_text(json.dumps(phase,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    rows.append(dict(stage=str(stage),before_address=before['address'],current_address=address,
        validation=str(stage/'offline_validation_B_ble_assignment.json')))
record=dict(recorded_at=datetime.now(timezone.utc).isoformat(),user_instruction='BLEの割り当て注意',
    assignment=dict(A=dict(name='HY0020-MOUSE-A',address='C1:B5:60:87:91:73'),
        B=dict(name='HY0020-MOUSE-B',address=address)),assignment_source=str(source),
    assignment_source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),pending_stages_updated=rows,
    captured_configs_and_archives_preserved=True,hardware_operations=0,
    observed_other_active_trial='turn1600_B_rv90_20261120_v1/geometry_ff0p0006_Ls3p3_Le18p9_c1p3_02',
    resource_policy='別試行のプロセス/中継を停止・変更しない。このout135から実機接続を始めない。再開時に他の取得が完了したことと現在の実機配置を確認する。',
    historical_identity='取得済み試行は操作者の機体B申告と取得時のBLEアドレスを保持。今回の割当変更だけで過去の実機をA/Bへ再判定しない。')
save(base/'ble_assignment_update_20261120_v1.json',record)
state=read(base/'resume_state.json')
state.update(state='LENGTHS5P3_31_BLE_ASSIGNMENT_CORRECTED_UNSENT_OTHER_TRIAL_OBSERVED',
    power_state='NOT_RECONFIRMED_WHILE_OTHER_TRIAL_ACTIVE',
    ble_address=address,ble_advertisement_name='HY0020-MOUSE-B',
    ble_assignment_evidence=str(base/'ble_assignment_update_20261120_v1.json'),
    ble_connected=False,operator_ready=False,execution_pending=False,next_request_sent=False,
    current_placement_confirmed=False,current_hands_clear_confirmed=False,
    next_action='別のB試行が動作中のため接続/走行なし。次のout135はB用E1:7F:86:21:84:73を使用。他の実機取得終了と今回の出口配置/ON/手離し確認後に新IDで進める。')
(base/'resume_state.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
with (base/'README.md').open('a',encoding='utf-8') as stream:
    stream.write('\nBLE割当更新：tools/ble/README.mdの2026-11-20割当（A=C1:B5:60:87:91:73、B=E1:7F:86:21:84:73）を照合。未実行geometry_c1p0_Ls5p3_Le31_01〜03だけB用アドレスへ変更しオフラインprepareを再検証。元configを別名保存し、取得済み設定・動画・ログ・archiveは変更していない。別のB rV90実機試行が動作中のため、このout135からの接続/走行は0件。過去試行は当時の機体申告とアドレスを保ち、割当変更から遡って機体を推定しない。詳細ble_assignment_update_20261120_v1.json。\n')
print(json.dumps(dict(B_address=address,pending_stages_updated=len(rows),hardware_operations=0),ensure_ascii=False))
