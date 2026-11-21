"""前日の評価手順を保持したまま今日の独立出力先へ複製。実機操作なし。"""
from pathlib import Path
import hashlib
import json
root=Path(__file__).resolve().parents[1]
base=root/'tools/logs/pixel_camera/turn1600_B_outr135_20261121_resume_v1'
names=['review_B_outr135_saved_full','review_turn1600_B_outr135_tracking',
       'review_turn1600_B_outr135_midpoint','review_B_outr135_lengths002_visual']
rows=[]
for name in names:
    source=root/'tmp'/f'{name}_20261120.py'
    target=root/'tmp'/f'{name}_20261121.py'
    assert not target.exists()
    original=source.read_text(encoding='utf-8-sig')
    old_base='tools/logs/pixel_camera/turn1600_B_outr135_20261120_v1'
    assert old_base in original
    changed=original.replace(old_base,'tools/logs/pixel_camera/turn1600_B_outr135_20261121_resume_v1')
    for other in names:
        changed=changed.replace(other+'_20261120.py',other+'_20261121.py')
    compile(changed,str(target),'exec')
    with target.open('x',encoding='utf-8',newline='\n') as stream:stream.write(changed)
    rows.append(dict(source=str(source),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        target=str(target),target_sha256=hashlib.sha256(target.read_bytes()).hexdigest()))
with (base/'analysis_tool_sources.json').open('x',encoding='utf-8') as stream:
    json.dump(dict(changes='出力BASEと複製評価スクリプト名だけを今日用へ変更。品質条件と評価計算は維持。',
        sources=rows,original_sources_unchanged=True,hardware_operations=0),stream,ensure_ascii=False,indent=2)
state_file=base/'resume_state.json'
state=json.loads(state_file.read_text(encoding='utf-8-sig'))
state.update(state='GEOMETRY5P3_31_C1P0_001_CAPTURED_BOTH_OFF_REVIEWING',
    power_state='BOTH_OFF_USER_CONFIRMED',operator_ready=False,operator_power_off_confirmation='両方OFF・接触なし',
    execution_pending=False,current_placement_confirmed=False,current_hands_clear_confirmed=False,
    contact_detected=False,ble_connected=False,last_execution_write_attempted=True,execution_commands_sent=1,
    last_request_id='B_OUTR135_1600_c1p0_Ls5p3_Le31_green_20261121_001',
    last_run=str(base/'geometry_c1p0_Ls5p3_Le31_01/local_execution/run_B_OUTR135_1600_c1p0_Ls5p3_Le31_green_20261121_001'),
    next_action='両方OFFで保存済み動画・ログを解析。今回の候補を評価し次の候補/反復を準備。')
state_file.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print('Prepared four saved-data reviewers; no hardware operations')
