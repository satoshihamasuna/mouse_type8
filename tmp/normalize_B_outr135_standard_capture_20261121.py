"""将来の未実行stageだけを標準MCP接続手順へ揃える。既存試行と幾何条件は維持。"""
from pathlib import Path
from datetime import datetime, timezone
import ast
import difflib
import hashlib
import json
import shutil
import sys
root=Path(__file__).resolve().parents[1]
base=root/'tools/logs/pixel_camera/turn1600_B_outr135_20261121_resume_v1'
read=lambda path:json.loads(path.read_text(encoding='utf-8-sig'))
sys.path.insert(0,str(root/'tools'))
from pixel_camera_tuning_mcp import McpTuningService
rows=[]
for number in (1,2):
    stage=base/f'standard_flow_c1p0_Ls5p3_Le31_{number:02d}'
    path=stage/'run_recorded_once.py';source=path.read_text(encoding='utf-8-sig')
    before=ast.parse(source);expected=ast.parse(source)
    cls=next(node for node in before.body if isinstance(node,ast.ClassDef) and node.name=='RecordedService')
    method=next(node for node in cls.body if isinstance(node,ast.FunctionDef) and node.name=='perform')
    expected_cls=next(node for node in expected.body if isinstance(node,ast.ClassDef) and node.name=='RecordedService')
    expected_cls.body=[node for node in expected_cls.body if not isinstance(node,ast.FunctionDef) or node.name!='perform']
    lines=source.splitlines(keepends=True)
    changed=''.join(lines[:method.lineno-1]+lines[method.end_lineno:])
    assert ast.dump(ast.parse(changed),include_attributes=False)==ast.dump(expected,include_attributes=False)
    compile(changed,str(path),'exec')
    shutil.copyfile(path,stage/'run_recorded_once.before_standardization.py')
    path.write_text(changed,encoding='utf-8',newline='\n')
    with (stage/'standardization.patch').open('x',encoding='utf-8') as stream:
        stream.writelines(difflib.unified_diff(source.splitlines(keepends=True),changed.splitlines(keepends=True),
            fromfile='run_recorded_once.py (before)',tofile='run_recorded_once.py (standard)'))
    config=read(stage/'config.json');settings=read(stage/'candidate.settings.json');plan=read(stage/'placement_plan.json')
    assert config['address']=='E1:7F:86:21:84:73' and config.get('stream_period_ms',5)==5
    payload=McpTuningService(stage/'standard_offline_validation',{}).prepare(str(stage/'config.json'),
        str(stage/'candidate.settings.json'),plan['placement'],None,20.)
    with (stage/'offline_validation_standard_flow.json').open('x',encoding='utf-8') as stream:
        json.dump(dict(state='VALIDATED_NOT_EXECUTED',payload=payload,
            wrapper_change='RecordedService.performだけ除去し標準McpTuningService.performを継承。',
            static_quality_policy_unchanged=True,capture_settings_unchanged=True,hardware_operations=0),stream,ensure_ascii=False,indent=2)
    rows.append(dict(stage=str(stage),wrapper=str(path),wrapper_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        stream_period_ms=5,char_delay=config['char_delay'],parameters=settings['parameters']))
record=dict(prepared_at=datetime.now(timezone.utc).isoformat(),future_stages=rows,
    comparison=str(base/'other_B_workflow_comparison_v1/comparison.json'),
    source='tools/pixel_camera_tuning_mcp.py:McpTuningService.perform',
    facts='標準performは既に静止撮影/転送/照合を終えてからBLEを開始する。独自performで同じ前処理を包む必要はない。',
    removed='別PixelCameraClientでの前処理と、標準静止確認をキャッシュ結果へ差し替える独自override。実際に二重録画していたという意味ではない。',
    retained='粗い配置照合、全フレームArUco/95%基準、Bアドレス、5ms、文字間隔0.08秒、PID/FF/吸引、角度・長さ、後走加速度20m/s²。',
    cause_proven=False,hardware_operations=0,auto_motion_after_communication_test=False)
with (base/'other_B_workflow_comparison_v1/standardization.json').open('x',encoding='utf-8') as stream:
    json.dump(record,stream,ensure_ascii=False,indent=2)
state_path=base/'resume_state.json';state=read(state_path)
state.update(standard_flow_future_stages=[row['stage'] for row in rows],
    workflow_comparison=str(base/'other_B_workflow_comparison_v1/comparison.json'),
    workflow_standardization=str(base/'other_B_workflow_comparison_v1/standardization.json'))
state_path.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
with (base/'README.md').open('a',encoding='utf-8') as stream:
    stream.write('\n他のB記録と比較：E1アドレスでgatewayを開始した取得だけでは、out135が2/5完了・3切断、rV90が7/8完了・1切断。どちらも5ms/文字間隔0.08秒。設定だけで切断原因を断定しない。将来用の独立standard_flow stage2件を作り、現在の標準MCPが既にcamera-before-BLEを実装しているため、out135固有のperform overrideだけを削除。粗い配置条件と検出/走行/通信条件は保持し、ASTでそれ以外の変更がないことを確認してprepareを再検証。旧失敗/成功試行は変更しない。通信のみの診断計画を先に実施し、診断から勝手に走行へ移らない。詳細other_B_workflow_comparison_v1/。\n')
print(json.dumps(dict(standard_flow_stages=len(rows),stream_period_ms=5,hardware_operations=0,cause_proven=False),ensure_ascii=False))
