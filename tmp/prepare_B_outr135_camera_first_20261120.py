"""B1600右135度出口の未実行wrapperにも確認済みの撮影→BLE順を適用。機器操作なし。"""
from pathlib import Path
import ast
import hashlib
import json
import shutil
ROOT=Path(__file__).resolve().parents[1]
source_path=ROOT/'tools/logs/pixel_camera/turn1600_B_inr135_20261120_v1/geometry_c0p4_Ls10p5_Le17p2_02/run_recorded_once.py'
target=ROOT/'tools/logs/pixel_camera/turn1600_B_outr135_20261120_v1/baseline01/run_recorded_once.py'
source=source_path.read_text(encoding='utf-8-sig');tree=ast.parse(source)
service=next(node for node in tree.body if isinstance(node,ast.ClassDef) and node.name=='RecordedService')
method=next(node for node in service.body if isinstance(node,ast.FunctionDef) and node.name=='perform')
segment=''.join(source.splitlines(keepends=True)[method.lineno-1:method.end_lineno])+'\n\n'
text=target.read_text(encoding='utf-8-sig')
needle='class RecordedService(tuning.McpTuningService):\n'
assert text.count(needle)==1 and 'camera_before_ble' not in text
backup=target.with_name('run_recorded_once_before_camera_first.py')
assert not backup.exists();shutil.copyfile(target,backup)
text=text.replace(needle,needle+segment);compile(text,str(target),'exec')
target.write_text(text,encoding='utf-8',newline='\n')
with target.with_name('camera_before_ble_sequence.json').open('x',encoding='utf-8') as file:
    json.dump(dict(source_method=str(source_path),source_sha256=hashlib.sha256(source_path.read_bytes()).hexdigest(),
        original_wrapper=str(backup),state='PREPARED_NOT_EXECUTED',sequence='static_capture_transfer_check_then_BLE_then_readback_then_once',
        settings_and_placement_unchanged=True,hardware_operations=0),file,ensure_ascii=False,indent=2)
print('Prepared out_r135 camera-before-BLE sequence; hardware operations: 0')
