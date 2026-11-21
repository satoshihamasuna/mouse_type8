"""比較条件のモード照合を一時ラッパーへ加える。通信呼出しは増やさない。"""
from pathlib import Path
import ast
import hashlib
import json

root=Path(__file__).resolve().parents[1]
base=root/'tools/logs/pixel_camera/turn1600_B_outr135_20261121_resume_v1'
stage=base/'refine_c1p0_Ls5p3_Le29p2_01'
wrapper=stage/'run_recorded_once.py'
before=wrapper.read_text(encoding='utf-8')
assert not (stage/'run_recorded_once.before_mode_guard.py').exists()
needle="            tuning.save_json(self.folder/'runtime_modes.json', modes)\n            self.modes_recorded = True"
replacement="""            tuning.save_json(self.folder/'runtime_modes.json', modes)
            expected_modes = {'wall_ekf show': 'WALL_EKF mode:shadow ',
                              'wall_edge show': 'WALL_EDGE mode:legacy ',
                              'turn_ekf show': 'TURN_EKF mode:control '}
            for command, expected_prefix in expected_modes.items():
                if not modes[command].startswith(expected_prefix):
                    raise RuntimeError('out135 comparison mode changed: '+command+' / '+modes[command])
            self.modes_recorded = True"""
assert before.count(needle)==1
after=before.replace(needle,replacement)
compile(after,str(wrapper),'exec')
assert '    def perform(' not in after
ast.parse(after)
(stage/'run_recorded_once.before_mode_guard.py').write_text(before,encoding='utf-8')
wrapper.write_text(after,encoding='utf-8')
value=dict(state='COMPILED_OFFLINE_NO_HARDWARE',
    only_change='RecordedDeviceの既存モード照会結果を、前回と同じshadow/legacy/controlで照合。追加照会・設定・再送なし。',
    before_sha256=hashlib.sha256(before.encode()).hexdigest(),after_sha256=hashlib.sha256(after.encode()).hexdigest(),
    expected_modes=dict(wall_ekf='shadow',wall_edge='legacy',turn_ekf='control'),
    hardware_operations=0,firmware_source_changed=False)
with (stage/'runtime_mode_guard.json').open('x',encoding='utf-8') as stream:
    json.dump(value,stream,ensure_ascii=False,indent=2)
print(json.dumps(value,ensure_ascii=False),flush=True)
