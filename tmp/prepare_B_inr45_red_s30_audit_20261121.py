"""S30変更の最大差分と回復フレームを、既存の目視監査へ渡す。"""
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
source=(ROOT/'tmp/audit_B_inr45_red_hue160_frames_20261121.py').read_text(encoding='utf-8')
replacements={"BASE/'red_hue160_validation_v1'":"BASE/'red_s30_validation_v1'",
 "OUT=VALIDATION/'frame_audit_v2'":"OUT=VALIDATION/'frame_audit_v1'",
 "('006','007','008')":"('010','011','012')",
 "key=='008'":"key=='012'","key=='006'":"key=='010'",
 "'independent_motion_window_v1' if key=='012' else 'full'":"'full'",
 "'rear H160'":"'rear S30'"}
for old,new in replacements.items():
 assert old in source,old
 source=source.replace(old,new)
destination=ROOT/'tmp/audit_B_inr45_red_s30_frames_20261121.py';compile(source,str(destination),'exec')
with destination.open('x',encoding='utf-8') as f:f.write(source)
print('Prepared S30 frame audit; no hardware command')
