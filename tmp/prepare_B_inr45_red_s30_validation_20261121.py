"""既存の独立全フレーム検証をS30候補と010/011/012へ限定して準備する。"""
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
template=ROOT/'tmp/validate_B_inr45_red_hue160_20261121.py';source=template.read_text(encoding='utf-8')
start=source.index('RUNS={');end=source.index('\nkey=sys.argv[1]',start)
mapping="""RUNS={
 '010':'geometry04_correction_m0p9/local_execution/run_B_INR45_1600_correction_m0p9_green_20261121_010',
 '011':'geometry05_correction_m1p2/local_execution/run_B_INR45_1600_correction_m1p2_green_20261121_011',
 '012':'geometry05_repeat01_correction_m1p2/local_execution/run_B_INR45_1600_m1p2_repeat01_green_20261121_012'}"""
source=source[:start]+mapping+source[end:]
source=source.replace('B専用赤色相候補を3本へ適用。','B専用赤彩度S30候補を3本へ適用。')
source=source.replace("BASE/'red_hue160_validation_v1'","BASE/'red_s30_validation_v1'")
source=source.replace("BASE/'body_geometry_B_green_red_h160_candidate_v4.json'","BASE/'body_geometry_B_green_red_h160_s30_candidate_v5.json'")
old="old_red=dict(original_geometry['marker_detection']['red']);new_red=dict(geometry['marker_detection']['red']);new_red.pop('hue_ranges')"
new="old_red=dict(original_geometry['marker_detection']['red']);new_red=dict(geometry['marker_detection']['red'])\nassert old_red.pop('minimum_saturation')==40 and new_red.pop('minimum_saturation')==30"
assert source.count(old)==1;source=source.replace(old,new)
old="save(OUT/'summary.json',summary)";assert source.count(old)==1
source=source.replace(old,"if not (OUT/'summary.json').exists():save(OUT/'summary.json',summary)")
destination=ROOT/'tmp/validate_B_inr45_red_s30_20261121.py';compile(source,str(destination),'exec')
with destination.open('x',encoding='utf-8') as f:f.write(source)
print('Prepared S30-only validation for normal captures010/011/012; no hardware command')
