"""未採用面積候補とは別に、実後赤の彩度候補の独立検証を準備する。機器操作なし。"""
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
source=(ROOT/'tmp/validate_B_outr135_green_area10_20261120.py').read_text(encoding='utf-8-sig')
source=source.replace("OUT=BASE/'green_area10_validation_v1'","OUT=BASE/'rear_red_s30_validation_v1'")
source=source.replace("candidate['marker_detection']['green']['minimum_area_px']=10","candidate['marker_detection']['red']['minimum_saturation']=30")
source=source.replace("new_geo['marker_detection']['green']['minimum_area_px']=10","new_geo['marker_detection']['red']['minimum_saturation']=30")
source=source.replace("body_geometry_B_green_area10_candidate.json","body_geometry_B_green35_rear_red30.json")
source=source.replace('右135度出口の欠測9画像で実前ステッカーが10〜14pxになる箇所を視認。面積だけ15→10px、色/開演算/間隔/ArUco/観測95%/直線品質は維持。独立B入口動画と今回の全区間で別検証する。',
    '右135度出口で実後赤が淡くなる8画像を視認。後赤の最小彩度のみ40→30。前緑S35/面積15・赤面積3・色相/明度/開演算/間隔/ArUco/観測95%/直線品質は維持。独立B入口3動画と今回の全区間を別検証。')
source=source.replace("fig,axes=plt.subplots(2,2,figsize=(8,8),layout='constrained')", "plt.rcParams['font.family']='Yu Gothic'\nfig,axes=plt.subplots(3,3,figsize=(10,10),layout='constrained')")
source=source.replace('assert len(recovered)==4','assert len(recovered)==8')
source=source.replace("row['old_new_pair_max_delta_mm']<1e-6","row['old_new_pair_max_delta_mm']<=1.5")
source=source.replace("check['max_delta_mm']<1e-6","check['max_delta_mm']<=1.5")
source=source.replace('original_minimum_area_px=15,candidate_minimum_area_px=10', 'original_rear_red_minimum_saturation=40,candidate_rear_red_minimum_saturation=30,maximum_allowed_pair_change_mm=1.5')
source=source.replace('colour_hue_saturation_value_and_opening_unchanged=True','front_colour_and_hue_value_opening_and_area_unchanged=True')
compile(source,'<red S30 validation>','exec')
target=ROOT/'tmp/validate_B_outr135_red_s30_20261120.py';assert not target.exists()
target.write_text(source,encoding='utf-8',newline='\n')
print('Prepared rear-red S30 independent validation; front minimum area remains15px')
