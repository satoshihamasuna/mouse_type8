"""OFF動画の色分割診断に基づく独立検証用の機体JSONを保存。実機操作なし。"""
from pathlib import Path
import copy
import hashlib
import json

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_r180_20261120_v1'
read=lambda p:json.loads(Path(p).read_text(encoding='utf-8-sig'))


def save(path,value):
    with path.open('x',encoding='utf-8') as output:
        json.dump(value,output,ensure_ascii=False,indent=2)


geometry=read(BASE/'body_geometry_B_green_front_v1.json')
geometry['marker_detection']['green']['minimum_saturation']=45
geometry['marker_detection_source']='B R180 OFF01の実マスクを視認。S25は車体反射と連結し前点が最大3.52px変動。S45は実ステッカーへ分離。Hue65..95/V65/面積15/開演算3ellipse/赤条件は維持。独立OFF02と通電時・走行時の検証は別に行う。'
geometry['previous_geometry']=dict(path=str(BASE/'body_geometry_B_green_front_v1.json'),
    sha256=hashlib.sha256((BASE/'body_geometry_B_green_front_v1.json').read_bytes()).hexdigest())
geometry['led_on_marker_validation']='未検証。新しい試写と走行の全区間で確認する。'
path=BASE/'body_geometry_B_green_front_s45_v2.json'
save(path,geometry)
config=read(BASE/'baseline01/config.json');config['geometry']=str(path)
save(BASE/'baseline01/config.green_s45.v2.json',config)
plan=copy.deepcopy(read(BASE/'off_static01/plan.json'));plan['geometry']=str(path)
out=BASE/'off_static01/s45_full_window_review';out.mkdir(exist_ok=False)
save(out/'plan.json',plan)
save(BASE/'marker_s45_candidate_basis.json',dict(state='CANDIDATE_INDEPENDENT_VALIDATION_REQUIRED',
    original_failure_preserved=str(BASE/'off_static01/static_check.json'),
    diagnostic=str(BASE/'off_static01/segmentation_s45_visual_v4/report.json'),
    new_geometry=str(path),source_video_points=17,
    original_heading_rms_deg=1.0084748268127441,
    diagnostic_S45_heading_rms_deg=.2905393011031507,
    acceptance_limits_changed=False,calibration_changed=False,robot_settings_changed=False))
print(path)
