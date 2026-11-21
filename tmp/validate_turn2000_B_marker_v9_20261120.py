"""B用の色検出候補を保存動画で検証する。機器接続・元データ上書きなし。"""
from pathlib import Path
import copy
import hashlib
import json
import sys

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn2000_B_r90_20261120_v1'
OUT=BASE/'marker_detection_revision_v9'
OUT.mkdir(exist_ok=False)
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'tools/analysis')]
from pixel_camera_static_pose import measure
from video_marker_pose import read_video_pose


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def save(path,value):
    with Path(path).open('x',encoding='utf-8') as stream:
        json.dump(value,stream,ensure_ascii=False,indent=2,allow_nan=False)


config=read(BASE/'config.json')
reference=read(config['reference'])
geometry=copy.deepcopy(read(config['geometry']))
geometry['marker_detection']['yellow'].update(minimum_saturation=30,hue_ranges=[[20,75]],opening_kernel_size=0)
geometry['previous_marker_geometry']=dict(path=config['geometry'],sha256=hashlib.sha256(Path(config['geometry']).read_bytes()).hexdigest())
geometry['marker_detection_source']='B2000候補v9: S>=30,hue20..75。白色反射による前黄重心変動を抑える。赤・距離範囲・高さ・盤面は不変。保存動画で検証中、実機未適用。'
save(OUT/'body_geometry_B_s30_h20_75.json',geometry)
config['geometry']=str(OUT/'body_geometry_B_s30_h20_75.json')
save(BASE/'config.marker_v9.json',config)
placement=read(BASE/'Ki005_relative_start_plan.json')['placement']
static_results=[]
for tag in ('powered03','ki005_repeat02_off','ki005_repeat02_settled_off','powered04'):
    capture=read(BASE/(tag+'_static_capture.json'))
    checked=measure(Path(capture['video_file']),reference,geometry,placement)
    checked['source_capture']=str(BASE/(tag+'_static_capture.json'))
    checked['sensor_stability_passed']=bool(checked.get('observed_fraction',0)>=.95
        and checked.get('heading_jitter_rms_deg',999)<=.5
        and checked.get('heading_half_window_difference_deg',999)<=.5
        and checked.get('heading_jitter_max_deg',999)<=2.)
    save(OUT/(tag+'_static_check_v9.json'),checked)
    static_results.append(dict(tag=tag,passed=checked['passed'],sensor_stability_passed=checked['sensor_stability_passed'],
        center_board_mm=checked.get('center_board_mm'),heading_deg=checked.get('heading_deg'),
        observed_fraction=checked.get('observed_fraction'),heading_jitter_rms_deg=checked.get('heading_jitter_rms_deg'),
        failure_reasons=checked['failure_reasons']))
    print(json.dumps(static_results[-1],ensure_ascii=True),flush=True)

motion_results=[]
comparison=read(BASE/'baseline_vs_ki005_video_review_v1/video_comparison.json')['comparisons']
for case in comparison:
    trial=read(case['source_trial'])
    window=(case['entry_window_pts_s'][0]-.1,case['exit_window_pts_s'][1]+.15)
    pose,_,_,_=read_video_pose(Path(trial['video_file']),'yellow',marker_reference=reference,
        video_window=window,frame_stride=4,marker_detection=geometry['marker_detection'],rear_colour='red',
        nominal_marker_span_mm=geometry['nominal_marker_span_mm'],marker_span_limits_mm=geometry['marker_span_limits_mm'])
    pose.to_csv(OUT/(case['label']+'_motion_screen_v9.csv'),index=False)
    valid=pose.marker_detected_raw.to_numpy(bool)&pose.calibration_valid.to_numpy(bool)
    motion_results.append(dict(label=case['label'],frames=len(pose),valid_frames=int(valid.sum()),
        observation_rate=float(valid.mean()),calibration=pose.attrs['marker_calibration'],
        window_pts_s=list(window),scope='stride4 provisional motion screen; not full-frame validation'))
    print(json.dumps(motion_results[-1],ensure_ascii=True),flush=True)
save(OUT/'validation.json',dict(mouse='B',static_results=static_results,motion_screens=motion_results,
    adopted=False,robot_commands_sent=0,source_geometry_unchanged=True,
    full_motion_reanalysis_required=True,
    input_sha256={str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest() for path in
        (Path(__file__),BASE/'marker_detection_sweep_v1/sweep_verified.json',BASE/'Ki005_relative_start_plan.json')}))
