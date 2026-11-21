"""OFF静止像と走行動画の初期マーカー向き・入口方向を区別する。機器操作なし。"""
from pathlib import Path
import hashlib
import json
import shutil
import sys

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'tools'),str(ROOT/'tools/analysis')]
from video_marker_pose import board_to_pose_transform,transform_points

BASE = ROOT/'tools/logs/pixel_camera/turn1600_B_r90_20261120_v1'
REVIEW = BASE/'green_correction1p0_001_video_review_v1'
RUN = BASE/'cli_session08/run_B_R90_1600_Lstart13p6_corr1p0_green_20261120_001'


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def new(path,value):
    with path.open('x',encoding='utf-8') as output:
        json.dump(value,output,ensure_ascii=False,indent=2)


assert not (RUN/'archive.json').exists()
baseline = read(BASE/'green_Lstart13p6_three_review_v1/video_comparison.json')['comparisons']
current = read(REVIEW/'video_comparison.json')['comparisons'][0]
cases = baseline+[current]
static_names = ['B_green_front_OFF02','green_Lstart13p6_after_ble_diagnosis_placed01',
    'green_Lstart13p6_repeat03_placed01','green_correction1p0_placed02']
rows = []
for case,static_name in zip(cases,static_names):
    folder = Path(case['source_trial']).parent
    report = read(folder/'full/report.json')
    pose = pd.read_csv(folder/'full/apparent_floor_marker_pose.csv')
    start = report['required_motion_window_quality']['video_motion_start_s']
    window = [start-.6,start-.1]
    keep = pose.marker_detected_raw & pose.calibration_valid & pose.time_video_s.between(*window)
    reference = read(folder/'marker_reference_used.json')
    inverse = np.linalg.inv(board_to_pose_transform(reference))
    front = transform_points(pose.loc[keep,['blue_x_mm','blue_y_mm']].to_numpy(),inverse)
    rear = transform_points(pose.loc[keep,['yellow_x_mm','yellow_y_mm']].to_numpy(),inverse)
    assert len(front)>=30
    heading = np.degrees(np.arctan2((front-rear)[:,1],(front-rear)[:,0]))
    off = read(BASE/(static_name+'_static_check.json'))
    tracking = read(folder/'tracking_1ms_review_v2/verification.json')
    log = pd.read_csv(folder/'saved_1ms.csv')
    first_turn = np.flatnonzero(log['motion.pattern'].to_numpy()==3)[0]
    omega_before = log['ego.rad_velo'].to_numpy()[:first_turn]
    gyro_before = float(np.sum((omega_before[:-1]+omega_before[1:])*.5)
        *tracking['measured_log_period_ms']/1000*180/np.pi)
    rows.append(dict(label=case['label'],correction_deg=case['correction_deg'],
        video_pre_motion_window_s=window,stationary_observations=int(keep.sum()),
        off_apparent_marker_heading_deg=off['heading_deg'],
        video_stationary_apparent_marker_heading_deg=float(np.median(heading)),
        off_to_video_marker_heading_difference_deg=float(np.median(heading)-off['heading_deg']),
        measured_entry_path_board_heading_deg=case['entry_board_heading_deg'],
        measured_exit_path_board_heading_deg=case['exit_board_heading_deg'],
        path_turn_deg=case['entry_to_exit_path_rotation_deg'],
        pre_turn_signed_gyro_integral_deg=gyro_before,
        pre_turn_control_ir_abs_max_mm=float(log['control_ir'].iloc[:first_turn].abs().max()) if 'control_ir' in log else None))
document = dict(scope='保存済み資料だけの診断。実マーカー中点・見かけの向きであり、機軸の絶対yaw校正ではない。',
    rows=rows,causal_attribution_proven=False,
    limitations=['時間窓は軌跡当てはめ同期に依存。高さ補正と機軸オフセットは未校正。',
        'control_irは使用横偏差mmで、操舵出力そのものではない。角速度・IR偏差だけで壁制御やスリップを原因と断定しない。'])
new(REVIEW/'start_heading_diagnostic.json',document)
new(RUN/'video_shape_review_v1/start_heading_diagnostic.json',document)
shutil.copy2(__file__,RUN/'analysis_sources'/Path(__file__).name)
manifest_path = RUN/'analysis_sources/source_hashes.json'
manifest = read(manifest_path)
manifest[str(Path(__file__).relative_to(ROOT))] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
manifest_path.write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(rows,ensure_ascii=False,indent=2))
