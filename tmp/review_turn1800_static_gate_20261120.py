"""保存済み開始姿勢の判定を診断する。実機接続も合否条件の変更もしない。"""
from pathlib import Path
import json
import hashlib
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/analysis'))
from video_marker_pose import board_to_pose_transform, read_video_pose, transform_points

BASE = ROOT / 'tools/logs/pixel_camera/turn1800_A_r90_20261119_v1'
OUT = BASE / 'static_gate_review_20261120_v1'
cases = [
    ('failed_repeat02', BASE / 'repeat_c1p25_02/local_execution/prepare_A_R90_1800_c1p25_repeat02_20261120_001'),
    ('passed_c1p25_01', BASE / 'next_c1p25_Ls7p8_Le25/local_execution/prepare_A_R90_1800_c1p25_Ls7p8_Le25_20261120_002'),
]
results = []
for label, source in cases:
    folder = OUT / label
    folder.mkdir(exist_ok=False)
    plan_path = source / 'static_plan.json'
    plan = json.loads(plan_path.read_text(encoding='utf-8'))
    old = json.loads((source / 'static_check.json').read_text(encoding='utf-8'))
    reference = json.loads(Path(plan['reference']).read_text(encoding='utf-8'))
    geometry = json.loads(Path(plan['geometry']).read_text(encoding='utf-8'))
    pose, _, _, _ = read_video_pose(Path(plan['video']), geometry['front_colour'],
        marker_reference=reference, video_window=(3.3, 4.95), frame_stride=2,
        marker_detection=geometry.get('marker_detection'), rear_colour=geometry['rear_colour'],
        nominal_marker_span_mm=geometry['nominal_marker_span_mm'],
        marker_span_limits_mm=geometry['marker_span_limits_mm'])
    valid = pose.marker_detected_raw.to_numpy(bool) & pose.calibration_valid.to_numpy(bool)
    transform = np.linalg.inv(board_to_pose_transform(reference))
    front = transform_points(pose.loc[valid, ['blue_x_mm', 'blue_y_mm']].to_numpy(), transform)
    rear = transform_points(pose.loc[valid, ['yellow_x_mm', 'yellow_y_mm']].to_numpy(), transform)
    midpoint = (front + rear) / 2
    heading = np.degrees(np.arctan2((front-rear)[:, 1], (front-rear)[:, 0]))
    error = (heading-plan['placement']['heading_deg']+180) % 360 - 180
    median = float(np.median(error))
    assert abs(float(np.max(np.abs(error)))-old['max_heading_error_deg']) < 1e-5
    time = pose.loc[valid, 'time_video_s'].to_numpy()
    pose.to_csv(folder/'apparent_pose.csv', index=False)
    n = len(time)//2
    result = dict(label=label, source=str(source/'static_check.json'), original_passed=old['passed'],
        frames=len(pose), observed=int(valid.sum()),
        heading_median_deg=plan['placement']['heading_deg']+median,
        heading_error_quantiles_deg=dict(zip(['min','p2p5','median','p97p5','max'],
            np.percentile(error, [0,2.5,50,97.5,100]).tolist())),
        heading_jitter_rms_deg=float(np.sqrt(np.mean((error-median)**2))),
        heading_jitter_max_deg=float(np.max(np.abs(error-median))),
        heading_half_window_difference_deg=float(np.median(error[n:])-np.median(error[:n])),
        frames_outside_2deg=int(np.count_nonzero(np.abs(error)>2.)),
        midpoint_jitter_rms_mm=float(np.sqrt(np.mean(np.sum((midpoint-np.median(midpoint,axis=0))**2,axis=1)))),
        marker_span_mm=np.percentile(np.linalg.norm(front-rear,axis=1),[0,50,100]).tolist(),
        reference='Apparent marker pair projected onto board; uncorrected marker-height parallax, unverified body yaw',
        recognition_thresholds_changed=False, original_acceptance_unchanged=True,
        inputs_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in
            (plan_path, source/'static_check.json', Path(plan['reference']), Path(plan['geometry']))})
    with (folder/'diagnostics.json').open('x',encoding='utf-8') as f:
        json.dump(result,f,ensure_ascii=False,indent=2)
    cap=cv2.VideoCapture(plan['video'])
    cap.set(cv2.CAP_PROP_POS_MSEC,4200)
    ok,frame=cap.read(); cap.release()
    if ok:
        cv2.imwrite(str(folder/'frame_4p2s.jpg'),frame)
    results.append(result)
    print(json.dumps(result,ensure_ascii=False),flush=True)
with (OUT/'diagnostics.json').open('x',encoding='utf-8') as f:
    json.dump(dict(cases=results,hardware_commands_sent=0,
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()),f,ensure_ascii=False,indent=2)
