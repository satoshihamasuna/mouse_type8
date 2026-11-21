"""保存済みA1800 in_r45のログ停止確認後を、固定の動画窓で照合する。"""
from pathlib import Path
from hashlib import sha256
import argparse
import json
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tools/logs/pixel_camera/turn1800_A_inr45_20261120_v1'
sys.path[:0] = [str(ROOT / 'tools'), str(ROOT / 'tools/analysis')]
from video_marker_pose import read_video_pose, board_to_pose_transform, transform_points
from pixel_camera_geometry import relative_marker_midpoint

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--run', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
run, out = args.run.resolve(), args.output.resolve()
assert run.is_relative_to(BASE) and out.is_relative_to(BASE)
assert not out.exists()
trial_path, geometry_path, reference_path = run/'trial.json', run/'body_geometry_used.json', run/'marker_reference_used.json'
trial, geometry, reference = [json.loads(p.read_text(encoding='utf-8')) for p in
    (trial_path, geometry_path, reference_path)]
assert trial['state'] == 'CAPTURED_REVIEW_REQUIRED'
assert trial['mouse'] == 'A' and trial['motion'] == 'in_r45' and trial['preset'] == 1800
assert trial['log_quality']['full_motion_retained']
assert not geometry.get('body_appearance_model', {}).get('enabled', False)
confirmation = trial['recording_stop']['confirmation_camera_seconds']
window = (confirmation+.10, confirmation+.54)
out.mkdir(parents=True)
cv2.setNumThreads(2)
pose, _, _, _ = read_video_pose(Path(trial['video_file']), geometry['front_colour'],
    marker_reference=reference, video_window=window, frame_stride=1,
    marker_detection=geometry['marker_detection'], rear_colour=geometry['rear_colour'],
    nominal_marker_span_mm=geometry['nominal_marker_span_mm'],
    marker_span_limits_mm=geometry['marker_span_limits_mm'])
pose.to_csv(out/'pose.csv', index=False)
assert pose.attrs.get('detector', {}).get('inferred_points_used_as_observations', False) is False
valid = pose.marker_detected_raw.to_numpy(bool) & pose.calibration_valid.to_numpy(bool)
matrix = np.linalg.inv(board_to_pose_transform(reference))
front = transform_points(pose[['blue_x_mm', 'blue_y_mm']].to_numpy(), matrix)
rear = transform_points(pose[['yellow_x_mm', 'yellow_y_mm']].to_numpy(), matrix)
relative = relative_marker_midpoint(front, rear, trial['camera_height_mm'],
    geometry['front_height_mm'], geometry['rear_height_mm'], valid)
xy = relative['displacement_xy_mm'][valid]
scatter = float(np.sqrt(np.sum(np.var(xy, axis=0)))) if len(xy) else None
maximum = float(np.linalg.norm(xy-np.median(xy, axis=0), axis=1).max()) if len(xy) else None
span = float(np.ptp(pose.time_video_s[valid])) if valid.any() else 0.
result = dict(source_trial=str(trial_path), video_file=trial['video_file'],
    window_s=list(window), window_basis='recording_stop confirmation+[0.10,0.54]s fixed before measurement',
    frame_stride=1, observed=int(valid.sum()), frames=len(pose),
    observation_fraction=float(valid.mean()), calibration_fraction=float(pose.calibration_valid.mean()),
    observed_span_s=span, scatter_rms_mm=scatter, maximum_from_median_mm=maximum,
    per_frame_calibration=pose.attrs['marker_calibration'], detector=pose.attrs.get('detector'),
    passed=bool(valid.sum() >= 12 and valid.mean() >= .95 and span >= .25
        and scatter is not None and scatter <= 1. and maximum <= 3.),
    input_sha256={str(p):sha256(p.read_bytes()).hexdigest() for p in
        (trial_path, geometry_path, reference_path, Path(__file__))},
    robot_connections=0, source_trial_modified=False)
with (out/'review.json').open('x', encoding='utf-8') as stream:
    json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
print(json.dumps(result, ensure_ascii=False), flush=True)
