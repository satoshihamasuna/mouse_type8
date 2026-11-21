"""S20候補の回復点の実像と、ログ停止確認後の固定窓を保存する。実機接続なし。"""
from pathlib import Path
import argparse
import json
import sys

import cv2
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tools/logs/pixel_camera/turn1800_A_inr45_20261120_v1'
sys.path[:0] = [str(ROOT / 'tools'), str(ROOT / 'tools/analysis')]
from pixel_camera_geometry import relative_marker_midpoint
from pixel_camera_relative_pose import load_pose_cache
from video_marker_pose import (read_video_pose, board_to_pose_transform, transform_points,
    marker_registration, registration_quad, pose_world_corners)

TRIALS = {
    'first': ('front_Ls2p0_Le40_01/local_execution/run_A_INR45_1800_front_Ls2p0_Le40_20261120_001', 'full'),
    'second': ('resume1620_01/local_execution/run_A_INR45_1800_resume1620_20261120_001', 'full'),
    'third': ('repeat03_resume01/local_execution/run_A_INR45_1800_repeat03_resume_20261120_001', 'full_core_v1'),
}

def read(path):
    return json.loads(path.read_text(encoding='utf-8'))

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--label', choices=TRIALS, required=True)
args = parser.parse_args()
relative, original_name = TRIALS[args.label]
run = BASE / relative
trial = read(run / 'trial.json')
root = BASE / 'rear_yellow_s20_validation_v1'
core = root / (args.label + '_full_core')
out = root / (args.label + '_stop_and_recovered_images')
out.mkdir(exist_ok=False)
reference, geometry = read(core / 'marker_reference_used.json'), read(core / 'body_geometry_used.json')
provenance = read(core / 'apparent_floor_marker_pose.provenance.json')
new = load_pose_cache(core / 'apparent_floor_marker_pose.csv', trial, reference, geometry,
    provenance['video_window'], 1)
old_folder = run / original_name
old_reference, old_geometry = read(old_folder / 'marker_reference_used.json'), read(old_folder / 'body_geometry_used.json')
old_provenance = read(old_folder / 'apparent_floor_marker_pose.provenance.json')
old = load_pose_cache(old_folder / 'apparent_floor_marker_pose.csv', trial, old_reference,
    old_geometry, old_provenance['video_window'], 1)
old_flags = old.set_index('frame').marker_detected_raw
recovered = new.loc[new.marker_detected_raw & ~new.frame.map(old_flags).fillna(False)].copy()
recovered.to_csv(out / 'recovered_actual_observations.csv', index=False)
cv2.setNumThreads(2)
cap = cv2.VideoCapture(trial['video_file'])
cap.set(cv2.CAP_PROP_ORIENTATION_AUTO, 0)
count = len(recovered)
assert count > 0
cols, rows = 3, int(np.ceil(count / 3))
fig, axes = plt.subplots(rows, cols, figsize=(cols * 3.4, rows * 3.4), squeeze=False, layout='constrained')
for axis, (_, point) in zip(axes.flat, recovered.iterrows()):
    cap.set(cv2.CAP_PROP_POS_FRAMES, int(point.frame))
    ok, frame = cap.read()
    assert ok
    registration = marker_registration(frame, reference)
    assert {2, 3} <= set(registration['detected_marker_ids'])
    matrix = cv2.getPerspectiveTransform(registration_quad(registration), pose_world_corners(reference['board_side_mm']))
    pixels = transform_points([[point.blue_x_mm, point.blue_y_mm], [point.yellow_x_mm, point.yellow_y_mm]], np.linalg.inv(matrix))
    centre = pixels.mean(0)
    axis.imshow(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    axis.scatter(*pixels[0], s=130, facecolors='none', edgecolors='cyan', linewidths=1)
    axis.scatter(*pixels[1], s=130, facecolors='none', edgecolors='magenta', linewidths=1)
    axis.set(xlim=(centre[0] - 48, centre[0] + 48), ylim=(centre[1] + 48, centre[1] - 48),
        title=f'frame {int(point.frame)} / {point.time_video_s:.4f}s')
    axis.axis('off')
for axis in list(axes.flat)[count:]:
    axis.axis('off')
fig.suptitle(args.label + ': actual recovered pairs / cyan front, magenta rear / decoded ArUco 2,3')
fig.savefig(out / 'recovered_pairs_contact_sheet.png', dpi=140)
plt.close(fig)
cap.release()

# 走行停止確認時刻から同一の固定窓を取る。窓を選択して散乱を小さくしない。
confirmation = trial['recording_stop']['confirmation_camera_seconds']
window = (confirmation + .10, confirmation + .54)
pose, _, _, _ = read_video_pose(Path(trial['video_file']), geometry['front_colour'],
    marker_reference=reference, video_window=window, frame_stride=1,
    marker_detection=geometry['marker_detection'], rear_colour=geometry['rear_colour'],
    nominal_marker_span_mm=geometry['nominal_marker_span_mm'],
    marker_span_limits_mm=geometry['marker_span_limits_mm'])
pose.to_csv(out / 'confirmed_stop_pose.csv', index=False)
valid = pose.marker_detected_raw.to_numpy(bool) & pose.calibration_valid.to_numpy(bool)
matrix = np.linalg.inv(board_to_pose_transform(reference))
front = transform_points(pose[['blue_x_mm', 'blue_y_mm']].to_numpy(), matrix)
rear = transform_points(pose[['yellow_x_mm', 'yellow_y_mm']].to_numpy(), matrix)
relative = relative_marker_midpoint(front, rear, 1500., 10., 1., valid)
xy = relative['displacement_xy_mm'][valid]
scatter = float(np.sqrt(np.sum(np.var(xy, axis=0)))) if len(xy) else None
span = float(np.ptp(pose.time_video_s[valid])) if valid.any() else 0.
maximum = float(np.linalg.norm(xy - np.median(xy, axis=0), axis=1).max()) if len(xy) else None
result = dict(source_trial=str(run / 'trial.json'), video_window_s=list(window), frame_stride=1,
    source_of_window='recording_stop.confirmation_camera_seconds + [0.10,0.54]',
    observed=int(valid.sum()), frames=len(pose), observation_fraction=float(valid.mean()),
    calibration_fraction=float(pose.calibration_valid.mean()), observed_span_s=span,
    scatter_rms_mm=scatter, maximum_distance_from_median_mm=maximum,
    recovered_actual_pairs=count, per_frame_calibration=pose.attrs['marker_calibration'],
    passed=bool(valid.sum() >= 12 and valid.mean() >= .95 and span >= .25
        and scatter is not None and scatter <= 1. and maximum <= 3.),
    parameter_adoption_allowed=False, robot_connections=0)
with (out / 'confirmed_stop_review.json').open('x', encoding='utf-8') as stream:
    json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
print(json.dumps(result, ensure_ascii=False), flush=True)
