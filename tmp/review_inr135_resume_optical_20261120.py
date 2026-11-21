"""ログ不完全の1回を動画だけで診断する。試行の品質判定・採用条件は変更しない。"""
from pathlib import Path
import hashlib
import json
import sys
import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'tools'), str(ROOT / 'tools/analysis')]
from video_marker_pose import read_video_pose, board_to_pose_transform, transform_points
from pixel_camera_geometry import relative_marker_midpoint

BASE = ROOT / 'tools/logs/pixel_camera/turn1800_A_inr135_20261120_v1'
RUN = BASE / 'c3p5_Ls7p85_01/local_execution/run_A_INR135_1800_c3p5_Ls7p85_req_20261120_001'
OUT = BASE / 'resume_20261120_inr135_v1/optical_diagnostic'
OUT.mkdir(exist_ok=False)
read = lambda p: json.loads(p.read_text(encoding='utf-8-sig'))
trial = read(RUN / 'trial.json')
reference = read(RUN / 'marker_reference_used.json')
geometry = read(RUN / 'body_geometry_used.json')
settings = read(RUN / 'telemetry.settings.json')
assert trial['state'] == 'INCOMPLETE_LOG' and trial['camera_state'] == 'PASSED'
assert settings['motion_name'] == 'in_r135' and settings['preset_speed_mm_s'] == 1800
cv2.setNumThreads(4)
window = (8.3, 11.5)
pose, _, _, _ = read_video_pose(Path(trial['video_file']), 'red', marker_reference=reference,
    video_window=window, frame_stride=1, marker_detection=geometry.get('marker_detection'),
    rear_colour='yellow', nominal_marker_span_mm=geometry.get('nominal_marker_span_mm'),
    marker_span_limits_mm=geometry.get('marker_span_limits_mm'))
pose.to_csv(OUT / 'apparent_marker_observations.csv', index=False)
valid = pose.marker_detected_raw.to_numpy(bool) & pose.calibration_valid.to_numpy(bool)
transform = np.linalg.inv(board_to_pose_transform(reference))
front = transform_points(pose[['blue_x_mm', 'blue_y_mm']].to_numpy(), transform)
rear = transform_points(pose[['yellow_x_mm', 'yellow_y_mm']].to_numpy(), transform)
relative = relative_marker_midpoint(front, rear, 1500., geometry['front_height_mm'], geometry['rear_height_mm'], valid)
xy = relative['displacement_xy_mm']
valid = relative['valid']
t = pose.time_video_s.to_numpy()
before = valid & (t < 8.6)
assert before.sum() >= 12
origin = np.median(xy[before], axis=0)
delta = xy - origin
distance = np.linalg.norm(delta, axis=1)
moving = valid & (distance > 5)
assert moving.any()
onset = float(t[np.flatnonzero(moving)[0]])


def line(points):
    assert len(points) >= 15, len(points)
    direction = np.linalg.svd(points - points.mean(0), full_matrices=False)[2][0]
    if direction @ (points[-1] - points[0]) < 0:
        direction = -direction
    residual = points - points.mean(0)
    span = float(np.ptp(points @ direction))
    assert span > 30, span
    residual -= np.outer(residual @ direction, direction)
    return direction, dict(points=len(points), span_mm=span,
        rms_mm=float(np.sqrt(np.mean(np.sum(residual ** 2, axis=1)))))


entry_mask = valid & (t >= onset) & (distance >= 5) & (distance <= 70)
entry, entry_quality = line(xy[entry_mask])
basis = np.column_stack(([-entry[1], entry[0]], entry))
local = delta @ basis
exit_mask = valid & (t > t[entry_mask].max()) & (local[:, 0] >= 110) & (local[:, 0] <= 175)
exit_direction, exit_quality = line(local[exit_mask])
angle = float(np.degrees(np.arctan2(exit_direction[0], exit_direction[1])) % 360)
normal = np.array([-exit_direction[1], exit_direction[0]])
line_delta_ls = float((np.array([90., 135.]) @ normal - np.median(local[exit_mask] @ normal)) / normal[1])
interval = (t >= onset - .2) & (t <= t[exit_mask].max() + .1)
report = dict(state='OPTICAL_DIAGNOSTIC_ONLY_INCOMPLETE_LOG', trial_state_unchanged=trial['state'],
    source_trial=str(RUN / 'trial.json'), requested_parameters=read(BASE / 'c3p5_Ls7p85_01/candidate.settings.json')['parameters'],
    actual_parameters=settings['parameters'], video_window_s=window, frame_stride=1,
    registration=pose.attrs['marker_calibration'], frames=int(interval.sum()),
    observation_fraction=float(valid[interval].mean()),
    registration_fraction=float(pose.calibration_valid.to_numpy(bool)[interval].mean()),
    observation_definition='actual front/rear pair and decoded registered ArUco in every used frame',
    entry_line=entry_quality, exit_line=exit_quality, exit_path_direction_change_deg=angle,
    exit_path_error_from135_deg=angle-135.,
    diagnostic_delta_Lstart_from_exit_line_mm=line_delta_ls,
    entry_window_s=[float(t[entry_mask].min()), float(t[entry_mask].max())],
    exit_window_s=[float(t[exit_mask].min()), float(t[exit_mask].max())],
    coordinate_basis='height-corrected relative marker midpoint and measured entry direction; no axle calibration',
    target_Lend_right_forward_mm=[90., 135.],
    precise_Lend_measured=False, independent_hardware_time_sync=False,
    parameter_adoption_allowed=False, physical_240fps_verified=False, hardware_operations=0,
    limitations=['1 msログ未取得・連続ログ終端欠損のため通常の試行品質は不合格のまま。',
        '出口方向と出口直線の診断のみ。Lend時刻、FF/EKF同定、再現性・連続接続は未確認。',
        '入口方向に合わせた相対評価であり、盤面固定の接続・開始角は別評価。'],
    sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in
        (RUN / 'trial.json', RUN / 'telemetry.settings.json', RUN / 'marker_reference_used.json',
         RUN / 'body_geometry_used.json', Path(__file__))})
with (OUT / 'report.json').open('x', encoding='utf-8') as f:
    json.dump(report, f, ensure_ascii=False, indent=2, allow_nan=False)
pose.assign(relative_right_mm=local[:, 0], relative_forward_mm=local[:, 1],
            measured_pair_and_registration_valid=valid).to_csv(OUT / 'relative_midpoint.csv', index=False)
plt.rcParams['font.family'] = 'Yu Gothic'
fig, ax = plt.subplots(figsize=(6, 7), layout='constrained')
ax.plot(*np.where(valid[:, None], local, np.nan)[interval].T, label='今回：相対マーカー中点')
ax.scatter(*local[entry_mask].T, s=8, label='入口直線')
ax.scatter(*local[exit_mask].T, s=8, label='出口直線')
ax.scatter(90, 135, marker='x', c='black', label='Lend格子目標')
ax.set(xlabel='入口に対する右 [mm]', ylabel='入口に対する前 [mm]', aspect='equal',
       title=f'右135°入口1800：動画のみの診断\n出口方向 {angle:.3f}° / ログ不完全・採用不可')
ax.grid(alpha=.3)
ax.legend(fontsize=8)
fig.savefig(OUT / 'trajectory.png', dpi=140)
plt.close(fig)
print(json.dumps({k: report[k] for k in ('state', 'observation_fraction', 'registration_fraction',
    'exit_path_direction_change_deg', 'diagnostic_delta_Lstart_from_exit_line_mm')}, ensure_ascii=False), flush=True)
