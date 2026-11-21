"""一回試行の保存動画・CSVを照合し、調整の可否を記録する。"""
import json
import hashlib
from pathlib import Path
import sys

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'tools/analysis'), str(ROOT / 'tools')]
from pixel_camera_tuning_report import summarize
from video_marker_pose import board_to_pose_transform, transform_points

RUN = ROOT / 'tools/logs/pixel_camera/tuning_gui_20261116_041226_ee682b/run_041508_bc4f21'
FULL = RUN / 'offline_review_v1/relative_full'
try:
    summary = summarize(FULL)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
except ValueError as error:
    print('SUMMARY_REJECTED:', error)
data = pd.read_csv(RUN / 'telemetry.csv')
motion = data[(data.time_ms >= 270500) & (data.time_ms <= 273100)]
print(motion[['time_ms', 'ideal.velo', 'ego.velo', 'ideal.rad_velo', 'ego.rad_velo',
              'V_r', 'V_l', 'motion.pattern']].iloc[::20].to_string(index=False))
pose = pd.read_csv(FULL / 'relative_marker_midpoint.csv')
quality = json.loads((FULL / 'report.json').read_text(encoding='utf-8'))
print('window_quality:', json.dumps(quality['required_motion_window_quality']))
print('offset:', quality['log_time_zero_at_video_s'])
for start, end in [(6.7, 8.5), (9.7, 9.9), (10.5, 11.5)]:
    points = pose[(pose.time_video_s >= start) & (pose.time_video_s <= end) & pose.detected]
    print('video_displacement', start, end, points[['center_dx_mm', 'center_dy_mm']].median().to_dict())

steady = pose.detected & pose.time_video_s.between(10.5, 11.5)
basis = np.asarray(summary['trajectory_reference_frame']['camera_plane_to_right_forward_matrix'])
relative = pose[['center_dx_mm', 'center_dy_mm']].to_numpy() @ basis
endpoint = np.median(relative[steady], axis=0)
scatter = float(np.sqrt(np.sum(np.var(relative[steady], axis=0))))
reference = json.loads((RUN / 'marker_reference_used.json').read_text(encoding='utf-8'))
apparent = pd.read_csv(FULL / 'apparent_floor_marker_pose.csv')
inverse = np.linalg.inv(board_to_pose_transform(reference))
front = transform_points(apparent[['blue_x_mm', 'blue_y_mm']].to_numpy(), inverse)
rear = transform_points(apparent[['yellow_x_mm', 'yellow_y_mm']].to_numpy(), inverse)
heading = np.arctan2((front-rear)[:, 1], (front-rear)[:, 0])
baseline = pose.detected & pose.time_video_s.between(7, 8.5)
def circular_mean(values):
    return np.arctan2(np.sin(values).mean(), np.cos(values).mean())
angle_before = circular_mean(heading[baseline])
angle_after = circular_mean(heading[steady])
angle_change = (angle_after-angle_before+np.pi) % (2*np.pi)-np.pi
result = dict(schema_version=1, mouse='A', motion='long_r90', preset_mm_s=1600,
    execution_count=1, execution_resend_count=0, suction_enabled=True, suction_duty=650,
    original_trial=str(RUN / 'trial.json'), offline_full_report=str(FULL / 'report.json'),
    analysis_video_window_pts_s=[6.7, 12.63],
    terminal_window_pts_s=[10.5, 11.5], terminal_window_source='visually stationary in saved video contact sheet',
    terminal_observations=int(steady.sum()), terminal_scatter_rms_mm=scatter,
    terminal_right_forward_mm=endpoint.tolist(), nominal_start_to_stop_right_forward_mm=[270, 270],
    difference_from_nominal_mm=(endpoint-[270, 270]).tolist(),
    apparent_marker_heading_before_image_deg=float(np.degrees(angle_before)),
    apparent_marker_heading_after_image_deg=float(np.degrees(angle_after)),
    apparent_marker_clockwise_turn_deg=float(np.degrees(angle_change)),
    required_motion_calibration_rate=quality['required_motion_window_quality']['calibration_rate'],
    full_window_calibration_rate=quality['quality']['marker_calibration']['calibration_frame_rate'],
    tracking_fit_rmse_mm=quality['trajectory_fit_rmse_mm'],
    log_stop_confirmed_from_original_capture=True, detailed_auto_stop_confirmed=summary['auto_stop_confirmed'],
    capture_complete=True, parameter_adoption_allowed=False, source_params_changed=False,
    analysis_source_sha256={name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest() for name in [
        'tools/analysis/pixel_camera_relative_pose.py', 'tools/analysis/video_marker_pose.py',
        'tools/analysis/pixel_camera_tuning_report.py', 'tmp/finish_turn1600_A_r90_review_20261116.py']},
    power_state='Both OFF confirmed by user', ble_connection='closed',
    conclusion='Single baseline completed. Keep existing Lstart/Lend; video/log mismatch and terminal drift require review.',
    limitations=['Relative marker midpoint; entry direction measured from video; no fit of endpoint or scale.',
        'Nominal endpoint is a placement plan, not a calibrated absolute axle target.',
        'Height is user supplied 1500 mm; absolute axle position and height-corrected yaw are unverified.',
        '5 ms BLE stream, maximum 6 ms gap, 14 turn samples; retained 1 ms log unsupported.',
        'Original quick/full used a too-short window. They are preserved; expanded offline results are separate.',
        'Motion CSV ends before final settling; full telemetry retains final stop. Auto-stop quality remains false.'])
(RUN / 'offline_review_v1/verification_summary.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
fig, axis = plt.subplots(figsize=(6, 6), layout='constrained')
axis.plot(relative[:, 0], relative[:, 1], label='ArUco-referenced marker midpoint', lw=1.5)
axis.scatter(*endpoint, c='green', label='Stationary window 10.5-11.5 s')
axis.scatter(270, 270, marker='x', c='black', label='Nominal placement plan')
axis.set(xlabel='Right displacement [mm]', ylabel='Forward displacement [mm]', title='A long_r90 / 1.6 m/s / suction 650')
axis.axis('equal')
axis.grid(alpha=.25)
axis.legend(fontsize=8)
fig.savefig(RUN / 'offline_review_v1/observed_trajectory.png', dpi=160)
plt.close(fig)
print('VERIFICATION:', json.dumps(result, ensure_ascii=False, indent=2))
