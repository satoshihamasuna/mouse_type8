"""保存済み右90°の実時間・速度基準・停止点を照合する。実機操作なし。"""
from pathlib import Path
import hashlib
import json
import sys

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/analysis'))
from video_marker_pose import board_to_pose_transform, transform_points, fit_time_and_pose

BASE = ROOT / 'tools/logs/pixel_camera/turn1600_A_r90_20261118_frame18_v1'
RUN = BASE / 'cli_session03/run_A_R90_1600_baseline_20261118_001'
OUT = RUN / 'offline_diagnostics_v1'
OUT.mkdir(exist_ok=False)
trial = json.loads((RUN / 'trial.json').read_text(encoding='utf-8'))
summary = json.loads((RUN / 'full/summary.json').read_text(encoding='utf-8'))
report = json.loads((RUN / 'full/report.json').read_text(encoding='utf-8'))
retained = pd.read_csv(RUN / 'saved_1ms.csv')
stream = pd.read_csv(RUN / 'telemetry.csv')
signature = ['ideal.velo', 'ego.velo', 'ideal.rad_velo', 'ego.rad_velo', 'V_r', 'V_l']
matches = retained.merge(stream[['time_ms'] + signature], on=signature, how='inner')
period_ms, first_mcu_ms = np.polyfit(matches.cnt, matches.time_ms, 1)
residual_ms = matches.time_ms - (period_ms * matches.cnt + first_mcu_ms)
matches.to_csv(OUT / 'retained_live_matches.csv', index=False)

reference = json.loads((RUN / 'marker_reference_used.json').read_text(encoding='utf-8'))
relative = pd.read_csv(RUN / 'full/relative_marker_midpoint.csv')
pose = pd.read_csv(RUN / 'full/apparent_floor_marker_pose.csv')
inverse = np.linalg.inv(board_to_pose_transform(reference))
front = transform_points(pose[['blue_x_mm', 'blue_y_mm']].to_numpy(), inverse)
rear = transform_points(pose[['yellow_x_mm', 'yellow_y_mm']].to_numpy(), inverse)
heading = np.arctan2((front - rear)[:, 1], (front - rear)[:, 0])
center = (front + rear) / 2
valid = relative.detected.to_numpy(bool)
steady = valid & pose.time_video_s.between(12.4, 13.3)
before = valid & pose.time_video_s.between(8.4, 9.7)
basis = np.asarray(summary['trajectory_reference_frame']['camera_plane_to_right_forward_matrix'])
xy = relative[['center_dx_mm', 'center_dy_mm']].to_numpy() @ basis
endpoint = np.median(xy[steady], axis=0)
scatter = float(np.sqrt(np.sum(np.var(xy[steady], axis=0))))
def circular_mean(angle):
    return np.arctan2(np.sin(angle).mean(), np.cos(angle).mean())
angle_change = (circular_mean(heading[steady]) - circular_mean(heading[before]) + np.pi) % (2 * np.pi) - np.pi

# 同一の全フレーム観測へ、異なる速度定義の積分を照合する。
# 原点/回転/同期のみ剛体照合し、距離スケールは調整しない。
alignment = pose.copy()
board_to_legacy = board_to_pose_transform(reference)[:2, :2]
legacy_xy = relative[['center_dx_mm', 'center_dy_mm']].to_numpy() @ board_to_legacy.T
alignment['center_x_mm'] = pd.Series(legacy_xy[:, 0]).interpolate(limit_direction='both')
alignment['center_y_mm'] = pd.Series(legacy_xy[:, 1]).interpolate(limit_direction='both')
filled = alignment[['center_x_mm', 'center_y_mm']].to_numpy()
alignment['speed_playback_mm_s'] = np.linalg.norm(np.gradient(filled, pose.time_video_s, axis=0), axis=1)
alignment['marker_detected_raw'] = valid
live = stream[stream.time_ms.between(671200, 675300)].copy()
t = live.time_ms.to_numpy(float) / 1000
time = t - t[0]
omega = live['ego.rad_velo'].to_numpy(float)
encoder_average = (live['ego.velo'] - .010 * live['ego.accel']).to_numpy() * 1000 / period_ms
nominal_fused = live['ego.velo'].to_numpy() * 1000
advanced_encoder = np.interp(t + 9.5 * period_ms / 1000, t, encoder_average)
required = ((trial['log_quality']['motion_start_time_ms'] - live.time_ms.iloc[0]) / 1000 - .2,
            (trial['log_quality']['motion_end_time_ms'] - live.time_ms.iloc[0]) / 1000 + .1)
diagnostics = {}
fig, axes = plt.subplots(1, 2, figsize=(12, 5), layout='constrained')
axes[0].plot(relative.center_dx_mm[valid], relative.center_dy_mm[valid], color='black', label='Video marker midpoint')
for label, velocity in [('fused_speed', nominal_fused), ('encoder_average', encoder_average), ('encoder_average_advanced', advanced_encoder)]:
    theta = np.r_[0, np.cumsum(.5 * (omega[:-1] + omega[1:]) * np.diff(t))]
    theta_mid = .5 * (theta[:-1] + theta[1:])
    speed_mid = .5 * (velocity[:-1] + velocity[1:])
    x = np.r_[0, np.cumsum(-speed_mid * np.sin(theta_mid) * np.diff(t))]
    y = np.r_[0, np.cumsum(speed_mid * np.cos(theta_mid) * np.diff(t))]
    log = pd.DataFrame(dict(time_log_s=time, x_local_mm=x, y_local_mm=y,
        heading_local_rad=theta, speed_log_mm_s=velocity, omega_log_rad_s=omega))
    comparison, _, offset, rotation, translation, rms = fit_time_and_pose(alignment, log, 1., required)
    for columns in (['center_x_mm', 'center_y_mm'], ['log_x_mm', 'log_y_mm']):
        comparison[columns] = comparison[columns].to_numpy() @ np.linalg.inv(board_to_legacy).T
    comparison.to_csv(OUT / (label + '_comparison.csv'), index=False)
    motion = comparison.time_log_s.between(required[0] + .2, required[1] - .1)
    diagnostics[label] = dict(trajectory_fit_rmse_mm=float(rms),
        commanded_motion_rmse_mm=float(np.sqrt(np.mean(comparison.loc[motion, 'position_error_mm'] ** 2))),
        video_time_offset_s=float(offset), scale_fitted=False,
        rotation_and_translation_fitted=True,
        note='診断用。全履歴の速度定義を比較。車軸の絶対位置・パラメータ採用の根拠にはしない。')
    axes[0].plot(comparison.log_x_mm, comparison.log_y_mm, label=label)
    axes[1].plot(comparison.time_log_s, comparison.position_error_mm, label=label)
axes[0].set(xlabel='Board x displacement [mm]', ylabel='Board y displacement [mm]')
axes[0].axis('equal')
axes[1].set(xlabel='Log elapsed time [s]', ylabel='Rigid-fit position difference [mm]')
for ax in axes:
    ax.grid(alpha=.25)
    ax.legend(fontsize=8)
fig.savefig(OUT / 'speed_definition_comparison.png', dpi=150)
plt.close(fig)

tracking = json.loads((RUN / 'tracking_1ms_review_v1/verification.json').read_text(encoding='utf-8'))
result = dict(mouse='A', motion='long_r90', preset_mm_s=1600, execution_count=1,
    execution_resend_count=0, source_trial=str(RUN / 'trial.json'),
    declared_log_period_ms=1, matched_live_rows=len(matches), measured_log_period_ms=float(period_ms),
    measured_period_method='Exact six-field row matches between retained log and MCU-timestamped live stream; linear fit.',
    measured_period_fit_rmse_ms=float(np.sqrt(np.mean(residual_ms ** 2))),
    irq_period_mean_us=float(retained['irq.period_us'].mean()),
    gyro_clockwise_turn_profile_measured_time_deg=tracking['gyro_clockwise_turn_profile_deg'] * float(period_ms),
    gyro_clockwise_through_lend_measured_time_deg=tracking['gyro_clockwise_through_lend_deg'] * float(period_ms),
    nominal_1ms_tracking_review=str(RUN / 'tracking_1ms_review_v1/verification.json'),
    final_stationary_window_video_pts_s=[12.4, 13.3], stationary_observations=int(steady.sum()),
    stationary_scatter_rms_mm=scatter, final_relative_right_forward_mm=endpoint.tolist(),
    nominal_placement_plan_right_forward_mm=[270, 270],
    difference_from_nominal_plan_mm=(endpoint - [270, 270]).tolist(),
    final_apparent_midpoint_board_mm=np.median(center[steady], axis=0).tolist(),
    apparent_marker_clockwise_rotation_deg=float(np.degrees(angle_change)),
    apparent_heading_before_image_deg=float(np.degrees(circular_mean(heading[before]))),
    apparent_heading_final_image_deg=float(np.degrees(circular_mean(heading[steady]))),
    original_auto_stop_confirmed=False, original_auto_stop_summary_preserved=True,
    capture_stop_evidence=trial['recording_stop']['log_stop_evidence'],
    full_motion_observation_rate=report['required_motion_window_quality']['observation_rate'],
    full_motion_calibration_rate=report['required_motion_window_quality']['calibration_rate'],
    diagnostics=diagnostics,
    operator_contact='No hole/wall contact confirmed by user', power_state='Both OFF confirmed',
    ble_session='closed', execution_pending=False, parameter_adoption_allowed=False,
    limitations=['Marker midpoint relative displacement; body-yaw height correction and camera foot are unverified.',
        'The nominal endpoint is a placement plan, not a calibrated axle target.',
        'Original full report fit exceeds8mm; no Lstart/Lend adoption from this trial.',
        'Encoder-average speed is reconstructed from current source formula; flashed ELF identity is not verified.',
        'Period estimate uses MCU timestamps; independent hardware clock accuracy is unverified.'],
    source_sha256={str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest() for path in
        [RUN / 'saved_1ms.csv', RUN / 'telemetry.csv', RUN / 'full/report.json', ROOT / 'Module/Src/interrupt.cpp', ROOT / 'Params/mouse_config.h', Path(__file__)]})
(OUT / 'review.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), flush=True)
