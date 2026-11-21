"""1ms取得失敗を保持し、保存動画と欠落のない5ms配信から相対形状だけを診断する。"""
from pathlib import Path
import argparse
import hashlib
import json
import sys
import cv2
import numpy as np
import pandas as pd
from scipy.integrate import cumulative_trapezoid
from scipy.optimize import minimize_scalar
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'tools'), str(ROOT / 'tools/analysis')]
from pixel_camera_geometry import relative_marker_midpoint
from video_marker_pose import read_video_pose, board_to_pose_transform, transform_points

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--run', type=Path, required=True)
parser.add_argument('--prior', type=Path, required=True)
args = parser.parse_args()
run, prior = args.run.resolve(), args.prior.resolve()
assert run.is_relative_to(ROOT) and prior.is_relative_to(ROOT)
out = run / 'partial_capture_geometry_diagnostic_v1'
out.mkdir(exist_ok=False)
read = lambda p: json.loads(p.read_text(encoding='utf-8-sig'))
trial, settings = read(run / 'trial.json'), read(run / 'motion.settings.json')
old = read(prior / 'report.json')
old_run = Path(old['source_trial']).parent
old_settings = read(old_run / 'motion.settings.json')
keys = ['mouse', 'motion_name', 'preset_speed_mm_s', 'parameters', 'pid', 'feedforward', 'suction', 'turn_count']
assert all(settings[key] == old_settings[key] for key in keys)
assert trial['state'] == 'FAILED' and trial['saved_log']['status'] == 'failed'
assert not (run / 'saved_1ms.csv').exists()
assert trial['camera_state'] == 'PASSED' and trial['camera_run_start_confirmed']
assert trial['execution_write_attempted'] and trial['execution_resend_count'] == 0
assert trial['log_quality']['full_motion_retained'] and trial['log_quality']['lost_frames'] == 0
reference, geometry = read(run / 'marker_reference_used.json'), read(run / 'body_geometry_used.json')
log = pd.read_csv(run / 'motion.csv')
assert np.all(np.diff(log.sequence) == 1) and np.all(np.diff(log.time_ms) > 0)
assert np.diff(log.time_ms).max() <= 6 and log.lost_frames.sum() == 0
assert settings['log']['period_ms'] == 5
window = [trial['video_motion_start_estimate_s'] + .8, trial['video_motion_start_estimate_s'] + 2.8]
cv2.setNumThreads(4)
pose, _, _, _ = read_video_pose(Path(trial['video_file']), geometry['front_colour'],
    marker_reference=reference, video_window=tuple(window), frame_stride=1,
    marker_detection=geometry['marker_detection'], rear_colour=geometry['rear_colour'],
    nominal_marker_span_mm=geometry['nominal_marker_span_mm'], marker_span_limits_mm=geometry['marker_span_limits_mm'],
    progress=lambda stage, count, pts: print(f'{stage}: {count} frames, PTS {pts:.3f} s', flush=True))
calibration = pose.attrs['marker_calibration']
assert calibration['per_frame_registration_required'] and set(calibration['used_marker_ids']) >= {2, 3}
pose.to_csv(out / 'apparent_floor_marker_pose.csv', index=False)
to_board = np.linalg.inv(board_to_pose_transform(reference))
front = transform_points(pose[['blue_x_mm', 'blue_y_mm']].to_numpy(), to_board)
rear = transform_points(pose[['yellow_x_mm', 'yellow_y_mm']].to_numpy(), to_board)
relative = relative_marker_midpoint(front, rear, trial['camera_height_mm'],
    geometry['front_height_mm'], geometry['rear_height_mm'],
    pose.marker_detected_raw.to_numpy(bool) & pose.calibration_valid.to_numpy(bool))
xy, valid = relative['displacement_xy_mm'], relative['valid']
t = pose.time_video_s.to_numpy(float)
baseline = valid & (t <= t[0] + .2)
assert baseline.sum() >= 12
xy -= np.median(xy[baseline], axis=0)
active = valid & (np.nan_to_num(np.linalg.norm(xy, axis=1), nan=0) >= 2)
onset = next(i for i in range(len(t) - 2) if t[i] > t[0] + .2 and active[i:i + 3].all() and t[i + 2] - t[i] < .020)
seed = float(t[onset] - np.sqrt(2 * .002 / 18))
start_ms, stop_ms = [trial['log_quality'][key] for key in ('motion_start_time_ms', 'motion_end_time_ms')]
lt = (log.time_ms.to_numpy(float) - start_ms) / 1000
tick = float(log['irq.period_us'].dropna().median() / 1000)
target_omega = np.abs(log['ideal.rad_velo'].to_numpy())
turn = np.flatnonzero(target_omega > .02 * target_omega.max())
entry_window = [seed + .010, seed + lt[turn[0]] - .008]
entry_mask = valid & (t >= entry_window[0]) & (t <= entry_window[1])


def line(points):
    if len(points) < 15:
        raise ValueError('直線観測15点未満')
    direction = np.linalg.svd(points - points.mean(0), full_matrices=False)[2][0]
    if direction @ (points[-1] - points[0]) < 0:
        direction = -direction
    residual = points - points.mean(0)
    residual -= np.outer(residual @ direction, direction)
    span = float(np.ptp(points @ direction))
    if span < 30:
        raise ValueError('直線の範囲30mm未満')
    return direction, dict(points=len(points), span_mm=span,
        rms_mm=float(np.sqrt(np.mean(np.sum(residual**2, axis=1)))))


entry, entry_quality = line(xy[entry_mask])
stationary = valid & (t >= seed - .18) & (t <= seed - .04)
assert stationary.sum() >= 12
xy = (xy - np.median(xy[stationary], axis=0)) @ np.column_stack(([-entry[1], entry[0]], entry))
# 5ms点から1ms生パルスを補完しない。既存の20点平均速度を、その既知の遅れだけ合わせる。
wheel_average_mm_s = (log['ego.velo'] - .010 * log['ego.accel']).to_numpy() * 1000 / tick
velocity = np.interp(lt + 9.5 * tick / 1000, lt, wheel_average_mm_s)
distance = cumulative_trapezoid(velocity, lt, initial=0)
distance -= np.interp(0, lt, distance)


def clock(mask):
    fit = minimize_scalar(lambda z: np.mean((np.interp(t[mask] - z, lt, distance) - xy[mask, 1])**2),
        bounds=(seed - .025, seed + .025), method='bounded', options={'xatol': 1e-10})
    return float(fit.x), float(np.sqrt(fit.fun))


zero, rms = clock(entry_mask)
middle = np.median(t[entry_mask])
half = [clock(entry_mask & (t <= middle))[0], clock(entry_mask & (t > middle))[0]]
pattern = np.flatnonzero(log['motion.pattern'].to_numpy() == 5)
assert len(pattern) and pattern[-1] + 1 < len(log)
lend_log = float((lt[pattern[-1]] + lt[pattern[-1] + 1]) / 2)
phase_half_gap_ms = float((lt[pattern[-1] + 1] - lt[pattern[-1]]) * 500)
stop = zero + (stop_ms - start_ms) / 1000
exit_window = [zero + lend_log + .005, stop + .010]
exit_mask = valid & (t >= exit_window[0]) & (t <= exit_window[1])
leaving, exit_quality = line(xy[exit_mask])
angle = float(np.degrees(np.arctan2(leaving[0], leaving[1])) % 360)
centre = xy[exit_mask].mean(0)
extent = [float(xy[exit_mask, 1].min()), float(xy[exit_mask, 1].max())]
width = float(centre[0] + (45 - centre[1]) * leaving[0] / leaving[1])
required = (t >= zero - .2) & (t <= stop + .1)
moving = (t >= zero) & (t <= stop)
steady = valid & (t >= stop + .2) & (t <= stop + .6)
scatter = float(np.sqrt(np.var(xy[steady], axis=0).sum()))
checks = dict(required_observations=bool(valid[required].mean() >= .95),
    motion_observations=bool(valid[moving].mean() >= .95),
    whole_window=bool(t[0] <= zero - .2 and t[-1] >= stop + .6),
    clock_rms=bool(rms <= 1.5), half_clock_spread=bool(abs(half[0] - half[1]) <= .002),
    clock_not_boundary=bool(abs(zero - seed) < .024),
    entry_line=bool(entry_quality['rms_mm'] <= .75), exit_line=bool(exit_quality['rms_mm'] <= .75),
    width45_inside_observations=bool(extent[0] <= 45 <= extent[1]),
    video_stationary=bool(steady.sum() >= 12 and scatter <= 1.0))
result = dict(state='PARTIAL_CAPTURE_VIDEO_DIAGNOSTIC_ONLY', original_trial_state=trial['state'],
    original_error=trial['error'], original_state_preserved=True, saved_1ms_available=False,
    source_trial=str(run / 'trial.json'), comparison_review=str(prior / 'report.json'),
    actual_settings_identical=True, parameters=settings['parameters'], checks=checks,
    diagnostic_geometry_quality_passed=all(checks.values()),
    path_angle_deg=angle, return_spacing_at_forward45_mm=width,
    previous_path_angle_deg=old['path_angle_deg'], previous_width45_mm=old['return_spacing_at_forward45_mm'],
    delta_angle_deg=angle - old['path_angle_deg'], delta_width_mm=width - old['return_spacing_at_forward45_mm'],
    timing=dict(method='5ms MCU timestamps and delay-corrected wheel-average distance; entry only',
        video_motion_start_s=zero, seed_video_s=seed, incoming_rms_mm=rms,
        half_window_spread_ms=abs(half[0] - half[1]) * 1000, phase_half_gap_ms=phase_half_gap_ms,
        independent_hardware_sync=False, raw_1ms_pulses_reconstructed=False,
        causal_online_result=False, scale_fitted=False, exit_or_turn_used_for_clock=False),
    entry_window_s=entry_window, exit_window_s=exit_window, entry_line=entry_quality, exit_line=exit_quality,
    required_observation_rate=float(valid[required].mean()), motion_observation_rate=float(valid[moving].mean()),
    marker_calibration=calibration, frame_stride=1, analyzed_video_window_s=window,
    stationary_scatter_rms_mm=scatter, full_log_max_gap_ms=float(np.diff(log.time_ms).max()),
    minimum_motion_battery_v=float(log.Battery[(np.abs(log['ideal.velo']) > .01) | (target_omega > .01)].min()),
    prior_minimum_battery_v=float(pd.read_csv(old_run / 'saved_1ms.csv').Battery.min()),
    parameter_adoption_allowed=False, precise_lend_identification_allowed=False, fixed_board_accepted=False,
    limitations=['1ms保存ログの取得失敗を保持。5ms点の位相境界には約±2.5msの標本化差がある。',
        '前回は1ms生パルス時計、今回は5ms平均速度時計。時刻・Lendの厳密な同条件比較ではない。',
        '入口を合わせた相対マーカー中点の診断。車軸絶対位置・機軸yaw・最短接続の採用ではない。'])
sources = [run / 'trial.json', run / 'motion.csv', run / 'motion.settings.json',
    run / 'marker_reference_used.json', run / 'body_geometry_used.json', Path(trial['capture_json']),
    Path(trial['video_file']), prior / 'report.json', Path(__file__)]
result['source_sha256'] = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
(out / 'report.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
pd.DataFrame(dict(time_video_s=t, detected=valid, right_mm=xy[:, 0], forward_mm=xy[:, 1])).to_csv(out / 'relative_trajectory.csv', index=False)
plt.rcParams['font.family'] = 'Yu Gothic'
fig, ax = plt.subplots(figsize=(6.5, 7.5), layout='constrained')
old_curve = pd.read_csv(prior / 'relative_trajectory.csv')
old_xy = old_curve[['right_mm', 'forward_mm']].to_numpy()
old_xy[~old_curve.detected.to_numpy(bool)] = np.nan
ax.plot(old_xy[:, 0], old_xy[:, 1], label='前の走行：1ms時計')
ax.plot(xy[valid, 0], xy[valid, 1], label='同一設定の反復：5ms時計・参考')
ax.axvline(90, ls='--', color='.5', label='名目の返り幅90mm')
ax.set(xlabel='実測入口に対する右 [mm]', ylabel='実測入口に対する前 [mm]',
    title='A / 1800 R180 / 同一設定48mm・＋4.25°', aspect='equal')
ax.grid(); ax.legend(fontsize=9)
fig.savefig(out / 'comparison.png', dpi=140); plt.close(fig)
print(json.dumps(dict(output=str(out), diagnostic_quality=all(checks.values()), angle_deg=angle,
    width45_mm=width, delta_angle_deg=result['delta_angle_deg'], delta_width_mm=result['delta_width_mm'], checks=checks), ensure_ascii=False), flush=True)
