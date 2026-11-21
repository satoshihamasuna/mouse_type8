"""保存済みの全フレーム中点を、入口車輪距離だけの時計で評価する。実機接続なし。"""
from pathlib import Path
import argparse
import hashlib
import json
import sys
import numpy as np
import pandas as pd
from scipy.optimize import minimize_scalar
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--run', type=Path, required=True)
parser.add_argument('--front', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--pose-output', type=Path, help='原本を保持して再解析した全フレームposeのフォルダ')
args = parser.parse_args()
run, front_path, out = args.run.resolve(), args.front.resolve(), args.output.resolve()
assert all(p.is_relative_to(ROOT) for p in (run, front_path, out))
assert not out.exists()
read = lambda p: json.loads(p.read_text(encoding='utf-8-sig'))
trial, settings, front = read(run / 'trial.json'), read(run / 'saved_1ms.settings.json'), read(front_path)
assert settings['mouse'] == 'A' and settings['motion_name'] == 'long_r180'
assert settings['turn_count'] == 1 and settings['preset_speed_mm_s'] == 1800
assert Path(front['source_trial']).resolve() == run / 'trial.json'
assert front['clock']['passed'] and front['actual_registered_aruco_per_frame']
pose_dir = args.pose_output.resolve() if args.pose_output else run / 'full'
assert pose_dir.is_relative_to(ROOT)
report = read(pose_dir / 'report.json')
summary_path = pose_dir / 'summary.json'
# 標準解析が必要区間の品質で停止した場合、summaryは生成されない。
# 未生成の原本を作らず、欠測と元の不合格statusをそのまま残す。
summary = read(summary_path) if summary_path.is_file() else {}
pose = pd.read_csv(pose_dir / 'apparent_floor_marker_pose.csv')
relative = pd.read_csv(pose_dir / 'relative_marker_midpoint.csv')
assert np.array_equal(pose.frame, relative.frame)
calibration = report['quality']['marker_calibration']
assert calibration['per_frame_registration_required'] and set(calibration['used_marker_ids']) >= {2, 3}
assert report['frame_stride'] == 1
t = relative.time_video_s.to_numpy(float)
v = relative.detected.to_numpy(bool) & pose.calibration_valid.to_numpy(bool)
xy = relative[['center_dx_mm', 'center_dy_mm']].to_numpy(float)
raw, live = pd.read_csv(run / 'saved_1ms.csv'), pd.read_csv(run / 'telemetry.csv')
assert np.array_equal(raw.cnt, np.arange(len(raw)))
signature = ['ideal.velo', 'ego.velo', 'ideal.rad_velo', 'ego.rad_velo', 'V_r', 'V_l']
matches = raw.merge(live[['time_ms'] + signature], on=signature)
tick, origin_ms = np.polyfit(matches.cnt, matches.time_ms, 1)
saved_t = raw.cnt.to_numpy(float) * tick / 1000
distance = np.cumsum((raw['Encoder_GetProperty_Right().sp_pulse'] -
    raw['Encoder_GetProperty_Left().sp_pulse']).to_numpy() * 15.02 * np.pi / 172 / 2)
start_ms, end_ms = (trial['log_quality'][key] for key in ('motion_start_time_ms', 'motion_end_time_ms'))
delta_start, duration = (origin_ms - start_ms) / 1000, (end_ms - start_ms) / 1000


def line(mask):
    points = xy[mask]
    if len(points) < 15:
        raise ValueError('直線観測15点未満')
    direction = np.linalg.svd(points - points.mean(0), full_matrices=False)[2][0]
    if direction @ (points[-1] - points[0]) < 0:
        direction = -direction
    span = float(np.ptp(points @ direction))
    residual = points - points.mean(0)
    residual -= np.outer(residual @ direction, direction)
    if span < 30:
        raise ValueError('直線観測範囲30mm未満')
    return direction, dict(points=len(points), span_mm=span,
        rms_mm=float(np.sqrt(np.mean(np.sum(residual**2, axis=1)))))


entry_window = front['entry_window_pts_s']
entry_mask = v & (t >= entry_window[0]) & (t <= entry_window[1])
entry, eq = line(entry_mask)
basis = np.column_stack(([-entry[1], entry[0]], entry))
seed = front['clock']['saved_zero_video_s']
stationary = v & (t >= seed - delta_start - .18) & (t <= seed - delta_start - .04)
assert stationary.sum() >= 12
xy = (xy - np.median(xy[stationary], axis=0)) @ basis


def fit(mask):
    value = minimize_scalar(lambda z: np.mean((np.interp(t[mask] - z, saved_t, distance) - xy[mask, 1])**2),
        bounds=(seed - .025, seed + .025), method='bounded', options={'xatol': 1e-10})
    return float(value.x), float(np.sqrt(value.fun))


zero, clock_rms = fit(entry_mask)
half = np.median(t[entry_mask])
half_zeros = [fit(entry_mask & (t <= half))[0], fit(entry_mask & (t > half))[0]]
start, stop, lend = zero - delta_start, zero - delta_start + duration, zero + saved_t[-1]
# 短い90mm減速直線の全体を使用。退出後の静止点を多数追加して向きを決めない。
exit_window = [lend + .005, stop + .010]
exit_mask = v & (t >= exit_window[0]) & (t <= exit_window[1])
leaving, xq = line(exit_mask)
angle = float(np.degrees(np.arctan2(leaving[0], leaving[1])) % 360)
exit_center = xy[exit_mask].mean(0)
exit_extent = [float(xy[exit_mask, 1].min()), float(xy[exit_mask, 1].max())]
width45 = float(exit_center[0] + (45 - exit_center[1]) * leaving[0] / leaving[1])


def point(stamp):
    times = t[v]
    idx = np.searchsorted(times, stamp)
    assert 0 < idx < len(times) and times[idx] - times[idx - 1] <= .020
    return [float(np.interp(stamp, times, xy[v, k])) for k in (0, 1)]


required = (t >= start - .2) & (t <= stop + .1)
motion = (t >= start) & (t <= stop)
steady_window = [trial['recording_stop']['confirmation_camera_seconds'] + .1,
    trial['recording_stop']['decision_camera_seconds'] - .1]
steady = v & (t >= steady_window[0]) & (t <= steady_window[1])
terminal = np.median(xy[steady], axis=0)
scatter = float(np.sqrt(np.sum(np.var(xy[steady], axis=0))))
checks = dict(required_observation_rate=bool(v[required].mean() >= .95),
    motion_observation_rate=bool(v[motion].mean() >= .95),
    incoming_clock_rms=bool(clock_rms <= 1.5),
    half_window_clock_spread=bool(abs(half_zeros[0] - half_zeros[1]) <= .002),
    clock_search_not_boundary=bool(abs(zero - seed) < .024),
    entry_line_rms=bool(eq['rms_mm'] <= .75), exit_line_rms=bool(xq['rms_mm'] <= .75),
    width45_inside_observation=bool(exit_extent[0] <= 45 <= exit_extent[1]),
    stationary_video=bool(steady.sum() >= 12 and scatter <= 1.0),
    full_window_covered=bool(t[0] <= start - .2 and t[-1] >= stop + .1))
result = dict(state='INDEPENDENT_ENTRY_CLOCK_RELATIVE_MIDPOINT_REVIEW', mouse='A', motion='long_r180',
    source_trial=str(run / 'trial.json'), source_front_clock=str(front_path), source_pose_output=str(pose_dir),
    parameters=settings['parameters'], checks=checks, relative_geometry_eligible=all(checks.values()),
    timing=dict(saved_zero_video_s=zero, entry_only=True, scale_fitted=False,
        exit_or_turn_used_for_clock=False, incoming_rms_mm=clock_rms,
        half_window_spread_ms=abs(half_zeros[0] - half_zeros[1]) * 1000,
        front_clock_difference_ms=(zero - seed) * 1000, measured_tick_ms=float(tick), independent_hardware_sync=False),
    entry_window_s=entry_window, exit_window_s=exit_window, entry_line=eq, exit_line=xq,
    required_window_s=[start - .2, stop + .1], required_observation_rate=float(v[required].mean()),
    motion_observation_rate=float(v[motion].mean()), marker_calibration=calibration,
    path_angle_deg=angle, path_error_from180_deg=angle - 180,
    return_spacing_at_forward45_mm=width45, return_spacing_error_from90_mm=width45 - 90,
    exit_forward_extent_mm=exit_extent, lend_video_s=lend, lend_right_forward_mm=point(lend),
    lend_sensitivity_minus1ms_mm=point(lend - .001), lend_sensitivity_plus1ms_mm=point(lend + .001),
    forward_apex_mm=float(xy[v & motion, 1].max()),
    stationary_window_s=steady_window, stationary_scatter_rms_mm=scatter,
    terminal_right_forward_mm=terminal.tolist(), original_auto_stop_confirmed=bool(summary.get('auto_stop_confirmed', False)),
    original_summary=summary if summary_path.is_file() else None, original_summary_exists=summary_path.is_file(),
    original_full_status=report.get('status'), original_trajectory_fit_rmse_mm=report['trajectory_fit_rmse_mm'],
    original_failures_preserved=True, parameter_adoption_allowed=False, fixed_board_accepted=False,
    scope='実測入口方向と静止マーカー中点を原点とする相対形状。絶対車軸位置・body yaw・盤面固定接続の合格ではない。',
    source_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in
        [run / 'trial.json', run / 'saved_1ms.csv', run / 'saved_1ms.settings.json',
         pose_dir / 'report.json', pose_dir / 'summary.json', pose_dir / 'relative_marker_midpoint.csv',
         pose_dir / 'apparent_floor_marker_pose.csv', front_path, Path(__file__)] if p.is_file()})
out.mkdir()
(out / 'report.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
pd.DataFrame(dict(time_video_s=t, detected=v, right_mm=xy[:, 0], forward_mm=xy[:, 1])).to_csv(out / 'relative_trajectory.csv', index=False)
plt.rcParams['font.family'] = 'Yu Gothic'
fig, ax = plt.subplots(figsize=(6, 7), layout='constrained')
ax.plot(xy[v, 0], xy[v, 1], label='実測マーカー中点')
ax.axvline(90, color='.5', ls='--', label='目標の返り幅90 mm')
ax.scatter(*point(lend), label='入口時計で照合したLend', c='red')
ax.scatter(90, 90, marker='x', c='black', label='名目Lend (90,90) mm')
ax.set(xlabel='実測入口に対する右 [mm]', ylabel='実測入口に対する前 [mm]', aspect='equal',
    title=f"A / R180 / 1800 / 補正＋{settings['parameters']['correction']}°")
ax.grid(); ax.legend(fontsize=8)
fig.savefig(out / 'relative_trajectory.png', dpi=140); plt.close(fig)
print(json.dumps(dict(output=str(out), eligible=result['relative_geometry_eligible'],
    angle_deg=angle, width45_mm=width45, lend=point(lend), checks=checks), ensure_ascii=False))
