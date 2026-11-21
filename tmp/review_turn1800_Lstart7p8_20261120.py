"""同じ検出条件の2動画を、実時間とエンコーダ基準で比較する。実機操作なし。"""
from pathlib import Path
import json
import hashlib
import sys

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tools/logs/pixel_camera/turn1800_A_r90_20261119_v1'
OUT = BASE / 'comparison_Lstart7p8_v1'
OUT.mkdir(exist_ok=False)
sys.path.insert(0, str(ROOT / 'tools/analysis'))
from video_marker_pose import (board_to_pose_transform, transform_points,
                              fit_time_and_pose, rigid_fit)


def line_fit(points):
    if len(points) < 15:
        raise ValueError('直線を評価する観測点が不足しています')
    direction = np.linalg.svd(points - points.mean(axis=0), full_matrices=False)[2][0]
    if direction @ (points[-1] - points[0]) < 0:
        direction = -direction
    span = float(np.ptp(points @ direction))
    if span < 30:
        raise ValueError('直線の観測範囲が30mm未満です')
    error = points - points.mean(axis=0)
    error -= np.outer(error @ direction, direction)
    return direction, dict(observations=len(points), span_mm=span,
        rms_mm=float(np.sqrt(np.mean(np.sum(error ** 2, axis=1)))))


def analyze(label, relative_run):
    run = BASE / relative_run
    folder = run / 'full'
    trial = json.loads((run / 'trial.json').read_text(encoding='utf-8'))
    report = json.loads((folder / 'report.json').read_text(encoding='utf-8'))
    retained = pd.read_csv(run / 'saved_1ms.csv')
    stream = pd.read_csv(run / 'telemetry.csv')
    signature = ['ideal.velo', 'ego.velo', 'ideal.rad_velo', 'ego.rad_velo', 'V_r', 'V_l']
    matches = retained.merge(stream[['time_ms'] + signature], on=signature, how='inner')
    period, origin = np.polyfit(matches.cnt, matches.time_ms, 1)
    mcu_start = trial['log_quality']['motion_start_time_ms']
    mcu_stop = trial['log_quality']['motion_end_time_ms']
    mcu_lend = origin + period * (len(retained) - 1)
    mcu_pre_end = origin + period * np.flatnonzero(retained['motion.pattern'] == 3)[0]
    stop = trial['recording_stop']
    live = stream[stream.time_ms.between(mcu_start - 500,
        stop['log_stop_evidence']['stationary_start_time_ms'] + 1500)].copy()
    t = live.time_ms.to_numpy(float) / 1000
    time = t - t[0]
    omega = live['ego.rad_velo'].to_numpy(float)
    encoder_average = (live['ego.velo'] - .010 * live['ego.accel']).to_numpy() * 1000 / period
    velocity = np.interp(t + 9.5 * period / 1000, t, encoder_average)
    theta = np.r_[0, np.cumsum(.5 * (omega[:-1] + omega[1:]) * np.diff(t))]
    middle = .5 * (theta[:-1] + theta[1:])
    speed = .5 * (velocity[:-1] + velocity[1:])
    x = np.r_[0, np.cumsum(-speed * np.sin(middle) * np.diff(t))]
    y = np.r_[0, np.cumsum(speed * np.cos(middle) * np.diff(t))]
    log = pd.DataFrame(dict(time_log_s=time, x_local_mm=x, y_local_mm=y,
        heading_local_rad=theta, speed_log_mm_s=velocity, omega_log_rad_s=omega))
    reference = json.loads((run / 'marker_reference_used.json').read_text(encoding='utf-8'))
    pose = pd.read_csv(folder / 'apparent_floor_marker_pose.csv')
    relative = pd.read_csv(folder / 'relative_marker_midpoint.csv')
    valid = relative.detected.to_numpy(bool)
    xy = relative[['center_dx_mm', 'center_dy_mm']].to_numpy()
    board_to_legacy = board_to_pose_transform(reference)[:2, :2]
    alignment = pose.copy()
    legacy_xy = xy @ board_to_legacy.T
    alignment['center_x_mm'] = pd.Series(legacy_xy[:, 0]).interpolate(limit_direction='both')
    alignment['center_y_mm'] = pd.Series(legacy_xy[:, 1]).interpolate(limit_direction='both')
    filled = alignment[['center_x_mm', 'center_y_mm']].to_numpy()
    alignment['speed_playback_mm_s'] = np.linalg.norm(np.gradient(filled, pose.time_video_s, axis=0), axis=1)
    alignment['marker_detected_raw'] = valid
    required = ((mcu_start - live.time_ms.iloc[0]) / 1000 - .2,
                (mcu_stop - live.time_ms.iloc[0]) / 1000 + .1)
    comparison, _, coarse, rotation, translation, _ = fit_time_and_pose(alignment, log, 1., required)
    video_t = pose.time_video_s.to_numpy()
    best = None
    # 元の粗い501点探索を局所的に細分化。距離スケールは調整しない。
    for offset in np.linspace(coarse - .012, coarse + .012, 161):
        mapped = video_t - offset
        keep = valid & (mapped >= time[0]) & (mapped <= time[-1])
        source = np.column_stack([np.interp(mapped[keep], time, axis) for axis in (x, y)])
        fit_rotation, fit_translation, rms = rigid_fit(source, legacy_xy[keep])
        if best is None or rms < best[0]:
            best = (rms, offset, fit_rotation, fit_translation, keep, mapped)
    rms, offset, rotation, translation, keep, mapped = best
    fitted_source = np.column_stack([np.interp(mapped[keep], time, axis) for axis in (x, y)])
    fitted = (fitted_source @ rotation + translation) @ np.linalg.inv(board_to_legacy).T
    errors = np.linalg.norm(fitted - xy[keep], axis=1)
    selected_time = mapped[keep]
    active = (selected_time >= required[0] + .2) & (selected_time <= required[1] - .1)
    motion_rms = float(np.sqrt(np.mean(errors[active] ** 2)))
    pd.DataFrame(dict(time_video_s=video_t[keep], time_log_s=selected_time,
        log_x_mm=fitted[:, 0], log_y_mm=fitted[:, 1], error_mm=errors)).to_csv(
            OUT / (label + '_encoder_alignment.csv'), index=False)

    def pts(mcu):
        return float(offset + (mcu - live.time_ms.iloc[0]) / 1000)
    entry_window = [pts(mcu_start + 70), pts(mcu_pre_end - 8)]
    exit_window = [pts(mcu_lend + 20), pts(mcu_stop - 20)]
    entry_mask = valid & (video_t >= entry_window[0]) & (video_t <= entry_window[1])
    exit_mask = valid & (video_t >= exit_window[0]) & (video_t <= exit_window[1])
    entry, entry_quality = line_fit(xy[entry_mask])
    exit_direction, exit_quality = line_fit(xy[exit_mask])
    rotation_deg = float(np.degrees((np.arctan2(exit_direction[1], exit_direction[0])
        - np.arctan2(entry[1], entry[0]) + np.pi) % (2 * np.pi) - np.pi))
    basis = np.column_stack([[-entry[1], entry[0]], entry])
    before_window = [trial['video_motion_start_estimate_s'] - .6,
                     trial['video_motion_start_estimate_s'] + .1]
    before = valid & (video_t >= before_window[0]) & (video_t <= before_window[1])
    origin_xy = np.median(xy[before], axis=0)
    normalized = (xy - origin_xy) @ basis
    steady_window = [stop['confirmation_camera_seconds'] + .1,
                     stop['decision_camera_seconds'] - .1]
    steady = valid & (video_t >= steady_window[0]) & (video_t <= steady_window[1])
    terminal = np.median(normalized[steady], axis=0)
    scatter = float(np.sqrt(np.sum(np.var(normalized[steady], axis=0))))
    def point_at(stamp):
        return np.array([np.interp(stamp, video_t[valid], normalized[valid, axis]) for axis in range(2)])
    at_lend = point_at(pts(mcu_lend))
    at_stop = point_at(pts(mcu_stop))
    pd.DataFrame(dict(time_video_s=video_t, detected=valid,
        right_mm=normalized[:, 0], forward_mm=normalized[:, 1])).to_csv(
            OUT / (label + '_relative_trajectory.csv'), index=False)
    settings = json.loads((run / 'saved_1ms.settings.json').read_text(encoding='utf-8'))
    result = dict(mouse='A', motion='long_r90', lstart_mm=settings['parameters']['lstart'], correction_deg=settings['parameters']['correction'],
        source_trial=str(run / 'trial.json'), source_profile=str(folder / 'body_geometry_used.json'),
        motion_observation_rate=report['required_motion_window_quality']['observation_rate'],
        motion_calibration_rate=report['required_motion_window_quality']['calibration_rate'],
        relative_observation_quality_passed=report['required_motion_window_quality']['accepted_for_relative_review'],
        measured_period_ms=float(period), encoder_model_fit_rmse_mm=float(rms), encoder_model_motion_rmse_mm=motion_rms,
        fitted_video_time_offset_s=float(offset), timing_method='Encoder-average reconstruction with known20sample FIR delay and measured period; local rigid trajectory fit. No independent hardware synchronization.',
        scale_fitted=False, entry_window_pts_s=entry_window, exit_window_pts_s=exit_window,
        entry_line=entry_quality, exit_line=exit_quality,
        entry_board_heading_deg=float(np.degrees(np.arctan2(entry[1], entry[0]))),
        exit_board_heading_deg=float(np.degrees(np.arctan2(exit_direction[1], exit_direction[0]))),
        entry_to_exit_path_rotation_deg=rotation_deg, path_rotation_error_from90_deg=rotation_deg - 90,
        reference_frame='Relative marker midpoint origin and measured entry direction; fixed-board placement is not accepted.',
        estimated_Lend_video_pts_s=pts(mcu_lend), estimated_Lend_right_forward_mm=at_lend.tolist(),
        estimated_Lend_sensitivity_minus5ms_mm=point_at(pts(mcu_lend) - .005).tolist(),
        estimated_Lend_sensitivity_plus5ms_mm=point_at(pts(mcu_lend) + .005).tolist(),
        post_ideal_stop_video_pts_s=pts(mcu_stop), post_ideal_stop_right_forward_mm=at_stop.tolist(),
        stationary_window_pts_s=steady_window, stationary_observations=int(steady.sum()),
        stationary_scatter_rms_mm=scatter, final_relative_right_forward_mm=terminal.tolist(),
        movement_after_post_ideal_stop_mm=(terminal - at_stop).tolist(),
        original_auto_stop_confirmed=json.loads((folder/'summary.json').read_text(encoding='utf-8'))['auto_stop_confirmed'], actual_capture_stop_evidence=stop['log_stop_evidence'],
        relative_review_eligible=bool(report['required_motion_window_quality']['accepted_for_relative_review']
            and rms <= 8 and motion_rms <= 8 and steady.sum() >= 12 and scatter <= 1),
        parameter_adoption_allowed=False,
        limitations=['Relative path direction and marker midpoint. Axle absolute position and height-corrected body yaw are unverified.',
            'The original auto-stop summary remains preserved. The actual late stationary window is independently checked.',
            'Lend points depend on model-fitted timing; the5ms sensitivity is reported. Do not adopt turn lengths from terminal stopping distance.',
            'One run per correction setting; repeatability is not established.'],
        input_sha256={str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest() for path in
            [run / 'trial.json', run / 'saved_1ms.csv', run / 'telemetry.csv', folder / 'report.json', folder / 'body_geometry_used.json']})
    return result, normalized, valid, at_lend, terminal


cases = [('corr1p0_01', 'next_corr1p0/local_execution/run_A_R90_1800_corr1p0_20261120_002'),
 ('corr1p0_02', 'repeat_corr1p0_02/local_execution/run_A_R90_1800_corr1p0_repeat02_20261120_001'),
 ('corr1p0_03', 'repeat_corr1p0_03/local_execution/run_A_R90_1800_corr1p0_repeat03_20261120_001'),
 ('Ls7p8_01', 'next_corr1p0_Ls7p8/local_execution/run_A_R90_1800_c1_Ls7p8_20261120_002')]
results = []
fig, axes = plt.subplots(1, 2, figsize=(11, 4.5), layout='constrained')
for label, run in cases:
    result, xy, valid, at_lend, terminal = analyze(label, run)
    results.append(result)
    line = axes[0].plot(xy[valid, 0], xy[valid, 1], label=label)[0]
    axes[0].scatter(*at_lend, color=line.get_color(), marker='s')
    axes[0].scatter(*terminal, color=line.get_color(), marker='o')
    axes[1].bar(label, result['path_rotation_error_from90_deg'])
axes[0].scatter(90, 270, color='black', marker='x', label='Nominal turn-end plan')
axes[0].set(xlabel='Right displacement [mm]', ylabel='Forward displacement [mm]')
axes[0].axis('equal')
axes[0].legend(fontsize=8)
axes[1].axhline(0, color='black', lw=1)
axes[1].set(ylabel='Entry-to-exit path angle error [deg]')
for ax in axes:
    ax.grid(alpha=.25)
fig.savefig(OUT / 'video_path_comparison.png', dpi=160)
plt.close(fig)
document = dict(schema_version=1, mouse='A', motion='long_r90', comparisons=results,
    source_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    source_params_changed=False, power_state='Both OFF confirmed', ble_disconnected=True,
    parameter_adoption_allowed=False, execution_pending=False)
with (OUT / 'video_comparison.json').open('x', encoding='utf-8') as output:
    json.dump(document, output, ensure_ascii=False, indent=2, allow_nan=False)
    output.write('\n')
print(json.dumps([{k:r.get(k) for k in ('lstart_mm','correction_deg','entry_to_exit_path_rotation_deg','path_rotation_error_from90_deg','estimated_Lend_right_forward_mm','final_relative_right_forward_mm','motion_observation_rate','encoder_model_motion_rmse_mm','relative_review_eligible')} for r in results],ensure_ascii=False),flush=True)
