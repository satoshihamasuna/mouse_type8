"""Reproduce metrics for the single saved B baseline; no device connection."""
import hashlib
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[3]
RUN = ROOT / 'tools/logs/pixel_camera/tuning_cli_B_portable_20261114_164124/run_B_R180_baseline_run01'
sys.path.insert(0, str(ROOT / 'tools/analysis'))
from pixel_camera_tuning_report import summarize
from video_marker_pose import log_elapsed_seconds


def main():
    output = RUN / 'full_v3'
    summary = summarize(output)
    trial = json.loads((RUN / 'trial.json').read_text(encoding='utf-8'))
    settings = json.loads((RUN / 'motion.settings.json').read_text(encoding='utf-8'))
    log = pd.read_csv(RUN / 'motion.csv')
    t = log_elapsed_seconds(log)
    target, actual = (log[key].to_numpy() for key in ('ideal.rad_velo', 'ego.rad_velo'))
    turn = np.abs(target) >= .02 * np.max(np.abs(target))
    indices = np.flatnonzero(turn)
    expanded = slice(max(0, indices[0]-1), min(len(log), indices[-1]+2))
    start = (trial['log_quality']['motion_start_time_ms']-float(log.time_ms.iloc[0])) % 2**32 / 1000
    end = (trial['log_quality']['motion_end_time_ms']-float(log.time_ms.iloc[0])) % 2**32 / 1000
    motion = (t >= start) & (t <= end)
    error = actual[turn]-target[turn]
    speed_error = log['ego.velo'].to_numpy()[turn]-log['ideal.velo'].to_numpy()[turn]
    control = dict(time_basis='MCU time_ms; trapezoid at actual sample times, not row count',
        rows=len(log), motion_duration_s=float(end-start), turn_duration_s=float(t[indices[-1]]-t[indices[0]]),
        command_degree=settings['parameters']['degree'],
        sampled_target_omega_integral_deg=float(np.degrees(np.trapezoid(target[expanded], t[expanded]))),
        sampled_gyro_integral_deg=float(np.degrees(np.trapezoid(actual[expanded], t[expanded]))),
        gyro_integral_is_calibrated_physical_yaw=False,
        omega_rms_error_rad_s=float(np.sqrt(np.mean(error**2))),
        maximum_abs_omega_error_rad_s=float(np.max(np.abs(error))),
        mean_turn_speed_m_s=float(log['ego.velo'][turn].mean()),
        speed_rms_error_m_s=float(np.sqrt(np.mean(speed_error**2))),
        maximum_requested_motor_voltage_V=float(np.abs(log.loc[motion, ['V_r', 'V_l']].to_numpy()).max()),
        minimum_battery_V=float(log.loc[motion, 'Battery'].min()),
        maximum_sample_gap_ms=float(np.diff(t).max()*1000),
        log_quality=trial['log_quality'])
    trajectory = pd.read_csv(output / 'normalized_trajectory.csv')
    stamps = trajectory.time_from_motion_start_s.to_numpy()+start
    valid = trajectory.detected.to_numpy(bool)
    xy = trajectory[['right_mm', 'forward_mm']].to_numpy()
    exit_mask = (valid & (stamps > t[indices[-1]]+.015) & (stamps <= end)
                 & (np.interp(stamps, t, log['ideal.velo'])*1000 > trial['preset']*.5))
    points = xy[exit_mask]
    direction = np.linalg.svd(points-points.mean(axis=0), full_matrices=False)[2][0]
    if direction @ (points[-1]-points[0]) < 0:
        direction = -direction
    residual = points-points.mean(axis=0)
    residual -= np.outer(residual @ direction, direction)
    exit_line = dict(method='measured exit travel direction; not height-corrected body yaw',
        observations=len(points), span_mm=float(np.ptp(points @ direction)),
        line_rms_mm=float(np.sqrt(np.mean(np.sum(residual**2, axis=1)))),
        deviation_from_reverse_entry_deg=float(np.degrees(np.arctan2(direction[0], -direction[1]))))
    result = dict(mouse='B', motion='long_r180', baseline_trial=str(RUN / 'trial.json'),
        capture_count=1, camera_height_mm=1500, camera_height_is_approximate=True,
        settings=settings, control=control, relative_video_summary=summary,
        exit_line=exit_line, target_lateral_mm=90,
        lateral_error_mm=summary['endpoint_right_mm']-90,
        parameter_adoption_allowed=False, repeatability_verified=False,
        independent_hardware_sync_verified=False,
        next_candidate='B_R180_A_radius_only_candidate.settings.json',
        rationale='Compare A radius -45 mm alone first. Keep B lengths, correction, PID and FF.',
        input_sha256={str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in
            (RUN/'trial.json', RUN/'motion.csv', RUN/'motion.settings.json', output/'report.json')})
    with (output / 'baseline_review.json').open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
    figure, axes = plt.subplots(1, 2, figsize=(10, 4.5), layout='constrained')
    axes[0].plot(*xy[valid].T, label='B baseline, marker midpoint')
    axes[0].plot(summary['endpoint_right_mm'], summary['endpoint_forward_mm'], 'o', label='Stopped')
    axes[0].axvline(90, linestyle='--', color='gray', label='90 mm lateral target')
    axes[0].set(xlabel='Right [mm]', ylabel='Forward [mm]', title='R180, 0.50 m/s')
    axes[0].axis('equal')
    axes[0].legend(fontsize=8)
    axes[1].plot(t-start, target, label='Target omega')
    axes[1].plot(t-start, actual, label='Gyro omega')
    axes[1].set(xlabel='Time from motion start [s]', ylabel='Angular velocity [rad/s]')
    axes[1].legend(fontsize=8)
    figure.savefig(output/'baseline_review.png', dpi=160)
    plt.close(figure)
    print(json.dumps(dict(summary=summary, control=control, exit_line=exit_line), ensure_ascii=True))


if __name__ == '__main__':
    main()
