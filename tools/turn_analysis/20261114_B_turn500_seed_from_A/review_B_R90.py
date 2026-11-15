"""保存済みB右90度の相対軌道・出口方向・制御追従を評価する。機器への接続なし。"""
import argparse
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
sys.path[:0] = [str(ROOT/'tools'), str(ROOT/'tools/analysis')]
from pixel_camera_tuning_report import summarize
from video_marker_pose import log_elapsed_seconds


def review(folder, output):
    summary = summarize(folder)
    report = json.loads((folder/'report.json').read_text(encoding='utf-8'))
    trial_path = Path(report['trial_json'])
    trial = json.loads(trial_path.read_text(encoding='utf-8'))
    settings_path = Path(trial['type8_settings_json'])
    settings = json.loads(settings_path.read_text(encoding='utf-8'))
    assert trial['mouse'] == 'B' and settings['motion_name'] == 'long_r90'
    assert report['frame_stride'] == 1 and settings['turn_count'] == 1
    log = pd.read_csv(trial['type8_log_csv'])
    time = log_elapsed_seconds(log)
    target, actual = [log[k].to_numpy() for k in ('ideal.rad_velo', 'ego.rad_velo')]
    turn = abs(target) >= .02*abs(target).max()
    indices = np.flatnonzero(turn)
    start, end = [(trial['log_quality'][k]-float(log.time_ms.iloc[0])) % 2**32/1000
                  for k in ('motion_start_time_ms', 'motion_end_time_ms')]
    trajectory = pd.read_csv(folder/'normalized_trajectory.csv')
    phase = trajectory.time_from_motion_start_s.to_numpy()+start
    valid = trajectory.detected.to_numpy(bool)
    xy = trajectory[['right_mm', 'forward_mm']].to_numpy()
    exit_mask = (valid & (phase > time[indices[-1]]+.015) & (phase <= end)
                 & (np.interp(phase, time, log['ideal.velo']) > .25))
    points = xy[exit_mask]
    assert len(points) >= 15
    direction = np.linalg.svd(points-points.mean(axis=0), full_matrices=False)[2][0]
    if direction @ (points[-1]-points[0]) < 0:
        direction = -direction
    exit_span = float(np.ptp(points @ direction))
    assert exit_span >= 30
    speed_error = (log['ego.velo']-log['ideal.velo']).to_numpy()[turn]
    result = dict(mouse='B', motion='long_r90', target_right_forward_mm=[270, 270],
        status='VALID_SINGLE_TRIAL_REVIEW' if summary['auto_stop_confirmed'] else 'VIDEO_QUALITY_INSUFFICIENT',
        valid_trials=int(summary['auto_stop_confirmed']), summary=summary, settings=settings,
        endpoint_error_right_mm=summary['endpoint_right_mm']-270,
        endpoint_error_forward_mm=summary['endpoint_forward_mm']-270,
        exit_travel_error_deg=float(np.degrees(np.arctan2(direction[1], direction[0]))),
        exit_observations=len(points), exit_span_mm=exit_span,
        omega_rmse_rad_s=float(np.sqrt(np.mean((actual[turn]-target[turn])**2))),
        speed_rmse_m_s=float(np.sqrt(np.mean(speed_error**2))),
        minimum_battery_V=float(log.Battery.min()), log_quality=trial['log_quality'],
        parameter_adoption_allowed=False, repeatability_verified=False,
        body_yaw_verified=False, absolute_axle_position_verified=False,
        wall_ekf_fresh_readback_available=False, independent_hardware_sync_verified=False,
        input_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in
                      (trial_path, settings_path, Path(trial['type8_log_csv']), folder/'report.json', Path(__file__))})
    output.mkdir(exist_ok=False)
    (output/'review.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    fig, axes = plt.subplots(1, 2, figsize=(10, 4), layout='constrained')
    axes[0].plot(*xy[valid].T, label='Observed marker midpoint')
    axes[0].plot(270, 270, 'x', label='Target 270 / 270 mm')
    axes[0].set(xlabel='Right [mm]', ylabel='Forward [mm]', title='B R90, 500 mm/s')
    axes[0].axis('equal')
    axes[0].legend(fontsize=8)
    motion = (time >= start-.05) & (time <= end+.05)
    axes[1].plot(time[motion]-start, target[motion], label='Target')
    axes[1].plot(time[motion]-start, actual[motion], label='Measured')
    axes[1].set(xlabel='Time from motion start [s]', ylabel='Angular velocity [rad/s]')
    axes[1].legend(fontsize=8)
    fig.savefig(output/'trajectory_and_tracking.png', dpi=150)
    plt.close(fig)
    print(json.dumps({k: result[k] for k in ['status', 'endpoint_error_right_mm',
        'endpoint_error_forward_mm', 'exit_travel_error_deg', 'minimum_battery_V']}, ensure_ascii=False))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--analysis', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    review(args.analysis, args.output)
