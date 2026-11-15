"""Compare two saved B R180 trials in measured entry frames; no device access."""
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
sys.path.insert(0, str(ROOT/'tools/analysis'))
from pixel_camera_tuning_report import summarize
from video_marker_pose import log_elapsed_seconds


def read_trial(folder):
    summary = summarize(folder)
    report = json.loads((folder/'report.json').read_text(encoding='utf-8'))
    trial = json.loads(Path(report['trial_json']).read_text(encoding='utf-8'))
    settings = json.loads(Path(trial['type8_settings_json']).read_text(encoding='utf-8'))
    log = pd.read_csv(trial['type8_log_csv'])
    t = log_elapsed_seconds(log)
    target, actual = (log[key].to_numpy() for key in ('ideal.rad_velo', 'ego.rad_velo'))
    turn = abs(target) >= .02*abs(target).max()
    indices = np.flatnonzero(turn)
    start = (trial['log_quality']['motion_start_time_ms']-float(log.time_ms.iloc[0])) % 2**32/1000
    end = (trial['log_quality']['motion_end_time_ms']-float(log.time_ms.iloc[0])) % 2**32/1000
    trajectory = pd.read_csv(folder/'normalized_trajectory.csv')
    phase = trajectory.time_from_motion_start_s.to_numpy()+start
    valid = trajectory.detected.to_numpy(bool)
    xy = trajectory[['right_mm', 'forward_mm']].to_numpy()
    exit_mask = (valid & (phase > t[indices[-1]]+.015) & (phase <= end)
                 & (np.interp(phase, t, log['ideal.velo']) > .25))
    points = xy[exit_mask]
    direction = np.linalg.svd(points-points.mean(axis=0), full_matrices=False)[2][0]
    if direction @ (points[-1]-points[0]) < 0:
        direction = -direction
    exit_span = float(np.ptp(points @ direction))
    assert len(points) >= 15 and exit_span >= 30
    speed_error = (log['ego.velo']-log['ideal.velo']).to_numpy()[turn]
    pose = pd.read_csv(folder/'apparent_floor_marker_pose.csv')
    spans = pose.loc[pose.marker_detected_raw, 'marker_separation_mm']
    geometry = json.loads((folder/'body_geometry_used.json').read_text(encoding='utf-8'))
    result = dict(trial_json=report['trial_json'], analysis_folder=str(folder),
        marker_detection=geometry['marker_detection'],
        settings=settings, camera_height_mm=trial['camera_height_mm'],
        relative_summary=summary, log_quality=trial['log_quality'],
        lateral_target_mm=90, lateral_error_mm=summary['endpoint_right_mm']-90,
        exit_travel_direction_error_deg=float(np.degrees(np.arctan2(direction[0], -direction[1]))),
        exit_travel_direction_observations=len(points), exit_travel_span_mm=exit_span,
        body_yaw_verified=False,
        omega_rms_error_rad_s=float(np.sqrt(np.mean((actual[turn]-target[turn])**2))),
        maximum_abs_omega_error_rad_s=float(abs(actual[turn]-target[turn]).max()),
        mean_turn_speed_m_s=float(log['ego.velo'][turn].mean()),
        speed_rms_error_m_s=float(np.sqrt(np.mean(speed_error**2))),
        minimum_battery_V=float(log.Battery.min()),
        marker_span_range_mm=[float(spans.min()), float(spans.max())],
        wall_ekf_fresh_readback_available=False,
        maximum_logged_control_ir=float(abs(log.control_ir).max()),
        control_ir_is_actual_applied_correction_verified=False,
        execution_resends=trial['execution_resend_count'],
        input_sha256={str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in
                      (Path(report['trial_json']), Path(trial['type8_log_csv']),
                       Path(trial['type8_settings_json']), folder/'report.json')})
    return result, xy, valid


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--candidate', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    baseline, bxy, bvalid = read_trial(args.baseline)
    candidate, cxy, cvalid = read_trial(args.candidate)
    for key in ('pid', 'feedforward', 'suction'):
        assert baseline['settings'][key] == candidate['settings'][key]
    changes = {key: [value, candidate['settings']['parameters'][key]]
               for key, value in baseline['settings']['parameters'].items()
               if value != candidate['settings']['parameters'][key]}
    assert changes == {'r_min': [-42.5, -45.0]}, changes
    accepted = all(item['relative_summary']['auto_stop_confirmed'] for item in (baseline, candidate))
    args.output.mkdir(exist_ok=False)
    result = dict(mouse='B', motion='long_r180', observations_per_setting=1,
        status='SINGLE_TRIAL_COMPARISON_REVIEW_REQUIRED' if accepted else 'INSUFFICIENT_VIDEO_QUALITY',
        baseline=baseline, candidate=candidate, settings_changes=changes,
        same_colour_detector=baseline['marker_detection'] == candidate['marker_detection'],
        lateral_change_mm=candidate['relative_summary']['endpoint_right_mm']-baseline['relative_summary']['endpoint_right_mm'],
        repeatability_verified=False, parameter_adoption_allowed=False,
        firmware_parameters_modified=False,
        next_step='Repeat the same -45 mm candidate to assess scatter before further fine adjustment',
        limitations=['relative marker midpoint, not absolute axle position',
                    'camera height approximately 1500 mm', 'no independent hardware synchronization',
                    'wall EKF mode was not freshly queried during either capture',
                    'Each trial uses its independently checked colour detector; exact detector settings differ'])
    with (args.output/'comparison.json').open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
    figure, axis = plt.subplots(figsize=(6, 5), layout='constrained')
    axis.plot(*bxy[bvalid].T, label='B baseline R=-42.5 mm')
    axis.plot(*cxy[cvalid].T, label='A-reference R=-45 mm')
    axis.axvline(90, color='gray', linestyle='--', label='90 mm lateral target')
    axis.set(xlabel='Right [mm]', ylabel='Forward [mm]', title='B R180, 0.50 m/s; one trial each')
    axis.axis('equal')
    axis.legend(fontsize=8)
    figure.savefig(args.output/'comparison.png', dpi=160)
    plt.close(figure)
    print(json.dumps(result, ensure_ascii=True))


if __name__ == '__main__':
    main()
