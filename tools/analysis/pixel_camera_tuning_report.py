"""撮影済み単体試験の速報・詳細解析。機体やカメラには接続しない。"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pixel_camera_turn_capture import read_rows, write_json
from pixel_camera_tuning import log_stop_evidence
from video_marker_pose import log_elapsed_seconds


def summarize(folder):
    folder = Path(folder)
    report = json.loads((folder/'report.json').read_text(encoding='utf-8'))
    trial = json.loads(Path(report['trial_json']).read_text(encoding='utf-8'))
    raw = pd.read_csv(trial['type8_log_csv'])
    time = log_elapsed_seconds(raw)
    start = (trial['log_quality']['motion_start_time_ms']-float(raw.time_ms.iloc[0])) % 2**32/1000
    end = (trial['log_quality']['motion_end_time_ms']-float(raw.time_ms.iloc[0])) % 2**32/1000
    target = raw['ideal.rad_velo'].to_numpy()
    turning = time[abs(target) >= .02*abs(target).max()]
    comparison = pd.read_csv(folder/'relative_video_log_comparison.csv')
    stamps = comparison.time_log_s.to_numpy()
    before = ((stamps >= start) & (stamps < turning[0]-.015)
              & (np.interp(stamps, time, raw['ideal.velo'])*1000 > trial['preset']*.5))
    points = comparison[['center_x_mm', 'center_y_mm']].to_numpy()[before]
    minimum = 6 if report['frame_stride'] > 1 else 15
    if len(points) < minimum:
        raise ValueError('初期直進の観測が不足しています')
    forward = np.linalg.svd(points-points.mean(axis=0), full_matrices=False)[2][0]
    if forward @ (points[-1]-points[0]) < 0:
        forward = -forward
    if np.ptp(points @ forward) < 30:
        raise ValueError('初期直進の観測範囲が30mm未満です')
    basis = np.column_stack([[-forward[1], forward[0]], forward])
    entry_residual = points-points.mean(axis=0)
    entry_residual -= np.outer(entry_residual @ forward, forward)
    reference_frame = dict(method='relative_origin_and_measured_entry_direction',
        origin='stationary pre-motion marker midpoint from relative_marker_midpoint.csv',
        forward_axis_in_camera_plane=forward.tolist(),
        camera_plane_to_right_forward_matrix=basis.tolist(),
        entry_heading_camera_plane_deg=float(np.degrees(np.arctan2(forward[1], forward[0]))),
        entry_observations=len(points), entry_span_mm=float(np.ptp(points @ forward)),
        entry_line_rms_mm=float(np.sqrt(np.mean(np.sum(entry_residual**2, axis=1)))),
        scale_fitted=False, endpoint_or_exit_used_for_frame=False)
    pose = pd.read_csv(folder/'relative_marker_midpoint.csv')
    xy = pose[['center_dx_mm','center_dy_mm']].to_numpy() @ basis
    phase = pose.time_video_s.to_numpy()-report['log_time_zero_at_video_s']
    valid = pose.detected.to_numpy(bool)
    stopped = valid & (phase >= end+.2) & (phase <= end+.6)
    endpoint = np.median(xy[stopped], axis=0) if stopped.any() else [np.nan, np.nan]
    scatter = float(np.sqrt(np.sum(np.var(xy[stopped], axis=0)))) if stopped.any() else float('inf')
    span = float(np.ptp(phase[stopped])) if stopped.any() else 0
    video_stopped = bool(stopped.sum() >= 12 and span >= .25 and scatter <= 1.0
                         and np.linalg.norm(xy[stopped]-endpoint, axis=1).max() <= 3)
    log_stopped = log_stop_evidence(read_rows(Path(trial['type8_log_csv'])))
    quality = report['required_motion_window_quality']['accepted_for_relative_review']
    confirmed = bool(quality and report['trajectory_fit_rmse_mm'] <= 8 and video_stopped
                     and log_stopped['confirmed'] and trial.get('camera_state') == 'PASSED'
                     and trial.get('camera_run_start_confirmed') is True)
    summary = dict(status='PROVISIONAL' if report['frame_stride'] > 1 else 'DETAILED_REVIEW_REQUIRED',
        auto_stop_confirmed=confirmed, endpoint_right_mm=float(endpoint[0]), endpoint_forward_mm=float(endpoint[1]),
        stationary_observations=int(stopped.sum()), stationary_span_s=span, stationary_scatter_rms_mm=scatter,
        video_stationary=video_stopped, log_stop_evidence=log_stopped,
        motion_observation_rate=report['required_motion_window_quality']['observation_rate'],
        trajectory_fit_rmse_mm=report['trajectory_fit_rmse_mm'], frame_stride=report['frame_stride'],
        observation_scope=report['observation_scope'], parameter_adoption_allowed=False,
        trajectory_reference_frame=reference_frame,
        trial_json=report['trial_json'], report_json=str(folder/'report.json'),
        report_sha256=hashlib.sha256((folder/'report.json').read_bytes()).hexdigest(),
        note='相対目印中点。絶対位置・高さ補正済み機体yaw・独立ハードウェア同期は未検証')
    # 欠測時のNaN/infをJSONの数値に見せない。
    for key, value in list(summary.items()):
        if isinstance(value, float) and not np.isfinite(value):
            summary[key] = None
    write_json(folder/'summary.json', summary)
    pd.DataFrame(dict(time_from_motion_start_s=phase-start, right_mm=xy[:,0], forward_mm=xy[:,1],
                      detected=valid)).to_csv(folder/'normalized_trajectory.csv', index=False)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial', type=Path, required=True)
    parser.add_argument('--stride', type=int, choices=(1,4), required=True)
    parser.add_argument('--video-start', type=float)
    parser.add_argument('--video-end', type=float)
    args = parser.parse_args()
    folder = args.trial.parent
    trial = json.loads(args.trial.read_text(encoding='utf-8'))
    duration = ((trial['log_quality']['motion_end_time_ms']-trial['log_quality']['motion_start_time_ms']) % 2**32)/1000
    estimate = trial.get('video_motion_start_estimate_s', 7.0)
    start = args.video_start if args.video_start is not None else max(0, estimate-1.0)
    end = args.video_end if args.video_end is not None else estimate+duration+1.0
    output = folder/('quick' if args.stride == 4 else 'full')
    command = [sys.executable, str(Path(__file__).with_name('pixel_camera_relative_pose.py')),
        '--trial', str(args.trial), '--marker-reference', str(folder/'marker_reference_used.json'),
        '--body-geometry', str(folder/'body_geometry_used.json'), '--camera-height-mm', str(trial['camera_height_mm']),
        '--height-source', 'approximate_for_capture', '--fixed-camera', '--captured-trial',
        '--video-start', str(start), '--video-end', str(end), '--frame-stride', str(args.stride), '--output', str(output)]
    code = subprocess.call(command)
    if code:
        return code
    try:
        summary = summarize(output)
        print(json.dumps(summary, ensure_ascii=False), flush=True)
    except Exception as error:
        write_json(output/'summary.json', dict(status='NEEDS_OPERATOR_REVIEW', auto_stop_confirmed=False, error=str(error)))
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
