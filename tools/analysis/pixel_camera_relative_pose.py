"""固定カメラの真下が不明でも、目印中点の高さ補正済み相対軌跡を比較する。"""
from __future__ import annotations

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

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pixel_camera_geometry import relative_marker_midpoint
from video_marker_pose import read_video_pose, board_to_pose_transform, transform_points, integrate_log, fit_time_and_pose


def file_hash(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def reference_digest(reference):
    return hashlib.sha256(json.dumps(reference, sort_keys=True).encode()).hexdigest()


def load_pose_cache(path, trial, reference, geometry, window, frame_stride=1):
    provenance = json.loads(path.with_suffix('.provenance.json').read_text(encoding='utf-8'))
    if (provenance.get('frame_stride', 1) != frame_stride
            or provenance['video_sha256'] != file_hash(trial['video_file'])
            or provenance['cached_pose_sha256'] != file_hash(path)
            or provenance['reference_digest'] != reference_digest(reference)
            or provenance['front_colour'] != geometry['front_colour']
            or provenance.get('marker_detection') != geometry.get('marker_detection')
            or window[0] < provenance['video_window'][0] or window[1] > provenance['video_window'][1]):
        raise ValueError('pose cache does not match video, calibration, detector, or analysis window')
    pose = pd.read_csv(path)
    if 'calibration_valid' not in pose:
        if not provenance.get('calibration_mask_inference'):
            raise ValueError('pose cache has no calibration validity evidence')
        pose['calibration_valid'] = pose.marker_detected_raw
    pose = pose[(pose.time_video_s >= window[0]) & (pose.time_video_s <= window[1])].reset_index(drop=True)
    pose.attrs['marker_calibration'] = provenance['marker_calibration']
    return pose


def required_motion_quality(time, valid, calibration_valid, log_time, trial, offset_s, frame_stride=1):
    """MCUの動作区間と、予め固定した前200 ms・後100 msを検査する。"""
    origin_ms = float(log_time[0])
    start = offset_s+((trial['log_quality']['motion_start_time_ms']-origin_ms) % (2**32))/1000
    end = offset_s+((trial['log_quality']['motion_end_time_ms']-origin_ms) % (2**32))/1000
    required = (time >= start-.2) & (time <= end+.1)
    before = valid & (time >= start-.2) & (time < start)
    after = valid & (time > end) & (time <= end+.1)
    contained = bool(time[0] <= start-.2 and time[-1] >= end+.1)
    rate = float(valid[required].mean()) if required.any() else 0.0
    calibration_rate = float(calibration_valid[required].mean()) if required.any() else 0.0
    minimum = int(np.ceil(10/frame_stride))
    return dict(video_motion_start_s=float(start), video_motion_end_s=float(end),
        required_interval_pts_s=[float(start-.2), float(end+.1)], interval_contained=contained,
        observation_rate=rate, calibration_rate=calibration_rate,
        stationary_before_observations=int(before.sum()), stationary_after_observations=int(after.sum()),
        frame_stride=frame_stride, minimum_stationary_observations=minimum,
        accepted_for_relative_review=bool(contained and rate >= .95 and before.sum() >= minimum and after.sum() >= minimum),
        timing_basis='MCU motion boundaries mapped with trajectory-fitted time offset; no independent hardware sync')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial', type=Path, required=True, help='saved trial.json linking video, CSV and settings')
    parser.add_argument('--marker-reference', type=Path, required=True)
    parser.add_argument('--body-geometry', type=Path, required=True)
    parser.add_argument('--camera-height-mm', type=float, required=True)
    parser.add_argument('--height-source', choices=('measured_for_capture', 'approximate_for_capture', 'historical_assumption'), required=True)
    parser.add_argument('--fixed-camera', action='store_true', help='camera position and height were fixed within this video')
    parser.add_argument('--video-start', type=float, required=True)
    parser.add_argument('--video-end', type=float, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--pose-cache', type=Path, help='explicit apparent pose CSV with matching provenance JSON')
    parser.add_argument('--frame-stride', type=int, choices=(1, 4), default=1, help='4は約60fpsの速報。全フレーム検証と区別する')
    parser.add_argument('--captured-trial', action='store_true', help='GUI撮影済みの未目視試行を解析する。採用はしない')
    args = parser.parse_args()
    if not args.fixed_camera:
        parser.error('this method requires --fixed-camera for the source video')
    trial = json.loads(args.trial.read_text(encoding='utf-8'))
    settings_path = Path(trial['type8_settings_json'])
    settings = json.loads(settings_path.read_text(encoding='utf-8'))
    geometry = json.loads(args.body_geometry.read_text(encoding='utf-8'))
    reference = json.loads(args.marker_reference.read_text(encoding='utf-8'))
    captured = (args.captured_trial and trial.get('state') == 'CAPTURED_REVIEW_REQUIRED'
                and trial.get('camera_state') == 'PASSED' and trial.get('camera_run_start_confirmed') is True
                and trial.get('log_quality', {}).get('full_motion_retained') is True)
    if (trial.get('state') != 'PASSED' and not captured) or trial.get('mouse') != geometry.get('mouse') or settings.get('mouse') != trial['mouse']:
        parser.error('trial, settings and body geometry must identify the same machine and a passed capture')
    offset = geometry.get('axle_offset_from_marker_midpoint_mm')
    if offset is not None and not np.allclose(offset, [0, 0], atol=1e-9):
        parser.error('unknown camera foot supports marker midpoint only; nonzero axle offsets require body heading')
    # 入力が不正なら動画を解析する前に拒否する。
    relative_marker_midpoint([[0, 0]], [[0, 0]], args.camera_height_mm,
                             geometry['front_height_mm'], geometry['rear_height_mm'])
    args.output.mkdir(parents=True, exist_ok=False)
    if args.pose_cache:
        pose = load_pose_cache(args.pose_cache, trial, reference, geometry, (args.video_start, args.video_end), args.frame_stride)
    else:
        pose, _, _, _ = read_video_pose(Path(trial['video_file']), geometry['front_colour'],
            marker_reference=reference, video_window=(args.video_start, args.video_end),
            marker_detection=geometry.get('marker_detection'),
            frame_stride=args.frame_stride,
            progress=lambda stage, count, pts: print(f'{stage}: {count} frames, PTS {pts:.3f} s', flush=True))
    pose.to_csv(args.output/'apparent_floor_marker_pose.csv', index=False)
    provenance = dict(schema_version=1, video_file=trial['video_file'], video_sha256=file_hash(trial['video_file']),
        cached_pose_sha256=file_hash(args.output/'apparent_floor_marker_pose.csv'),
        reference_digest=reference_digest(reference), front_colour=geometry['front_colour'],
        marker_detection=geometry.get('marker_detection'), video_window=[args.video_start, args.video_end], frame_stride=args.frame_stride,
        marker_calibration=pose.attrs['marker_calibration'])
    (args.output/'apparent_floor_marker_pose.provenance.json').write_text(json.dumps(provenance, ensure_ascii=False, indent=2), encoding='utf-8')
    board_transform = np.linalg.inv(board_to_pose_transform(reference))
    front = transform_points(pose[['blue_x_mm', 'blue_y_mm']].to_numpy(), board_transform)
    rear = transform_points(pose[['yellow_x_mm', 'yellow_y_mm']].to_numpy(), board_transform)
    relative = relative_marker_midpoint(front, rear, args.camera_height_mm,
        geometry['front_height_mm'], geometry['rear_height_mm'], pose.marker_detected_raw.to_numpy(bool))
    xy = relative['displacement_xy_mm']
    valid = relative['valid']
    time = pose.time_video_s.to_numpy()
    baseline = valid & (time <= args.video_start+.2)
    if baseline.sum() >= 10:
        xy -= np.nanmedian(xy[baseline], axis=0)
    result = pd.DataFrame(dict(frame=pose.frame, time_video_s=time, detected=valid,
        center_dx_mm=xy[:, 0], center_dy_mm=xy[:, 1]))
    result.to_csv(args.output/'relative_marker_midpoint.csv', index=False)
    quality = dict(body_detection_rate=float(valid.mean()), frames=len(valid), valid_frames=int(valid.sum()),
                   baseline_observations=int(baseline.sum()), marker_calibration=pose.attrs['marker_calibration'])
    calibration_valid = pose.calibration_valid.to_numpy(bool)
    quality['body_detection_rate_given_calibration'] = float(valid.sum()/calibration_valid.sum()) if calibration_valid.any() else 0.0
    snapshot = dict(geometry, analysis_mode='relative_marker_midpoint', camera_height_mm=args.camera_height_mm,
        camera_height_source=args.height_source, measurement_video_file=trial['video_file'],
        absolute_position_available=False, height_corrected_heading_available=False)
    (args.output/'body_geometry_used.json').write_text(json.dumps(snapshot, ensure_ascii=False, indent=2), encoding='utf-8')
    (args.output/'marker_reference_used.json').write_text(json.dumps(reference, ensure_ascii=False, indent=2), encoding='utf-8')
    report = dict(status='REVIEW_REQUIRED', mouse=trial['mouse'], motion=trial['motion'],
        frame_stride=args.frame_stride, observation_scope='all_frames' if args.frame_stride == 1 else 'sampled_provisional',
        trial_json=str(args.trial.resolve()), video_file=trial['video_file'], csv=trial['type8_log_csv'],
        settings_json=str(settings_path.resolve()), camera_height_mm=args.camera_height_mm, height_source=args.height_source,
        quality=quality, absolute_position_available=False, height_corrected_heading_available=False,
        parameter_adoption_allowed=False,
        centre='marker midpoint; approximate vehicle centre from user supplied symmetric 25 mm positions',
        timing='decoded video PTS; MCU time_ms; trajectory fitted alignment is not independent hardware synchronization',
        input_sha256={str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest() for p in
                      (args.trial, args.body_geometry, args.marker_reference, settings_path, Path(trial['type8_log_csv']))})
    log = integrate_log(Path(trial['type8_log_csv']))
    figure, axes = plt.subplots(1, 2, figsize=(11, 4.5), layout='constrained')
    axes[0].plot(xy[:, 0], xy[:, 1], label='Video midpoint displacement')
    axes[0].set(xlabel='Board x displacement [mm]', ylabel='Board y displacement [mm]')
    axes[0].axis('equal')
    if valid.sum() >= 20 and baseline.sum() >= 10:
        alignment_pose = pose.copy()
        # 間欠的な欠測は処理用にだけ補間し、位置合わせ自体からは除外する。
        # ログ積分と既存解析の軸は、盤面画像のx軸に対して反転している。
        # 同じ軸で剛体位置合わせを行い、出力は盤面の右x正・下y正へ戻す。
        board_to_legacy = board_to_pose_transform(reference)[:2, :2]
        aligned_xy = xy @ board_to_legacy.T
        alignment_pose['center_x_mm'] = pd.Series(aligned_xy[:, 0]).interpolate(limit_direction='both')
        alignment_pose['center_y_mm'] = pd.Series(aligned_xy[:, 1]).interpolate(limit_direction='both')
        filled = alignment_pose[['center_x_mm', 'center_y_mm']].to_numpy()
        alignment_pose['speed_playback_mm_s'] = np.linalg.norm(np.gradient(filled, time, axis=0), axis=1)
        raw_log = pd.read_csv(trial['type8_log_csv'])
        origin_ms = float(raw_log.time_ms.iloc[0])
        required_log_interval = (
            ((trial['log_quality']['motion_start_time_ms']-origin_ms) % (2**32))/1000-.2,
            ((trial['log_quality']['motion_end_time_ms']-origin_ms) % (2**32))/1000+.1)
        comparison, _, offset_s, _, _, rms = fit_time_and_pose(alignment_pose, log, 1.0, required_log_interval)
        legacy_to_board = np.linalg.inv(board_to_legacy)
        for columns in (['center_x_mm', 'center_y_mm'], ['log_x_mm', 'log_y_mm']):
            comparison[columns] = comparison[columns].to_numpy() @ legacy_to_board.T
        comparison.rename(columns={'heading_rad': 'apparent_marker_angle_rad_not_height_corrected'}, inplace=True)
        comparison.to_csv(args.output/'relative_video_log_comparison.csv', index=False)
        report.update(trajectory_fit_rmse_mm=float(rms), log_time_zero_at_video_s=float(offset_s))
        window_quality = required_motion_quality(time, valid, calibration_valid,
            raw_log.time_ms.to_numpy(), trial, offset_s, args.frame_stride)
        report['required_motion_window_quality'] = window_quality
        if not window_quality['accepted_for_relative_review']:
            report['status'] = 'INSUFFICIENT_REQUIRED_MOTION_WINDOW'
        elif args.frame_stride > 1:
            report['status'] = 'PROVISIONAL'
        axes[0].plot(comparison.log_x_mm, comparison.log_y_mm, label='Log, rigidly aligned')
        axes[1].plot(comparison.time_log_s, comparison.position_error_mm)
        axes[1].set(xlabel='Log elapsed time [s]', ylabel='Aligned position difference [mm]')
    else:
        report['status'] = 'INSUFFICIENT_BODY_OR_BASELINE_OBSERVATIONS'
        axes[1].plot(time, valid.astype(int))
        axes[1].set(xlabel='Video PTS [s]', ylabel='Valid observation')
    axes[0].legend()
    figure.savefig(args.output/'relative_trajectory.png', dpi=150)
    plt.close(figure)
    # 半径やLstart/Lendの採用値を、この比較だけで自動確定しない。
    (args.output/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(dict(status=report['status'], **quality,
                         trajectory_fit_rmse_mm=report.get('trajectory_fit_rmse_mm')), ensure_ascii=False), flush=True)
    return 0 if report['status'] in ('REVIEW_REQUIRED', 'PROVISIONAL') else 2


if __name__ == '__main__':
    raise SystemExit(main())
