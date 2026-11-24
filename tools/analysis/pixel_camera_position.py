"""動画だけから盤面上の機体目印中点を推定する。ログ照合・機器操作は行わない。"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pixel_camera_detection import file_sha256, load_detection_geometry, freeze_geometry_detection
from pixel_camera_reference import relocate_marker_search
from video_marker_pose import board_to_pose_transform, read_video_pose, transform_points, marker_registration


def prepare_reference(video, reference, start, end):
    """この動画で探索範囲を検査する。座標変換は後段で各フレームから再計算する。"""
    diagnostic = dict(available=False, search_rois_relocated=False, attempts=[],
        per_frame_registration_required=True, calibration_thresholds_changed=False)
    cap = cv2.VideoCapture(str(video))
    try:
        cap.set(cv2.CAP_PROP_ORIENTATION_AUTO, 0)
        for delta in (0., .25, .5, 1.):
            stamp = start+delta
            if end is not None and stamp >= end:
                break
            cap.set(cv2.CAP_PROP_POS_MSEC, stamp*1000)
            ok, frame = cap.read()
            if not ok:
                continue
            pts = float(cap.get(cv2.CAP_PROP_POS_MSEC))/1000
            registration = marker_registration(frame, reference)
            attempt = dict(video_pts_s=pts, original_available=registration.get('available', False),
                original_reason=registration.get('reason'), decoded_marker_ids=registration.get('detected_marker_ids', []))
            diagnostic['attempts'].append(attempt)
            if registration.get('available'):
                diagnostic.update(available=True, used_marker_ids=registration['used_marker_ids'])
                return reference, diagnostic
            recovery = relocate_marker_search(frame, reference)
            attempt.update(recovery_available=recovery['available'], recovery_reason=recovery.get('reason'))
            if recovery['available']:
                diagnostic.update(available=True, search_rois_relocated=True, source_video_pts_s=pts,
                    used_marker_ids=recovery['registration']['used_marker_ids'], method=recovery['method'])
                return recovery['profile'], diagnostic
    finally:
        cap.release()
    return reference, diagnostic


def board_positions(pose, reference):
    """従来追跡の座標を登録盤面のXYへ戻す。未観測フレームの補間値は出力しない。"""
    valid = pose.calibration_valid.to_numpy(bool) & pose.marker_detected_raw.to_numpy(bool)
    counts = pose.registered_marker_count.to_numpy(int)
    valid &= counts >= 2
    transform = np.linalg.inv(board_to_pose_transform(reference))
    front = transform_points(pose[['blue_x_mm', 'blue_y_mm']].to_numpy(), transform)
    rear = transform_points(pose[['yellow_x_mm', 'yellow_y_mm']].to_numpy(), transform)
    valid &= np.isfinite(front).all(axis=1) & np.isfinite(rear).all(axis=1)
    estimated = pose.get('marker_estimated_raw', pd.Series(False, index=pose.index)).to_numpy(bool)
    estimated &= pose.calibration_valid.to_numpy(bool) & (counts >= 2)
    estimated &= ~valid
    estimated &= np.isfinite(front).all(axis=1) & np.isfinite(rear).all(axis=1)
    estimated_center = (front + rear)/2
    estimated_center[~estimated] = np.nan
    front[~valid] = np.nan; rear[~valid] = np.nan
    center = (front + rear)/2
    heading = np.degrees(np.arctan2((front-rear)[:, 1], (front-rear)[:, 0]))
    return pd.DataFrame(dict(frame=pose.frame, time_video_s=pose.time_video_s,
        calibration_valid=pose.calibration_valid, registered_marker_count=counts, position_valid=valid,
        front_x_mm=front[:, 0], front_y_mm=front[:, 1], rear_x_mm=rear[:, 0], rear_y_mm=rear[:, 1],
        center_x_mm=center[:, 0], center_y_mm=center[:, 1], apparent_heading_deg=heading,
        position_estimated=estimated, estimated_center_x_mm=estimated_center[:, 0], estimated_center_y_mm=estimated_center[:, 1],
        detection_method=pose.get('detection_method', pd.Series('colour_pair', index=pose.index)),
        body_ncc=pose.get('body_ncc', pd.Series(np.nan, index=pose.index))))


def estimate(video, reference_path, geometry_path, output, *, stride=1, start=0., end=None, detection_mode='profile'):
    video, reference_path, geometry_path, output = map(Path, (video, reference_path, geometry_path, output))
    if stride not in (1, 4):
        raise ValueError('stride must be 1 or 4')
    reference = json.loads(reference_path.read_text(encoding='utf-8'))
    geometry = load_detection_geometry(geometry_path, detection_mode)
    if geometry.get('mouse') not in ('A', 'B') or geometry.get('reference_id') != reference.get('reference_id'):
        raise ValueError('機体・盤面基準に対応する目印設定が必要です')
    colours = ('red', 'yellow', 'green', 'blue', 'magenta')
    front, rear = geometry.get('front_colour'), geometry.get('rear_colour')
    if front not in colours or rear not in colours or front == rear:
        raise ValueError('前後に異なる目印の色を指定してください')
    output.mkdir(parents=True, exist_ok=False)
    geometry = freeze_geometry_detection(geometry, output)
    def save(name, value):
        with (output/name).open('x', encoding='utf-8') as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
    summary = dict(analysis_mode='position_only', position_available=False, mouse=geometry['mouse'],
        video_file=str(video.resolve()), video_sha256=file_sha256(video), frame_stride=stride,
        video_window_s=[start, end], reference_id=reference['reference_id'],
        reference_sha256=file_sha256(reference_path), geometry_sha256=file_sha256(geometry_path),
        coordinate_system='reference board TL origin; X toward TR; Y toward BL; mm',
        center_kind='apparent_marker_midpoint', height_correction_applied=False,
        axle_position_calibrated=False, log_used=False, robot_commands_sent=False,
        parameter_adoption_allowed=False, missing_position_policy='NaN; no output interpolation',
        position_filter='none; image observations and learned estimates kept separate',
        heading_kind='apparent front/rear line; clockwise from board +X; not calibrated body yaw')
    save('marker_reference_input.json', reference); save('body_geometry_used.json', geometry)
    try:
        reference, search = prepare_reference(video, reference, start, end)
        summary['reference_search'] = search
        save('marker_reference_used.json', reference)
        summary['effective_reference_sha256'] = file_sha256(output/'marker_reference_used.json')
        if not search['available']:
            raise RuntimeError('盤面のAR座標合わせに失敗しました。登録IDを全て画角へ入れ、重複ID・反射・格子の遮蔽を確認してください。reference_searchに詳細があります')
        pose, _, fps, _ = read_video_pose(video, front, marker_reference=reference,
            rear_colour=rear, marker_detection=geometry.get('marker_detection'),
            nominal_marker_span_mm=geometry.get('nominal_marker_span_mm'),
            marker_span_limits_mm=geometry.get('marker_span_limits_mm'), video_window=(start, end),
            frame_stride=stride, track_only=True,
            progress=lambda stage, count, pts: print(f'{stage}: {count} frames, PTS {pts:.3f} s', flush=True))
        points = board_positions(pose, reference)
        points.to_csv(output/'position.csv', index=False)
        valid = points.position_valid.to_numpy(bool)
        inferred = points.position_estimated.to_numpy(bool)
        observed = points.loc[valid | inferred].copy()
        observed['center_x_mm'] = observed.center_x_mm.fillna(observed.estimated_center_x_mm)
        observed['center_y_mm'] = observed.center_y_mm.fillna(observed.estimated_center_y_mm)
        if observed.empty:
            raise ValueError('ARマーカーと前後の目印を同時に観測できませんでした')
        summary.update(position_available=True, frames=len(points), valid_frames=int(valid.sum()),
            observation_rate=float(valid.mean()), decoded_fps=fps,
            estimated_frames=int(inferred.sum()), position_coverage_rate=float((valid | inferred).mean()),
            body_detection=pose.attrs.get('body_detection', {}),
            marker_calibration=pose.attrs['marker_calibration'],
            endpoint_board_x_mm=float(observed.center_x_mm.iloc[-1]),
            endpoint_board_y_mm=float(observed.center_y_mm.iloc[-1]),
            endpoint_video_s=float(observed.time_video_s.iloc[-1]),
            endpoint_position_source='learned_body_estimate' if observed.position_estimated.iloc[-1] else 'observed_colour_pair',
            status='POSITION_ESTIMATED_REVIEW_REQUIRED')
        side = reference['board_side_mm']
        figure, axes = plt.subplots(1, 2, figsize=(10, 4.8), constrained_layout=True)
        axes[0].plot(points.center_x_mm, points.center_y_mm, color='tab:blue', linewidth=1)
        if inferred.any():
            axes[0].scatter(points.estimated_center_x_mm, points.estimated_center_y_mm,
                color='tab:orange', s=9, label='Learned body estimate')
        axes[0].scatter(observed.center_x_mm.iloc[0], observed.center_y_mm.iloc[0], c='tab:green', label='First available')
        axes[0].scatter(observed.center_x_mm.iloc[-1], observed.center_y_mm.iloc[-1], c='tab:red', label='Last available')
        axes[0].set(xlim=(0, side), ylim=(side, 0), xlabel='Board X [mm]', ylabel='Board Y [mm]',
            title='Marker midpoint on board (height uncorrected)', aspect='equal')
        pitch = reference.get('grid_refinement', {}).get('pitch_mm')
        if pitch:
            axes[0].set_xticks(np.arange(0, side+.01, pitch)); axes[0].set_yticks(np.arange(0, side+.01, pitch))
        axes[0].grid(alpha=.3); axes[0].legend(fontsize=8)
        axes[1].plot(points.time_video_s, points.center_x_mm, label='X')
        axes[1].plot(points.time_video_s, points.center_y_mm, label='Y')
        if inferred.any():
            axes[1].scatter(points.time_video_s[inferred], points.estimated_center_x_mm[inferred], color='tab:orange', s=9, label='Estimated X')
            axes[1].scatter(points.time_video_s[inferred], points.estimated_center_y_mm[inferred], color='tab:red', s=9, label='Estimated Y')
        axes[1].set(xlabel='Video PTS [s]', ylabel='Position [mm]', title=f'Observed {valid.mean():.1%}; gaps excluded')
        axes[1].grid(alpha=.3); axes[1].legend()
        figure.suptitle(f'Mouse {geometry["mouse"]} / video only / stride {stride}')
        try:
            figure.savefig(output/'position_trajectory.png', dpi=150)
        finally:
            plt.close(figure)
    except Exception as error:
        summary.update(position_available=False, status='POSITION_UNAVAILABLE', error=type(error).__name__+': '+str(error))
        save('summary.json', summary)
        raise
    save('summary.json', summary)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial', type=Path, help='保存済み試行から動画・参照・目印のパスだけを読む。CSVは使用しない')
    parser.add_argument('--video', type=Path)
    parser.add_argument('--reference', type=Path)
    parser.add_argument('--geometry', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--stride', type=int, choices=(1, 4), default=1)
    parser.add_argument('--start', type=float)
    parser.add_argument('--end', type=float)
    parser.add_argument('--detector', choices=('profile','auto','legacy'), default='profile')
    args = parser.parse_args()
    start = args.start if args.start is not None else 0.
    if args.trial:
        trial = json.loads(args.trial.read_text(encoding='utf-8'))
        args.video = args.video or Path(trial['video_file'])
        args.reference = args.reference or args.trial.parent/'marker_reference_used.json'
        args.geometry = args.geometry or args.trial.parent/'body_geometry_used.json'
        args.output = args.output or args.trial.parent/('full' if args.stride == 1 else 'quick')
        if args.start is None:
            start = max(0., trial.get('video_motion_start_estimate_s', 1.)-1.)
    if not all((args.video, args.reference, args.geometry, args.output)):
        parser.error('--video --reference --geometry --output または --trial を指定してください')
    estimate(args.video, args.reference, args.geometry, args.output, stride=args.stride, start=start, end=args.end, detection_mode=args.detector)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
