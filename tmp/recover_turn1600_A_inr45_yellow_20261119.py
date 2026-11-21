"""赤・黄の元観測を保持し、旋回付近で欠測した黄色だけを再検出する。"""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys

import cv2
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'tools'), str(ROOT/'tools/analysis')]
from pixel_camera_geometry import relative_marker_midpoint
from pixel_camera_relative_pose import required_motion_quality
from video_marker_pose import board_to_pose_transform, read_video_pose, transform_points


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def write(path, value):
    with path.open('x', encoding='utf-8') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)


def file_hash(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('out', type=Path)
    args = parser.parse_args()
    cv2.setNumThreads(2)
    original = args.run/'full'
    trial = read(args.run/'trial.json')
    report = read(original/'report.json')
    geometry = read(original/'body_geometry_used.json')
    reference = read(args.run/'marker_reference_used.json')
    pose_file = original/'apparent_floor_marker_pose.csv'
    pose = pd.read_csv(pose_file).set_index('frame')
    old_valid = pose.marker_detected_raw & pose.calibration_valid
    old_quality = report['required_motion_window_quality']
    start, end = old_quality['required_interval_pts_s']
    lower_saturation = deepcopy(geometry['marker_detection'])
    lower_saturation['yellow']['minimum_saturation'] = 20
    fresh, *_ = read_video_pose(Path(trial['video_file']), geometry['front_colour'], marker_reference=reference,
        video_window=(start-.05, end+.05), marker_detection=lower_saturation, rear_colour=geometry['rear_colour'],
        nominal_marker_span_mm=geometry['nominal_marker_span_mm'],
        marker_span_limits_mm=geometry['marker_span_limits_mm'])
    fresh = fresh.set_index('frame')
    good = pose[old_valid]
    recovery = []
    rejected = []
    missing = pose[pose.time_video_s.between(start,end) & pose.calibration_valid & ~pose.marker_detected_raw]
    for index, row in missing.iterrows():
        if index not in fresh.index or not (fresh.loc[index,'marker_detected_raw'] and fresh.loc[index,'calibration_valid']):
            continue
        candidate = fresh.loc[index]
        distance = np.hypot(*(candidate[key]-np.interp(row.time_video_s,good.time_video_s,good[key])
                              for key in ('center_x_mm','center_y_mm')))
        record = dict(frame=int(index), interpolation_distance_mm=float(distance))
        if distance > 2.5:
            rejected.append(record)
            continue
        for key in ('blue_x_mm','blue_y_mm','yellow_x_mm','yellow_y_mm','center_x_mm','center_y_mm',
                    'heading_rad','marker_separation_mm','front_image_x_px','front_image_y_px',
                    'midpoint_image_x_px','midpoint_image_y_px','front_floor_x_mm','front_floor_y_mm',
                    'midpoint_floor_x_mm','midpoint_floor_y_mm'):
            pose.loc[index,key] = candidate[key]
        pose.loc[index,'marker_detected_raw'] = True
        recovery.append(record)
    board = np.linalg.inv(board_to_pose_transform(reference))
    front = transform_points(pose[['blue_x_mm','blue_y_mm']].to_numpy(),board)
    rear = transform_points(pose[['yellow_x_mm','yellow_y_mm']].to_numpy(),board)
    time = pose.time_video_s.to_numpy()
    result = relative_marker_midpoint(front, rear, trial['camera_height_mm'], geometry['front_height_mm'],
        geometry['rear_height_mm'], (pose.marker_detected_raw & pose.calibration_valid).to_numpy(bool))
    xy, valid = result['displacement_xy_mm'], result['valid']
    baseline = valid & (time <= time[0]+.2)
    if baseline.sum() >= 10:
        xy -= np.nanmedian(xy[baseline], axis=0)
    quality = required_motion_quality(time, valid, pose.calibration_valid.to_numpy(bool),
        pd.read_csv(trial['type8_log_csv'],usecols=['time_ms']).time_ms.to_numpy(), trial,
        report['log_time_zero_at_video_s'])
    args.out.mkdir(parents=True, exist_ok=False)
    pose.to_csv(args.out/'apparent_floor_marker_pose.csv')
    pd.DataFrame(dict(frame=pose.index,time_video_s=time,detected=valid,
        center_dx_mm=xy[:,0],center_dy_mm=xy[:,1])).to_csv(args.out/'relative_marker_midpoint.csv',index=False)
    write(args.out/'body_geometry_used.json', geometry)
    provenance = dict(source_pose=str(pose_file.resolve()), source_sha256=file_hash(pose_file),
        profile_for_missing_only=lower_saturation, candidate_max_interpolation_distance_mm=2.5,
        recovered=recovery,rejected=rejected, old_quality=old_quality, new_quality=quality)
    report.update(status='REANALYZED_REVIEW_REQUIRED',required_motion_window_quality=quality,
                  recovery=provenance, parameter_adoption_allowed=False)
    write(args.out/'report.json', report)
    print(json.dumps(dict(recovered=len(recovery), rejected=len(rejected), rate=quality['observation_rate']),
                     ensure_ascii=False),flush=True)


if __name__=='__main__':
    main()
