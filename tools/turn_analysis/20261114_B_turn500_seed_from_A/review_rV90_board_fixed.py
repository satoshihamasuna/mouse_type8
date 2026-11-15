"""保存済み右135出口の盤面固定診断。入口へ軸を回転しない。実機操作なし。"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT/'tools'), str(ROOT/'tools/analysis')]
from video_marker_pose import board_to_pose_transform, transform_points, log_elapsed_seconds

def load(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))

def diagnose(run, plan_path):
    report = load(run/'full/report.json')
    trial = load(run/'trial.json')
    settings = load(run/'motion.settings.json')
    plan = load(plan_path)
    assert settings['mouse'] == trial['mouse'] == plan['mouse'] == 'B'
    assert settings['motion_name'] == plan['motion'] == 'r_v90'
    assert report['frame_stride'] == 1
    inverse = np.linalg.inv(board_to_pose_transform(load(run/'marker_reference_used.json')))
    pose = pd.read_csv(run/'full/apparent_floor_marker_pose.csv')
    log = pd.read_csv(run/'motion.csv')
    time = log_elapsed_seconds(log)
    start, end = [(trial['log_quality'][k]-float(log.time_ms.iloc[0]))%2**32/1000
                  for k in ['motion_start_time_ms', 'motion_end_time_ms']]
    pt = pose.time_video_s.to_numpy()-report['log_time_zero_at_video_s']
    good = (pose.marker_detected_raw & pose.calibration_valid).to_numpy(bool)
    xy = transform_points(pose[['center_x_mm', 'center_y_mm']].to_numpy(float), inverse)
    om = log['ideal.rad_velo'].to_numpy()
    indices = np.flatnonzero(abs(om) >= .02*abs(om).max())
    def angle(mask):
        points = xy[mask]
        assert len(points) >= 15
        direction = np.linalg.svd(points-points.mean(axis=0), full_matrices=False)[2][0]
        if direction@(points[-1]-points[0]) < 0:
            direction = -direction
        assert np.ptp(points@direction) >= 30
        return float(np.degrees(np.arctan2(direction[1], direction[0]))%360)
    moving = np.interp(pt, time, log['ideal.velo']) > .25
    entry = angle(good & (pt >= start) & (pt < time[indices[0]]-.015) & moving)
    exit_angle = angle(good & (pt > time[indices[-1]]+.03) & (pt <= end-.02) & moving)
    pre = good & (pt >= start-.35) & (pt <= start-.15)
    post = good & (pt >= end+.2) & (pt <= end+.6)
    assert pre.sum() >= 20 and post.sum() >= 20
    before, after = np.median(xy[pre], axis=0), np.median(xy[post], axis=0)
    paths = [run/'trial.json', run/'motion.settings.json', run/'motion.csv', run/'full/report.json',
             run/'marker_reference_used.json', run/'full/apparent_floor_marker_pose.csv', plan_path, Path(__file__)]
    return dict(evaluation_frame='board_fixed_apparent_floor_midpoint_diagnostic_only',
        planned_start_board_xy_mm=plan['initial_position_board_xy_mm'], observed_start_board_xy_mm=before.tolist(),
        planned_stop_board_xy_mm=plan['planned_stop_board_xy_mm'], observed_stop_board_xy_mm=after.tolist(),
        stop_error_xy_mm=(after-np.array(plan['planned_stop_board_xy_mm'])).tolist(), entry_angle_board_deg=entry,
        entry_error_from135_deg=(entry-135+180)%360-180, exit_angle_board_deg=exit_angle,
        exit_error_from225_deg=(exit_angle-225+180)%360-180, coordinates_aligned_to_robot=False,
        absolute_axle_or_height_corrected_position_verified=False,
        input_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths})

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = diagnose(args.run, args.plan)
    with args.output.open('x', encoding='utf-8', newline='\n') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({k: result[k] for k in ['stop_error_xy_mm', 'entry_error_from135_deg', 'exit_error_from225_deg']}))
