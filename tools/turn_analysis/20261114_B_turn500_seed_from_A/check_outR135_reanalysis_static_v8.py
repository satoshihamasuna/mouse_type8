"""B右135度入口の保存済み静止動画を固定窓・全フレームで照合する。機器操作なし。"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT/'tools'), str(ROOT/'tools/analysis')]
from video_marker_pose import read_video_pose, board_to_pose_transform, transform_points


def load(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial', type=Path, required=True)
    parser.add_argument('--geometry-override', type=Path, required=True)
    parser.add_argument('--analysis-override', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--prepare-only', action='store_true')
    parser.add_argument('--cache-folder', type=Path)
    args = parser.parse_args()
    run = args.trial.resolve()
    session = run.parent
    config = load(session/'config.used.json')
    static = load(session/'static_preflight.json')
    reference, geometry = load(config['reference']), load(args.geometry_override)
    assert config['mouse'] == geometry['mouse'] == 'B'
    assert static['state'] == 'PASSED'
    assert geometry['front_colour'] == 'yellow' and geometry['rear_colour'] == 'red'
    args.output.mkdir(exist_ok=False)
    protocol = dict(static_window_s=[1.2, 2.4], frame_stride=1, minimum_frames=280,
        minimum_detection_and_calibration_rate=.99, maximum_static_scatter_rms_mm=.5,
        maximum_static_deviation_mm=1.5, maximum_centre_change_mm=3,
        maximum_heading_change_deg=1, window_selection='Fixed before dense decode; includes every frame',
        original_sparse_report_preserved=True, grid_and_colour_thresholds_unchanged=True)
    (args.output/'protocol.json').write_text(json.dumps(protocol, indent=2)+'\n', encoding='utf-8')
    provenance = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in
        [Path(static['video_file']), Path(config['reference']), args.geometry_override,
         ROOT/'tools/analysis/video_marker_pose.py', ROOT/'tools/pixel_camera_reference.py', Path(__file__)]}
    if args.cache_folder:
        assert not args.prepare_only
        cached = load(args.cache_folder/'cache_provenance.json')
        assert cached['input_sha256'] == provenance and cached['protocol'] == protocol
        cached_csv = args.cache_folder/'static_pose_all_frames.csv'
        assert hashlib.sha256(cached_csv.read_bytes()).hexdigest() == cached['pose_csv_sha256']
        pose = pd.read_csv(cached_csv)
    else:
        pose, _, _, _ = read_video_pose(Path(static['video_file']), front_colour='yellow', rear_colour='red',
            marker_reference=reference, marker_detection=geometry['marker_detection'],
            nominal_marker_span_mm=geometry['nominal_marker_span_mm'],
            marker_span_limits_mm=geometry['marker_span_limits_mm'], video_window=(1.2, 2.4), frame_stride=1)
    pose.to_csv(args.output/'static_pose_all_frames.csv', index=False)
    if args.prepare_only:
        cache = dict(state='STATIC_CACHE_PREPARED_NO_MOTION_VALIDATION', protocol=protocol,
            input_sha256=provenance, frames=len(pose),
            pose_csv_sha256=hashlib.sha256((args.output/'static_pose_all_frames.csv').read_bytes()).hexdigest())
        (args.output/'cache_provenance.json').write_text(json.dumps(cache, indent=2)+'\n', encoding='utf-8')
        print(json.dumps(dict(state=cache['state'], frames=len(pose))), flush=True)
        return
    good = pose.marker_detected_raw & pose.calibration_valid
    points = pose.loc[good, ['center_x_mm', 'center_y_mm']].to_numpy(float)
    median = np.median(points, axis=0)
    distances = np.linalg.norm(points-median, axis=1)
    inverse = np.linalg.inv(board_to_pose_transform(reference))
    front = pose.loc[good, ['blue_x_mm', 'blue_y_mm']].median().to_numpy(float)
    rear = pose.loc[good, ['yellow_x_mm', 'yellow_y_mm']].median().to_numpy(float)
    front, rear = transform_points([front, rear], inverse)
    centre = (front+rear)/2
    direction = (front-rear)/np.linalg.norm(front-rear)
    report = load(args.analysis_override/'report.json')
    trial = load(run/'trial.json')
    log = pd.read_csv(run/'motion.csv')
    motion_start = report['log_time_zero_at_video_s']+(trial['log_quality']['motion_start_time_ms']-float(log.time_ms.iloc[0]))%2**32/1000
    recorded = pd.read_csv(args.analysis_override/'apparent_floor_marker_pose.csv')
    before = (recorded.marker_detected_raw & recorded.calibration_valid
              & recorded.time_video_s.between(motion_start-.35, motion_start-.15))
    assert before.sum() >= 20
    rf = recorded.loc[before, ['blue_x_mm', 'blue_y_mm']].median().to_numpy(float)
    rr = recorded.loc[before, ['yellow_x_mm', 'yellow_y_mm']].median().to_numpy(float)
    rf, rr = transform_points([rf, rr], inverse)
    recorded_centre = (rf+rr)/2
    recorded_direction = (rf-rr)/np.linalg.norm(rf-rr)
    delta = float(np.linalg.norm(recorded_centre-centre))
    angle = float((math.degrees(math.atan2(recorded_direction[1], recorded_direction[0])
        -math.atan2(direction[1], direction[0]))+180)%360-180)
    rms = float(np.sqrt(np.mean(distances**2)))
    accepted = bool(len(pose) >= 280 and good.mean() >= .99 and rms <= .5
        and distances.max() <= 1.5 and delta <= 3 and abs(angle) <= 1)
    paths = [Path(static['video_file']), session/'static_preflight.json',
        session/'static_preflight_check.json', Path(config['reference']), args.geometry_override,
        run/'trial.json', args.analysis_override/'report.json', args.analysis_override/'apparent_floor_marker_pose.csv', Path(__file__)]
    if args.cache_folder:
        paths += [args.cache_folder/'cache_provenance.json', args.cache_folder/'static_pose_all_frames.csv']
    result = dict(state='PASSED' if accepted else 'FAILED', protocol=protocol,
        frames=len(pose), detection_and_calibration_rate=float(good.mean()),
        stationary_scatter_rms_mm=rms, maximum_stationary_deviation_mm=float(distances.max()),
        static_centre_board_xy_mm=centre.tolist(), static_forward_board_xy=direction.tolist(),
        recorded_centre_board_xy_mm=recorded_centre.tolist(),
        centre_change_since_dense_static_mm=delta, heading_change_since_dense_static_deg=angle,
        recorded_prerun_observations=int(before.sum()),
        scope='Additional fixed-window dense measurement; sparse failure preserved, apparent marker yaw only',
        input_sha256={str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in paths})
    (args.output/'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(result), flush=True)
    assert accepted, 'Dense static comparison failed; retain trial as excluded'


if __name__ == '__main__':
    main()
