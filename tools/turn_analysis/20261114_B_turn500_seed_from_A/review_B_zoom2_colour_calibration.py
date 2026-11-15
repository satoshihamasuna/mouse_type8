"""機体OFFの固定窓で1倍/2倍の静止精度を比較する。機器への指令なし。"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
BASE = Path(__file__).resolve().parent
sys.path[:0] = [str(ROOT/'tools'), str(ROOT/'tools/analysis')]
from video_marker_pose import read_video_pose


def load(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--zoom', type=int, choices=(1, 2), required=True)
    args = parser.parse_args()
    folder = BASE/'camera_zoom2_colour_calibration_v6'
    captures = load(folder/'captures.json')
    capture = next(r for r in captures['records'] if r['zoom_ratio'] == args.zoom)
    config = load(BASE/'tuning_cli_config_B_inR135_Aseed_v5.json')
    reference_path = Path(config['reference']) if args.zoom == 1 else folder/'marker_reference_B_zoom2.json'
    config['geometry'] = str(BASE/'body_geometry_B_zoom2_v6.json')
    reference, geometry = load(reference_path), load(config['geometry'])
    output = folder/f'zoom{args.zoom}_full_static'
    output.mkdir(exist_ok=False)
    protocol = dict(window_s=[1.2, 2.4], frame_stride=1, minimum_frames=280,
        minimum_detection_and_calibration_rate=.99, maximum_scatter_rms_mm=.5,
        maximum_deviation_mm=1.5, comparative_midpoint_limit_mm=.75,
        comparative_apparent_heading_limit_deg=1,
        scope='B/HY OFF static only; independent yaw, absolute axle and running visibility unverified')
    (output/'protocol.json').write_text(json.dumps(protocol, indent=2)+'\n', encoding='utf-8')
    cap = load(capture['capture_json'])
    metadata_path = Path(cap['metadata_file'])
    metadata = load(metadata_path)
    assert metadata['zoom_ratio_requested'] == args.zoom
    assert metadata['zoom_verification']['confirmed'] and metadata['focus_lock']['state'] == 'LOCKED'
    paths = [Path(capture['video_file']), Path(capture['capture_json']), metadata_path,
        reference_path, Path(config['geometry']), Path(__file__),
        ROOT/'tools/pixel_camera_reference.py', ROOT/'tools/analysis/pixel_camera_accuracy.py',
        ROOT/'tools/analysis/video_marker_pose.py']
    inputs = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    (output/'input_sha256.json').write_text(json.dumps(inputs, indent=2)+'\n', encoding='utf-8')
    pose, _, _, _ = read_video_pose(Path(capture['video_file']), front_colour='yellow', rear_colour='red',
        marker_reference=reference, marker_detection=geometry['marker_detection'],
        nominal_marker_span_mm=geometry['nominal_marker_span_mm'],
        marker_span_limits_mm=geometry['marker_span_limits_mm'], video_window=(1.2, 2.4), frame_stride=1,
        progress=lambda stage, count, pts: print(f'{stage}: {count} frames', flush=True))
    pose.to_csv(output/'pose.csv', index=False)
    good = pose.marker_detected_raw & pose.calibration_valid
    points = pose.loc[good, ['center_x_mm', 'center_y_mm']].to_numpy(float)
    assert len(points)
    median = np.median(points, axis=0)
    distance = np.linalg.norm(points-median, axis=1)
    rms = float(np.sqrt(np.mean(distance**2)))
    front = pose.loc[good, ['blue_x_mm', 'blue_y_mm']].median().to_numpy(float)
    rear = pose.loc[good, ['yellow_x_mm', 'yellow_y_mm']].median().to_numpy(float)
    direction = front-rear
    accepted = len(pose) >= 280 and good.mean() >= .99 and rms <= .5 and distance.max() <= 1.5
    result = dict(state='PASSED' if accepted else 'FAILED', zoom_ratio=args.zoom,
        frames=len(pose), detection_and_calibration_rate=float(good.mean()),
        calibration_rate=float(pose.calibration_valid.mean()), scatter_rms_mm=rms,
        maximum_deviation_mm=float(distance.max()), midpoint_pose_xy_mm=median.tolist(),
        front_median_pose_xy_mm=front.tolist(), rear_median_pose_xy_mm=rear.tolist(),
        apparent_heading_pose_deg=float(np.degrees(np.arctan2(direction[1], direction[0]))),
        marker_span_median_mm=float(pose.loc[good, 'marker_separation_mm'].median()),
        protocol=protocol, input_sha256=inputs, robot_commands_sent=0)
    (output/'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
