"""右135度入口の全フレーム動画と静止試写の姿勢を照合する。機器操作なし。"""
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
from video_marker_pose import board_to_pose_transform, transform_points


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    run = args.trial.resolve()
    session = run.parent
    trial = json.loads((run/'trial.json').read_text(encoding='utf-8'))
    report = json.loads((run/'full/report.json').read_text(encoding='utf-8'))
    settings = json.loads((run/'motion.settings.json').read_text(encoding='utf-8'))
    assert trial['mouse'] == settings['mouse'] == 'B'
    assert settings['motion_name'] == 'in_r135' and report['frame_stride'] == 1
    static = json.loads((session/'static_preflight_check.json').read_text(encoding='utf-8'))
    assert static['ready']
    reference = json.loads((run/'marker_reference_used.json').read_text(encoding='utf-8'))
    log = pd.read_csv(run/'motion.csv')
    pose = pd.read_csv(run/'full/apparent_floor_marker_pose.csv')
    start = (trial['log_quality']['motion_start_time_ms']-float(log.time_ms.iloc[0]))%2**32/1000
    video_start = report['log_time_zero_at_video_s']+start
    valid = (pose.marker_detected_raw & pose.calibration_valid
             & pose.time_video_s.between(video_start-.35, video_start-.15))
    assert valid.sum() >= 20
    # 保存列名blue/yellowは設定された前後マーカー。Bでは前が黄・後が赤。
    front = pose.loc[valid, ['blue_x_mm', 'blue_y_mm']].median().to_numpy(float)
    rear = pose.loc[valid, ['yellow_x_mm', 'yellow_y_mm']].median().to_numpy(float)
    front, rear = transform_points(np.vstack([front, rear]),
        np.linalg.inv(board_to_pose_transform(reference)))
    centre = (front+rear)/2
    forward = (front-rear)/np.linalg.norm(front-rear)
    static_centre = np.median([item['centre_board_xy_mm'] for item in static['frames']], axis=0)
    static_forward = np.median([item['forward_board_xy'] for item in static['frames']], axis=0)
    distance = float(np.linalg.norm(centre-static_centre))
    angle = float((math.degrees(math.atan2(forward[1], forward[0])
        -math.atan2(static_forward[1], static_forward[0]))+180)%360-180)
    paths = [run/'trial.json', run/'motion.settings.json', run/'full/report.json',
        run/'full/apparent_floor_marker_pose.csv', session/'static_preflight_check.json',
        run/'marker_reference_used.json', Path(__file__)]
    result = dict(state='PASSED' if distance <= 3 and abs(angle) <= 1 else 'FAILED',
        observations=int(valid.sum()), full_frame_scope=True, centre_board_xy_mm=centre.tolist(),
        forward_board_xy=forward.tolist(), centre_change_since_static_mm=distance,
        heading_change_since_static_deg=angle, maximum_centre_change_mm=3, maximum_heading_change_deg=1,
        scope='Apparent floor marker pose transformed to top-left/down-positive board axes; no height-corrected body yaw claim',
        input_sha256={str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in paths})
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(result))
    assert result['state'] == 'PASSED', 'Recorded setup changed since checked static pose; preserve trial'


if __name__ == '__main__':
    main()
