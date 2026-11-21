"""走行とは独立した保存済み静止動画で黄色検出候補を照合する。"""
from pathlib import Path
import json
import sys
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'tools/analysis'), str(ROOT / 'tools')]
from video_marker_pose import (marker_registration, registration_quad, pose_world_corners,
    colour_components, select_marker_pair, board_to_pose_transform, transform_points)

BASE = ROOT / 'tools/logs/pixel_camera/turn1600_A_r90_20261118_frame18_v1'
FOLDER = BASE / 'analysis_yellow_v2'
capture = json.loads((BASE / 'corr2p5_static01/static_capture.json').read_text(encoding='utf-8'))
reference = json.loads((BASE / 'marker_reference_A_current.json').read_text(encoding='utf-8'))
old = json.loads((BASE / 'body_geometry_A_current.json').read_text(encoding='utf-8'))
new = json.loads((FOLDER / 'body_geometry_A_yellow_v2.json').read_text(encoding='utf-8'))
inverse = np.linalg.inv(board_to_pose_transform(reference))
video = cv2.VideoCapture(capture['video_file'])
video.set(cv2.CAP_PROP_ORIENTATION_AUTO, 0)
records = []
for stamp in np.linspace(1., 2.9, 13):
    video.set(cv2.CAP_PROP_POS_MSEC, stamp * 1000)
    ok, frame = video.read()
    if not ok:
        raise RuntimeError('静止動画のフレームが不足しています')
    registration = marker_registration(frame, reference)
    record = dict(time_video_s=float(stamp), registered_ids=registration.get('used_marker_ids', []),
                  calibration_valid=registration['available'])
    if registration['available']:
        homography = cv2.getPerspectiveTransform(registration_quad(registration), pose_world_corners(360.))
        for name, profile in [('old', old), ('new', new)]:
            detection = profile['marker_detection']
            pair = select_marker_pair(colour_components(frame, 'red', detection),
                colour_components(frame, 'yellow', detection), homography, None,
                nominal_marker_span_mm=50., marker_span_limits_mm=[40., 65.])
            record[name + '_detected'] = pair is not None
            if pair is not None:
                front, rear = transform_points(pair, inverse)
                record[name + '_front_board_mm'] = front.tolist()
                record[name + '_rear_board_mm'] = rear.tolist()
                record[name + '_midpoint_board_mm'] = ((front + rear) / 2).tolist()
                record[name + '_span_mm'] = float(np.linalg.norm(front - rear))
    records.append(record)
video.release()
centres = np.array([r['new_midpoint_board_mm'] for r in records if r.get('new_detected')])
agreement = [np.linalg.norm(np.array(r['new_midpoint_board_mm']) - r['old_midpoint_board_mm'])
             for r in records if r.get('new_detected') and r.get('old_detected')]
result = dict(mouse='A', independent_static_video=capture['video_file'], frames=len(records),
    calibration_rate=float(np.mean([r['calibration_valid'] for r in records])),
    old_detection_rate=float(np.mean([r.get('old_detected', False) for r in records])),
    new_detection_rate=float(np.mean([r.get('new_detected', False) for r in records])),
    new_midpoint_board_mm=np.median(centres, axis=0).tolist(),
    new_midpoint_scatter_rms_mm=float(np.sqrt(np.sum(np.var(centres, axis=0)))),
    largest_midpoint_difference_on_old_detected_mm=float(max(agreement)) if agreement else None,
    unchanged_span_limits_mm=[40, 65], records=records,
    scope='独立した静止動画。実際の前赤/後黄ディスク位置と照合。動的な全フレーム検証は別途必要。')
with (FOLDER / 'independent_static_validation.json').open('x', encoding='utf-8') as output:
    json.dump(result, output, ensure_ascii=False, indent=2)
    output.write('\n')
print(json.dumps({key: value for key, value in result.items() if key != 'records'}, ensure_ascii=False, indent=2), flush=True)
