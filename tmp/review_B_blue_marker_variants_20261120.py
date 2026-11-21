"""保存済みの固定10フレームで青マーカーの色条件を比較する。機器操作なし。"""
from pathlib import Path
import copy
import json
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'tools'), str(ROOT/'tools/analysis')]
from video_marker_pose import (marker_registration, board_to_pose_transform,
    colour_components, select_marker_pair, transform_points)

BASE = ROOT/'tools/logs/pixel_camera/turn1600_B_r90_20261120_v1'
def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))

config = read(BASE/'config.blue_front.v1.json')
geometry = read(config['geometry'])
reference = read(config['reference'])
capture = read(BASE/'B_blue_front_OFF01_static_capture.json')
video = cv2.VideoCapture(capture['video_file'])
video.set(cv2.CAP_PROP_ORIENTATION_AUTO, 0)
variants = [('wide_S35', 35, [90, 125]), ('wide_S45', 45, [90, 125]),
            ('narrow_S45', 45, [100, 115]), ('narrow_S50', 50, [100, 115])]
records = []
for requested_pts in np.arange(1., 3., .2):
    video.set(cv2.CAP_PROP_POS_MSEC, requested_pts*1000)
    ok, frame = video.read()
    assert ok
    registration = marker_registration(frame, reference)
    assert registration['available']
    matrix = (board_to_pose_transform(reference)
              @ np.array(registration['homography_image_to_board_mm']))
    rears = colour_components(frame, 'red', geometry['marker_detection'])
    for name, saturation, hue in variants:
        detection = copy.deepcopy(geometry['marker_detection'])
        detection['blue'].update(minimum_saturation=saturation, hue_ranges=[hue])
        fronts = colour_components(frame, 'blue', detection)
        pair = select_marker_pair(fronts, rears, matrix, None,
            nominal_marker_span_mm=50, marker_span_limits_mm=[40, 65])
        pixel = transform_points(pair, np.linalg.inv(matrix))[0].tolist() if pair is not None else None
        correct = bool(pixel and 735 <= pixel[0] <= 746 and 205 <= pixel[1] <= 216)
        records.append(dict(requested_pts=float(requested_pts), variant=name,
                            front_pixel=pixel, on_actual_round_marker=correct))
video.release()
summary = {}
for name, _, _ in variants:
    selected = [row for row in records if row['variant'] == name]
    points = np.asarray([row['front_pixel'] for row in selected if row['front_pixel']])
    summary[name] = dict(correct=sum(row['on_actual_round_marker'] for row in selected),
        frames=len(selected), front_x_range_px=float(np.ptp(points[:, 0])),
        front_y_range_px=float(np.ptp(points[:, 1])))
with (BASE/'blue_front_fixed10_colour_variants_v1.json').open('x', encoding='utf-8') as output:
    json.dump(dict(manual_round_marker_box_px=[735, 205, 746, 216],
        comparison_scope='Fixed10 saved OFF frames; round-marker location from actual image. LED-on and independent static unverified.',
        summary=summary, frames=records), output, indent=2)
print(json.dumps(summary, indent=2), flush=True)
