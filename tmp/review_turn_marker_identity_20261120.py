"""保存済み動画の固定8点で、前後の実検出点と走行LEDを目視照合する。"""
from pathlib import Path
import json
import sys

import cv2
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'tools'), str(ROOT/'tools/analysis')]
from video_marker_pose import marker_registration, board_to_pose_transform, transform_points

BASE = ROOT/'tools/logs/pixel_camera/turn1600_B_r90_20261120_v1'
RUN = (BASE/sys.argv[1]).resolve()
assert RUN.is_relative_to(BASE.resolve())
trial = json.loads((RUN/'trial.json').read_text(encoding='utf-8'))
reference = json.loads((RUN/'marker_reference_used.json').read_text(encoding='utf-8'))
report = json.loads((RUN/'quick/report.json').read_text(encoding='utf-8'))
pose = pd.read_csv(RUN/'quick/apparent_floor_marker_pose.csv')
start = report['required_motion_window_quality']['video_motion_start_s']
late = (trial['recording_stop']['confirmation_camera_seconds']
        + trial['recording_stop']['decision_camera_seconds'])/2
requested = [start+.07, start+.20, start+.245, start+.265,
             start+.285, start+.315, start+.41, late]
video = cv2.VideoCapture(trial['video_file'])
video.set(cv2.CAP_PROP_ORIENTATION_AUTO, 0)
tiles, observations = [], []
for requested_pts in requested:
    row = pose.loc[(pose.time_video_s-requested_pts).abs().idxmin()]
    assert row.marker_detected_raw and row.calibration_valid
    video.set(cv2.CAP_PROP_POS_FRAMES, int(row.frame))
    ok, frame = video.read()
    assert ok
    actual_pts = video.get(cv2.CAP_PROP_POS_MSEC)/1000
    assert abs(actual_pts-row.time_video_s) < .002
    registration = marker_registration(frame, reference)
    assert registration['available']
    matrix = (board_to_pose_transform(reference)
              @ np.array(registration['homography_image_to_board_mm']))
    pixels = transform_points([[row.blue_x_mm, row.blue_y_mm],
                               [row.yellow_x_mm, row.yellow_y_mm]], np.linalg.inv(matrix))
    for point, colour in zip(pixels, [(0,255,0), (255,0,255)]):
        cv2.circle(frame, tuple(np.rint(point).astype(int)), 5, colour, 1, cv2.LINE_AA)
    center = np.rint(pixels.mean(axis=0)).astype(int)
    x = int(np.clip(center[0]-90, 0, frame.shape[1]-180))
    y = int(np.clip(center[1]-90, 0, frame.shape[0]-180))
    tile = cv2.resize(frame[y:y+180, x:x+180], (360,360))
    cv2.rectangle(tile, (0,0), (360,46), (0,0,0), -1)
    cv2.putText(tile, f'PTS {actual_pts:.4f}s frame{int(row.frame)}', (7,18),
        cv2.FONT_HERSHEY_SIMPLEX, .45, (255,255,255), 1, cv2.LINE_AA)
    cv2.putText(tile, f'LED blue pixels {int(row.turn_led_blue_pixels)}', (7,38),
        cv2.FONT_HERSHEY_SIMPLEX, .45, (255,255,255), 1, cv2.LINE_AA)
    tiles.append(tile)
    observations.append(dict(requested_pts=float(requested_pts), actual_pts=float(actual_pts),
        frame=int(row.frame), front_image_px=pixels[0].tolist(), rear_image_px=pixels[1].tolist(),
        actual_decoded_ids=registration['detected_marker_ids'],
        turn_led_blue_pixels=int(row.turn_led_blue_pixels)))
video.release()
cv2.imwrite(str(RUN/'fixed_LED_marker_identity_montage.jpg'),
            np.vstack([np.hstack(tiles[:4]), np.hstack(tiles[4:])]))
with (RUN/'fixed_LED_marker_identity_metadata.json').open('x', encoding='utf-8') as output:
    json.dump(dict(source_video=trial['video_file'], source_pose='quick/apparent_floor_marker_pose.csv',
        front_outline='green', rear_outline='magenta', manual_review_required=True,
        scope='Fixed8 source-frame identity check, not full-frame quality validation.',
        frames=observations), output, indent=2)
print(json.dumps(dict(frames=8, LED_pixels=[row['turn_led_blue_pixels'] for row in observations])), flush=True)
