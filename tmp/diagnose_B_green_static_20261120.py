"""淡い緑の静止マスクを保存済み動画で検証する。実機への送信なし。"""
from pathlib import Path
import copy
import json
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'tools'), str(ROOT/'tools/analysis')]
from video_marker_pose import colour_components, marker_registration, board_to_pose_transform
from video_marker_pose import transform_points, select_marker_pair

BASE = ROOT/'tools/logs/pixel_camera/turn1600_B_r90_20261120_v1'
OUT = BASE/'green_repeat02_segmentation_diagnostic_v1'
OUT.mkdir(exist_ok=False)
geometry = json.loads((BASE/'body_geometry_B_green_front_v1.json').read_text(encoding='utf-8'))
reference = json.loads((BASE/'marker_reference_B_current_rotated.json').read_text(encoding='utf-8'))
tags = ['B_green_front_OFF02', 'green_Lstart13p6_repeat02_placed01', 'green_Lstart13p6_repeat02_placed02']
results = []
for tag in tags:
    capture = json.loads((BASE/(tag+'_static_capture.json')).read_text(encoding='utf-8'))
    video = cv2.VideoCapture(capture['video_file'])
    video.set(cv2.CAP_PROP_ORIENTATION_AUTO, 0)
    rows = []
    tiles = []
    for requested in np.arange(1., 3., .2):
        video.set(cv2.CAP_PROP_POS_MSEC, requested*1000)
        ok, frame = video.read()
        assert ok
        registration = marker_registration(frame, reference)
        assert registration['available']
        matrix = board_to_pose_transform(reference) @ np.array(registration['homography_image_to_board_mm'])
        fronts = []
        for saturation in (25, 20, 15):
            profile = copy.deepcopy(geometry['marker_detection'])
            profile['green']['minimum_saturation'] = saturation
            pair = select_marker_pair(colour_components(frame, 'green', profile),
                colour_components(frame, 'red', profile), matrix, None,
                nominal_marker_span_mm=50, marker_span_limits_mm=(40, 65))
            assert pair is not None
            pixels = transform_points(pair, np.linalg.inv(matrix))
            board = transform_points(pair, np.linalg.inv(board_to_pose_transform(reference)))
            heading = float(np.degrees(np.arctan2(*(board[0]-board[1])[::-1])))
            rows.append(dict(requested_pts=float(requested), saturation=saturation,
                front_image_px=pixels[0].tolist(), rear_image_px=pixels[1].tolist(), heading_deg=heading))
            fronts.append(pixels[0])
        # 色分割を重ね、閾値変更が実ステッカーを保つか確認する。
        for point, colour in zip(fronts, [(0,0,255), (0,255,255), (0,255,0)]):
            cv2.circle(frame, tuple(np.rint(point).astype(int)), 3, colour, 1)
        tile = cv2.resize(frame[185:222, 720:760], (240,222), interpolation=cv2.INTER_NEAREST)
        cv2.rectangle(tile, (0,0), (240,23), (0,0,0), -1)
        cv2.putText(tile, f'{requested:.1f}s S25red20yellow15green', (3,16),
            cv2.FONT_HERSHEY_SIMPLEX,.38,(255,255,255),1)
        tiles.append(tile)
    video.release()
    cv2.imwrite(str(OUT/(tag+'_threshold_points.jpg')), np.vstack([np.hstack(tiles[:5]), np.hstack(tiles[5:])]))
    results.append(dict(tag=tag, observations=rows, statistics=[dict(saturation=saturation,
        heading_median_deg=float(np.median([row['heading_deg'] for row in rows if row['saturation']==saturation])),
        heading_rms_about_median_deg=float(np.sqrt(np.mean((np.array([row['heading_deg'] for row in rows if row['saturation']==saturation])-
            np.median([row['heading_deg'] for row in rows if row['saturation']==saturation]))**2))))
        for saturation in (25,20,15)]))
with (OUT/'report.json').open('x',encoding='utf-8') as output:
    json.dump(dict(scope='固定10点×3動画。静止合格判定と独立走行検証の代用にはしない。',
        results=results), output, ensure_ascii=False, indent=2)
print(json.dumps([dict(tag=row['tag'], statistics=row['statistics']) for row in results], ensure_ascii=False))
