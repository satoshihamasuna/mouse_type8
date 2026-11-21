"""保存済み静止動画でBの色検出を比較する。機器操作・設定反映は行わない。"""
from pathlib import Path
import copy
import hashlib
import json
import sys

import cv2
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT/'tools/logs/pixel_camera/turn2000_B_r90_20261120_v1'
OUT = BASE/'marker_detection_sweep_v1'
OUT.mkdir(exist_ok=False)
sys.path[:0] = [str(ROOT/'tools'), str(ROOT/'tools/analysis')]
from video_marker_pose import marker_registration, colour_components, select_marker_pair, transform_points


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


config = read(BASE/'config.json')
reference, geometry = read(config['reference']), read(config['geometry'])
variants = []
for saturation in (5, 10, 15, 20, 25, 30, 40, 60):
    for hue in ((25, 65), (20, 75)):
        for opening in (0, 3):
            name = f's{saturation}_h{hue[0]}_{hue[1]}_o{opening}'
            detection = copy.deepcopy(geometry['marker_detection'])
            detection['yellow'].update(minimum_saturation=saturation, hue_ranges=[list(hue)],
                opening_kernel_size=opening, opening_kernel_shape='ellipse')
            variants.append((name, detection))

clips = ('powered03', 'ki005_repeat02_off', 'ki005_repeat02_settled_off', 'powered04')
rows, source_files = [], []
for tag in clips:
    capture_file = BASE/(tag+'_static_capture.json')
    capture = read(capture_file)
    assert capture['state'] == 'PASSED'
    source_files.append(dict(capture=str(capture_file), video=capture['video_file'],
        capture_sha256=hashlib.sha256(capture_file.read_bytes()).hexdigest()))
    cap = cv2.VideoCapture(capture['video_file'])
    cap.set(cv2.CAP_PROP_ORIENTATION_AUTO, 0)
    for target_s in (1.1, 1.4, 1.7, 2., 2.3, 2.7):
        cap.set(cv2.CAP_PROP_POS_MSEC, target_s*1000)
        ok, frame = cap.read()
        assert ok
        pts = cap.get(cv2.CAP_PROP_POS_MSEC)/1000
        registration = marker_registration(frame, reference)
        assert registration['available']
        h = np.asarray(registration['homography_image_to_board_mm'])
        rears = colour_components(frame, 'red', geometry['marker_detection'])
        for name, detection in variants:
            pair = select_marker_pair(colour_components(frame, 'yellow', detection), rears, h, None,
                board_side_mm=360, nominal_marker_span_mm=50,
                marker_span_limits_mm=geometry['marker_span_limits_mm'])
            row = dict(clip=tag, pts_s=pts, variant=name, observed=pair is not None)
            if pair is not None:
                front, rear = pair
                pixels = transform_points([front, rear], np.linalg.inv(h))
                centre = (front+rear)/2
                row.update(heading_deg=float(np.degrees(np.arctan2(*(front-rear)[::-1]))%360),
                    x_mm=float(centre[0]), y_mm=float(centre[1]), span_mm=float(np.linalg.norm(front-rear)),
                    front_x_px=float(pixels[0,0]), front_y_px=float(pixels[0,1]),
                    rear_x_px=float(pixels[1,0]), rear_y_px=float(pixels[1,1]))
            rows.append(row)
    cap.release()
    print('sampled '+tag, flush=True)

data = pd.DataFrame(rows)
data.to_csv(OUT/'sampled_points.csv', index=False)
summary = []
for name, detection in variants:
    clip_metrics = []
    for tag in clips:
        sample = data[(data['variant'] == name)&(data['clip'] == tag)]
        valid = sample[sample.observed]
        record = dict(clip=tag, observations=len(valid), frames=len(sample))
        if len(valid):
            median = float(np.median(valid.heading_deg))
            jitter = ((valid.heading_deg-median+180)%360-180).to_numpy()
            record.update(heading_deg=median, heading_jitter_rms_deg=float(np.sqrt(np.mean(jitter**2))),
                heading_jitter_max_deg=float(np.abs(jitter).max()),
                center_mm=valid[['x_mm','y_mm']].median().tolist(),
                median_front_px=valid[['front_x_px','front_y_px']].median().tolist(),
                span_range_mm=[float(valid.span_mm.min()),float(valid.span_mm.max())])
        clip_metrics.append(record)
    complete = all(m['frames'] == 6 and m['observations'] == 6 for m in clip_metrics)
    worst = max(m.get('heading_jitter_rms_deg',999) for m in clip_metrics)
    summary.append(dict(variant=name,detection=detection,complete=complete,
        worst_sampled_heading_jitter_rms_deg=worst,clips=clip_metrics))
summary.sort(key=lambda r:(not r['complete'],r['worst_sampled_heading_jitter_rms_deg']))
result = dict(mouse='B',sources=source_files,reference=config['reference'],geometry=config['geometry'],
    scope='Each configuration screened on six actual registered frames per clip. Not full static/video acceptance; true circle identity must be visually checked.',
    adopted=False,results=summary,source_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
with (OUT/'sweep.json').open('x',encoding='utf-8') as f:
    json.dump(result,f,ensure_ascii=False,indent=2,allow_nan=False)
print(json.dumps([dict(variant=r['variant'],complete=r['complete'],worst_rms_deg=r['worst_sampled_heading_jitter_rms_deg'],front_points=[m.get('median_front_px') for m in r['clips']]) for r in summary[:8]],ensure_ascii=True),flush=True)
