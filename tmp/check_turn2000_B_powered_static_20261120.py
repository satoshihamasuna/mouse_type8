"""通電後の短い静止照合。14画像で前後の実色観測と各画像のArUco登録を必須にする。"""
from pathlib import Path
import argparse
import json
import sys
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'tools'), str(ROOT/'tools/analysis')]
from body_appearance_model import LearnedBodyKeypointModel, detect_pair_with_body_model
from video_marker_pose import iter_video_frames, marker_registration


def check(capture_path, config_path, output):
    read = lambda p: json.loads(Path(p).read_text(encoding='utf-8'))
    capture, config = read(capture_path), read(config_path)
    if capture['state'] != 'PASSED':
        raise ValueError('静止動画の取得品質不合格')
    geometry, reference = read(config['geometry']), read(config['reference'])
    detection = geometry['marker_detection']
    model = LearnedBodyKeypointModel.load(Path(detection['body_model']['model_file']))
    targets = set((np.linspace(240, 704, 14)/4).round().astype(int)*4)
    rows, front, rear = [], [], []
    started = time.monotonic()
    previous = np.array([315., 45.])
    for index, frame, pts in iter_video_frames(Path(capture['video_file']), True, (1., 2.95), 4):
        if index not in targets:
            continue
        if time.monotonic()-started > 25:
            raise TimeoutError('静止照合が25秒を超過。走行せず終了する')
        reg = marker_registration(frame, reference)
        row = dict(frame=index, pts_s=pts, observed=False, registered=bool(reg['available']),
                   decoded_ids=reg['detected_marker_ids'])
        if reg['available'] and len(set(reg['detected_marker_ids'])) >= 2:
            h = np.asarray(reg['homography_image_to_board_mm'])
            pair = detect_pair_with_body_model(frame, 'green', 'red', detection, h, np.linalg.inv(h), model,
                previous_center_mm=previous, nominal_span_mm=50, span_limits_mm=[40,65],
                require_colour_pair=True)
            if pair is not None:
                f, r, info = pair
                assert info['method'] == 'pair_body_scored'
                row.update(observed=True, front_board_mm=f.tolist(), rear_board_mm=r.tolist(),
                    front_px=info['front_px'].tolist(), rear_px=info['rear_px'].tolist(),
                    body_ncc=info['body_ncc'], method=info['method'])
                front.append(f); rear.append(r); previous = (f+r)/2
        rows.append(row)
    result = dict(passed=False, frames=len(rows), observed=len(front),
        observation_scope='14 fixed samples across1.0..2.95s; all selected images decoded individually',
        colour_pair_required=True, per_frame_aruco_required=True, rows=rows,
        source_capture=str(capture_path), elapsed_s=time.monotonic()-started)
    if len(front) >= 12:
        f, r = np.array(front), np.array(rear)
        heading = np.degrees(np.arctan2((f-r)[:,1],(f-r)[:,0]))
        wrap = lambda a: (a+180)%360-180
        anchor = np.degrees(np.angle(np.mean(np.exp(1j*np.radians(heading)))))
        representative = float((anchor+np.median(wrap(heading-anchor)))%360)
        jitter = wrap(heading-representative)
        rms = float(np.sqrt(np.mean(jitter*jitter)))
        drift = float(abs(np.median(jitter[len(jitter)//2:])-np.median(jitter[:len(jitter)//2])))
        budget = float(abs(wrap(representative-90.))+2*rms)
        max_position = float(np.linalg.norm((f+r)/2-[315.,45.],axis=1).max())
        result.update(center_board_mm=np.median((f+r)/2,axis=0).tolist(), heading_deg=representative,
            heading_jitter_rms_deg=rms, heading_half_window_deg=drift, heading_budget_deg=budget,
            max_position_error_mm=max_position)
        result['passed'] = bool(len(rows)==14 and len(front)/len(rows)>=.95 and max_position<=5
            and budget<=2 and rms<=.5 and drift<=.5 and np.abs(jitter).max()<=2)
    with Path(output).open('x', encoding='utf-8') as stream:
        json.dump(result,stream,ensure_ascii=False,indent=2,allow_nan=False)
    print(json.dumps({k:v for k,v in result.items() if k!='rows'},ensure_ascii=True),flush=True)
    return result['passed']


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('config', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    raise SystemExit(0 if check(args.capture,args.config,args.output) else 2)
