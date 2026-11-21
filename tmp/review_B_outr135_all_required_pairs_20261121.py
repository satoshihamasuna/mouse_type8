"""保存済み3動画の必要区間を全コマ目視用に並べる。検出値は変更しない。"""
from pathlib import Path
import hashlib
import json
import cv2
import numpy as np
import pandas as pd

root = Path(__file__).resolve().parents[1]
base = root/'tools/logs/pixel_camera/turn1600_B_outr135_20261121_resume_v1'
out = base/'all_required_marker_pairs_review_v1'
assert not out.exists()
out.mkdir()
read = lambda p: json.loads(p.read_text(encoding='utf-8-sig'))
comparison = read(base/'geometry_same_candidate_three_review_v1/review.json')
manifest = dict(state='VISUAL_REVIEW_PENDING', hardware_operations=0,
                thresholds_changed=False, source_data_modified=False, runs=[])
cv2.setNumThreads(4)
for number, source in enumerate(comparison['runs'], 1):
    run = Path(source['analysis_trial']).parent
    review = read(run/'midpoint_raw_wheel_clock_v1/midpoint_review.json')
    pose = pd.read_csv(run/'full/apparent_floor_marker_pose.csv')
    trajectory = pd.read_csv(run/'midpoint_raw_wheel_clock_v1/midpoint_relative_trajectory.csv')
    bounds = review['independent_clock_required_motion_quality']['required_interval_pts_s']
    selection = pose.index[pose.time_video_s.between(*bounds)].tolist()
    cap = cv2.VideoCapture(read(run/'trial.json')['video_file'])
    cap.set(cv2.CAP_PROP_ORIENTATION_AUTO, 0)
    # 欠測時の切り出し位置だけを補間し、検出・観測率には使わない。
    centers = pose[['midpoint_image_x_px', 'midpoint_image_y_px']].interpolate(limit_direction='both').to_numpy()
    records, pages = [], []
    for page_start in range(0, len(selection), 40):
        canvas = np.full((8*172, 5*174, 3), 255, np.uint8)
        for slot, index in enumerate(selection[page_start:page_start+40]):
            row = pose.iloc[index]
            cap.set(cv2.CAP_PROP_POS_FRAMES, int(row.frame))
            ok, frame = cap.read()
            assert ok
            actual = bool(row.marker_detected_raw and row.calibration_valid)
            if actual:
                front = np.array([row.front_image_x_px, row.front_image_y_px])
                center = np.array([row.midpoint_image_x_px, row.midpoint_image_y_px])
                rear = 2*center-front
                for point, colour in ((front, (0,255,0)), (rear, (0,0,255))):
                    cv2.circle(frame, tuple(np.rint(point).astype(int)), 5, colour, 1)
            x, y = np.rint(centers[index]).astype(int)
            crop = frame[max(0,y-70):y+70, max(0,x-70):x+70]
            crop = cv2.resize(crop, (140,140))
            top, left = (slot//5)*172, (slot%5)*174
            canvas[top+30:top+170, left+16:left+156] = crop
            text = f'R{number} F{int(row.frame)} '+('OBS' if actual else 'MISSING')
            cv2.putText(canvas, text, (left+2,top+20), cv2.FONT_HERSHEY_SIMPLEX,
                        .39, (0,0,0) if actual else (0,0,220), 1, cv2.LINE_AA)
            records.append(dict(frame=int(row.frame), time_video_s=float(row.time_video_s),
                                original_valid=actual))
        filename = f'run{number}_required_{page_start//40+1:02d}.jpg'
        assert cv2.imwrite(str(out/filename), canvas, [cv2.IMWRITE_JPEG_QUALITY,95])
        pages.append(filename)
    cap.release()
    manifest['runs'].append(dict(number=number, source_trial=source['analysis_trial'],
        pose_sha256=hashlib.sha256((run/'full/apparent_floor_marker_pose.csv').read_bytes()).hexdigest(),
        required_bounds_s=bounds, frames=records, pages=pages))
with (out/'manifest.json').open('x',encoding='utf-8') as stream:
    json.dump(manifest,stream,ensure_ascii=False,indent=2)
state_path = base/'resume_state.json'
state = read(state_path)
state.update(state='THREE_RUN_MARKER_PAIR_QA_REQUIRED_BOTH_OFF',
    candidate_status='ADDITIONAL_FALSE_PAIR_REVIEW_REQUIRED',
    comparison_superseding_review_pending=str(out),
    next_action='保存済み動画で誤対応を除外し、観測率と数値の影響を再評価する。実機操作は予定しない。')
state_path.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps([dict(run=r['number'],frames=len(r['frames']),pages=r['pages']) for r in manifest['runs']],ensure_ascii=False),flush=True)
