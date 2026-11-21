"""緑・赤の色分離候補を、別の保存済み静止像と走行で検証する。実機通信なし。"""
from pathlib import Path
import hashlib
import json
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'tools'), str(ROOT/'tools/analysis')]
from pixel_camera_static_pose import measure
from video_marker_pose import read_video_pose, iter_video_frames

BASE = ROOT/'tools/logs/pixel_camera/turn2000_B_r90_20261120_v1'
RUN = BASE/'cli_green_session01/run_B_R90_2000_omKi005_green_20261120_001'
OTHER = ROOT/'tools/logs/pixel_camera/turn1600_B_r90_20261120_v1'
OUT = RUN/'independent_colour_candidate_v4'
OUT.mkdir(exist_ok=False)
read = lambda p: json.loads(Path(p).read_text(encoding='utf-8'))
write = lambda p, d: Path(p).write_text(json.dumps(d, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
profile = RUN/'green_tracking_probe_v4/body_geometry_candidate.json'
geometry = read(profile)
reference = read(RUN/'marker_reference_used.json')
static = []
for tag in ('B_green_front_OFF01', 'B_green_front_OFF02'):
    capture = read(OTHER/(tag+'_static_capture.json'))
    original = read(OTHER/(tag+'_static_check.json'))
    result = measure(Path(capture['video_file']), reference, geometry, original['placement'])
    result.update(video_file=capture['video_file'], original_check=str(OTHER/(tag+'_static_check.json')))
    write(OUT/(tag+'.json'), result)
    static.append(result)
    print(tag, 'passed', result['passed'], 'observed', result['observed'], 'jitter', result.get('heading_jitter_rms_deg'), flush=True)

other_run = OTHER/'cli_session04/run_B_R90_1600_Lstart13p6_green_20261120_001'
trial = read(other_run/'trial.json')
original_report = read(other_run/'full/report.json')
window = original_report['required_motion_window_quality']['required_interval_pts_s']
other_reference = read(other_run/'marker_reference_used.json')
pose, _, _, _ = read_video_pose(Path(trial['video_file']), 'green', marker_reference=other_reference,
    video_window=window, frame_stride=1, marker_detection=geometry['marker_detection'], rear_colour='red',
    nominal_marker_span_mm=50, marker_span_limits_mm=[40, 65],
    progress=lambda stage, count, pts: print(stage, count, pts, flush=True))
pose.to_csv(OUT/'independent1600_motion_pose.csv', index=False)
valid = pose.marker_detected_raw.to_numpy(bool) & pose.calibration_valid.to_numpy(bool)
result = dict(video_file=trial['video_file'], video_window=window, frames=len(pose), observed=int(valid.sum()),
    observation_rate=float(valid.mean()), calibration_rate=float(pose.calibration_valid.mean()),
    observation_rate_passed=bool(valid.mean() >= .95), marker_identity_manual_review='pending',
    original_report=str(other_run/'full/report.json'))
write(OUT/'independent1600_motion.json', result)

# 観測率と別に、一定間隔の元画像と検出中心を目視照合する。
sample_positions = np.linspace(0, len(pose)-1, 20).round().astype(int)
sample_frames = set(pose.iloc[sample_positions].frame.astype(int))
by_frame = pose.set_index('frame')
tiles = []
for index, frame, pts in iter_video_frames(Path(trial['video_file']), True, window, 1):
    if index not in sample_frames:
        continue
    row = by_frame.loc[index]
    if bool(row.marker_detected_raw) and bool(row.calibration_valid):
        front = np.array([row.front_image_x_px, row.front_image_y_px])
        rear = 2*np.array([row.midpoint_image_x_px, row.midpoint_image_y_px])-front
        centre = (front+rear)/2
        for point, colour in zip([front, rear], [(0,255,0), (0,0,255)]):
            cv2.circle(frame, tuple(np.rint(point).astype(int)), 4, colour, 1, cv2.LINE_AA)
            cv2.drawMarker(frame, tuple(np.rint(point).astype(int)), colour, cv2.MARKER_CROSS, 3, 1)
    else:
        centre = np.array([740, 400])
    x, y = np.rint(centre).astype(int)
    x0, y0 = max(0, min(frame.shape[1]-160, x-80)), max(0, min(frame.shape[0]-160, y-80))
    tile = cv2.resize(frame[y0:y0+160,x0:x0+160], (320,320), interpolation=cv2.INTER_NEAREST)
    cv2.putText(tile, str(index), (8,22), cv2.FONT_HERSHEY_SIMPLEX, .6, (255,255,255), 1, cv2.LINE_AA)
    tiles.append(tile)
while len(tiles)%4:
    tiles.append(np.zeros_like(tiles[0]))
cv2.imwrite(str(OUT/'independent1600_sample20.png'), np.vstack([np.hstack(tiles[i:i+4]) for i in range(0,len(tiles),4)]))
write(OUT/'summary.json', dict(static_checks_passed=all(r['passed'] for r in static),
    independent_motion=result, candidate_adopted=False, hardware_commands_sent=0,
    profile=str(profile), profile_sha256=hashlib.sha256(profile.read_bytes()).hexdigest()))
print(json.dumps(result), flush=True)
