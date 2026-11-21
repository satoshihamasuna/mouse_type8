"""出口の独立候補動画の実マーカーと登録を保存画像で照合する。機器操作なし。"""
from pathlib import Path
import json
import sys
import cv2
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parents[1]
base = root/'tools/logs/pixel_camera/turn1600_B_outr135_20261121_resume_v1'
run = (base/(sys.argv[1] if len(sys.argv)>1 else 'lengths_Ls6_Le27p9_c1p9_02/local_execution/run_B_OUTR135_1600_Ls6_Le27p9_c1p9_green_20261120_002')).resolve()
assert run.is_relative_to(base.resolve())
out = run/'marker_visual_check_v1'
read = lambda path: json.loads(path.read_text(encoding='utf-8-sig'))
review = read(run/'midpoint_raw_wheel_clock_v1/midpoint_review.json')
trial = read(run/'trial.json')
reference = read(run/'marker_reference_used.json')
pose = pd.read_csv(run/'full/apparent_floor_marker_pose.csv')
start = review['entry_window_pts_s'][0]
end = review['exit_window_pts_s'][1]
valid = pose[(pose.time_video_s >= start) & (pose.time_video_s <= end) & pose.marker_detected_raw.astype(bool)]
indices = np.unique(np.rint(np.linspace(0, len(valid)-1, 16)).astype(int))
out.mkdir()
sys.path[:0] = [str(root/'tools'), str(root/'tools/analysis')]
from video_marker_pose import marker_registration
cv2.setNumThreads(4)
cap = cv2.VideoCapture(trial['video_file'])
cap.set(cv2.CAP_PROP_ORIENTATION_AUTO, 0)
fig, axes = plt.subplots(4, 4, figsize=(11, 11), layout='constrained')
records = []
for ax, index in zip(axes.flat, indices):
    row = valid.iloc[index]
    cap.set(cv2.CAP_PROP_POS_FRAMES, int(row.frame))
    ok, frame = cap.read()
    assert ok
    registration = marker_registration(frame, reference)
    ids = registration['detected_marker_ids']
    assert len(set(ids)) >= 2
    front = np.array([row.front_image_x_px, row.front_image_y_px])
    center = np.array([row.midpoint_image_x_px, row.midpoint_image_y_px])
    rear = 2*center-front
    canvas = frame.copy()
    for point, colour in ((front, (0, 255, 0)), (rear, (0, 0, 255))):
        cv2.circle(canvas, tuple(np.rint(point).astype(int)), 6, colour, 1)
    x, y = np.rint(center).astype(int)
    crop = canvas[max(0,y-70):min(frame.shape[0],y+70), max(0,x-70):min(frame.shape[1],x+70)]
    ax.imshow(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB))
    ax.set_title(f'{int(row.frame)} / {row.time_video_s:.4f}s', fontsize=9)
    ax.axis('off')
    records.append(dict(frame=int(row.frame), time_video_s=float(row.time_video_s),
        registered_aruco_ids=ids, cached_raw_pair=True, front_px=front.tolist(), rear_px=rear.tolist()))
cap.release()
fig.savefig(out/'actual_marker_pairs.jpg', dpi=145)
plt.close(fig)
with (out/'samples.json').open('x', encoding='utf-8') as file:
    json.dump(dict(state='VISUAL_REVIEW_PENDING', samples=records,
        original_profile_and_thresholds_unchanged=True, hardware_operations=0), file, ensure_ascii=False, indent=2)
print(out/'actual_marker_pairs.jpg', flush=True)
