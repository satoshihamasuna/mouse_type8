"""保存済み静止測定へ配置目標を重ねる。再測定・機器操作なし。"""
from pathlib import Path
import argparse
import json
import sys
import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--plan', type=Path, required=True)
p.add_argument('--result', type=Path, required=True)
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
assert all(x.resolve().is_relative_to(ROOT) for x in (a.plan, a.result, a.output))
assert not a.output.exists()
read = lambda x: json.loads(Path(x).read_text(encoding='utf-8-sig'))
plan, result = read(a.plan), read(a.result)
sys.path.insert(0, str(ROOT / 'tools/analysis'))
from video_marker_pose import (marker_registration, registration_quad,
    pose_world_corners, board_to_pose_transform, transform_points)
reference = read(plan['reference'])
capture = cv2.VideoCapture(str(plan['video']))
capture.set(cv2.CAP_PROP_ORIENTATION_AUTO, 0)
capture.set(cv2.CAP_PROP_POS_MSEC, 4200)
ok, frame = capture.read()
capture.release()
assert ok
registration = marker_registration(frame, reference)
quad = registration_quad(registration)
assert quad is not None and {2, 3}.issubset(registration['detected_marker_ids'])
transform = cv2.getPerspectiveTransform(pose_world_corners(reference['board_side_mm']), quad) @ board_to_pose_transform(reference)
points = transform_points(np.array([[315., 135.], result['center_board_mm'], [315., 110.], [315., 160.]]), transform)
plt.rcParams['font.family'] = 'Yu Gothic'
fig, ax = plt.subplots(figsize=(12.8, 7.2), layout='constrained')
ax.imshow(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
ax.scatter(*points[0], s=210, marker='+', c='cyan', linewidths=2, label='開始中点の目標 (315,135) mm')
ax.scatter(*points[1], s=140, marker='x', c='lime', linewidths=2, label='保存像の実測中点 (317.6,130.3) mm')
ax.plot(points[[2, 3], 0], points[[2, 3], 1], '--', color='cyan', linewidth=2, label='赤前を下へ向ける目標線')
ax.text(.02, .97, '両方OFFで\n左へ約3 mm・下へ約5 mm\n向きは約2°時計回り',
    transform=ax.transAxes, va='top', color='white', fontsize=12,
    bbox=dict(facecolor='black', alpha=.72, edgecolor='none'))
ax.set_title('R180開始配置ガイド / 保存済みの通電時静止像・現在は両方OFF確認済み')
ax.legend(loc='lower left', fontsize=9)
ax.axis('off')
fig.savefig(a.output, dpi=120)
plt.close(fig)
print(a.output)
