"""保存画像へ開始点を表示し、既存カメラの接続状態だけを読む。"""
from pathlib import Path
import json
import sys
import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT/'tools/logs/pixel_camera/turn1600_B_inr135_20261120_v1'
OLD = ROOT/'tools/logs/pixel_camera/turn1600_B_r180_20261120_v1'
sys.path[:0] = [str(ROOT/'tools'), str(ROOT/'tools/analysis')]
from video_marker_pose import (marker_registration, registration_quad, pose_world_corners,
    board_to_pose_transform, transform_points)
from pixel_camera_capture import camera_workspace
read = lambda path: json.loads(path.read_text(encoding='utf-8-sig'))
frame_path = OLD/'angle_c0p5_s35_02/local_execution/prepare_B_R180_1600_c0p5_s35_green_20261120_002/static_check.jpg'
frame = cv2.imread(str(frame_path))
reference = read(BASE/'marker_reference_B_current.json')
registration = marker_registration(frame, reference)
quad = registration_quad(registration)
assert quad is not None and len(set(registration['detected_marker_ids'])) >= 2
matrix = cv2.getPerspectiveTransform(pose_world_corners(reference['board_side_mm']), quad) @ board_to_pose_transform(reference)
points = transform_points(np.array([[315.,135.], [315.,110.], [315.,160.]]), matrix)
plt.rcParams['font.family'] = 'Yu Gothic'
fig, ax = plt.subplots(figsize=(12.8,7.2), layout='constrained')
ax.imshow(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
ax.plot(points[[1,2],0], points[[1,2],1], c='cyan', lw=2)
ax.scatter(*points[0], s=220, marker='+', c='cyan', linewidths=2)
ax.scatter(*points[2], s=100, marker='v', c='lime', edgecolors='black', label='前：緑（下向き）')
ax.scatter(*points[1], s=90, c='red', edgecolors='white', label='後：赤')
ax.annotate('開始中点 (315,135) mm\n右端から1マス・上端から3マス', xy=points[0], xytext=(380,230),
    color='white', fontsize=12, arrowprops=dict(arrowstyle='->',color='cyan'),
    bbox=dict(facecolor='black', alpha=.8, edgecolor='none'))
ax.set_title('機体B・1600 右135度入口：両方OFFで配置\n保存写真への案内（現在位置の測定ではありません）')
ax.legend(loc='lower left'); ax.axis('off')
fig.savefig(BASE/'start_placement_guide.jpg', dpi=130); plt.close(fig)
print(json.dumps(dict(guide=str(BASE/'start_placement_guide.jpg'), source_image=str(frame_path),
    aruco_ids=registration['detected_marker_ids']), ensure_ascii=False))
sys.path.insert(0, str(camera_workspace()/'python'))
from pixel_camera import PixelCameraClient
status = PixelCameraClient(connection='wifi', port=8765, auto_start=False).get_status()
with (BASE/'camera_status01.json').open('x', encoding='utf-8') as stream:
    json.dump(status, stream, ensure_ascii=False, indent=2)
print(json.dumps(dict(connected=status.get('connected'), device=status.get('device'),
    save_dir=status.get('save_dir')), ensure_ascii=False))
