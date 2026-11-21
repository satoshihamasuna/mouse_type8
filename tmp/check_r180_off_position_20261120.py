"""OFF移動後の保存試写を既存の静止判定で確認する。実機接続なし。"""
from pathlib import Path
import argparse
import json
import subprocess
import sys
import os
import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--stage', type=Path, default=ROOT / 'tools/logs/pixel_camera/turn1800_A_r180_20261120_v1/angle_c4p25_03')
parser.add_argument('--output-name', default='off_position_after_move_20261120_01')
parser.add_argument('--manifest', type=Path, default=ROOT / 'tools/logs/pixel_camera/20261120_120913_8dcf8261.capture.json')
parser.add_argument('--video', type=Path, default=Path('D:/TYPE8_Camera/recordings/remote/20261120_120913_240fps_1abc9505.mp4'))
args = parser.parse_args()
STAGE = args.stage.resolve()
OUT = (STAGE / args.output_name).resolve()
assert OUT.is_relative_to(STAGE) and STAGE.is_relative_to(ROOT)
OUT.mkdir(exist_ok=False)
config = json.loads((STAGE / 'config.json').read_text(encoding='utf-8-sig'))
manifest = args.manifest.resolve()
video = args.video.resolve()
plan = dict(video=str(video), camera_manifest=str(manifest),
    reference=config['reference'], geometry=config['geometry'],
    placement=dict(x_mm=315., y_mm=135., heading_deg=90.,
        position_tolerance_mm=5., heading_tolerance_deg=2.),
    stationary_video_window_s=[3.3, 4.95], stationary_frame_stride=2)
(OUT / 'static_plan.json').write_text(json.dumps(plan, indent=2), encoding='utf-8')
subprocess.run([sys.executable, str(STAGE / 'static_pose_after_settle.py'), 'measure',
    str(OUT / 'static_plan.json'), str(OUT / 'static_check.json')], check=True,
    env=dict(os.environ, OPENCV_FOR_THREADS_NUM='4', PYTHONUTF8='1'))
result = json.loads((OUT / 'static_check.json').read_text(encoding='utf-8'))
sys.path.insert(0, str(ROOT / 'tools/analysis'))
from video_marker_pose import marker_registration, registration_quad, pose_world_corners, board_to_pose_transform, transform_points
reference = json.loads(Path(config['reference']).read_text(encoding='utf-8-sig'))
capture = cv2.VideoCapture(str(video))
capture.set(cv2.CAP_PROP_ORIENTATION_AUTO, 0)
capture.set(cv2.CAP_PROP_POS_MSEC, 4200)
ok, frame = capture.read()
capture.release()
assert ok
registration = marker_registration(frame, reference)
quad = registration_quad(registration)
assert quad is not None and len(set(registration['detected_marker_ids'])) >= 2
to_image = cv2.getPerspectiveTransform(pose_world_corners(reference['board_side_mm']), quad) @ board_to_pose_transform(reference)
points = transform_points(np.array([[315., 135.], result['center_board_mm'], [315., 110.], [315., 160.]]), to_image)
plt.rcParams['font.family'] = 'Yu Gothic'
fig, ax = plt.subplots(figsize=(12.8, 7.2), layout='constrained')
ax.imshow(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
ax.scatter(*points[0], s=180, marker='+', c='cyan', linewidths=2, label='開始中点の目標 (315,135) mm')
ax.scatter(*points[1], s=130, marker='x', c='lime', linewidths=2, label='今回の実測中点')
ax.plot(points[[2, 3], 0], points[[2, 3], 1], '--', color='cyan', linewidth=2, label='赤前を下へ向ける線')
ax.set_title('R180開始配置・両方OFFの試写 / 位置・向き照合 ' + ('合格' if result['passed'] else '未合格'))
ax.legend(loc='lower left', fontsize=9)
ax.axis('off')
fig.savefig(OUT / 'position_check.png', dpi=120)
plt.close(fig)
print(json.dumps(dict(output=str(OUT), passed=result['passed'],
    center_board_mm=result['center_board_mm'], heading_deg=result['heading_deg'],
    failures=result['failure_reasons'], observed_fraction=result['observed_fraction']), ensure_ascii=False))
