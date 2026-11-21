"""右90°再開前のカメラ静止確認。機体の接続・設定・実行は行わない。"""
from pathlib import Path
import json
import sys

ROOT = Path(__file__).resolve().parents[1]
FOLDER = ROOT / 'tools/logs/pixel_camera/turn1600_A_r180_20261118_v1/baseline01/static01'
if len(sys.argv) > 2:
    FOLDER = ROOT / sys.argv[2]
sys.path[:0] = [str(ROOT / 'tools'), str(ROOT / 'tools/analysis')]


def save(name, value):
    path = FOLDER / name
    with path.open('x', encoding='utf-8') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')


if sys.argv[1] == 'capture':
    from pixel_camera_capture import camera_workspace, record_trial
    sys.path.insert(0, str(camera_workspace() / 'python'))
    from pixel_camera import PixelCameraClient
    if (FOLDER / 'static_capture.json').exists():
        raise RuntimeError('既存の静止撮影は上書き・再撮影しません')
    config = json.loads((FOLDER / 'config.json').read_text(encoding='utf-8'))
    camera = PixelCameraClient(connection='wifi', port=8765, auto_start=False)
    status = camera.get_status()
    if not status.get('connected') or status.get('device', {}).get('phase') != 'IDLE':
        raise RuntimeError('カメラが接続済み・待機中ではありません')
    manifest, result = record_trial(camera, ROOT, seconds=3, mouse='A',
        label='turn1600_A_r180_static_no_robot_command_20261118',
        camera_settings=config['camera_settings'], capture_profile='tuning')
    save('static_capture.json', dict(manifest=str(manifest), **result))
    print(json.dumps({key: result.get(key) for key in ('state', 'video_file', 'error')}, ensure_ascii=False), flush=True)
elif sys.argv[1] == 'inspect':
    import cv2
    import numpy as np
    from video_marker_pose import read_video_pose, board_to_pose_transform, transform_points
    capture = json.loads((FOLDER / 'static_capture.json').read_text(encoding='utf-8'))
    if capture['state'] != 'PASSED':
        raise RuntimeError('撮影の検証が成功していません')
    reference = json.loads((FOLDER / 'marker_reference_A_current.json').read_text(encoding='utf-8'))
    geometry = json.loads((FOLDER / 'body_geometry_A_current.json').read_text(encoding='utf-8'))
    video = cv2.VideoCapture(capture['video_file'])
    video.set(cv2.CAP_PROP_ORIENTATION_AUTO, 0)
    video.set(cv2.CAP_PROP_POS_MSEC, 2000)
    ok, frame = video.read()
    video.release()
    if not ok:
        raise RuntimeError('静止フレームを読み出せません')
    cv2.imwrite(str(FOLDER / 'static_frame.jpg'), frame)
    pose, _, _, _ = read_video_pose(Path(capture['video_file']), 'red', marker_reference=reference,
        video_window=(1., 2.95), marker_detection=geometry.get('marker_detection'), frame_stride=4,
        rear_colour='yellow', nominal_marker_span_mm=50., marker_span_limits_mm=[40, 65])
    valid = pose.marker_detected_raw.to_numpy(bool) & pose.calibration_valid.to_numpy(bool)
    inverse = np.linalg.inv(board_to_pose_transform(reference))
    front = transform_points(pose[['blue_x_mm', 'blue_y_mm']].to_numpy(), inverse)
    rear = transform_points(pose[['yellow_x_mm', 'yellow_y_mm']].to_numpy(), inverse)
    center = (front + rear) / 2
    heading = np.degrees(np.arctan2((front - rear)[:, 1], (front - rear)[:, 0]))
    target = np.array([315., 135.])
    position_error = np.linalg.norm(center[valid] - target, axis=1)
    heading_error = np.abs((heading[valid] - 90. + 180.) % 360. - 180.)
    result = dict(mouse='A', robot_commands_sent=0, samples=len(pose), valid_rate=float(valid.mean()),
        calibration=pose.attrs['marker_calibration'], center_board_mm=np.median(center[valid], axis=0).tolist() if valid.any() else None,
        heading_image_deg=float(np.median(heading[valid])) if valid.any() else None,
        planned_start_board_mm=target.tolist(), planned_heading_image_deg=90.,
        maximum_position_error_mm=float(position_error.max()) if len(position_error) else None,
        maximum_heading_error_deg=float(heading_error.max()) if len(heading_error) else None,
        placement_gate_passed=bool(valid.all() and len(pose) > 50 and len(position_error) and position_error.max() <= 5. and heading_error.max() <= 2.),
        scope='静止したマーカー中点の観測。機体識別・全接地点・穴・車幅・停止余裕・手離しは操作者の今回の確認が必要。')
    save('static_check.json', result)
    print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
else:
    raise ValueError('capture または inspect を指定してください')
