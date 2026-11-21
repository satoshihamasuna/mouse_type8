"""機体OFFのまま、保存済み1500 mm・2倍の調整設定で静止確認する。"""
from pathlib import Path
import json
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'tools'), str(ROOT/'tools/analysis')]
FOLDER = ROOT/'tools/logs/pixel_camera/turn1600_A_r90_20261116_preflight_v1'
SOURCE = ROOT/'tools/logs/pixel_camera/A_search370_left_kp5_lend9_1msaudit_hold_20261116_prepared'


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')


if sys.argv[1] == 'capture':
    from pixel_camera_capture import camera_workspace, record_trial
    from pixel_camera_profiles import save_camera_profile
    sys.path.insert(0, str(camera_workspace()/'python'))
    from pixel_camera import PixelCameraClient
    FOLDER.mkdir(parents=True, exist_ok=False)
    reference = json.loads((SOURCE/'marker_reference_A_current.json').read_text(encoding='utf-8'))
    geometry = json.loads((SOURCE/'body_geometry_A_current.json').read_text(encoding='utf-8'))
    save(FOLDER/'marker_reference_A_current.json', reference)
    save(FOLDER/'body_geometry_A_current.json', geometry)
    preferences = ROOT/'tools/logs/pixel_camera/tuning_ui.json'
    previous = json.loads(preferences.read_text(encoding='utf-8'))
    save(FOLDER/'tuning_ui.before.json', previous)
    camera_settings = dict(ev=-2., zoom_ratio=2., torch=False, focus_lock=True)
    save_camera_profile('tuning', camera_settings)
    previous.update(mouse='A',height_mm=1500.,reference=str(FOLDER/'marker_reference_A_current.json'),
                    geometry=str(FOLDER/'body_geometry_A_current.json'))
    save(preferences, previous)
    save(FOLDER/'preparation.json', dict(mouse='A', motion='long_r90', preset=1600, count=1,
        suction=False, camera_height_mm=1500., camera_height_source='user confirmed this turn',
        camera_settings=camera_settings, source_config=str(SOURCE/'config.json'),
        placement_confirmed=False, robot_power='OFF confirmed at end of previous run; ON not requested',
        robot_commands_sent=0))
    camera = PixelCameraClient(connection='wifi', port=8765, auto_start=False)
    status = camera.get_status()
    if not status.get('connected') or status.get('device', {}).get('phase') != 'IDLE':
        raise RuntimeError('Camera must be connected and idle; do not execute robot')
    manifest, value = record_trial(camera, ROOT, seconds=3, mouse='A',
        label='A_turn1600_right90_OFF_static_no_robot_command',
        camera_settings=camera_settings, capture_profile='tuning')
    save(FOLDER/'static_capture.json', dict(manifest=str(manifest), **value))
    print(json.dumps(dict(state=value['state'],folder=str(FOLDER),video=value.get('video_file')),ensure_ascii=False),flush=True)
    if value['state'] != 'PASSED':
        raise RuntimeError('Static video did not pass; no robot execution')
elif sys.argv[1] == 'capture_current':
    from pixel_camera_capture import camera_workspace, record_trial
    from pixel_camera_profiles import load_camera_profile
    sys.path.insert(0, str(camera_workspace()/'python'))
    from pixel_camera import PixelCameraClient
    output = FOLDER/'powered_static_capture.json'
    if output.exists():
        raise RuntimeError('Do not overwrite or automatically repeat the static capture')
    camera = PixelCameraClient(connection='wifi', port=8765, auto_start=False)
    status = camera.get_status()
    if not status.get('connected') or status.get('device', {}).get('phase') != 'IDLE':
        raise RuntimeError('Camera must be idle')
    manifest, value = record_trial(camera, ROOT, seconds=3, mouse='A',
        label='A_turn1600_right90_powered_static_no_robot_command',
        camera_settings=load_camera_profile('tuning'), capture_profile='tuning')
    save(output, dict(manifest=str(manifest), **value))
    print(json.dumps(dict(state=value['state'],video=value.get('video_file')),ensure_ascii=False),flush=True)
elif sys.argv[1] in ('inspect','inspect_current'):
    import cv2
    import numpy as np
    from video_marker_pose import read_video_pose, board_to_pose_transform, transform_points
    current = sys.argv[1] == 'inspect_current'
    capture = json.loads((FOLDER/('powered_static_capture.json' if current else 'static_capture.json')).read_text(encoding='utf-8'))
    if capture['state'] != 'PASSED':
        raise RuntimeError('Capture validation failed; do not execute robot')
    reference = json.loads((FOLDER/'marker_reference_A_current.json').read_text(encoding='utf-8'))
    geometry = json.loads((FOLDER/'body_geometry_A_current.json').read_text(encoding='utf-8'))
    video = cv2.VideoCapture(capture['video_file'])
    video.set(cv2.CAP_PROP_ORIENTATION_AUTO, 0)
    video.set(cv2.CAP_PROP_POS_MSEC, 2000)
    ok, frame = video.read()
    video.release()
    if ok:
        cv2.imwrite(str(FOLDER/('powered_static_frame.jpg' if current else 'static_frame.jpg')), frame)
    pose, _, _, _ = read_video_pose(Path(capture['video_file']), 'red', marker_reference=reference,
        video_window=(1.,2.95), marker_detection=geometry.get('marker_detection'), frame_stride=4,
        rear_colour='yellow', nominal_marker_span_mm=50., marker_span_limits_mm=[40,65])
    valid = pose.marker_detected_raw.to_numpy(bool) & pose.calibration_valid.to_numpy(bool)
    inverse = np.linalg.inv(board_to_pose_transform(reference))
    front = transform_points(pose[['blue_x_mm','blue_y_mm']].to_numpy(), inverse)
    rear = transform_points(pose[['yellow_x_mm','yellow_y_mm']].to_numpy(), inverse)
    center = (front+rear)/2
    direction = np.degrees(np.arctan2((front-rear)[:,1], (front-rear)[:,0]))
    result = dict(robot_commands_sent=0, samples=len(pose), valid_rate=float(valid.mean()),
        calibration=pose.attrs['marker_calibration'], camera_height_mm=1500., zoom_ratio=2.,
        center_board_mm=np.median(center[valid],axis=0).tolist() if valid.any() else None,
        heading_image_deg=float(np.median(direction[valid])) if valid.any() else None,
        frame=str(FOLDER/('powered_static_frame.jpg' if current else 'static_frame.jpg')),
        scope='static sampled pose only; placement and clearance require operator confirmation')
    if valid.any():
        errors = np.linalg.norm(center[valid]-[315.,45.],axis=1)
        angle_errors = (direction[valid]-90.+180.)%360.-180.
        scatter = np.linalg.norm(center[valid]-np.median(center[valid],axis=0),axis=1)
        result.update(placement_gate_passed=bool(valid.mean() >= .95 and errors.max() <= 8
            and abs(angle_errors).max() <= 2 and scatter.max() <= 1.5),
            planned_start_board_mm=[315,45],planned_heading_image_deg=90,
            maximum_position_error_mm=float(errors.max()),maximum_heading_error_deg=float(abs(angle_errors).max()),
            maximum_static_scatter_mm=float(scatter.max()))
    else:
        result['placement_gate_passed'] = False
    save(FOLDER/('powered_static_check.json' if current else 'static_check.json'), result)
    print(json.dumps(result,ensure_ascii=False),flush=True)
else:
    raise ValueError('capture or inspect')
