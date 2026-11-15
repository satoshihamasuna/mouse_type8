"""B右45度出口の静止試写と保存動画の確認。機体へ指令を送らない。"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
BASE = Path(__file__).resolve().parent
sys.path[:0] = [str(ROOT/'tools'), str(ROOT/'tools/analysis'), str(BASE)]


def load(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def save_new(path, value):
    with Path(path).open('x', encoding='utf-8', newline='\n') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')


def inspect_frame(frame, reference, geometry, parameters, plan):
    import numpy as np
    from video_marker_pose import marker_registration, colour_components, select_marker_pair
    from nominal_inR135 import make_path
    board = marker_registration(frame, reference)
    result = dict(ready=False, board=board)
    if not board.get('available'):
        return result
    matrix = np.asarray(board['homography_image_to_board_mm'])
    pair = select_marker_pair(colour_components(frame, 'yellow', geometry['marker_detection']),
        colour_components(frame, 'red', geometry['marker_detection']), matrix, None,
        nominal_marker_span_mm=geometry['nominal_marker_span_mm'],
        marker_span_limits_mm=geometry['marker_span_limits_mm'])
    if pair is None:
        result['reason'] = 'B yellow-front/red-rear pair unavailable'
        return result
    front, rear = pair
    centre = (front+rear)/2
    span = float(np.linalg.norm(front-rear))
    forward = (front-rear)/span
    # 盤面座標は画像同様にYが下向き。機体が下向きなら右旋回は画像左へ進む。
    right = np.array([-forward[1], forward[0]])
    heading_error = float((np.degrees(np.arctan2(forward[1], forward[0]))-plan['initial_heading_board_deg']+180)%360-180)
    path, duration = make_path(parameters, plan['pre_run_mm'], plan['post_run_mm'])
    world = centre+path.right_mm.to_numpy()[:, None]*right+path.forward_mm.to_numpy()[:, None]*forward
    axes = (np.cos(path.heading_right_positive_rad.to_numpy())[:, None]*forward
            +np.sin(path.heading_right_positive_rad.to_numpy())[:, None]*right)
    markers = np.concatenate([world+span/2*axes, world-span/2*axes])
    centre_inside = bool(np.all((world >= plan['board_centre_margin_mm'])
                               & (world <= 360-plan['board_centre_margin_mm'])))
    markers_inside = bool(np.all((markers >= plan['board_marker_margin_mm'])
                                & (markers <= 360-plan['board_marker_margin_mm'])))
    position_error = float(np.linalg.norm(centre-np.asarray(plan['initial_position_board_xy_mm'])))
    result.update(ready=bool(centre_inside and markers_inside and abs(heading_error) <= plan['maximum_start_heading_change_deg'] and position_error <= plan['maximum_start_displacement_from_planned_mm']),
        planned_start_distance_mm=position_error,
        centre_board_xy_mm=centre.tolist(), forward_board_xy=forward.tolist(), span_mm=span,
        heading_error_from_planned_deg=heading_error,
        nominal_center_inside_board=centre_inside, nominal_markers_inside_board=markers_inside,
        nominal_center_min_board_xy_mm=world.min(axis=0).tolist(),
        nominal_center_max_board_xy_mm=world.max(axis=0).tolist(),
        nominal_marker_min_board_xy_mm=markers.min(axis=0).tolist(),
        nominal_marker_max_board_xy_mm=markers.max(axis=0).tolist(),
        nominal_turn_duration_s=duration, trajectory_is_nominal_only=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('record', 'review'))
    parser.add_argument('--session', type=Path, required=True)
    args = parser.parse_args()
    session = args.session.resolve()
    meta = load(BASE/'next_session_B_outR45_lengths_repeat02_v1.json')
    assert session == Path(meta['session']).resolve()
    config = load(meta['config'])
    plan = load(meta['plan'])
    if args.action == 'record':
        assert meta['operator_ready'] == 'USER_CONFIRMED_POSITION_HANDS_OFF_BOTH_ON'
        ready = load(session/'operator_ready.json')
        assert ready['placement_stopped_clearance_hands_off_both_on_confirmed'] is True
        assert (datetime.now(timezone.utc)-datetime.fromisoformat(ready['recorded_at'])).total_seconds() < 300
        assert not (session/'static_preflight.json').exists()
        from pixel_camera import PixelCameraClient
        from pixel_camera_capture import record_trial
        camera = PixelCameraClient(connection='usb', port=8765, auto_start=False)
        camera.connect()
        try:
            status = camera.get_status()
            assert status['connected'] and status['device']['phase'] == 'IDLE'
            configured = camera.configure(exposure_compensation_steps=-6, torch=False, focus_lock=True, zoom_ratio=2.0)
            assert configured['exposure_compensation_steps'] == -6
            assert configured['zoom_ratio'] == 2.0
            assert configured['focus_lock'] and configured['torch'] is False
        finally:
            camera.disconnect()
        manifest, capture = record_trial(camera, ROOT, seconds=3, mouse='B',
            label='B out_R45 static preflight; NO ROBOT COMMAND')
        save_new(session/'static_preflight.json', dict(capture_json=str(manifest),
            state=capture['state'], video_file=capture.get('video_file'), configured=configured,
            error=capture.get('error'), robot_commands_sent=False))
        assert capture['state'] == 'PASSED'
        print(json.dumps(dict(state=capture['state'], video=capture['video_file'])))
        return

    import numpy as np
    import pandas as pd
    import cv2
    from pixel_camera_framing import read_frame
    baseline = load(meta['preset_readback'])
    candidate = load(meta['settings'])
    assert baseline['mouse'] == candidate['mouse'] == 'B'
    assert baseline['motion_name'] == candidate['motion_name'] == 'out_r45'
    assert baseline['preset_speed_mm_s'] == candidate['preset_speed_mm_s'] == 500
    assert baseline['turn_count'] == candidate['turn_count'] == 1
    for key, value in plan['expected_compiled_B'].items():
        assert abs(baseline['parameters'][key]-value) <= .0010000001, key
    for key, value in plan['expected_candidate_parameters'].items():
        assert abs(candidate['parameters'][key]-value) <= .0010000001, key
    for key, value in plan['expected_B_pid'].items():
        assert abs(baseline['pid'][key]-value) <= 1e-6, key
    for key, value in plan['expected_candidate_pid'].items():
        assert abs(candidate['pid'][key]-value) <= 1e-6, key
    assert {key:[value,candidate['pid'][key]] for key,value in baseline['pid'].items() if value != candidate['pid'][key]} == plan['allowed_B_pid_changes']
    assert baseline['feedforward'] == candidate['feedforward']
    assert baseline['suction'] == candidate['suction'] and not candidate['suction']['enabled']
    capture_path = session/'static_preflight.json'
    capture = load(capture_path)
    full_capture = load(capture['capture_json'])
    metadata_path = Path(full_capture['metadata_file'])
    metadata = load(metadata_path)
    assert capture['state'] == full_capture['state'] == 'PASSED'
    assert metadata['focus_lock']['state'] == 'LOCKED' and metadata['focus_lock']['requested']
    assert metadata['zoom_ratio_requested'] == 2.0 and metadata['zoom_verification']['confirmed']
    assert metadata['zoom_verification']['error'] is None
    assert load(meta['camera_static_validation'])['state'] == 'PASSED_INDEPENDENT_STATIC_VALIDATION'
    assert not metadata['errors'] and metadata['ev_steps_result_values'] == [-6]
    assert metadata['flash_mode_result_values'] == [0]
    sensor_path = metadata_path.parent/metadata['sensor_csv']
    sensor = pd.read_csv(sensor_path)
    locked = sensor.index[sensor.af_state.eq(4)]
    assert len(locked) and sensor.loc[locked[0]:, 'af_state'].eq(4).all()
    assert sensor.loc[locked[0]:, 'zoom_ratio_result'].eq(2.0).all()
    reference, geometry = load(config['reference']), load(config['geometry'])
    frames = []
    for stamp in [1.2, 1.5, 1.8, 2.1, 2.4]:
        frame, actual_stamp = read_frame(capture['video_file'], stamp)
        result = inspect_frame(frame, reference, geometry, candidate['parameters'], plan)
        result['time_video_s'] = actual_stamp
        frames.append(result)
        if stamp == 1.8:
            cv2.imwrite(str(session/'static_preflight_frame.png'), frame)
    ready = all(frame['ready'] for frame in frames)
    if ready:
        centres = np.asarray([frame['centre_board_xy_mm'] for frame in frames])
        scatter = float(np.sqrt(np.mean(np.sum((centres-centres.mean(axis=0))**2, axis=1))))
        ready = bool(scatter <= 1 and np.max(np.linalg.norm(centres-centres.mean(axis=0), axis=1)) <= 3)
    else:
        scatter = None
    sources = [capture_path, Path(capture['capture_json']), metadata_path, sensor_path,
        Path(config['reference']), Path(config['geometry']), Path(meta['preset_readback']),
        Path(meta['settings']), Path(meta['plan']), BASE/'nominal_inR135.py', Path(__file__)]
    result = dict(ready=ready, frames=frames, stationary_sparse_scatter_rms_mm=scatter,
        fresh_B_FF_preserved=True, candidate_PID_matches_plan=True, compiled_B_PID_readback_matches_expected=True, target_right_forward_mm=plan['target_right_forward_mm'],
        candidate_pid=candidate['pid'], operator_ready=meta['operator_ready'], execution_commands_sent=0,
        scope='Sparse static setup check only; motion still requires all-frame review',
        input_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources})
    save_new(session/'static_preflight_check.json', result)
    print(json.dumps(dict(ready=ready, sparse_scatter_rms_mm=scatter, frames=len(frames))))
    assert ready, 'Preflight failed; do not send capture/exe'


if __name__ == '__main__':
    main()
