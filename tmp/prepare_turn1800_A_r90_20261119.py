"""機体A turn1800右90度の準備・カメラのみの静止撮影。走行指令なし。"""
from pathlib import Path
import hashlib
import json
import shutil
import sys

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tools/logs/pixel_camera/turn1800_A_r90_20261119_v1'
OLD = ROOT / 'tools/logs/pixel_camera/turn1600_A_r90_20261118_frame18_v1'
sys.path[:0] = [str(ROOT / 'tools'), str(ROOT / 'tools/analysis')]


def save(path, value):
    with path.open('x', encoding='utf-8') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')


if sys.argv[1] == 'prepare':
    from pixel_camera_tuning_cli import load_config, TuningService
    from pixel_camera_profiles import load_camera_profile
    BASE.mkdir(exist_ok=False)
    reference = BASE / 'marker_reference_A_current.json'
    geometry = BASE / 'body_geometry_A_yellow_v2.json'
    shutil.copyfile(OLD / 'marker_reference_A_current.json', reference)
    shutil.copyfile(OLD / 'analysis_yellow_v2/body_geometry_A_yellow_v2.json', geometry)
    camera = load_camera_profile('tuning')
    config = dict(mouse='A', address='C1:B5:60:87:91:73', height_mm=1500.,
        reference=str(reference), geometry=str(geometry), char_delay=.08,
        seconds=20, camera_settings=camera)
    save(BASE / 'config.json', config)
    settings = dict(schema_version=1, mouse='A', mouse_identity_source='user_supplied',
        motion_name='long_r90', preset_speed_mm_s=1800, turn_count=1,
        parameters=dict(velo=1.8, r_min=-52., lstart=11.38, lend=30.60,
            degree=-90., correction=0., pre_accel=6.5),
        pid=dict(sp_kp=2., sp_ki=.016, sp_kd=0., om_kp=.2, om_ki=.01, om_kd=0.),
        feedforward=dict(ff_sp_velo=.9030126, ff_sp_accel=.08183161, ff_sp_bias=.1190275,
            ff_om_velo=.0168, ff_om_accel=.00092, ff_om_decel=.00089,
            ff_om_bias=.0613, ff_om_jerk=.0000032,
            ff_sp_turn_accel_mag=-.00010034, ff_sp_turn_velo_sq=.00062859),
        suction=dict(enabled=True, duty=650),
        source='A turn1800 source table, PARAM_A_USE_PINK_LED_TURN_LENGTHS=0; not yet read from device or executed. Suction650 carried from preceding A turn1600 tuning.')
    save(BASE / 'candidate.settings.json', settings)
    config_checked = load_config(BASE / 'config.json')
    TuningService(BASE / 'validation_only', config_checked).trial_request(settings, True)
    save(BASE / 'preparation.json', dict(status='PREPARED_NOT_CONNECTED_NOT_EXECUTED',
        mouse='A', motion='long_r90', preset_mm_s=1800, count=1,
        planned_start_board_mm=[315., 45.], planned_heading_image_deg=90.,
        front_colour='red', rear_colour='yellow', pre_run_mm=180., post_run_mm=180.,
        reference_nominal_stop_board_mm=[45.,315.],
        placement_basis='A1600 right90 successfully used same starting grid point; current A1800 placement, full vehicle and stopping clearance remain to be checked.',
        requested_post_straight_accel_m_s2=20.,
        effective_pre_accel_m_s2=9., stopping_distance_at6p5_mm=1000*1.8**2/(2*6.5),
        stopping_distance_at20_mm=1000*1.8**2/(2*20),
        camera_height_mm=1500., current_camera_height_confirmation_pending=True,
        camera_settings=camera, power_state='Current physical power not confirmed; OFF requested during preparation',
        current_placement_confirmed=False, current_clearance_confirmed=False,
        current_hands_clear_confirmed=False, operator_ready=False,
        source_params_changed=False, execution_commands_sent=0, execution_pending=False,
        settings_write_commands_sent=0, execution_resend_count=0,
        wall_control_mode='pending device read', wall_edge_mode='pending device read',
        turn_ekf_mode='pending device read; A1800 is outside current A1600 calibration',
        reference_policy='At least two registered ArUcos actually decoded per evaluated frame; exclude missing registration.',
        evaluation_scope='Relative marker midpoint and measured entry; fixed-board placement also reported separately. Absolute axle/body heading uncalibrated.',
        source_hashes={str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest()
            for path in (ROOT/'Params/Param_A/turn_1800/turn_1800.cpp',
                ROOT/'Params/Param_A/run_param_A.h', ROOT/'System/Src/myshell.cpp', reference, geometry)}))
    print(json.dumps(dict(state='PREPARED', folder=str(BASE), parameters=settings['parameters'],
        suction=settings['suction'], camera_settings=camera), ensure_ascii=False), flush=True)
elif sys.argv[1] in ('capture', 'capture_placed', 'capture_settled'):
    from pixel_camera_capture import camera_workspace, record_trial
    sys.path.insert(0, str(camera_workspace() / 'python'))
    from pixel_camera import PixelCameraClient
    placed = sys.argv[1] != 'capture'
    tag = sys.argv[2] if placed and len(sys.argv) > 2 else 'placed' if placed else 'initial'
    if not tag.replace('_', '').isalnum():
        raise ValueError('Static tag must be alphanumeric')
    output = BASE / (tag + '_static_capture.json')
    if output.exists():
        raise RuntimeError('Existing initial static capture must be preserved')
    config = json.loads((BASE/'config.json').read_text(encoding='utf-8'))
    client = PixelCameraClient(connection='wifi', port=8765, auto_start=False)
    status = client.get_status()
    if not status.get('connected') or status.get('device', {}).get('phase') != 'IDLE':
        raise RuntimeError('Pixel must be connected and idle')
    manifest, result = record_trial(client, ROOT,
        seconds=5 if sys.argv[1] == 'capture_settled' else 3, mouse='A',
        label='turn1800_A_r90_' + tag + '_static_no_robot_commands',
        camera_settings=config['camera_settings'], capture_profile='tuning')
    save(output, dict(manifest=str(manifest), **result))
    print(json.dumps({k:result.get(k) for k in ('state','video_file','error')}, ensure_ascii=False), flush=True)
elif sys.argv[1] == 'inspect_placed':
    import cv2
    from pixel_camera_static_pose import measure
    tag = sys.argv[2] if len(sys.argv) > 2 else 'placed'
    if not tag.replace('_', '').isalnum():
        raise ValueError('Static tag must be alphanumeric')
    capture = json.loads((BASE/(tag+'_static_capture.json')).read_text(encoding='utf-8'))
    if capture['state'] != 'PASSED':
        raise RuntimeError('Static capture did not pass')
    config = json.loads((BASE/'config.json').read_text(encoding='utf-8'))
    reference = json.loads(Path(config['reference']).read_text(encoding='utf-8'))
    geometry = json.loads(Path(config['geometry']).read_text(encoding='utf-8'))
    placement = dict(x_mm=315., y_mm=45., heading_deg=90.,
        position_tolerance_mm=5., heading_tolerance_deg=2.)
    result = measure(Path(capture['video_file']), reference, geometry, placement)
    video = cv2.VideoCapture(capture['video_file'])
    video.set(cv2.CAP_PROP_ORIENTATION_AUTO, 0)
    video.set(cv2.CAP_PROP_POS_MSEC, 2000)
    ok, frame = video.read()
    video.release()
    if ok:
        cv2.imwrite(str(BASE/(tag+'_static_frame.jpg')), frame)
    save(BASE/(tag+'_static_check.json'), result)
    print(json.dumps(result, ensure_ascii=False), flush=True)
else:
    raise ValueError('prepare or capture')
