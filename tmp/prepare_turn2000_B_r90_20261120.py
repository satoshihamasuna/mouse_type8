"""B turn2000右90°の準備と静止撮影。機体の接続・設定・走行は行わない。"""
from pathlib import Path
import copy
import hashlib
import json
import re
import shutil
import sys

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tools/logs/pixel_camera/turn2000_B_r90_20261120_v1'
OLD = ROOT / 'tools/logs/pixel_camera/turn1600_B_r90_20261120_v1'
sys.path[:0] = [str(ROOT / 'tools'), str(ROOT / 'tools/analysis')]


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def save(path, value):
    with Path(path).open('x', encoding='utf-8') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')


action = sys.argv[1]
if action == 'prepare':
    from pixel_camera_profiles import load_camera_profile
    from pixel_camera_tuning_cli import load_config, TuningService
    BASE.mkdir(exist_ok=False)
    config = read(OLD / 'config.current.json')
    for key in ('reference', 'geometry'):
        source = Path(config[key])
        destination = BASE / source.name
        shutil.copyfile(source, destination)
        config[key] = str(destination)
    config['camera_settings'] = load_camera_profile('tuning')
    save(BASE / 'config.json', config)
    source = ROOT / 'Params/Param_B/turn_2000/turn_2000.cpp'
    code = source.read_text(encoding='utf-8')
    tables = {}
    for name, body in re.findall(r'const t_turn_param_table\s+(\w+)\s*=\s*\{([^}]+)\}', code):
        parts = [v.strip() for v in body.split(',')]
        tables[name] = dict(zip(('velo', 'r_min', 'lstart', 'lend', 'degree'),
            (float(v.rstrip('fF')) for v in parts[:5])))
        tables[name]['correction'] = float(parts[6].rstrip('fF')) if len(parts) > 6 else 0.
    if len(tables) != 14:
        raise RuntimeError('B2000の14ターンを読み取れない')
    settings = copy.deepcopy(read(OLD / 'device_prepare03/actual.settings.json'))
    for key in ('captured_at', 'raw_response', 'command'):
        settings.pop(key, None)
    settings.update(source='B turn2000ソース初期値。FFはBの直近読返しを準備用に保存し、今回の実機値で再照合する。未送信・未実走。',
        preset_speed_mm_s=2000, turn_count=1)
    settings['parameters'] = dict(tables['slalom_R90_2000_table'], pre_accel=6.5)
    for kind, group in (('sp', 'sp'), ('om', 'om')):
        match = re.search(r'const t_pid_gain\s+' + kind + r'_gain_turn90_2000\s*=\s*\{([^}]+)\}', code)
        values = [float(v.strip().rstrip('fF')) for v in match[1].split(',')]
        settings.setdefault('pid', {}).update(dict(zip((group+'_kp', group+'_ki', group+'_kd'), values)))
    settings['suction'] = dict(enabled=True, duty=650)
    save(BASE / 'candidate.settings.json', settings)
    save(BASE / 'source_tables_B2000.json', dict(source=str(source), tables=tables,
        sha256=hashlib.sha256(source.read_bytes()).hexdigest()))
    TuningService(BASE / 'offline_validation', load_config(BASE / 'config.json')).trial_request(settings, True)
    save(BASE / 'preparation.json', dict(state='PREPARED_NOT_CONNECTED_NOT_EXECUTED', mouse='B',
        motion='long_r90', preset_mm_s=2000, count=1, suction=settings['suction'],
        planned_start_board_mm=[315.,45.], planned_heading_image_deg=90.,
        front_colour='yellow', rear_colour='red', pre_run_mm=180., post_run_mm=180.,
        ideal_connection_target_board_mm=[225.,315.], reference_target_stop_board_mm=[45.,315.],
        placement_basis='直前のB1600右90°で実使用した配置。今回の静止検査・全接地点・穴・車幅・停止余裕は改めて確認する。',
        camera_height_mm=1500., same_camera_board_user_confirmed=True,
        camera_settings=config['camera_settings'],
        effective_pre_accel_m_s2=2.**2/(2*.180), requested_post_straight_accel_m_s2=20.,
        ideal_stopping_distance_at6p5_mm=1000*2.**2/(2*6.5), ideal_stopping_distance_at20_mm=100.,
        physical_power='直前B1600終了時は両方OFFをユーザー確認。今回の物理状態は未確認、OFF継続を案内。',
        placement_confirmed=False, hands_clear_confirmed=False, clearance_confirmed=False,
        execution_commands_sent=0, settings_write_commands_sent=0, execution_pending=False,
        firmware_write_performed=False, source_params_changed=False,
        wall_ekf='pending device read', wall_edge='pending device read', turn_ekf='pending device read; B2000未校正',
        ekf_identification='A用同定値をBへ流用しない。対応動画・実測横加速度・静止零点とB対応解析の成立を確認してから候補評価。',
        evaluation_scope='実測入口に対する相対形状と盤面固定の差を別記。マーカー中点と車軸の一致は未校正。',
        reference_policy='各評価フレームで登録済みArUcoを2個以上実復号。欠測フレームの登録を補間しない。',
        next_action='カメラのみの静止撮影、配置案内、操作者の今回の設置確認後に実機値照合。',
        source_hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (source, ROOT/'Params/Param_B/run_param_B.cpp', ROOT/'System/Src/myshell.cpp',
                Path(config['reference']), Path(config['geometry']))}))
    print(json.dumps(dict(state='PREPARED', folder=str(BASE), parameters=settings['parameters'],
        pid=settings['pid'], suction=settings['suction']), ensure_ascii=True), flush=True)
elif action == 'capture':
    from pixel_camera_capture import camera_workspace, record_trial
    sys.path.insert(0, str(camera_workspace() / 'python'))
    from pixel_camera import PixelCameraClient
    tag = sys.argv[2]
    if not re.fullmatch(r'[A-Za-z0-9_]+', tag):
        raise ValueError('英数字の撮影タグが必要')
    output = BASE / (tag + '_static_capture.json')
    if output.exists():
        raise RuntimeError('既存撮影の上書き・同じタグの再送はしない')
    client = PixelCameraClient(connection='wifi', port=8765, auto_start=False)
    status = client.get_status()
    if not status.get('connected') or status.get('device', {}).get('phase') != 'IDLE':
        raise RuntimeError('カメラ接続・IDLEを確認できない')
    config_path = BASE/sys.argv[3] if len(sys.argv)>3 else BASE/'config.json'
    if not config_path.resolve().is_relative_to(BASE.resolve()):
        raise ValueError('今回の測定フォルダ内の設定を指定する')
    manifest, result = record_trial(client, ROOT, seconds=3, mouse='B',
        label='B_turn2000_r90_' + tag + '_static_no_robot_command',
        camera_settings=read(config_path)['camera_settings'], capture_profile='tuning')
    save(output, dict(manifest=str(manifest), configuration_source=str(config_path), **result))
    print(json.dumps({k:result.get(k) for k in ('state', 'video_file', 'error')}, ensure_ascii=True), flush=True)
elif action == 'inspect':
    import cv2
    from pixel_camera_static_pose import measure
    tag = sys.argv[2]
    if not re.fullmatch(r'[A-Za-z0-9_]+', tag):
        raise ValueError('英数字の撮影タグが必要')
    capture = read(BASE/(tag+'_static_capture.json'))
    if capture['state'] != 'PASSED':
        raise RuntimeError('静止撮影の品質検査不合格。元の結果を保持する。')
    config_path = BASE/sys.argv[4] if len(sys.argv)>4 else BASE/'config.json'
    if not config_path.resolve().is_relative_to(BASE.resolve()):
        raise ValueError('今回の測定フォルダ内の設定を指定する')
    config = read(config_path)
    placement = dict(x_mm=315., y_mm=45., heading_deg=90., position_tolerance_mm=5., heading_tolerance_deg=2.)
    relative = len(sys.argv) > 3 and sys.argv[3] == 'relative'
    if relative:
        placement = read(BASE/'Ki005_relative_start_plan.json')['placement']
    checked = measure(Path(capture['video_file']), read(config['reference']), read(config['geometry']), placement)
    checked['evaluation_frame'] = ('predeclared_actual_start_for_entry_relative_PID_comparison; original90degree_setup_not_passed'
        if relative else 'original_fixed_board_start; same target as powered01_original_board_static_check')
    save(BASE/(tag+'_static_check.json'), checked)
    video = cv2.VideoCapture(capture['video_file'])
    video.set(cv2.CAP_PROP_ORIENTATION_AUTO, 0)
    video.set(cv2.CAP_PROP_POS_MSEC, 2000)
    ok, frame = video.read()
    video.release()
    if ok:
        cv2.imwrite(str(BASE/(tag+'_static_frame.jpg')), frame)
    print(json.dumps({k:checked.get(k) for k in ('passed', 'frames', 'observed', 'center_board_mm',
        'heading_deg', 'heading_budget_deg', 'failure_reasons')}, ensure_ascii=True), flush=True)
elif action == 'guide':
    import cv2
    import numpy as np
    from turn_ff_report import load_turn_table, ACCEL_INTEGRAL
    from video_marker_pose import marker_registration, transform_points
    settings = read(BASE/'candidate.settings.json')
    p = settings['parameters']
    duration = abs(np.radians(p['degree'] + p['correction']) / (ACCEL_INTEGRAL * p['velo'] / (p['r_min']/1000)))
    times = np.linspace(0., duration, 2001)
    omega = p['velo']/(p['r_min']/1000)*np.interp(times/duration*1000, np.arange(1001), load_turn_table())
    dt = np.diff(times)
    yaw = np.r_[0., np.cumsum((omega[:-1]+omega[1:])*.5*dt)]
    xy = np.c_[np.r_[0., np.cumsum(np.sin(-(yaw[:-1]+yaw[1:])*.5)*p['velo']*1000*dt)],
        np.r_[0., np.cumsum(np.cos((yaw[:-1]+yaw[1:])*.5)*p['velo']*1000*dt)]]
    xy += [0., 180.+p['lstart']]
    endpoint = xy[-1] + np.array([np.sin(-yaw[-1]), np.cos(yaw[-1])])*p['lend']
    stop = endpoint + np.array([np.sin(-yaw[-1]), np.cos(yaw[-1])])*180.
    relative = np.vstack(([0.,0.], [0.,180.], xy, endpoint, stop))
    board = relative * [-1.,1.] + [315.,45.]
    save(BASE/'nominal_plan.json', dict(mouse='B', motion='long_r90', parameters=p,
        nominal_model='Firmware accel_table integrated at2000 intervals; ideal constant turn speed, no slip/control error.',
        turn_duration_s=float(duration), nominal_yaw_deg=float(np.degrees(yaw[-1])),
        nominal_start_board_mm=[315.,45.], nominal_Lend_board_mm=(endpoint*[-1,1]+[315,45]).tolist(),
        nominal_stop_board_mm=board[-1].tolist(), centerline_min_mm=board.min(axis=0).tolist(),
        centerline_max_mm=board.max(axis=0).tolist(), physical_full_contact_hole_clearance_verified=False,
        marker_midpoint_to_axle_offset_calibrated=False, execution_commands_sent=0))
    np.savetxt(BASE/'nominal_centerline_board_mm.csv', board, delimiter=',',header='x_mm,y_mm',comments='')
    frame = cv2.imread(str(BASE/'initial_static_frame.jpg'))
    config = read(BASE/'config.json')
    registration = marker_registration(frame, read(config['reference']))
    if not registration['available']:
        raise RuntimeError('この静止画像のArUco登録が成立しない')
    save(BASE/'guide_frame_registration.json', registration)
    inverse = np.linalg.inv(np.asarray(registration['homography_image_to_board_mm']))
    def px(points):
        return np.rint(transform_points(points, inverse)).astype(int)
    cv2.polylines(frame,[px(board)],False,(255,220,0),2,cv2.LINE_AA)
    start = tuple(px([[315,45]])[0])
    cv2.circle(frame,start,12,(255,255,255),2,cv2.LINE_AA)
    cv2.arrowedLine(frame,start,tuple(px([[315,108]])[0]),(0,255,255),3,cv2.LINE_AA,tipLength=.18)
    cv2.putText(frame,'B RIGHT90 / 2000 mm/s / SUCTION650',(25,30),cv2.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),2,cv2.LINE_AA)
    cv2.putText(frame,'KEEP midpoint near (315,45) mm',(825,195),cv2.FONT_HERSHEY_SIMPLEX,.46,(255,255,255),1,cv2.LINE_AA)
    cv2.putText(frame,'YELLOW FRONT: straight DOWN',(825,220),cv2.FONT_HERSHEY_SIMPLEX,.46,(0,255,255),1,cv2.LINE_AA)
    cv2.putText(frame,'Rotate CLOCKWISE about 2 deg',(825,245),cv2.FONT_HERSHEY_SIMPLEX,.46,(0,255,255),1,cv2.LINE_AA)
    cv2.putText(frame,'Nominal centerline only; operator checks tyres / holes / stopping space',(25,688),cv2.FONT_HERSHEY_SIMPLEX,.54,(255,255,255),2,cv2.LINE_AA)
    cv2.imwrite(str(BASE/'start_placement_guide.jpg'),frame)
    print(json.dumps(read(BASE/'nominal_plan.json'),ensure_ascii=True),flush=True)
else:
    raise ValueError('prepare/capture/inspect/guide')
