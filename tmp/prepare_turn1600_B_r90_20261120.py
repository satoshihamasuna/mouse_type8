"""機体Bのturn1600右90°を準備する。実機接続・走行指令は送らない。"""
from pathlib import Path
import copy
import hashlib
import json
import shutil
import sys

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tools/logs/pixel_camera/turn1600_B_r90_20261120_v1'
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
    source_ref = ROOT / 'tools/logs/pixel_camera/turn1800_A_inr135_20261120_v1/camera_reference_004_fixed10_v1/marker_reference_A_current_rotated004_fixed10.json'
    source_geo = ROOT / 'tools/turn_analysis/20261114_B_turn500_seed_from_A/body_geometry_B_zoom2_v8.json'
    reference = BASE / 'marker_reference_current.json'
    geometry = BASE / 'body_geometry_B_current.json'
    shutil.copyfile(source_ref, reference)
    body = read(source_geo)
    body['previous_reference_id'] = body['reference_id']
    body['reference_id'] = read(reference)['reference_id']
    body['reference_binding_source'] = '同じ盤面・高さ1500mmをユーザー確認。今日の1.8倍基準へ再結合。B検出は新しい静止画像で要検証。'
    body['camera_height_source'] = '2026-11-20 ユーザー「一緒です」：高さ1500mm・同じ調整盤を継続'
    save(geometry, body)
    camera = load_camera_profile('tuning')
    config = dict(mouse='B', address='C1:B5:60:87:91:73', height_mm=1500.,
        reference=str(reference), geometry=str(geometry), char_delay=.08,
        seconds=20, camera_settings=camera)
    save(BASE / 'config.json', config)
    previous = read(ROOT / 'tools/turn_analysis/20261114_B_turn500_seed_from_A/B_R90_single_turn_candidate.settings.json')
    settings = dict(schema_version=1, mouse='B', mouse_identity_source='user_supplied',
        motion_name='long_r90', preset_speed_mm_s=1600, turn_count=1,
        parameters=dict(velo=1.6, r_min=-52., lstart=12., lend=26.31,
            degree=-90., correction=1.5, pre_accel=6.5),
        pid=dict(sp_kp=4., sp_ki=.05, sp_kd=0., om_kp=.2, om_ki=.01, om_kd=0.),
        feedforward=copy.deepcopy(previous['feedforward']), suction=dict(enabled=True,duty=650),
        source='B現行turn1600の形状・PID。FFはB過去実機値を準備用に引用し、今回読返しで確認する。未送信・未実走。')
    save(BASE / 'candidate.settings.json', settings)
    service = TuningService(BASE / 'offline_validation', load_config(BASE / 'config.json'))
    service.trial_request(settings, True)
    save(BASE / 'preparation.json', dict(state='PREPARED_NOT_CONNECTED_NOT_EXECUTED', mouse='B',
        motion='long_r90', preset_mm_s=1600, count=1, suction=settings['suction'],
        planned_start_board_mm=[315.,45.], planned_heading_image_deg=90.,
        front_colour='yellow', rear_colour='red', pre_run_mm=180., post_run_mm=180.,
        reference_nominal_stop_board_mm=[45.,315.],
        placement_basis='同じ盤のA1600右90°で実使用した格子点。B500の過去配置と今回の全接地点・穴・車幅・停止余裕を新規照合する。',
        camera_height_mm=1500., same_camera_board_user_confirmed=True, camera_settings=camera,
        robot_power_state='OFF案内済み。ユーザーの同条件回答はあるが、物理OFFの明示確認は未取得。',
        placement_confirmed=False, hands_clear_confirmed=False, clearance_confirmed=False,
        execution_commands_sent=0, settings_write_commands_sent=0, execution_pending=False,
        firmware_write_requested=False, firmware_version='今回実機未読。最新の保存書込み記録は2026-11-16。',
        wall_ekf='pending device read', wall_edge='pending device read', turn_ekf='pending device read; B未校正',
        ekf_identification='Bへの既存A専用同定ツール適用は未対応。AのK・雑音を流用しない。',
        source_hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in [ROOT/'Params/Param_B/turn_1600/turn_1600.cpp', ROOT/'Params/Param_B/run_param_B.cpp',
                      ROOT/'Params/mouse_config.h', ROOT/'System/Src/myshell.cpp', source_ref, source_geo]}))
    print(json.dumps(dict(state='PREPARED', folder=str(BASE), parameters=settings['parameters'],camera=camera),ensure_ascii=False))
elif action == 'capture':
    from pixel_camera_capture import camera_workspace, record_trial
    sys.path.insert(0,str(camera_workspace()/'python'))
    from pixel_camera import PixelCameraClient
    tag = sys.argv[2] if len(sys.argv)>2 else 'initial'
    if not tag.replace('_','').isalnum():
        raise ValueError('英数字の撮影タグが必要')
    output = BASE/(tag+'_static_capture.json')
    if output.exists():
        raise RuntimeError('撮影記録の上書き・同じタグの再送はしない')
    client = PixelCameraClient(connection='wifi',port=8765,auto_start=False)
    status = client.get_status()
    if not status.get('connected') or status.get('device',{}).get('phase')!='IDLE':
        raise RuntimeError('カメラ接続・IDLEを確認できない')
    capture_config = read(Path(sys.argv[3]) if len(sys.argv)>3 else BASE/'config.json')
    manifest,result = record_trial(client,ROOT,seconds=3,mouse='B',
        label='B_turn1600_r90_'+tag+'_static_no_robot_command',
        camera_settings=capture_config['camera_settings'],capture_profile='tuning')
    save(output,dict(manifest=str(manifest),**result))
    print(json.dumps({k:result.get(k) for k in ('state','video_file','error')},ensure_ascii=False),flush=True)
elif action == 'inspect':
    import cv2
    from pixel_camera_static_pose import measure
    tag = sys.argv[2]
    if not tag.replace('_','').isalnum():
        raise ValueError('英数字の撮影タグが必要')
    capture = read(BASE/(tag+'_static_capture.json'))
    if capture['state'] != 'PASSED':
        raise RuntimeError('撮影品質の確認に失敗している')
    config = read(Path(sys.argv[3]) if len(sys.argv)>3 else BASE/'config.current.json')
    placement = dict(x_mm=315.,y_mm=45.,heading_deg=90.,position_tolerance_mm=5.,heading_tolerance_deg=2.)
    checked = measure(Path(capture['video_file']),read(config['reference']),read(config['geometry']),placement)
    save(BASE/(tag+'_static_check.json'),checked)
    video = cv2.VideoCapture(capture['video_file'])
    video.set(cv2.CAP_PROP_ORIENTATION_AUTO,0)
    video.set(cv2.CAP_PROP_POS_MSEC,2000)
    ok,frame = video.read()
    video.release()
    if ok and not (BASE/(tag+'_static_frame.jpg')).exists():
        cv2.imwrite(str(BASE/(tag+'_static_frame.jpg')),frame)
    print(json.dumps({k:checked.get(k) for k in ('passed','frames','observed','center_board_mm',
        'heading_deg','heading_budget_deg','heading_jitter_rms_deg','heading_half_window_difference_deg',
        'failure_reasons')},ensure_ascii=False),flush=True)
else:
    raise ValueError('prepare/capture/inspect')
