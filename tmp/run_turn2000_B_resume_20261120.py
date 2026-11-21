"""今回の操作者確認後だけ、設定照合・短い通電静止検査・録画付き右90°を1回行う。再送なし。"""
from pathlib import Path
import argparse
import hashlib
import json
import math
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'.tools/ble_python311'), str(ROOT/'tools')]
from myshell_debug import PARAM_RE
from pixel_camera_capture import record_trial
from pixel_camera_tuning import LiveSession, setting_differences, settings_set_command
from pixel_camera_tuning_cli import TuningService, load_config
from pixel_camera_turn_capture import utc_now

PREP = ROOT/'tools/logs/pixel_camera/turn2000_B_r90_20261120_v1/resume_20261120_detector_v1'
IDENTIFIER = 'B_R90_2000_omKi005_green_resume_20261120_001'
KEYS = ('distance','acc','max_velo','end_velo','sp_kp','sp_ki','sp_kd','om_kp','om_ki','om_kd',
        'ff_sp_velo','ff_sp_accel','ff_sp_bias','ff_om_velo','ff_om_accel','ff_om_decel','ff_om_bias')
read = lambda p: json.loads(Path(p).read_text(encoding='utf-8'))


def save(path, value):
    with Path(path).open('x',encoding='utf-8') as stream:
        json.dump(value,stream,ensure_ascii=False,indent=2,allow_nan=False)


def straight_value(line):
    match = PARAM_RE.search(line)
    if not match:
        raise ValueError('直進実機値を取得できない')
    value = {key:float(match[key]) for key in KEYS}
    value.update(suction=int(match['suction']),duty=int(match['duty']))
    if not all(math.isfinite(v) for v in value.values()):
        raise ValueError('非有限の直進実機値')
    return value


def main(check_only):
    config, expected = load_config(PREP/'config.candidate.json'), read(PREP/'candidate.settings.json')
    plan = read(PREP/'ready_plan.json')
    for path, digest in plan['source_sha256'].items():
        if hashlib.sha256(Path(path).read_bytes()).hexdigest() != digest:
            raise RuntimeError('準備後に検出・設定の実装が変更された。走行しない: '+path)
    if not plan['ready_for_operator_confirmation']:
        raise RuntimeError('保存動画・現在OFF像の準備が完了していない')
    if check_only:
        TuningService(PREP/'offline_request_validation',config).trial_request(expected,True)
        print('Software request and source hashes verified; no device connection.',flush=True)
        return
    confirmation = read(PREP/'operator_ready.json')
    if confirmation.get('request_id') != IDENTIFIER or confirmation.get('ready_for_one_run') is not True:
        raise RuntimeError('今回1回の操作者確認がない')
    folder = PREP/'run_session01'
    folder.mkdir(exist_ok=False)
    save(folder/'operator_ready.json',confirmation)
    result = dict(state='PREPARING',request_id=IDENTIFIER,started_at=utc_now(),execution_resend_count=0)
    session = LiveSession(ROOT,folder/'transport')
    try:
        session.prepare(config['address'])
        service = TuningService(folder,config,session=session,
            confirmation_source='Current operator response recorded in operator_ready.json; powered static and device checks required before one capture.')
        preflight = folder/'preflight';preflight.mkdir()
        with service.shell(preflight) as shell:
            before = service.read_settings(shell,'long_r90')
            result['turn_before'] = before
            if before['preset_speed_mm_s'] not in (700,2000):
                raise RuntimeError('別条件のRAMを検出。preset/set/exeを送らない')
            preset = service.read_settings(shell,'long_r90',2000)
            if preset['feedforward'] != expected['feedforward']:
                raise RuntimeError('前回のB2000 FFと実機が一致しない。比較走行を中止')
            for name,prefix in [('wall_ekf','WALL_EKF '),('wall_edge','WALL_EDGE '),('turn_ekf','TURN_EKF ')]:
                result[name] = shell.query(name+' show',lambda line,prefix=prefix:line.startswith(prefix),timeout=3)
            before_straight = straight_value(shell.query('debug straight show',lambda line:line.startswith('DEBUG_PARAM straight ')))
            result['straight_before'] = before_straight
            wanted = dict(before_straight,acc=20.)
            if before_straight['acc'] != 20.:
                command = 'debug straight set '+' '.join(f'{wanted[k]:.7g}' for k in KEYS)+f" {wanted['suction']} {wanted['duty']}"
                shell.send(command);shell.wait_line(lambda line:line=='DEBUG_PARAM_SET_DONE',timeout=8)
            actual_straight = straight_value(shell.query('debug straight show',lambda line:line.startswith('DEBUG_PARAM straight ')))
            if any(abs(actual_straight[k]-wanted[k])>1e-6 for k in wanted):
                raise RuntimeError('直進読み返し不一致。再送しない')
            result['straight_actual'] = actual_straight
            if setting_differences(preset,expected):
                shell.send(settings_set_command(expected))
                shell.wait_line(lambda line:line=='DEBUG_TURN_SET_DONE',timeout=8)
            actual = service.read_settings(shell,'long_r90')
            if setting_differences(actual,expected,quantized=True):
                raise RuntimeError('ターン読み返し不一致。再送しない')
            save(preflight/'actual.settings.json',actual)
        print('Device2000/Ki0.005/FF/straight20 verified. Capturing powered static only.',flush=True)
        manifest, capture = record_trial(session.camera,ROOT,seconds=3,mouse='B',
            label='B2000_resume_powered_static_no_execution',capture_profile='tuning',
            camera_settings=config['camera_settings'])
        capture_path = folder/'powered_static_capture.json'
        save(capture_path,dict(manifest=str(manifest),**capture))
        completed = subprocess.run(['D:/venv/venv3-11/Scripts/python.exe',
            str(ROOT/'tmp/check_turn2000_B_powered_static_20261120.py'),str(capture_path),
            str(PREP/'config.candidate.json'),str(folder/'powered_static_check.json')],timeout=35)
        if completed.returncode != 0 or not read(folder/'powered_static_check.json')['passed']:
            raise RuntimeError('通電静止検査に不合格。走行せず終了する')
        print('Powered static passed. Submitting the authorised single recorded turn.',flush=True)
        result['submitted'] = service.submit('capture',dict(settings=expected,operator_ready=True,defer_analysis=True),IDENTIFIER)
        while True:
            job = service.status(IDENTIFIER)
            if job['state'] in ('DONE','FAILED'):
                break
            time.sleep(.25)
        result.update(state=job['state'],job=job)
    except Exception as error:
        result.update(state='FAILED',error=type(error).__name__+': '+str(error))
    finally:
        session.close()
        result.update(ble_disconnected=True,finished_at=utc_now())
        save(folder/'controller_result.json',result)
        print(json.dumps({k:v for k,v in result.items() if k not in ('turn_before','job')},ensure_ascii=True),flush=True)
    if result['state'] != 'DONE':
        raise SystemExit(1)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check',action='store_true',help='設定・実装ハッシュ照合のみ。接続しない')
    main(parser.parse_args().check)
