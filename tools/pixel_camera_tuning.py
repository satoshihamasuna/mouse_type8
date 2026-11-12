"""既存debug GUIの接続を保持して、Pixel撮影付き単体試験を行う。"""
from __future__ import annotations

import csv
import argparse
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import threading
import time

from pixel_camera_turn_capture import (TURNS, MotionOnce, Shell, read_rows, save_motion_slice,
    utc_now, validate_recording, validate_settings, write_json)


def hidden_process():
    return subprocess.CREATE_NO_WINDOW if sys.platform == 'win32' else 0


def last_health(path):
    if not path.exists():
        raise RuntimeError('連続CSVの通信状態を受信していません')
    with path.open('rb') as stream:
        stream.seek(max(0, path.stat().st_size-8192))
        lines = stream.read().splitlines()
    for line in reversed(lines):
        try:
            value = json.loads(line)
            if all(key in value for key in ('rows', 'lost', 'crc_errors', 'monotonic')):
                return value
        except (ValueError, UnicodeError):
            pass
    raise RuntimeError('連続CSVの通信状態が不完全です')


def setting_differences(actual, expected, *, quantized=False):
    """意図値と実機応答を比較。set後だけ整数表示1単位の切捨てを許容する。"""
    differences = []
    for key in ('mouse', 'motion_name', 'preset_speed_mm_s', 'turn_count'):
        if actual.get(key) != expected.get(key):
            differences.append(key)
    for group in ('parameters', 'pid', 'feedforward', 'suction'):
        want, have = expected.get(group, {}), actual.get(group, {})
        if not want or set(have) != set(want):
            differences.append(group)
            continue
        for key, value in want.items():
            other = have[key]
            if isinstance(value, bool) or group == 'suction':
                equal = type(other) is type(value) and other == value
            else:
                scale = (1e-3 if group == 'parameters' else 1e-6)
                if key in ('ff_om_jerk', 'ff_sp_turn_accel_mag', 'ff_sp_turn_velo_sq'):
                    scale = 1e-9
                equal = (isinstance(other, (int, float)) and math.isfinite(other)
                         and math.isfinite(value) and abs(other-value) <= (scale if quantized else 0)+1e-12)
            if not equal:
                differences.append(group+'.'+key)
    return differences


TURN_PARAMETER_KEYS = ('velo', 'r_min', 'lstart', 'lend', 'degree', 'correction', 'pre_accel')
TURN_PID_KEYS = ('sp_kp', 'sp_ki', 'sp_kd', 'om_kp', 'om_ki', 'om_kd')
TURN_FF_KEYS = ('ff_sp_velo', 'ff_sp_accel', 'ff_sp_bias', 'ff_om_velo', 'ff_om_accel',
                'ff_om_decel', 'ff_om_bias', 'ff_om_jerk', 'ff_sp_turn_accel_mag', 'ff_sp_turn_velo_sq')


def format_turn_set_command(motion, values, suction_enabled, duty, count):
    """GUIとCLIで共通の単位・引数順・数値表記を使う。"""
    if motion not in TURNS or len(values) != 23:
        raise ValueError('ターン名または設定項目数が不正です')
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
        raise ValueError('設定値は有限の数値にしてください')
    if values[0] <= 0 or values[1] == 0 or values[2] < 0 or values[3] < 0 or values[4] == 0 or values[6] <= 0:
        raise ValueError('速度・半径・角度・入口出口長・前走加速度が不正です')
    right = '_r' in motion or motion.startswith('r_')
    if any((v >= 0 if right else v <= 0) for v in (values[1], values[4])):
        raise ValueError('右は半径・角度が負、左は正です')
    if any(v < 0 for v in values[13:]):
        raise ValueError('FF係数は0以上にしてください')
    if type(suction_enabled) is not bool or type(duty) is not int or not 0 <= duty <= 990 or type(count) is not int or not 1 <= count <= 100:
        raise ValueError('吸引設定または回数が不正です')
    return 'debug turn {} set {} {} {} {}'.format(
        motion, ' '.join(f'{v:.7g}' for v in values), int(suction_enabled), duty, count)


def settings_set_command(settings):
    values = [settings[group][key] for group, keys in (
        ('parameters', TURN_PARAMETER_KEYS), ('pid', TURN_PID_KEYS), ('feedforward', TURN_FF_KEYS)) for key in keys]
    return format_turn_set_command(settings['motion_name'], values, settings['suction']['enabled'],
                                   settings['suction']['duty'], settings['turn_count'])


def analyze_trial(folder, stride):
    """保存済み試行だけを解析し、GUI・CLIの同じ品質条件を適用する。"""
    folder = Path(folder)
    if stride not in (1, 4):
        raise ValueError('frame stride must be1 or4')
    with (folder/('quick.stdout.txt' if stride == 4 else 'full.stdout.txt')).open('x', encoding='utf-8') as output:
        return subprocess.call([str(analysis_python()), str(Path(__file__).parent/'analysis/pixel_camera_tuning_report.py'),
            '--trial', str(folder/'trial.json'), '--stride', str(stride)], stdout=output, stderr=subprocess.STDOUT,
            env=dict(os.environ, PYTHONUTF8='1'), creationflags=hidden_process())


@dataclass(frozen=True)
class TrialRequest:
    expected: dict
    set_command: str
    char_delay: float
    height_mm: float
    reference: Path
    geometry: Path
    seconds: int = 20
    operator_ready: bool = False

    def validate(self):
        if not self.operator_ready:
            raise ValueError('設置・停止・手離しを毎回確認してください')
        validate_settings(self.expected, self.expected['motion_name'], self.expected['preset_speed_mm_s'])
        if self.expected.get('mouse') not in ('A', 'B') or self.seconds not in (20, 30):
            raise ValueError('機体A/Bと録画上限20秒または30秒を指定してください')
        if not math.isfinite(self.height_mm) or not 20 < self.height_mm < 5000:
            raise ValueError('今回のカメラ高さをmmで指定してください')
        if not math.isfinite(self.char_delay) or self.char_delay < 0:
            raise ValueError('文字送信間隔が不正です')
        if not self.set_command.startswith(f"debug turn {self.expected['motion_name']} set "):
            raise ValueError('GUIの設定コマンドがターン名と一致しません')
        if any(c in self.set_command for c in '\r\n'):
            raise ValueError('複数コマンドは指定できません')
        if setting_differences(self.expected, self.expected):
            raise ValueError('設定に欠落・非有限値があります')
        reference = json.loads(self.reference.read_text(encoding='utf-8'))
        geometry = json.loads(self.geometry.read_text(encoding='utf-8'))
        if geometry.get('mouse') != self.expected['mouse'] or geometry.get('reference_id') != reference.get('reference_id'):
            raise ValueError('機体・盤面参照と画像プロファイルが一致しません')
        if any(not isinstance(geometry.get(key), (int,float)) or not math.isfinite(geometry[key])
               or not 0 <= geometry[key] < self.height_mm for key in ('front_height_mm','rear_height_mm')):
            raise ValueError('目印高さが今回のカメラ高さに適合しません')
        if geometry.get('axle_offset_from_marker_midpoint_mm') not in (None,[0,0]):
            raise ValueError('真下不明の相対解析は目印中点だけに対応します')


class CsvTail:
    """中継の追記CSVだけを読む。更新中のstatus.jsonを開かない。"""
    def __init__(self, source, output):
        self.source = source.open('r', encoding='utf-8', newline='')
        self.header = next(csv.reader([self.source.readline()]))
        if 'time_ms' not in self.header or 'sequence' not in self.header:
            self.source.close()
            raise RuntimeError('連続CSVのヘッダーがありません')
        # UTF-8の日本語を含まない数値CSV。最後の未完成行は完成後に一度だけ読む。
        self.source.seek(0, 2)
        end = self.source.tell()
        if end:
            self.source.seek(end-1)
            if self.source.read(1) != '\n':
                self.source.seek(0)
                position = 0
                while True:
                    before = self.source.tell()
                    line = self.source.readline()
                    if not line or not line.endswith('\n'):
                        position = before
                        break
                self.source.seek(position)
        self.output = output.open('x', encoding='utf-8', newline='')
        self.output.write(','.join(self.header)+'\n')
        self.rows, self.error, self.latest_at = [], None, 0.0
        self.done, self.lock = threading.Event(), threading.Lock()
        self.thread = threading.Thread(target=self._read, daemon=True)
        self.thread.start()

    def _read(self):
        try:
            while True:
                position = self.source.tell()
                line = self.source.readline()
                if not line or not line.endswith('\n'):
                    self.source.seek(position)
                    if self.done.wait(.025):
                        break
                    continue
                values = next(csv.reader([line]))
                if len(values) != len(self.header):
                    raise RuntimeError('連続CSVの列数が変わりました')
                with self.lock:
                    self.rows.append(dict(zip(self.header, values)))
                    self.latest_at = time.monotonic()
                self.output.write(line)
                self.output.flush()
        except Exception as error:
            self.error = error

    def snapshot(self):
        if self.error:
            raise self.error
        with self.lock:
            return list(self.rows)

    def close(self):
        self.done.set()
        self.thread.join(timeout=3)
        if self.thread.is_alive():
            raise RuntimeError('CSV保存処理を終了できません')
        self.output.close()
        self.source.close()
        if self.error:
            raise self.error


def log_stop_evidence(rows):
    """指令の動作と、その後200ms以上の実測停止を要求する。"""
    try:
        if any(not math.isfinite(float(row[key])) for row in rows
               for key in ('time_ms','ideal.velo','ideal.rad_velo','ego.velo','ego.rad_velo')):
            return dict(confirmed=False, reason='nonfinite_motion_log')
    except (KeyError, ValueError, TypeError):
        return dict(confirmed=False, reason='invalid_motion_log')
    active = [i for i, row in enumerate(rows)
              if abs(float(row['ideal.velo'])) > .01 or abs(float(row['ideal.rad_velo'])) > .01]
    if not active or active[0] == 0:
        return dict(confirmed=False, reason='no_complete_commanded_motion')
    tail = rows[active[-1]+1:]
    still = []
    for row in tail:
        values = [float(row[k]) for k in ('ideal.velo', 'ideal.rad_velo', 'ego.velo', 'ego.rad_velo')]
        if not all(math.isfinite(v) for v in values) or abs(values[2]) > .02 or abs(values[3]) > .15:
            still = []
        else:
            if still and not 0 < (int(row['time_ms'])-int(still[-1]['time_ms'])) % 2**32 <= 25:
                still = []
            still.append(row)
    span = (int(still[-1]['time_ms'])-int(still[0]['time_ms'])) % 2**32 if still else 0
    return dict(confirmed=len(still) >= 10 and 200 <= span < 60000,
                stationary_ms=span, stationary_rows=len(still),
                stationary_start_time_ms=int(still[0]['time_ms']) if still else None)


class LiveSession:
    """明示した準備操作でだけ起動する。一度接続した中継を試行間で再利用する。"""
    def __init__(self, project, folder):
        self.project, self.folder = Path(project), Path(folder)
        self.gateway = self.connection = self.camera = None

    def prepare(self, address, port=8766):
        from pixel_camera_capture import camera_workspace
        workspace = camera_workspace()
        sys.path.insert(0, str(workspace/'python'))
        try:
            import av  # 録画やsetより先に依存を確認する。
        except ImportError as error:
            raise RuntimeError('カメラ用環境が必要です。Start_Pixel_Tuning.cmdから開いてください') from error
        import serial
        from pixel_camera import PixelCameraClient
        check = subprocess.run([str(analysis_python()), '-c',
            'import cv2,numpy,pandas,scipy,matplotlib; assert hasattr(cv2,"aruco")'],
            capture_output=True, creationflags=hidden_process())
        if check.returncode:
            raise RuntimeError('画像解析環境がありません。TYPE8_ANALYSIS_PYTHONを設定してください')
        self.camera = PixelCameraClient(connection='wifi', port=8765, auto_start=False)
        status = self.camera.get_status()
        if not status.get('connected') or status.get('device', {}).get('phase') != 'IDLE':
            raise RuntimeError('Pixel CameraをPCへ接続し、録画・転送の終了を待ってください')
        self.folder.mkdir(parents=True, exist_ok=False)
        command = [sys.executable, str(self.project/'tools/ble/mouse_ble_gateway.py'),
                   '--port', str(port), '--address', address,
                   '--capture-output', str(self.folder/'telemetry.csv'), '--capture-period-ms', '5']
        try:
            with (self.folder/'gateway.stdout.txt').open('xb') as out, (self.folder/'gateway.stderr.txt').open('xb') as err:
                self.gateway = subprocess.Popen(command, cwd=self.project, stdout=out, stderr=err,
                    env=dict(os.environ, PYTHONUTF8='1'), creationflags=hidden_process())
            deadline = time.monotonic()+35
            while f'Ready: socket://127.0.0.1:{port}' not in (self.folder/'gateway.stdout.txt').read_text(encoding='utf-8', errors='replace'):
                if not self.alive() or time.monotonic() > deadline:
                    raise RuntimeError('BLE中継を準備できません。他のBLEツールを切断し、gateway.stderr.txtを確認してください')
                time.sleep(.1)
            self.connection = serial.serial_for_url(f'socket://127.0.0.1:{port}', timeout=.2, write_timeout=3)
            deadline = time.monotonic()+5
            while True:
                try:
                    if last_health(self.folder/'telemetry.health.jsonl').get('rows'):
                        break
                except RuntimeError:
                    pass
                if not self.alive() or time.monotonic() > deadline:
                    raise RuntimeError('連続CSVを受信できません')
                time.sleep(.1)
        except BaseException:
            self.close()
            raise
        return self

    def alive(self):
        return self.gateway is not None and self.gateway.poll() is None

    def close(self):
        if self.connection is not None:
            self.connection.close()
        if self.gateway is not None:
            try:
                self.gateway.wait(timeout=4)
            except subprocess.TimeoutExpired:
                self.gateway.terminate()
                self.gateway.wait(timeout=4)


def capture_connected(session, request, folder, parse, progress=lambda *args: None,
                      record=None, shell_type=Shell, tail_type=CsvTail):
    """呼出し側はGUI通信ロックを保持する。既存接続を閉じず、実行は既存MotionOnceだけ。"""
    from pixel_camera_capture import record_trial
    request.validate()
    record = record or record_trial
    if not session.alive() or not session.connection.is_open:
        raise RuntimeError('撮影用の接続を準備し直してください')
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=False)
    expected = request.expected
    mouse, turn, preset = expected['mouse'], expected['motion_name'], expected['preset_speed_mm_s']
    value = dict(schema_version=1, state='PREPARING', started_at=utc_now(), mouse=mouse,
        mouse_identity_source='user_supplied', motion=turn, preset=preset, turn_count=1,
        suction_enabled=False, settings_mode='gui_apply_then_show', stream_period_ms=5,
        camera_height_mm=request.height_mm, height_source='approximate_for_capture',
        camera_fixed_confirmed=True, execution_resend_count=0, physical_240fps_verified=False,
        parameter_adoption_allowed=False, type8_log_csv=str(folder/'telemetry.csv'),
        recording_stop_mode='motion_end_plus_postroll', recording_limit_seconds=request.seconds,
        recording_postroll_seconds=1.0,
        type8_settings_json=str(folder/'telemetry.settings.json'), live_session=str(session.folder),
        quick_report=str(folder/'quick/summary.json'), full_report=str(folder/'full/summary.json'))
    for source, name in ((request.reference, 'marker_reference_used.json'), (request.geometry, 'body_geometry_used.json')):
        write_json(folder/name, json.loads(source.read_text(encoding='utf-8')))
    write_json(folder/'requested.settings.json', expected)
    write_json(folder/'trial.json', value)
    tail, motion, settings, manifest, camera_value = None, None, None, None, None
    clock = None
    with (folder/'events.jsonl').open('x', encoding='utf-8') as events, (folder/'pc.shell.bin').open('xb') as output:
        def event(name, **fields):
            nonlocal clock
            events.write(json.dumps(dict(at=utc_now(), event=name, **fields))+'\n')
            events.flush()
            if name == 'camera_motion_start_confirmed' and clock is not None:
                value['video_motion_start_estimate_s'] = clock[0]+time.monotonic()-clock[1]
            progress(name, fields)
        try:
            status = session.camera.get_status()
            if not status.get('connected') or status.get('device', {}).get('phase') != 'IDLE':
                raise RuntimeError('Pixelが接続済み・待機中ではありません')
            health = last_health(session.folder/'telemetry.health.jsonl')
            if time.monotonic()-health['monotonic'] > 3:
                raise RuntimeError('連続ログの通信状態が古いため開始しません')
            baseline_crc = health['crc_errors']
            tail = tail_type(session.folder/'telemetry.csv', folder/'telemetry.csv')
            shell = shell_type(session.connection, output, event, char_delay=request.char_delay)
            shell.send('shell')
            time.sleep(.5)
            shell.check_battery()
            capability = shell.query('debug camera show', lambda line: line.startswith('DEBUG_CAMERA_CAP '))
            if capability != 'DEBUG_CAMERA_CAP protocol:1 ttl_ms:15000':
                raise RuntimeError('撮影専用の一回許可に対応するファームウェアが必要です')
            def show():
                command = f'debug turn {turn} show'
                line = shell.query(command, lambda line: line.startswith('DEBUG_TURN_PARAM_X1000 '+turn+' '))
                result = parse(line, turn, command) or {}
                validate_settings(result, turn, preset)
                result.update(mouse=mouse, mouse_identity_source='user_supplied')
                return result
            settings = show()
            if setting_differences(settings, expected):
                write_json(folder/'before.settings.json', settings)
                settings = None  # 途中送信の結果が不明なら、変更前値を実行時設定としない。
                event('settings_write_attempt', command=request.set_command)
                shell.send(request.set_command)
                shell.wait_line(lambda line: line == 'DEBUG_TURN_SET_DONE', timeout=8)
                settings = show()
                write_json(folder/'telemetry.settings.json', settings)
                differences = setting_differences(settings, expected, quantized=True)
                if differences:
                    raise RuntimeError('設定の読み返しが一致しません: '+', '.join(differences))
                value['settings_write_count'] = 1
            else:
                value['settings_write_count'] = 0
            write_json(folder/'telemetry.settings.json', settings)
            event('settings_verified', settings=settings, write_count=value['settings_write_count'])
            deadline = time.monotonic()+3
            while not tail.snapshot():
                if not session.alive() or time.monotonic() > deadline:
                    raise RuntimeError('今回の連続CSVを受信できません')
                time.sleep(.05)
            before = tail.snapshot()[-1]
            if time.monotonic()-tail.latest_at > .5 or any(not math.isfinite(float(before[k])) or abs(float(before[k])) > .01
                   for k in ('ideal.velo', 'ideal.rad_velo')):
                raise RuntimeError('他の動作が指令されているため開始しません')
            def recording_check():
                nonlocal clock
                if not session.alive() or not session.connection.is_open:
                    raise RuntimeError('BLE接続を確認できません')
                samples = tail.snapshot()
                if time.monotonic()-tail.latest_at > .5:
                    raise RuntimeError('連続CSVが途絶えたため走行を開始しません')
                if not samples or any(not math.isfinite(float(samples[-1][key])) or abs(float(samples[-1][key])) > .01
                                      for key in ('ideal.velo','ideal.rad_velo')):
                    raise RuntimeError('他の動作指令を検出したため走行を開始しません')
                status = session.camera.get_status()
                validate_recording(status, session.camera.last_recording_id, session.camera.client_id, request.seconds)
                clock = (status['device']['elapsed_seconds'], time.monotonic())
            motion = MotionOnce(shell, turn, event, recording_check)
            motion_log_start = max(0, len(tail.snapshot())-1)
            def stop_policy(client, recording_id, seconds):
                from pixel_camera_stop import wait_motion_stop
                if not motion.start_confirmed:
                    return dict(request_stop=False, fallback_reason='motion_start_not_confirmed')
                # 実行直前の停止行から読む。前回の動作を今回の停止条件にしない。
                class MotionTail:
                    @property
                    def latest_at(self):
                        return tail.latest_at
                    def snapshot(self):
                        return tail.snapshot()[motion_log_start:]
                return wait_motion_stop(client, recording_id, seconds, tail=MotionTail(),
                    health=lambda: last_health(session.folder/'telemetry.health.jsonl'),
                    baseline_crc=baseline_crc, alive=lambda: session.alive() and session.connection.is_open,
                    stop_evidence=log_stop_evidence, event=event)
            event('recording_requested', seconds=request.seconds)
            manifest, camera_value = record(session.camera, session.project, seconds=request.seconds,
                mouse=mouse, label=folder.name, on_ready=motion, stop_policy=stop_policy)
            value.update(capture_json=str(manifest), camera_state=camera_value['state'],
                         recording_stop=camera_value.get('recording_stop'),
                         video_file=camera_value.get('video_file'), recording_id=camera_value.get('recording_id'))
            if camera_value['state'] != 'PASSED' or not motion.start_confirmed:
                raise RuntimeError('録画・転送・開始応答を確認できません。実行は再送しません')
            event('recording_transferred', stop=camera_value.get('recording_stop'))
            value['state'] = 'CAPTURED_REVIEW_REQUIRED'
        except Exception as error:
            value.update(state='FAILED', error=type(error).__name__+': '+str(error))
        finally:
            if tail is not None:
                try:
                    tail.close()
                except Exception as error:
                    value.update(state='FAILED', error='CSV: '+str(error))
            value.update(ready_callback_invoked=bool(motion and motion.attempted),
                execution_write_attempted=bool(motion and motion.execution_write_attempted),
                camera_run_start_confirmed=bool(motion and motion.start_confirmed), trigger_mode='recording_only_without_hand')
    try:
        if settings is not None and (folder/'telemetry.csv').exists():
            quality = save_motion_slice(folder, settings)
            rows = read_rows(folder/'telemetry.csv')
            value['log_stop_evidence'] = log_stop_evidence(rows)
            deadline = time.monotonic()+3
            latest = None
            while rows:
                latest = last_health(session.folder/'telemetry.health.jsonl')
                if ((latest.get('latest_sequence', -1)-int(rows[-1]['sequence'])) % 2**32) < 2**31:
                    break
                if time.monotonic() > deadline:
                    latest = None
                    break
                time.sleep(.1)
            quality['crc_errors_during_trial'] = latest['crc_errors']-baseline_crc if latest else None
            value['log_quality'] = quality
            if quality.get('full_motion_retained'):
                value.update(type8_log_csv=str(folder/'motion.csv'), type8_settings_json=str(folder/'motion.settings.json'))
            selected = read_rows(folder/'motion.csv') if (folder/'motion.csv').exists() else []
            steps = [(int(b['time_ms'])-int(a['time_ms'])) % 2**32 for a,b in zip(selected,selected[1:])]
            finite = all(math.isfinite(float(row[key])) for row in selected
                         for key in ('ideal.velo','ideal.rad_velo','ego.velo','ego.rad_velo'))
            quality['motion_timebase_valid'] = bool(steps and all(0 < step <= 25 for step in steps) and finite)
            quality['maximum_motion_sample_gap_ms'] = max(steps) if steps else None
            if not quality.get('full_motion_retained') or quality.get('lost_frames') or quality['crc_errors_during_trial'] != 0 or not quality['motion_timebase_valid']:
                value['state'] = 'INCOMPLETE_LOG'
    except Exception as error:
        value.update(state='INCOMPLETE_LOG', error='CSV照合: '+str(error))
    value['finished_at'] = utc_now()
    write_json(folder/'trial.json', value)
    if manifest is not None:
        camera_value.update(type8_trial_json=str(folder/'trial.json'), type8_log_csv=value['type8_log_csv'],
                            type8_settings_json=value['type8_settings_json'], type8_execution_resend_count=0)
        write_json(manifest, camera_value)
    return value


def analysis_python():
    configured = os.environ.get('TYPE8_ANALYSIS_PYTHON')
    candidates = [Path(configured)] if configured else [Path('D:/venv/venv3-11/Scripts/python.exe'), Path(sys.executable)]
    return next((path for path in candidates if path.is_file()), candidates[-1])


def main():
    """Tk/PyAVのある既存カメラ環境で既存GUIを起動する。起動だけでは接続・走行しない。"""
    from pixel_camera_capture import camera_workspace, camera_python
    python = camera_python(camera_workspace())
    if Path(sys.executable).resolve() != python.resolve():
        return subprocess.call([str(python), str(Path(__file__).resolve()), *sys.argv[1:]], env=dict(os.environ, PYTHONUTF8='1'))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--review-trial', type=Path, help='保存済み試行を表示して開く。接続・設定送信・走行はしない')
    args = parser.parse_args()
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'.tools/ble_python311'))
    from ble.mouse_gui_ble import install_ble_port
    install_ble_port(8766)
    from myshell_debug import DebugGui
    gui = DebugGui()
    gui.port.set('BLE')
    gui.motion.set('turn')
    gui.turn_type.set('long_r180')
    gui.turn_preset_speed.set(500)
    for variable in gui.values.values():
        variable.set('')  # 初回の実機読取り前に別プリセットの値を表示しない。
    gui._update_motion_fields()
    gui.camera_panel.status.set('機体を選択し「撮影用接続を準備」から開始してください')
    if args.review_trial:
        gui.camera_panel.show_saved(args.review_trial)
    gui.mainloop()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
