"""画面を操作せず、接続を保持してPixel撮影付き単体ターンを調整する。"""
from __future__ import annotations

import argparse
from contextlib import contextmanager
from copy import deepcopy
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import re
import secrets
import sys
import threading
import time
from urllib.error import HTTPError
from urllib.request import Request, urlopen
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / '.tools' / f'ble_python{sys.version_info.major}{sys.version_info.minor}'))

from myshell_debug import DebugGui, TURN_PRESET_SPEEDS
from pixel_camera_tuning import (LiveSession, TrialRequest, analyze_trial, capture_connected,
    setting_differences, settings_set_command)
from pixel_camera_turn_capture import TURNS, Shell, utc_now, validate_settings


def save_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + '.tmp')
    text = json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
    delays = (.05, .1, .2)
    for attempt in range(len(delays) + 1):
        try:
            temporary.write_text(text, encoding='utf-8')
            temporary.replace(path)
            return
        except PermissionError as error:
            if getattr(error, 'winerror', None) not in {5, 32, 33} or attempt == len(delays):
                raise
            time.sleep(delays[attempt])


def request_id(value):
    if not isinstance(value, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,80}', value):
        raise ValueError('request-idは英数字・ハイフン・下線の1～80文字です')
    return value


class BusyError(RuntimeError):
    pass


class TuningService:
    """コマンドを直列化し、消費済みIDをディスクへ残して実行の再送を拒否する。"""
    def __init__(self, folder, config, session=None):
        self.folder, self.config, self.session = Path(folder).resolve(), deepcopy(config), session
        self.folder.mkdir(parents=True, exist_ok=True)
        (self.folder / 'jobs').mkdir(exist_ok=True)
        self.operation_lock, self.state_lock = threading.Lock(), threading.RLock()
        self.on_close = lambda: None

    def trial_request(self, settings, ready):
        if settings.get('mouse') != self.config['mouse']:
            raise ValueError('設定JSONと対象機体が一致しません')
        request = TrialRequest(deepcopy(settings), settings_set_command(settings),
            self.config.get('char_delay', .08), self.config['height_mm'],
            Path(self.config['reference']), Path(self.config['geometry']),
            self.config.get('seconds', 20), ready)
        request.validate()
        return request

    def validate(self, action, payload):
        if action in ('read', 'preset'):
            if payload.get('motion') not in TURNS:
                raise ValueError('対応するターン名を指定してください')
            if action == 'preset' and (type(payload.get('preset')) is not int or payload['preset'] not in TURN_PRESET_SPEEDS):
                raise ValueError('対応するプリセット速度を指定してください')
        elif action in ('apply', 'capture'):
            # readyは要求ごとに必要。前回やサーバー起動時の確認を保持しない。
            if action == 'capture' and payload.get('operator_ready') is not True:
                raise ValueError('今回の設置・停止・手離し確認が必要です（--operator-ready）')
            self.trial_request(payload['settings'], True)
        elif action != 'close':
            raise ValueError('対応しない操作です')

    def status(self, identifier=None):
        if identifier is not None:
            path = self.folder / 'jobs' / (request_id(identifier) + '.json')
            with self.state_lock:
                return json.loads(path.read_text(encoding='utf-8'))
        session = self.session
        return dict(session=str(self.folder), connected=bool(session and session.alive()),
            busy=self.operation_lock.locked(), transport=str(session.folder) if session else None)

    def submit(self, action, payload, identifier):
        identifier = request_id(identifier)
        self.validate(action, payload)
        path = self.folder / 'jobs' / (identifier + '.json')
        if not self.operation_lock.acquire(blocking=False):
            raise BusyError('処理中です。statusで完了を確認してください')
        try:
            job = dict(id=identifier, action=action, state='ACCEPTED', submitted_at=utc_now(), request=deepcopy(payload))
            # IDを受理する段階で確保する。途中送信・応答消失・再起動でも再実行しない。
            with self.state_lock:
                with path.open('x', encoding='utf-8') as stream:
                    json.dump(job, stream, ensure_ascii=False, allow_nan=False)
            worker = threading.Thread(target=self._run, args=(path, job), daemon=False)
            worker.start()
        except BaseException:
            self.operation_lock.release()
            raise
        return dict(id=identifier, state='ACCEPTED')

    def _run(self, path, job):
        def publish(**values):
            with self.state_lock:
                job.update(values)
                save_json(path, job)
        try:
            publish(state='RUNNING', started_at=utc_now())
            result = self.perform(job['action'], job['request'], job['id'],
                                  lambda name, fields: publish(event=name, event_fields=fields))
            publish(state='DONE', result=result, finished_at=utc_now())
        except Exception as error:
            publish(state='FAILED', error=f'{type(error).__name__}: {error}', finished_at=utc_now(),
                    note='要求は再送しません。trial.jsonとevents.jsonlで結果を確認してください')
        finally:
            self.operation_lock.release()
        if job['action'] == 'close' and job['state'] == 'DONE':
            self.on_close()

    @contextmanager
    def shell(self, folder):
        if not self.session or not self.session.alive() or not self.session.connection.is_open:
            raise RuntimeError('BLE接続がありません。自動再接続・要求再送はしません')
        with (folder / 'shell.bin').open('xb') as output, (folder / 'events.jsonl').open('x', encoding='utf-8') as events:
            def event(name, **fields):
                events.write(json.dumps(dict(at=utc_now(), event=name, **fields)) + '\n')
                events.flush()
            shell = Shell(self.session.connection, output, event, char_delay=self.config.get('char_delay', .08))
            shell.send('shell')
            time.sleep(.5)
            shell.check_battery()
            yield shell

    def read_settings(self, shell, motion, preset=None):
        command = f'debug turn {motion} show' if preset is None else f'debug turn {motion} preset {preset}'
        line = shell.query(command, lambda value: value.startswith('DEBUG_TURN_PARAM_X1000 ' + motion + ' '))
        result = DebugGui._parse_device_settings(line, motion, command)
        if not result or result.get('error'):
            raise RuntimeError('実機設定を取得できません')
        result.update(mouse=self.config['mouse'], mouse_identity_source='user_supplied')
        validate_settings(result, motion, preset if preset is not None else result['preset_speed_mm_s'])
        settings_set_command(result)
        return result

    def perform(self, action, payload, identifier, progress):
        if action == 'close':
            self.close()
            return dict(closed=True)
        if not self.session or not self.session.alive():
            raise RuntimeError('BLEが切断されています。自動再接続はしません')
        folder = self.folder / (('run_' if action == 'capture' else 'command_') + identifier)
        if action == 'capture':
            request = self.trial_request(payload['settings'], payload['operator_ready'])
            result = capture_connected(self.session, request, folder, DebugGui._parse_device_settings, progress)
            save_json(folder / 'operator_confirmation.json', dict(source='explicit CLI --operator-ready',
                request_id=identifier, settings=request.expected, confirmed_for_this_request_only=True))
            output = dict(trial_json=str(folder / 'trial.json'), capture=result)
            if result['state'] == 'CAPTURED_REVIEW_REQUIRED':
                for stride, name in ((4, 'quick'), (1, 'full')):
                    progress(name + '_analysis', {})
                    try:
                        output[name + '_returncode'] = analyze_trial(folder, stride)
                        summary = folder / name / 'summary.json'
                        if summary.exists():
                            output[name] = json.loads(summary.read_text(encoding='utf-8'))
                    except Exception as error:
                        output[name + '_error'] = str(error)
                try:
                    from pixel_camera_archive import archive_trial, write_index
                    archived = archive_trial(folder / 'trial.json')
                    write_index(archived.parent)
                    output['archive'] = str(archived)
                except Exception as error:
                    output['archive_error'] = str(error)
            return output
        folder.mkdir(exist_ok=False)
        with self.shell(folder) as shell:
            if action in ('read', 'preset'):
                settings = self.read_settings(shell, payload['motion'], payload.get('preset'))
            else:
                expected = payload['settings']
                settings = self.read_settings(shell, expected['motion_name'])
                if settings['preset_speed_mm_s'] != expected['preset_speed_mm_s']:
                    raise RuntimeError('実機presetが一致しません。先にpresetコマンドで速度テーブルを選択してください')
                if setting_differences(settings, expected):
                    shell.send(settings_set_command(expected))
                    shell.wait_line(lambda line: line == 'DEBUG_TURN_SET_DONE', timeout=8)
                    settings = self.read_settings(shell, expected['motion_name'])
                    differences = setting_differences(settings, expected, quantized=True)
                    if differences:
                        raise RuntimeError('設定の読み返しが一致しません: ' + ', '.join(differences))
            save_json(folder / 'settings.json', settings)
        return settings

    def close(self):
        if self.session is not None:
            self.session.close()
            self.session = None


def make_http_server(service, token, port=8767):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def reply(self, code, value):
            data = json.dumps(value, ensure_ascii=False, allow_nan=False).encode('utf-8')
            self.send_response(code)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass  # クライアント切断でも受理済み要求を再実行しない。

        def authorized(self):
            if not hmac.compare_digest(self.headers.get('X-TYPE8-Key', ''), token):
                self.reply(403, dict(error='このセッションの接続情報が必要です'))
                return False
            return True

        def do_GET(self):
            if not self.authorized():
                return
            try:
                if self.path == '/status':
                    value = service.status()
                elif self.path.startswith('/jobs/'):
                    value = service.status(self.path[len('/jobs/'):])
                else:
                    raise FileNotFoundError(self.path)
                self.reply(200, value)
            except (ValueError, FileNotFoundError) as error:
                self.reply(404, dict(error=str(error)))

        def do_POST(self):
            if not self.authorized():
                return
            if self.path != '/jobs':
                self.reply(404, dict(error='unknown endpoint'))
                return
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 65536:
                    raise ValueError('要求サイズが不正です')
                value = json.loads(self.rfile.read(size))
                result = service.submit(value['action'], value.get('payload', {}), value['id'])
                self.reply(202, result)
            except (FileExistsError, BusyError) as error:
                self.reply(409, dict(error=str(error), note='同じIDの再送はせずstatusで結果を確認してください'))
            except (ValueError, TypeError, KeyError) as error:
                self.reply(400, dict(error=str(error)))
    return ThreadingHTTPServer(('127.0.0.1', port), Handler)


def serve(args):
    config = json.loads(args.config.read_text(encoding='utf-8'))
    if config.get('mouse') not in ('A', 'B') or not isinstance(config.get('height_mm'), (int, float)) \
            or not math.isfinite(config['height_mm']) or not 20 < config['height_mm'] < 5000:
        raise ValueError('configには対象機体A/Bと今回のheight_mmが必要です')
    delay = config.get('char_delay', .08)
    if not isinstance(delay, (int, float)) or not math.isfinite(delay) or delay < 0 or config.get('seconds', 20) not in (20, 30):
        raise ValueError('文字送信間隔または録画上限が不正です')
    for key in ('reference', 'geometry'):
        json.loads(Path(config[key]).read_text(encoding='utf-8'))
    folder = args.session.resolve()
    folder.mkdir(parents=True, exist_ok=True)
    lock = folder / 'service.lock'
    with lock.open('x', encoding='utf-8') as stream:
        stream.write(utc_now())
    service = TuningService(folder, config)
    server = None
    try:
        save_json(folder / 'config.used.json', config)
        transport = folder / ('transport_' + uuid.uuid4().hex[:8])
        service.session = LiveSession(ROOT, transport)
        service.session.prepare(config['address'])
        if 'exposure_compensation_steps' in config:
            camera = service.session.camera
            camera.connect()
            try:
                configured = camera.configure(exposure_compensation_steps=config['exposure_compensation_steps'], torch=False)
                if configured.get('exposure_compensation_steps') != config['exposure_compensation_steps'] or configured.get('torch') is not False:
                    raise RuntimeError('カメラの露出・ライト設定を照合できません')
                save_json(folder / 'camera.configured.json', configured)
            finally:
                camera.disconnect()
        token = secrets.token_hex(24)
        server = make_http_server(service, token, args.port)
        service.on_close = server.shutdown
        save_json(folder / 'service.json', dict(host='127.0.0.1', port=server.server_port,
            token=token, session=str(folder), started_at=utc_now()))
        print(json.dumps(dict(state='READY', session=str(folder)), ensure_ascii=False), flush=True)
        server.serve_forever(poll_interval=.2)
    finally:
        if server is not None:
            server.server_close()
        with service.operation_lock:
            service.close()
        lock.unlink(missing_ok=True)


def exchange(descriptor, path, payload=None):
    data = None if payload is None else json.dumps(payload, allow_nan=False).encode('utf-8')
    request = Request(f'http://127.0.0.1:{int(descriptor["port"])}{path}', data=data,
        headers={'X-TYPE8-Key': descriptor['token'], 'Content-Type': 'application/json'})
    # POSTは1回だけ。応答消失時にも再送せず、保存済みIDのGETで確認する。
    try:
        with urlopen(request, timeout=5) as response:
            return json.loads(response.read())
    except HTTPError as error:
        raise RuntimeError(error.read().decode('utf-8', errors='replace')) from error


def client(args):
    descriptor = json.loads((args.session / 'service.json').read_text(encoding='utf-8'))
    if args.action == 'status':
        return exchange(descriptor, '/jobs/' + request_id(args.request_id) if args.request_id else '/status')
    payload = {}
    if args.action in ('read', 'preset'):
        payload['motion'] = args.motion
        if args.action == 'preset':
            payload['preset'] = args.preset
    if args.action in ('apply', 'capture'):
        settings = json.loads(args.settings.read_text(encoding='utf-8'))
        settings = settings.get('actual_settings', settings)
        for change in args.set_value:
            key, value = change.split('=', 1)
            group, key = key.split('.', 1) if '.' in key else ('parameters', key)
            if group not in ('parameters', 'pid', 'feedforward') or key not in settings[group]:
                raise ValueError('対応しない設定項目: ' + change)
            settings[group][key] = float(value)
        payload['settings'] = settings
        if args.action == 'capture':
            if not args.operator_ready:
                raise ValueError('今回の設置確認には--operator-readyが必要です')
            payload['operator_ready'] = True
    identifier = args.request_id or ('cmd_' + uuid.uuid4().hex[:16])
    result = exchange(descriptor, '/jobs', dict(id=identifier, action=args.action, payload=payload))
    if args.no_wait or args.action == 'close':
        return result
    deadline = time.monotonic() + args.wait_seconds
    while True:
        result = exchange(descriptor, '/jobs/' + identifier)
        if result['state'] in ('DONE', 'FAILED'):
            if args.output and result['state'] == 'DONE':
                save_json(args.output, result['result'])
            return result
        if time.monotonic() >= deadline:
            return dict(id=identifier, state='WAIT_TIMEOUT', note='処理は継続しています。status --request-idで確認し、再送しないでください')
        time.sleep(.5)


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument('--session', type=Path, required=True, help='接続情報・要求・結果の保存フォルダー')
    sub = result.add_subparsers(dest='action', required=True)
    start = sub.add_parser('serve', help='画面なしでBLE接続・連続ログを保持する')
    start.add_argument('--config', type=Path, required=True)
    start.add_argument('--port', type=int, default=8767)
    status = sub.add_parser('status', help='状態照会だけ。機体へ指令を送らない')
    status.add_argument('--request-id')
    for action in ('read', 'preset', 'apply', 'capture', 'close'):
        command = sub.add_parser(action)
        command.add_argument('--request-id', required=action == 'capture')
        command.add_argument('--no-wait', action='store_true')
        command.add_argument('--wait-seconds', type=float, default=600)
        command.add_argument('--output', type=Path)
        if action in ('read', 'preset'):
            command.add_argument('--motion', choices=TURNS, required=True)
        if action == 'preset':
            command.add_argument('--preset', type=int, choices=TURN_PRESET_SPEEDS, required=True)
        if action in ('apply', 'capture'):
            command.add_argument('--settings', type=Path, required=True)
            command.add_argument('--set', dest='set_value', action='append', default=[], help='lend=25、pid.om_kp=0.1など')
        if action == 'capture':
            command.add_argument('--operator-ready', action='store_true')
    return result


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        if args.action == 'serve':
            serve(args)
            return 0
        value = client(args)
        print(json.dumps(value, ensure_ascii=False, allow_nan=False), flush=True)
        if value.get('state') in ('FAILED', 'WAIT_TIMEOUT'):
            return 1
        result = value.get('result', {})
        if 'capture' in result:
            if result['capture']['state'] != 'CAPTURED_REVIEW_REQUIRED':
                return 1
            if result.get('full_returncode') != 0 or not result.get('full', {}).get('auto_stop_confirmed') or result.get('archive_error'):
                return 2
        return 0
    except (Exception, KeyboardInterrupt) as error:
        print(json.dumps(dict(error=f'{type(error).__name__}: {error}',
            note='要求は自動再送しません。送信結果が不明な場合は同じrequest-idのstatusを確認してください'), ensure_ascii=False), file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
