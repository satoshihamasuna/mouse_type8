"""One BLE connection with telemetry enabled for exploration and shortest runs."""
import asyncio
from dataclasses import asdict, dataclass
from datetime import datetime
import json
from pathlib import Path
import queue
import sys
import time
import traceback

import mouse_ble_gateway as transport
from mouse_ble_log import CsvCapture, LOG_LABELS, control_packet


WALL_EKF_MODES = ('keep', 'legacy', 'shadow', 'control')
SHELL_CHAR_INTERVAL_S = 0.08
WALL_EKF_TIMEOUT_S = 2.0


def validate_wall_ekf_mode(mode):
    if mode not in WALL_EKF_MODES:
        raise ValueError('壁制御モードが不正です')


@dataclass(frozen=True)
class ExploreOptions:
    goal_x: int = 7
    goal_y: int = 7
    goal_size: int = 2
    accelerated: bool = False
    roundtrip: bool = True
    limit_seconds: int = 360
    wall_ekf_mode: str = 'keep'

    def command(self, max_x=31, max_y=31):
        validate_wall_ekf_mode(self.wall_ekf_mode)
        if not (0 <= self.goal_x <= max_x and 0 <= self.goal_y <= max_y and
                1 <= self.goal_size <= min(max_x + 1, max_y + 1) and
                self.goal_x + self.goal_size <= max_x + 1 and
                self.goal_y + self.goal_size <= max_y + 1 and
                (self.goal_x != 0 or self.goal_y != 0) and 1 <= self.limit_seconds <= 3600):
            raise ValueError('ゴール範囲または制限時間が不正です')
        return 'explore start {} {} {} {} {} {}'.format(
            self.goal_x, self.goal_y, self.goal_size, 'acc' if self.accelerated else 'plain',
            'roundtrip' if self.roundtrip else 'outward', self.limit_seconds)


def fields(line):
    return dict(item.split(':', 1) for item in line.split()[1:] if ':' in item)


@dataclass(frozen=True)
class ShortestOptions:
    goal_x: int = 7
    goal_y: int = 7
    goal_size: int = 2
    profile: str = 'plain500'
    maze_source: str = 'ram'
    limit_seconds: int = 120
    wall_ekf_mode: str = 'keep'

    def command(self, max_x=31, max_y=31):
        validate_wall_ekf_mode(self.wall_ekf_mode)
        ExploreOptions(self.goal_x, self.goal_y, self.goal_size,
                       limit_seconds=self.limit_seconds).command(max_x, max_y)
        if (not self.profile or not self.profile.isascii() or
                not all(c.isalnum() or c == '_' for c in self.profile) or
                self.maze_source not in ('ram', 'flash')):
            raise ValueError('走行プロファイルまたは迷路の指定が不正です')
        return 'shortest start {} {} {} {} {} {}'.format(
            self.goal_x, self.goal_y, self.goal_size, self.profile, self.maze_source, self.limit_seconds)


class ExploreSession:
    """UI and tests enqueue actions; one task owns the link and all CSV writes."""
    def __init__(self, *, name='HY0020-MOUSE', address=None, period_ms=20,
                 output_root=None, emit=None):
        control_packet(period_ms)  # Validate before connecting.
        self.name, self.address, self.period_ms = name, address, period_ms
        self.output_root = Path(output_root or Path(__file__).resolve().parents[1] / 'logs')
        self.emit = emit or (lambda kind, data: None)
        self.actions = queue.Queue()
        self.capability = None
        self.shortest_capability = None
        self.battery_ok = False
        self.state = 'disconnected'
        self.capture = None
        self.directory = None
        self.last_sample_at = None
        self._line_buffer = bytearray()
        self._events_file = self._text_file = None
        self._closed = False
        self._disconnect_requested = False
        self._run_requested_at = None
        self._stop_requested_at = None
        self._streaming = False
        self._stream_started_at = None
        self._stream_stop_pending = False
        self.wall_ekf_status = None
        self._wall_ekf_probe_failed = False
        self._wall_ekf_reply = None
        self._wall_ekf_event = None
        self._wall_ekf_waiting = False
        self._run_settings = self._run_settings_path = None
        self._post_run_pending = False

    def _save_run_settings(self):
        if self._run_settings_path is not None:
            self._run_settings_path.write_text(json.dumps(self._run_settings, ensure_ascii=False, indent=2),
                                               encoding='utf-8')

    def _finish_run(self, result):
        if self._run_settings is not None:
            self._run_settings['result'] = result
            diagnostics = self._run_settings.get('turn_diagnostics')
            if diagnostics is not None:
                summary = diagnostics.get('summary') or {}
                try:
                    stored = int(summary['stored'])
                    diagnostics['complete'] = (0 <= stored <= 64 and len(diagnostics['events']) == stored and
                                               [int(e['index']) for e in diagnostics['events']] == list(range(stored)))
                except (KeyError, TypeError, ValueError):
                    diagnostics['complete'] = False
            sample = self.capture.latest if self.capture else None
            self._run_settings['telemetry_end'] = ({'sequence': sample.sequence, 'time_ms': sample.time_ms}
                                                   if sample else None)
            self._post_run_pending = self.wall_ekf_status is not None
            self._save_run_settings()

    def action(self, name, payload=None):
        self.actions.put_nowait((name, payload))

    def _event(self, kind, data):
        if self._events_file and kind != 'telemetry':
            recorded = {k: v for k, v in data.items() if k != 'values'} if kind == 'summary' else data
            self._events_file.write(json.dumps({'at': datetime.now().astimezone().isoformat(),
                                                'event': kind, 'data': recorded}, ensure_ascii=False) + '\n')
            self._events_file.flush()
        self.emit(kind, data)

    def _set_state(self, state):
        self.state = state
        self._event('state', state)

    def _text(self, data):
        self._text_file.write(data.decode('utf-8', errors='replace'))
        self._text_file.flush()
        self._line_buffer.extend(data)
        if len(self._line_buffer) > 65536:
            del self._line_buffer[:-4096]
        while b'\n' in self._line_buffer:
            line, _, rest = self._line_buffer.partition(b'\n')
            self._line_buffer = bytearray(rest)
            # ntshell can prefix a prompt/echo; protocol tags are line-local.
            text = line.decode('ascii', errors='replace').strip()
            self._event('text', text)
            shortest = text.startswith('SHORTEST_')
            tag = 'SHORTEST' if shortest else 'EXPLORE'
            if text.startswith('WALL_EKF '):
                status = fields(text)
                if status.get('mode') in WALL_EKF_MODES[1:]:
                    self.wall_ekf_status = status
                    self._wall_ekf_reply = status
                    self._event('wall_ekf', dict(status))
                else:
                    self._wall_ekf_reply = '壁制御モードの応答が不正です: ' + text
                if self._wall_ekf_event:
                    self._wall_ekf_event.set()
            elif self._wall_ekf_waiting and (text.startswith('WALL_EKF_ERROR') or
                                             text.startswith('Unknown command') or
                                             text.startswith('Unknown sub command')):
                self._wall_ekf_reply = text
                self._wall_ekf_event.set()
            elif text.startswith('SHORTEST_WALL_STATS '):
                status = fields(text)
                if (self.state in ('starting', 'waiting_sensor', 'running', 'stopping') and
                        self._run_settings is not None and self._run_settings.get('run_type') == 'shortest'):
                    self._run_settings['wall_control_stats'] = status
                    self._save_run_settings()
                    self._event('wall_stats', status)
            elif text.startswith('EXPLORE_TURN_STATS ') or text.startswith('EXPLORE_TURN '):
                status = fields(text)
                kind = 'turn_stats' if text.startswith('EXPLORE_TURN_STATS ') else 'turn_decision'
                if self._run_settings is not None and self._run_settings.get('run_type') == 'explore':
                    diagnostics = self._run_settings.setdefault('turn_diagnostics', {'summary': None, 'events': []})
                    if kind == 'turn_stats':
                        diagnostics['summary'] = status
                    elif len(diagnostics['events']) < 64:
                        diagnostics['events'].append(status)
                    self._save_run_settings()
                self._event(kind, status)
            elif text.startswith('EXPLORE_CAP '):
                capability = fields(text)
                if capability.get('protocol') == '1':
                    self.capability = capability
                    self._event('capability', capability)
            elif text.startswith('SHORTEST_CAP '):
                capability = fields(text)
                if capability.get('protocol') == '1':
                    self.shortest_capability = capability
                    self._event('shortest_capability', capability)
            elif text.startswith('BATTERY '):
                status = fields(text)
                self.battery_ok = status.get('status') == 'OK'
                self._event('battery', status)
            elif text.startswith(tag + '_WAIT_SENSOR '):
                self._run_requested_at = None
                self._set_state('waiting_sensor')
            elif text.startswith(tag + '_PHASE '):
                self._run_requested_at = None
                self._set_state('running')
                self._event('phase', fields(text).get('name', ''))
            elif text.startswith(tag + '_END '):
                self._run_requested_at = self._stop_requested_at = None
                self._finish_stream()
                if not shortest and self.shortest_capability is not None:
                    self.shortest_capability['ram_maze'] = '1' if fields(text).get('result') == 'complete' else '0'
                    self._event('shortest_capability', self.shortest_capability)
                self._event('result', fields(text))
                self._finish_run(fields(text))
                self._set_state('idle')
                if self._disconnect_requested:
                    self._closed = True
            elif text.startswith(tag + '_ERROR '):
                self._run_requested_at = self._stop_requested_at = None
                self._finish_stream()
                self._event('run_error', text)
                self._finish_run({'error': text})
                self._set_state('idle')
                if self._disconnect_requested:
                    self._closed = True

    def _finish_stream(self):
        # Stop renewals immediately, even if a command write is still awaiting its ACK.
        self._streaming = False
        self._stream_started_at = None
        self._stream_stop_pending = True

    def ready(self, options=None):
        capability = self.shortest_capability if isinstance(options, ShortestOptions) else self.capability
        if not (self.state == 'idle' and capability is not None and self.battery_ok
                and not self._stream_stop_pending and not self._post_run_pending and not self._wall_ekf_probe_failed):
            return False
        if isinstance(options, ShortestOptions):
            return (options.profile in capability.get('profiles', '').split(',') and
                    (options.maze_source == 'flash' or capability.get('ram_maze') == '1'))
        return True

    async def _command(self, client, lock, command, *, enter=True):
        if '\r' in command or '\n' in command:
            raise ValueError('one command per request')
        self._event('command', command)
        for byte in command.encode('ascii') + (b'\r' if enter else b''):
            await transport.write_bytes(client, bytes([byte]), lock)
            await asyncio.sleep(SHELL_CHAR_INTERVAL_S)  # Same pacing as the existing mouse shell GUI.

    async def _read_wall_ekf(self, client, lock, mode='show'):
        if mode != 'show':
            validate_wall_ekf_mode(mode)
            if mode == 'keep':
                raise ValueError('keep is not a shell command')
        if self._wall_ekf_event is None:
            self._wall_ekf_event = asyncio.Event()
        self._wall_ekf_event.clear()
        self._wall_ekf_reply = None
        self._wall_ekf_waiting = True
        try:
            await self._command(client, lock, 'wall_ekf ' + mode)
            await asyncio.wait_for(self._wall_ekf_event.wait(), timeout=WALL_EKF_TIMEOUT_S)
            if not isinstance(self._wall_ekf_reply, dict):
                raise ValueError('壁制御モードを確認できません: ' + str(self._wall_ekf_reply))
            return dict(self._wall_ekf_reply)
        except asyncio.TimeoutError as error:
            raise ValueError('壁制御モードの応答がありません') from error
        finally:
            self._wall_ekf_waiting = False

    async def _prepare_wall_ekf(self, client, lock, requested):
        validate_wall_ekf_mode(requested)
        if requested == 'keep' and self.wall_ekf_status is None:
            return None  # Compatibility with firmware predating wall_ekf.
        if requested != 'keep':
            applied = await self._read_wall_ekf(client, lock, requested)
            if applied['mode'] != requested:
                raise ValueError('壁制御モードが選択値へ切り替わりませんでした')
        # Use a fresh readback, not the connection-time status or only the write ACK.
        confirmed = await self._read_wall_ekf(client, lock)
        if requested != 'keep' and confirmed['mode'] != requested:
            raise ValueError('壁制御モードの確認値が選択値と一致しません')
        return confirmed

    async def _capture_wall_ekf_after(self, client, lock):
        try:
            after = await self._read_wall_ekf(client, lock)
            self._run_settings['wall_ekf']['after'] = after
        except ValueError as error:
            self._run_settings['wall_ekf']['after_error'] = str(error)
            self._event('wall_ekf_unavailable', str(error))
        finally:
            self._post_run_pending = False
            self._save_run_settings()

    def _cancel_queued_start(self):
        """Honor a stop/disconnect received while preparing a non-motion command."""
        pending, cancelled = [], False
        while True:
            try:
                name, payload = self.actions.get_nowait()
            except queue.Empty:
                break
            if name in ('stop', 'disconnect'):
                cancelled = True
                if name == 'disconnect':
                    self._disconnect_requested = self._closed = True
            else:
                pending.append((name, payload))
        if not cancelled:
            for action in pending:
                self.actions.put_nowait(action)
        return cancelled

    async def run(self):
        from bleak import BleakClient, BleakScanner
        self._set_state('connecting')
        device = await (BleakScanner.find_device_by_address(self.address, timeout=15) if self.address
                        else BleakScanner.find_device_by_name(self.name, timeout=15))
        if device is None:
            self._set_state('disconnected')
            raise RuntimeError('HY0020-MOUSEが見つかりません')
        self.output_root.mkdir(parents=True, exist_ok=True)
        self.directory = self.output_root / datetime.now().strftime('%Y%m%d_%H%M%S_%f_ble_explore')
        self.directory.mkdir()
        disconnected = asyncio.Event()
        incoming = asyncio.Queue(maxsize=1024)
        overflow = False
        link_failure = None

        def received(characteristic, data):
            nonlocal overflow
            try:
                incoming.put_nowait(bytes(data))
            except asyncio.QueueFull:
                overflow = True
                disconnected.set()

        def link_disconnected(client):
            self._event('link_disconnected', {'state': self.state, 'streaming': self._streaming,
                                               'requested': self._disconnect_requested or self._closed})
            disconnected.set()

        options = {'winrt': {'use_cached_services': False}} if sys.platform == 'win32' else {}
        with (self.directory / 'telemetry.csv').open('x', newline='', encoding='utf-8') as csv_file, \
             (self.directory / 'shell.txt').open('x', encoding='utf-8') as text_file, \
             (self.directory / 'events.jsonl').open('x', encoding='utf-8') as events_file:
            self._text_file, self._events_file = text_file, events_file
            self.capture = CsvCapture(csv_file)
            self._event('output', str(self.directory))
            try:
                async with BleakClient(device, disconnected_callback=link_disconnected, **options) as client:
                    lock = asyncio.Lock()
                    baseline = transport.decode_status(await client.read_gatt_char(transport.STATUS))
                    self._event('bridge_status', baseline)
                    await client.start_notify(transport.NUS_TX, received)
                    self._set_state('checking')
                    first_sample = asyncio.Event()
                    next_ui = 0

                    def publish():
                        nonlocal next_ui
                        if time.monotonic() >= next_ui:
                            self._event('telemetry', self.snapshot())
                            next_ui = time.monotonic() + 0.1

                    async def keep_stream_alive():
                        nonlocal link_failure
                        try:
                            while True:
                                await asyncio.sleep(0.5)
                                if self._streaming:
                                    await transport.write_bytes(client, control_packet(self.period_ms), lock)
                        except asyncio.CancelledError:
                            raise
                        except Exception as error:
                            link_failure = error
                            self._event('task_error', {'task': 'heartbeat', 'type': type(error).__name__,
                                                       'detail': str(error), 'traceback': traceback.format_exc()})
                            disconnected.set()

                    heartbeat = asyncio.create_task(keep_stream_alive())

                    async def receive_stream():
                        nonlocal link_failure
                        try:
                            while True:
                                chunk = await incoming.get()
                                if self.capture.feed(chunk, self._text):
                                    self.last_sample_at = time.monotonic()
                                    if self._streaming:
                                        first_sample.set()
                                publish()
                        except asyncio.CancelledError:
                            raise
                        except Exception as error:
                            link_failure = error
                            self._event('task_error', {'task': 'receiver', 'type': type(error).__name__,
                                                       'detail': str(error), 'traceback': traceback.format_exc()})
                            disconnected.set()
                            try:
                                await asyncio.wait_for(transport.write_bytes(
                                    client, control_packet(self.period_ms, action=2), lock), 1)
                            except Exception:
                                pass

                    receiver = asyncio.create_task(receive_stream())
                    next_status = time.monotonic()
                    uncommitted_command = False
                    try:
                        # End a previous client's lease; an idle connection does not request logs.
                        await transport.write_bytes(client, control_packet(self.period_ms, action=2), lock)
                        await transport.write_bytes(client, control_packet(self.period_ms, stop=True), lock)
                        # Clear a partial command left by a previous connection, then enter shell.
                        await self._command(client, lock, '')
                        await self._command(client, lock, 'shell')
                        await self._command(client, lock, 'explore show')
                        await self._command(client, lock, 'shortest show')
                        await self._command(client, lock, 'info battery')
                        try:
                            await self._read_wall_ekf(client, lock)
                        except ValueError as error:
                            # A silent/invalid probe is not evidence of old firmware.
                            self._wall_ekf_probe_failed = not any(tag in str(error) for tag in
                                                                  ('Unknown command', 'Unknown sub command'))
                            self._event('wall_ekf_unavailable', str(error))
                        self._set_state('idle')
                        while not self._closed:
                            now = time.monotonic()
                            if disconnected.is_set():
                                raise link_failure or ConnectionError('受信キューが満杯です' if overflow else 'BLE接続が切れました')
                            if now >= next_status:
                                current = transport.decode_status(await client.read_gatt_char(transport.STATUS))
                                self._event('bridge_status', current)
                                transport.check_status(baseline, current)
                                next_status = time.monotonic() + 1
                            await asyncio.sleep(0.02)
                            if disconnected.is_set():
                                raise link_failure or ConnectionError('BLE接続が切れました')
                            if self._stream_stop_pending:
                                await transport.write_bytes(client, control_packet(self.period_ms, stop=True), lock)
                                self._stream_stop_pending = False
                                self._event('stream', {'enabled': False})
                            if self._post_run_pending:
                                await self._capture_wall_ekf_after(client, lock)
                            publish()
                            if self._streaming and time.monotonic() - (self.last_sample_at or self._stream_started_at) > 5:
                                raise TimeoutError('5秒間ログを受信できません。対応STM32ファームウェアを確認してください')
                            if self._run_requested_at and now - self._run_requested_at > 5:
                                await transport.write_bytes(client, control_packet(self.period_ms, action=2), lock)
                                self._run_requested_at = None
                                self._set_state('stopping')
                                self._stop_requested_at = now
                                self._event('run_error', '走行開始の応答がありません。停止を要求しました')
                            if self._stop_requested_at and now - self._stop_requested_at > 4:
                                raise TimeoutError('走行停止の応答がありません。接続を閉じます')
                            try:
                                action, payload = self.actions.get_nowait()
                            except queue.Empty:
                                continue
                            if action == 'start':
                                if not isinstance(payload, (ExploreOptions, ShortestOptions)):
                                    raise ValueError('invalid run options')
                                if not self.ready(payload):
                                    self._event('run_error', '走行コマンド対応・迷路・プロファイル・電圧の確認と前回のログ配信停止が必要です')
                                    continue
                                capability = self.shortest_capability if isinstance(payload, ShortestOptions) else self.capability
                                command = payload.command(int(capability['max_x']), int(capability['max_y']))
                                self._set_state('starting')
                                try:
                                    wall_before = await self._prepare_wall_ekf(client, lock, payload.wall_ekf_mode)
                                except ValueError as error:
                                    self._event('run_error', str(error) + '。走行は開始しません')
                                    self._set_state('idle')
                                    continue
                                if self._cancel_queued_start():
                                    self._event('run_error', '走行開始前に中断しました')
                                    self._set_state('idle')
                                    continue
                                settings = {'options': asdict(payload), 'firmware': dict(capability),
                                            'run_type': 'shortest' if isinstance(payload, ShortestOptions) else 'explore',
                                            'period_ms': self.period_ms, 'wait_sensor': True,
                                            'wall_ekf': {'requested_mode': payload.wall_ekf_mode,
                                                         'before': wall_before, 'after': None,
                                                         'confirmed_mode': wall_before['mode'] if wall_before else None}}
                                if not isinstance(payload, ShortestOptions) and capability.get('turn_diag') == '1':
                                    settings['turn_diagnostics'] = {'summary': None, 'events': [], 'complete': False}
                                if isinstance(payload, ShortestOptions) and capability.get('diagonal_ekf') == '1':
                                    settings['wall_control_stats'] = None
                                self._run_settings_path = self.directory / ('run_' + datetime.now().strftime('%H%M%S_%f') + '.json')
                                with self._run_settings_path.open('x', encoding='utf-8') as f:
                                    json.dump(settings, f, ensure_ascii=False, indent=2)
                                self._run_settings = settings
                                # Type the command while logs are stopped. CR is the execution boundary.
                                uncommitted_command = True
                                await self._command(client, lock, command, enter=False)
                                self.last_sample_at = None
                                first_sample.clear()
                                self._stream_started_at = time.monotonic()
                                self._streaming = True
                                await transport.write_bytes(client, control_packet(self.period_ms), lock)
                                self._event('stream', {'enabled': True})
                                # No motion command is sent until the log path actually works.
                                try:
                                    await asyncio.wait_for(first_sample.wait(), timeout=5)
                                except asyncio.TimeoutError as error:
                                    raise TimeoutError('ログを受信できないため走行を開始しません。対応STM32ファームウェアを確認してください') from error
                                if self._cancel_queued_start():
                                    await transport.write_bytes(client, b'\x03', lock)
                                    uncommitted_command = False
                                    self._finish_stream()
                                    self._finish_run({'result': 'cancelled_before_start'})
                                    self._event('run_error', '走行開始前に中断しました')
                                    self._set_state('idle')
                                    continue
                                settings['telemetry_start'] = {'sequence': self.capture.latest.sequence,
                                                               'time_ms': self.capture.latest.time_ms}
                                self._save_run_settings()
                                await transport.write_bytes(client, control_packet(self.period_ms, action=3), lock)
                                # Once CR is attempted, its delivery may be uncertain: never replay it.
                                uncommitted_command = False
                                self._event('command_commit', command)
                                await transport.write_bytes(client, b'\r', lock)
                                self._run_requested_at = time.monotonic() if self.state == 'starting' else None
                            elif action == 'stop':
                                if self.state in ('starting', 'waiting_sensor', 'running', 'stopping'):
                                    self._set_state('stopping')
                                    self._stop_requested_at = now
                                    await transport.write_bytes(client, control_packet(self.period_ms, action=2), lock)
                            elif action == 'disconnect':
                                self._disconnect_requested = True
                                if self.state in ('starting', 'waiting_sensor', 'running', 'stopping'):
                                    self._set_state('stopping')
                                    self._stop_requested_at = now
                                    await transport.write_bytes(client, control_packet(self.period_ms, action=2), lock)
                                else:
                                    self._closed = True
                    finally:
                        self._streaming = False
                        heartbeat.cancel()
                        await asyncio.gather(heartbeat, return_exceptions=True)
                        if not disconnected.is_set():
                            try:
                                await asyncio.wait_for(transport.write_bytes(client, control_packet(self.period_ms, action=2), lock), 1)
                                await asyncio.wait_for(transport.write_bytes(client, control_packet(self.period_ms, stop=True), lock), 1)
                                if uncommitted_command:
                                    # NT-Shell Ctrl-C cancels the line without executing its partial text.
                                    await asyncio.wait_for(transport.write_bytes(client, b'\x03', lock), 1)
                                if self._post_run_pending:
                                    await self._capture_wall_ekf_after(client, lock)
                            except Exception:
                                pass
                        receiver.cancel()
                        await asyncio.gather(receiver, return_exceptions=True)
                        while not incoming.empty():
                            self.capture.feed(incoming.get_nowait(), self._text)
                        self._event('summary', self.snapshot())
            except Exception as error:
                self._event('error_detail', {'type': type(error).__name__, 'detail': str(error),
                                            'traceback': traceback.format_exc()})
                self._event('error', str(link_failure or error))
                if link_failure is not None and link_failure is not error:
                    raise link_failure from error
                raise
            finally:
                self._set_state('disconnected')
                self._text_file = self._events_file = None

    def snapshot(self):
        sample = self.capture.latest if self.capture else None
        return {'rows': self.capture.rows if self.capture else 0,
                'lost': self.capture.lost if self.capture else 0,
                'crc_errors': self.capture.decoder.crc_errors if self.capture else 0,
                'ready': self.ready(), 'state': self.state, 'streaming': self._streaming,
                'wall_ekf_supported': self.wall_ekf_status is not None,
                'wall_ekf_probe_failed': self._wall_ekf_probe_failed,
                'wall_ekf': dict(self.wall_ekf_status) if self.wall_ekf_status else None,
                'shortest_supported': self.shortest_capability is not None,
                'shortest_ready': (self.state == 'idle' and self.shortest_capability is not None and
                                   self.battery_ok and not self._stream_stop_pending and not self._post_run_pending
                                   and not self._wall_ekf_probe_failed),
                'ram_maze': (self.shortest_capability or {}).get('ram_maze') == '1',
                'time_ms': sample.time_ms if sample else None,
                'values': dict(zip(LOG_LABELS, sample.values)) if sample else {}}
