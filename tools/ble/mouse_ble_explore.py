"""One BLE connection with telemetry enabled only for an exploration."""
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


@dataclass(frozen=True)
class ExploreOptions:
    goal_x: int = 7
    goal_y: int = 7
    goal_size: int = 2
    accelerated: bool = False
    roundtrip: bool = True
    limit_seconds: int = 360

    def command(self, max_x=31, max_y=31):
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
            if text.startswith('EXPLORE_CAP '):
                capability = fields(text)
                if capability.get('protocol') == '1':
                    self.capability = capability
                    self._event('capability', capability)
            elif text.startswith('BATTERY '):
                status = fields(text)
                self.battery_ok = status.get('status') == 'OK'
                self._event('battery', status)
            elif text.startswith('EXPLORE_WAIT_SENSOR '):
                self._run_requested_at = None
                self._set_state('waiting_sensor')
            elif text.startswith('EXPLORE_PHASE '):
                self._run_requested_at = None
                self._set_state('running')
                self._event('phase', fields(text).get('name', ''))
            elif text.startswith('EXPLORE_END '):
                self._run_requested_at = self._stop_requested_at = None
                self._finish_stream()
                self._event('result', fields(text))
                self._set_state('idle')
                if self._disconnect_requested:
                    self._closed = True
            elif text.startswith('EXPLORE_ERROR '):
                self._run_requested_at = self._stop_requested_at = None
                self._finish_stream()
                self._event('run_error', text)
                self._set_state('idle')
                if self._disconnect_requested:
                    self._closed = True

    def _finish_stream(self):
        # Stop renewals immediately, even if a command write is still awaiting its ACK.
        self._streaming = False
        self._stream_started_at = None
        self._stream_stop_pending = True

    def ready(self):
        return (self.state == 'idle' and self.capability is not None and self.battery_ok
                and not self._stream_stop_pending)

    async def _command(self, client, lock, command, *, enter=True):
        if '\r' in command or '\n' in command:
            raise ValueError('one command per request')
        self._event('command', command)
        for byte in command.encode('ascii') + (b'\r' if enter else b''):
            await transport.write_bytes(client, bytes([byte]), lock)
            await asyncio.sleep(0.08)  # Same pacing as the existing mouse shell GUI.

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
                        await self._command(client, lock, 'info battery')
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
                            publish()
                            if self._streaming and time.monotonic() - (self.last_sample_at or self._stream_started_at) > 5:
                                raise TimeoutError('5秒間ログを受信できません。対応STM32ファームウェアを確認してください')
                            if self._run_requested_at and now - self._run_requested_at > 5:
                                await transport.write_bytes(client, control_packet(self.period_ms, action=2), lock)
                                self._run_requested_at = None
                                self._set_state('stopping')
                                self._stop_requested_at = now
                                self._event('run_error', '探索開始の応答がありません。停止を要求しました')
                            if self._stop_requested_at and now - self._stop_requested_at > 4:
                                raise TimeoutError('探索停止の応答がありません。接続を閉じます')
                            try:
                                action, payload = self.actions.get_nowait()
                            except queue.Empty:
                                continue
                            if action == 'start':
                                if not self.ready():
                                    self._event('run_error', '探索対応・電圧の確認と前回のログ配信停止を待ってから開始してください')
                                    continue
                                if not isinstance(payload, ExploreOptions):
                                    raise ValueError('invalid exploration options')
                                command = payload.command(int(self.capability['max_x']), int(self.capability['max_y']))
                                settings = {'options': asdict(payload), 'firmware': self.capability,
                                            'period_ms': self.period_ms, 'wait_sensor': True}
                                with (self.directory / ('run_' + datetime.now().strftime('%H%M%S_%f') + '.json')).open('x', encoding='utf-8') as f:
                                    json.dump(settings, f, ensure_ascii=False, indent=2)
                                self._set_state('starting')
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
                                    raise TimeoutError('ログを受信できないため探索を開始しません。対応STM32ファームウェアを確認してください') from error
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
                'time_ms': sample.time_ms if sample else None,
                'values': dict(zip(LOG_LABELS, sample.values)) if sample else {}}
