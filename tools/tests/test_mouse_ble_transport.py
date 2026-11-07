"""Protocol tests use localhost and fake BLE peers; they never contact a robot."""
import asyncio
import binascii
import contextlib
import csv
import io
import json
from pathlib import Path
import shutil
import socket
import struct
import subprocess
import sys
import threading
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
for directory in (ROOT / '.tools' / 'ble_python', ROOT / '.tools' / 'python', ROOT / 'tools' / 'ble'):
    sys.path.insert(0, str(directory))
sys.path.insert(0, str(ROOT / '.tools' / ('ble_python{}{}'.format(*sys.version_info[:2]))))

if sys.version_info < (3, 10):
    raise unittest.SkipTest('BLE transport tests require Python 3.10+; see tools/ble/README.md')
try:
    import serial
    from serial.tools import list_ports
    from bleak.exc import BleakCharacteristicNotFoundError, BleakGATTProtocolError
except ImportError as error:
    raise unittest.SkipTest('Install BLE test dependencies; see tools/ble/README.md: {}'.format(error))
import mouse_ble_gateway as gateway
import mouse_gui_ble as launcher
import mouse_ble_shell as shell
import mouse_ble_log as log


class GuiCaptureTests(unittest.TestCase):
    def test_live_frames_are_saved_and_other_binary_bytes_are_unchanged(self):
        payload = struct.pack('<4sIIH55e', log.MAGIC, 7, 100, 5, *range(55))
        live_frame = payload + struct.pack('<H', binascii.crc_hqx(payload, 0xffff))
        saved_frame = struct.pack('<HH51H', 0xA55A, 7, *range(51))
        other = b'DEBUG_WAIT_SENSOR\r\nBINARY\r\n' + saved_frame + b'\r\n> '
        wire = other[:10] + live_frame + other[10:]
        forwarded = []
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'trial.csv'
            recorder = gateway.GuiCapture(output, 5)
            try:
                for offset in range(0, len(wire), 7):
                    recorder.feed(wire[offset:offset + 7], forwarded.append)
                self.assertEqual(recorder.report()['rows'], 1)
            finally:
                recorder.close()
            self.assertEqual(b''.join(forwarded), other)
            self.assertEqual(output.with_suffix('.raw.bin').read_bytes(), wire)
            self.assertEqual(output.with_suffix('.shell.bin').read_bytes(), other)
            status = json.loads(output.with_suffix('.status.json').read_text())
            self.assertEqual((status['rows'], status['lost'], status['crc_errors']), (1, 0, 0))
            with output.open(newline='') as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(rows[0]['sequence'], '7')
            self.assertEqual(rows[0]['time_ms'], '100')

    def test_existing_csv_is_not_overwritten_and_invalid_period_creates_no_file(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'trial.csv'
            with self.assertRaises(ValueError):
                gateway.GuiCapture(output, 1)
            self.assertFalse(output.exists())
            output.write_text('keep this measurement', encoding='utf-8')
            with self.assertRaises(FileExistsError):
                gateway.GuiCapture(output, 5)
            self.assertEqual(output.read_text(encoding='utf-8'), 'keep this measurement')


class SerialCompatibilityTests(unittest.TestCase):
    def setUp(self):
        self.original_serial = serial.Serial
        self.original_ports = list_ports.comports

    def tearDown(self):
        serial.Serial = self.original_serial
        list_ports.comports = self.original_ports

    def test_binary_maze_and_log_are_unchanged(self):
        listener = socket.socket()
        listener.bind(('127.0.0.1', 0))
        listener.listen(1)
        listener.settimeout(3)
        payload = bytes(range(256)) * 4
        log_frame = struct.pack('<HH' + 'H' * 51, 0xA55A, 7, *range(51))
        received, errors = [], []

        def peer():
            try:
                with listener.accept()[0] as conn:
                    conn.settimeout(3)
                    command = bytearray()
                    while not command.endswith(b'\r'):
                        chunk = conn.recv(64)
                        if not chunk:
                            raise ConnectionError('Client closed before sending the command')
                        command.extend(chunk)
                    received.append(bytes(command))
                    wire = b'MAZE_BIN_START\r\n' + payload + b'BINARY\r\n' + log_frame
                    for offset in range(0, len(wire), 7):
                        conn.sendall(wire[offset:offset + 7])
            except Exception as error:
                errors.append(error)

        thread = threading.Thread(target=peer, daemon=True)
        thread.start()
        try:
            launcher.install_ble_port(listener.getsockname()[1])
            with serial.Serial('BLE', 115200, timeout=3) as port:
                port.write(b'info ver\r')
                port.flush()
                self.assertEqual(port.readline(), b'MAZE_BIN_START\r\n')
                self.assertEqual(port.read(1024), payload)
                self.assertEqual(port.readline(), b'BINARY\r\n')
                self.assertEqual(port.read(106), log_frame)
            thread.join(4)
            self.assertFalse(thread.is_alive())
            self.assertFalse(errors, errors)
            self.assertEqual(received, [b'info ver\r'])
        finally:
            listener.close()

    def test_real_com_port_is_delegated(self):
        with patch.object(serial, 'Serial', return_value='real-port') as original:
            launcher.install_ble_port(8765)
            self.assertEqual(serial.Serial('COM_TEST', 115200, timeout=0.1), 'real-port')
            original.assert_called_once_with('COM_TEST', 115200, timeout=0.1)
        self.assertIn('BLE', [port.device for port in list_ports.comports()])


class GuiLauncherTests(unittest.TestCase):
    def test_standalone_launcher_loads_local_pyserial(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            script = root / 'tools' / 'ble' / 'mouse_gui_ble.py'
            script.parent.mkdir(parents=True)
            shutil.copyfile(ROOT / 'tools' / 'ble' / 'mouse_gui_ble.py', script)
            dependencies = root / '.tools' / 'ble_python'
            dependencies.mkdir(parents=True)
            shutil.copytree(Path(serial.__file__).parent, dependencies / 'serial')
            code = (
                'import runpy, sys\n'
                "launcher = runpy.run_path(sys.argv[1], run_name='gui_test')\n"
                "launcher['install_ble_port'](8765)\n"
                'from serial.tools import list_ports\n'
                "assert any(port.device == 'BLE' for port in list_ports.comports())\n"
            )
            result = subprocess.run([sys.executable, '-I', '-S', '-B', '-c', code, str(script)],
                capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_defaults_to_this_repository(self):
        with patch.object(sys, 'argv', ['mouse_gui_ble.py', '--gui', 'debug']), \
             patch.object(sys, 'path', list(sys.path)), \
             patch.object(launcher, 'install_ble_port') as install, \
             patch.object(launcher.runpy, 'run_path') as run:
            launcher.main()
        install.assert_called_once_with(8765)
        run.assert_called_once_with(str(ROOT / 'tools' / 'myshell_debug.py'), run_name='__main__')

    def test_explicit_repository_and_port(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory).resolve()
            (root / 'tools').mkdir()
            script = root / 'tools' / 'myshell_gui.py'
            script.write_text('# Test GUI placeholder\n', encoding='utf-8')
            with patch.object(sys, 'argv', ['mouse_gui_ble.py', '--mouse-repo', str(root), '--port', '9876']), \
                 patch.object(sys, 'path', list(sys.path)), \
                 patch.object(launcher, 'install_ble_port') as install, \
                 patch.object(launcher.runpy, 'run_path') as run:
                launcher.main()
            install.assert_called_once_with(9876)
            run.assert_called_once_with(str(script), run_name='__main__')


class DiagnosticsTests(unittest.IsolatedAsyncioTestCase):
    @staticmethod
    def expected_diagnostics():
        return {
            'version': 1, 'last_stop_reason': 0x80000001, 'stop_reason_or': 0x8000003F,
            'overrun_count': 3, 'parity_count': 4, 'framing_count': 5, 'break_count': 6,
            'collision_count': 7, 'noise_count': 8, 'other_reason_count': 9,
            'rx_disabled_count': 10, 'rx_restart_attempts': 11, 'rx_restart_failures': 12,
            'rx_buffer_alloc_failures': 13, 'rx_buffer_rsp_failures': 14,
            'rx_buf_request_events': 15, 'rx_buffer_size': 252, 'rx_buffer_count': 4,
            'rx_timeout_us': 1000, 'rx_mode_flags': 1,
        }

    def test_diagnostics_decodes_all_fields_and_preserves_unknown_reason_bits(self):
        expected = self.expected_diagnostics()
        payload = struct.pack('<20I', *expected.values())
        self.assertEqual(gateway.decode_diagnostics(payload), expected)
        self.assertEqual(gateway.DIAGNOSTICS_COUNTER_FIELDS, (
            'overrun_count', 'parity_count', 'framing_count', 'break_count', 'collision_count',
            'noise_count', 'other_reason_count', 'rx_disabled_count', 'rx_restart_attempts',
            'rx_restart_failures', 'rx_buffer_alloc_failures', 'rx_buffer_rsp_failures',
            'rx_buf_request_events',
        ))

    def test_diagnostics_rejects_wrong_length_and_unknown_version(self):
        for payload in (bytes(79), bytes(81), struct.pack('<20I', 2, *range(19))):
            with self.subTest(length=len(payload), version=payload[0]):
                with self.assertRaises(ValueError):
                    gateway.decode_diagnostics(payload)

    async def test_diagnostics_read_uses_the_shared_request_lock(self):
        lock = asyncio.Lock()
        calls = []
        expected = self.expected_diagnostics()

        class Client:
            async def read_gatt_char(self, uuid):
                calls.append(uuid)
                if not lock.locked():
                    raise AssertionError('Diagnostic read did not acquire the transport lock')
                return struct.pack('<20I', *expected.values())

        await lock.acquire()
        task = asyncio.create_task(gateway.read_diagnostics(Client(), lock))
        try:
            await asyncio.sleep(0)
            self.assertEqual(calls, [])
            lock.release()
            self.assertEqual(await asyncio.wait_for(task, 1), expected)
            self.assertEqual(calls, [gateway.DIAGNOSTICS])
            self.assertFalse(lock.locked())
        finally:
            if lock.locked():
                lock.release()
            if not task.done():
                task.cancel()
                await asyncio.gather(task, return_exceptions=True)

    async def test_diagnostics_is_optional_only_for_explicit_attribute_absence(self):
        for error in (BleakCharacteristicNotFoundError(gateway.DIAGNOSTICS),
                      BleakGATTProtocolError(0x0A)):
            with self.subTest(error=type(error).__name__):
                class Client:
                    async def read_gatt_char(self, uuid):
                        raise error

                lock = asyncio.Lock()
                self.assertIsNone(await gateway.read_diagnostics(Client(), lock))
                self.assertFalse(lock.locked())

    async def test_diagnostics_read_failures_are_not_reported_as_old_firmware(self):
        for error in (BleakGATTProtocolError(0x0E), TimeoutError('read timed out'),
                      ConnectionError('peer disconnected'), ValueError('invalid backend result')):
            with self.subTest(error=type(error).__name__):
                class Client:
                    async def read_gatt_char(self, uuid):
                        raise error

                lock = asyncio.Lock()
                with self.assertRaises(type(error)) as raised:
                    await gateway.read_diagnostics(Client(), lock)
                self.assertIs(raised.exception, error)
                self.assertFalse(lock.locked())
        with self.assertRaises(AttributeError):
            await gateway.read_diagnostics(object(), asyncio.Lock())

    async def test_invalid_diagnostics_read_payload_is_not_attribute_absence(self):
        class Client:
            async def read_gatt_char(self, uuid):
                return bytes(16)

        with self.assertRaises(ValueError):
            await gateway.read_diagnostics(Client(), asyncio.Lock())


class LinkDiagnosticsTests(unittest.IsolatedAsyncioTestCase):
    @staticmethod
    def wire():
        return struct.pack('<24I', 1, 247, 15000, 0, 4000, 2, 1, 251, 1060, 120,
                           1700, 500, 15000, 30, 7000, 2, 3, 4, 7, 10, 12, 251,
                           16384, 0x80000001)

    def test_link_codec_preserves_all_negotiated_and_relay_fields(self):
        self.assertEqual(gateway.decode_link_diagnostics(self.wire()), {
            'version': 1, 'mtu': 247, 'interval_us': 15000, 'latency': 0,
            'supervision_timeout_ms': 4000, 'tx_phy': 2, 'rx_phy': 1,
            'tx_max_len': 251, 'tx_max_time_us': 1060, 'rx_max_len': 120,
            'rx_max_time_us': 1700, 'ring_bytes': 500, 'ring_highwater': 15000,
            'notify_calls': 30, 'notify_bytes': 7000, 'notify_failures': 2,
            'notify_call_max_ms': 3, 'notify_retry_events': 4, 'batch_flush_ms': 7,
            'sdc_tx_packet_count': 10, 'acl_tx_count': 12, 'acl_tx_size': 251,
            'ring_capacity': 16384, 'flags': 0x80000001,
        })

    def test_link_codec_rejects_wrong_length_and_unknown_version(self):
        for payload in (bytes(95), bytes(97), bytes(96), struct.pack('<24I', 2, *range(23))):
            with self.subTest(length=len(payload)):
                with self.assertRaises(ValueError):
                    gateway.decode_link_diagnostics(payload)

    async def test_link_read_uses_shared_request_lock_and_exact_uuid(self):
        lock = asyncio.Lock()
        calls = []
        payload = self.wire()
        class Client:
            async def read_gatt_char(self, uuid):
                calls.append(uuid)
                if not lock.locked():
                    raise AssertionError('Link read did not acquire the transport lock')
                return payload
        await lock.acquire()
        task = asyncio.create_task(gateway.read_link_diagnostics(Client(), lock))
        try:
            await asyncio.sleep(0)
            self.assertEqual(calls, [])
            lock.release()
            self.assertEqual(await asyncio.wait_for(task, 1), gateway.decode_link_diagnostics(payload))
            self.assertEqual(calls, [gateway.LINK_DIAGNOSTICS])
            self.assertFalse(lock.locked())
        finally:
            if lock.locked():
                lock.release()
            if not task.done():
                task.cancel()
                await asyncio.gather(task, return_exceptions=True)

    async def test_link_absence_is_the_only_unsupported_result(self):
        for error in (BleakCharacteristicNotFoundError(gateway.LINK_DIAGNOSTICS),
                      BleakGATTProtocolError(0x0A)):
            with self.subTest(error=type(error).__name__):
                class Client:
                    async def read_gatt_char(self, uuid):
                        raise error
                self.assertIsNone(await gateway.read_link_diagnostics(Client(), asyncio.Lock()))

    async def test_link_timeout_disconnect_and_other_att_errors_propagate(self):
        for error in (TimeoutError('link timeout'), ConnectionError('disconnected'),
                      BleakGATTProtocolError(0x0E)):
            with self.subTest(error=type(error).__name__):
                class Client:
                    async def read_gatt_char(self, uuid):
                        raise error
                lock = asyncio.Lock()
                with self.assertRaises(type(error)) as caught:
                    await gateway.read_link_diagnostics(Client(), lock)
                self.assertIs(caught.exception, error)
                self.assertFalse(lock.locked())

    async def test_link_invalid_payload_is_not_old_firmware(self):
        class Client:
            async def read_gatt_char(self, uuid):
                return bytes(80)
        with self.assertRaises(ValueError):
            await gateway.read_link_diagnostics(Client(), asyncio.Lock())


class GattTransportTests(unittest.IsolatedAsyncioTestCase):
    async def test_live_capture_starts_renews_flushes_and_stops_without_shell_commands(self):
        writes = []

        class Scanner:
            @staticmethod
            async def find_device_by_name(*args, **kwargs):
                return SimpleNamespace(address='TEST')

        class Client:
            async def __aenter__(self):
                return self

            async def __aexit__(self, *args):
                pass

            def __init__(self, *args, **kwargs):
                self.sequence = 0

            async def read_gatt_char(self, uuid):
                return bytes(16)

            async def start_notify(self, uuid, callback):
                self.notify = callback

            async def write_gatt_char(self, uuid, data, response):
                writes.append(bytes(data))
                if data[3] == 1:
                    payload = struct.pack('<4sIIH51e', b'L8S1', self.sequence,
                                          100 + self.sequence * 20, 20, *range(51))
                    import binascii
                    payload += struct.pack('<H', binascii.crc_hqx(payload, 0xffff))
                    self.notify(None, payload[:17])
                    self.notify(None, payload[17:])
                    self.sequence += 1

        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'live.csv'
            args = SimpleNamespace(name='TEST', address=None, period_ms=20, duration=0.65, output=output)
            with patch('bleak.BleakClient', Client), patch('bleak.BleakScanner', Scanner), \
                 contextlib.redirect_stdout(io.StringIO()):
                capture = await log.run(args)
            self.assertEqual(capture.rows, 2)
            self.assertEqual(capture.lost, 0)
            self.assertEqual(writes, [log.control_packet(20)] * 2 + [log.control_packet(20, stop=True)])
            self.assertEqual(len(output.read_text(encoding='utf-8').splitlines()), 3)

    async def test_live_capture_reports_missing_firmware_and_still_stops(self):
        writes = []

        class Scanner:
            @staticmethod
            async def find_device_by_name(*args, **kwargs):
                return SimpleNamespace(address='TEST')

        class Client:
            def __init__(self, *args, **kwargs):
                pass

            async def __aenter__(self):
                return self

            async def __aexit__(self, *args):
                pass

            async def read_gatt_char(self, uuid):
                return bytes(16)

            async def start_notify(self, uuid, callback):
                pass

            async def write_gatt_char(self, uuid, data, response):
                writes.append(bytes(data))

        with tempfile.TemporaryDirectory() as directory:
            args = SimpleNamespace(name='TEST', address=None, period_ms=20, duration=0.01,
                                   output=Path(directory) / 'empty.csv')
            with patch('bleak.BleakClient', Client), patch('bleak.BleakScanner', Scanner), \
                 contextlib.redirect_stdout(io.StringIO()), self.assertRaises(TimeoutError):
                await log.run(args)
            self.assertEqual(writes[-1], log.control_packet(20, stop=True))

    async def test_shell_enters_mode_without_ack_and_saves_exact_response(self):
        commands = []
        response = b'MOUSE_VERSION 1.0\r\n'

        class Scanner:
            @staticmethod
            async def find_device_by_name(*args, **kwargs):
                return SimpleNamespace(address='TEST')

        class Client:
            def __init__(self, *args, **kwargs):
                self.buffer = bytearray()

            async def __aenter__(self):
                return self

            async def __aexit__(self, *args):
                pass

            async def read_gatt_char(self, *args):
                return bytes(16)

            async def start_notify(self, uuid, callback):
                self.notify = callback

            async def write_gatt_char(self, uuid, data, response):
                self.buffer.extend(data)
                if data == b'\r':
                    commands.append(bytes(self.buffer))
                    if self.buffer == b'info ver\r':
                        self.notify(None, bytearray(b'MOUSE_VERSION 1.0\r\n'))
                    self.buffer.clear()

        with tempfile.TemporaryDirectory() as directory:
            raw_output = Path(directory) / 'response.bin'
            args = SimpleNamespace(name='TEST', address=None, command=['info ver'],
                enter_shell=True, char_delay=0, response_timeout=1, idle_timeout=0.01,
                raw_output=raw_output)
            with patch('bleak.BleakClient', Client), patch('bleak.BleakScanner', Scanner), \
                 contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                await shell.run(args)
            self.assertEqual(commands, [b'shell\r', b'info ver\r'])
            self.assertEqual(raw_output.read_bytes(), response)

    async def test_shell_matches_ascii_cr_and_character_pacing(self):
        class Client:
            writes = []

            async def write_gatt_char(self, uuid, data, response):
                self.writes.append(data)

        client = Client()
        await shell.send_command(client, 'info ver', asyncio.Lock(), 0)
        self.assertEqual(client.writes, [bytes([byte]) for byte in b'info ver\r'])
        with self.assertRaises(UnicodeEncodeError):
            await shell.send_command(client, '日本語', asyncio.Lock(), 0)
        self.assertEqual(b''.join(client.writes), b'info ver\r')

    async def test_gateway_round_trip_and_gui_reset_with_a_fake_ble_peer(self):
        server_ready = asyncio.Event()
        listen_port = []
        original_start_server = asyncio.start_server

        async def start_server(*args, **kwargs):
            server = await original_start_server(*args, **kwargs)
            listen_port.append(server.sockets[0].getsockname()[1])
            server_ready.set()
            return server

        class Scanner:
            @staticmethod
            async def find_device_by_name(*args, **kwargs):
                return SimpleNamespace(address='TEST')

        class Client:
            def __init__(self, *args, **kwargs):
                self.notify = None

            async def __aenter__(self):
                return self

            async def __aexit__(self, *args):
                pass

            async def read_gatt_char(self, uuid):
                return bytes(16)

            async def start_notify(self, uuid, callback):
                self.notify = callback

            async def write_gatt_char(self, uuid, data, response):
                self.notify(None, bytearray(data))

        with patch('bleak.BleakClient', Client), patch('bleak.BleakScanner', Scanner), \
             patch.object(gateway.asyncio, 'start_server', start_server):
            task = asyncio.create_task(gateway.run(SimpleNamespace(name='TEST', address=None, port=0)))
            try:
                await asyncio.wait_for(server_ready.wait(), timeout=3)
                reader, writer = await asyncio.open_connection('127.0.0.1', listen_port[0])
                payload = b'info ver\r' + bytes(range(256)) * 4
                writer.write(payload)
                await writer.drain()
                self.assertEqual(await asyncio.wait_for(reader.readexactly(len(payload)), timeout=3), payload)
                linger_format = 'HH' if sys.platform == 'win32' else 'ii'
                writer.get_extra_info('socket').setsockopt(
                    socket.SOL_SOCKET, socket.SO_LINGER, struct.pack(linger_format, 1, 0))
                writer.transport.abort()
                await asyncio.wait_for(task, timeout=3)
            finally:
                if not task.done():
                    task.cancel()
                    await asyncio.gather(task, return_exceptions=True)

    async def test_binary_writes_are_chunked_and_busy_retried(self):
        class Client:
            accepted = bytearray()
            calls = 0

            async def write_gatt_char(self, uuid, data, response):
                self.calls += 1
                if self.calls == 2:
                    raise BleakGATTProtocolError(0x11)
                self.accepted.extend(data)
                self_test.assertLessEqual(len(data), 20)
                self_test.assertEqual(uuid, gateway.NUS_RX)
                self_test.assertTrue(response)

        self_test = self
        client = Client()
        payload = bytes(range(256)) * 4
        await gateway.write_bytes(client, payload, asyncio.Lock())
        self.assertEqual(client.accepted, payload)
        self.assertEqual(client.calls, 53)  # 52 accepted writes plus one rejection.

    async def test_ambiguous_timeout_is_not_replayed(self):
        class Client:
            calls = 0

            async def write_gatt_char(self, *args, **kwargs):
                self.calls += 1
                raise TimeoutError('may have reached the robot')

        client = Client()
        with self.assertRaises(TimeoutError):
            await gateway.write_bytes(client, b'info ver\r', asyncio.Lock())
        self.assertEqual(client.calls, 1)

    async def test_explicit_other_gatt_error_is_not_retried(self):
        class Client:
            calls = 0

            async def write_gatt_char(self, *args, **kwargs):
                self.calls += 1
                raise BleakGATTProtocolError(0x03)

        client = Client()
        with self.assertRaises(BleakGATTProtocolError):
            await gateway.write_bytes(client, b'help\r', asyncio.Lock())
        self.assertEqual(client.calls, 1)

    def test_status_detects_lost_bytes_but_not_rejected_writes(self):
        initial = gateway.decode_status(struct.pack('<IIII', 2, 0, 0, 0))
        gateway.check_status(initial, (2, 0, 15, 0))
        with self.assertRaises(RuntimeError):
            gateway.check_status(initial, (3, 0, 15, 0))
        with self.assertRaises(ValueError):
            gateway.decode_status(b'wrong firmware')


if __name__ == '__main__':
    unittest.main()
