"""Protocol tests use localhost and fake BLE peers; they never contact a robot."""
import asyncio
import contextlib
import io
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

if sys.version_info < (3, 10):
    raise unittest.SkipTest('BLE transport tests require Python 3.10+; see tools/ble/README.md')
try:
    import serial
    from serial.tools import list_ports
    from bleak.exc import BleakGATTProtocolError
except ImportError as error:
    raise unittest.SkipTest('Install BLE test dependencies; see tools/ble/README.md: {}'.format(error))
import mouse_ble_gateway as gateway
import mouse_gui_ble as launcher
import mouse_ble_shell as shell


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


class GattTransportTests(unittest.IsolatedAsyncioTestCase):
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

    async def test_gateway_round_trip_with_a_fake_ble_peer(self):
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
                writer.close()
                await writer.wait_closed()
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
