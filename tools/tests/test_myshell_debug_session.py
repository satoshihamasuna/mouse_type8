"""Headless debug-GUI session tests; no Tk window, serial port or BLE is opened."""
from collections import deque
from pathlib import Path
import queue
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import myshell_debug as debug


PARAMETER_LINE = (
    b'DEBUG_PARAM straight distance:360 acc:6.5 max_velo:0.7 end_velo:0 '
    b'sp:4,0.05,0 om:0.1,0.01,0 '
    b'ff:0.4239,0.09665,0.3017,0.00602,0.0011,0.0011,0 '
    b'suction:0 duty:650\r\n'
)


class Clock:
    def __init__(self):
        self.now = 0.0
        self.sleeps = []

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds
        self.sleeps.append(seconds)


class Variable:
    def __init__(self, value):
        self.value = value

    def get(self):
        return self.value

    def set(self, value):
        self.value = value


class FakeSerial:
    """Mode selection accepts shell only; shell emits realistic line responses."""
    def __init__(self, clock, *, mode='select', voltage=7500, status='OK', respond=True):
        self.clock = clock
        self.mode = mode
        self.voltage = voltage
        self.status = status
        self.respond = respond
        self.is_open = True
        self.pending = bytearray()
        self.lines = deque()
        self.commands = []
        self.owner_lock = None
        self.after_set_voltage = None

    def write(self, data):
        if not self.is_open:
            raise ConnectionError('closed')
        if self.owner_lock is not None:
            assert self.owner_lock.locked(), 'Serial operation escaped the worker lock'
        for byte in data:
            if byte != 13:
                self.pending.append(byte)
                continue
            command = self.pending.decode('ascii')
            self.pending.clear()
            self.commands.append(command)
            if self.mode == 'select':
                if command == 'shell':
                    self.mode = 'shell'
                continue
            if command == 'shell':
                self.lines.append(b'Unknown command found.\r\n')
            elif command == 'info battery' and self.respond:
                self.lines.append(('BATTERY voltage_mv:{} limit_mv:7000 status:{}\r\n'.format(
                    self.voltage, self.status)).encode('ascii'))
            elif command == 'debug straight show':
                self.lines.append(PARAMETER_LINE)
            elif ' set ' in command:
                self.lines.append(b'DEBUG_PARAM_SET_DONE\r\n')
                if self.after_set_voltage is not None:
                    self.voltage = self.after_set_voltage
        return len(data)

    def flush(self):
        pass

    def reset_input_buffer(self):
        self.lines.clear()

    def readline(self):
        if not self.is_open:
            raise ConnectionError('closed')
        if self.lines:
            return self.lines.popleft()
        self.clock.sleep(0.1)
        return b''

    def close(self):
        self.is_open = False

    def reboot_mouse(self):
        self.mode = 'select'
        self.pending.clear()
        self.lines.clear()


class HeadlessGui:
    # Run the production session/worker methods, replacing only UI and serial I/O.
    _toggle = debug.DebugGui._toggle
    _assert_connection = debug.DebugGui._assert_connection
    _prepare_shell = debug.DebugGui._prepare_shell
    _check_battery_voltage = debug.DebugGui._check_battery_voltage
    _write_command = debug.DebugGui._write_command
    _start = debug.DebugGui._start
    _worker = debug.DebugGui._worker
    _queue_float_params = debug.DebugGui._queue_float_params

    def __init__(self, port=None):
        self.ser = port
        self.lock = threading.Lock()
        if port is not None:
            port.owner_lock = self.lock
        self._connection_generation = 1 if port is not None else 0
        self._shell_ready = False
        self.char_delay = Variable(0)
        self.timeout = Variable(0.3)
        self.port = Variable('BLE')
        self.baud = Variable(460800)
        self.battery_status = Variable('Battery: --')
        self.connect_button = SimpleNamespace(configure=lambda **kwargs: None)
        self.events = queue.Queue()
        self.messages = []
        self.values = {'distance': Variable(123.45)}

    def _append(self, message):
        self.messages.append(message)

    def run_show(self):
        self._worker(('debug straight show',), self._connection_generation)

    def parameter_events(self):
        return [item for item in self.events.queue if isinstance(item, tuple) and item[0] == 'float_params']


class DebugSessionTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        clock_patch = patch.object(debug, 'time', self.clock)
        clock_patch.start()
        self.addCleanup(clock_patch.stop)

    def test_ble_socket_and_com_commands_keep_the_explicit_character_delay(self):
        for field_port, serial_port in (('BLE', None), ('OTHER', 'socket://127.0.0.1:8765'),
                                       ('COM_TEST', None)):
            with self.subTest(field_port=field_port):
                port = FakeSerial(self.clock, mode='shell')
                if serial_port is not None:
                    port.port = serial_port
                gui = HeadlessGui(port)
                gui.port.set(field_port)
                gui.char_delay.set(0.005)
                start = self.clock.monotonic()
                before = len(self.clock.sleeps)
                with gui.lock:
                    gui._write_command('info battery')
                self.assertAlmostEqual(self.clock.monotonic() - start, 13 * 0.005)
                self.assertEqual(self.clock.sleeps[before:], [0.005] * 13)
                self.assertEqual(gui.char_delay.get(), 0.005)
                self.assertEqual(port.commands, ['info battery'])

    def test_cold_device_timeout_without_shell_setup_then_real_setup_reads_values(self):
        old_port = FakeSerial(self.clock)
        old_gui = HeadlessGui(old_port)
        # Negative control removes only the new setup step, reproducing the old
        # GUI's direct battery query from mode selection.
        with patch.object(old_gui, '_prepare_shell', lambda: None):
            old_gui.run_show()
        self.assertFalse(old_gui.parameter_events())
        self.assertTrue(any('Battery voltage check timed out' in line for line in old_gui.messages))
        self.assertEqual(old_port.commands, ['info battery', 'info battery'])

        port = FakeSerial(self.clock)
        gui = HeadlessGui(port)
        gui.run_show()
        self.assertEqual(port.commands, ['shell', 'info battery', 'debug straight show'])
        self.assertEqual(gui.parameter_events()[0][1]['distance'], 360)
        self.assertTrue(gui._shell_ready)
        self.assertFalse(any(line.startswith('ERROR:') for line in gui.messages))

    def test_already_in_shell_prepares_once_but_checks_battery_for_every_command(self):
        port = FakeSerial(self.clock, mode='shell')
        gui = HeadlessGui(port)
        gui.run_show()
        gui.run_show()
        self.assertEqual(port.commands, ['shell', 'info battery', 'debug straight show',
                                         'info battery', 'debug straight show'])
        self.assertEqual(len(gui.parameter_events()), 2)
        self.assertFalse(any('Unknown command' in line for line in gui.messages))

    def test_mouse_reboot_with_an_open_connection_reenters_shell_once(self):
        port = FakeSerial(self.clock)
        gui = HeadlessGui(port)
        gui.run_show()
        port.reboot_mouse()
        prior = len(port.commands)
        gui.run_show()
        self.assertEqual(port.commands[prior:], ['info battery', 'shell', 'info battery',
                                                'debug straight show'])
        self.assertEqual(len(gui.parameter_events()), 2)
        self.assertTrue(gui._shell_ready)

    def test_low_battery_or_error_status_blocks_parameter_and_execution_commands(self):
        for voltage, status in ((6900, 'OK'), (7500, 'ERROR')):
            with self.subTest(voltage=voltage, status=status):
                port = FakeSerial(self.clock, mode='shell', voltage=voltage, status=status)
                gui = HeadlessGui(port)
                gui._worker(('debug straight set 360 6.5 0.7', 'debug straight exe'), 1)
                self.assertEqual(port.commands, ['shell', 'info battery'])
                self.assertTrue(any('too low' in line for line in gui.messages))
                self.assertTrue(any(item[0] == 'battery_error' for item in gui.events.queue))

    def test_unresponsive_device_never_receives_set_or_execution_commands(self):
        port = FakeSerial(self.clock, respond=False)
        gui = HeadlessGui(port)
        gui._worker(('debug straight set 360 6.5 0.7', 'debug straight exe'), 1)
        self.assertEqual(port.commands, ['shell', 'info battery', 'shell', 'info battery'])
        self.assertFalse(gui._shell_ready)
        self.assertTrue(any('timed out' in line for line in gui.messages))
        self.assertFalse(gui.lock.locked())

    def test_battery_is_rechecked_between_parameter_update_and_execution(self):
        port = FakeSerial(self.clock, mode='shell')
        port.after_set_voltage = 6900
        gui = HeadlessGui(port)
        gui._worker(('debug straight set 360 6.5 0.7', 'debug straight exe'), 1)
        self.assertEqual(port.commands, ['shell', 'info battery',
                                         'debug straight set 360 6.5 0.7', 'info battery'])
        self.assertTrue(any('too low' in line for line in gui.messages))

    def test_reconnection_prepares_new_session_without_resetting_input_values(self):
        old_port = FakeSerial(self.clock)
        gui = HeadlessGui(old_port)
        gui.run_show()
        old_generation = gui._connection_generation
        gui._toggle()
        self.assertFalse(old_port.is_open)
        self.assertFalse(gui._shell_ready)
        port = FakeSerial(self.clock)
        port.owner_lock = gui.lock
        with patch.object(debug, 'serial', SimpleNamespace(Serial=lambda *args, **kwargs: port)):
            gui._toggle()
        self.assertGreater(gui._connection_generation, old_generation)
        self.assertFalse(gui._shell_ready)
        self.assertEqual(gui.values['distance'].get(), 123.45)
        gui.run_show()
        self.assertEqual(port.commands, ['shell', 'info battery', 'debug straight show'])

    def test_busy_worker_prevents_connection_switch_without_blocking_the_ui(self):
        port = FakeSerial(self.clock)
        gui = HeadlessGui(port)
        gui._shell_ready = True
        with gui.lock:
            gui._toggle()
        self.assertIs(gui.ser, port)
        self.assertTrue(port.is_open)
        self.assertTrue(gui._shell_ready)
        self.assertEqual(gui._connection_generation, 1)
        self.assertTrue(any('通信処理中' in line for line in gui.messages))

    def test_queued_old_command_is_cancelled_after_disconnect_and_reconnect(self):
        gui = HeadlessGui(FakeSerial(self.clock))
        queued = []
        class DelayedThread:
            def __init__(self, *, target, args, daemon):
                self.target, self.args = target, args
            def start(self):
                queued.append(self)
        with patch.object(debug.threading, 'Thread', DelayedThread):
            gui._start('debug straight exe')
        gui._toggle()
        new_port = FakeSerial(self.clock)
        with patch.object(debug, 'serial', SimpleNamespace(Serial=lambda *args, **kwargs: new_port)):
            gui._toggle()
        queued[0].target(*queued[0].args)
        self.assertEqual(new_port.commands, [])
        self.assertTrue(any('Connection changed' in line for line in gui.messages))

    def test_failed_connection_setup_closes_port_and_clears_readiness(self):
        gui = HeadlessGui()
        port = FakeSerial(self.clock)
        def fail_reset():
            raise ConnectionError('input reset failed')
        port.reset_input_buffer = fail_reset
        with patch.object(debug, 'serial', SimpleNamespace(Serial=lambda *args, **kwargs: port)), \
                patch.object(debug.messagebox, 'showerror') as error:
            gui._toggle()
        self.assertIsNone(gui.ser)
        self.assertFalse(port.is_open)
        self.assertFalse(gui._shell_ready)
        self.assertFalse(gui.lock.locked())
        error.assert_called_once()


if __name__ == '__main__':
    unittest.main()
