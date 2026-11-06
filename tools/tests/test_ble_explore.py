"""Real-exploration UI/session tests use simulated peers and never move a mouse."""
import asyncio
import binascii
import csv
import io
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
for path in (ROOT / '.tools/ble_python',
             ROOT / '.tools' / ('ble_python{}{}'.format(*sys.version_info[:2])), ROOT / 'tools/ble'):
    sys.path.insert(0, str(path))
if sys.version_info < (3, 10):
    raise unittest.SkipTest('BLE session requires Python 3.10+')
try:
    import bleak
except ImportError as error:
    raise unittest.SkipTest(str(error))
import mouse_ble_explore as explore
import mouse_ble_explore_gui as gui
from mouse_ble_log import control_packet
from test_side_wall_stability import compiler_command


def sample(sequence):
    payload = struct.pack('<4sIIH55e', b'L8S2', sequence, 100 + sequence * 20, 20, *range(55))
    return payload + struct.pack('<H', binascii.crc_hqx(payload, 0xffff))


class OptionsTests(unittest.TestCase):
    def test_shortest_command_rejects_invalid_map_and_command_injection(self):
        self.assertEqual(explore.ShortestOptions().command(), 'shortest start 7 7 2 plain700 ram 120')
        for options in (explore.ShortestOptions(profile='plain700\r'), explore.ShortestOptions(maze_source='other'),
                        explore.ShortestOptions(goal_size=99), explore.ShortestOptions(profile='p;bad')):
            with self.assertRaises(ValueError):
                options.command()

    def test_shortest_requires_firmware_support_and_a_completed_ram_map(self):
        session = explore.ExploreSession()
        session.state, session.battery_ok = 'idle', True
        session.capability = {'protocol': '1'}
        self.assertTrue(session.ready())
        self.assertFalse(session.ready(explore.ShortestOptions(maze_source='flash')))
        session.shortest_capability = {'profiles': 'plain700,acc1600_v1', 'ram_maze': '0'}
        self.assertFalse(session.ready(explore.ShortestOptions()))
        self.assertTrue(session.ready(explore.ShortestOptions(maze_source='flash')))
        session.shortest_capability['ram_maze'] = '1'
        self.assertTrue(session.ready(explore.ShortestOptions()))
        self.assertFalse(session.ready(explore.ShortestOptions(profile='unknown')))

    def test_real_start_command_and_boundaries(self):
        self.assertEqual(explore.ExploreOptions().command(), 'explore start 7 7 2 plain roundtrip 360')
        self.assertEqual(explore.ExploreOptions(1, 0, 1, True, False, 20).command(),
                         'explore start 1 0 1 acc outward 20')
        for options in (explore.ExploreOptions(-1, 2), explore.ExploreOptions(31, 31, 2),
                        explore.ExploreOptions(0, 0), explore.ExploreOptions(limit_seconds=3601)):
            with self.assertRaises(ValueError):
                options.command()

    def test_motion_control_packets_do_not_stop_telemetry(self):
        self.assertEqual(control_packet(20, action=2)[3], 2)
        self.assertEqual(control_packet(20, action=3)[3], 3)
        with self.assertRaises(ValueError):
            control_packet(20, action=4)


class FirmwareCommandTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        command = compiler_command()
        if command is None:
            raise unittest.SkipTest('Host C++ compiler is required')
        build = ROOT / 'build'
        cls.work = tempfile.TemporaryDirectory(prefix='explore-command-test-', dir=build)
        cls.addClassCleanup(cls.work.cleanup)
        environment = os.environ.copy()
        if Path(command[0]).stem == 'zig':
            environment.setdefault('ZIG_LOCAL_CACHE_DIR', str(build / 'ir-confidence-zig-cache'))
            environment.setdefault('ZIG_GLOBAL_CACHE_DIR', str(build / 'ir-confidence-zig-global'))
            if os.name == 'nt':
                command += ['-target', 'x86_64-windows-gnu']
        cls.executables = {}
        for mouse in ('A', 'B'):
            executable = Path(cls.work.name) / ('explore-' + mouse + ('.exe' if os.name == 'nt' else ''))
            result = subprocess.run(command + [
                '-std=c++14', '-O0', '-Wall', '-Wextra', '-fno-exceptions', '-fno-rtti', '-DMOUSE_' + mouse,
                '-I', str(ROOT / 'tools/tests/explore_stubs'),
                '-I', str(ROOT / 'tools/tests/log_stream_stubs'), '-I', str(ROOT),
                '-I', str(ROOT / 'Component/ntshell-v0.3.1/src/lib'),
                '-x', 'c++',
                str(ROOT / 'tools/tests/explore_harness.cpp'), str(ROOT / 'System/Src/explore.cpp'),
                str(ROOT / 'tools/tests/shortest_harness.cpp'), str(ROOT / 'System/Src/shortest_run.cpp'),
                str(ROOT / 'Module/Src/remote_explore.cpp'),
                str(ROOT / 'Component/ntshell-v0.3.1/src/lib/core/ntlibc.c'), '-o', str(executable),
            ], capture_output=True, env=environment, timeout=240)
            if result.returncode:
                raise AssertionError(result.stdout.decode(errors='replace') + result.stderr.decode(errors='replace'))
            cls.executables[mouse] = executable

    def check_case(self, case, expected):
        for mouse, executable in self.executables.items():
            with self.subTest(mouse=mouse):
                result = subprocess.run([str(executable), case], capture_output=True, text=True, timeout=10)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertIn(expected, result.stdout)

    def test_capability_reports_target_without_starting(self):
        self.check_case('show', 'EXPLORE_CAP protocol:1')

    def test_complete_roundtrip_keeps_log_enabled_until_end(self):
        self.check_case('complete', 'EXPLORE_END result:complete')

    def test_stopping_the_hand_gesture_wait_does_not_start_motion(self):
        self.check_case('stop_wait', 'EXPLORE_END result:stopped')

    def test_stop_during_outward_does_not_start_a_return_leg(self):
        self.check_case('stop_motion', 'EXPLORE_END result:stopped')

    def test_start_requires_continuous_log_lease(self):
        self.check_case('no_stream', 'EXPLORE_ERROR not_armed_or_no_stream')

    def test_low_battery_does_not_start_motion(self):
        self.check_case('low_battery', 'EXPLORE_ERROR low_battery')

    def test_invalid_goal_size_does_not_start_motion(self):
        self.check_case('bad_goal', 'EXPLORE_ERROR invalid_argument')

    def test_shortest_capability_and_profiles_do_not_start_motion(self):
        self.check_case('shortest_show', 'SHORTEST_CAP protocol:1')

    def test_shortest_reuses_completed_ram_maze_and_existing_profile_types(self):
        for case in ('complete', 'suction', 'acc', 'flash'):
            self.check_case('shortest_' + case, 'SHORTEST_END result:complete')

    def test_shortest_requires_map_path_battery_arm_and_stream(self):
        for case, expected in (('missing_maze', 'END result:missing_maze'), ('no_path', 'END result:no_path'),
                               ('no_arm', 'ERROR not_armed_or_no_stream'), ('no_stream', 'ERROR not_armed_or_no_stream'),
                               ('low_battery', 'ERROR low_battery'), ('bad_profile', 'ERROR unknown_profile'),
                               ('bad_goal', 'ERROR invalid_argument'), ('bad_number', 'ERROR invalid_argument')):
            self.check_case('shortest_' + case, 'SHORTEST_' + expected)

    def test_shortest_cancellation_and_motion_error_never_report_complete(self):
        for case, result in (('stop_plan', 'stopped'), ('stop_wait', 'stopped'),
                             ('stop_motion', 'stopped'), ('time_limit', 'time_limit'),
                             ('link_timeout', 'link_timeout'), ('motion_error', 'motion_error')):
            self.check_case('shortest_' + case, 'SHORTEST_END result:' + result)


class SessionTests(unittest.IsolatedAsyncioTestCase):
    async def exercise(self, scenario='complete'):
        commands, controls, events, clients = [], [], [], []
        session = None
        running_rows = []

        def emit(kind, data):
            events.append((kind, data))
            if kind == 'state' and data == 'waiting_sensor':
                if scenario in ('stop', 'shortest_stop'):
                    session.action('stop')
                elif scenario == 'disconnect':
                    session.action('disconnect')
            elif kind == 'telemetry' and data['state'] == 'running':
                running_rows.append(data['rows'])

        class Scanner:
            @staticmethod
            async def find_device_by_name(*args, **kwargs):
                return SimpleNamespace(address='TEST')

        class Client:
            def __init__(self, *args, **kwargs):
                self.command = bytearray()
                self.sequence = 0
                self.producer = None
                self.complete_task = None
                self.exploring = False
                self.streaming = False
                self.last_request = None
                self.run_tag = 'EXPLORE'
                clients.append(self)

            async def __aenter__(self):
                return self

            async def __aexit__(self, *args):
                for task in (self.producer, self.complete_task):
                    if task:
                        task.cancel()
                        await asyncio.gather(task, return_exceptions=True)

            async def read_gatt_char(self, uuid):
                return bytes(16)

            def text(self, data):
                # Insert telemetry halfway through an ASCII status line.
                midpoint = len(data) // 2
                wire = data
                if self.streaming:
                    wire = data[:midpoint] + sample(self.sequence) + data[midpoint:]
                    self.sequence += 1
                for start in range(0, len(wire), 19):
                    self.notify(None, wire[start:start + 19])

            async def start_notify(self, uuid, callback):
                self.notify = callback

                async def produce():
                    while True:
                        if self.streaming and asyncio.get_running_loop().time() - self.last_request < 3:
                            self.notify(None, sample(self.sequence))
                            self.sequence += 1
                        await asyncio.sleep(0.02)
                self.producer = asyncio.create_task(produce())

            async def write_gatt_char(self, uuid, data, response):
                if data.startswith(b'\x00L8'):
                    controls.append(data[3])
                    if data[3] == 0:
                        self.streaming = False
                    elif data[3] == 1:
                        self.streaming = scenario != 'no_log'
                        self.last_request = asyncio.get_running_loop().time()
                    elif data[3] == 3 and self.sequence == 0:
                        raise AssertionError('Arm arrived before the first log sample')
                    elif data[3] == 2 and self.exploring:
                        self.exploring = False
                        self.text((self.run_tag + '_END result:stopped x:0 y:0 dir:0\r\n').encode())
                        await asyncio.sleep(0)  # ACK can arrive before the write await returns.
                    return
                if data == b'\x03':
                    self.command.clear()
                    return
                if scenario == 'duplex_sensitive' and self.streaming and data != b'\r':
                    raise ConnectionError('ASCII command overlapped binary log transfer')
                self.command.extend(data)
                if data == b'\r':
                    command = self.command.decode('ascii').strip()
                    self.command.clear()
                    commands.append(command)
                    if command == 'explore show' and scenario != 'old_firmware':
                        self.text(b'EXPLORE_CAP protocol:1 mouse:A goal_x:7 goal_y:7 goal_size:2 max_x:31 max_y:31 state:idle\r\n')
                    elif command == 'shortest show' and scenario != 'old_firmware':
                        self.text(b'SHORTEST_CAP protocol:1 mouse:A goal_x:7 goal_y:7 goal_size:2 max_x:31 max_y:31 state:idle ram_maze:0 profiles:plain700,acc1600_v1\r\n')
                    elif command == 'info battery':
                        self.text(b'BATTERY voltage_mv:7500 limit_mv:7000 status:OK\r\n')
                    elif command.startswith(('explore start', 'shortest start')):
                        self.run_tag = 'SHORTEST' if command.startswith('shortest') else 'EXPLORE'
                        self.assert_stream_ready()
                        if scenario == 'start_error':
                            self.text(b'EXPLORE_ERROR low_battery voltage_mv:6900 limit_mv:7000\r\n')
                            return
                        self.exploring = True
                        self.text((self.run_tag + '_WAIT_SENSOR goal:7,7,2 motion:plain route:roundtrip\r\n').encode())
                        if scenario in ('complete', 'restart', 'duplex_sensitive', 'explore_then_shortest'):
                            async def complete():
                                await asyncio.sleep(0.1)
                                self.text((self.run_tag + '_PHASE name:outward\r\n').encode())
                                await asyncio.sleep(3.4 if self.run_tag == 'SHORTEST' else 0.25)
                                if self.run_tag == 'SHORTEST':
                                    self.assert_stream_ready()  # Longer than a 3 s lease: renewal must continue.
                                self.text((self.run_tag + '_PHASE name:return\r\n').encode())
                                await asyncio.sleep(0.25)
                                self.exploring = False
                                self.text((self.run_tag + '_END result:complete x:0 y:0 dir:0\r\n').encode())
                            self.complete_task = asyncio.create_task(complete())

            def assert_stream_ready(self):
                if not self.streaming or asyncio.get_running_loop().time() - self.last_request >= 3:
                    raise AssertionError('Exploration command arrived without an active log lease')

        async def wait_until(condition):
            while not condition():
                await asyncio.sleep(0.01)

        async def drive():
            await wait_until(lambda: session.state == 'idle')
            self.assertEqual(session.capture.rows, 0)
            self.assertNotIn(1, controls)
            if scenario == 'old_firmware':
                self.assertFalse(session.ready())
                session.action('start', explore.ExploreOptions())
                await asyncio.sleep(0.2)
                session.action('disconnect')
                return
            self.assertTrue(session.ready())
            if scenario == 'idle':
                # Longer than the active-stream watchdog: idle silence is intentional.
                await asyncio.sleep(5.3)
                self.assertEqual(session.capture.rows, 0)
                self.assertNotIn(1, controls)
                self.assertTrue(session.ready())
                session.action('disconnect')
                return
            for run in range(2 if scenario in ('restart', 'explore_then_shortest') else 1):
                previous_results = sum(kind in ('result', 'run_error') for kind, _ in events)
                options = (explore.ShortestOptions() if scenario == 'explore_then_shortest' and run == 1 else
                           explore.ShortestOptions(maze_source='flash') if scenario == 'shortest_stop' else explore.ExploreOptions())
                if isinstance(options, explore.ShortestOptions):
                    self.assertTrue(session.ready(options))
                session.action('start', options)
                if scenario in ('disconnect', 'no_log'):
                    return
                await wait_until(lambda: sum(kind in ('result', 'run_error') for kind, _ in events) > previous_results)
                await wait_until(lambda: session.ready())
                self.assertFalse(session._streaming)
                await asyncio.sleep(0.1)  # Drain the final notification before checking idle.
                rows, requests = session.capture.rows, controls.count(1)
                await asyncio.sleep(0.65)
                self.assertEqual(session.capture.rows, rows)
                self.assertEqual(controls.count(1), requests)
                self.assertFalse(clients[0].streaming)
            session.action('disconnect')

        with tempfile.TemporaryDirectory() as directory:
            session = explore.ExploreSession(output_root=directory, emit=emit)
            with patch('bleak.BleakClient', Client), patch('bleak.BleakScanner', Scanner):
                driver = asyncio.create_task(drive())
                try:
                    if scenario == 'no_log':
                        with self.assertRaisesRegex(TimeoutError, 'ログを受信できない'):
                            await asyncio.wait_for(session.run(), timeout=20)
                    else:
                        await asyncio.wait_for(session.run(), timeout=30)
                    await driver
                finally:
                    driver.cancel()
                    await asyncio.gather(driver, return_exceptions=True)
            with (session.directory / 'telemetry.csv').open(encoding='utf-8', newline='') as f:
                rows = list(csv.DictReader(f))
            records = [json.loads(line) for line in (session.directory / 'events.jsonl').read_text(encoding='utf-8').splitlines()]
            if scenario in ('idle', 'old_firmware', 'no_log'):
                self.assertEqual(len(rows), 0)
            else:
                self.assertGreater(len(rows), 0)
            self.assertEqual(sum(int(r['lost_frames']) for r in rows), 0)
            self.assertEqual(session.capture.decoder.crc_errors, 0)
            self.assertEqual(session.state, 'disconnected')
            self.assertEqual(clients[0].command, b'')
            self.assertEqual(controls[-2:], [2, 0])
            self.assertTrue(any(record['event'] == 'summary' for record in records))
            if scenario in ('old_firmware', 'idle', 'no_log'):
                self.assertFalse(any(command.startswith('explore start') for command in commands))
                self.assertNotIn(3, controls)
                if scenario != 'no_log':
                    self.assertFalse(list(session.directory.glob('run_*.json')))
            else:
                count = 2 if scenario in ('restart', 'explore_then_shortest') else 1
                self.assertEqual(sum(command.startswith(('explore start', 'shortest start')) for command in commands), count)
                if scenario == 'explore_then_shortest':
                    self.assertEqual(sum(command.startswith('shortest start') for command in commands), 1)
                    self.assertEqual(len(clients), 1)
                    self.assertGreaterEqual(controls.count(1), 8)
                    settings = [json.loads(p.read_text(encoding='utf-8')) for p in sorted(session.directory.glob('run_*.json'))]
                    self.assertEqual([s['run_type'] for s in settings], ['explore', 'shortest'])
                    self.assertEqual(settings[1]['options']['maze_source'], 'ram')
                self.assertIn(3, controls)
                self.assertEqual(len(list(session.directory.glob('run_*.json'))), count)
                self.assertTrue(any(kind == ('run_error' if scenario == 'start_error' else 'result') for kind, _ in events))
            if scenario == 'complete':
                self.assertGreater(len(running_rows), 1)
                self.assertGreater(running_rows[-1], running_rows[0])
            return events

    async def test_one_link_captures_through_hand_wait_outward_and_return(self):
        await self.exercise()

    async def test_command_text_is_sent_before_logs_and_execution_waits_for_first_sample(self):
        await self.exercise('duplex_sensitive')

    async def test_user_stop_keeps_receiving_until_the_end_ack(self):
        await self.exercise('stop')

    async def test_connected_idle_does_not_request_logs_or_trigger_watchdog(self):
        await self.exercise('idle')

    async def test_next_exploration_restarts_logs_on_the_same_connection(self):
        await self.exercise('restart')

    async def test_exploration_then_shortest_reuses_one_connection_and_updates_ram_map(self):
        await self.exercise('explore_then_shortest')

    async def test_shortest_stop_waits_for_end_and_stops_logging(self):
        await self.exercise('shortest_stop')

    async def test_rejected_start_stops_logs_and_stays_connected(self):
        await self.exercise('start_error')

    async def test_missing_logs_never_sends_arm_or_exploration_command(self):
        await self.exercise('no_log')

    async def test_disconnect_waits_for_motion_stop_and_flushes_the_csv(self):
        await self.exercise('disconnect')

    async def test_older_firmware_never_receives_a_motion_command(self):
        await self.exercise('old_firmware')


class GuiTests(unittest.TestCase):
    def test_shortest_map_gate_profile_selection_and_stop_controls(self):
        root = gui.tk.Tk()
        root.withdraw()
        try:
            app = gui.ExploreApp(root)
            app.handle_event('shortest_capability', {'mouse': 'B', 'profiles': 'plain700,acc1600_v1', 'ram_maze': '0'})
            app.run_mode.set('最短走行')
            snapshot = {'rows': 0, 'lost': 0, 'crc_errors': 0, 'ready': True, 'shortest_ready': True,
                        'shortest_supported': True, 'ram_maze': False, 'state': 'idle', 'time_ms': None, 'values': {}}
            app.handle_event('telemetry', snapshot)
            self.assertEqual(str(app.start_button['state']), 'disabled')
            app.maze_source.set('保存済み迷路（Flash）')
            app.update_run_mode()
            self.assertEqual(str(app.start_button['state']), 'normal')
            self.assertEqual(str(app.profile_choice['state']), 'readonly')
            self.assertEqual(str(app.explore_controls[0]['state']), 'disabled')
            actions = []
            app.session = SimpleNamespace(action=lambda *args: actions.append(args))
            app.start()
            self.assertIsInstance(actions[0][1], explore.ShortestOptions)
            self.assertEqual(actions[0][1].maze_source, 'flash')
            app.handle_event('state', 'running')
            self.assertEqual(str(app.mode_choice['state']), 'disabled')
            self.assertEqual(str(app.stop_button['state']), 'normal')
        finally:
            root.destroy()

    def test_start_gate_live_values_and_running_buttons(self):
        root = gui.tk.Tk()
        root.withdraw()
        try:
            app = gui.ExploreApp(root)
            self.assertEqual(app.period.get(), 20)
            self.assertEqual(float(app.period_entry['from']), 3)
            self.assertEqual(float(app.period_entry['increment']), 1)
            app.handle_event('state', 'idle')
            snapshot = {'rows': 10, 'lost': 0, 'crc_errors': 0, 'ready': True,
                        'state': 'idle', 'time_ms': 100, 'values': {'ego.velo': 0.3}}
            app.handle_event('telemetry', snapshot)
            self.assertEqual(str(app.start_button['state']), 'normal')
            self.assertEqual(app.values.item('ego.velo')['values'][1], '0.3')
            self.assertIn('ログ配信停止', app.status.get())
            app.handle_event('telemetry', snapshot)
            self.assertEqual(len(app.history), 1)  # Idle status updates keep the final waveform.
            app.handle_event('state', 'waiting_sensor')
            self.assertEqual(str(app.start_button['state']), 'disabled')
            self.assertEqual(str(app.stop_button['state']), 'normal')
            app.handle_event('state', 'disconnected')
            self.assertEqual(str(app.stop_button['state']), 'disabled')
            self.assertEqual(str(app.connect_button['state']), 'normal')
        finally:
            root.destroy()


if __name__ == '__main__':
    unittest.main()
