"""GUI開始・録画なし・可変周期ログと画像を、実機を使わず検証する。"""
from copy import deepcopy
from dataclasses import replace
import csv
import io
import json
from pathlib import Path
import struct
import sys
import tempfile
import time
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'ble'))
import pixel_camera_tuning as tuning
from pixel_camera_turn_capture import Shell, GuiMotionOnce, save_retained_log
from test_pixel_camera_tuning import settings, motion_rows


CAP = ('DEBUG_GUI_CAP protocol:1 ttl_ms:15000 search_turn:1 turn_log_hold:1 '
       'search_turn_log_hold:1 saved_log_period_ms:1 saved_log_capacity:400 saved_log_max_period_ms:100')


class ButtonTrialTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.sent, self.started = [], False
        self.fail_write = False
        self.capability = CAP
        self.request = tuning.TrialRequest(settings(), tuning.settings_set_command(settings()), 0,
            800, self.root/'missing-reference', self.root/'missing-geometry', operator_ready=True,
            record_video=False, log_period_ms=5, camera_settings={})
        self.session = SimpleNamespace(project=self.root, folder=self.root, camera=None,
            alive=lambda: True, connection=SimpleNamespace(is_open=True))
        outer = self
        class FakeShell:
            def __init__(self, *args, **kwargs): pass
            def send(self, command):
                outer.sent.append(command)
                if ' exe_gui ' in command:
                    outer.started = True
                    if outer.fail_write: raise TimeoutError('ambiguous write')
            def check_battery(self): pass
            def query(self, command, predicate):
                outer.sent.append(command)
                response = (outer.capability if command == 'debug gui show' else
                    'DEBUG_GUI_PERIOD period_ms:5' if command == 'debug gui period 5' else 'settings')
                return response
            def wait_line(self, predicate, timeout=8):
                command = outer.sent[-1]
                response = ('DEBUG_GUI_ARMED token:' if command.startswith('debug gui arm ') else
                            'DEBUG_GUI_RUN_START token:')+command.split()[-1]
                assert predicate(response)
                return response
        self.shell = FakeShell
        class Tail:
            def __init__(self, source, target): self.target = target
            @property
            def latest_at(self): return time.monotonic()
            def snapshot(self): return motion_rows() if outer.started else motion_rows()[:1]
            def close(self):
                rows = self.snapshot()
                with self.target.open('x', newline='', encoding='utf-8') as output:
                    writer = csv.DictWriter(output, fieldnames=rows[0])
                    writer.writeheader(); writer.writerows(rows)
        self.tail = Tail
        self.health = patch.object(tuning, 'last_health', return_value=dict(
            period_ms=5, rows=999, lost=0, crc_errors=0, monotonic=time.monotonic(), latest_sequence=999))
        self.health.start(); self.addCleanup(self.health.stop)
        self.sleep = patch.object(tuning.time, 'sleep')
        self.sleep.start(); self.addCleanup(self.sleep.stop)

    def capture(self):
        return tuning.capture_connected(self.session, self.request, self.root/'run',
            lambda *args: deepcopy(settings()), shell_type=self.shell, tail_type=self.tail)

    def test_camera_free_button_start_verifies_period_and_saves_once(self):
        with patch.object(tuning, 'save_retained_log', return_value=dict(status='saved', period_ms=5)) as saved:
            result = self.capture()
        self.assertEqual(result['state'], 'LOGGED_REVIEW_REQUIRED')
        self.assertEqual(result['saved_log_period_ms'], 5)
        self.assertEqual(result['trigger_mode'], 'operator_button_without_hand')
        self.assertTrue(result['gui_run_start_confirmed'])
        self.assertTrue(result['log_stop_evidence']['confirmed'])
        self.assertTrue((self.root/'run/motion.csv').exists())
        self.assertEqual(sum(' exe_gui ' in command for command in self.sent), 1)
        self.assertFalse(any('camera' in command or 'preset' in command for command in self.sent))
        self.assertEqual(saved.call_args.kwargs, dict(period_ms=5, done_marker='DEBUG_GUI_RUN_DONE'))

    def test_ambiguous_execution_is_never_retried(self):
        self.fail_write = True
        result = self.capture()
        self.assertEqual(result['state'], 'FAILED')
        self.assertTrue(result['execution_write_attempted'])
        self.assertEqual(sum(' exe_gui ' in command for command in self.sent), 1)
        self.assertEqual(result['execution_resend_count'], 0)

    def test_old_firmware_never_executes_requested_nondefault_period(self):
        self.capability = CAP.replace(' saved_log_max_period_ms:100', '')
        result = self.capture()
        self.assertEqual(result['state'], 'FAILED')
        self.assertFalse(self.started)

    def test_recording_off_ignores_missing_camera_profiles_and_confirmation_is_required(self):
        self.request.validate()
        with self.assertRaises(ValueError): replace(self.request, operator_ready=False).validate()
        for period in (0, 101, 1.5, True):
            with self.subTest(period=period), self.assertRaises(ValueError):
                replace(self.request, log_period_ms=period).validate()


class VariableLogTests(unittest.TestCase):
    def shell(self, period, readback=None):
        from ble.mouse_ble_log import LOG_LABELS
        labels = ['cnt']+list(LOG_LABELS)
        frame = struct.Struct('<HH49e4h2H')
        payload = b'DEBUG_GUI_RUN_DONE\r\n'+('HEADER,'+','.join(labels)+'\r\nBINARY\r\n').encode()
        payload += b''.join(frame.pack(0xA55D, index*period, *([0.0]*49), -7, 2049, 5, 10, 2209, 7)
                            for index in range(5))
        payload += frame.pack(0xFFFF, 0, *([0]*55))+b'END\r\n'
        wire = io.BytesIO(payload)
        connection = Mock()
        connection.readline.side_effect = wire.readline
        connection.read.side_effect = wire.read
        shell = Shell(connection, io.BytesIO(), Mock(), char_delay=0)
        shell.query = Mock(return_value=f'DEBUG_GUI_CAP saved_log_period_ms:{period if readback is None else readback}')
        return shell

    def test_actual_period_and_integer_values_survive_dump_and_plot(self):
        from turn_tuning_log import render_log
        for period in (1, 2, 5, 100):
            with self.subTest(period=period), tempfile.TemporaryDirectory() as name:
                folder = Path(name)
                result = save_retained_log(self.shell(period), folder, settings(), CAP,
                    'long_r180', Mock(), period_ms=period, done_marker='DEBUG_GUI_RUN_DONE')
                rows = tuning.read_rows(folder/result['csv'])
                self.assertEqual([int(row['cnt']) for row in rows], [i*period for i in range(5)])
                self.assertEqual(rows[0]['sen_r.value'], '2049')
                trial = dict(mouse='A', motion='long_r180', saved_log=result)
                (folder/'trial.json').write_text(json.dumps(trial), encoding='utf-8')
                image = render_log(folder/'trial.json')
                self.assertGreater(image.stat().st_size, 10000)
                self.assertTrue((folder/'log_preview.png').exists())
                self.assertEqual(json.loads((folder/'trial.json').read_text())['log_image_period_ms'], period)

    def test_mismatched_period_fails_before_dump(self):
        shell = self.shell(5, readback=1)
        with tempfile.TemporaryDirectory() as name, self.assertRaisesRegex(RuntimeError, '記録周期'):
            save_retained_log(shell, name, settings(), CAP, 'long_r180', Mock(), period_ms=5,
                done_marker='DEBUG_GUI_RUN_DONE')
        shell.connection.write.assert_not_called()

    def test_button_permit_cannot_be_called_twice(self):
        shell = Mock()
        motion = GuiMotionOnce(shell, 'long_r180', Mock(), Mock(), token=123)
        motion()
        with self.assertRaises(RuntimeError): motion()
        self.assertEqual(shell.send.call_count, 2)


class BleTargetGuiTests(unittest.TestCase):
    def setUp(self):
        import tkinter as tk
        from tkinter import ttk
        import pixel_camera_tuning_panel as panel
        self.panel_module = panel
        self.root = tk.Tk(); self.root.withdraw()
        self.addCleanup(self.root.destroy)
        self.gui = SimpleNamespace(ser=None, _camera_busy=False, lock=threading.Lock(),
            values={'sp_kp': tk.StringVar(value='42')})
        self.panel = panel.PixelTuningPanel(self.gui, ttk.Notebook(self.root))

    def test_machine_switch_updates_both_identifiers_and_preserves_per_machine_edits(self):
        self.panel.mouse.set('A')
        self.assertEqual(self.panel.device_name.get(), 'HY0020-MOUSE-A')
        self.assertEqual(self.panel.address.get(), 'C1:B5:60:87:91:73')
        self.panel.address.set('AA:BB:CC:DD:EE:FF')
        self.panel.ready.set(True); self.gui.values['sp_kp'].set('42')
        self.panel.mouse.set('B')
        self.assertEqual(self.panel.device_name.get(), 'HY0020-MOUSE-B')
        self.assertEqual(self.panel.address.get(), 'E1:7F:86:21:84:73')
        self.assertIn('HY0020-MOUSE-B', self.panel.connection_target.get())
        self.assertFalse(self.panel.ready.get())
        self.assertEqual(self.gui.values['sp_kp'].get(), '')
        self.panel.mouse.set('A')
        self.assertEqual(self.panel.address.get(), 'AA:BB:CC:DD:EE:FF')

    def test_other_machine_address_is_rejected_before_preparation(self):
        self.panel.mouse.set('B'); self.panel.address.set('C1:B5:60:87:91:73')
        with patch.object(self.panel, '_launch') as launch:
            self.panel.prepare()
        launch.assert_not_called()
        self.assertIn('機体AのBLEアドレス', self.panel.status.get())

    def test_connected_machine_cannot_be_relabelled(self):
        self.panel.mouse.set('A')
        self.panel.connected_mouse = 'A'; self.gui.ser = SimpleNamespace(is_open=True)
        self.panel.mouse.set('B')
        self.assertEqual(self.panel.mouse.get(), 'A')
        self.assertEqual(self.panel.address.get(), 'C1:B5:60:87:91:73')
        self.assertIn('Disconnect', self.panel.status.get())

    def test_legacy_b_settings_never_restore_a_address(self):
        from tkinter import ttk
        original = Path.read_text
        def read(path, *args, **kwargs):
            if path.name == 'tuning_ui.json':
                return json.dumps(dict(mouse='B', address='C1:B5:60:87:91:73'))
            return original(path, *args, **kwargs)
        with patch.object(Path, 'read_text', read):
            panel = self.panel_module.PixelTuningPanel(self.gui, ttk.Notebook(self.root))
        self.assertEqual(panel.device_name.get(), 'HY0020-MOUSE-B')
        self.assertEqual(panel.address.get(), 'E1:7F:86:21:84:73')

    def test_top_connect_button_prepares_ble_instead_of_opening_missing_socket(self):
        from myshell_debug import DebugGui
        from turn_tuning import configure_tuner
        gui = DebugGui(); gui.withdraw()
        try:
            configure_tuner(gui)
            with patch.object(gui.camera_panel, 'prepare') as prepare:
                gui.connect_button.invoke()
            prepare.assert_called_once()
        finally:
            gui.destroy()

    def test_b_preparation_uses_b_name_and_address(self):
        import tkinter as tk
        from tkinter import ttk
        self.panel.mouse.set('B'); self.panel.record_video.set(False)
        self.gui._connection_generation = 0
        self.gui.port = tk.StringVar()
        self.gui.connect_button = ttk.Button(self.root)
        prepared = SimpleNamespace(connection=SimpleNamespace(is_open=True))
        with tempfile.TemporaryDirectory() as directory:
            self.panel.preferences = Path(directory)/'tuning_ui.json'
            with patch.object(self.panel_module, 'check_log_environment'), \
                    patch.object(self.panel_module, 'LiveSession') as factory, \
                    patch.object(self.panel, '_launch', side_effect=lambda operation, callback: callback(operation())):
                factory.return_value.prepare.return_value = prepared
                self.panel.prepare()
            call = factory.return_value.prepare.call_args
            self.assertEqual(call.args, ('E1:7F:86:21:84:73',))
            self.assertEqual(call.kwargs['device_name'], 'HY0020-MOUSE-B')
            self.assertFalse(call.kwargs['record_video'])
            saved = json.loads(self.panel.preferences.read_text(encoding='utf-8'))
            self.assertEqual(saved['addresses']['B'], 'E1:7F:86:21:84:73')


if __name__ == '__main__':
    unittest.main()
