"""BLE・カメラ・Tk画面を開かずに撮影連動の一回実行を検証する。"""
import csv
import copy
import importlib.util
import io
import json
from pathlib import Path
import sys
import struct
import tempfile
import unittest
from unittest.mock import Mock

TOOLS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))
sys.path.insert(0, str(TOOLS / 'ble'))
spec = importlib.util.spec_from_file_location("pixel_camera_turn_capture", TOOLS / "pixel_camera_turn_capture.py")
capture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture)


def settings():
    return dict(motion_name="long_r180", preset_speed_mm_s=500,
                parameters=dict(velo=0.5), turn_count=1, suction=dict(enabled=False))


def full_settings(lend=24.47):
    value = settings()
    value.update(mouse="A", pid={"sp_kp": 2.0}, feedforward={"ff_sp_velo": 0.870046})
    value["parameters"]["lend"] = lend
    return value


class SettingsHandoffTests(unittest.TestCase):
    def test_preserve_mode_reads_show_without_overwriting_edited_ram(self):
        shell = Mock()
        expected = full_settings(19.47)
        parse = Mock(return_value=copy.deepcopy(expected))
        actual = capture.read_device_settings(shell, "long_r180", 500, parse, "A", expected)
        self.assertEqual(actual["parameters"]["lend"], 19.47)
        self.assertEqual(shell.query.call_args.args[0], "debug turn long_r180 show")
        shell.send.assert_not_called()

    def test_default_mode_still_loads_requested_preset(self):
        shell = Mock()
        capture.read_device_settings(shell, "long_r180", 500, Mock(return_value=settings()), "A")
        self.assertEqual(shell.query.call_args.args[0], "debug turn long_r180 preset 500")

    def test_reset_or_changed_ram_is_rejected_before_recording(self):
        with self.assertRaisesRegex(RuntimeError, "parameters"):
            capture.read_device_settings(Mock(), "long_r180", 500,
                                         Mock(return_value=full_settings()), "A", full_settings(19.47))

    def test_all_physical_settings_must_match_but_metadata_may_differ(self):
        actual = full_settings()
        expected = dict(copy.deepcopy(actual), source="expected_ui_values", captured_at="old")
        capture.verify_expected_settings(actual, expected)
        for key in ("parameters", "pid", "feedforward", "suction", "motion_name",
                    "preset_speed_mm_s", "turn_count", "mouse"):
            changed = copy.deepcopy(expected)
            changed[key] = {"other": 0} if isinstance(changed[key], dict) else "other"
            with self.subTest(key=key), self.assertRaises(RuntimeError):
                capture.verify_expected_settings(actual, changed)
            del changed[key]
            with self.subTest(missing=key), self.assertRaises(RuntimeError):
                capture.verify_expected_settings(actual, changed)


class RetainedLogTests(unittest.TestCase):
    capability = ('DEBUG_CAMERA_CAP protocol:1 ttl_ms:15000 turn_log_hold:1 search_turn_log_hold:1 '
                  'saved_log_period_ms:1 saved_log_capacity:400')

    def shell(self, *, indices=(0, 1), magic=0xA55D, terminate=True, header=True):
        from mouse_ble_log import LOG_LABELS
        labels = ['cnt'] + list(LOG_LABELS)
        frame = struct.Struct('<HH49e4h2H')
        payload = b'DEBUG_CAMERA_RUN_DONE\r\n'
        if header:
            payload += ('HEADER,' + ','.join(labels) + '\r\n').encode()
        payload += b'BINARY\r\n'
        for index in indices:
            payload += frame.pack(magic, index, *([.5] * 49), -32768, -7, 2049, 32767, 2209, 7)
        if terminate:
            payload += frame.pack(0xFFFF, 0, *([0] * 55)) + b'END\r\n'
        wire = io.BytesIO(payload)
        connection = Mock()
        connection.readline.side_effect = wire.readline
        connection.read.side_effect = lambda size: wire.read(min(size, 7))
        return capture.Shell(connection, io.BytesIO(), Mock(), char_delay=0)

    def test_stopped_turn_and_search_export_exact_integer_values_and_relative_time(self):
        for motion in ('long_r180', 'search_turn_left'):
            with self.subTest(motion=motion), tempfile.TemporaryDirectory() as name:
                folder = Path(name)
                shell = self.shell()
                result = capture.save_retained_log(shell, folder, full_settings(), self.capability, motion, Mock())
                rows = capture.read_rows(folder / result['csv'])
                self.assertEqual(result['status'], 'saved')
                self.assertEqual((result['period_ms'], result['frame_count'], result['capacity']), (1, 2, 400))
                self.assertEqual([row['cnt'] for row in rows], ['0', '1'])
                self.assertEqual([int(rows[0][key]) for key in ('sen_l.value', 'sen_r.value', 'sen_fl.value',
                    'sen_fr.value', 'ir.range_states', 'motion.pattern')], [-32768, -7, 2049, 32767, 2209, 7])
                self.assertNotIn('time_ms', rows[0])
                sidecar = json.loads((folder / result['settings_json']).read_text())
                self.assertEqual(sidecar['parameters'], full_settings()['parameters'])
                self.assertIn('no absolute MCU time', sidecar['log']['timebase'])
                commands = b''.join(call.args[0] for call in shell.connection.write.call_args_list)
                self.assertEqual(commands, b'log mode0\rdisp log_bin\r')
                self.assertIn(b'\x5d\xa5', shell.output.getvalue())

    def test_old_firmware_does_not_claim_or_request_a_one_ms_log(self):
        shell = Mock()
        result = capture.save_retained_log(shell, Path('unused'), full_settings(),
            'DEBUG_CAMERA_CAP protocol:1 ttl_ms:15000', 'long_r180', Mock())
        self.assertEqual(result['status'], 'unsupported')
        shell.send.assert_not_called()
        shell.wait_line.assert_not_called()

    def test_invalid_frames_and_missing_header_are_rejected_without_resending(self):
        for options in (dict(indices=(0, 2)), dict(magic=0xA55A), dict(header=False), dict(indices=())):
            with self.subTest(options=options), tempfile.TemporaryDirectory() as name:
                shell = self.shell(**options)
                with self.assertRaises((RuntimeError, ValueError)):
                    capture.save_retained_log(shell, Path(name), full_settings(), self.capability,
                                              'long_r180', Mock())
                commands = b''.join(call.args[0] for call in shell.connection.write.call_args_list)
                self.assertEqual(commands.count(b'disp log_bin'), 1)
                self.assertNotIn(b'exe', commands)

    def test_missing_binary_end_times_out_and_never_reports_a_complete_log(self):
        with tempfile.TemporaryDirectory() as name:
            folder = Path(name)
            shell = self.shell(terminate=False)
            with self.assertRaises(TimeoutError):
                capture.save_retained_log(shell, folder, full_settings(), self.capability,
                                          'long_r180', Mock(), timeout=.02)
            self.assertFalse((folder / 'saved_1ms.settings.json').exists())

    def test_no_log_request_is_sent_without_the_run_done_response(self):
        shell = self.shell()
        shell.wait_line = Mock(side_effect=TimeoutError('missing run done'))
        with self.assertRaises(TimeoutError):
            capture.save_retained_log(shell, Path('unused'), full_settings(), self.capability,
                                      'long_r180', Mock())
        shell.connection.write.assert_not_called()

    def test_exactly_four_hundred_samples_are_allowed_and_overflow_is_rejected(self):
        for count in (400, 401):
            with self.subTest(count=count), tempfile.TemporaryDirectory() as name:
                shell = self.shell(indices=range(count))
                if count == 400:
                    result = capture.save_retained_log(shell, Path(name), full_settings(), self.capability,
                                                      'long_r180', Mock())
                    self.assertEqual(result['frame_count'], 400)
                else:
                    with self.assertRaises(RuntimeError):
                        capture.save_retained_log(shell, Path(name), full_settings(), self.capability,
                                                  'long_r180', Mock())


class OneShotTests(unittest.TestCase):
    def test_search_turn_uses_directional_camera_command_and_never_hand_trigger(self):
        for direction in ('right', 'left'):
            with self.subTest(direction=direction):
                shell = Mock()
                motion = capture.MotionOnce(shell, 'search_turn_'+direction, Mock(), Mock(), token=123)
                motion()
                self.assertEqual([call.args[0] for call in shell.send.call_args_list],
                    ['debug camera arm 123', 'debug search_turn '+direction+' exe_camera 123'])
                with self.assertRaises(RuntimeError):
                    motion()
                self.assertEqual(shell.send.call_count, 2)

    def test_search_turn_requires_explicit_firmware_capability(self):
        old = 'DEBUG_CAMERA_CAP protocol:1 ttl_ms:15000'
        new = old+' search_turn:1'
        self.assertTrue(capture.camera_capability_supported(old, 'long_r180'))
        self.assertTrue(capture.camera_capability_supported(new, 'long_r180'))
        for motion in capture.SEARCH_TURNS:
            self.assertFalse(capture.camera_capability_supported(old, motion))
            self.assertTrue(capture.camera_capability_supported(new, motion))
            self.assertTrue(capture.camera_capability_supported(new+' search_turn_log_hold:1', motion))
            self.assertFalse(capture.camera_capability_supported(new.replace('search_turn:1','search_turn:0'), motion))
        for invalid in ('', 'DEBUG_CAMERA_CAP broken', new.replace('protocol:1','protocol:2')):
            self.assertFalse(capture.camera_capability_supported(invalid, 'search_turn_right'))

    def test_search_preset_query_and_directional_response_preserve_canonical_name(self):
        shell = Mock()
        expected = dict(settings(), motion_name='search_turn_left', preset_speed_mm_s=370,
                        parameters=dict(velo=.37))
        result = capture.read_device_settings(shell, 'search_turn_left', 370, Mock(return_value=expected), 'A')
        self.assertEqual(result['motion_name'], 'search_turn_left')
        command, predicate = shell.query.call_args.args
        self.assertEqual(command, 'debug search_turn left preset 370')
        self.assertTrue(predicate('DEBUG_TURN_PARAM_X1000 left velo:370'))
        self.assertFalse(predicate('DEBUG_TURN_PARAM_X1000 right velo:370'))
        with self.assertRaises(RuntimeError):
            capture.validate_settings(expected, 'search_turn_left', 500)

    def test_camera_owner_id_phase_connection_and_remaining_time_must_match(self):
        device = dict(recording_id="recording", owner="operator", phase="RECORDING", elapsed_seconds=0.2)
        capture.validate_recording(dict(connected=True, device=device), "recording", "operator", 30)
        for key, value in (("recording_id", "old"), ("owner", "other"), ("phase", "PREPARING"),
                           ("phase", "IDLE"), ("elapsed_seconds", 25), ("elapsed_seconds", None),
                           ("elapsed_seconds", float("nan")), ("elapsed_seconds", -1)):
            with self.subTest(key=key, value=value), self.assertRaises(RuntimeError):
                capture.validate_recording(dict(connected=True, device=dict(device, **{key: value})),
                                           "recording", "operator", 30)
        with self.assertRaises(RuntimeError):
            capture.validate_recording(dict(connected=False, device=device), "recording", "operator", 30)

    def test_no_command_until_ready_callback(self):
        shell = Mock()
        capture.MotionOnce(shell, "long_r180", Mock(), Mock())
        shell.send.assert_not_called()
        shell.check_battery.assert_not_called()

    def test_recording_checks_permit_and_executes_once_without_hand_trigger(self):
        shell, event = Mock(), Mock()
        recording_check = Mock()
        motion = capture.MotionOnce(shell, "long_r180", event, recording_check, token=123)
        motion()
        self.assertEqual(shell.method_calls[0][0], "check_battery")
        self.assertEqual([call.args[0] for call in shell.send.call_args_list],
                         ["debug camera arm 123", "debug turn long_r180 exe_camera 123"])
        self.assertTrue(motion.start_confirmed)
        self.assertEqual(recording_check.call_count, 3)
        self.assertTrue(shell.wait_line.call_args_list[0].args[0]("DEBUG_CAMERA_ARMED token:123"))
        self.assertFalse(shell.wait_line.call_args_list[0].args[0]("DEBUG_CAMERA_ARMED token:456"))
        with self.assertRaises(RuntimeError):
            motion()
        self.assertEqual(shell.send.call_count, 2)

    def test_no_recording_never_arms_or_executes(self):
        shell = Mock()
        motion = capture.MotionOnce(shell, "long_r180", Mock(), Mock(side_effect=RuntimeError("not recording")))
        with self.assertRaises(RuntimeError):
            motion()
        shell.send.assert_not_called()

    def test_recording_lost_after_arm_does_not_execute(self):
        shell = Mock()
        motion = capture.MotionOnce(shell, "long_r180", Mock(),
                                    Mock(side_effect=[None, None, RuntimeError("recording ended")]), token=123)
        with self.assertRaises(RuntimeError):
            motion()
        shell.send.assert_called_once_with("debug camera arm 123")
        self.assertFalse(motion.execution_write_attempted)

    def test_low_battery_never_sends_execute(self):
        shell = Mock()
        shell.check_battery.side_effect = RuntimeError("low battery")
        motion = capture.MotionOnce(shell, "long_r180", Mock(), Mock())
        with self.assertRaises(RuntimeError):
            motion()
        with self.assertRaises(RuntimeError):
            motion()
        self.assertFalse(motion.execution_write_attempted)
        shell.send.assert_not_called()

    def test_ambiguous_write_failure_is_never_retried(self):
        shell = Mock()
        shell.send.side_effect = [None, TimeoutError("CR outcome unknown")]
        motion = capture.MotionOnce(shell, "long_r180", Mock(), Mock())
        with self.assertRaises(TimeoutError):
            motion()
        with self.assertRaises(RuntimeError):
            motion()
        self.assertEqual(shell.send.call_count, 2)
        self.assertTrue(motion.execution_write_attempted)

    def test_missing_arm_response_never_executes_or_retries(self):
        shell = Mock()
        shell.wait_line.side_effect = TimeoutError("no arm response")
        motion = capture.MotionOnce(shell, "long_r180", Mock(), Mock(), token=123)
        with self.assertRaises(TimeoutError):
            motion()
        with self.assertRaises(RuntimeError):
            motion()
        shell.send.assert_called_once_with("debug camera arm 123")
        self.assertFalse(motion.start_confirmed)

    def test_rejects_wrong_device_settings(self):
        capture.validate_settings(settings(), "long_r180", 500)
        for key, value in (("preset_speed_mm_s", 700), ("motion_name", "long_l180"),
                           ("turn_count", 2), ("error", "timeout"),
                           ("parameters", {"velo": 0.7}), ("suction", {"enabled": True})):
            with self.subTest(key=key), self.assertRaises(RuntimeError):
                capture.validate_settings(dict(settings(), **{key: value}), "long_r180", 500)

    def test_normal_turn_accepts_explicit_suction_on_with_valid_actual_duty(self):
        actual = dict(settings(), suction=dict(enabled=True, duty=650))
        capture.validate_settings(actual, 'long_r180', 500)
        for duty in (None, 0, -1, 991, True, 650.0):
            with self.subTest(duty=duty), self.assertRaises(RuntimeError):
                capture.validate_settings(dict(actual, suction=dict(enabled=True,duty=duty)), 'long_r180', 500)
        for enabled in (None, 0, 1, 'ON'):
            with self.subTest(enabled=enabled), self.assertRaises(RuntimeError):
                capture.validate_settings(dict(actual,suction=dict(enabled=enabled,duty=650)), 'long_r180', 500)
        with self.assertRaises(RuntimeError):
            capture.validate_settings(dict(actual,turn_count=True), 'long_r180', 500)
        search = dict(actual, motion_name='search_turn_right', preset_speed_mm_s=370, parameters=dict(velo=.37))
        with self.assertRaisesRegex(RuntimeError, '探索ターン'):
            capture.validate_settings(search, 'search_turn_right', 370)


class LogSliceTests(unittest.TestCase):
    def write_rows(self, folder, rows):
        with (folder / "telemetry.csv").open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=rows[0].keys())
            writer.writeheader()
            writer.writerows(rows)

    def test_preserves_actual_mcu_time_and_settings_for_five_ms_log(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            rows = [dict(cnt=i * 5, time_ms=10000 + i * 5, **{
                "ideal.velo": 0.5 if 2 <= i <= 5 else 0,
                "ideal.rad_velo": -8 if 3 <= i <= 4 else 0,
                "lost_frames": 0}) for i in range(8)]
            self.write_rows(folder, rows)
            result = capture.save_motion_slice(folder, settings())
            self.assertTrue(result["full_motion_retained"])
            self.assertEqual(result["motion_start_time_ms"], 10010)
            self.assertEqual(result["motion_end_time_ms"], 10025)
            self.assertEqual(result["turn_rows"], 2)
            self.assertEqual(capture.read_rows(folder / "motion.csv")[3]["time_ms"], "10015")
            self.assertTrue((folder / "motion.settings.json").exists())

    def test_idle_csv_is_not_treated_as_a_completed_motion(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            self.write_rows(folder, [{"cnt": 0, "time_ms": 20, "ideal.velo": 0,
                                      "ideal.rad_velo": 0, "lost_frames": 0}])
            self.assertFalse(capture.save_motion_slice(folder, settings())["full_motion_retained"])
            self.assertFalse((folder / "motion.csv").exists())

    def test_partial_csv_line_is_ignored_while_appending(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "telemetry.csv"
            path.write_text("cnt,time_ms\n0,100\n5,", encoding="utf-8")
            self.assertEqual(capture.read_rows(path), [{"cnt": "0", "time_ms": "100"}])


if __name__ == "__main__":
    unittest.main()
