"""BLE・カメラ・Tk画面を開かずに撮影連動の一回実行を検証する。"""
import csv
import copy
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock

TOOLS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))
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


class OneShotTests(unittest.TestCase):
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
