import importlib.util
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from pixel_camera.models import TransferResult

source = Path(os.environ.get("PIXEL_CAMERA_TYPE8_TEST_SOURCE",
    Path(__file__).resolve().parents[1] / "integrations/type8/pixel_camera_capture.py"))
spec = importlib.util.spec_from_file_location("type8_capture", source)
bridge = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bridge)


class Type8CaptureTests(unittest.TestCase):
    def camera(self, folder):
        client = Mock(client_id="operator")
        client.start_recording.return_value = "recording"
        client.get_status.side_effect = [
            {"connected": True, "device": {"recording_id": "recording", "owner": "operator", "phase": "PREPARING"}},
            {"connected": True, "device": {"recording_id": "recording", "owner": "operator",
                                          "phase": "RECORDING", "elapsed_seconds": 0.25}},
        ]
        client.wait_for_transfer.return_value = TransferResult("recording", folder / "camera.mp4",
            folder / "camera.json", "a" * 64, 100, 2398, "android_encoder_samples", True)
        return client

    def test_waits_for_real_recording_before_ready_callback_and_records_source_identity(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            client = self.camera(folder)
            ready = Mock()
            with patch("pixel_camera.stability.inspect_recording", return_value={"passed": True}), \
                    patch.object(bridge.time, "sleep"), patch("builtins.print"):
                path, value = bridge.record_trial(client, folder, 10, "A", "long_r180", ready)
            self.assertEqual(2, client.get_status.call_count)
            ready.assert_called_once()
            client.start_recording.assert_called_once_with(duration_s=10)
            self.assertEqual("PASSED", value["state"])
            saved = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual("a" * 64, saved["sha256"])
            self.assertEqual("user_supplied_not_device_verified", saved["mouse_identity_source"])
            self.assertFalse(saved["physical_240fps_verified"])
            self.assertIsNone(saved["type8_log_csv"])
            client.disconnect.assert_called_once()

    def test_another_client_owner_never_triggers_motion_callback(self):
        client = Mock(client_id="operator")
        client.get_status.return_value = {"connected": True, "device":
            {"recording_id": "recording", "owner": "someone-else", "phase": "RECORDING"}}
        with self.assertRaisesRegex(RuntimeError, "クライアント"):
            bridge.wait_recording(client, "recording")

    def test_same_recording_finishing_before_readiness_fails(self):
        client = Mock(client_id="operator")
        client.get_status.return_value = {"connected": True, "device":
            {"recording_id": "recording", "owner": "operator", "phase": "FINALIZING"}}
        with self.assertRaisesRegex(RuntimeError, "終了"):
            bridge.wait_recording(client, "recording")

    def test_unrelated_recording_is_never_used_as_readiness(self):
        client = Mock(client_id="operator")
        client.get_status.return_value = {"connected": True, "device":
            {"recording_id": "old", "owner": "operator", "phase": "RECORDING"}}
        with patch.object(bridge.time, "monotonic", side_effect=[0, 0, 15]), patch.object(bridge.time, "sleep"):
            with self.assertRaises(TimeoutError):
                bridge.wait_recording(client, "new")

    def test_readiness_failure_recovers_video_but_marks_trial_failed(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            client = self.camera(folder)
            client.get_status.side_effect = None
            client.get_status.return_value = {"connected": True, "device":
                {"recording_id": "recording", "owner": "operator", "phase": "PREPARING"}}
            ready = Mock()
            with patch.object(bridge, "wait_recording", side_effect=TimeoutError("not ready")), \
                    patch("builtins.print"):
                path, value = bridge.record_trial(client, folder, 10, on_ready=ready)
            ready.assert_not_called()
            client.stop_recording.assert_called_once()
            self.assertEqual("FAILED", value["state"])
            self.assertTrue(value["recovered_source_deleted"])
            self.assertTrue(path.is_file())

    def test_never_retries_an_accepted_start_with_lost_response(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            client = self.camera(folder)
            client.start_recording.side_effect = TimeoutError("acceptance response lost")
            ready = Mock()
            path, value = bridge.record_trial(client, folder, 10, on_ready=ready)
            self.assertEqual("FAILED", value["state"])
            client.start_recording.assert_called_once()
            client.stop_recording.assert_not_called()
            ready.assert_not_called()
            self.assertIn("response lost", path.read_text(encoding="utf-8"))

    def test_project_launcher_can_find_existing_camera_workspace(self):
        workspace = bridge.camera_workspace()
        self.assertTrue((workspace / "python/pixel_camera/client.py").is_file())
        self.assertNotEqual(bridge.CAMERA_PORT, bridge.BLE_PORT)


if __name__ == "__main__":
    unittest.main()
