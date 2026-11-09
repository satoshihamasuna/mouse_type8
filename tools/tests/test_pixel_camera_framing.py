"""カメラ・BLEなしで露出補正変換と画像座標の境界を確認する。"""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

SPEC = importlib.util.spec_from_file_location("pixel_camera_framing", Path(__file__).resolve().parents[1] / "pixel_camera_framing.py")
framing = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(framing)


class FramingTests(unittest.TestCase):
    def test_ev_conversion_uses_device_step(self):
        for ev, expected in [(-4, -24), (-1, -6), (0, 0), (.5, 3), (4, 24)]:
            self.assertEqual(framing.ev_to_steps(ev, "1/6", [-24, 24]), expected)
        self.assertEqual(framing.ev_to_steps(.3, "1/3", [-3, 3]), 1)

    def test_invalid_ev_rejected(self):
        for value in ("nan", "inf", "bad", 5):
            with self.subTest(value=value), self.assertRaises(ValueError):
                framing.ev_to_steps(value, "1/6", [-24, 24])

    def test_reversed_drag_and_image_boundaries(self):
        self.assertEqual(framing.clamp_roi([1300, 740, -30, -20], 1280, 720), [0, 0, 1280, 720])
        with self.assertRaises(ValueError):
            framing.clamp_roi([100, 100, 103, 102], 1280, 720)

    def test_ready_message_is_not_backend_success(self):
        output = 'RECORDING_READY: id\nFRAMING_RESULT={"ok":true,"video_file":"movie.mp4"}\n'
        self.assertEqual(framing.backend_result(output)["video_file"], "movie.mp4")
        with self.assertRaises(RuntimeError):
            framing.backend_result("RECORDING_READY: id")
        with self.assertRaisesRegex(RuntimeError, "BUSY"):
            framing.backend_result('FRAMING_RESULT={"ok":false,"error":"BUSY"}')

    def test_marker_coordinates_are_in_full_image(self):
        import cv2
        import numpy as np
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        marker = cv2.aruco.generateImageMarker(dictionary, 6, 80)
        frame = np.full((720, 1280, 3), 255, dtype=np.uint8)
        frame[200:280, 900:980] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
        result = framing.inspect_rois(frame, {"TR": [875, 180, 1000, 300]})["TR"]
        self.assertEqual(result["ids"], [6])
        points = np.array(result["corners_px"])[0]
        np.testing.assert_allclose(points.min(axis=0), [900, 200], atol=1)
        np.testing.assert_allclose(points.max(axis=0), [979, 279], atol=1)

    def test_blank_region_is_not_assigned_an_id(self):
        import numpy as np
        frame = np.full((720, 1280, 3), 255, dtype=np.uint8)
        result = framing.inspect_rois(frame, framing.DEFAULT_ROIS)
        self.assertTrue(all(not item["ids"] and not item["corners_px"] for item in result.values()))

    def test_backend_configures_capture_light_and_disconnects(self):
        client = Mock()
        client.configure.return_value = dict(exposure_compensation_steps=-6, torch=True)
        modules = {
            "pixel_camera_capture": SimpleNamespace(CAMERA_PORT=8765, camera_workspace=lambda: Path("."), record_trial=Mock()),
            "pixel_camera": SimpleNamespace(PixelCameraClient=Mock(return_value=client)),
        }
        with patch.dict(sys.modules, modules):
            result = framing.camera_backend("configure", -6, True)
        self.assertTrue(result["ok"])
        client.configure.assert_called_once_with(exposure_compensation_steps=-6, torch=True)
        client.disconnect.assert_called_once()

    def test_old_app_without_light_reply_is_rejected(self):
        client = Mock()
        client.configure.return_value = dict(exposure_compensation_steps=-6)
        modules = {
            "pixel_camera_capture": SimpleNamespace(CAMERA_PORT=8765, camera_workspace=lambda: Path("."), record_trial=Mock()),
            "pixel_camera": SimpleNamespace(PixelCameraClient=Mock(return_value=client)),
        }
        with patch.dict(sys.modules, modules), self.assertRaisesRegex(RuntimeError, "ライト対応"):
            framing.camera_backend("configure", -6)
        client.disconnect.assert_called_once()


if __name__ == "__main__":
    unittest.main()
