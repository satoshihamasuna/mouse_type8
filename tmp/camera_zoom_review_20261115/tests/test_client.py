from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from pixel_camera import PixelCameraClient, PixelCameraError


class ClientRecoveryTests(unittest.TestCase):
    def test_wait_for_transfer_survives_a_temporary_broker_disconnect(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            camera = PixelCameraClient(runtime_dir=root, auto_start=False)
            response = {
                "committed": True, "source_deleted": True, "file_path": str(root / "video.mp4"),
                "metadata_path": str(root / "video.json"), "sha256": "a" * 64, "size": 100,
                "frame_count": 2400, "frame_count_source": "android_encoder_samples",
            }
            with patch.object(camera, "_request", side_effect=[PixelCameraError("CONNECTION_FAILED", "restarting"), response]), \
                    patch("pixel_camera.client.time.sleep"):
                result = camera.wait_for_transfer(recording_id="recording", timeout=2)
            self.assertTrue(result.source_deleted)
            self.assertEqual(2400, result.frame_count)
            self.assertFalse(result.physical_240fps_verified)

    def test_authentication_failure_is_not_hidden_by_retrying(self):
        with tempfile.TemporaryDirectory() as temporary:
            camera = PixelCameraClient(runtime_dir=Path(temporary), auto_start=False)
            with patch.object(camera, "_request", side_effect=PixelCameraError("UNAUTHORIZED", "wrong token")) as request:
                with self.assertRaises(PixelCameraError) as caught:
                    camera.wait_for_transfer(recording_id="recording", timeout=2)
            self.assertEqual("UNAUTHORIZED", caught.exception.code)
            self.assertEqual(1, request.call_count)


if __name__ == "__main__":
    unittest.main()
