from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from pixel_camera import PixelCameraClient, PixelCameraError


class ClientRecoveryTests(unittest.TestCase):
    def camera(self, **kwargs):
        temporary=tempfile.TemporaryDirectory();self.addCleanup(temporary.cleanup)
        return PixelCameraClient(runtime_dir=Path(temporary.name),**kwargs)

    def test_connected_phone_skips_adb_and_wait_on_every_capture(self):
        camera=self.camera()
        with patch.object(camera,'ensure_server'),patch.object(camera,'get_status',return_value={'connected':True}), \
                patch.object(camera,'_usb_reverse') as reverse,patch('pixel_camera.client.time.sleep') as sleep:
            self.assertIs(camera.connect(),camera)
            camera.disconnect()
            self.assertIs(camera.connect(),camera)
        reverse.assert_not_called();sleep.assert_not_called()

    def test_explicit_serial_sets_usb_once_and_reconnects_if_link_is_lost(self):
        camera=self.camera(serial='selected-device')
        with patch.object(camera,'ensure_server'),patch.object(camera,'get_status',return_value={'connected':True}) as status, \
                patch.object(camera,'_usb_reverse') as reverse:
            camera.connect();camera.connect()
            self.assertEqual(reverse.call_count,1)
            status.side_effect=[{'connected':False},{'connected':True}]
            camera.connect()
            self.assertEqual(reverse.call_count,2)

    def test_device_wait_tolerates_only_transient_read_failures(self):
        camera=self.camera()
        with patch.object(camera,'ensure_server'),patch.object(camera,'_usb_reverse'), \
                patch.object(camera,'get_status',side_effect=[{'connected':False},
                    PixelCameraError('CONNECTION_FAILED','temporary'),{'connected':True}]), \
                patch('pixel_camera.client.time.sleep'):
            camera.connect()
        with patch.object(camera,'ensure_server'),patch.object(camera,'get_status',side_effect=PixelCameraError('UNAUTHORIZED','bad token')) as status:
            with self.assertRaises(PixelCameraError):camera.connect()
            status.assert_called_once()

    def test_status_read_uses_short_timeout_without_shortening_mutating_commands(self):
        camera=self.camera()
        with patch.object(camera,'_request',return_value={'connected':True}) as request:
            camera.get_status();self.assertEqual(request.call_args.kwargs['timeout'],2)
            camera.get_status(timeout=.2);self.assertEqual(request.call_args.kwargs['timeout'],.2)
            request.return_value={'ok':True,'result':{}}
            camera.configure();self.assertNotIn('timeout',request.call_args.kwargs)

    def test_server_auth_error_is_not_retried_as_startup_delay(self):
        camera=self.camera()
        with patch.object(camera,'_request',side_effect=[PixelCameraError('CONNECTION_FAILED','down'),
                PixelCameraError('UNAUTHORIZED','bad token')]) as request,patch('pixel_camera.client.subprocess.Popen'):
            with self.assertRaises(PixelCameraError) as caught:camera.ensure_server()
            self.assertEqual(caught.exception.code,'UNAUTHORIZED')
            self.assertEqual(request.call_count,2)

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
