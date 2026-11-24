"""起動前の依存確認を、機体・カメラへの接続なしで検証する。"""
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import pixel_camera_tuning as tuning


class AnalysisPreflightTests(unittest.TestCase):
    def setUp(self):
        self.folder=tempfile.TemporaryDirectory();self.addCleanup(self.folder.cleanup)
        self.executable=Path(self.folder.name)/'python.exe';self.executable.write_bytes(b'test fixture')
        self.addCleanup(patch.stopall)
        patch.object(tuning,'analysis_python',return_value=self.executable).start()
        patch.object(tuning,'_analysis_environment_ready',set()).start()

    def test_successful_check_is_reused_but_replaced_python_is_checked_again(self):
        with patch.object(tuning.subprocess,'run',return_value=Mock(returncode=0)) as run:
            tuning.check_analysis_environment();tuning.check_analysis_environment()
            self.assertEqual(run.call_count,1)
            stat=self.executable.stat()
            import os
            os.utime(self.executable,ns=(stat.st_atime_ns,stat.st_mtime_ns+10_000_000))
            tuning.check_analysis_environment()
            self.assertEqual(run.call_count,2)

    def test_timeout_and_failure_are_not_cached_as_ready(self):
        with patch.object(tuning.subprocess,'run',side_effect=[
                subprocess.TimeoutExpired('preflight',20),Mock(returncode=1),Mock(returncode=0)]) as run:
            for _ in range(2):
                with self.assertRaises(RuntimeError):tuning.check_analysis_environment()
            tuning.check_analysis_environment();tuning.check_analysis_environment()
            self.assertEqual(run.call_count,3)
            self.assertEqual(run.call_args.kwargs['timeout'],20)


class CameraOnlyPreparationTests(unittest.TestCase):
    def setUp(self):
        folder=tempfile.TemporaryDirectory();self.addCleanup(folder.cleanup)
        self.root=Path(folder.name)
        self.camera=Mock()
        self.camera.get_status.return_value=dict(connected=True,device=dict(phase='IDLE'))
        self.factory=Mock(return_value=self.camera)
        self.addCleanup(patch.stopall)
        patch('pixel_camera_capture.camera_workspace',return_value=self.root).start()
        patch('pixel_camera_environment.initialize_camera_environment').start()
        patch('pixel_camera_environment.camera_client_options',return_value={}).start()
        patch.object(tuning,'check_analysis_environment').start()
        patch.dict(sys.modules,{'av':Mock(),'serial':Mock(),
            'pixel_camera':Mock(PixelCameraClient=self.factory)}).start()
        self.spawn=patch.object(tuning.subprocess,'Popen').start()
        self.session=tuning.LiveSession(self.root,self.root/'transport')

    def test_camera_only_preparation_reuses_camera_without_starting_ble_or_csv(self):
        self.session.prepare_camera();self.session.prepare_camera()
        self.factory.assert_called_once_with(connection='wifi',port=8765,auto_start=False)
        self.assertEqual(self.camera.get_status.call_count,2)
        self.assertIs(self.session.camera,self.camera)
        self.assertIsNone(self.session.gateway)
        self.assertIsNone(self.session.connection)
        self.assertFalse(self.session.folder.exists())
        self.spawn.assert_not_called()

    def test_disconnected_or_busy_camera_fails_before_starting_ble(self):
        for connected,phase in ((False,'IDLE'),(True,'TRANSFERRING')):
            with self.subTest(connected=connected,phase=phase):
                self.camera.get_status.return_value=dict(connected=connected,device=dict(phase=phase))
                with self.assertRaises(RuntimeError):self.session.prepare_camera()
        self.spawn.assert_not_called()
        self.assertFalse(self.session.folder.exists())

    def test_record_only_preparation_skips_pose_analysis_dependencies(self):
        tuning.check_analysis_environment.side_effect = RuntimeError('pose packages unavailable')
        self.session.prepare_camera(video_analysis=False)
        tuning.check_analysis_environment.assert_not_called()
        self.camera.get_status.assert_called_once()
        self.spawn.assert_not_called()

    def test_stopped_camera_server_has_actionable_message_without_starting_ble(self):
        class ConnectionFailed(RuntimeError):
            code='CONNECTION_FAILED'
        self.camera.get_status.side_effect=ConnectionFailed('WinError 10061')
        with self.assertRaisesRegex(RuntimeError,'PC側のPixelカメラサーバー（8765番）'):
            self.session.prepare_camera(video_analysis=False)
        self.spawn.assert_not_called()
        self.assertIsNone(self.session.gateway)

    def test_phone_disconnection_is_distinct_from_a_stopped_server(self):
        self.camera.get_status.return_value=dict(connected=False)
        with self.assertRaisesRegex(RuntimeError,'スマホの「PC接続を開始」'):
            self.session.prepare_camera(video_analysis=False)
        self.spawn.assert_not_called()

    def test_invalid_stream_period_never_checks_camera_or_starts_ble(self):
        for period in (2,1001,10.5,True):
            with self.subTest(period=period), self.assertRaises(ValueError):
                tuning.LiveSession(self.root,self.root/'invalid',stream_period_ms=period)
        self.factory.assert_not_called()
        self.spawn.assert_not_called()

    def test_selected_stream_period_reaches_gateway(self):
        session=tuning.LiveSession(self.root,self.root/'ten_ms',stream_period_ms=10)
        def spawn(command,**kwargs):
            kwargs['stdout'].write(b'Ready: socket://127.0.0.1:8766\n')
            kwargs['stdout'].flush()
            return Mock(poll=Mock(return_value=None))
        self.spawn.side_effect=spawn
        with patch.object(session,'prepare_camera'), \
                patch('serial.serial_for_url') as connection, \
                patch.object(tuning,'last_health',return_value=dict(rows=1)):
            session.prepare('test-address')
        self.assertEqual(self.spawn.call_args.args[0][-2:],['--capture-period-ms','10'])
        self.assertIs(session.connection,connection.return_value)

    def test_selected_mouse_name_and_address_reach_gateway_without_camera(self):
        def spawn(command, **kwargs):
            kwargs['stdout'].write(b'Ready: socket://127.0.0.1:8766\n')
            kwargs['stdout'].flush()
            return Mock(poll=Mock(return_value=None))
        self.spawn.side_effect = spawn
        with patch.object(self.session, 'prepare_camera') as camera, \
                patch('serial.serial_for_url'), patch.object(tuning, 'last_health', return_value=dict(rows=1)):
            self.session.prepare('E1:7F:86:21:84:73', device_name='HY0020-MOUSE-B', record_video=False)
        command = self.spawn.call_args.args[0]
        self.assertEqual(command[command.index('--name')+1], 'HY0020-MOUSE-B')
        self.assertEqual(command[command.index('--address')+1], 'E1:7F:86:21:84:73')
        camera.assert_not_called()

    def test_gateway_failure_displays_target_and_original_error_and_closes(self):
        def spawn(command, **kwargs):
            kwargs['stderr'].write(b'RuntimeError: Mouse bridge not found\n')
            kwargs['stderr'].flush()
            return Mock(poll=Mock(return_value=1))
        self.spawn.side_effect = spawn
        with patch.object(self.session, 'close') as close, self.assertRaises(RuntimeError) as error:
            self.session.prepare('E1:7F:86:21:84:73', device_name='HY0020-MOUSE-B', record_video=False)
        self.assertIn('HY0020-MOUSE-B', str(error.exception))
        self.assertIn('E1:7F:86:21:84:73', str(error.exception))
        self.assertIn('Mouse bridge not found', str(error.exception))
        close.assert_called_once()


if __name__=='__main__':unittest.main()
