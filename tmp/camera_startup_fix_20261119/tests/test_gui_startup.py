"""実機なし・Qt offscreenで、操作と状態表示の競合を再現する。"""
import os
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock, patch

from PySide6.QtWidgets import QApplication
import shiboken6
from pixel_camera.gui import CameraWindow


class GuiStartupTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app=QApplication.instance() or QApplication([])
        cls.app.setQuitOnLastWindowClosed(False)

    @classmethod
    def tearDownClass(cls):
        cls.app.processEvents()
        shiboken6.delete(cls.app)

    def setUp(self):
        self.folder=tempfile.TemporaryDirectory();self.addCleanup(self.folder.cleanup)
        self.runtime=patch('pixel_camera.gui.DEFAULT_RUNTIME',Path(self.folder.name));self.runtime.start()
        self.addCleanup(self.runtime.stop)
        self.window=CameraWindow();self.window.timer.stop()
        self.window.client=Mock(client_id='ours')
        self.window.server_ready=True

    def tearDown(self):
        self.window.close()
        self.window.executor.shutdown(wait=True,cancel_futures=True)
        self.window.monitor_executor.shutdown(wait=True,cancel_futures=True)
        self.app.processEvents()
        shiboken6.delete(self.window)

    def packet(self,value,revision=None,generation=None):
        return (self.window.generation if generation is None else generation,
                self.window.status_revision if revision is None else revision,value)

    def status(self,phase='IDLE'):
        return dict(connected=True,device=dict(phase=phase,owner='ours',recording_id='clip'))

    def test_old_poll_cannot_enable_a_second_start_or_restore_a_disconnected_client(self):
        old=self.packet(self.status())
        with patch.object(self.window,'submit') as submit:
            self.window.start_recording()
            self.window.finished('poll',old)
            self.assertFalse(self.window.start_button.isEnabled())
            self.assertTrue(self.window.action_busy)
            self.window.start_recording();self.assertEqual(submit.call_count,1)
            self.window.finished('start',self.packet('clip'))
            old=self.packet(self.status('RECORDING'))
            self.window.disconnect_camera()
            self.window.finished('poll',old)
            self.assertFalse(self.window.start_button.isEnabled())
            self.assertFalse(self.window.stop_button.isEnabled())

    def test_capability_reply_does_not_unlock_an_inflight_start(self):
        with patch.object(self.window,'submit'):
            self.window.start_recording()
            self.window.finished('capabilities',self.packet({'device':{'model':'fake'}}))
            self.assertTrue(self.window.action_busy)
            self.assertFalse(self.window.connect_button.isEnabled())

    def test_stop_is_not_repeated_after_a_lost_response_or_stale_recording_status(self):
        self.window.display_status(self.status('RECORDING'))
        with patch.object(self.window,'submit') as submit:
            self.window.stop_recording()
            self.window.failed('stop',self.packet(TimeoutError('lost response')))
            self.window.display_status(self.status('RECORDING'))
            self.assertFalse(self.window.stop_button.isEnabled())
            self.window.stop_recording();self.assertEqual(submit.call_count,1)

    def test_slow_history_never_blocks_recording_status(self):
        release=threading.Event();entered=threading.Event()
        self.addCleanup(release.set)
        def history():
            entered.set();release.wait(2);return []
        self.window.client.history.side_effect=history
        self.window.client.get_status.return_value=self.status('RECORDING')
        self.window.poll()
        self.assertTrue(entered.wait(1))
        deadline=time.monotonic()+1
        while '録画中' not in self.window.status_label.text() and time.monotonic()<deadline:
            self.app.processEvents();time.sleep(.01)
        self.assertIn('録画中',self.window.status_label.text())
        self.assertTrue(self.window.history_polling)
        release.set()


if __name__=='__main__':unittest.main()
