from fractions import Fraction
import hashlib
import json
import os
from pathlib import Path
import tempfile
import time
import unittest

import av

from pixel_camera.review_media import FrameReader, scan_video


def write_video(path):
    with av.open(str(path), "w") as container:
        stream = container.add_stream("libx264", rate=240)
        stream.width, stream.height, stream.pix_fmt = 96, 64, "yuv420p"
        stream.codec_context.max_b_frames = 3
        stream.codec_context.gop_size = 12
        for index in range(48):
            frame = av.VideoFrame(96, 64, "yuv420p")
            for plane_index, plane in enumerate(frame.planes):
                plane.update(bytes([20 + index * 3 if plane_index == 0 else 128]) * plane.buffer_size)
            frame.pts, frame.time_base = index, Fraction(1, 240)
            for packet in stream.encode(frame):
                container.mux(packet)
        for packet in stream.encode():
            container.mux(packet)


def pixels_digest(rgb, width, height, stride):
    return hashlib.sha256(b"".join(rgb[row * stride:row * stride + width * 3] for row in range(height))).hexdigest()


class ReviewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication(["review-tests", "-platform", "offscreen"])
        cls.temporary = tempfile.TemporaryDirectory()
        cls.path = Path(cls.temporary.name) / "with_b_frames.mp4"
        write_video(cls.path)
        cls.reference = []
        with av.open(str(cls.path)) as container:
            for frame in container.decode(video=0):
                rgb = frame.reformat(format="rgb24")
                cls.reference.append(pixels_digest(bytes(rgb.planes[0]), rgb.width, rgb.height, rgb.planes[0].line_size))

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def test_async_viewer_loads_and_steps_exactly_one_frame(self):
        from PySide6.QtCore import Qt
        from PySide6.QtTest import QTest
        from pixel_camera.review import ReviewWindow
        window = ReviewWindow(self.path)
        window.show()
        def wait_until(condition):
            deadline = time.monotonic() + 5
            while not condition() and time.monotonic() < deadline:
                self.app.processEvents()
                QTest.qWait(5)
            self.assertTrue(condition(), window.status.text())
        try:
            wait_until(lambda: window.index is not None and window.next.isEnabled())
            QTest.mouseClick(window.next, Qt.MouseButton.LeftButton)
            wait_until(lambda: window.position == 1 and window.next.isEnabled())
            self.assertIn("0.004", window.timing.text())
            QTest.mouseClick(window.previous, Qt.MouseButton.LeftButton)
            wait_until(lambda: window.position == 0 and window.next.isEnabled())
            self.assertFalse(window.previous.isEnabled())
        finally:
            window.close()
            window.executor.shutdown(wait=True)

    def test_recording_gui_opens_selected_video_in_separate_review_process(self):
        from unittest.mock import patch
        from pixel_camera.gui import CameraWindow
        window = CameraWindow()
        path = str(self.path.parent / "video with spaces.mp4")
        try:
            window.display_history([dict(recording_id="selected", file_path=path, received_at=0,
                frame_count=48, size=100, source_deleted=True)])
            window.history_table.selectRow(0)
            with patch("pixel_camera.gui.subprocess.Popen") as launch:
                window.review_button.click()
                arguments = launch.call_args.args[0]
                self.assertEqual(["-m", "pixel_camera.review", path], arguments[1:])
        finally:
            window.close()
            window.executor.shutdown(wait=True)

    def test_index_uses_decoded_pts_and_source_hash(self):
        index = scan_video(self.path)
        self.assertEqual(48, index.count)
        self.assertEqual(hashlib.sha256(self.path.read_bytes()).hexdigest(), index.sha256)
        self.assertAlmostEqual(1 / 240, index.relative_seconds(1), places=5)
        self.assertAlmostEqual(1000 / 240, index.interval_ms(1), places=2)
        self.assertFalse(index.evidence()["physical_240fps_verified"])

    def test_random_and_sequential_seeks_return_exact_decoded_pixels(self):
        index = scan_video(self.path)
        with FrameReader(index) as reader:
            for position in (0, 1, 2, 35, 36, 12, 47, 3, 3):
                frame = reader.get(position)
                self.assertEqual(position, frame.index)
                self.assertEqual(self.reference[position],
                                 pixels_digest(frame.rgb, frame.width, frame.height, frame.stride))
            with self.assertRaises(IndexError):
                reader.get(index.count)
            with self.assertRaises(IndexError):
                reader.get(-1)

    def test_modified_source_is_rejected_instead_of_showing_wrong_frame(self):
        path = Path(self.temporary.name) / "modified.mp4"
        path.write_bytes(self.path.read_bytes())
        index = scan_video(path)
        os.utime(path, ns=(path.stat().st_atime_ns, index.modified_ns + 1_000_000_000))
        with self.assertRaisesRegex(ValueError, "変更"):
            FrameReader(index)

    def test_partial_recording_cannot_be_opened(self):
        with self.assertRaisesRegex(ValueError, "未確定"):
            scan_video(Path(self.temporary.name) / "unfinished.partial.mp4")

    def test_sheet_and_frame_export_preserve_indices_pts_and_unverified_status(self):
        from PySide6.QtGui import QImage
        from PySide6.QtWidgets import QApplication
        from pixel_camera.review import export_frames
        self.app = QApplication.instance() or QApplication([])
        index = scan_video(self.path)
        with tempfile.TemporaryDirectory() as directory:
            folder = export_frames(index, 10, count=24, parent=directory, reference_note="Uncalibrated fixture")
            evidence = json.loads((folder / "review.json").read_text(encoding="utf-8"))
            self.assertEqual(list(range(10, 34)), [frame["index"] for frame in evidence["frames"]])
            self.assertEqual(index.sha256, evidence["source_sha256"])
            self.assertFalse(evidence["physical_240fps_verified"])
            self.assertFalse(evidence["reference_independently_verified"])
            self.assertFalse(QImage(str(folder / "contact_sheet.png")).isNull())
            last = export_frames(index, 47, count=24, parent=directory)
            self.assertEqual(1, len(json.loads((last / "review.json").read_text())["frames"]))
            single = export_frames(index, 7, count=1, parent=directory)
            image = QImage(str(single / "frame.png"))
            self.assertEqual((96, 64), (image.width(), image.height()))


if __name__ == "__main__":
    unittest.main()
