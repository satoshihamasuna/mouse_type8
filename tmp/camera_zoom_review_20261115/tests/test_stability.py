from dataclasses import replace
from fractions import Fraction
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch

from pixel_camera.models import TransferResult
from pixel_camera.stability import decode_video, evaluate_recording, run_stability, save_report


class StabilityChecksTest(unittest.TestCase):
    def setUp(self):
        self.result = TransferResult("recording", Path("video.mp4"), Path("video.json"), "a" * 64,
                                     100, 2398, "android_encoder_samples", True)
        self.metadata = dict(state="FINALIZED", errors=[], stop_reason="DURATION",
            recording_window=dict(limit_reached=True, origin="first_encoded_frame_pts", written_frames=2398),
            capture_failures=0, buffer_losses=0, eos_received=True, muxer_finalized=True,
            encoded_timing=dict(count=2398), mp4=dict(sample_count=2398),
            video_size_bytes=100, sha256="a" * 64, pc_transfer_result="VERIFIED", thermal_status_after=0)
        self.decoded = dict(decoded_frames=2398, duration_seconds=10.0, pts_span_seconds=9.996,
            fps_from_decoded_pts=239.8, codec="h264", wrong_size_frames=0, gaps_over_1_5_frames=0,
            non_increasing_pts=0, missing_pts=0, corrupt_frames=0, first_pts_seconds=0)

    def errors(self):
        return evaluate_recording(self.result, self.metadata, self.decoded, 10)

    def test_correct_video_with_batched_sensor_metadata_can_pass_file_checks(self):
        self.metadata["sensor_sampling"] = {"mode": "ONE_CALLBACK_PER_BATCH"}
        self.assertEqual([], self.errors())

    def test_old_submission_timer_short_video_fails(self):
        self.decoded["duration_seconds"] = 9.7
        self.decoded["pts_span_seconds"] = 9.696
        self.assertIn("VIDEO_DURATION_MISMATCH", self.errors())
        self.assertIn("PTS_SPAN_MISMATCH", self.errors())

    def test_nominal_240_header_does_not_hide_30_fps_timestamps(self):
        self.decoded["fps_from_decoded_pts"] = 30
        self.assertIn("TARGET_FPS_NOT_OBSERVED", self.errors())

    def test_decode_count_disagrees_even_with_valid_hash(self):
        self.decoded["decoded_frames"] -= 1
        self.assertIn("FRAME_COUNT_MISMATCH", self.errors())

    def test_gap_and_missing_deletion_fail_independently(self):
        self.decoded["gaps_over_1_5_frames"] = 1
        self.result = replace(self.result, source_deleted=False)
        self.assertIn("DECODE_OR_PTS_ERROR", self.errors())
        self.assertIn("SOURCE_DELETION_NOT_CONFIRMED", self.errors())

    def test_manual_stop_does_not_pass_full_duration_trial(self):
        self.metadata["stop_reason"] = "USER_OR_ACTIVITY_STOP"
        self.metadata["recording_window"]["limit_reached"] = False
        self.assertIn("UNEXPECTED_STOP_REASON", self.errors())
        self.assertIn("FIRST_FRAME_DURATION_NOT_CONFIRMED", self.errors())

    def test_reports_escape_errors_and_write_even_with_no_completed_runs(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            save_report(folder, dict(state="FAILED", started_at="now", requested_runs=20, runs=[], error="<failure>"))
            document = (folder / "report.html").read_text(encoding="utf-8")
            self.assertIn("&lt;failure&gt;", document)
            self.assertNotIn("<failure>", document)
            self.assertTrue((folder / "report.json").is_file())
            self.assertTrue((folder / "runs.csv").is_file())

    def test_cancellation_finishes_current_transfer_and_starts_no_next_recording(self):
        cancel = threading.Event()
        with tempfile.TemporaryDirectory() as directory:
            client = Mock(save_dir=Path(directory))
            client.get_capabilities.return_value = {}
            client.get_status.return_value = {"connected": True, "device": {"phase": "IDLE"}}
            client.start_recording.return_value = "current"
            def completed_transfer(**kwargs):
                cancel.set()
                return self.result
            client.wait_for_transfer.side_effect = completed_transfer
            row = dict(passed=True, decoded_frames=2398, fps_from_decoded_pts=239.8,
                       duration_seconds=10, errors=[], source_deleted=True)
            with patch("pixel_camera.stability.inspect_recording", return_value=row):
                path, report = run_stability(client, repetitions=3, report_dir=directory,
                                             cancel=cancel, on_progress=lambda value: None)
            self.assertEqual("CANCELLED", report["state"])
            self.assertEqual(1, report["passed_runs"])
            client.start_recording.assert_called_once()
            client.wait_for_transfer.assert_called_once()
            self.assertTrue(path.exists())

    def test_transfer_failure_preserves_recording_id_in_durable_report(self):
        with tempfile.TemporaryDirectory() as directory:
            client = Mock(save_dir=Path(directory))
            client.get_capabilities.return_value = {}
            client.get_status.return_value = {"connected": True, "device": {"phase": "IDLE"}}
            client.start_recording.return_value = "awaiting-transfer"
            client.wait_for_transfer.side_effect = TimeoutError("disconnected")
            path, report = run_stability(client, repetitions=3, report_dir=directory, on_progress=lambda value: None)
            self.assertEqual("FAILED", report["state"])
            self.assertEqual("awaiting-transfer", report["unfinished_recording_id"])
            client.start_recording.assert_called_once()
            self.assertIn("disconnected", path.with_name("report.json").read_text(encoding="utf-8"))
            client.stop_recording.assert_not_called()

    def test_independent_decoder_counts_real_frames_and_pts(self):
        import av
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "synthetic.mp4"
            with av.open(str(path), "w") as container:
                stream = container.add_stream("libx264", rate=240)
                stream.width, stream.height, stream.pix_fmt = 1280, 720, "yuv420p"
                stream.options = {"preset": "ultrafast"}
                for index in range(24):
                    frame = av.VideoFrame(1280, 720, "yuv420p")
                    for plane in frame.planes:
                        plane.update(bytes([16 + index]) * plane.buffer_size)
                    frame.pts, frame.time_base = index, Fraction(1, 240)
                    for packet in stream.encode(frame):
                        container.mux(packet)
                for packet in stream.encode():
                    container.mux(packet)
            decoded = decode_video(path)
            self.assertEqual(24, decoded["decoded_frames"])
            self.assertEqual(0, decoded["corrupt_frames"])
            self.assertEqual(0, decoded["gaps_over_1_5_frames"])
            self.assertAlmostEqual(240, decoded["fps_from_decoded_pts"], places=2)
            self.assertAlmostEqual(0.1, decoded["duration_seconds"], places=3)


if __name__ == "__main__":
    unittest.main()
