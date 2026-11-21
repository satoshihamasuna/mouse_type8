import io
import json
from pathlib import Path
import subprocess
import sys
import tarfile
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import pull_probe
import verify_recording as verifier


class VerificationTests(unittest.TestCase):
    def test_slow_motion_is_visible_from_timestamps(self):
        stats = verifier.timing([index / 30 for index in range(240)])
        self.assertAlmostEqual(30, stats["fps_from_timestamps"])
        self.assertEqual(239, stats["gaps_over_1_5_frames"])

    def test_metadata_cannot_reference_another_directory(self):
        for name in ("../video.mp4", "..\\video.mp4", "C:video.mp4", ".."):
            with self.assertRaises(ValueError):
                verifier.sibling(Path("example.json"), name)

    def fixture(self, directory):
        video = directory / "test.mp4"
        video.write_bytes(b"mock video, decoder mocked separately")
        sensor = directory / "test.sensor.csv"
        sensor.write_text("sensor_timestamp_ns\n" + "\n".join(str(i * 1_000_000_000 // 240) for i in range(2400)),
                          encoding="utf-8")
        data = {"video_file": video.name, "sensor_csv": sensor.name, "state": "FINALIZED",
                "video_size_bytes": video.stat().st_size, "sha256": verifier.sha256(video),
                "encoded_timing": {"count": 2400}, "control_elapsed_seconds": 10}
        metadata = directory / "test.json"
        metadata.write_text(json.dumps(data), encoding="utf-8")
        decoded = {"streams": [{"width": 1280, "height": 720, "codec_name": "h264", "avg_frame_rate": "240/1"}],
                   "frames": [{"best_effort_timestamp_time": str(i / 240)} for i in range(2400)]}
        return metadata, decoded

    def test_matching_timelines_still_do_not_claim_physical_verification(self):
        with tempfile.TemporaryDirectory() as temp:
            metadata, decoded = self.fixture(Path(temp))
            with patch.object(subprocess, "run", return_value=subprocess.CompletedProcess([], 0, json.dumps(decoded), "")):
                result = verifier.verify(metadata)
            self.assertTrue(result["timing_checks_passed"])
            self.assertFalse(result["physical_240fps_verified"])

    def test_corrupt_transfer_fails_even_if_decoder_claims_240(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            metadata, decoded = self.fixture(directory)
            (directory / "test.mp4").write_bytes(b"corrupt")
            with patch.object(subprocess, "run", return_value=subprocess.CompletedProcess([], 0, json.dumps(decoded), "")):
                result = verifier.verify(metadata)
            self.assertFalse(result["timing_checks_passed"])
            self.assertIn("SIZE_OR_SHA256_MISMATCH", result["errors"])

    def test_eight_frame_callback_batches_are_inconclusive_not_frame_loss(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            metadata, decoded = self.fixture(directory)
            data = json.loads(metadata.read_text(encoding="utf-8"))
            data["high_speed_burst_size"] = 8
            metadata.write_text(json.dumps(data), encoding="utf-8")
            (directory / "test.sensor.csv").write_text(
                "frame_number,sensor_timestamp_ns\n" +
                "\n".join(f"{index},{index * 1_000_000_000 // 240}" for index in range(7, 2400, 8)),
                encoding="utf-8")
            with patch.object(subprocess, "run", return_value=subprocess.CompletedProcess([], 0, json.dumps(decoded), "")):
                result = verifier.verify(metadata)
            self.assertEqual([], result["errors"])
            self.assertEqual(["INCONCLUSIVE_BATCHED_SENSOR_METADATA"], result["warnings"])
            self.assertTrue(result["decoded_timing_consistent"])
            self.assertFalse(result["timing_checks_passed"])
            self.assertFalse(result["physical_240fps_verified"])

    def test_export_preserves_existing_different_video(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            (directory / "test.mp4").write_bytes(b"existing")
            archive_bytes = io.BytesIO()
            with tarfile.open(fileobj=archive_bytes, mode="w") as archive:
                info = tarfile.TarInfo("files/probe/test.mp4")
                info.size = len(b"new")
                archive.addfile(info, io.BytesIO(b"new"))

            def fake_adb(command, stdout, **kwargs):
                self.assertEqual(["tar", "-cf", "-", "files/probe"], command[-4:])
                stdout.write(archive_bytes.getvalue())
                return subprocess.CompletedProcess(command, 0, stderr=b"")

            with patch.object(subprocess, "run", side_effect=fake_adb):
                with self.assertRaises(FileExistsError):
                    pull_probe.pull("adb", directory)
            self.assertEqual(b"existing", (directory / "test.mp4").read_bytes())


if __name__ == "__main__":
    unittest.main()
