"""Board calibration checks use generated ArUco images; no camera is opened."""
from pathlib import Path
import sys
import unittest
import tempfile

try:
    import cv2
    import numpy as np
    if not hasattr(cv2, "aruco") or not hasattr(cv2.aruco, "ArucoDetector"):
        raise ImportError("OpenCV with ArucoDetector is required")
except ImportError as error:
    raise unittest.SkipTest(str(error))

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "analysis"))
import video_marker_pose as pose
import annotate_profile_points as annotate
import pandas as pd
import json
import lzero_turn_lengths as lengths


class CalibrationTests(unittest.TestCase):
    def board(self, rotations=(0, 0, 0, 0), missing=None):
        image = np.full((600, 600), 255, dtype=np.uint8)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        for marker_id, (x, y), rotation in zip(
                (5, 7, 6, 4), ((50, 50), (500, 50), (50, 500), (500, 500)), rotations):
            if marker_id != missing:
                marker = cv2.aruco.generateImageMarker(dictionary, marker_id, 50)
                image[y:y + 50, x:x + 50] = np.rot90(marker, rotation)
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

    def test_standard_board_uses_the_four_inner_vertices(self):
        quad = pose.detect_aruco_quad(self.board())
        np.testing.assert_allclose(quad, [[99, 99], [500, 99], [99, 500], [500, 500]], atol=1)

    def test_rotating_printed_markers_does_not_change_board_calibration(self):
        expected = pose.detect_aruco_quad(self.board())
        for rotations in ((0, 0, 2, 0), (1, 2, 3, 1), (3, 3, 3, 3)):
            with self.subTest(rotations=rotations):
                actual = pose.detect_aruco_quad(self.board(rotations))
                np.testing.assert_allclose(actual, expected, atol=1)

    def test_missing_corner_is_not_used_as_a_complete_calibration(self):
        self.assertIsNone(pose.detect_aruco_quad(self.board(missing=6)))

    def test_overlay_annotation_uses_the_same_rotated_inner_vertices(self):
        image = self.board((1, 2, 3, 1))
        np.testing.assert_allclose(annotate.complete_quad(annotate.marker_points(image)),
                                   pose.detect_aruco_quad(image), atol=1)


class LogTimingTests(unittest.TestCase):
    def test_recorded_milliseconds_override_row_count_and_normalize_crop(self):
        raw = pd.DataFrame({"time_ms": [91000, 91005, 91011, 91016],
                            "cnt": [8000, 8001, 8002, 8003]})
        np.testing.assert_allclose(pose.log_elapsed_seconds(raw), [0, .005, .011, .016])

    def test_legacy_cnt_milliseconds_and_tick_wrap(self):
        np.testing.assert_allclose(pose.log_elapsed_seconds(pd.DataFrame({"cnt": [15, 16, 17]})),
                                   [0, .001, .002])
        np.testing.assert_allclose(pose.log_elapsed_seconds(pd.DataFrame({
            "time_ms": [2 ** 32 - 5, 0, 6]})), [0, .005, .011])

    def test_invalid_timestamps_are_rejected(self):
        for values in ([1, 1], [1, 0], [0, np.nan], []):
            with self.subTest(values=values), self.assertRaises(ValueError):
                pose.log_elapsed_seconds(pd.DataFrame({"time_ms": values}))

    def test_integration_uses_irregular_sample_times(self):
        raw = pd.DataFrame({"time_ms": [1000, 1005, 1011, 1016],
                            "ego.velo": [.5] * 4, "ego.rad_velo": [0.] * 4,
                            "ideal.rad_velo": [0.] * 4})
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "irregular.csv"
            raw.to_csv(path, index=False)
            integrated = pose.integrate_log(path)
        self.assertAlmostEqual(integrated.y_local_mm.iloc[-1], 8.0)
        np.testing.assert_allclose(integrated.time_log_s, [0, .005, .011, .016])

    def test_phase_metrics_keep_recorded_times_for_video_mapping(self):
        raw = pd.DataFrame({"time_ms": [1000, 1005, 1011, 1016, 1021, 1026, 1032, 1037, 1042],
                            "ideal.length": [50, 60, 0, 10, 20, 0, 10, 20, 0],
                            "ego.length": [50, 60, 0, 10, 20, 0, 10, 20, 0],
                            "ideal.rad_velo": [0, 0, 0, 2, 2, 0, 0, 0, 0]})
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "phases.csv"
            raw.to_csv(path, index=False)
            metrics = pose.measure_log_length_phases(path)
        self.assertAlmostEqual(pose.profile_log_time_s(metrics, "start"), .016)
        self.assertAlmostEqual(pose.profile_log_time_s(metrics, "end"), .021)
        self.assertAlmostEqual(pose.profile_log_time_s({"log_omega_start_frame": 16}, "start"), .016)

    def test_profile_estimator_maps_recorded_times_instead_of_row_indexes(self):
        time = np.linspace(0, 1, 201)
        video = pd.DataFrame({"time_video_s": time, "center_x_mm": 500 * time,
                              "center_y_mm": 100 * time, "heading_rad": time * np.pi / 2})
        metrics = {"log_omega_start_frame": 3, "log_omega_end_frame": 4,
                   "log_omega_start_time_s": .016, "log_omega_end_time_s": .021}
        result = pose.estimate_profile_matched_lengths(video, metrics, 1, .25, 200)
        self.assertAlmostEqual(result["profile_video_start_s"], .266)
        self.assertAlmostEqual(result["profile_video_end_s"], .271)

    def test_lzero_solver_cuts_video_at_recorded_profile_times(self):
        time = np.linspace(0, 1, 201)
        video = pd.DataFrame({"time_video_s": time, "center_x_mm": 500 * time,
                              "center_y_mm": 0., "heading_rad": 0.})
        metrics = {"motion_name": "long_l180", "log_turn_angle_deg": 180,
                   "log_time_zero_at_video_s": 0., "slow_motion_factor": 1.,
                   "log_omega_start_frame": 3, "log_omega_end_frame": 4,
                   "log_omega_start_time_s": .016, "log_omega_end_time_s": .021}
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "metrics.json"
            path.write_text(json.dumps(metrics), encoding="utf-8")
            video.to_csv(path.with_name("video_marker_pose.csv"), index=False)
            result = lengths.video_result(path, 90, 0)
        self.assertAlmostEqual(result["video_body_forward_mm"], 2.5)


if __name__ == "__main__":
    unittest.main()
