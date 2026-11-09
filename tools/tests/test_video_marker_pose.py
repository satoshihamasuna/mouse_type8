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
from pixel_camera_reference import create_marker_reference


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

    def test_registered_calibration_accepts_any_two_real_markers(self):
        from itertools import combinations
        image = self.board()
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        corners, ids, _ = cv2.aruco.ArucoDetector(dictionary).detectMarkers(image)
        quad = pose.detect_aruco_quad(image)
        inspection = dict(corners={'all': dict(ids=ids.ravel().tolist(), corners_px=[q.reshape(4, 2).tolist() for q in corners])},
                          board_reference=dict(available=True, points_px=dict(TL=quad[0].tolist(), TR=quad[1].tolist(),
                                                                            BL=quad[2].tolist(), BR=quad[3].tolist())))
        profile = create_marker_reference(inspection, image_size=(600, 600), board_corner_ids=dict(TL=5, TR=7, BL=6, BR=4))
        positions = dict(zip((5, 7, 6, 4), ((50, 50), (500, 50), (50, 500), (500, 500))))
        for pair in combinations(positions, 2):
            with self.subTest(pair=pair):
                partial = image.copy()
                for marker_id, (x, y) in positions.items():
                    if marker_id not in pair:
                        partial[y:y+50, x:x+50] = 255
                np.testing.assert_allclose(pose.detect_aruco_quad(partial, profile), quad, atol=1)

    def test_video_pipeline_records_two_marker_calibration_separately_from_four_detection(self):
        image = self.board()
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        corners, ids, _ = cv2.aruco.ArucoDetector(dictionary).detectMarkers(image)
        quad = pose.detect_aruco_quad(image)
        profile = create_marker_reference(dict(
            corners={'all': dict(ids=ids.ravel().tolist(), corners_px=[q.reshape(4, 2).tolist() for q in corners])},
            board_reference=dict(available=True, points_px=dict(TL=quad[0].tolist(), TR=quad[1].tolist(),
                                                               BL=quad[2].tolist(), BR=quad[3].tolist()))),
            image_size=(600, 600), board_corner_ids=dict(TL=5, TR=7, BL=6, BR=4))
        image[500:550, 50:100] = 255
        image[500:550, 500:550] = 255
        cv2.circle(image, (300, 270), 10, (0, 0, 255), -1)
        cv2.circle(image, (300, 330), 10, (0, 255, 255), -1)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'two_markers.avi'
            writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*'MJPG'), 240, (600, 600))
            self.assertTrue(writer.isOpened(), 'MJPG video writer unavailable')
            for _ in range(12):
                writer.write(image)
            writer.release()
            result, actual_quad, _, complete_rate = pose.read_video_pose(path, 'red', marker_reference=profile)
            window = list(pose.iter_video_frames(path, True, (.012, .035)))
            self.assertEqual([index for index, _, _ in window], list(range(3, 9)))
            self.assertAlmostEqual(window[0][2], 3/240)
            sampled, _, _, _ = pose.read_video_pose(path, 'red', marker_reference=profile, frame_stride=4)
            self.assertEqual(sampled.frame.tolist(), [0,4,8])
            np.testing.assert_allclose(sampled.time_video_s, [0,4/240,8/240], atol=1e-7)
            self.assertTrue(sampled.marker_detected_raw.all())
        self.assertEqual(len(result), 12)
        self.assertEqual(complete_rate, 0)
        self.assertEqual(result.attrs['marker_calibration']['calibration_frame_rate'], 1)
        self.assertEqual(result.attrs['marker_calibration']['used_marker_ids'], [5, 7])
        self.assertTrue(result.marker_detected_raw.all())
        np.testing.assert_allclose(actual_quad, quad, atol=2)

    def test_height_correction_respects_reflected_pose_axes(self):
        profile = dict(board_corner_ids=dict(TL=4, TR=6, BR=5, BL=7))
        axes = pose.board_to_pose_transform(profile)
        geometry = dict(camera_center_board_mm=[140, 160, 600], front_height_mm=10,
                        rear_height_mm=1, axle_offset_from_marker_midpoint_mm=None)
        front, rear = np.array([240., 270.]), np.array([240., 215.])
        floor_front = np.array([140, 160])+(front-[140, 160])*600/590
        floor_rear = np.array([140, 160])+(rear-[140, 160])*600/599
        actual = pose.select_marker_pair([(floor_front, 200)], [(floor_rear, 200)], axes, None, geometry, axes)
        np.testing.assert_allclose(actual, pose.transform_points([front, rear], axes), atol=.001)

    def test_heights_without_camera_geometry_fail_before_video_is_opened(self):
        with self.assertRaisesRegex(ValueError, 'カメラ'):
            pose.read_video_pose(Path('does_not_exist.mp4'), body_geometry=dict(front_height_mm=10, rear_height_mm=1))

    def test_measured_nominal_span_selects_pair_without_forcing_image_scale(self):
        identity = np.eye(3)
        geometry = dict(camera_center_board_mm=[180, 180, 800], front_height_mm=0, rear_height_mm=0,
                        nominal_marker_span_mm=50, axle_offset_from_marker_midpoint_mm=[0, 0])
        candidates = [(np.array([200., 225.]), 200), (np.array([200., 230.]), 200)]
        rear = [(np.array([200., 175.]), 200)]
        chosen = pose.select_marker_pair(candidates, rear, identity, None, geometry, identity)
        np.testing.assert_allclose(chosen[0], [200, 225])
        actual = pose.select_marker_pair([(np.array([200., 226.]), 200)], rear, identity, None, geometry, identity)
        self.assertAlmostEqual(np.linalg.norm(actual[0]-actual[1]), 51)

    def test_explicit_colour_profile_recovers_washed_markers_without_detecting_gray(self):
        hsv = np.zeros((720, 1280, 3), np.uint8)
        hsv[:, :, 2] = 230
        cv2.circle(hsv, (600, 300), 8, (5, 85, 250), -1)
        cv2.circle(hsv, (600, 360), 8, (30, 100, 250), -1)
        image = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        profile = dict(red=dict(minimum_saturation=60, minimum_value=60),
                       yellow=dict(minimum_saturation=70, minimum_value=75))
        self.assertFalse(pose.colour_components(image, 'red'))
        self.assertFalse(pose.colour_components(image, 'yellow'))
        red = pose.colour_components(image, 'red', profile)
        yellow = pose.colour_components(image, 'yellow', profile)
        self.assertEqual(len(red), 1)
        self.assertEqual(len(yellow), 1)
        np.testing.assert_allclose(red[0][0], [600, 300], atol=.1)
        np.testing.assert_allclose(yellow[0][0], [600, 360], atol=.1)


class LogTimingTests(unittest.TestCase):
    def test_required_motion_interval_prevents_alignment_to_only_stationary_frames(self):
        time = np.arange(0, 2.001, .005)
        moving = np.clip(time-.5, 0, 1)*400
        log = pd.DataFrame(dict(time_log_s=time, x_local_mm=moving, y_local_mm=np.zeros(len(time)),
                                speed_log_mm_s=np.where((time >= .5)&(time <= 1.5), 400, 0)))
        video = pd.DataFrame(dict(frame=np.arange(len(time)), time_video_s=time+6.5,
            center_x_mm=moving+100, center_y_mm=np.full(len(time), 200), heading_rad=np.zeros(len(time)),
            speed_playback_mm_s=log.speed_log_mm_s, turn_led_blue_pixels=np.zeros(len(time)),
            turn_led_active=np.zeros(len(time), bool), marker_detected_raw=np.ones(len(time), bool)))
        comparison, _, offset, _, _, error = pose.fit_time_and_pose(video, log, 1, (.3, 1.6))
        self.assertAlmostEqual(offset, 6.5, delta=.003)
        self.assertLess(error, .5)
        self.assertGreater(comparison.time_log_s.max(), 1.6)
        self.assertLess(comparison.time_log_s.min(), .3)

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
