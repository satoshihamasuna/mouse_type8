"""未使用格子点による検査が既知の位置誤差を隠さないことを確認する。"""
from pathlib import Path
import sys
import unittest

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'analysis'))
import pixel_camera_accuracy as accuracy


class AccuracyTests(unittest.TestCase):
    def grid_scene(self):
        source = np.full((721, 721, 3), 20, dtype=np.uint8)
        for location in range(0, 721, 90):
            cv2.line(source, (location, 0), (location, 720), (150, 150, 150), 2)
            cv2.line(source, (0, location), (720, location), (150, 150, 150), 2)
        for row in range(8):
            for col in range(8):
                x, y = col*90, row*90
                cv2.line(source, (x, y), (x+90, y+90), (130, 130, 130), 2)
                cv2.line(source, (x+90, y), (x, y+90), (130, 130, 130), 2)
        for y in range(180, 720, 180):
            for x in range(180, 720, 180):
                cv2.circle(source, (x, y), 4, (0, 0, 0), -1)
        world_to_image = np.float64([[1.5, .08, 70], [.04, 1.45, 50], [.00015, .00008, 1]])
        source_to_image = world_to_image @ np.diag([.5, .5, 1])
        frame = cv2.warpPerspective(source, source_to_image, (800, 720))
        return frame, np.linalg.inv(world_to_image)

    def test_image_detector_recovers_unused_grid_intersections(self):
        frame, true_transform = self.grid_scene()
        checks = accuracy.find_grid_checkpoints(frame, true_transform)
        measured = accuracy.evaluate_checkpoints(checks, true_transform)
        self.assertGreaterEqual(len(measured), 40)
        self.assertLess(np.sqrt(np.mean([p['error_mm']**2 for p in measured])), .5)

    def test_wrong_calibration_is_measured_instead_of_fitted_away(self):
        frame, true_transform = self.grid_scene()
        biased = np.float64([[1, 0, 3], [0, 1, -2], [0, 0, 1]]) @ true_transform
        checks = accuracy.find_grid_checkpoints(frame, biased)
        measured = accuracy.evaluate_checkpoints(checks, biased)
        self.assertGreaterEqual(len(measured), 30)
        self.assertAlmostEqual(np.mean([p['error_x_mm'] for p in measured]), 3, delta=.4)
        self.assertAlmostEqual(np.mean([p['error_y_mm'] for p in measured]), -2, delta=.4)
        self.assertGreater(np.sqrt(np.mean([p['error_mm']**2 for p in measured])), 3)

    def test_empty_image_does_not_pass_with_zero_error(self):
        frame, true_transform = self.grid_scene()
        checks = accuracy.find_grid_checkpoints(np.zeros_like(frame), true_transform)
        self.assertFalse(any(point['accepted'] for point in checks))

    def test_per_frame_training_locations_are_excluded_for_each_transform(self):
        checks = [dict(col=col, row=1, accepted=True) for col in range(1, 5)]
        first = accuracy.heldout_checkpoints(checks, dict(grid_quality=dict(training_grid_indices=[[2,1]])), [(1,1)])
        second = accuracy.heldout_checkpoints(checks, dict(grid_quality=dict(training_grid_indices=[[3,1]])), [(1,1)])
        self.assertEqual([p['col'] for p in first if p['accepted']], [3,4])
        self.assertEqual([p['col'] for p in second if p['accepted']], [2,4])
        self.assertTrue(all(p['accepted'] for p in checks))

    def test_position_bias_and_distance_error_are_separate(self):
        checks = [dict(col=col, row=1, accepted=True, image_x_px=col*45, image_y_px=45,
                       expected_x_mm=col*45, expected_y_mm=45) for col in (1, 2, 3)]
        biased = np.float64([[1, 0, 3], [0, 1, 4], [0, 0, 1]])
        points = accuracy.evaluate_checkpoints(checks, biased)
        distances = accuracy.distance_checks(points, 45)
        report = accuracy.summarize(points, distances)
        self.assertAlmostEqual(report['position_rmse_mm'], 5)
        self.assertAlmostEqual(report['distance_rmse_mm'], 0)


if __name__ == '__main__':
    unittest.main()
