"""格子校正の未使用点検査と、品質不足時に変換を返さないことを確認する。"""
from copy import deepcopy
from pathlib import Path
import sys
import unittest

import cv2
import numpy as np

sys.path[:0] = [str(Path(__file__).resolve().parents[1]), str(Path(__file__).resolve().parents[1]/'analysis')]
from pixel_camera_calibrate import calibration_from_samples, training_location
from pixel_camera_reference import refine_board_from_checkpoints, split_grid_checkpoints


class GridCalibrationTests(unittest.TestCase):
    def scene(self):
        matrix = np.float64([[1.2, .04, 100], [.01, 1.15, 80], [.0001, -.0001, 1]])
        markers = {'actual': dict(ids=[5, 6, 7], corners_px=[
            [[500, 500], [520, 500], [520, 520], [500, 520]],
            [[500, 50], [520, 50], [520, 70], [500, 70]],
            [[50, 500], [70, 500], [70, 520], [50, 520]]])}
        seed = dict(image_size=[1280, 720], markers_px={str(key): value for key, value in
            zip(markers['actual']['ids'], markers['actual']['corners_px'])},
            board_corner_ids=dict(TL=4, TR=6, BR=5, BL=7))
        points = []
        for row in range(1, 8):
            for col in range(1, 8):
                x, y = cv2.perspectiveTransform(np.float64([[[45*col, 45*row]]]), matrix).reshape(2)
                points.append(dict(col=col, row=row, accepted=True, image_x_px=x, image_y_px=y,
                                   expected_x_mm=45*col, expected_y_mm=45*row))
        samples = [dict(sample=i, pts=1+i*.15, checks=deepcopy(points), markers=deepcopy(markers)) for i in range(12)]
        return samples, seed

    def test_calibration_leaves_33_independent_grid_positions(self):
        samples, seed = self.scene()
        profile, training, checks, report = calibration_from_samples(samples, seed, 'synthetic')
        self.assertEqual(len(training), 16)
        self.assertEqual(report['heldout']['unique_points'], 33)
        self.assertTrue(all(not training_location(p['col'], p['row']) for p in checks))
        self.assertLess(report['heldout']['position_rmse_mm'], .001)
        self.assertFalse(profile['metric_accuracy_verified'])

    def test_wrong_heldout_points_are_rejected_instead_of_fitted_away(self):
        samples, seed = self.scene()
        for sample in samples[6:]:
            for point in sample['checks']:
                if not training_location(point['col'], point['row']):
                    point['image_x_px'] += 8
        _, _, checks, report = calibration_from_samples(samples, seed, 'synthetic')
        self.assertFalse(checks)
        self.assertTrue(all(not frame['accepted'] for frame in report['validation_frames']))

    def test_insufficient_grid_or_marker_support_does_not_reuse_old_transform(self):
        samples, seed = self.scene()
        profile, _, _, _ = calibration_from_samples(samples, seed, 'synthetic')
        coarse = dict(available=True, used_marker_ids=[5, 6])
        result = refine_board_from_checkpoints(coarse, samples[0]['checks'][:7], profile)
        self.assertFalse(result['available'])
        self.assertNotIn('homography_image_to_board_mm', result)
        self.assertFalse(refine_board_from_checkpoints(dict(available=False, reason='one_marker'),
                                                      samples[0]['checks'], profile)['available'])

    def test_visible_spread_recovers_occluded_fixed_training_without_using_heldout(self):
        samples, seed = self.scene()
        profile, _, _, _ = calibration_from_samples(samples, seed, 'synthetic')
        profile['grid_refinement']['training_policy'] = 'visible_spread'
        points = samples[0]['checks']
        for point in points:
            if training_location(point['col'], point['row']) and point['row'] in (3, 5, 7):
                point['accepted'] = False
        result = refine_board_from_checkpoints(dict(available=True, used_marker_ids=[5, 7]), points, profile)
        self.assertTrue(result['available'], result)
        quality = result['grid_quality']
        self.assertGreaterEqual(quality['heldout_points'], 12)
        self.assertLess(quality['position_rmse_mm'], .001)
        train = {tuple(p) for p in quality['training_grid_indices']}
        check = {tuple(p) for p in quality['heldout_grid_indices']}
        self.assertFalse(train & check)
        self.assertTrue(all(point['accepted'] for point in points if (point['col'],point['row']) in train|check))

    def test_visible_split_does_not_choose_points_by_image_error_or_hide_bad_checks(self):
        samples, seed = self.scene()
        profile, _, _, _ = calibration_from_samples(samples, seed, 'synthetic')
        config = profile['grid_refinement']
        config['training_policy'] = 'visible_spread'
        points = samples[0]['checks']
        train, heldout = split_grid_checkpoints(points, config)
        ids = [(p['col'],p['row']) for p in train]
        for point in heldout:
            point['image_x_px'] += 8
        shuffled = list(reversed(points))
        self.assertEqual([(p['col'],p['row']) for p in split_grid_checkpoints(shuffled, config)[0]], ids)
        result = refine_board_from_checkpoints(dict(available=True, used_marker_ids=[5, 7]), shuffled, profile)
        self.assertFalse(result['available'])
        self.assertEqual(result['reason'], 'independent_grid_accuracy_limit')
        sparse = refine_board_from_checkpoints(dict(available=True, used_marker_ids=[5, 7]), points[:19], profile)
        self.assertFalse(sparse['available'])
        self.assertNotIn('homography_image_to_board_mm', sparse)


if __name__ == '__main__':
    unittest.main()
