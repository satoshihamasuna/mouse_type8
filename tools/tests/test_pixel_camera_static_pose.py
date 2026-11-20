"""開始方向の誤差、静止時の測定揺らぎ、誤検出を混同しないことを検証する。"""
from pathlib import Path
import json
import sys
import unittest
from unittest.mock import patch

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'analysis'))
import pixel_camera_static_pose as static


class StaticPoseTests(unittest.TestCase):
    def measure(self, heading, *, target=90., centre=(315., 45.), valid=None, registered=None):
        heading = np.asarray(heading, dtype=float)
        direction = np.column_stack([np.cos(np.radians(heading)), np.sin(np.radians(heading))])
        centre = np.broadcast_to(np.asarray(centre, dtype=float), direction.shape)
        front, rear = centre+27.5*direction, centre-27.5*direction
        pose = pd.DataFrame(dict(blue_x_mm=front[:, 0], blue_y_mm=front[:, 1],
            yellow_x_mm=rear[:, 0], yellow_y_mm=rear[:, 1],
            marker_detected_raw=np.ones(len(heading), bool) if valid is None else valid,
            calibration_valid=np.ones(len(heading), bool) if registered is None else registered))
        pose.attrs['marker_calibration'] = dict(per_frame_registration_required=True, used_marker_ids=[2,3])
        with patch.object(static, 'read_video_pose', return_value=(pose, None, None, None)), \
                patch.object(static, 'board_to_pose_transform', return_value=np.eye(3)):
            return static.measure(Path('unused.mp4'), {}, {'front_colour':'red'},
                dict(x_mm=315., y_mm=45., heading_deg=target,
                     position_tolerance_mm=5., heading_tolerance_deg=2.))

    def test_v4_budget_rejection_is_intentionally_diagnostic_only_in_v5(self):
        result = self.measure(np.r_[np.full(176, 88.3), np.full(22, 87.8)])
        self.assertTrue(result['passed'])
        self.assertGreater(result['max_heading_error_deg'], 2.)
        self.assertLess(result['representative_heading_error_deg'], 2.)
        self.assertNotIn('heading_tolerance_exceeded', result['failure_reasons'])
        self.assertGreater(result['heading_budget_deg'], 2.)
        self.assertTrue(result['heading_budget_is_diagnostic_only'])
        self.assertEqual(result['acceptance_version'], 5)

    def test_sparse_pixel_quantization_noise_within_target_bound_passes(self):
        result = self.measure(np.r_[np.full(98, 90.), 88.7, 91.3, np.full(98, 90.)])
        self.assertTrue(result['passed'])
        self.assertGreater(result['heading_jitter_max_deg'], 1.)
        self.assertLess(result['heading_jitter_rms_deg'], .5)
        self.assertLess(result['max_heading_error_deg'], 2.)
        self.assertFalse(result['heading_jitter_max_is_diagnostic_only'])
        self.assertTrue(result['raw_heading_tolerance_is_diagnostic_only'])
        self.assertEqual(result['heading_target_max_limit_deg'], 2.)
        self.assertEqual(result['heading_stability_limits_deg']['maximum_median_deviation'], 2.)

    def test_v3_single_target_outlier_remains_diagnostic_in_v5(self):
        result = self.measure(np.r_[np.full(197, 91.8), 92.6])
        self.assertTrue(result['passed'])
        self.assertFalse(result['all_raw_heading_observations_within_target_tolerance'])
        self.assertLess(result['representative_heading_error_deg'], 2.)
        self.assertLess(result['heading_jitter_max_deg'], 1.)
        self.assertLess(result['heading_jitter_rms_deg'], .5)
        self.assertLess(result['heading_budget_deg'], 2.)

    def test_unchanged_near_limit_fixture_passes_v5_with_budget_over_two(self):
        result = self.measure(np.r_[np.tile([91.6, 92.0], 98), 91.8, 92.6])
        self.assertTrue(result['passed'])
        self.assertGreater(result['heading_jitter_rms_deg'], .1)
        self.assertLess(result['heading_jitter_rms_deg'], .5)
        self.assertGreater(result['heading_budget_deg'], 2.)
        self.assertFalse(result['all_raw_heading_observations_within_target_tolerance'])
        self.assertTrue(result['heading_budget_is_diagnostic_only'])

    def test_representative_angle_just_outside_two_degrees_is_rejected(self):
        result = self.measure(np.full(198, 92.01))
        self.assertFalse(result['passed'])
        self.assertGreater(result['representative_heading_error_deg'], 2.)
        self.assertIn('heading_tolerance_exceeded', result['failure_reasons'])

    def test_quantization_tail_can_exceed_raw_target_limit_with_stable_budget(self):
        base = np.sqrt((198*.48**2-2*1.29**2)/192)
        heading = 90.95+np.r_[np.tile([-base, base], 48), -1.29, 0., 0.,
                              np.tile([-base, base], 48), 1.29, 0., 0.]
        result = self.measure(heading)
        self.assertTrue(result['passed'])
        self.assertAlmostEqual(result['heading_jitter_rms_deg'], .48, places=4)
        self.assertAlmostEqual(result['max_heading_error_deg'], 2.24, places=4)
        self.assertLess(result['heading_budget_deg'], 2.)
        self.assertFalse(result['all_raw_heading_observations_within_target_tolerance'])
        self.assertFalse(result['heading_budget_is_statistical_guarantee'])

    def test_persistent_wrong_direction_is_rejected(self):
        result = self.measure(np.full(198, 87.8))
        self.assertFalse(result['passed'])
        self.assertIn('heading_tolerance_exceeded', result['failure_reasons'])

    def test_single_three_degree_outlier_is_rejected_by_gross_guard(self):
        result = self.measure(np.r_[np.full(197, 90.), 93.])
        self.assertFalse(result['passed'])
        self.assertIn('heading_gross_outlier_exceeded', result['failure_reasons'])
        self.assertLess(result['heading_jitter_rms_deg'], .5)

    def test_slow_rotation_is_rejected(self):
        result = self.measure(np.linspace(89.25, 90.75, 198))
        self.assertFalse(result['passed'])
        self.assertIn('heading_drift_exceeded', result['failure_reasons'])

    def test_many_small_deviations_are_rejected_by_rms(self):
        result = self.measure(np.tile([89.4, 90.6], 99))
        self.assertFalse(result['passed'])
        self.assertIn('heading_jitter_rms_exceeded', result['failure_reasons'])

    def test_wrapping_at_180_does_not_hide_backwards_placement(self):
        values = np.tile([179.9, -179.9], 99)
        self.assertTrue(self.measure(values, target=180.)['passed'])
        result = self.measure(values, target=0.)
        self.assertFalse(result['passed'])
        self.assertIn('heading_tolerance_exceeded', result['failure_reasons'])

    def test_maximum_position_bound_is_preserved(self):
        centre = np.tile([315.,45.], (198,1)); centre[-1,0] += 5.1
        result = self.measure(np.full(198,90.), centre=centre)
        self.assertFalse(result['passed'])
        self.assertIn('position_tolerance_exceeded', result['failure_reasons'])

    def test_registration_and_observation_fraction_are_preserved(self):
        for field in ('valid','registered'):
            with self.subTest(field=field):
                result = self.measure(np.full(198,90.), **{field:np.arange(198)>=10})
                self.assertFalse(result['passed'])
                self.assertIn('insufficient_observation_fraction', result['failure_reasons'])

    def test_too_few_and_no_observations_fail(self):
        single = self.measure([90.])
        self.assertFalse(single['passed'])
        json.dumps(single, allow_nan=False)
        self.assertFalse(self.measure(np.full(11,90.))['passed'])
        missing = self.measure(np.full(12,90.), valid=np.zeros(12,bool))
        self.assertFalse(missing['passed'])
        self.assertIsNone(missing['max_heading_error_deg'])
        self.assertIsNone(missing['all_raw_heading_observations_within_target_tolerance'])
        json.dumps(missing, allow_nan=False)


if __name__ == '__main__':
    unittest.main()
