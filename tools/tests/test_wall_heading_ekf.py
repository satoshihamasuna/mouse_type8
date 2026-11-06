"""Numerical/observability checks for the offline wall-heading experiment."""
import math
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "analysis"))
import wall_heading_ekf as ekf


class WallHeadingEkfTests(unittest.TestCase):
    def test_motion_jacobian_agrees_with_independent_finite_difference(self):
        state = np.array([.3, -.2, .7, .015])
        args = (np.array([-.002, .004]), .65, .03, .006)
        _, analytic = ekf.motion(state, *args)
        numerical = np.zeros((4, 4))
        for column in range(4):
            perturbation = np.zeros(4)
            perturbation[column] = 1e-6
            numerical[:, column] = (ekf.motion(state + perturbation, *args)[0]
                                    - ekf.motion(state - perturbation, *args)[0]) / 2e-6
        np.testing.assert_allclose(analytic, numerical, rtol=1e-6, atol=1e-9)

    def test_zero_bias_without_updates_preserves_observed_path(self):
        filt = ekf.WallHeadingEkf()
        yaw = 0.
        expected = np.zeros(2)
        for index in range(100):
            delta = .02 * math.sin(index / 11)
            step = np.array([-.003 * math.sin(yaw + delta / 2), .003 * math.cos(yaw + delta / 2)])
            expected += step
            filt.predict(step, yaw, delta, .01)
            yaw += delta
        np.testing.assert_allclose(filt.state[:2], expected, atol=1e-12)
        self.assertAlmostEqual(filt.state[2], yaw, places=12)

    def test_straight_motion_with_known_bias_converges_towards_ground_truth(self):
        bias, dt, ds = .01, .02, .005
        filt = ekf.WallHeadingEkf()
        raw_yaw = 0.
        raw_xy = np.zeros(2)
        for index in range(1000):
            delta = bias * dt
            step = np.array([-ds * math.sin(raw_yaw + delta / 2), ds * math.cos(raw_yaw + delta / 2)])
            raw_xy += step
            filt.predict(step, raw_yaw, delta, dt)
            raw_yaw += delta
            if index % 5 == 0:
                filt.observe_heading(0., math.radians(.5) ** 2)
        truth = np.array([0., 1000 * ds])
        self.assertLess(np.linalg.norm(filt.state[:2] - truth), .02)
        self.assertLess(np.linalg.norm(filt.state[:2] - truth), np.linalg.norm(raw_xy - truth) / 5)
        self.assertLess(abs(filt.state[3] - bias), .002)

    def test_wraparound_heading_uses_short_innovation(self):
        filt = ekf.WallHeadingEkf()
        filt.state[2] = math.pi - .01
        result = filt.observe_heading(-math.pi + .01, math.radians(1) ** 2)
        self.assertTrue(result["accepted"])
        self.assertAlmostEqual(result["innovation_rad"], .02, places=12)

    def test_outlier_rejection_preserves_state_and_covariance(self):
        filt = ekf.WallHeadingEkf()
        state, covariance = filt.state.copy(), filt.covariance.copy()
        self.assertFalse(filt.observe_heading(math.pi / 2, .001)["accepted"])
        np.testing.assert_array_equal(filt.state, state)
        np.testing.assert_array_equal(filt.covariance, covariance)

    def test_covariance_stays_symmetric_positive(self):
        filt = ekf.WallHeadingEkf()
        for index in range(400):
            filt.predict(np.array([0., .002]), 0., 0., .005, reset=index % 19 == 0)
            filt.observe_heading(.003 * math.sin(index), math.radians(1.5) ** 2)
            np.testing.assert_allclose(filt.covariance, filt.covariance.T, atol=1e-14)
            self.assertGreaterEqual(np.linalg.eigvalsh(filt.covariance).min(), -1e-12)

    def test_left_and_right_ranges_independently_observe_same_heading(self):
        travel = np.linspace(0, .02, 9)
        angle = math.radians(4)
        left = 45 - travel * math.sin(angle) * 1000
        right = 45 + travel * math.sin(angle) * 1000
        ls, _, _ = ekf.wall_heading_measurement(travel, left, "left")
        rs, _, _ = ekf.wall_heading_measurement(travel, right, "right")
        self.assertAlmostEqual(math.asin(ls), angle, places=12)
        self.assertAlmostEqual(math.asin(rs), angle, places=12)

    def test_heldout_values_cannot_change_filter_state(self):
        count = 50
        odometry = pd.DataFrame({"east_mm": np.zeros(count), "north_mm": np.arange(count) * 3.,
                                 "heading_rad": np.arange(count) * .001,
                                 "estimated_control_interval_ms": np.full(count, 10.),
                                 "interpolated_interval": np.zeros(count, dtype=bool)})
        observations = [{"index": index, "heading_rad": 0., "variance_rad2": math.radians(2) ** 2,
                         "holdout": index % 10 == 0} for index in (5, 10, 15, 20, 25, 30)]
        altered = [{**item, "heading_rad": 2.} if item["holdout"] else item for item in observations]
        first = ekf.replay(odometry, observations, 0.)[0]
        second = ekf.replay(odometry, altered, 0.)[0]
        np.testing.assert_array_equal(first, second)

    def test_spatial_observations_reject_clamped_or_front_blocked_ranges(self):
        count = 60
        travel = np.arange(count) * .003
        angle = math.radians(2)
        data = pd.DataFrame({"sequence": np.arange(count), "ideal.velo": .37, "ego.velo": .37,
                             "ideal.rad_velo": 0., "ego.rad_velo": 0.,
                             "sen_l.distance": 45 - travel * math.sin(angle) * 1000,
                             "sen_r.distance": 45 + travel * math.sin(angle) * 1000,
                             "sen_fl.distance": 125., "sen_fr.distance": 125.})
        odometry = pd.DataFrame({"east_mm": -travel * math.sin(angle) * 1000,
                                 "north_mm": travel * math.cos(angle) * 1000, "heading_rad": angle})
        observations, _ = ekf.build_observations(data, odometry)
        self.assertGreaterEqual(len(observations), 3)
        self.assertGreaterEqual(sum(item["holdout"] for item in observations), 1)
        for item in observations:
            self.assertAlmostEqual(item["heading_rad"], angle, places=10)
        blocked = data.copy()
        blocked["sen_fl.distance"] = 40.
        self.assertEqual(ekf.build_observations(blocked, odometry)[0], [])
        clamped = data.copy()
        clamped[["sen_l.distance", "sen_r.distance"]] = 80.
        self.assertEqual(ekf.build_observations(clamped, odometry)[0], [])

    def test_loader_rejects_incorrect_control_time_units(self):
        data = pd.DataFrame({"sequence": [0, 1], "time_ms": [100, 105],
                             "ideal.velo": .37, "ego.velo": .37, "ideal.rad_velo": 0., "ego.rad_velo": 0.,
                             "sen_l.distance": 45., "sen_r.distance": 45.,
                             "sen_fl.distance": 125., "sen_fr.distance": 125.})
        odometry = pd.DataFrame({"sequence": [0, 1], "mcu_time_ms": [100, 105],
                                 "east_mm": [0., 0.], "north_mm": [0., 2.], "heading_rad": [0., 0.],
                                 "estimated_control_interval_ms": [5000., 5000.], "interpolated_interval": False})
        with tempfile.TemporaryDirectory() as directory:
            log_path, odom_path = Path(directory) / "log.csv", Path(directory) / "odometry.csv"
            data.to_csv(log_path, index=False)
            odometry.to_csv(odom_path, index=False)
            with self.assertRaisesRegex(ValueError, "control interval"):
                ekf.load_inputs(log_path, odom_path)

    def test_invalid_numerical_inputs_are_rejected(self):
        with self.assertRaises(ValueError):
            ekf.Config(holdout_stride=1)
        with self.assertRaises(ValueError):
            ekf.WallHeadingEkf().predict([0, 0], 0, 0, 0)
        with self.assertRaises(ValueError):
            ekf.wall_heading_measurement([0] * 5, [45] * 5, "left")
        with self.assertRaises(ValueError):
            ekf.wall_heading_measurement(range(5), [45, 45, float("nan"), 45, 45], "right")


if __name__ == "__main__":
    unittest.main()
