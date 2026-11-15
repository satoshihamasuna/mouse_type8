"""速度転用モデルの物理的な不変性・単位・パラメータ解の検証。"""
import copy
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "analysis"))
import turn_speed_prediction as prediction


def setting(motion="long_r90"):
    is180 = motion.endswith("180")
    return dict(motion_name=motion, parameters=dict(velo=.5, r_min=-45. if is180 else -37.5,
        lstart=23.63 if is180 else 44., lend=16.3 if is180 else 35., degree=-180. if is180 else -90.,
        correction=0., pre_accel=6.5), pid={}, feedforward={}, suction=dict(enabled=False))


def model():
    return dict(clock_ratio=1.0175, tau_v_s=.003, tau_w_s=.002,
                spatial=[.99, .985, .004, .025, .003])


class TurnSpeedPredictionTest(unittest.TestCase):
    def test_yaw_integral_units_sign_and_logical_angle(self):
        s = setting()
        s["parameters"]["correction"] = .5
        time, _, omega, _ = prediction.commands(s, 1.0175)
        actual = np.sum((omega[1:] + omega[:-1]) * np.diff(time) / 2)
        self.assertAlmostEqual(actual, np.deg2rad(89.5) * 1.0175, delta=.0001)
        self.assertEqual(s["parameters"]["degree"], -90.)
        self.assertAlmostEqual(omega.max(), .5 / .0375, delta=.001)

    def test_left_right_are_mirrored(self):
        right = setting()
        left = setting("long_l90")
        left["parameters"]["r_min"] *= -1
        left["parameters"]["degree"] *= -1
        rp, _ = prediction.predict(right, model())
        lp, _ = prediction.predict(left, model())
        np.testing.assert_allclose(rp.right_mm, -lp.right_mm, atol=1e-10)
        np.testing.assert_allclose(rp.forward_mm, lp.forward_mm, atol=1e-10)

    def test_90_transfer_preserves_endpoint_without_mutating_reference(self):
        s, m = setting(), model()
        original = copy.deepcopy(s)
        base, _ = prediction.predict(s, m)
        for speed in (300, 400, 600):
            candidate, pred, solve = prediction.transfer(s, m, speed)
            self.assertTrue(solve["solver_success"])
            np.testing.assert_allclose(pred[["right_mm", "forward_mm"]].iloc[-1],
                                       base[["right_mm", "forward_mm"]].iloc[-1], atol=.002)
            self.assertEqual(candidate["parameters"]["r_min"], original["parameters"]["r_min"])
            self.assertEqual(candidate["parameters"]["degree"], -90.)
            self.assertEqual(candidate["parameters"]["velo"], speed / 1000)
            self.assertGreaterEqual(candidate["parameters"]["lstart"], 0)
            self.assertGreaterEqual(candidate["parameters"]["lend"], 0)
        self.assertEqual(s, original)

    def test_180_transfer_preserves_apex_as_well_as_endpoint(self):
        s, m = setting("long_r180"), model()
        base, _ = prediction.predict(s, m)
        for speed in (300, 400, 600):
            _, pred, solve = prediction.transfer(s, m, speed)
            self.assertTrue(solve["solver_success"])
            np.testing.assert_allclose(prediction.features(pred, True), prediction.features(base, True), atol=.003)

    def test_velocity_law_sensitivity_and_stopped_tail(self):
        s, m = setting(), model()
        s["parameters"]["velo"] = .3
        fixed_time, _ = prediction.predict(s, m, 0)
        scaled_time, _ = prediction.predict(s, m, 1)
        self.assertGreater(np.linalg.norm(fixed_time[["right_mm", "forward_mm"]].iloc[-1]
                                         - scaled_time[["right_mm", "forward_mm"]].iloc[-1]), .1)
        self.assertLess(float(scaled_time.velocity_m_s.iloc[-1]), 1e-8)
        self.assertTrue(np.isfinite(scaled_time.to_numpy()).all())
        self.assertTrue(scaled_time.time_s.diff().iloc[1:].gt(0).all())


if __name__ == "__main__":
    unittest.main()
