"""Geometry and isolation checks for mapping two-state outputs into a path."""
import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"analysis"))
from compare_ekf_paths import LocalPosePath


def rows(count=50, gyro_yaw=0., local_e=0., psi=0.):
    for i in range(count):
        yield {"sequence": i, "time_ms": i*5, "dt_ms": 5., "raw.east_mm": -2*i*math.sin(gyro_yaw),
               "raw.north_mm": 2*i*math.cos(gyro_yaw), "raw.yaw_rad": gyro_yaw if i else 0.,
               "initial_bias_deg_s": 0., "segment": 0, "local_active": True, "local_observed": True,
               "local.e_mm": local_e, "local.psi_deg": psi, "local.std_psi_deg": 1.}


class LocalPosePathTests(unittest.TestCase):
    def test_disabled_updates_preserve_raw_path_when_bias_zero(self):
        path = LocalPosePath(use_local=False)
        for row in rows(100, gyro_yaw=.04):
            result = path.update(row)
        np.testing.assert_allclose([result["east_mm"], result["north_mm"]],
                                   [row["raw.east_mm"], row["raw.north_mm"]], atol=1e-10)

    def test_large_initial_lateral_value_does_not_assign_absolute_world_position(self):
        path = LocalPosePath()
        result = [path.update(row) for row in rows(20, local_e=12.)][-1]
        self.assertAlmostEqual(result["east_mm"], 0., places=10)
        self.assertAlmostEqual(result["north_mm"], 38., places=10)

    def test_lateral_increment_has_left_positive_sign(self):
        path = LocalPosePath()
        data = list(rows(3))
        data[2]["local.e_mm"] = 3.
        result = [path.update(row) for row in data][-1]
        self.assertAlmostEqual(result["east_mm"], -3., places=10)

    def test_heading_correction_reduces_synthetic_straight_drift_without_xy_snap(self):
        path = LocalPosePath()
        data = list(rows(100, gyro_yaw=math.radians(3), psi=0.))
        result = [path.update(row) for row in data][-1]
        self.assertLess(abs(result["east_mm"]), abs(data[-1]["raw.east_mm"])/10)
        self.assertNotEqual(result["east_mm"], 0.)

    def test_unobserved_or_uncertain_local_states_are_not_applied(self):
        for key, value in [("local_observed", False), ("local.std_psi_deg", 8.), ("local_active", False)]:
            path = LocalPosePath()
            data = list(rows(30, gyro_yaw=.05))
            for row in data:
                row[key] = value
                result = path.update(row)
            self.assertEqual(path.used_samples, 0)
            self.assertAlmostEqual(result["east_mm"], data[-1]["raw.east_mm"], places=10)

    def test_segment_change_reanchors_relative_lateral_state(self):
        path = LocalPosePath()
        data = list(rows(4))
        data[3].update(segment=1, **{"local.e_mm": 15.})
        result = [path.update(row) for row in data][-1]
        self.assertAlmostEqual(result["east_mm"], 0., places=10)

    def test_four_state_estimates_cannot_influence_two_state_path(self):
        data = list(rows())
        first = LocalPosePath(); second = LocalPosePath()
        for row in data:
            a = first.update(row)
            b = second.update({**row, "global.east_mm": 1e8, "global.yaw_rad": -1000.})
            self.assertEqual(a, b)

    def test_future_samples_cannot_change_previous_results(self):
        data = list(rows(50))
        first = LocalPosePath(); second = LocalPosePath()
        prefix = [first.update(row) for row in data[:20]]
        for row in data[20:]: row["local.e_mm"] = 10.
        full = [second.update(row) for row in data]
        self.assertEqual(prefix, full[:20])

    def test_discontinuous_samples_fail(self):
        path = LocalPosePath()
        data = list(rows(3)); data[1]["sequence"] = 4
        path.update(data[0])
        with self.assertRaisesRegex(ValueError, "Discontinuous"): path.update(data[1])

    def test_left_positive_increment_after_a_west_facing_turn_points_south(self):
        path = LocalPosePath()
        data = list(rows(8, gyro_yaw=math.pi/2))
        for row in data[:5]: row["local_active"] = False
        for row in data[5:]: row["segment"] = 1
        data[-1]["local.e_mm"] = 3.
        result = [path.update(row) for row in data][-1]
        self.assertAlmostEqual(result["north_mm"], -3., places=10)

    def test_fixed_bias_rotation_matches_independent_integrated_solution(self):
        path = LocalPosePath(use_local=False)
        count, ds, dt, yaw_step, bias = 80, .002, .005, .01, .005
        data = list(rows(count))
        raw_midpoint = (np.arange(1, count)-.5)*yaw_step
        east = np.r_[0., np.cumsum(-ds*np.sin(raw_midpoint))]*1000
        north = np.r_[0., np.cumsum(ds*np.cos(raw_midpoint))]*1000
        for i, row in enumerate(data):
            row.update({"raw.east_mm": east[i], "raw.north_mm": north[i],
                        "raw.yaw_rad": i*yaw_step, "initial_bias_deg_s": math.degrees(bias)})
            result = path.update(row)
        corrected_midpoint = (np.arange(1, count)-.5)*(yaw_step-bias*dt)
        expected = [-ds*np.sin(corrected_midpoint).sum()*1000, ds*np.cos(corrected_midpoint).sum()*1000]
        np.testing.assert_allclose([result["east_mm"], result["north_mm"]], expected, atol=1e-10)


if __name__ == "__main__":
    unittest.main()
