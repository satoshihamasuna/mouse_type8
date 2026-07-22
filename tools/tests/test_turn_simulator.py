import math
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np


TOOLS_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS_DIR))

from turn_simulator import (  # noqa: E402
    MOTIONS,
    PRESETS,
    acceleration_table,
    simulate_turn,
    write_csv,
)


class TurnSimulatorTest(unittest.TestCase):
    def test_acceleration_table_matches_matlab_definition(self):
        values, time_ms, integral = acceleration_table()
        self.assertEqual(len(values), 1001)
        np.testing.assert_array_equal(time_ms, np.arange(1001))
        self.assertAlmostEqual(values[0], 0.0)
        self.assertAlmostEqual(values[400], 1.0)
        self.assertAlmostEqual(values[500], 1.0)
        self.assertAlmostEqual(values[600], 1.0)
        self.assertAlmostEqual(values[-1], 0.0)
        self.assertAlmostEqual(integral, 0.7043120379914867, places=14)
        np.testing.assert_allclose(values, values[::-1], atol=1e-14)

    def test_type8i_long_turn90_reference_values(self):
        preset = PRESETS["type8i"]
        result = simulate_turn(
            "long_turn90", preset.velocity, preset.motor,
            preset.radii_mm["long_turn90"], preset.kp, preset.ki, preset.alpha,
        )
        self.assertAlmostEqual(result.duration_s, 0.05798666257804466, places=14)
        self.assertAlmostEqual(result.start_length_mm, 15.003737721839713, places=11)
        self.assertAlmostEqual(result.end_length_mm, 37.45458009100148, places=11)
        self.assertAlmostEqual(math.degrees(result.final_angle_rad), 89.89305409075574, places=11)

    def test_turn1600_model_matches_long90_measured_ranges(self):
        preset = PRESETS["turn1600"]
        result = simulate_turn(
            "long_turn90", preset.velocity, preset.motor,
            preset.radii_mm["long_turn90"], preset.kp, preset.ki,
            preset.alpha, preset.dynamics,
        )
        origin_x = -45.0 + result.start_length_mm
        # Latest R/L logs contain two report-padding samples at each profile
        # edge. Compare the final non-padding model point with 25 measured
        # turns: forward 68.254 +/- 0.621 mm, lateral 67.776 +/- 0.338 mm.
        forward_mm = result.x_mm[-2] - origin_x
        lateral_mm = result.y_mm[-2] + 45.0
        self.assertAlmostEqual(forward_mm, 68.254, delta=1.0)
        self.assertAlmostEqual(lateral_mm, 67.776, delta=1.5)
        self.assertEqual(result.model_name, "turn1600_measured")
        np.testing.assert_allclose(result.velocity_m_s, 1.6)
        # Measured R/L slip peaks are 0.2012 and 0.1971 rad; measured horizon
        # velocity peaks are 0.3299 and 0.3245 m/s.
        self.assertAlmostEqual(np.max(np.abs(result.beta_rad)), 0.199, delta=0.01)
        self.assertAlmostEqual(
            np.max(np.abs(result.lateral_velocity_m_s)), 0.327, delta=0.02
        )

    def test_turn1600_contains_firmware_parameter_table(self):
        preset = PRESETS["turn1600"]
        self.assertEqual(preset.velocity, 1.6)
        self.assertEqual(preset.radii_mm["turn_in135"], 42.0)
        self.assertEqual(preset.radii_mm["turn_out135"], 39.3)
        self.assertEqual(preset.configured_lengths_mm, {
            "long_turn90": (14.69, 34.06),
            "long_turn180": (10.79, 32.40),
            "turn_in45": (25.41, 23.05),
            "turn_out45": (21.00, 20.61),
            "turn_in135": (11.60, 25.47),
            "turn_out135": (12.47, 40.99),
            "turn_v90": (4.51, 23.78),
        })

    def test_turn1600_accepts_variable_velocity_profile(self):
        preset = PRESETS["turn1600"]
        profile = np.array([1.45, 1.60, 1.72, 1.55])
        result = simulate_turn(
            "long_turn90", preset.velocity, preset.motor,
            preset.radii_mm["long_turn90"], preset.kp, preset.ki,
            preset.alpha, preset.dynamics, velocity_profile=profile,
        )
        expected = np.interp(
            np.linspace(0.0, 1.0, len(result.velocity_m_s)),
            np.linspace(0.0, 1.0, len(profile)),
            profile,
        )
        np.testing.assert_allclose(result.velocity_m_s, expected)

        with self.assertRaises(ValueError):
            simulate_turn(
                "long_turn90", preset.velocity, preset.motor,
                preset.radii_mm["long_turn90"], preset.kp, preset.ki,
                preset.alpha, preset.dynamics,
                velocity_profile=np.array([1.6, 0.0]),
            )

    def test_every_motion_produces_consistent_finite_series(self):
        preset = PRESETS["type8i"]
        for motion in MOTIONS:
            with self.subTest(motion=motion):
                result = simulate_turn(motion, preset.velocity, preset.motor, 50.0)
                count = len(result.time_s)
                self.assertGreater(count, 0)
                for values in (
                    result.yaw_rate_rad_s,
                    result.yaw_accel_rad_s2,
                    result.beta_rad,
                    result.beta_rate_rad_s,
                    result.velocity_m_s,
                    result.lateral_velocity_m_s,
                    result.longitudinal_accel_m_s2,
                    result.lateral_accel_m_s2,
                    result.centripetal_accel_m_s2,
                ):
                    self.assertEqual(len(values), count)
                    self.assertTrue(np.all(np.isfinite(values)))
                self.assertEqual(len(result.x_mm), count + 2)
                self.assertEqual(len(result.y_mm), count + 2)
                self.assertTrue(math.isfinite(result.start_length_mm))
                self.assertTrue(math.isfinite(result.end_length_mm))

    def test_csv_has_one_row_per_simulation_sample(self):
        preset = PRESETS["search"]
        result = simulate_turn("turn90", preset.velocity, preset.motor, 26.0)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "turn.csv"
            write_csv(result, path)
            self.assertEqual(len(path.read_text(encoding="utf-8").splitlines()), len(result.time_s) + 1)

    def test_invalid_parameters_are_rejected(self):
        motor = PRESETS["type8i"].motor
        with self.assertRaises(ValueError):
            simulate_turn("unknown", 2.0, motor, 50.0)
        with self.assertRaises(ValueError):
            simulate_turn("long_turn90", 0.0, motor, 50.0)


if __name__ == "__main__":
    unittest.main()
