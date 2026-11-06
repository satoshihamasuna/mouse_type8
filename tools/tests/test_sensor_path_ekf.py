"""Independent numerical/causality checks for the velocity-state sensor EKF."""
from pathlib import Path
import math
import sys
import tempfile
import unittest

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"analysis"))
import sensor_path_ekf as ekf


def samples(count=200):
    config = ekf.Config()
    rows = []
    for i in range(count):
        e = .4*.005*i*math.sin(math.radians(2))
        rows.append({"sequence": i, "time_ms": 1000+5*i, "irq.period_us": 1000.,
                     ekf.RIGHT: .4/config.mm_per_tick, ekf.LEFT: -.4/config.mm_per_tick,
                     "ego.rad_velo": .02, "ego.accel": .1,
                     "sen_l.distance": 45-e*1000, "sen_r.distance": 45+e*1000,
                     "sen_fl.distance": 125., "sen_fr.distance": 125.})
    return rows


class SensorPathTests(unittest.TestCase):
    def test_prediction_jacobian_against_numerical_differences(self):
        state = np.array([.2, .3, .7, .6, 2., .01, .1, .02])
        _, actual = ekf.motion(state, 1.7, .005)
        numerical = np.zeros((8, 8))
        for i in range(8):
            delta = np.eye(8)[i]*1e-6
            numerical[:, i] = (ekf.motion(state+delta, 1.7, .005)[0]-ekf.motion(state-delta, 1.7, .005)[0])/2e-6
        np.testing.assert_allclose(actual, numerical, atol=1e-10)

    def test_wheel_polarity_and_measured_tick_duration(self):
        row = samples(1)[0]
        row[ekf.RIGHT], row[ekf.LEFT], row["irq.period_us"] = 1., -1., 1018.
        right, left, variance = ekf.wheel_measurements(row, ekf.Config())
        self.assertAlmostEqual(right, ekf.Config().mm_per_tick/1.018)
        self.assertEqual(right, left)
        self.assertGreater(variance, 0.)
        row["irq.period_us"] = 5000.
        with self.assertRaises(ValueError):
            ekf.wheel_measurements(row, ekf.Config())

    def test_wall_anchor_does_not_change_or_shrink_global_pose(self):
        estimator = ekf.SensorPathEkf()
        estimator.state[:3] = [.3, -.7, .02]
        estimator.covariance[:2, :2] = [[.04, .01], [.01, .09]]
        state, covariance = estimator.state.copy(), estimator.covariance.copy()
        estimator.axis = math.pi/2
        estimator.anchor_wall(.007, 1e-6)
        np.testing.assert_array_equal(estimator.state[:7], state[:7])
        np.testing.assert_array_equal(estimator.covariance[:7, :7], covariance[:7, :7])
        self.assertGreaterEqual(np.linalg.eigvalsh(estimator.covariance).min(), -1e-12)
        h = np.zeros(8)
        h[:2], h[7] = [-math.cos(estimator.axis), -math.sin(estimator.axis)], -1.
        self.assertAlmostEqual(h@estimator.state, .007)
        self.assertAlmostEqual(h@estimator.covariance@h, 1e-6)

    def test_known_straight_motion_and_bias_preserve_finite_covariance(self):
        estimator = ekf.SensorPathEkf(use_walls=False)
        estimator.initialize_bias(.02, .1, True)
        for row in samples():
            result = estimator.update(row)
        self.assertAlmostEqual(result["velocity_m_s"], .4, delta=.005)
        self.assertAlmostEqual(result["north_mm"], .4*.005*199*1000, delta=8.)
        self.assertAlmostEqual(result["heading_rad"], 0., delta=.002)
        self.assertGreaterEqual(np.linalg.eigvalsh(estimator.covariance).min(), -1e-12)

    def test_range_changes_observe_heading_without_known_absolute_wall(self):
        with_wall, no_wall = ekf.SensorPathEkf(), ekf.SensorPathEkf(use_walls=False)
        for estimator in (with_wall, no_wall):
            estimator.initialize_bias(.02, .1, True)
        for row in samples():
            with_wall.update(row)
            no_wall.update(row)
        self.assertGreater(with_wall.wall_updates, 20)
        target = math.radians(2)
        self.assertLess(abs(with_wall.state[2]-target), abs(no_wall.state[2]-target))
        self.assertAlmostEqual(with_wall.state[2], target, delta=math.radians(.8))

    def test_constant_turn_matches_analytic_circle_with_correct_left_yaw_sign(self):
        estimator = ekf.SensorPathEkf(use_walls=False)
        estimator.initialize_bias(.02, .1, True)
        config = ekf.Config()
        velocity, omega = .4, 2.
        rows = samples(500)
        for row in rows:
            row[ekf.RIGHT] = (velocity+config.tread_m*omega/2)/config.mm_per_tick
            row[ekf.LEFT] = -(velocity-config.tread_m*omega/2)/config.mm_per_tick
            row["ego.rad_velo"] = omega+.02
            estimator.update(row)
        angle = omega*.005*(len(rows)-1)
        expected = velocity/omega*np.array([math.cos(angle)-1, math.sin(angle)])
        np.testing.assert_allclose(estimator.state[:2], expected, atol=.004)
        self.assertAlmostEqual(estimator.state[2], angle, delta=.004)

    def test_derived_velocity_pose_and_commands_do_not_enter_estimator(self):
        first, second = ekf.SensorPathEkf(), ekf.SensorPathEkf()
        for row in samples(80):
            actual = first.update(row)
            alternate = second.update({**row, "ego.velo": 1000., "ego.radian": -100.,
                "ego.turn_x": 1e7, "ego.turn_y": -1e7, "ideal.velo": 999., "ideal.rad_velo": 12.})
            self.assertEqual(actual, alternate)
            np.testing.assert_array_equal(first.covariance, second.covariance)

    def test_heldout_wall_values_never_change_state_or_gate(self):
        first, second = ekf.SensorPathEkf(holdout_fold=1), ekf.SensorPathEkf(holdout_fold=1)
        heldout_count = 0
        for row in samples():
            result = first.update(row)
            alternate = row.copy()
            if result["heldout"]:
                alternate.update({key: -10000. for key in ekf.WALL_COLUMNS})
                heldout_count += 1
            self.assertEqual(result, second.update(alternate))
            np.testing.assert_array_equal(first.covariance, second.covariance)
        self.assertGreater(heldout_count, 10)

    def test_future_append_does_not_change_prefix(self):
        short, long = ekf.SensorPathEkf(), ekf.SensorPathEkf()
        prefix = [short.update(row) for row in samples(60)]
        full = [long.update(row) for row in samples(100)]
        self.assertEqual(prefix, full[:60])

    def test_frozen_zero_telemetry_is_not_valid_bias_calibration(self):
        quiet = [{**row, ekf.RIGHT: 0., ekf.LEFT: 0., "ego.rad_velo": 0., "ego.accel": 0.}
                 for row in samples(60)]
        self.assertFalse(ekf.calibrate(quiet, ekf.Config())["ready"])
        for i, row in enumerate(quiet):
            row["ego.rad_velo"] = .005+.001*math.sin(i)
            row["ego.accel"] = .1+.01*math.cos(i)
        calibration = ekf.calibrate(quiet, ekf.Config())
        self.assertTrue(calibration["ready"])
        self.assertAlmostEqual(calibration["gyro_bias_rad_s"], .005, delta=.0002)
        frozen = [{**row, "ego.rad_velo": 0., "ego.accel": 0.} for row in quiet]*5
        mixed = ekf.calibrate(frozen+quiet, ekf.Config())
        self.assertEqual(mixed["skipped_frozen_zero_sensor_pairs"], 300)
        self.assertAlmostEqual(mixed["gyro_bias_rad_s"], calibration["gyro_bias_rad_s"])
        constant = [{**row, "ego.rad_velo": .007366, "ego.accel": -.1} for row in quiet]
        quantized = ekf.calibrate(constant, ekf.Config())
        self.assertTrue(quantized["ready"])
        self.assertAlmostEqual(quantized["gyro_bias_rad_s"], .007366)

    def test_cli_handles_pre_motion_timing_reset_without_filling_missing_timing(self):
        quiet = [{**row, ekf.RIGHT: 0., ekf.LEFT: 0., "ideal.velo": 0., "ideal.rad_velo": 0., "ego.velo": 0.}
                 for row in samples(40)]
        quiet[0]["irq.period_us"] = float("nan")
        moving = [{**row, "sequence": 40+i, "time_ms": 1200+5*i, "ideal.velo": .4,
                   "ideal.rad_velo": 0., "ego.velo": .4} for i, row in enumerate(samples(10))]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            pd.DataFrame(quiet+moving).to_csv(path/"telemetry.csv", index=False)
            (path/"run.json").write_text('{"firmware":{"mouse":"A"}}', encoding="utf-8")
            ekf.main(["--log", str(path/"telemetry.csv"), "--settings", str(path/"run.json"), "--output", str(path/"out")])
            self.assertTrue((path/"out/report.json").exists())
            self.assertTrue((path/"out/paths_overlay.png").exists())
        moving[0]["irq.period_us"] = float("nan")
        with self.assertRaises(ValueError):
            ekf.replay(pd.DataFrame(quiet+moving))

    def test_speed_comparator_cannot_change_filtered_paths(self):
        frame = pd.DataFrame(samples(50))
        frame["ideal.velo"], frame["ideal.rad_velo"], frame["ego.velo"] = .4, 0., .4
        first, _ = ekf.replay(frame)
        frame["ego.velo"] = 100.
        second, _ = ekf.replay(frame)
        filtered = [c for c in first if c.startswith("sensor_")]
        pd.testing.assert_frame_equal(first[filtered], second[filtered])
        self.assertNotEqual(first["logged_velocity.north_mm"].iloc[-1], second["logged_velocity.north_mm"].iloc[-1])

    def test_rejects_gaps_and_nonfinite_values(self):
        estimator = ekf.SensorPathEkf()
        rows = samples(3)
        estimator.update(rows[0])
        with self.assertRaises(ValueError):
            estimator.update(rows[2])
        with self.assertRaises(ValueError):
            ekf.SensorPathEkf().update({**rows[0], "ego.accel": float("nan")})

    def test_estimated_velocity_mode_needs_no_encoder_or_interrupt_timing(self):
        first = ekf.SensorPathEkf(velocity_source="estimated_velocity")
        second = ekf.SensorPathEkf(velocity_source="estimated_velocity")
        for estimator in (first, second):
            estimator.initialize_bias(.02, .1, True)
        for row in samples(120):
            row = {key: value for key, value in row.items() if key not in (ekf.RIGHT, ekf.LEFT, "irq.period_us")}
            row["ego.velo"] = .401
            actual = first.update(row)
            alternate = second.update({**row, ekf.RIGHT: float("nan"), ekf.LEFT: 1e6, "irq.period_us": float("nan")})
            self.assertEqual(actual, alternate)
            np.testing.assert_array_equal(first.covariance, second.covariance)
        self.assertAlmostEqual(first.state[3], .4, delta=.005)
        self.assertGreater(first.wall_updates, 10)

    def test_estimated_velocity_observation_changes_path(self):
        estimators = [ekf.SensorPathEkf(use_walls=False, velocity_source="estimated_velocity") for _ in range(2)]
        for estimator in estimators:
            estimator.initialize_bias(.02, .1, True)
        for row in samples(80):
            estimators[0].update({**row, "ego.velo": .401})
            estimators[1].update({**row, "ego.velo": .801})
        self.assertGreater(estimators[1].state[1]-estimators[0].state[1], .14)

    def test_estimated_velocity_accel_bias_and_correlated_covariance(self):
        estimator = ekf.SensorPathEkf(use_walls=False, velocity_source="estimated_velocity")
        estimator.initialize_bias(.02, .1, True)
        for i, row in enumerate(samples(300)):
            velocity = .3+.8*.005*i
            row.update({"ego.velo": velocity+.001, "ego.accel": .9})
            estimator.update(row)
        self.assertAlmostEqual(estimator.state[3], velocity, delta=.012)
        self.assertAlmostEqual(estimator.state[6], .1, delta=.015)
        self.assertGreaterEqual(np.linalg.eigvalsh(estimator.covariance).min(), -1e-12)

    def test_correlated_update_matches_gaussian_error_simulation(self):
        # Known joint prior error/observation noise, checked against independent
        # sampled estimation errors rather than repeating the filter equations.
        estimator = ekf.SensorPathEkf()
        estimator.covariance = np.eye(8)
        h, cross = np.zeros(8), np.zeros(8)
        h[3], cross[3] = 1., -.5
        estimator.observe(2., h, 1., error_noise_cross=cross)
        self.assertAlmostEqual(estimator.state[3], 1.)
        rng = np.random.RandomState(987)
        prior_error, independent = rng.normal(size=(2, 200000))
        measurement_noise = -.5*prior_error+math.sqrt(.75)*independent
        posterior_error = prior_error-.5*(prior_error+measurement_noise)
        self.assertAlmostEqual(estimator.covariance[3, 3], np.var(posterior_error), delta=.01)

    def test_estimated_mode_future_and_external_holdout_are_causal(self):
        first = ekf.SensorPathEkf(velocity_source="estimated_velocity")
        second = ekf.SensorPathEkf(velocity_source="estimated_velocity")
        prefix = []
        for i, row in enumerate(samples(120)):
            row["ego.velo"] = .4
            heldout = 30 <= i < 60
            result = first.update(row, wall_heldout=heldout)
            alternate = row.copy()
            if heldout:
                alternate.update({key: float("nan") for key in ekf.WALL_COLUMNS})
            self.assertEqual(result, second.update(alternate, wall_heldout=heldout))
            np.testing.assert_array_equal(first.covariance, second.covariance)
            prefix.append(result)
        short = ekf.SensorPathEkf(velocity_source="estimated_velocity")
        actual = [short.update({**row, "ego.velo": .4}, wall_heldout=30 <= i < 60)
                  for i, row in enumerate(samples(80))]
        self.assertEqual(actual, prefix[:80])

    def test_estimated_velocity_cli_selects_mode(self):
        quiet = [{**row, ekf.RIGHT: 0., ekf.LEFT: 0., "ideal.velo": 0., "ideal.rad_velo": 0., "ego.velo": .001}
                 for row in samples(40)]
        moving = [{**row, "sequence": 40+i, "time_ms": 1200+5*i, "ideal.velo": .4,
                   "ideal.rad_velo": 0., "ego.velo": .401} for i, row in enumerate(samples(10))]
        with tempfile.TemporaryDirectory() as directory:
            import contextlib
            import io
            import json
            path = Path(directory)
            pd.DataFrame(quiet+moving).to_csv(path/"telemetry.csv", index=False)
            (path/"run.json").write_text('{"firmware":{"mouse":"A"}}', encoding="utf-8")
            with contextlib.redirect_stdout(io.StringIO()):
                ekf.main(["--log", str(path/"telemetry.csv"), "--settings", str(path/"run.json"),
                          "--output", str(path/"out"), "--velocity-source", "estimated_velocity"])
            report = json.loads((path/"out/report.json").read_text(encoding="utf8"))
            self.assertEqual(report["velocity_source"], "estimated_velocity")


if __name__ == "__main__":
    unittest.main()
