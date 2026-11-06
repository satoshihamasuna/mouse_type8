"""Signs, input uncertainty, causality and observability of the direct-sensor EKF."""
from pathlib import Path
import csv
import math
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"analysis"))
import encoder_wall_imu_ekf as fusion


def samples(count=180, quiet=40):
    rows = []
    for i in range(count):
        moving = i >= quiet
        travel = max(0, i-quiet)*.2743414049820854*5
        e = travel*math.sin(math.radians(2))
        rows.append({"sequence": i, "time_ms": 1000+5*i, "period_ms": 5,
            fusion.RIGHT: 1. if moving else 0., fusion.LEFT: -1. if moving else 0.,
            "ideal.velo": .3 if moving else 0., "ideal.rad_velo": 0., "ideal.radian": 0.,
            "ego.rad_velo": .005, "sen_l.distance": 45-e, "sen_r.distance": 45+e,
            "sen_fl.distance": 125., "sen_fr.distance": 125.})
    return rows


class SensorFusionTests(unittest.TestCase):
    def test_encoder_polarity_and_one_ms_snapshot_units(self):
        config = fusion.SensorConfig()
        r, l = fusion.decode_wheel_rates(1., -1., config)
        self.assertAlmostEqual(r, config.mm_per_tick)
        self.assertAlmostEqual(l, config.mm_per_tick)
        control = fusion.fused_controls(r, l, 0., (1., 1.), 1., .028)
        self.assertEqual(control["encoder_yaw"], 0.)

    def test_faster_right_wheel_is_positive_left_yaw(self):
        control = fusion.fused_controls(.42, .38, 0., (1e-4, 1e-4), .001, .028, "encoder_only")
        self.assertAlmostEqual(control["omega"], .04/.028)
        self.assertAlmostEqual(control["velocity"], .4)

    def test_input_fusion_prefers_the_less_uncertain_sensor(self):
        control = fusion.fused_controls(.42, .38, .1, (1., 1.), .001, .028, disagreement_sigma=1000.)
        self.assertLess(control["encoder_weight"], .001)
        self.assertLess(abs(control["omega"]-.1), .01)

    def test_wheel_disagreement_inflates_encoder_uncertainty(self):
        gated = fusion.fused_controls(1., -1., 0., (1e-5, 1e-5), .0001, .028)
        plain = fusion.fused_controls(1., -1., 0., (1e-5, 1e-5), .0001, .028, disagreement_sigma=1e6)
        self.assertGreater(gated["encoder_variance_inflation"], 1.)
        self.assertLess(gated["encoder_weight"], plain["encoder_weight"])

    def test_correlated_control_covariance_is_positive_semidefinite(self):
        control = fusion.fused_controls(.3, .3, 0., (1e-4, 4e-4), .0001, .028)
        self.assertNotEqual(control["control_covariance"][0, 1], 0.)
        self.assertGreater(np.linalg.eigvalsh(control["control_covariance"]).min(), 0.)

    def test_state_and_control_jacobians_match_numerical_derivatives(self):
        x = np.array([.003, .14]); v, w, dt = .37, .08, .005
        _, f, g = fusion.sensor_motion(x, v, w, dt)
        nf, ng = np.zeros((2, 2)), np.zeros((2, 2))
        for column in range(2):
            dx = np.eye(2)[column]*1e-6
            nf[:, column] = (fusion.sensor_motion(x+dx, v, w, dt)[0]-fusion.sensor_motion(x-dx, v, w, dt)[0])/2e-6
            dv, dw = np.eye(2)[column]*1e-6
            ng[:, column] = (fusion.sensor_motion(x, v+dv, w+dw, dt)[0]-fusion.sensor_motion(x, v-dv, w-dw, dt)[0])/2e-6
        np.testing.assert_allclose(f, nf, atol=1e-10)
        np.testing.assert_allclose(g, ng, atol=1e-10)

    def test_known_lateral_heading_truth_converges_with_two_states(self):
        filt = fusion.EncoderWallImuEkf()
        truth = np.array([-.005, math.radians(2)])
        covariance = np.diag([.005**2, .03**2])
        for i in range(240):
            truth, _, _ = fusion.sensor_motion(truth, .3, 0., .005)
            filt.predict_sensors(.3, 0., .005, covariance)
            filt.observe_lateral(truth[0], .0005**2)
            self.assertEqual(filt.state.shape, (2,))
            self.assertEqual(filt.covariance.shape, (2, 2))
            self.assertGreater(np.linalg.eigvalsh(filt.covariance).min(), 0.)
        self.assertLess(abs(filt.state[0]-truth[0]), .0005)
        self.assertLess(abs(math.degrees(filt.state[1]-truth[1])), .3)

    def test_larger_control_uncertainty_increases_state_uncertainty(self):
        low, high = fusion.EncoderWallImuEkf(), fusion.EncoderWallImuEkf()
        low.predict_sensors(.3, .1, .005, np.eye(2)*1e-6)
        high.predict_sensors(.3, .1, .005, np.eye(2)*.1)
        self.assertGreater(np.trace(high.covariance), np.trace(low.covariance))

    def test_estimator_does_not_use_derived_ego_pose_or_velocity(self):
        first, second = fusion.StreamingEncoderFusion(), fusion.StreamingEncoderFusion()
        for row in samples():
            a = first.update(row)
            b = second.update({**row, "ego.velo": 1e6, "ego.radian": -100., "ego.turn_x": 1e8,
                               "ego.turn_y": -1e8, "ego.length": float("nan")})
            self.assertEqual(a, b)

    def test_appended_future_samples_do_not_change_prefix(self):
        first, second = fusion.StreamingEncoderFusion(), fusion.StreamingEncoderFusion()
        data = samples()
        prefix = [first.update(row) for row in data[:100]]
        for row in data[100:]: row[fusion.RIGHT] = 20.
        full = [second.update(row) for row in data]
        self.assertEqual(prefix, full[:100])

    def test_heldout_wall_values_cannot_change_filter_or_path(self):
        data = samples()
        first = fusion.StreamingEncoderFusion(holdout_fold=0)
        original = [first.update(row) for row in data]
        altered = [dict(row) for row in data]
        for row, output in zip(altered, original):
            if output.get("heldout"):
                row["sen_l.distance"] = 80.; row["sen_r.distance"] = 25.
        second = fusion.StreamingEncoderFusion(holdout_fold=0)
        changed = [second.update(row) for row in altered]
        self.assertEqual(original, changed)

    def test_all_modes_have_identical_validation_intervals(self):
        estimators = {mode: fusion.StreamingEncoderFusion(mode, 0) for mode in fusion.MODES}
        records = {mode: [] for mode in fusion.MODES}
        for row in samples():
            for mode, filt in estimators.items():
                filt.update(row)
                if filt.last_validation:
                    records[mode].append((filt.last_validation["first_sequence"], filt.last_validation["last_sequence"]))
        self.assertGreater(len(records["fused"]), 1)
        self.assertEqual(records["fused"], records["gyro_only"])
        self.assertEqual(records["fused"], records["encoder_only"])

    def test_bias_uses_only_stationary_pre_motion_samples(self):
        filt = fusion.StreamingEncoderFusion()
        data = samples()
        for row in data[100:]: row["ego.rad_velo"] = .02
        for row in data: filt.update(row)
        self.assertAlmostEqual(filt.bias, .005)
        self.assertEqual(len(filt.quiet), 40)

    def test_history_storage_is_bounded(self):
        filt = fusion.StreamingEncoderFusion()
        for row in samples(900, quiet=600): filt.update(row)
        self.assertLessEqual(len(filt.quiet), 500)
        self.assertLessEqual(len(filt.wheels), 5)
        self.assertLess(len(filt.blocks.rows), 128)

    def test_turn_restarts_local_passage_but_keeps_global_odometry(self):
        filt = fusion.StreamingEncoderFusion()
        data = samples(120)
        for row in data[75:85]: row["ideal.rad_velo"] = 6.; row["ideal.radian"] = .5
        output = [filt.update(row) for row in data]
        self.assertFalse(output[80]["local_active"])
        self.assertEqual(output[85]["segment"], output[74]["segment"]+1)
        self.assertGreater(output[85]["raw.north_mm"], output[74]["raw.north_mm"])

    def test_invalid_variance_time_or_sequence_is_rejected(self):
        with self.assertRaises(ValueError): fusion.SensorConfig(encoder_tick_s=0.)
        with self.assertRaises(ValueError): fusion.fused_controls(.3, .3, 0., (0., 1.), .1, .028)
        with self.assertRaises(ValueError): fusion.EncoderWallImuEkf().predict_sensors(.3, 0., 5., np.eye(2))
        data = samples(3); data[1]["sequence"] = 4
        filt = fusion.StreamingEncoderFusion(); filt.update(data[0])
        with self.assertRaises(ValueError): filt.update(data[1])

    def test_csv_reader_needs_only_direct_measurement_columns(self):
        data = samples(3)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"direct.csv"
            with path.open("w", newline="", encoding="utf-8") as file:
                writer = csv.DictWriter(file, fieldnames=fusion.REQUIRED)
                writer.writeheader(); writer.writerows(data)
            actual = list(fusion.CsvSamples(path, required_columns=fusion.REQUIRED))
            self.assertEqual(actual, data)


if __name__ == "__main__":
    unittest.main()
