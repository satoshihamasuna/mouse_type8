"""Causality, local observability and numerical checks for streaming EKFs."""
import csv
import contextlib
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"analysis"))
import streaming_wall_ekf as ekf


def samples(count=160, quiet=40):
    rows = []
    for i in range(count):
        moving = i >= quiet
        distance = max(0, i-quiet)*2.
        e = distance*math.sin(math.radians(2))
        row = {key: 0. for key in ekf.REQUIRED}
        row.update(sequence=i, time_ms=1000+i*5, period_ms=5)
        row.update({"ideal.velo": .4 if moving else 0., "ego.velo": .4 if moving else 0.,
                    "ego.rad_velo": .005, "ego.radian": .005*max(0, i-quiet)*.005,
                    "ego.turn_x": distance*math.sin(math.radians(2)),
                    "ego.turn_y": distance*math.cos(math.radians(2)),
                    "sen_l.distance": 45-e, "sen_r.distance": 45+e,
                    "sen_fl.distance": 125., "sen_fr.distance": 125.})
        rows.append(row)
    return rows


def replay(rows, fold=None):
    filt = ekf.StreamingWallEstimator(holdout_fold=fold)
    outputs = [filt.update(row) for row in rows]
    return filt, outputs


class LocalEkfTests(unittest.TestCase):
    def test_local_jacobian_matches_finite_difference(self):
        state = np.array([.004, .13])
        _, analytic = ekf.local_motion(state, .003, .007)
        numerical = np.zeros((2, 2))
        for column in range(2):
            step = np.eye(2)[column]*1e-6
            numerical[:, column] = (ekf.local_motion(state+step, .003, .007)[0] -
                                    ekf.local_motion(state-step, .003, .007)[0])/2e-6
        np.testing.assert_allclose(analytic, numerical, atol=1e-10)

    def test_known_lateral_and_heading_truth_converges(self):
        rng = np.random.default_rng(41)
        filt = ekf.LateralHeadingEkf()
        truth = np.array([-.005, math.radians(2)])
        for i in range(200):
            truth, _ = ekf.local_motion(truth, .00175, 0.)
            filt.predict(.00175, 0., .005)
            filt.observe_lateral(truth[0]+rng.normal(0, .00015), .0005**2)
            np.testing.assert_allclose(filt.covariance, filt.covariance.T, atol=1e-14)
            self.assertGreater(np.linalg.eigvalsh(filt.covariance).min(), 0.)
        self.assertLess(abs(filt.state[0]-truth[0]), .0005)
        self.assertLess(abs(math.degrees(filt.state[1]-truth[1])), .4)

    def test_left_and_right_ranges_have_consistent_sign(self):
        cfg = ekf.LocalConfig()
        row = {"sen_l.distance": 41., "sen_r.distance": 49.}
        for mask in (1, 2, 3):
            value, _ = ekf.lateral_observation(row, mask, cfg)
            self.assertAlmostEqual(value, .004, places=12)

    def test_both_wall_difference_rejects_common_offset(self):
        row = {"sen_l.distance": 44., "sen_r.distance": 52.}
        value, _ = ekf.lateral_observation(row, 3, ekf.LocalConfig())
        self.assertAlmostEqual(value, .004, places=12)

    def test_no_travel_does_not_claim_to_observe_heading(self):
        filt = ekf.LateralHeadingEkf()
        filt.state[1] = .1
        before = filt.covariance[1, 1]
        for _ in range(50):
            filt.predict(0., 0., .005)
            filt.observe_lateral(.002, 1e-6)
        self.assertAlmostEqual(filt.state[1], .1, places=12)
        self.assertGreaterEqual(filt.covariance[1, 1], before)

    def test_dropout_predicts_nonzero_lateral_drift_and_uncertainty(self):
        filt = ekf.LateralHeadingEkf()
        filt.state[1] = math.radians(3)
        before = filt.covariance.copy()
        for _ in range(50):
            filt.predict(.002, 0., .005)
        self.assertAlmostEqual(filt.state[0], .1*math.sin(math.radians(3)), places=12)
        self.assertGreater(filt.covariance[0, 0], before[0, 0])

    def test_outlier_rejection_leaves_state_unchanged(self):
        filt = ekf.LateralHeadingEkf()
        state, p = filt.state.copy(), filt.covariance.copy()
        self.assertFalse(filt.observe_lateral(.08, 1e-6))
        np.testing.assert_array_equal(filt.state, state)
        np.testing.assert_array_equal(filt.covariance, p)

    def test_invalid_units_and_parameters_rejected(self):
        with self.assertRaises(ValueError):
            ekf.LocalConfig(side_noise_mm=0.)
        with self.assertRaises(ValueError):
            ekf.LateralHeadingEkf().predict(.002, 0., 5.)
        with self.assertRaises(ValueError):
            ekf.LateralHeadingEkf().observe_lateral(float("nan"), 1e-6)


class StreamingTests(unittest.TestCase):
    def test_appending_or_changing_future_rows_cannot_change_prefix(self):
        data = samples()
        _, first = replay(data[:95])
        altered = [dict(row) for row in data]
        for row in altered[95:]:
            row["ego.turn_x"] += 70
            row["ego.radian"] += 1.
            row["sen_l.distance"] = 80.
        _, second = replay(altered)
        self.assertEqual(first, second[:95])

    def test_wall_values_in_whole_heldout_blocks_cannot_change_states(self):
        data = samples()
        _, original = replay(data, 0)
        altered = [dict(row) for row in data]
        for row, out in zip(altered, original):
            if out.get("heldout"):
                row["sen_l.distance"] = 80.
                row["sen_r.distance"] = 25.
        _, changed = replay(altered, 0)
        keys = ("global.east_mm", "global.north_mm", "global.yaw_rad", "global.bias_deg_s",
                "local.e_mm", "local.psi_deg", "local_updated", "lateral_only.e_mm")
        for a, b in zip(original, changed):
            if a["started"]:
                for key in keys:
                    self.assertEqual(a[key], b[key], key)

    def test_gyro_bias_is_frozen_from_past_stationary_samples(self):
        data = samples(100)
        for row in data[80:]:
            row.update({"ideal.velo": 0., "ego.velo": 0., "ego.rad_velo": .02})
        filt, out = replay(data)
        self.assertTrue(filt.bias_ready)
        self.assertAlmostEqual(filt.bias, .005, places=12)
        self.assertEqual(len(filt.quiet), 40)

    def test_start_without_stationary_history_is_explicitly_uncalibrated(self):
        data = samples(50, quiet=0)
        filt, _ = replay(data)
        self.assertFalse(filt.bias_ready)
        self.assertEqual(filt.bias, 0.)

    def test_local_coordinate_reset_does_not_restart_straight_filter(self):
        data = samples(120)
        for row in data[75:]:
            row["ego.turn_x"] -= data[74]["ego.turn_x"]
            row["ego.turn_y"] -= data[74]["ego.turn_y"]
            row["ego.radian"] -= data[74]["ego.radian"]
        _, out = replay(data)
        self.assertEqual(out[74]["segment"], out[75]["segment"])
        self.assertGreaterEqual(out[75]["local_updates_in_segment"], out[74]["local_updates_in_segment"])

    def test_turn_suspends_and_restarts_passage_model(self):
        data = samples(120)
        for row in data[75:85]:
            row["ideal.rad_velo"] = 6.
            row["ideal.radian"] = .5
        _, out = replay(data)
        self.assertFalse(out[80]["local_active"])
        self.assertIsNone(out[80]["local.e_mm"])
        self.assertEqual(out[85]["segment"], out[74]["segment"]+1)

    def test_clamped_or_front_blocked_samples_are_not_used(self):
        cfg = ekf.LocalConfig()
        gate = ekf.WallRangeGate(cfg)
        row = samples(1)[0]
        row.update({"sen_l.distance": 80., "sen_r.distance": 25.})
        for _ in range(10):
            self.assertEqual(gate.update(row, .002), 0)
        row.update({"sen_l.distance": 45., "sen_r.distance": 45., "sen_fl.distance": 40.})
        for _ in range(10):
            self.assertEqual(gate.update(row, .002), 0)

    def test_spatial_blocks_are_disjoint_and_have_bounded_memory(self):
        block = ekf.SpatialBlocks(0)
        completed = []
        for i in range(80):
            block.begin(True)
            result = block.append({"travel_m": .002, "sequence": i})
            self.assertLess(len(block.rows), 128)
            if result:
                completed.append(result)
        for a, b in zip(completed, completed[1:]):
            self.assertLess(a[-1]["sequence"], b[0]["sequence"])
        self.assertGreater(len(completed), 3)

    def test_runtime_history_buffers_are_bounded(self):
        filt, _ = replay(samples(900, quiet=600))
        self.assertLessEqual(len(filt.quiet), 500)
        self.assertLessEqual(len(filt.odometry.durations), 21)
        self.assertLess(len(filt.blocks.rows), 128)

    def test_missing_sequence_and_time_units_fail_explicitly(self):
        data = samples(3)
        data[1]["sequence"] = 5
        with self.assertRaisesRegex(ValueError, "sequence"):
            replay(data)
        data = samples(3)
        data[1]["time_ms"] += 1000
        with self.assertRaisesRegex(ValueError, "timestamp"):
            replay(data)

    def test_csv_reader_yields_only_requested_prefix(self):
        data = samples(8)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"samples.csv"
            with path.open("w", encoding="utf-8", newline="") as file:
                writer = csv.DictWriter(file, fieldnames=ekf.REQUIRED)
                writer.writeheader()
                writer.writerows(data)
            reader = ekf.CsvSamples(path, max_rows=3)
            actual = list(reader)
            self.assertEqual(actual, data[:3])
            self.assertEqual(reader.rows, 3)
            self.assertEqual(len(reader.digest.hexdigest()), 64)

    def test_csv_reader_rejects_incomplete_final_record(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"samples.csv"
            path.write_text(",".join(ekf.REQUIRED)+"\n0,1,", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "Incomplete"):
                list(ekf.CsvSamples(path))

    def test_live_follow_waits_for_completion_of_an_appended_record(self):
        data = samples(2)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"growing.csv"
            line = ",".join(str(data[1][key]) for key in ekf.REQUIRED)+"\n"
            split = len(line)//2
            path.write_text(",".join(ekf.REQUIRED)+"\n" +
                            ",".join(str(data[0][key]) for key in ekf.REQUIRED)+"\n"+line[:split], encoding="utf-8")
            reader = iter(ekf.CsvSamples(path, follow=True, max_rows=2))
            self.assertEqual(next(reader), data[0])

            def finish_record():
                time.sleep(.05)
                with path.open("a", encoding="utf-8") as file:
                    file.write(line[split:])
            writer = threading.Thread(target=finish_record)
            writer.start()
            self.assertEqual(next(reader), data[1])
            writer.join(timeout=1.)
            with self.assertRaises(StopIteration):
                next(reader)

    def test_header_only_capture_has_an_explicit_empty_report(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"empty.csv"
            path.write_text(",".join(ekf.REQUIRED)+"\n", encoding="utf-8")
            output = Path(directory)/"result"
            with contextlib.redirect_stdout(io.StringIO()):
                ekf.main(["--log", str(path), "--output", str(output)])
            report = json.loads((output/"report.json").read_text(encoding="utf-8"))
            self.assertEqual(report["processed_rows"], 0)
            self.assertEqual(report["validation_blocks"], 0)
            self.assertFalse(report["initial_bias_ready"])


if __name__ == "__main__":
    unittest.main()
