"""角度補正付きの短いR180をFF解析から誤除外しないことを確認する。"""
from pathlib import Path
import math
import sys
import tempfile
import unittest

import matplotlib
matplotlib.use("Agg")
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools/analysis"))
from turn_ff_report import analyze_group


class TurnFfReportTests(unittest.TestCase):
    def settings(self, direction=-1, correction=4.25):
        parameters = dict(velo=1.8, r_min=direction * 46.2, degree=direction * 180)
        if correction is not None:
            parameters["correction"] = -direction * correction
        return dict(motion_name="long_r180" if direction < 0 else "long_l180",
            parameters=parameters, feedforward=dict(ff_om_velo=.0168, ff_om_accel=.00088,
                ff_om_decel=.0009, ff_om_bias=.0613))

    def frame(self, samples=108, direction=-1, angle_deg=175.75):
        # Match the observed 108-sample corrected R180 duration and commanded
        # angle with a pulse independent of the report's duration formula.
        pulse = np.sqrt(np.sin(np.linspace(.01, math.pi - .01, samples - 4)))
        pulse *= math.radians(angle_deg) / (pulse.sum() * .001)
        ideal = np.r_[np.zeros(30), direction * pulse, np.zeros(35)]
        ego = np.r_[np.zeros(2), ideal[:-2]] * .99
        voltage = .0168 * ideal + .00089 * np.gradient(ideal, .001)
        return pd.DataFrame({"ideal.rad_velo": ideal, "ego.rad_velo": ego,
            "om_feedback": .1 * (ideal - ego), "om_feedforward": voltage,
            "V_r": voltage, "V_l": voltage, "Battery": np.full(len(ideal), 8.)})

    def test_corrected_short_r180_is_retained_in_both_directions(self):
        for direction in (-1, 1):
            with self.subTest(direction=direction), tempfile.TemporaryDirectory() as directory:
                folder = Path(directory)
                source = folder / "input.csv"
                self.frame(direction=direction).to_csv(source, index=False)
                summary = analyze_group([(source, self.settings(direction))], folder / "report")
                self.assertEqual(summary["turn_count"], 1)
                self.assertEqual(summary["aggregate_turn_count"], 1)
                self.assertEqual(summary["metrics_mean"]["duration_ms"], 108)
                self.assertTrue(math.isfinite(summary["metrics_mean"]["tracking_rms_rad_s"]))
                self.assertTrue((folder / "report/response.png").is_file())

    def test_missing_correction_keeps_legacy_zero_correction_logs(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            source = folder / "input.csv"
            self.frame(samples=116, angle_deg=180).to_csv(source, index=False)
            summary = analyze_group([(source, self.settings(correction=None))], folder / "report")
            self.assertEqual(summary["turn_count"], 1)
            self.assertEqual(summary["metrics_mean"]["duration_ms"], 116)

    def test_inconsistent_duration_reports_the_observed_segment(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            source = folder / "input.csv"
            self.frame().to_csv(source, index=False)
            with self.assertRaisesRegex(ValueError, r"expected 117 ms.*observed \[108\]"):
                analyze_group([(source, self.settings(correction=None))], folder / "report")

    def test_no_turn_reports_an_empty_observation_instead_of_key_error(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            source = folder / "input.csv"
            frame = self.frame()
            frame["ideal.rad_velo"] = 0.
            frame.to_csv(source, index=False)
            with self.assertRaisesRegex(ValueError, r"No turn segment.*observed \[\]"):
                analyze_group([(source, self.settings())], folder / "report")


if __name__ == "__main__":
    unittest.main()
