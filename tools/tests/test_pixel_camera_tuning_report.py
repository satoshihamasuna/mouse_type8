"""Relative turn evaluation preserves errors under a changed setup pose."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'analysis'))
import pixel_camera_tuning_report as report


class EntryFrameTests(unittest.TestCase):
    def evaluate(self, angle_deg, translation):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            stamps = np.arange(0, 3.001, .005)
            local = np.column_stack([
                np.interp(stamps, [0, .5, 1, 1.5, 2, 3], [0, 0, 0, 90, 179, 179]),
                np.interp(stamps, [0, .5, 1, 1.5, 2, 3], [0, 0, 140, 160, 47, 47])])
            theta = np.radians(angle_deg)
            forward = np.array([np.cos(theta), np.sin(theta)])
            basis = np.column_stack([[-forward[1], forward[0]], forward])
            displacement = local @ basis.T
            comparison = displacement + np.asarray(translation)
            raw = pd.DataFrame({'time_ms': np.rint(stamps * 1000).astype(int),
                'ideal.rad_velo': np.where((stamps >= 1) & (stamps <= 1.5), -5., 0.),
                'ideal.velo': np.where((stamps >= .5) & (stamps <= 2), .5, 0.)})
            raw.to_csv(folder / 'motion.csv', index=False)
            trial = dict(type8_log_csv=str(folder / 'motion.csv'), preset=500,
                log_quality=dict(motion_start_time_ms=500, motion_end_time_ms=2000),
                camera_state='PASSED', camera_run_start_confirmed=True)
            (folder / 'trial.json').write_text(json.dumps(trial), encoding='utf-8')
            details = dict(trial_json=str(folder / 'trial.json'), frame_stride=1,
                observation_scope='all_frames', log_time_zero_at_video_s=0,
                required_motion_window_quality=dict(accepted_for_relative_review=True, observation_rate=1.),
                trajectory_fit_rmse_mm=.1)
            (folder / 'report.json').write_text(json.dumps(details), encoding='utf-8')
            pd.DataFrame(dict(time_log_s=stamps, center_x_mm=comparison[:, 0],
                center_y_mm=comparison[:, 1])).to_csv(folder / 'relative_video_log_comparison.csv', index=False)
            pd.DataFrame(dict(time_video_s=stamps, detected=True, center_dx_mm=displacement[:, 0],
                center_dy_mm=displacement[:, 1])).to_csv(folder / 'relative_marker_midpoint.csv', index=False)
            with patch.object(report, 'read_rows', return_value=[]), patch.object(
                    report, 'log_stop_evidence', return_value={'confirmed': True}):
                summary = report.summarize(folder)
            normalized = pd.read_csv(folder / 'normalized_trajectory.csv')
            return summary, normalized

    def test_rotation_and_camera_plane_translation_do_not_change_relative_path(self):
        baseline, base_path = self.evaluate(90, [0, 0])
        for heading in [85, 88.2, 91.8, 95, 130, 132.71648934820024, 135, 138.02399911183025, 140, -135]:
            with self.subTest(heading=heading):
                changed, path = self.evaluate(heading, [137.5, -82.3])
                np.testing.assert_allclose(path[['right_mm', 'forward_mm']],
                    base_path[['right_mm', 'forward_mm']], atol=1e-9)
                self.assertAlmostEqual(changed['endpoint_right_mm'], baseline['endpoint_right_mm'], places=9)
                self.assertAlmostEqual(changed['endpoint_forward_mm'], baseline['endpoint_forward_mm'], places=9)
                self.assertTrue(changed['auto_stop_confirmed'])

    def test_rotation_does_not_fit_away_endpoint_error_or_scale(self):
        summary, _ = self.evaluate(88.2, [20, 70])
        self.assertAlmostEqual(summary['endpoint_right_mm'], 179, places=9)
        self.assertAlmostEqual(summary['endpoint_forward_mm'], 47, places=9)
        self.assertAlmostEqual(np.hypot(summary['endpoint_right_mm'] - 180,
            summary['endpoint_forward_mm'] - 45), np.sqrt(5), places=9)
        frame = summary['trajectory_reference_frame']
        self.assertFalse(frame['scale_fitted'])
        self.assertFalse(frame['endpoint_or_exit_used_for_frame'])
        self.assertGreater(frame['entry_span_mm'], 30)
        self.assertLess(frame['entry_line_rms_mm'], 1e-9)


if __name__ == '__main__':
    unittest.main()
