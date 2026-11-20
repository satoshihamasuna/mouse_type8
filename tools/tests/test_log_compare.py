"""新ログの姿勢2列によって、既存の実測軌跡が非表示にならないことを検証する。"""
from pathlib import Path
import sys
import unittest

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'analysis'))
from log_compare import COMPARISONS, plot_comparisons


class LogCompareTests(unittest.TestCase):
    def test_ego_trajectory_survives_missing_old_ideal_slots_and_nan_headings(self):
        data = {name: np.zeros(5) for target, measured, _, _ in COMPARISONS for name in (target, measured)}
        data.update({'ego.turn_x': np.arange(5), 'ego.turn_y': np.arange(5)*2,
                     'estimate.heading_rad': np.full(5, np.nan),
                     'estimate.sigma_heading_rad': np.full(5, np.nan)})
        for legacy in (False, True):
            with self.subTest(legacy=legacy):
                frame = pd.DataFrame(data)
                if legacy:
                    frame = frame.drop(columns=['estimate.heading_rad', 'estimate.sigma_heading_rad'])
                    frame['ideal.turn_x'] = frame['ideal.turn_y'] = 0
                figure = plot_comparisons(frame, 'synthetic.csv')
                try:
                    lines = {line.get_label(): line for line in figure.axes[6].get_lines()}
                    self.assertIn('measured (logged position)', lines)
                    self.assertEqual('target (logged position)' in lines, legacy)
                    np.testing.assert_array_equal(lines['measured (logged position)'].get_xdata(), data['ego.turn_x'])
                finally:
                    plt.close(figure)


if __name__ == '__main__':
    unittest.main()
