import sys
import unittest
from pathlib import Path

import numpy as np


ANALYSIS_DIR = Path(__file__).resolve().parents[1] / "analysis"
sys.path.insert(0, str(ANALYSIS_DIR))

from odometry_compare import integrate_odometry  # noqa: E402


class OdometryCompareTest(unittest.TestCase):
    def test_straight_integration(self):
        x, y, theta = integrate_odometry(np.ones(10), np.zeros(10))
        self.assertAlmostEqual(x[-1], 0.0)
        self.assertAlmostEqual(y[-1], 10.0)
        self.assertAlmostEqual(theta[-1], 0.0)

    def test_midpoint_arc_integration(self):
        count = 1000
        dtheta = np.full(count, np.pi / 2 / count)
        radius = 50.0
        ds = radius * dtheta
        x, y, theta = integrate_odometry(ds, dtheta)
        self.assertAlmostEqual(x[-1], radius, places=4)
        self.assertAlmostEqual(y[-1], radius, places=4)
        self.assertAlmostEqual(theta[-1], np.pi / 2, places=12)


if __name__ == "__main__":
    unittest.main()
