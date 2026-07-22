import sys
import unittest
from pathlib import Path

import numpy as np


ANALYSIS_DIR = Path(__file__).resolve().parents[1] / "analysis"
TOOLS_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ANALYSIS_DIR))
sys.path.insert(0, str(TOOLS_DIR))

from turn1600_variable_speed_lengths import repair_terminal_beta_reset  # noqa: E402


class TerminalBetaResetTest(unittest.TestCase):
    def test_forced_zero_is_replaced_by_continuous_beta_step(self):
        beta = np.array([-0.10, -0.09, 0.0])
        velocity = np.array([1.6, 1.6, 1.6])
        yaw_rate = np.array([2.0, 0.5, -3.0])

        repaired = repair_terminal_beta_reset(beta, velocity, yaw_rate, 250.0)

        expected = -0.09 + (-250.0 * -0.09 / 1.6 - -3.0) * 0.001
        self.assertAlmostEqual(repaired[-1], expected)
        self.assertNotEqual(repaired[-1], 0.0)
        np.testing.assert_array_equal(beta, [-0.10, -0.09, 0.0])

    def test_physical_small_terminal_beta_is_not_changed(self):
        beta = np.array([-0.01, 0.0])
        velocity = np.array([1.6, 1.6])
        yaw_rate = np.array([0.2, 0.0])

        repaired = repair_terminal_beta_reset(beta, velocity, yaw_rate, 250.0)

        np.testing.assert_array_equal(repaired, beta)


if __name__ == "__main__":
    unittest.main()
