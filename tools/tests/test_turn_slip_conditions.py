from pathlib import Path
import math
import sys
import unittest

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"analysis"))
import turn_slip_conditions as slip

PROFILES = {"suction_off": {"k_m_s2":45.,"gyro_gain":.99,"ay_std_m_s2":.8},
            "suction_on": {"k_m_s2":250.,"gyro_gain":.99,"ay_std_m_s2":5.}}


class ConditionsTests(unittest.TestCase):
    def test_force_fit_recovers_coefficient_with_outliers(self):
        beta=-np.sin(np.linspace(0,math.pi,80))*.15
        ay=-180*beta+.12
        ay[20]=100
        result=slip.force_fit(beta,ay,np.ones(80,bool))
        self.assertAlmostEqual(result["k_m_s2"],180,delta=3.)
        self.assertFalse(result["at_bound"])

    def test_no_excitation_is_not_a_calibration(self):
        with self.assertRaises(ValueError):
            slip.force_fit(np.zeros(20),np.zeros(20),np.ones(20,bool))

    def test_slip_pose_jacobian_matches_numerical_derivative(self):
        x=np.array([.1,.2,.7,-.12,.02,.1])
        _,f=slip.slip_motion(x,1.6,25.,.005,250.,.99)
        columns=[]
        for i in range(6):
            delta=np.zeros(6);delta[i]=1e-6
            columns.append((slip.slip_motion(x+delta,1.6,25.,.005,250.,.99)[0]
                            -slip.slip_motion(x-delta,1.6,25.,.005,250.,.99)[0])/2e-6)
        np.testing.assert_allclose(f,np.array(columns).T,atol=1e-9)

    def test_actual_suction_selects_coefficients_at_same_velocity(self):
        estimates=[]
        for suction in (False,True):
            f=slip.ConditionedTurnEkf(suction,PROFILES)
            for _ in range(200):
                f.predict(.5,10.,.005)
            estimates.append(abs(f.state[3]))
        self.assertGreater(estimates[0],estimates[1]*5)
        self.assertAlmostEqual(estimates[0],.5*9.9/45,places=7)
        with self.assertRaises(ValueError):
            slip.ConditionedTurnEkf(None,PROFILES)

    def test_acceleration_updates_and_covariance_remain_finite(self):
        f=slip.ConditionedTurnEkf(True,PROFILES,gyro_bias=.02,accel_bias=.1)
        for _ in range(200):
            f.predict(1.6,25.02,.005)
            self.assertTrue(f.observe_acceleration(1.6*25*.99+.1))
        self.assertLess(f.state[3],0)
        self.assertGreaterEqual(np.linalg.eigvalsh(f.covariance).min(),-1e-12)
        before=f.state.copy()
        self.assertFalse(f.observe_acceleration(1e5))
        np.testing.assert_array_equal(before,f.state)


if __name__=="__main__":
    unittest.main()
