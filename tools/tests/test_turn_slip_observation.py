"""滑りの観測解析における微分・投影補正・時刻・位置ずれの検証。"""
from pathlib import Path
import sys
import unittest

import numpy as np
import pandas as pd
from scipy.integrate import cumulative_trapezoid

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"analysis"))
import turn_slip_observation as observation


class SlipObservationTest(unittest.TestCase):
    def test_local_derivatives_use_real_irregular_timestamps(self):
        t=np.arange(201)*.004+np.sin(np.arange(201))*.0003
        xy=np.column_stack([4*t**3+2*t**2+3*t,7*t**2-2*t])
        result=observation.local_polynomial(t,xy,np.ones(len(t),bool))
        indices=np.arange(10,190)
        np.testing.assert_allclose(result[indices,1,0],12*t[indices]**2+4*t[indices]+3,atol=1e-9)
        np.testing.assert_allclose(result[indices,2,1],14,atol=1e-8)

    def test_derivative_does_not_bridge_long_missing_interval(self):
        t=np.arange(201)*.004
        valid=np.ones(len(t),bool); valid[95:106]=False
        result=observation.local_polynomial(t,np.column_stack([t,t*t]),valid)
        self.assertTrue(np.isnan(result[100]).all())
        self.assertTrue(np.isnan(result[94]).all())
        with self.assertRaises(ValueError):
            observation.local_polynomial(t[::-1],np.column_stack([t,t]),valid)

    def test_unknown_parallax_offset_is_recovered_from_rigid_rotation(self):
        angle=np.linspace(-.2,2.5,150)
        centre=np.array([4.2,-3.5]); radius=54.2
        vectors=centre+radius*np.column_stack([np.sin(angle),np.cos(angle)])
        fit=observation.fit_marker_circle(vectors)
        np.testing.assert_allclose(fit["center"],centre,atol=1e-8)
        self.assertAlmostEqual(fit["radius_mm"],radius,places=8)
        self.assertLess(fit["rms_mm"],1e-8)

    def test_short_arc_is_less_identifiable_than_long_arc(self):
        def condition(span):
            angle=np.linspace(0,span,100)
            return observation.fit_marker_circle(54*np.column_stack([np.sin(angle),np.cos(angle)]))["condition"]
        self.assertGreater(condition(np.pi/4),5*condition(np.pi))
        with self.assertRaises(ValueError):
            observation.fit_marker_circle(np.ones((20,2)))

    def test_heading_alignment_recovers_independent_time_offset(self):
        t=np.arange(-.2,1.201,.001)
        omega=10*np.sin(np.clip((t-.2)/.5,0,1)*np.pi)**2
        omega[(t<.2)|(t>.7)]=0
        raw=pd.DataFrame(dict(elapsed_s=t,**{"ego.rad_velo":-omega}))
        angle=cumulative_trapezoid(omega,t,initial=0)
        target=.985*np.interp(t+.013,t,angle)+.04
        heading=dict(gyro_bias_rad_s=0.,gyro_gain=.98,moving=np.ones(len(t),bool),turn_first_s=.2,turn_last_s=.7)
        fit=observation.fit_heading_time(t,dict(heading=target),heading,raw)
        self.assertAlmostEqual(fit["shift_s"],.013,delta=1e-5)
        self.assertAlmostEqual(fit["gain"],.985,delta=1e-5)
        self.assertLess(fit["rmse_deg"],1e-4)

    def test_marker_offset_can_create_apparent_slip_without_axle_slip(self):
        t=np.arange(-.2,1.201,.002)
        omega=10*np.sin(np.clip((t-.2)/.5,0,1)*np.pi)**2
        omega[(t<.2)|(t>.7)]=0
        raw=pd.DataFrame(dict(elapsed_s=t,**{"ego.rad_velo":-omega,"ego.velo":np.full(len(t),.5)}))
        beta=np.arctan(-.005*omega/.5)
        heading=dict(gyro_bias_rad_s=0.,gyro_gain=1.,fit_mask=omega>1,beta=beta)
        result=observation.compare_point_offset(t,heading,raw)
        self.assertAlmostEqual(result["point_only_offset_mm"],-5,delta=.001)
        self.assertLess(result["point_only_residual_deg"],1e-4)


if __name__=="__main__":
    unittest.main()
