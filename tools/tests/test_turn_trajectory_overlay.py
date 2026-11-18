"""軌跡比較が個別の経路合わせ・尺度合わせで差を消さないことを確認する。"""
from pathlib import Path
import math
import sys
import unittest

import numpy as np
import pandas as pd

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"analysis"))
from turn_trajectory_overlay import integrate_simple,interpolate_valid,video_reference,error_metrics


class OverlayTests(unittest.TestCase):
    def test_simple_circle_uses_actual_time_and_left_yaw(self):
        time=np.arange(1001)*.001
        xy=integrate_simple(time,np.full(len(time),.5),np.full(len(time),2.))
        expected=np.array([math.sin(2),1-math.cos(2)])*.25*1000
        np.testing.assert_allclose(xy[-1],expected,atol=.0001)

    def test_known_stationary_gyro_bias_is_removed(self):
        time=np.array([0.,.006,.011,.019])
        xy=integrate_simple(time,np.full(4,.5),np.full(4,.04),bias=.04)
        np.testing.assert_allclose(xy[-1],[9.5,0.],atol=1e-12)

    def test_no_extrapolation_or_interpolation_across_video_loss(self):
        time=np.array([0.,.005,.05,.055])
        xy=np.column_stack([time,time*2])
        result=interpolate_valid(time,xy,[-.01,.0025,.02,.05,.06])
        self.assertTrue(np.isnan(result[[0,2,4]]).all())
        np.testing.assert_allclose(result[1],[.0025,.005])
        np.testing.assert_allclose(result[3],[.05,.1])

    def test_video_origin_entry_direction_and_handedness_only(self):
        time=np.arange(100)*.005
        local=np.column_stack([time*500,np.maximum(time-.38,0)**2*400])
        angle=.7
        for hand in (1.,-1.):
            basis=np.array([[math.cos(angle),-hand*math.sin(angle)],
                            [math.sin(angle),hand*math.cos(angle)]])
            source=local @ basis.T+[120.,260.]
            frame=pd.DataFrame(dict(log_time_s=time,time_s=time,detected=True,course_rad=angle,
                                    center_x_mm=source[:,0],center_y_mm=source[:,1]))
            mapped,_,report=video_reference(frame,0.,.4,hand)
            np.testing.assert_allclose(mapped,local,atol=1e-10)
            self.assertFalse(report['scale_fitted'])
            self.assertFalse(report['separate_estimator_alignment'])
            frame.loc[99,'center_x_mm']+=100
            remapped,_,again=video_reference(frame,0.,.4,hand)
            np.testing.assert_allclose(remapped[:99],mapped[:99],atol=1e-10)
            self.assertEqual(report,again)

    def test_position_errors_are_not_fit_away(self):
        time=np.arange(100)*.005
        video=np.column_stack([time*500,time*0])
        predicted=video+[6.,8.]
        metrics,error=error_metrics(time,video,time,predicted,[.1,.4])
        self.assertAlmostEqual(metrics['rmse_mm'],10.)
        self.assertAlmostEqual(metrics['turn_rmse_mm'],10.)
        self.assertAlmostEqual(metrics['endpoint_mm'],10.)
        np.testing.assert_allclose(error,10.)

    def test_discontinuous_integration_times_are_rejected(self):
        with self.assertRaises(ValueError):
            integrate_simple([0.,.005,.005],[.5,.5,.5],[0.,0.,0.])


if __name__=='__main__':
    unittest.main()
