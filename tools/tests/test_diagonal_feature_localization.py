"""新斜め補正の符号・因果性・飽和区間・回転補償・前センサ。"""
from pathlib import Path
from dataclasses import replace
import math,sys,unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from analysis.diagonal_edge_geometry import motion_history
from analysis.diagonal_feature_localization import (FeatureModel,PillarObservation,FrontObservation,
    pillar_value_jacobian,measurement_update,replay_confirmed,propagate,calibrate_feature_model,
    CorrectionSlew,associate_landmark,transform_axis)


class DiagonalLocalizationTests(unittest.TestCase):
    def scene(self,turn=False):
        times=np.arange(0,501,5);speed=.6
        history=motion_history(times,times*speed,[speed]*len(times),[.2 if turn else 0]*len(times))
        truth=np.array([-20.,6.,math.radians(2.)]);model=FeatureModel(offsets_mm={'1':20.,'2':20.},noise_mm={'1:point':.5,'2:point':.5})
        observations=[]
        for i,tm in enumerate((0,80,160,240,320,400)):
            pose,_=propagate(truth,np.eye(3),history,0,tm);s,e,psi=pose;side=2 if i%2==0 else 1
            sign=1 if side==1 else -1
            # Independent ray construction, emitter mounted 20mm ahead.
            rotation=np.array([[math.cos(psi),-math.sin(psi)],[math.sin(psi),math.cos(psi)]])
            emitter=pose[:2]+rotation@np.array([20.,0.])
            ray=rotation@np.array([math.cos(model.angle_rad),sign*math.sin(model.angle_rad)])
            corner=emitter+32/math.sin(model.angle_rad)*ray
            observations.append(PillarObservation(str(i),side,*corner,tm,tm+15,'point',gyro_radps=.2 if turn else 0,flags=1 if turn else 0))
        return history,truth,model,observations

    def test_known_ray_scene_recovers_three_states_without_heading_reset(self):
        h,truth,m,obs=self.scene();r=replay_confirmed(obs,h,m,[-23.,0.,0.],0,450)
        target,_=propagate(truth,np.eye(3),h,0,450)
        self.assertTrue(r['ready'])
        self.assertLess(abs(r['state'][1]-target[1]),1.)
        self.assertLess(abs(r['state'][2]-target[2]),math.radians(.5))
        self.assertGreater(abs(r['state'][2]),.01)
        self.assertGreaterEqual(np.linalg.eigvalsh(r['covariance']).min(),-1e-9)

    def test_rotation_flag_keeps_motion_compensated_observation(self):
        h,truth,m,obs=self.scene(True);r=replay_confirmed(obs,h,m,[-23.,0.,0.],0,450)
        target,_=propagate(truth,np.eye(3),h,0,450)
        self.assertGreaterEqual(r['accepted'],4)
        self.assertLess(abs(r['state'][1]-target[1]),1.5)
        self.assertGreater(r['state'][2],.05)

    def test_future_confirmation_is_not_used(self):
        h,_,m,obs=self.scene();a=replay_confirmed(obs,h,m,[-23.,0.,0.],0,250)
        b=replay_confirmed(obs[:3],h,m,[-23.,0.,0.],0,250)
        np.testing.assert_allclose(a['state'],b['state'])
        self.assertLessEqual(a['maximum_confirmation_ms'],250)

    def test_saturation_interval_does_not_pull_to_midpoint_or_shrink_inside(self):
        m=FeatureModel(offsets_mm={'1':20},noise_mm={'1:point':1})
        o=PillarObservation('flat',1,20,0,0,5,'point',20,True)
        x=np.array([3.,0.,.02]);p=np.diag([2.,2.,.01])
        y,q,r=measurement_update(x,p,o,m)
        self.assertEqual(r['policy'],'interval_contains_prediction')
        np.testing.assert_array_equal(x,y);np.testing.assert_array_equal(p,q)

    def test_unbounded_saturation_and_coordinate_jump_are_rejected(self):
        m=FeatureModel();x=np.zeros(3);p=np.eye(3)
        for o in (PillarObservation('clip',1,20,0,0,5,saturated=True),PillarObservation('jump',1,20,0,0,5,flags=4)):
            y,q,r=measurement_update(x,p,o,m);self.assertFalse(r['accepted']);np.testing.assert_array_equal(y,x)

    def test_topology_calibration_is_side_specific_and_shrinks_small_groups(self):
        rows=[dict(side=side,surface=kind,forward_mm=offset+32/math.tan(math.radians(55)),side_mm=32.)
              for side,kind,offset in [(1,'single',15),(1,'double',30),(2,'single',22),(2,'single',22)]]
        model,audit=calibrate_feature_model(rows)
        self.assertNotEqual(model.offset(1,'single'),model.offset(1,'double'))
        self.assertAlmostEqual(model.offset(2,'single'),22.)
        self.assertGreaterEqual(model.noise(1,'double'),6.)

    def front(self,side,calibrated=True,state=0):
        return FrontObservation('front'+str(side),0,0,80.,state,1.,0.,100.,20.,side*15.,0.,.1,calibrated)

    def test_front_pair_updates_angle_only_when_calibrated_and_metric(self):
        m=FeatureModel();x=np.array([0.,4.,.04]);p=np.diag([4.,9.,.01])
        y,q,r=measurement_update(x,p,self.front(1,False),m);np.testing.assert_array_equal(x,y)
        y,q,r=measurement_update(x,p,self.front(1,True,2),m);np.testing.assert_array_equal(x,y)
        for side in (1,-1):x,p,r=measurement_update(x,p,self.front(side),m)
        self.assertLess(abs(x[2]),.005);self.assertAlmostEqual(x[1],4.)

    def test_invalid_front_geometry_cannot_enter_filter(self):
        for change in ({'distance_mm':float('nan')},{'normal_s':0.},{'sigma_mm':0.},
                       {'confirmation_ms':-1.}):
            with self.assertRaises(ValueError):replace(self.front(1),**change)

    def test_skipped_pillar_association_uses_map_side_and_order(self):
        m=FeatureModel(offsets_mm={'1':20},noise_mm={'1:point':1});x=np.array([120.,0.,0.]);p=np.diag([4.,4.,.001])
        obs=PillarObservation('unknown',1,0,0,0,5,'point')
        landmarks=[dict(id=str(i),side=1,s_mm=20+60*i,e_mm=0,surface='point',progress_mm=60*i) for i in range(4)]
        r=associate_landmark(x,p,obs,landmarks,m,last_progress=0)
        self.assertTrue(r['accepted']);self.assertEqual(r['observation'].label,'2')

    def test_ambiguous_association_does_not_force_a_landmark(self):
        m=FeatureModel(offsets_mm={'1':20},noise_mm={'1:point':1});x=np.array([30.,0.,0.]);p=np.diag([1000.,4.,.001])
        obs=PillarObservation('unknown',1,0,0,0,5,'point')
        landmarks=[dict(id=str(i),side=1,s_mm=20+60*i,e_mm=0,surface='point',progress_mm=60*i) for i in range(2)]
        self.assertEqual(associate_landmark(x,p,obs,landmarks,m)['reason'],'ambiguous_landmark')

    def test_control_slew_obeys_both_time_and_travel_limits(self):
        slew=CorrectionSlew();e,psi=slew.update(10.,.2,.01,1.)
        self.assertAlmostEqual(e,.1);self.assertAlmostEqual(psi,math.radians(.025))
        e2,p2=slew.update(10.,.2,.1,0.);self.assertEqual((e2,p2),(e,psi))

    def test_exit_axis_change_preserves_pose_and_covariance(self):
        x=np.array([120.,8.,math.radians(47.)]);p=np.array([[4.,1.,.01],[1.,9.,.02],[.01,.02,.001]])
        y,q=transform_axis(x,p,math.pi/4,(10.,-30.))
        recovered,r=transform_axis(y,q,-math.pi/4,-np.array([[1,1],[-1,1]])@np.array([10.,-30.])/math.sqrt(2))
        np.testing.assert_allclose(recovered,x,atol=1e-10)
        np.testing.assert_allclose(r,p,atol=1e-10)
        self.assertAlmostEqual(math.degrees(y[2]),2.)
        self.assertNotEqual(y[1],0.)

    def test_unknown_common_offset_is_not_reported_as_calibrated_progress(self):
        h,truth,m,obs=self.scene()
        model=FeatureModel(m.angle_rad,m.offsets_mm,m.noise_mm,10.,False)
        result=replay_confirmed(obs,h,model,[-23.,0.,0.],0,450)
        self.assertEqual(len(result['state']),4)
        self.assertTrue(result['lateral_heading_ready'])
        self.assertFalse(result['longitudinal_ready'])
        self.assertGreater(result['std_s_mm'],8.)
        self.assertGreaterEqual(np.linalg.eigvalsh(result['covariance']).min(),-1e-9)

    def test_calibrated_front_anchor_can_resolve_common_phase_uncertainty(self):
        h,truth,m,obs=self.scene()
        model=FeatureModel(m.angle_rad,m.offsets_mm,m.noise_mm,10.,False)
        pose,_=propagate(truth,np.eye(3),h,0,450)
        # Independently construct an axial range to a wall beyond the robot.
        wall_s=pose[0]+100.
        distance=(wall_s-pose[0])/math.cos(pose[2])
        front=FrontObservation('known_wall',450,450,distance,0,1,0,wall_s,0,0,0,.5,True)
        result=replay_confirmed([*obs,front],h,model,[-23.,0.,0.],0,450)
        self.assertTrue(result['longitudinal_ready'])
        self.assertLess(result['std_s_mm'],2.)
        self.assertEqual(result['updates'][-1]['kind'],'front')

    def test_lateral_only_front_plane_cannot_certify_progress(self):
        h,truth,m,obs=self.scene()
        model=FeatureModel(m.angle_rad,m.offsets_mm,m.noise_mm,10.,False)
        pose,_=propagate(truth,np.eye(3),h,0,450)
        wall_e=pose[1]+100.
        front=FrontObservation('lateral_wall',450,450,100/math.cos(pose[2]),0,0,1,wall_e,0,0,math.pi/2,.5,True)
        result=replay_confirmed([*obs,front],h,model,[-23.,0.,0.],0,450)
        self.assertTrue(result['updates'][-1]['accepted'])
        self.assertFalse(result['longitudinal_ready'])


if __name__=='__main__':unittest.main()
