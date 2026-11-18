"""画像を入力にしない斜め特徴点モデルの符号・可観測性・距離連続性。"""
from pathlib import Path
import math
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from analysis.diagonal_edge_geometry import (Geometry, Observation, fit, jacobian,
    motion_history, point_exclusion, residuals, state_at, ekf_replay, constrain_gyro_integrals)


class DiagonalGeometryTests(unittest.TestCase):
    def scene(self, theta=55, sides=(2, 1, 2, 1, 2, 1)):
        angle = math.radians(theta); geometry = Geometry(angle, 20.)
        initial = np.array([-21., 5., math.radians(2.)]); psi = initial[2]
        direction = np.array([math.cos(psi), math.sin(psi)])
        rotation = np.array([[math.cos(psi), -math.sin(psi)], [math.sin(psi), math.cos(psi)]])
        observations = []
        for i, side in enumerate(sides):
            sign = 1 if side == 1 else -1
            corner = np.array([31.819805153+63.639610307*i, sign*31.819805153])
            # Independent construction: a=25mm, choose b so A=a-b*cot(theta)=20.
            sensor = initial[:2]+rotation@np.array([25., sign*5*math.tan(angle)])
            ray = rotation@np.array([math.cos(angle), sign*math.sin(angle)])
            cross = lambda a, b: a[0]*b[1]-a[1]*b[0]
            travel = cross(corner-sensor, ray)/cross(direction, ray)
            observations.append(Observation(str(i), side, *corner, travel, 0, 0, 1.))
        return initial, observations, geometry

    def test_known_ray_scene_recovers_position_and_signed_heading(self):
        initial, observations, geometry = self.scene()
        result = fit(observations, geometry)
        self.assertTrue(result['valid'])
        np.testing.assert_allclose(result['initial'], initial, atol=1e-7)

    def test_negative_lateral_and_heading_have_correct_sign(self):
        initial, observations, geometry = self.scene()
        changed = [Observation(o.label, 3-o.side, o.corner_x_mm, -o.corner_y_mm,
            o.displacement_x_mm, -o.displacement_y_mm, -o.gyro_angle_rad) for o in observations]
        result = fit(changed, geometry)
        self.assertTrue(result['valid'])
        np.testing.assert_allclose(result['initial'], initial*[1, -1, -1], atol=1e-7)

    def test_one_left_right_pair_is_insufficient_for_three_unknowns(self):
        _, observations, geometry = self.scene()
        self.assertEqual(fit(observations[:2], geometry)['reason'], 'fewer_than_three_events')
        self.assertTrue(fit(observations[:3], geometry)['valid'])

    def test_perpendicular_point_sensor_cannot_resolve_absolute_lateral(self):
        _, observations, geometry = self.scene(theta=90)
        self.assertEqual(fit(observations, geometry)['reason'], 'rank_deficient')

    def test_one_sided_events_remain_rank_deficient_without_lateral_reference(self):
        _, observations, geometry = self.scene(sides=(1, 1, 1, 1))
        self.assertEqual(fit(observations, geometry)['reason'], 'rank_deficient')

    def test_jacobian_matches_independent_finite_differences(self):
        initial, observations, geometry = self.scene()
        numeric = np.column_stack([(residuals(initial+np.eye(3)[j]*1e-6, observations, geometry)
            -residuals(initial-np.eye(3)[j]*1e-6, observations, geometry))/2e-6 for j in range(3)])
        np.testing.assert_allclose(jacobian(initial, observations, geometry), numeric, atol=1e-6)

    def test_frame_reset_does_not_become_backward_travel(self):
        h = motion_history([0, 5, 10, 15], [24, 27, 1, 4], [.6]*4, [0]*4)
        np.testing.assert_allclose(h['distance_mm'], [0, 3, 6, 9])
        self.assertEqual(len(h['reset_bridges']), 1)

    def test_missing_samples_are_not_silently_integrated(self):
        with self.assertRaises(ValueError):
            motion_history([0, 5, 20], [0, 3, 12], [.6]*3, [0]*3)

    def test_floor_and_motion_flags_are_rejected_as_point_observations(self):
        row = dict(flags='0', side='2', sr_mm='25', adc_r='3990')
        self.assertEqual(point_exclusion(row), 'side_floor_or_adc_saturation')
        row.update(sr_mm='40', flags='1')
        self.assertEqual(point_exclusion(row), 'firmware_quality_flags_1')
        row.update(flags='0')
        self.assertEqual(point_exclusion(row), '')

    def test_propagation_rotates_displacement_without_heading_reset(self):
        pose = state_at([3, 4, math.pi/2], 10, 0, .1)
        np.testing.assert_allclose(pose, [3, 14, math.pi/2+.1], atol=1e-12)

    def test_ekf_never_uses_an_unconfirmed_future_event(self):
        geometry = Geometry(math.radians(55), 20)
        h = motion_history([0, 5, 10, 15, 20, 25, 30], [0, 3, 6, 9, 12, 15, 18], [.6]*7, [0]*7)
        obs = []
        for i, (feature, confirmation) in enumerate([(0, 5), (10, 15), (20, 25)]):
            side = 1 if i%2 else 2
            obs.append(dict(observation=Observation(str(i), side, 10+i*6, (-1 if side==2 else 1)*20, 0, 0, 0),
                            feature_ms=feature, confirmation_ms=confirmation))
        before = ekf_replay(obs, h, geometry, [0, 0, 0], 0, 5, 24)
        without_future = ekf_replay(obs[:2], h, geometry, [0, 0, 0], 0, 5, 24)
        self.assertEqual(before['considered'], 2)
        self.assertLessEqual(before['maximum_confirmation_ms'], 24)
        np.testing.assert_allclose(before['state'], without_future['state'])
        self.assertGreaterEqual(np.linalg.eigvalsh(before['covariance']).min(), -1e-10)

    def test_ekf_initial_snapshot_is_unavailable_before_confirmation(self):
        h = motion_history([0, 5, 10], [0, 3, 6], [.6]*3, [0]*3)
        self.assertIsNone(ekf_replay([], h, Geometry(math.radians(55), 20), [0, 0, 0], 0, 5, 4))

    def test_integral_constraint_waits_for_snapshot_confirmation(self):
        h = motion_history([0, 5, 10, 15, 20, 25, 30], [0, 3, 6, 9, 12, 15, 18], [.6]*7, [1]*7)
        snaps=[dict(minimum_ms=0,confirmation_ms=5,angle_rad=.5),dict(minimum_ms=20,confirmation_ms=25,angle_rad=.51)]
        early=constrain_gyro_integrals(h,snaps,24)
        np.testing.assert_allclose(early['gamma_rad'],h['gamma_rad'])
        late=constrain_gyro_integrals(h,snaps,26)
        self.assertAlmostEqual(np.interp(20,late['time_ms'],late['gamma_rad']),.01)
        self.assertAlmostEqual(np.interp(25,late['time_ms'],late['gamma_rad']),.015)
        np.testing.assert_allclose(h['gamma_rad'],np.arange(7)*.005)


if __name__ == '__main__':
    unittest.main()
