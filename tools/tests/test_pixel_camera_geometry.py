"""高さの異なる目印を既知の光線で検証し、未確定の幾何を黙って適用しない。"""
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pixel_camera_geometry import (correct_height, apparent_floor, correct_body_markers,
                                   validate_geometry, pose_from_corrected_markers, geometry_for_measurement,
                                   relative_marker_midpoint)


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.camera = [140, 160, 600]
        self.geometry = dict(camera_center_board_mm=self.camera, front_height_mm=10, rear_height_mm=1,
                             axle_offset_from_marker_midpoint_mm=None)

    def test_different_heights_recover_true_positions_and_heading(self):
        front, rear = np.array([240., 270.]), np.array([240., 215.])
        # カメラから高さhの点への直線を床z=0まで延長した画像上の見かけ位置。
        floor_front = np.array(self.camera[:2])+(front-self.camera[:2])*600/590
        floor_rear = np.array(self.camera[:2])+(rear-self.camera[:2])*600/599
        result = correct_body_markers(floor_front, floor_rear, self.geometry)
        np.testing.assert_allclose(result['front_xy_mm'], front)
        np.testing.assert_allclose(result['rear_xy_mm'], rear)
        self.assertAlmostEqual(result['heading_rad'], np.pi/2)
        self.assertIsNone(result['axle_xy_mm'])
        self.assertEqual(result['center_kind'], 'marker_midpoint')
        self.assertGreater(np.linalg.norm(floor_front-front), 2)

    def test_axle_offset_uses_forward_and_physical_left_in_image_axes(self):
        self.geometry['axle_offset_from_marker_midpoint_mm'] = [-5, 2]
        result = pose_from_corrected_markers([200, 250], [200, 200], self.geometry)
        np.testing.assert_allclose(result['axle_xy_mm'], [202, 220])

    def test_zero_height_and_projection_round_trip(self):
        points = np.array([[40., 100.], [300., 280.]])
        np.testing.assert_allclose(correct_height(points, 0, self.camera), points)
        np.testing.assert_allclose(apparent_floor(correct_height(points, 10, self.camera), 10, self.camera), points)

    def test_heights_alone_do_not_enable_an_unmeasured_camera_model(self):
        for value in (None, [600], [180, 180, 0], [180, np.nan, 600]):
            with self.subTest(value=value), self.assertRaises(ValueError):
                validate_geometry(dict(self.geometry, camera_center_board_mm=value))

    def test_impossible_heights_and_axle_geometry_are_rejected(self):
        for height in (-1, 600, np.nan):
            with self.subTest(height=height), self.assertRaises(ValueError):
                correct_height([10, 20], height, self.camera)
        with self.assertRaises(ValueError):
            validate_geometry(dict(self.geometry, axle_offset_from_marker_midpoint_mm=[1]))
        with self.assertRaises(ValueError):
            pose_from_corrected_markers([10, 10], [10, 10], self.geometry)

    def test_camera_height_is_resolved_separately_for_each_measurement(self):
        self.geometry['camera_height_policy'] = 'per_measurement'
        first = geometry_for_measurement(self.geometry, 600)
        second = geometry_for_measurement(self.geometry, 800)
        np.testing.assert_allclose(first['camera_center_board_mm'], [140, 160, 600])
        np.testing.assert_allclose(second['camera_center_board_mm'], [140, 160, 800])
        self.assertNotIn('camera_height_measurement_confirmed', self.geometry)
        delta = (correct_height([340, 160], 10, second['camera_center_board_mm'])
                 - correct_height([340, 160], 10, first['camera_center_board_mm']))
        self.assertAlmostEqual(delta[0], 5/6)

    def test_old_height_is_not_silently_reused_for_a_new_measurement(self):
        self.geometry.update(camera_height_policy='per_measurement', camera_height_mm=800)
        with self.assertRaisesRegex(ValueError, '測定ごと'):
            geometry_for_measurement(self.geometry)
        with self.assertRaisesRegex(ValueError, '測定ごと'):
            validate_geometry(self.geometry)

    def test_height_without_camera_foot_is_not_replaced_by_image_centre(self):
        geometry = dict(self.geometry, camera_center_board_mm=None)
        with self.assertRaisesRegex(ValueError, '真下'):
            geometry_for_measurement(geometry, 800)
        resolved = geometry_for_measurement(geometry, 800, [120, 140])
        np.testing.assert_allclose(resolved['camera_center_board_mm'], [120, 140, 800])

    def test_symmetric_25mm_markers_recover_axle_after_individual_height_correction(self):
        geometry = dict(self.geometry, axle_offset_from_marker_midpoint_mm=[0, 0])
        front, rear = np.array([240., 265.]), np.array([240., 215.])
        front_floor = apparent_floor(front, 10, self.camera)
        rear_floor = apparent_floor(rear, 1, self.camera)
        result = correct_body_markers(front_floor, rear_floor, geometry)
        np.testing.assert_allclose(result['axle_xy_mm'], [240, 240])
        self.assertAlmostEqual(result['marker_separation_mm'], 50)

    def test_relative_midpoint_cancels_unknown_camera_foot_during_half_turn(self):
        angles = np.linspace(0, np.pi, 121)
        unit = np.column_stack([np.cos(angles), np.sin(angles)])
        centres = [150, 150]+40*unit
        front, rear = centres+25*unit, centres-25*unit
        for height in (600, 800):
            for foot in ([0, 0], [180, 180], [-150, 700]):
                with self.subTest(height=height, foot=foot):
                    camera = [*foot, height]
                    result = relative_marker_midpoint(apparent_floor(front, 10, camera),
                        apparent_floor(rear, 1, camera), height)
                    np.testing.assert_allclose(result['displacement_xy_mm'], centres-centres[0], atol=1e-10)
                    self.assertFalse(result['absolute_position_available'])
                    self.assertFalse(result['height_corrected_heading_available'])

    def test_relative_midpoint_does_not_fill_invalid_observations(self):
        result = relative_marker_midpoint([[0, 0], [1, 1], [2, 2]], [[0, 0], [1, 1], [2, 2]],
                                         800, valid=[False, True, False])
        self.assertEqual(result['origin_index'], 1)
        self.assertTrue(np.isnan(result['displacement_xy_mm'][[0, 2]]).all())
        with self.assertRaises(ValueError):
            relative_marker_midpoint([[0, 0]], [[0, 0]], 10)


if __name__ == '__main__':
    unittest.main()
