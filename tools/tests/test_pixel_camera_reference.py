"""任意の2枚以上の対応頂点による射影補正。カメラや機体には接続しない。"""
from itertools import combinations
from pathlib import Path
import sys
import unittest

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pixel_camera_reference import (create_marker_reference, detect_board_reference, register_marker_reference,
    rectify_board, fit_grid_reference, refine_marker_vertices)


class ReferenceTests(unittest.TestCase):
    def setUp(self):
        self.ids = [11, 23, 31, 44]
        self.quads = {}
        for index, (marker_id, (x, y)) in enumerate(zip(self.ids, [(60, 60), (610, 60), (60, 610), (610, 610)])):
            quad = np.float64([[x, y], [x+29, y], [x+29, y+29], [x, y+29]])
            self.quads[marker_id] = np.roll(quad, index, axis=0)
        self.board = dict(TL=[100, 100], TR=[600, 100], BR=[600, 600], BL=[100, 600])
        inspection = dict(corners=self.pack(self.quads), board_reference=dict(
            available=True, method='test_reference', points_px=self.board, metric_accuracy_verified=False))
        self.profile = create_marker_reference(inspection, image_size=(800, 800),
            board_corner_ids=dict(TL=11, TR=23, BL=31, BR=44))
        self.warp = np.float64([[.95, .04, 30], [-.03, 1.01, 24], [.00008, -.00004, 1]])
        self.current = {key: self.transform(value, self.warp) for key, value in self.quads.items()}

    def pack(self, quads):
        return {'arbitrary_roi_names': dict(ids=list(quads), corners_px=[value.tolist() for value in quads.values()])}

    def transform(self, points, matrix):
        return cv2.perspectiveTransform(np.float64(points).reshape(-1, 1, 2), matrix).reshape(-1, 2)

    def test_every_pair_of_positions_works_without_grid_or_special_corner(self):
        expected = self.transform(list(self.board.values()), self.warp)
        for pair in combinations(self.ids, 2):
            with self.subTest(pair=pair):
                markers = self.pack({key: self.current[key] for key in pair})
                result = detect_board_reference(np.zeros((800, 800, 3), np.uint8), markers, self.profile)
                self.assertTrue(result['available'], result)
                self.assertEqual(result['used_marker_ids'], sorted(pair))
                self.assertEqual(result['inlier_count'], 8)
                np.testing.assert_allclose(list(result['points_px'].values()), expected, atol=.005)

    def test_three_markers_use_twelve_vertices(self):
        result = register_marker_reference(self.pack({key: self.current[key] for key in self.ids[:3]}), self.profile, (800, 800))
        self.assertTrue(result['available'])
        self.assertEqual(result['correspondence_count'], 12)
        self.assertEqual(result['inlier_count'], 12)
        self.assertEqual(result['reference_id'], self.profile['reference_id'])
        self.assertFalse(result['metric_accuracy_verified'])

    def test_accuracy_evaluation_is_carried_separately_from_fit_error(self):
        self.profile['metric_accuracy_evaluated'] = True
        self.profile['accuracy_evaluation'] = dict(scope='board surface', cases={'11,23': dict(position_rmse_mm=2)})
        result = register_marker_reference(self.pack({11: self.current[11], 23: self.current[23]}), self.profile)
        self.assertTrue(result['metric_accuracy_evaluated'])
        self.assertEqual(result['accuracy_evaluation']['cases']['11,23']['position_rmse_mm'], 2)
        self.assertFalse(result['metric_accuracy_verified'])

    def test_missing_or_unknown_markers_do_not_reuse_a_previous_transform(self):
        for quads in ({}, {11: self.current[11]}, {98: self.current[11], 99: self.current[23]}):
            with self.subTest(ids=list(quads)):
                result = register_marker_reference(self.pack(quads), self.profile)
                self.assertFalse(result['available'])
                self.assertNotIn('homography_current_to_reference', result)

    def test_duplicate_or_nonfinite_corners_rejected(self):
        duplicate = {'one': self.pack({11: self.current[11]})['arbitrary_roi_names'],
                     'two': self.pack({11: self.current[11]})['arbitrary_roi_names']}
        self.assertFalse(register_marker_reference(duplicate, self.profile)['available'])
        bad = self.current[11].copy()
        bad[0, 0] = np.nan
        self.assertFalse(register_marker_reference(self.pack({11: bad, 23: self.current[23]}), self.profile)['available'])

    def test_degenerate_overlapping_correspondences_rejected(self):
        result = register_marker_reference(self.pack({11: self.current[11], 23: self.current[11]}), self.profile)
        self.assertFalse(result['available'])

    def test_two_markers_with_inconsistent_vertex_geometry_rejected(self):
        bad = self.current[23].copy()
        bad[0] += [18, 12]
        result = register_marker_reference(self.pack({11: self.current[11], 23: bad}), self.profile)
        self.assertFalse(result['available'])

    def test_ransac_rejects_a_mismatched_marker(self):
        quads = {11: self.current[11], 23: self.current[23], 31: self.current[31]+[40, 25]}
        result = register_marker_reference(self.pack(quads), self.profile, (800, 800))
        self.assertTrue(result['available'], result)
        self.assertEqual(result['used_marker_ids'], [11, 23])
        self.assertEqual(result['inlier_corners_by_id'][31], 0)

    def test_unregistered_markers_are_not_inserted_into_reference(self):
        before = set(self.profile['markers_px'])
        result = register_marker_reference(self.pack({11: self.current[11], 23: self.current[23], 49: self.current[31]}), self.profile)
        self.assertTrue(result['available'])
        self.assertEqual(result['used_marker_ids'], [11, 23])
        self.assertEqual(set(self.profile['markers_px']), before)

    def test_profile_requires_an_established_reference(self):
        with self.assertRaises(ValueError):
            create_marker_reference(dict(corners=self.pack(self.quads), board_reference=dict(available=False)))

    def test_profile_does_not_invent_unread_ids(self):
        inspection = dict(corners=self.pack({key: self.quads[key] for key in (11, 23, 31)}),
                          board_reference=dict(available=True, points_px=self.board))
        profile = create_marker_reference(inspection)
        self.assertEqual(set(profile['markers_px']), {'11', '23', '31'})

    def test_rectified_image_uses_the_recorded_projective_transform(self):
        reference = np.zeros((800, 800, 3), dtype=np.uint8)
        cv2.circle(reference, (350, 350), 10, (0, 0, 255), -1)
        frame = cv2.warpPerspective(reference, self.warp, (800, 800))
        result = register_marker_reference(self.pack(self.current), self.profile, (800, 800))
        output = rectify_board(frame, result, 361)
        self.assertEqual(output.shape, (361, 361, 3))
        np.testing.assert_allclose(output[180, 180], [0, 0, 255], atol=3)
        with self.assertRaises(ValueError):
            rectify_board(frame, dict(available=False))

    def test_grid_fit_recovers_metric_geometry_without_inventing_marker_ids(self):
        matrix = np.float64([[1.25, .04, 80], [.03, 1.2, 70], [.0001, -.0001, 1]])
        checks = []
        for y in (45, 135, 225, 315):
            for x in (45, 135, 225, 315):
                image = self.transform([[x, y]], matrix)[0]
                checks.append(dict(image_x_px=image[0], image_y_px=image[1], expected_x_mm=x, expected_y_mm=y))
        profile = fit_grid_reference(checks, self.pack({key: self.quads[key] for key in (11, 23, 31)}))
        actual = self.transform([[200, 250]], np.array(profile['image_to_board_mm']) @ matrix)
        np.testing.assert_allclose(actual, [[200, 250]], atol=.001)
        self.assertEqual(set(profile['markers_px']), {'11', '23', '31'})
        self.assertFalse(profile['grid_calibration']['training_fit_is_accuracy_validation'])
        self.assertFalse(profile['metric_accuracy_verified'])

    def test_sparse_collinear_or_duplicate_grid_points_are_not_calibration(self):
        for points in ([dict(image_x_px=x, image_y_px=x, expected_x_mm=x, expected_y_mm=x) for x in range(0, 360, 20)],
                       [dict(image_x_px=10, image_y_px=10, expected_x_mm=45, expected_y_mm=45)]*16):
            with self.assertRaises(ValueError):
                fit_grid_reference(points, self.pack(self.quads))

    def test_refinement_does_not_invent_ids_or_mutate_input(self):
        frame = np.full((800, 800, 3), 255, np.uint8)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        frame[200:280, 200:280] = cv2.cvtColor(cv2.aruco.generateImageMarker(dictionary, 23, 80), cv2.COLOR_GRAY2BGR)
        markers = self.pack({23: np.float64([[200, 200], [279, 200], [279, 279], [200, 279]])})
        before = np.array(markers['arbitrary_roi_names']['corners_px'])
        result = refine_marker_vertices(frame, markers)
        self.assertEqual(set(marker for item in result.values() for marker in item['ids']), {23})
        np.testing.assert_array_equal(markers['arbitrary_roi_names']['corners_px'], before)


if __name__ == '__main__':
    unittest.main()
