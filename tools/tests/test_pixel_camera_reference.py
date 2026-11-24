"""任意の2枚以上の対応頂点による射影補正。カメラや機体には接続しない。"""
from itertools import combinations
from pathlib import Path
import sys
import unittest

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pixel_camera_reference import (create_marker_reference, detect_board_reference, register_marker_reference,
    rectify_board, fit_grid_reference, refine_marker_vertices, detect_registered_vertices, marker_corners_by_id,
    relocate_marker_search)


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

    def test_explicit_rois_disambiguate_repeated_ids_without_reusing_an_outside_copy(self):
        image=np.full((600,600,3),255,np.uint8)
        dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        for marker_id,x,y in ((11,100,100),(11,400,100),(23,400,400)):
            marker=cv2.aruco.generateImageMarker(dictionary,marker_id,50)
            image[y:y+50,x:x+50]=cv2.cvtColor(marker,cv2.COLOR_GRAY2BGR)
        profile=dict(markers_px={'11':[[400,100],[449,100],[449,149],[400,149]],
                                 '23':[[400,400],[449,400],[449,449],[400,449]]},
            marker_search_rois_px={'11':[380,80,470,170],'23':[380,380,470,470]})
        found=marker_corners_by_id(detect_registered_vertices(image,profile))
        self.assertEqual(set(found),{11,23})
        np.testing.assert_allclose(found[11].mean(axis=0),[424.5,124.5],atol=1)
        image[400:450,400:450]=255
        found=marker_corners_by_id(detect_registered_vertices(image,profile))
        self.assertEqual(set(found),{11})
        image[100:150,400:450]=255
        self.assertEqual(marker_corners_by_id(detect_registered_vertices(image,profile)),{})

    def test_explicit_roi_ids_must_match_the_registered_ids(self):
        profile=dict(self.profile,marker_search_rois_px={'11':[0,0,100,100]})
        with self.assertRaisesRegex(ValueError,'ROIs'):
            detect_registered_vertices(np.zeros((800,800,3),np.uint8),profile)

    def shifted_search_board(self):
        image=np.full((800,800,3),255,np.uint8)
        dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        for marker_id,x,y in ((5,50,50),(7,500,50),(6,50,500),(4,500,500)):
            image[y:y+50,x:x+50]=cv2.cvtColor(cv2.aruco.generateImageMarker(dictionary,marker_id,50),cv2.COLOR_GRAY2BGR)
        corners,ids,_=cv2.aruco.ArucoDetector(dictionary).detectMarkers(image)
        profile=create_marker_reference(dict(corners={'all':dict(ids=ids.ravel().tolist(),
            corners_px=[quad.reshape(4,2).tolist() for quad in corners])},board_reference=dict(available=True,
                points_px=dict(TL=[99,99],TR=[500,99],BR=[500,500],BL=[99,500]))),
            image_size=(800,800),board_corner_ids=dict(TL=5,TR=7,BL=6,BR=4))
        profile['marker_search_rois_px']={str(key):[x-10,y-10,x+60,y+60]
            for key,x,y in ((5,50,50),(7,500,50),(6,50,500),(4,500,500))}
        shifted=cv2.warpAffine(image,np.float32([[1,0,100],[0,1,80]]),(800,800),borderValue=(255,255,255))
        return shifted,profile,dictionary

    def test_search_relocation_recovers_camera_shift_without_changing_reference_geometry(self):
        image,profile,_=self.shifted_search_board()
        before={key:list(box) for key,box in profile['marker_search_rois_px'].items()}
        self.assertFalse(detect_board_reference(image,detect_registered_vertices(image,profile),profile)['available'])
        result=relocate_marker_search(image,profile)
        self.assertTrue(result['available'],result)
        self.assertEqual(result['profile']['reference_id'],profile['reference_id'])
        self.assertEqual(result['profile']['image_to_board_mm'],profile['image_to_board_mm'])
        self.assertEqual(result['profile']['board_points_px'],profile['board_points_px'])
        self.assertEqual(profile['marker_search_rois_px'],before)
        self.assertNotEqual(result['profile']['marker_search_rois_px'],before)
        self.assertEqual(result['registration']['used_marker_ids'],[4,5,6,7])

    def test_search_relocation_rejects_duplicate_registered_id_outside_the_old_rois(self):
        image,profile,dictionary=self.shifted_search_board()
        image[700:750,50:100]=cv2.cvtColor(cv2.aruco.generateImageMarker(dictionary,5,50),cv2.COLOR_GRAY2BGR)
        result=relocate_marker_search(image,profile)
        self.assertFalse(result['available'])
        self.assertEqual(result['reason'],'ambiguous_full_frame_markers')
        self.assertNotIn('profile',result)

    def test_search_relocation_does_not_invent_missing_ids_or_relax_grid_validation(self):
        image,profile,_=self.shifted_search_board()
        image[580:630,150:200]=255
        self.assertFalse(relocate_marker_search(image,profile)['available'])
        image,profile,_=self.shifted_search_board()
        profile['grid_refinement']=dict(enabled=True,pitch_mm=45,
            training_grid_indices=[[x,y] for x in (1,3,5,7) for y in (1,3,5,7)])
        self.assertFalse(relocate_marker_search(image,profile)['available'])

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
        self.assertNotIn('corner_refinement', profile)
        refined = self.pack({key: self.quads[key] for key in (11, 23, 31)})
        refined['arbitrary_roi_names']['corner_refinement'] = 'subpix_original_win3'
        refined_profile = fit_grid_reference(checks, refined)
        self.assertEqual(refined_profile['corner_refinement'], 'subpix_original_win3')

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
