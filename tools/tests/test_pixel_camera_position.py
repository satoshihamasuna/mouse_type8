"""ログなしの盤面XY推定。AR欠測を埋めず、カメラの移動と機体の移動を区別する。"""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np
import pandas as pd

sys.path[:0] = [str(Path(__file__).resolve().parents[1]), str(Path(__file__).resolve().parents[1]/'analysis')]
from pixel_camera_reference import create_marker_reference
from pixel_camera_position import estimate, board_positions
import test_video_marker_pose as pose_tests


class PositionTests(unittest.TestCase):
    def test_video_only_tracks_board_position_and_does_not_fill_aruco_dropouts(self):
        image = pose_tests.CalibrationTests().board()
        corners, ids, _ = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)).detectMarkers(image)
        reference = create_marker_reference(dict(corners={'all': dict(ids=ids.ravel().tolist(),
            corners_px=[quad.reshape(4, 2).tolist() for quad in corners])}, board_reference=dict(
                available=True, points_px=dict(TL=[99,99], TR=[500,99], BR=[500,500], BL=[99,500]))),
            image_size=(600,600), board_corner_ids=dict(TL=5,TR=7,BL=6,BR=4))
        cv2.circle(image, (300,270), 10, (0,0,255), -1)
        cv2.circle(image, (300,330), 10, (0,255,255), -1)
        missing = image.copy()
        for x,y in ((50,50),(500,50),(50,500),(500,500)):
            missing[y:y+50,x:x+50] = 255
        shifted = cv2.warpAffine(image, np.float32([[1,0,15],[0,1,0]]), (600,600), borderValue=(255,255,255))
        moved = image.copy()
        moved[255:345,285:315] = 255
        cv2.circle(moved,(320,270),10,(0,0,255),-1)
        cv2.circle(moved,(320,330),10,(0,255,255),-1)
        with tempfile.TemporaryDirectory() as name:
            root=Path(name); video=root/'only_video.avi'
            writer=cv2.VideoWriter(str(video),cv2.VideoWriter_fourcc(*'MJPG'),240,(600,600))
            self.assertTrue(writer.isOpened())
            for frame in [image,image,moved,image]+[missing]*4+[shifted]*4: writer.write(frame)
            writer.release()
            reference_path=root/'reference.json'; reference_path.write_text(json.dumps(reference))
            geometry=root/'geometry.json'; geometry.write_text(json.dumps(dict(mouse='A',
                reference_id=reference['reference_id'], front_colour='red',rear_colour='yellow')))
            result=estimate(video,reference_path,geometry,root/'output')
            points=pd.read_csv(root/'output/position.csv')
            self.assertEqual(points.position_valid.tolist(),[True]*4+[False]*4+[True]*4)
            self.assertTrue(points.loc[4:7,['center_x_mm','center_y_mm']].isna().all().all())
            np.testing.assert_allclose(points.loc[0,['center_x_mm','center_y_mm']].to_numpy(float),[180.45,180.45],atol=1)
            np.testing.assert_allclose(points.loc[0,['center_x_mm','center_y_mm']].to_numpy(float),
                points.loc[11,['center_x_mm','center_y_mm']].to_numpy(float),atol=1.5)
            self.assertAlmostEqual(points.apparent_heading_deg.iloc[0],-90,delta=1)
            self.assertFalse(result['log_used']); self.assertFalse(result['height_correction_applied'])
            self.assertFalse(result['axle_position_calibrated']); self.assertNotIn('auto_stop_confirmed',result)
            self.assertEqual(result['valid_frames'],8)
            # 一度だけ変化した実観測点を前後フレームの中央値で消さない。
            self.assertAlmostEqual(points.center_x_mm.iloc[2]-points.center_x_mm.iloc[1],20*360/401,delta=1)
            self.assertTrue((root/'output/position_trajectory.png').is_file())

    def test_two_real_registered_markers_are_required_even_if_a_cached_pose_is_finite(self):
        reference=dict(board_corner_ids=dict(TL=0,TR=3,BL=1,BR=2),board_side_mm=360)
        pose=pd.DataFrame(dict(frame=[8,12],time_video_s=[.1,.2],calibration_valid=[True,True],
            marker_detected_raw=[True,True],registered_marker_count=[1,2],
            blue_x_mm=[100.,100.],blue_y_mm=[100.,100.],yellow_x_mm=[100.,100.],yellow_y_mm=[150.,150.]))
        result=board_positions(pose,reference)
        self.assertEqual(result.position_valid.tolist(),[False,True])
        self.assertTrue(np.isnan(result.center_x_mm.iloc[0]))
        self.assertEqual(result.frame.tolist(),[8,12])

    def test_learned_estimates_are_separate_from_observations_and_require_current_arucos(self):
        reference=dict(board_corner_ids=dict(TL=0,TR=3,BL=1,BR=2),board_side_mm=360)
        pose=pd.DataFrame(dict(frame=[1,2,3],time_video_s=[.1,.2,.3],calibration_valid=[True,True,False],
            marker_detected_raw=[True,False,False],marker_estimated_raw=[False,True,True],registered_marker_count=[2,2,0],
            blue_x_mm=[100.,101.,102.],blue_y_mm=[100.,100.,100.],
            yellow_x_mm=[100.,101.,102.],yellow_y_mm=[150.,150.,150.]))
        result=board_positions(pose,reference)
        self.assertEqual(result.position_valid.tolist(),[True,False,False])
        self.assertEqual(result.position_estimated.tolist(),[False,True,False])
        self.assertTrue(np.isnan(result.center_x_mm.iloc[1]))
        self.assertTrue(np.isfinite(result.estimated_center_x_mm.iloc[1]))
        self.assertTrue(np.isnan(result.estimated_center_x_mm.iloc[2]))

    def test_mismatched_reference_is_rejected_before_video_processing(self):
        with tempfile.TemporaryDirectory() as name:
            root=Path(name)
            (root/'reference.json').write_text('{"reference_id":"board"}')
            (root/'geometry.json').write_text('{"mouse":"A","reference_id":"different"}')
            with patch('pixel_camera_position.read_video_pose') as read, self.assertRaises(ValueError):
                estimate(root/'video.mp4',root/'reference.json',root/'geometry.json',root/'output')
            read.assert_not_called()
            self.assertFalse((root/'output').exists())

    def test_failed_reference_preflight_is_reported_before_tracking(self):
        with tempfile.TemporaryDirectory() as name:
            root=Path(name); (root/'video.mp4').write_bytes(b'video fixture')
            (root/'reference.json').write_text('{"reference_id":"board"}')
            (root/'geometry.json').write_text(json.dumps(dict(mouse='A',reference_id='board',
                front_colour='red',rear_colour='yellow')))
            diagnostic=dict(available=False,search_rois_relocated=False,attempts=[dict(recovery_reason='ambiguous_full_frame_markers')])
            with patch('pixel_camera_position.prepare_reference',return_value=({'reference_id':'board'},diagnostic)), \
                    patch('pixel_camera_position.read_video_pose') as tracker, self.assertRaisesRegex(RuntimeError,'AR座標合わせ'):
                estimate(root/'video.mp4',root/'reference.json',root/'geometry.json',root/'output')
            tracker.assert_not_called()
            result=json.loads((root/'output/summary.json').read_text(encoding='utf-8'))
            self.assertFalse(result['position_available'])
            self.assertEqual(result['reference_search'],diagnostic)
            self.assertTrue((root/'output/marker_reference_input.json').exists())


if __name__ == '__main__': unittest.main()
