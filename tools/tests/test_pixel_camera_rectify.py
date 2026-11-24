"""実復号ARだけの正対補正、欠測、軸向き、カメラ移動、可変PTSを合成動画で検証する。"""
import csv
from fractions import Fraction
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path[:0]=[str(Path(__file__).resolve().parents[1]),str(Path(__file__).resolve().parents[1]/'analysis')]
from pixel_camera_environment import initialize_camera_environment
initialize_camera_environment()
import av
import cv2
import numpy as np
from pixel_camera_reference import create_marker_reference
from pixel_camera_rectify import rectify


def board_and_reference():
    image=np.full((600,600,3),255,np.uint8)
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    for key,(x,y) in zip((5,7,6,4),((50,50),(500,50),(50,500),(500,500))):
        image[y:y+50,x:x+50]=cv2.cvtColor(cv2.aruco.generateImageMarker(dictionary,key,50),cv2.COLOR_GRAY2BGR)
    corners,ids,_=cv2.aruco.ArucoDetector(dictionary).detectMarkers(image)
    reference=create_marker_reference(dict(corners={'all':dict(ids=ids.ravel().tolist(),
        corners_px=[q.reshape(4,2).tolist() for q in corners])},board_reference=dict(available=True,
        points_px=dict(TL=[99,99],TR=[500,99],BR=[500,500],BL=[99,500]))),
        image_size=(600,600),board_corner_ids=dict(TL=5,TR=7,BL=6,BR=4))
    return image,reference


class RectificationTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)
        self.image,self.reference=board_and_reference()
        self.ref=self.root/'reference.json';self.ref.write_text(json.dumps(self.reference))

    def write_video(self,frames):
        video=self.root/'source.mkv'
        with av.open(str(video),'w') as destination:
            stream=destination.add_stream('ffv1',rate=240)
            stream.width=stream.height=600
            stream.pix_fmt='bgr0'
            stream.time_base=stream.codec_context.time_base=Fraction(1,1000)
            for stamp,image in zip((100,104,109,117,121,130),frames):
                frame=av.VideoFrame.from_ndarray(image,format='bgr24')
                frame.pts=stamp;frame.time_base=Fraction(1,1000)
                for packet in stream.encode(frame):destination.mux(packet)
            for packet in stream.encode():destination.mux(packet)
        return video

    def missing(self,image=None):
        result=(self.image if image is None else image).copy()
        for x,y in ((50,50),(500,50),(50,500),(500,500)):
            result[y:y+50,x:x+50]=255
        return result

    def test_current_markers_camera_motion_axes_and_variable_pts(self):
        image=self.image.copy()
        cv2.circle(image,(200,250),10,(0,0,255),-1)
        cv2.circle(image,(420,400),10,(255,0,0),-1)
        shifted=cv2.warpAffine(image,np.float32([[1,0,15],[0,1,-10]]),(600,600),borderValue=(255,255,255))
        two=image.copy();two[500:550,50:100]=255;two[500:550,500:550]=255
        video=self.write_video([image,shifted,self.missing(image),two,self.missing(image),image])
        before=video.read_bytes()
        with patch('video_marker_pose.read_video_pose',side_effect=AssertionError('no body tracker')), \
                patch('video_marker_pose.colour_components',side_effect=AssertionError('no colour tracker')):
            summary=rectify(video,self.ref,self.root/'result',size=360)
        with (self.root/'result/rectification.csv').open() as stream:rows=list(csv.DictReader(stream))
        self.assertEqual([r['calibration_valid'] for r in rows],['True','True','False','True','False','True'])
        self.assertEqual(rows[3]['registered_marker_count'],'2')
        self.assertEqual(summary['valid_frames'],4);self.assertEqual(summary['frames'],6)
        with av.open(str(self.root/'result/rectified.mp4')) as checked:
            frames=list(checked.decode(video=0))
        np.testing.assert_allclose([float(f.pts*f.time_base) for f in frames],[0,.004,.009,.017,.021,.030],atol=.001)
        self.assertLess(frames[2].to_ndarray(format='bgr24').mean(),10)
        for index in (0,1,3,5):
            result=frames[index].to_ndarray(format='bgr24')
            y,x=np.nonzero((result[:,:,2]>150)&(result[:,:,1]<80)&(result[:,:,0]<80))
            np.testing.assert_allclose([x.mean(),y.mean()],np.array([101,151])*359/401,atol=2)
        self.assertTrue((self.root/'result/rectified_preview.png').is_file())
        self.assertFalse(summary['body_tracking_used']);self.assertFalse(summary['log_used'])
        self.assertEqual(video.read_bytes(),before)
        with self.assertRaises(FileExistsError):rectify(video,self.ref,self.root/'result',size=360)

    def test_stride_retains_source_frame_numbers_and_time_intervals(self):
        video=self.write_video([self.image]*6)
        result=rectify(video,self.ref,self.root/'quick',stride=4,size=128)
        with (self.root/'quick/rectification.csv').open() as stream:rows=list(csv.DictReader(stream))
        self.assertEqual([int(r['frame']) for r in rows],[0,4])
        self.assertAlmostEqual(float(rows[1]['time_output_s']),.021,places=3)
        self.assertEqual(result['frames'],2)
        self.assertTrue(result['output_timebase_verified'])

    def test_all_missing_is_reported_without_a_cached_transform(self):
        video=self.write_video([self.missing()]*2)
        with self.assertRaisesRegex(ValueError,'全フレーム'):
            rectify(video,self.ref,self.root/'missing',size=128)
        summary=json.loads((self.root/'missing/summary.json').read_text(encoding='utf-8'))
        self.assertFalse(summary['rectification_available']);self.assertEqual(summary['valid_frames'],0)
        self.assertTrue((self.root/'missing/rectification.csv').is_file())

    def test_invalid_reference_is_rejected_before_video_or_output(self):
        self.ref.write_text('{"schema_version":1}')
        with self.assertRaisesRegex(ValueError,'登録参照JSON'):
            rectify(self.root/'no-video',self.ref,self.root/'invalid')
        self.assertFalse((self.root/'invalid').exists())


if __name__=='__main__':unittest.main()
