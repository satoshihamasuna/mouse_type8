"""保存動画の座標登録、欠測、同期の退行を実装本体で確認する。"""
import importlib.util
import math
from pathlib import Path
import sys
import unittest
import csv
import json
import tempfile
from unittest.mock import Mock, patch

import cv2
import numpy as np

MODULE=Path(__file__).resolve().parents[1]/'analysis/maze_video_pose.py'
spec=importlib.util.spec_from_file_location('maze_video_pose',MODULE)
pose=importlib.util.module_from_spec(spec); spec.loader.exec_module(pose)


class RegistrationTests(unittest.TestCase):
    def reference(self):
        image=np.full((720,1000,3),255,np.uint8)
        corners={0:(100,100),1:(800,100),2:(100,550),3:(800,550)}
        dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        for marker,(x,y) in corners.items():
            patch=cv2.aruco.generateImageMarker(dictionary,marker,60)
            image[y:y+60,x:x+60]=cv2.cvtColor(patch,cv2.COLOR_GRAY2BGR)
        markers=pose.detected_markers(image,pose.detector())
        matrix=np.array([[.01,0,-.5],[0,.01,-.2],[0,0,1.]])
        return image,dict(markers_cell={k:pose.project(q,matrix).tolist() for k,q in markers.items()}),matrix

    def test_camera_movement_is_registered_from_marker_vertices(self):
        image,reference,matrix=self.reference()
        move=np.array([[.92,.04,22],[-.02,.95,25],[.00002,-.00003,1.]])
        frame=cv2.warpPerspective(image,move,(1000,720),borderValue=(255,255,255))
        estimated,quality=pose.registration(frame,pose.detector(),reference)
        self.assertIsNotNone(estimated,quality)
        original=np.array([[300,250],[600,450]],np.float32)
        actual=pose.project(pose.project(original,move),estimated)
        np.testing.assert_allclose(actual,pose.project(original,matrix),atol=.025)

    def test_one_marker_never_reuses_previous_homography(self):
        image,reference,_=self.reference()
        image[400:]=255; image[:400,500:]=255
        matrix,quality=pose.registration(image,pose.detector(),reference)
        self.assertIsNone(matrix)
        self.assertEqual(quality['reason'],'fewer_than_two_aruco')

    def test_red_walls_without_rear_marker_are_not_a_robot(self):
        image,reference,matrix=self.reference()
        cv2.line(image,(300,200),(300,500),(0,0,255),5)
        hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        candidate,reason=pose.select_pair(pose.components(hsv,'red',matrix),pose.components(hsv,'yellow',matrix))
        self.assertIsNone(candidate)


class TrackingTests(unittest.TestCase):
    @staticmethod
    def component(point):
        return dict(cell=np.array(point,dtype=float),pixel=np.array(point,dtype=float),area=20,circularity=.8)

    @staticmethod
    def marker_frame():
        image,reference,_=RegistrationTests().reference()
        cv2.circle(image,(370,300),5,(0,0,255),-1)
        cv2.circle(image,(425,300),5,(0,255,255),-1)
        reference.update(schema=1,rows=9,columns=9,axes='board TL origin, x right, y down')
        return image,reference

    @staticmethod
    def capture(image):
        capture=Mock()
        capture.grab.side_effect=[True,False]
        capture.retrieve.return_value=(True,image)
        capture.get.return_value=1000.
        return capture

    def test_mouse_b_uses_yellow_front_and_red_rear_without_moving_the_midpoint(self):
        image,reference=self.marker_frame()
        rows={}
        for mouse in ('A','B'):
            with patch.object(pose.cv2,'VideoCapture',return_value=self.capture(image)):
                rows[mouse]=pose.track('synthetic.mp4',reference,progress=False,mouse=mouse)[0]
            self.assertTrue(rows[mouse]['pose_valid'],rows[mouse])
        self.assertEqual((rows['A']['front_image_x'],rows['A']['rear_image_x']),(370,425))
        self.assertEqual((rows['B']['front_image_x'],rows['B']['rear_image_x']),(425,370))
        self.assertAlmostEqual(rows['B']['heading_cw_rad'],0,delta=.01)
        self.assertAlmostEqual(abs(rows['A']['heading_cw_rad']),math.pi,delta=.01)
        self.assertEqual(rows['A']['x_cell'],rows['B']['x_cell'])
        self.assertEqual(rows['A']['y_cell'],rows['B']['y_cell'])

    def test_unknown_mouse_is_rejected_before_opening_the_video(self):
        for mouse in (None,'unknown','C'):
            with self.subTest(mouse=mouse), patch.object(pose.cv2,'VideoCapture') as capture:
                with self.assertRaisesRegex(ValueError,'firmware.mouse'):
                    pose.track('synthetic.mp4',{},mouse=mouse)
                capture.assert_not_called()

    def test_impossible_jump_is_rejected_and_ambiguity_is_not_filled(self):
        c=self.component
        prev=dict(centre=np.array([1.,1.]),heading=0.)
        pair,_=pose.select_pair([c([5.55,5])],[c([5,5])],prev,.017)
        self.assertIsNone(pair)
        pair,reason=pose.select_pair([c([1.55,1]),c([.45,1])],[c([1,1])])
        self.assertIsNone(pair); self.assertEqual(reason,'ambiguous_pair')

    def test_heading_wrap_and_missing_frames_do_not_create_velocity_spikes(self):
        rows=[]
        for i in range(100):
            t=i/60; angle=3+2*t
            rows.append(dict(time_video_s=t,pose_valid=i!=50,x_cell=t,y_cell=.5,
                             heading_cw_rad=math.atan2(math.sin(angle),math.cos(angle))))
        pose.derivatives(rows,90)
        self.assertNotIn('omega_video_ccw_rad_s',rows[50])
        self.assertAlmostEqual(rows[25]['omega_video_ccw_rad_s'],-2,places=6)
        self.assertAlmostEqual(rows[25]['speed_video_apparent_m_s'],.09,places=6)


class SyncTests(unittest.TestCase):
    def test_affine_clock_model_recovers_drift_without_spatial_fitting(self):
        t=np.arange(0,75,.005)
        centres=[2,6,13,18,23,28,32,38,41,48,52,58,64,70]
        def omega(x):
            return sum((4+i%4)*(-1 if i%3==0 else 1)*np.exp(-((x-c)/(.12+.04*(i%3)))**2)
                       for i,c in enumerate(centres))
        ble=[dict(time_ms=1000+1000*x,**{'ideal.velo':.3,'ideal.rad_velo':omega(x),'ego.rad_velo':omega(x)}) for x in t]
        video=[dict(time_video_s=x*.993+12.1,omega_video_ccw_rad_s=omega(x)/.993) for x in np.arange(0,75,1/60)]
        initial=pose.fit_sync(video,ble,1000,12.)
        sync=pose.fit_clock_drift(video,ble,1000,initial)
        self.assertTrue(sync['accepted'],sync)
        self.assertAlmostEqual(sync['clock_scale'],.993,delta=.0003)
        self.assertAlmostEqual(sync['video_time_at_mcu_origin_s'],12.1,delta=.015)
        self.assertGreater(sync['local_fit_span_s'],25)

    def data(self):
        # Irregularly spaced MCU timestamps and asymmetric signed turns distinguish
        # a true temporal offset from repeated or straight-only motion.
        t=np.arange(0,12,.005)
        def omega(t):
            return 5*np.exp(-((t-2)/.15)**2)-7*np.exp(-((t-4.3)/.2)**2)+4*np.exp(-((t-8)/.3)**2)
        ble=[dict(time_ms=1000+1000*x,**{'ego.rad_velo':omega(x)}) for x in t]
        video=[dict(time_video_s=x+1.37,omega_video_ccw_rad_s=omega(x)) for x in np.arange(0,12,1/60)]
        return video,ble

    def test_offset_uses_signed_gyro_and_original_mcu_time(self):
        video,ble=self.data()
        sync=pose.fit_sync(video,ble,1000,1.15)
        self.assertTrue(sync['accepted'],sync)
        self.assertAlmostEqual(sync['video_time_at_mcu_origin_s'],1.37,places=3)
        self.assertEqual(sync['mcu_origin_ms'],1000)

    def test_straight_only_and_wrong_sign_are_not_confident_sync(self):
        video,ble=self.data()
        for p in video: p['omega_video_ccw_rad_s']=0
        self.assertFalse(pose.fit_sync(video,ble,1000,1.15)['accepted'])
        video,ble=self.data()
        for p in video: p['omega_video_ccw_rad_s']*=-1
        self.assertFalse(pose.fit_sync(video,ble,1000,1.15)['accepted'])


class WallReviewTests(unittest.TestCase):
    def test_saved_run_settings_select_marker_colours_for_analysis_and_report(self):
        image,reference=TrackingTests.marker_frame()
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            (root/'video.mp4').write_bytes(b'synthetic video decoded by mock capture')
            pose.json_save(root/'reference.json',reference)
            fields=('sequence','ideal.velo','ego.velo','ideal.rad_velo','ego.rad_velo',
                'sen_l.distance','sen_r.distance','sen_fl.distance','sen_fr.distance',
                'sp_feedback','om_feedback','Battery','V_l','V_r')
            values=dict.fromkeys(fields,0)
            values.update({'ideal.velo':.3,'ego.velo':.3})
            with (root/'telemetry.csv').open('w',newline='',encoding='utf-8') as stream:
                writer=csv.DictWriter(stream,fieldnames=['time_ms',*fields]); writer.writeheader()
                writer.writerows(dict(values,time_ms=t) for t in (100,105,110))
            trial=dict(run_settings=str(root/'run.json'),ble_directory=str(root),video_file=str(root/'video.mp4'),
                position_analysis=dict(front_colour='red',rear_colour='yellow'))
            pose.json_save(root/'trial.json',trial)
            for mouse,front,rear,front_x in (('A','red','yellow',370),('B','yellow','red',425)):
                with self.subTest(mouse=mouse):
                    pose.json_save(root/'run.json',dict(firmware=dict(mouse=mouse),
                        telemetry_start=dict(time_ms=100),telemetry_end=dict(time_ms=110)))
                    output=root/mouse
                    with patch.object(pose.cv2,'VideoCapture',return_value=TrackingTests.capture(image)):
                        pose.analyze(root/'trial.json',root/'reference.json',output,cell_mm=90.)
                    report=json.loads((output/'analysis.json').read_text(encoding='utf-8'))
                    self.assertEqual((report['mouse'],report['front_colour'],report['rear_colour']),(mouse,front,rear))
                    self.assertEqual(report['valid_frames'],1)
                    with (output/'pose.csv').open(encoding='utf-8') as stream:
                        row=next(csv.DictReader(stream))
                    self.assertEqual(float(row['front_image_x']),front_x)
                    html=(output/'review.html').read_text(encoding='utf-8')
                    payload=html.split('const D=',1)[1].split(',R=D.report',1)[0]
                    self.assertEqual(json.loads(payload)['report']['front_colour'],front)

    def test_full_rate_adc_survives_video_loading_and_legacy_fields_stay_missing(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            settings=root/'run.json'
            settings.write_text(json.dumps(dict(telemetry_start=dict(time_ms=100),telemetry_end=dict(time_ms=141))),encoding='utf-8')
            trial=dict(run_settings=str(settings),ble_directory=str(root),video_file=str(root/'video.mp4'))
            base=dict.fromkeys(('sequence','ideal.velo','ego.velo','ideal.rad_velo','ego.rad_velo',
                'sen_l.distance','sen_r.distance','sen_fl.distance','sen_fr.distance','sp_feedback','om_feedback','Battery','V_l','V_r'),0)
            base['ideal.velo']=.37
            raw=dict(base,**{'sen_l.value':-7,'sen_r.value':2049,'sen_fl.value':4093,'sen_fr.value':125,
                            'ir.range_states':2209,'motion.pattern':19})
            times=[100,105,110,115,120,125,130,135,141]
            for optional in (True,False):
                values=raw if optional else base
                with (root/'telemetry.csv').open('w',newline='',encoding='utf-8') as stream:
                    writer=csv.DictWriter(stream,fieldnames=['time_ms',*values]);writer.writeheader()
                    writer.writerows(dict(values,time_ms=t) for t in times)
                ble,_,zero=pose.read_ble(trial)
                self.assertEqual([r['time_ms'] for r in ble],times)
                self.assertEqual(zero,100)
                if optional:self.assertEqual(ble[0]['sen_r.value'],2049)
                else:self.assertNotIn('sen_r.value',ble[0])
                report=dict(sync=dict(mcu_origin_ms=100,video_time_at_mcu_origin_s=1.,clock_scale=1.,accepted=True))
                pose.build_review(root,trial,ble,[],dict(wall_segments=[]),report)
                html=(root/'review.html').read_text(encoding='utf-8')
                payload=html.split('const D=',1)[1].split(',R=D.report',1)[0]
                data=json.loads(payload)
                self.assertEqual(len(data['wall_signals']),9)
                self.assertEqual(data['wall_signals'][-1][0],141)
                self.assertEqual(data['wall_signals'][0][5:9],[-7,2049,4093,125] if optional else [None]*4)
                self.assertIn('id="zoom"',html)


if __name__=='__main__': unittest.main()
