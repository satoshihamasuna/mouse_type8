"""接続を開かず、GUI撮影の設定照合・一回実行・ログ保存を模擬検証する。"""
from copy import deepcopy
import csv
from dataclasses import replace
import json
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import pixel_camera_tuning as tuning


def settings(lend=19.47):
    return dict(mouse='A', motion_name='long_r180', preset_speed_mm_s=500, turn_count=1,
        parameters=dict(velo=.5, r_min=-42.5, lstart=23.63, lend=lend, degree=-180, correction=0, pre_accel=6.5),
        pid=dict(sp_kp=2, sp_ki=.016, sp_kd=0, om_kp=.1, om_ki=.01, om_kd=0),
        feedforward=dict(ff_sp_velo=.870046, ff_sp_accel=.114741, ff_sp_bias=.094599,
            ff_om_velo=.002126, ff_om_accel=.000531, ff_om_decel=.001443, ff_om_bias=.085591,
            ff_om_jerk=0, ff_sp_turn_accel_mag=0, ff_sp_turn_velo_sq=0),
        suction=dict(enabled=False, duty=650), raw_response='mock response')


def motion_rows():
    return [dict(cnt=i*5, time_ms=1000+i*5, sequence=i, lost_frames=0, **{
        'ideal.velo':.5 if 20<=i<=80 else 0, 'ideal.rad_velo':-8 if 40<=i<=60 else 0,
        'ego.velo':.5 if 20<=i<=80 else 0, 'ego.rad_velo':-8 if 40<=i<=60 else 0}) for i in range(200)]


class FakeTail:
    def __init__(self, source, target):
        self.rows, self.target = motion_rows(), target

    @property
    def latest_at(self):
        return time.monotonic()

    def snapshot(self):
        return self.rows

    def close(self):
        with self.target.open('w',encoding='utf-8',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=self.rows[0].keys())
            writer.writeheader()
            writer.writerows(self.rows)


class ConnectedTrialTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.reference, self.geometry = self.root/'ref.json', self.root/'geometry.json'
        self.reference.write_text(json.dumps(dict(reference_id='test')),encoding='utf-8')
        self.geometry.write_text(json.dumps(dict(mouse='A',reference_id='test',front_height_mm=10,
            rear_height_mm=1,axle_offset_from_marker_midpoint_mm=[0,0])),encoding='utf-8')
        self.request = tuning.TrialRequest(settings(), 'debug turn long_r180 set mock', .08, 800,
                                          self.reference, self.geometry, 20, True)
        self.camera = SimpleNamespace(last_recording_id=None, client_id='operator', phase='IDLE', owner='operator')
        self.camera.get_status = lambda: dict(connected=True,device=dict(phase=self.camera.phase,
            recording_id=self.camera.last_recording_id,owner=self.camera.owner,elapsed_seconds=.1))
        self.session = SimpleNamespace(project=self.root,folder=self.root,camera=self.camera,
            connection=SimpleNamespace(is_open=True),alive=lambda:True,close=Mock())
        self.health = dict(rows=999,lost=0,crc_errors=0,monotonic=time.monotonic(),latest_sequence=999)
        self.current = settings()
        self.after = settings()
        self.sent, self.record_count = [], 0
        self.capability = 'DEBUG_CAMERA_CAP protocol:1 ttl_ms:15000'
        outer = self
        class FakeShell:
            def __init__(self,*args,**kwargs):
                self.applied=False
            def send(self,command):
                outer.sent.append(command)
                if ' set ' in command:self.applied=True
                if ' exe_camera ' in command and outer.fail_write:raise TimeoutError('ambiguous execution write')
            def check_battery(self):
                if outer.low_battery:raise RuntimeError('low battery')
            def query(self,command,predicate):
                outer.sent.append(command)
                return outer.capability if command=='debug camera show' else ('after' if self.applied else 'before')
            def wait_line(self,predicate,timeout=5):
                if ' set ' in outer.sent[-1]:line='DEBUG_SEARCH_SET_DONE' if outer.sent[-1].startswith('debug search_turn ') else 'DEBUG_TURN_SET_DONE'
                elif outer.sent[-1].startswith('debug camera arm '):line='DEBUG_CAMERA_ARMED token:'+outer.sent[-1].split()[-1]
                else:line='DEBUG_CAMERA_RUN_START token:'+outer.sent[-1].split()[-1]
                assert predicate(line)
                return line
        self.shell_type=FakeShell
        self.fail_write = self.low_battery = False
        self.phase_on_ready = 'RECORDING'
        self.addCleanup(patch.stopall)
        patch.object(tuning.time,'sleep',return_value=None).start()
        patch.object(tuning,'last_health',side_effect=lambda path:dict(self.health)).start()

    def record(self,client,project,**kwargs):
        self.record_count += 1
        self.assertEqual(kwargs['capture_profile'], 'tuning')
        self.assertEqual(kwargs['camera_settings'], self.request.camera_settings)
        self.camera.last_recording_id='recording'+str(self.record_count)
        self.camera.phase=self.phase_on_ready
        kwargs['on_ready']()
        self.camera.phase='IDLE'
        manifest=self.root/('capture'+str(self.record_count)+'.json')
        return manifest, dict(state='PASSED',video_file='mock.mp4',recording_id=self.camera.last_recording_id)

    def capture(self,name='run'):
        return tuning.capture_connected(self.session,self.request,self.root/name,
            lambda line,*args:deepcopy(self.after if line=='after' else self.current),
            record=self.record,shell_type=self.shell_type,tail_type=FakeTail)

    def test_saved_one_ms_log_is_requested_after_recording_and_continuous_csv_is_kept(self):
        saved = dict(status='saved', csv='saved_1ms.csv', period_ms=1, frame_count=400)
        def retrieve(shell, folder, actual, capability, motion, event):
            self.assertEqual(self.camera.phase, 'IDLE')
            self.assertEqual(self.record_count, 1)
            self.assertTrue((folder/'telemetry.csv').exists())
            self.assertEqual(actual['parameters'], self.current['parameters'])
            self.assertEqual((capability, motion), (self.capability, 'long_r180'))
            return saved
        with patch.object(tuning, 'save_retained_log', side_effect=retrieve) as exported:
            value = self.capture()
        exported.assert_called_once()
        self.assertEqual(value['saved_log'], saved)
        self.assertEqual(value['state'], 'CAPTURED_REVIEW_REQUIRED')
        self.assertTrue((self.root/'run/motion.csv').exists())
        self.assertTrue(value['type8_log_csv'].endswith('motion.csv'))

    def test_record_only_captures_once_without_analysis_profiles_or_height(self):
        self.request = replace(self.request, video_analysis=False, height_mm=float('nan'),
            reference=self.root/'missing-reference.json', geometry=self.root/'missing-geometry.json')
        value = self.capture()
        self.assertEqual(value['state'],'CAPTURED_REVIEW_REQUIRED')
        self.assertEqual(self.record_count,1)
        self.assertEqual(sum(' exe_camera ' in command for command in self.sent),1)
        self.assertFalse(value['video_analysis_requested'])
        self.assertIsNone(value['camera_height_mm'])
        self.assertIsNone(value['quick_report']); self.assertIsNone(value['full_report'])
        self.assertFalse((self.root/'run/marker_reference_used.json').exists())
        self.assertFalse((self.root/'run/body_geometry_used.json').exists())
        self.assertTrue((self.root/'run/motion.csv').exists())
        self.assertIn('camera_configuration',value)
        with self.assertRaises(ValueError): replace(self.request,operator_ready=False).validate()
        with self.assertRaises(ValueError): replace(self.request,seconds=0).validate()
        with self.assertRaises(ValueError): replace(self.request,camera_settings={'zoom_ratio':0}).validate()

    def test_rectification_only_keeps_reference_without_loading_body_settings(self):
        from test_pixel_camera_rectify import board_and_reference
        _,reference=board_and_reference()
        self.reference.write_text(json.dumps(reference),encoding='utf-8')
        self.request=replace(self.request,rectify_only=True,height_mm=float('nan'),
            geometry=self.root/'missing-body-model.json')
        result=self.capture()
        self.assertEqual(result['state'],'CAPTURED_REVIEW_REQUIRED')
        self.assertTrue(result['video_analysis_requested'])
        self.assertEqual(result['analysis_mode'],'rectify_only')
        self.assertIsNone(result['camera_height_mm'])
        self.assertTrue((self.root/'run/marker_reference_used.json').is_file())
        self.assertFalse((self.root/'run/body_geometry_used.json').exists())
        self.assertEqual(self.record_count,1)
        self.assertEqual(sum(' exe_camera ' in command for command in self.sent),1)
        with self.assertRaises(FileNotFoundError):
            replace(self.request,reference=self.root/'missing-reference.json').validate()

    def test_trial_metadata_survives_reference_and_geometry_snapshots(self):
        geometry = json.loads(self.geometry.read_text(encoding='utf-8'))
        geometry.update(motion='profile_only', preset=999, execution_resend_count=99)
        self.geometry.write_text(json.dumps(geometry), encoding='utf-8')
        value = self.capture()
        saved = json.loads((self.root/'run/trial.json').read_text(encoding='utf-8'))
        for result in (value, saved):
            self.assertEqual(result['motion'], 'long_r180')
            self.assertEqual(result['preset'], 500)
            self.assertEqual(result['turn_count'], 1)
            self.assertEqual(result['execution_resend_count'], 0)
            self.assertEqual(result['recording_limit_seconds'], 20)
            self.assertTrue(result['camera_fixed_confirmed'])
        snapshot = json.loads((self.root/'run/body_geometry_used.json').read_text(encoding='utf-8'))
        self.assertEqual(snapshot['motion'], 'profile_only')
        self.assertEqual(snapshot['preset'], 999)
        self.assertEqual(sum(' exe_camera ' in command for command in self.sent), 1)

    def test_ten_ms_stream_is_recorded_while_retained_log_stays_one_ms(self):
        self.session.stream_period_ms=10
        self.health['period_ms']=10
        class TenMsTail(FakeTail):
            def __init__(self,*args):
                super().__init__(*args)
                self.rows=[dict(row,sequence=i) for i,row in enumerate(self.rows[::2])]
        with patch.object(tuning,'save_retained_log',return_value=dict(status='saved',period_ms=1)):
            value=tuning.capture_connected(self.session,self.request,self.root/'ten_ms',
                lambda *args:deepcopy(self.current),record=self.record,shell_type=self.shell_type,tail_type=TenMsTail)
        self.assertEqual(value['state'],'CAPTURED_REVIEW_REQUIRED')
        self.assertEqual(value['stream_period_ms'],10)
        sidecar=json.loads((self.root/'ten_ms/motion.settings.json').read_text(encoding='utf-8'))
        self.assertEqual(sidecar['log']['period_ms'],10)
        self.assertEqual(value['saved_log']['period_ms'],1)
        self.assertEqual(sum(' exe_camera ' in command for command in self.sent),1)

    def test_recording_also_applies_requested_five_ms_retained_log_period(self):
        self.request = replace(self.request, log_period_ms=5)
        self.capability += ' saved_log_max_period_ms:100'
        with patch.object(tuning, 'save_retained_log', return_value=dict(status='saved', period_ms=5)) as saved:
            value = self.capture('recording_five_ms')
        self.assertEqual(value['state'], 'CAPTURED_REVIEW_REQUIRED')
        self.assertIn('debug gui period 5', self.sent)
        self.assertEqual(saved.call_args.kwargs, dict(period_ms=5, done_marker='DEBUG_CAMERA_RUN_DONE'))
        self.assertEqual(sum(' exe_camera ' in command for command in self.sent), 1)

    def test_stream_period_readback_mismatch_never_records_or_executes(self):
        self.session.stream_period_ms=10
        self.health['period_ms']=5
        value=self.capture('period_mismatch')
        self.assertEqual(value['state'],'FAILED')
        self.assertFalse(value['execution_write_attempted'])
        self.assertEqual(self.record_count,0)
        self.assertFalse(any(' exe_camera ' in command for command in self.sent))

    def record_with_stop_policy(self, client, project, **kwargs):
        manifest, value = ConnectedTrialTests.record(self, client, project, **kwargs)
        self.camera.phase = 'RECORDING'
        value['recording_stop'] = kwargs['stop_policy'](client, client.last_recording_id, kwargs['seconds'])
        self.camera.phase = 'IDLE'
        self.transfer_seen = True
        return manifest, value

    def test_confirmed_postroll_saves_one_ms_log_before_video_transfer(self):
        self.record = self.record_with_stop_policy
        self.transfer_seen = False
        saved = dict(status='saved', period_ms=1, frame_count=157)
        def retrieve(shell, folder, actual, capability, motion, event):
            stopped.assert_called_once()
            self.assertEqual(self.camera.phase, 'RECORDING')
            self.assertFalse(self.transfer_seen)
            self.assertTrue((folder/'telemetry.csv').exists())
            return saved
        with patch('pixel_camera_stop.wait_motion_stop', return_value=dict(request_stop=True)) as stopped, \
             patch.object(tuning, 'save_retained_log', side_effect=retrieve) as exported:
            value = self.capture()
        exported.assert_called_once()
        self.assertTrue(self.transfer_seen)
        self.assertEqual(value['saved_log'], saved)
        self.assertEqual(value['saved_log_capture_phase'], 'after_confirmed_postroll_before_video_transfer')
        self.assertEqual(value['state'], 'CAPTURED_REVIEW_REQUIRED')
        self.assertTrue(value['log_quality']['full_motion_retained'])
        self.assertEqual(sum(' exe_camera ' in command for command in self.sent), 1)

    def test_pretransfer_log_failure_still_finalizes_video_without_dump_or_motion_retry(self):
        self.record = self.record_with_stop_policy
        self.transfer_seen = False
        with patch('pixel_camera_stop.wait_motion_stop', return_value=dict(request_stop=True)), \
             patch.object(tuning, 'save_retained_log', side_effect=TimeoutError('incomplete log')) as exported:
            value = self.capture()
        self.assertTrue(self.transfer_seen)
        exported.assert_called_once()
        self.assertEqual(value['state'], 'FAILED')
        self.assertEqual(value['saved_log']['status'], 'failed')
        self.assertEqual(value['video_file'], 'mock.mp4')
        self.assertTrue((self.root/'run/motion.csv').exists())
        self.assertEqual(sum(' exe_camera ' in command for command in self.sent), 1)

    def test_video_failure_preserves_pretransfer_saved_log(self):
        def record(*args, **kwargs):
            manifest, value = self.record_with_stop_policy(*args, **kwargs)
            value['state'] = 'FAILED'
            return manifest, value
        self.record = record
        saved = dict(status='saved', period_ms=1, frame_count=157)
        with patch('pixel_camera_stop.wait_motion_stop', return_value=dict(request_stop=True)), \
             patch.object(tuning, 'save_retained_log', return_value=saved) as exported:
            value = self.capture()
        exported.assert_called_once()
        self.assertEqual(value['state'], 'FAILED')
        self.assertEqual(value['saved_log'], saved)
        self.assertTrue(value['execution_write_attempted'])
        self.assertEqual(sum(' exe_camera ' in command for command in self.sent), 1)

    def test_unconfirmed_postroll_keeps_after_transfer_log_fallback(self):
        self.record = self.record_with_stop_policy
        self.transfer_seen = False
        def retrieve(*args):
            self.assertTrue(self.transfer_seen)
            self.assertEqual(self.camera.phase, 'IDLE')
            return dict(status='saved')
        with patch('pixel_camera_stop.wait_motion_stop', return_value=dict(request_stop=False)), \
             patch.object(tuning, 'save_retained_log', side_effect=retrieve) as exported:
            value = self.capture()
        exported.assert_called_once()
        self.assertEqual(value['state'], 'CAPTURED_REVIEW_REQUIRED')
        self.assertEqual(value['saved_log_capture_phase'], 'after_video_transfer_fallback')

    def test_failed_one_ms_transfer_keeps_the_video_and_continuous_log_without_rerun(self):
        with patch.object(tuning, 'save_retained_log', side_effect=TimeoutError('incomplete log')):
            value = self.capture()
        self.assertEqual(value['state'], 'FAILED')
        self.assertEqual(value['saved_log']['status'], 'failed')
        self.assertEqual(value['video_file'], 'mock.mp4')
        self.assertTrue((self.root/'run/motion.csv').exists())
        self.assertEqual(sum(' exe_camera ' in command for command in self.sent), 1)

    def test_search_camera_preserves_signed_feedforward_and_runs_without_hand_wait(self):
        expected = settings()
        expected.update(motion_name='search_turn_right', preset_speed_mm_s=370)
        expected['parameters'].update(velo=.37, r_min=-26, degree=-90, lstart=9.46, lend=11.16)
        expected['feedforward']['ff_sp_turn_accel_mag'] = -.00053817
        self.current = deepcopy(expected)
        self.current['parameters']['lend'] = 12.0
        self.after = expected
        command = tuning.settings_set_command(expected)
        self.assertTrue(command.startswith('debug search_turn right set '))
        self.request = replace(self.request, expected=expected, set_command=command)
        blocked = self.capture('old_firmware')
        self.assertEqual(blocked['state'], 'FAILED')
        self.assertEqual(self.record_count, 0)
        self.assertFalse(any(' arm ' in c or ' exe' in c for c in self.sent))
        self.capability += ' search_turn:1'
        self.sent = []
        result = self.capture('search_supported')
        self.assertEqual(result['state'], 'CAPTURED_REVIEW_REQUIRED')
        self.assertEqual(result['settings_write_count'], 1)
        self.assertTrue(result['camera_run_start_confirmed'])
        self.assertIn('debug search_turn right show', self.sent)
        self.assertEqual(sum(c.startswith('debug search_turn right exe_camera ') for c in self.sent), 1)
        self.assertNotIn('debug search_turn right exe', self.sent)
        self.assertEqual(json.loads((self.root/'search_supported/telemetry.settings.json').read_text())['feedforward'], expected['feedforward'])

    def test_gui_search_settings_select_direction_and_search_preset(self):
        from pixel_camera_tuning_panel import gui_expected_settings
        from types import SimpleNamespace
        def var(value):
            return SimpleNamespace(get=lambda:value)
        expected = settings()
        values = dict(distance=.37,acc=26.,max_velo=9.46,end_velo=11.16,degree=90,degree_correction=0,
                      **expected['pid'], **expected['feedforward'])
        gui = SimpleNamespace(motion=var('search_turn'), direction=var('left'),
            search_turn_preset_speed=var(370), turn_type=var('long_r180'),turn_preset_speed=var(500),
            turn_count=var(1), pre_accel=var(6.5), suction_enable=var(False),suction_duty=var(650),
            values={key:var(value) for key,value in values.items()})
        actual = gui_expected_settings(gui,'A')
        self.assertEqual(actual['motion_name'],'search_turn_left')
        self.assertEqual(actual['preset_speed_mm_s'],370)
        self.assertEqual(actual['parameters']['r_min'],26.)

    def test_two_trials_reuse_connection_and_skip_unchanged_parameter_writes(self):
        for name in ('one','two'):
            value=self.capture(name)
            self.assertEqual(value['state'],'CAPTURED_REVIEW_REQUIRED')
            self.assertEqual(value['settings_write_count'],0)
            self.assertTrue(value['log_stop_evidence']['confirmed'])
            self.assertTrue(Path(value['type8_settings_json']).exists())
        self.assertFalse(any(' set ' in command or ' preset ' in command for command in self.sent))
        self.assertEqual(sum(' exe_camera ' in command for command in self.sent),2)
        self.assertTrue(self.session.connection.is_open)
        self.session.close.assert_not_called()

    def test_changed_setting_is_written_once_and_read_back_before_recording(self):
        self.current=settings(24.47)
        value=self.capture()
        self.assertEqual(value['settings_write_count'],1)
        self.assertEqual(self.sent.count(self.request.set_command),1)
        self.assertLess(self.sent.index(self.request.set_command),next(i for i,c in enumerate(self.sent) if ' exe_camera ' in c))

    def suction_request(self):
        expected = settings()
        expected.update(motion_name='long_r90',preset_speed_mm_s=1600)
        expected['parameters'].update(velo=1.6,r_min=-52.,lstart=15.97,lend=27.58,degree=-90.)
        expected['feedforward']['ff_sp_turn_accel_mag'] = -.00019277
        expected['suction'] = dict(enabled=True,duty=650)
        self.current = deepcopy(expected)
        self.current['suction']['enabled'] = False
        self.after = deepcopy(expected)
        self.request = replace(self.request,expected=expected,set_command=tuning.settings_set_command(expected))

    def test_turn1600_suction_on_is_set_once_verified_and_recorded(self):
        self.suction_request()
        value = self.capture()
        self.assertEqual(value['state'],'CAPTURED_REVIEW_REQUIRED')
        self.assertEqual(self.sent.count(self.request.set_command),1)
        self.assertEqual(self.request.set_command.split()[-3:],['1','650','1'])
        self.assertEqual((value['suction_enabled'],value['suction_duty']),(True,650))
        actual = json.loads((self.root/'run/telemetry.settings.json').read_text())
        self.assertEqual(actual['suction'],dict(enabled=True,duty=650))
        self.assertEqual(sum(' exe_camera ' in command for command in self.sent),1)

    def test_suction_on_readback_mismatch_never_records_or_executes(self):
        self.suction_request()
        self.after['suction']['enabled'] = False
        value = self.capture()
        self.assertEqual(value['state'],'FAILED')
        self.assertEqual(self.record_count,0)
        self.assertFalse(any(' arm ' in command or ' exe' in command for command in self.sent))

    def test_stop_policy_is_attached_after_single_execution_and_uses_only_new_motion(self):
        original=self.record
        observed = []
        def monitor_stop(*args, **kwargs):
            observed.extend(kwargs['tail'].snapshot())
            return dict(request_stop=True)
        with patch('pixel_camera_stop.wait_motion_stop',side_effect=monitor_stop) as monitor:
            def record(client,project,**kwargs):
                manifest,value=original(client,project,**kwargs)
                self.assertEqual(sum(' exe_camera ' in c for c in self.sent),1)
                value['recording_stop']=kwargs['stop_policy'](client,client.last_recording_id,kwargs['seconds'])
                return manifest,value
            self.record=record
            value=self.capture()
        self.assertEqual(value['state'],'CAPTURED_REVIEW_REQUIRED')
        self.assertEqual(value['recording_stop'],dict(request_stop=True))
        monitor.assert_called_once()
        # FakeTailには既に過去の動作がある。それを今回の停止判定に流用しない。
        self.assertEqual(len(observed),1)
        self.assertEqual(value['recording_limit_seconds'],20)

    def test_mismatched_readback_does_not_record_arm_or_execute(self):
        self.current=settings(24.47)
        self.after=settings(18)
        value=self.capture()
        self.assertEqual(value['state'],'FAILED')
        self.assertEqual(self.record_count,0)
        self.assertFalse(any(' arm ' in c or ' exe_camera ' in c for c in self.sent))
        self.assertEqual(json.loads((self.root/'run/telemetry.settings.json').read_text())['parameters']['lend'],18)

    def test_record_acceptance_or_another_owner_is_not_recording_ready(self):
        for phase,owner in [('PREPARING','operator'),('RECORDING','other')]:
            with self.subTest(phase=phase,owner=owner):
                self.phase_on_ready,self.camera.owner=phase,owner
                self.camera.phase='IDLE';self.sent=[]
                value=self.capture(phase+owner)
                self.assertFalse(value['execution_write_attempted'])
                self.assertFalse(any(' arm ' in c or ' exe_camera ' in c for c in self.sent))

    def test_ambiguous_execution_is_saved_without_retry_and_folder_cannot_be_reused(self):
        self.fail_write=True
        value=self.capture()
        self.assertTrue(value['execution_write_attempted'])
        self.assertFalse(value['camera_run_start_confirmed'])
        self.assertEqual(sum(' exe_camera ' in c for c in self.sent),1)
        self.assertEqual(json.loads((self.root/'run/trial.json').read_text())['execution_resend_count'],0)
        with self.assertRaises(FileExistsError):self.capture()
        self.assertEqual(sum(' exe_camera ' in c for c in self.sent),1)

    def test_operator_confirmation_low_battery_and_stale_health_block_motion(self):
        self.request=replace(self.request,operator_ready=False)
        with self.assertRaises(ValueError):self.capture('unconfirmed')
        self.assertEqual(self.sent,[])
        self.request=replace(self.request,operator_ready=True)
        self.low_battery=True
        value=self.capture('battery')
        self.assertFalse(value['execution_write_attempted'])
        self.low_battery=False;self.health['monotonic']-=10
        self.sent=[]
        value=self.capture('stale')
        self.assertEqual(self.sent,[])
        self.assertFalse(value['execution_write_attempted'])

    def test_crc_error_or_missing_motion_never_reports_a_complete_trial(self):
        original=self.record
        def corrupted(*args,**kwargs):
            result=original(*args,**kwargs)
            self.health['crc_errors']+=1
            return result
        self.record=corrupted
        value=self.capture('crc')
        self.assertEqual(value['state'],'INCOMPLETE_LOG')
        self.assertEqual(value['log_quality']['crc_errors_during_trial'],1)
        self.record=original
        class IdleTail(FakeTail):
            def __init__(self,*args):
                super().__init__(*args)
                for row in self.rows:
                    for key in ('ideal.velo','ideal.rad_velo','ego.velo','ego.rad_velo'):row[key]=0
        value=tuning.capture_connected(self.session,self.request,self.root/'idle',
            lambda *args:deepcopy(self.current),record=self.record,shell_type=self.shell_type,tail_type=IdleTail)
        self.assertEqual(value['state'],'INCOMPLETE_LOG')
        self.assertFalse(value['log_stop_evidence']['confirmed'])

    def test_csv_loss_after_recording_acceptance_never_arms_or_executes(self):
        class LostTail(FakeTail):
            lost=False
            @property
            def latest_at(self):
                return time.monotonic()-(10 if self.lost else 0)
        holder=[]
        def factory(*args):
            result=LostTail(*args);holder.append(result);return result
        def record(client,project,**kwargs):
            self.camera.last_recording_id='recording';self.camera.phase='RECORDING'
            holder[0].lost=True
            kwargs['on_ready']()
            raise AssertionError('stale telemetry should prevent callback completion')
        value=tuning.capture_connected(self.session,self.request,self.root/'stale_csv',
            lambda *args:deepcopy(self.current),record=record,shell_type=self.shell_type,tail_type=factory)
        self.assertFalse(value['execution_write_attempted'])
        self.assertFalse(any(' arm ' in c or ' exe_camera ' in c for c in self.sent))


class GuiProfileTests(unittest.TestCase):
    def test_button_uses_edited_parameters_without_video_and_consumes_confirmation(self):
        from myshell_debug import DebugGui
        import pixel_camera_tuning_panel as panel
        gui = DebugGui()
        gui.withdraw()
        try:
            control = gui.camera_panel
            control.mouse.set('A'); control.record_video.set(False); control.log_period.set(5)
            gui.motion.set('turn'); gui.turn_type.set('long_r180'); gui.turn_preset_speed.set(500)
            value = settings()
            mapping = dict(distance='velo', acc='r_min', max_velo='lstart', end_velo='lend',
                           degree='degree', degree_correction='correction')
            for name, key in mapping.items(): gui.values[name].set(value['parameters'][key])
            gui.pre_accel.set(value['parameters']['pre_accel'])
            for group in ('pid', 'feedforward'):
                for name, number in value[group].items(): gui.values[name].set(number)
            gui.values['end_velo'].set(17.25)
            control.ready.set(True)
            gui.ser = SimpleNamespace(is_open=True)
            control.session = SimpleNamespace(connection=gui.ser, folder=Path('unused-test-session'))
            with patch.object(control, '_launch', return_value=True) as launch, \
                    patch.object(panel, 'capture_connected', return_value=dict(state='LOGGED_REVIEW_REQUIRED')) as capture:
                control.run()
                self.assertFalse(control.ready.get())
                launch.assert_called_once()
                launch.call_args.args[0]()
                request = capture.call_args.args[1]
                self.assertFalse(request.record_video)
                self.assertEqual(request.log_period_ms, 5)
                self.assertEqual(request.expected['parameters']['lend'], 17.25)
                self.assertIn('17.25', request.set_command)
                gui._camera_busy = True
                control.run()
                launch.assert_called_once()
        finally:
            gui.ser = None
            gui.camera_panel.session = None
            gui.destroy()

    def test_tuning_controls_are_independent_and_opening_does_not_prepare_devices(self):
        import tkinter as tk
        from tkinter import ttk
        import pixel_camera_tuning_panel as panel
        root = tk.Tk(); root.withdraw()
        try:
            settings = dict(ev=-2.5, torch=False, zoom_ratio=1.2, focus_lock=True)
            with patch.object(panel, 'load_camera_profile', return_value=settings) as load, \
                    patch.object(panel, 'LiveSession') as session:
                controls = panel.PixelTuningPanel(SimpleNamespace(), ttk.Notebook(root))
                load.assert_called_once_with('tuning')
                session.assert_not_called()
                self.assertEqual(controls.selected_camera_settings(), settings)
                controls.capture_zoom.set(1.4)
                self.assertEqual(controls.selected_camera_settings()['zoom_ratio'], 1.4)
                controls.record_video.set(False)
                controls.log_period.set(5)
                self.assertFalse(controls.record_video.get())
                self.assertEqual(controls.log_period.get(), 5)
                self.assertIsInstance(controls.preview, tk.Canvas)
        finally:
            root.destroy()


class EvidenceTests(unittest.TestCase):
    def test_quantization_never_hides_a_real_parameter_change_or_nonfinite(self):
        actual,expected=settings(),settings()
        expected['feedforward']['ff_sp_accel']+=1e-6
        self.assertTrue(tuning.setting_differences(actual,expected))
        self.assertFalse(tuning.setting_differences(actual,expected,quantized=True))
        expected['parameters']['lend']-=4
        self.assertIn('parameters.lend',tuning.setting_differences(actual,expected,quantized=True))
        expected=settings();expected['parameters']['lend']=float('nan')
        self.assertTrue(tuning.setting_differences(expected,expected))

    def test_stop_requires_motion_finite_values_and_two_hundred_ms_of_observed_rest(self):
        rows=motion_rows()
        self.assertTrue(tuning.log_stop_evidence(rows)['confirmed'])
        self.assertFalse(tuning.log_stop_evidence(rows[:100])['confirmed'])
        idle=deepcopy(rows)
        for row in idle:row['ideal.velo']=row['ideal.rad_velo']=0
        self.assertFalse(tuning.log_stop_evidence(idle)['confirmed'])
        rows[30]['ego.velo']=float('nan')
        self.assertFalse(tuning.log_stop_evidence(rows)['confirmed'])

    def test_tail_starts_at_new_rows_and_waits_for_partial_line_completion(self):
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory);source=folder/'source.csv'
            source.write_text('time_ms,sequence\n100,1\n105,',encoding='utf-8')
            tail=tuning.CsvTail(source,folder/'out.csv')
            try:
                with source.open('a',encoding='utf-8') as stream:stream.write('2\n110,3\n')
                deadline=time.monotonic()+2
                while len(tail.snapshot())<2 and time.monotonic()<deadline:time.sleep(.02)
            finally:tail.close()
            self.assertEqual(tail.snapshot(),[dict(time_ms='105',sequence='2'),dict(time_ms='110',sequence='3')])
            tail2=tuning.CsvTail(source,folder/'next.csv')
            try:
                with source.open('a',encoding='utf-8') as stream:stream.write('115,4\n')
                deadline=time.monotonic()+2
                while not tail2.snapshot() and time.monotonic()<deadline:time.sleep(.02)
            finally:tail2.close()
            self.assertEqual(tail2.snapshot(),[dict(time_ms='115',sequence='4')])

    def test_health_reader_ignores_incomplete_append(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'health.jsonl'
            good=dict(rows=1,lost=0,crc_errors=0,monotonic=time.monotonic())
            path.write_text(json.dumps(good)+'\n{"rows":',encoding='utf-8')
            self.assertEqual(tuning.last_health(path),good)


if __name__=='__main__':unittest.main()
