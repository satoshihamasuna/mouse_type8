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
                return 'DEBUG_CAMERA_CAP protocol:1 ttl_ms:15000' if command=='debug camera show' else ('after' if self.applied else 'before')
            def wait_line(self,predicate,timeout=5):
                if ' set ' in outer.sent[-1]:line='DEBUG_TURN_SET_DONE'
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

    def test_stop_policy_is_attached_after_single_execution_and_uses_only_new_motion(self):
        original=self.record
        with patch('pixel_camera_stop.wait_motion_stop',return_value=dict(request_stop=True)) as monitor:
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
        self.assertEqual(len(monitor.call_args.kwargs['tail'].snapshot()),1)
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
