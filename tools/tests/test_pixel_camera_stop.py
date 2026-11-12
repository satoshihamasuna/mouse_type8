"""実機へ接続せず、停止後の余白・上限への退避・短縮動画検証を確認する。"""
from contextlib import redirect_stdout
from copy import deepcopy
import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pixel_camera_capture import record_trial
from pixel_camera_stop import inspect_controlled_stop, wait_motion_stop
from pixel_camera_tuning import log_stop_evidence


class Clock:
    t = 0.0
    def now(self):
        return 100+self.t
    def sleep(self, seconds):
        self.t += seconds


class StopMonitorTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.client = SimpleNamespace(client_id='ours')
        self.client.get_status = lambda: dict(connected=True, device=dict(owner='ours',
            recording_id='clip', phase='RECORDING', elapsed_seconds=1+int(self.clock.t*4)/4))
        self.rows = self.samples
        self.stale = self.crc = self.lost = self.gap = self.nan = False
        outer = self
        class Tail:
            @property
            def latest_at(self):
                return outer.clock.now()-(1 if outer.stale else 0)
            def snapshot(self):
                return outer.rows()
        self.tail = Tail()
        self.events = []

    def samples(self):
        rows = []
        for i in range(int(self.clock.t/.005)+1):
            active = .2 <= i*.005 <= .7
            rows.append(dict(time_ms=i*5, sequence=i, lost_frames=int(self.lost), **{
                'ideal.velo':.5 if active else 0, 'ideal.rad_velo':0,
                'ego.velo':.5 if active else 0, 'ego.rad_velo':0}))
        if self.gap and len(rows)>5:
            del rows[3]
        if self.nan:
            rows[-1]['ego.velo']=float('nan')
        return rows

    def run_monitor(self):
        return wait_motion_stop(self.client,'clip',20,tail=self.tail,
            health=lambda:dict(monotonic=self.clock.now(),crc_errors=int(self.crc),
                               latest_sequence=int(self.clock.t/.005)), baseline_crc=0,
            alive=lambda:True,stop_evidence=log_stop_evidence,
            event=lambda name,**fields:self.events.append((name,fields)),
            now=self.clock.now,sleep=self.clock.sleep)

    def test_waits_for_real_motion_then_stop_and_one_second_in_all_three_clocks(self):
        result=self.run_monitor()
        self.assertTrue(result['request_stop'])
        self.assertGreaterEqual(result['stationary_ms'],1200)
        for key, minimum in [('postroll_elapsed_seconds',1),('postroll_camera_seconds',1),('postroll_mcu_ms',1000)]:
            self.assertGreaterEqual(result[key],minimum)
        self.assertLess(self.clock.t,4)
        self.assertTrue(any(name=='recording_postroll' for name,_ in self.events))

    def test_measured_movement_during_postroll_starts_a_new_one_second_wait(self):
        original=self.rows
        def interrupted():
            rows=original()
            for row in rows:
                if 1400 <= row['time_ms'] <= 1600:row['ego.velo']=.1
            return rows
        self.rows=interrupted
        result=self.run_monitor()
        self.assertTrue(result['request_stop'])
        self.assertGreaterEqual(self.clock.t,2.8)
        self.assertEqual(sum(name=='recording_postroll' for name,_ in self.events),2)

    def test_only_idle_never_stops_early_and_exits_at_camera_cap(self):
        original=self.rows
        def idle():
            rows=original()
            for row in rows:row['ideal.velo']=row['ego.velo']=0
            return rows
        self.rows=idle
        result=self.run_monitor()
        self.assertFalse(result['request_stop'])
        self.assertEqual(result['fallback_reason'],'duration_cap_near')

    def test_stale_csv_crc_missing_frames_and_nonfinite_values_use_cap(self):
        for flag in ('stale','crc','lost','gap','nan'):
            with self.subTest(flag=flag):
                self.setUp();setattr(self,flag,True)
                self.assertFalse(self.run_monitor()['request_stop'])

    def test_owner_id_phase_or_connection_change_cannot_request_stop(self):
        for key,value in [('owner','another'),('recording_id','another'),('phase','FINALIZING')]:
            self.client.get_status=lambda key=key,value=value: dict(connected=True,
                device=dict(dict(owner='ours',recording_id='clip',phase='RECORDING',elapsed_seconds=2),**{key:value}))
            self.assertFalse(self.run_monitor()['request_stop'])
        self.client.get_status=lambda:dict(connected=False)
        self.assertFalse(self.run_monitor()['request_stop'])


def inspection_fixture():
    return dict(passed=False,errors=['UNEXPECTED_STOP_REASON','FIRST_FRAME_DURATION_NOT_CONFIRMED',
                'VIDEO_DURATION_MISMATCH','PTS_SPAN_MISMATCH'],
                pts_span_seconds=4.,duration_seconds=4+1/240,decoded_frames=961,mp4_time_base_seconds=1/90000)


def metadata_fixture():
    return dict(stop_reason='USER_OR_ACTIVITY_STOP',requested_duration_seconds=20,
        encoded_timing=dict(span_seconds=4.),mp4=dict(sample_timing=dict(span_seconds=4.)),
        recording_window=dict(duration_limit_us=20_000_000,origin='first_encoded_frame_pts',
            timestamps_rescaled=False,limit_reached=False,last_output_relative_pts_us=4_000_000))


def decision_fixture():
    return dict(request_stop=True,stationary_ms=1250,postroll_elapsed_seconds=1.05,
        postroll_mcu_ms=1050,postroll_camera_seconds=1.,request_camera_seconds=3.8,
        stop_write_attempted=True)


class ControlledInspectionTests(unittest.TestCase):
    def test_intentional_short_clip_replaces_only_duration_requirements(self):
        result=inspect_controlled_stop(inspection_fixture(),metadata_fixture(),20,decision_fixture())
        self.assertTrue(result['passed'])
        self.assertEqual(result['recording_stop_mode'],'motion_end_plus_postroll')
        self.assertEqual(len(result['fixed_duration_errors']),4)

    def test_hash_frame_count_decode_fps_finalization_and_transfer_errors_remain_failures(self):
        for error in ('PC_RECHECK_SIZE_OR_SHA256_MISMATCH','FRAME_COUNT_MISMATCH',
            'DECODE_OR_PTS_ERROR','TARGET_FPS_NOT_OBSERVED','INCOMPLETE_MP4',
            'ANDROID_RECORDING_ERROR','SOURCE_DELETION_NOT_CONFIRMED','PTS_ORIGIN_MISMATCH'):
            with self.subTest(error=error):
                inspection=inspection_fixture();inspection['errors'].append(error)
                result=inspect_controlled_stop(inspection,metadata_fixture(),20,decision_fixture())
                self.assertFalse(result['passed'])
                self.assertIn(error,result['errors'])

    def test_unrequested_or_insufficient_postroll_does_not_relax_fixed_validation(self):
        for key,value in [('stop_write_attempted',False),('request_stop',False),
            ('postroll_elapsed_seconds',.99),('postroll_mcu_ms',999),('postroll_camera_seconds',.99),
            ('stationary_ms',1199),('postroll_elapsed_seconds',float('nan'))]:
            stop=decision_fixture();stop[key]=value
            result=inspect_controlled_stop(inspection_fixture(),metadata_fixture(),20,stop)
            self.assertFalse(result['passed'])
            self.assertIn('UNEXPECTED_STOP_REASON',result['errors'])

    def test_changed_window_pts_and_duration_are_rejected(self):
        for key,value in [('duration_limit_us',30_000_000),('origin','wall_clock'),
            ('timestamps_rescaled',True),('last_output_relative_pts_us',3_800_000),('limit_reached',True)]:
            metadata=metadata_fixture();metadata['recording_window'][key]=value
            self.assertFalse(inspect_controlled_stop(inspection_fixture(),metadata,20,decision_fixture())['passed'])
        for span,duration in [(20,20.004),(4,4.1),(1,1.004),(float('nan'),4.004)]:
            inspection=inspection_fixture();inspection.update(pts_span_seconds=span,duration_seconds=duration)
            self.assertFalse(inspect_controlled_stop(inspection,metadata_fixture(),20,decision_fixture())['passed'])
        stop=decision_fixture();stop['request_camera_seconds']=6
        self.assertFalse(inspect_controlled_stop(inspection_fixture(),metadata_fixture(),20,stop)['passed'])

    def test_real_pixel_muxer_rounding_matches_mp4_timeline_without_stretching(self):
        # 2026-11-10の初回短縮録画。旧検査は1.289msの差だけで失敗していた。
        inspection=inspection_fixture()
        inspection.update(pts_span_seconds=9.56991111111111,duration_seconds=9.574077777777777,decoded_frames=2296)
        metadata=metadata_fixture()
        metadata['recording_window']['last_output_relative_pts_us']=9571200
        metadata['encoded_timing']['span_seconds']=9.5712
        metadata['mp4']['sample_timing']['span_seconds']=9.569911
        stop=decision_fixture();stop['request_camera_seconds']=9.087401
        result=inspect_controlled_stop(inspection,metadata,20,stop)
        self.assertTrue(result['passed'])
        self.assertEqual(result['pts_span_seconds'],inspection['pts_span_seconds'])
        self.assertGreater(result['muxer_quantization_bound_seconds'],.001289)
        metadata['mp4']['sample_timing']['span_seconds']=9.5702
        self.assertFalse(inspect_controlled_stop(inspection,metadata,20,stop)['passed'])

    def test_unknown_clock_and_encoder_drift_beyond_actual_tick_bound_fail(self):
        for tick in (None,0,float('nan'),1/30):
            inspection=inspection_fixture();inspection['mp4_time_base_seconds']=tick
            self.assertFalse(inspect_controlled_stop(inspection,metadata_fixture(),20,decision_fixture())['passed'])
        metadata=metadata_fixture()
        metadata['recording_window']['last_output_relative_pts_us']=4_020_000
        metadata['encoded_timing']['span_seconds']=4.02
        result=inspect_controlled_stop(inspection_fixture(),metadata,20,decision_fixture())
        self.assertIn('CONTROLLED_STOP_TIMEBASE_MISMATCH',result['errors'])

    def test_duration_cap_keeps_original_strict_inspection_unchanged(self):
        metadata=metadata_fixture();metadata['stop_reason']='DURATION'
        inspection=inspection_fixture()
        result=inspect_controlled_stop(inspection,metadata,20,decision_fixture())
        self.assertFalse(result['passed'])
        self.assertEqual(result['errors'],inspection['errors'])
        inspection.update(passed=True,errors=[])
        self.assertTrue(inspect_controlled_stop(inspection,metadata,20,{})['passed'])


class CaptureStopTests(unittest.TestCase):
    def setUp(self):
        self.directory=tempfile.TemporaryDirectory();self.addCleanup(self.directory.cleanup)
        self.root=Path(self.directory.name)
        self.metadata=self.root/'video.json'
        self.metadata.write_text(json.dumps(metadata_fixture()),encoding='utf-8')
        self.result=SimpleNamespace(file_path=self.root/'clip.mp4',metadata_path=self.metadata,
                                    sha256='test',size=123,source_deleted=True)
        self.client=Mock(client_id='ours')
        self.client.start_recording.return_value='clip'
        self.status=dict(connected=True,device=dict(recording_id='clip',owner='ours',phase='RECORDING',elapsed_seconds=3.8))
        self.client.get_status.side_effect=lambda:deepcopy(self.status)
        self.client.stop_recording.return_value=dict(accepted=True)
        self.client.wait_for_transfer.return_value=self.result
        self.ready=Mock()
        self.policy=Mock(side_effect=lambda *args:decision_fixture())
        self.inspector=Mock(return_value=inspection_fixture())
        def atomic(path,value):
            path.write_text(json.dumps(value),encoding='utf-8')
        modules={'pixel_camera.transfer':SimpleNamespace(atomic_json=atomic),
                 'pixel_camera.stability':SimpleNamespace(inspect_recording=self.inspector)}
        self.addCleanup(patch.stopall)
        patch.dict(sys.modules,modules).start()
        patch('pixel_camera_stop.video_time_base',return_value=1/90000).start()

    def capture(self):
        with redirect_stdout(io.StringIO()):
            return record_trial(self.client,self.root,seconds=20,on_ready=self.ready,stop_policy=self.policy)[1]

    def test_ready_and_motion_callback_precede_stop_policy_and_request_is_once(self):
        self.policy.side_effect=lambda *args: (self.ready.assert_called_once() or decision_fixture())
        value=self.capture()
        self.assertEqual(value['state'],'PASSED')
        self.client.start_recording.assert_called_once()
        self.client.stop_recording.assert_called_once()
        self.assertTrue(value['recording_stop']['stop_write_attempted'])
        saved=list((self.root/'tools/logs/pixel_camera').glob('*.capture.json'))
        self.assertTrue(json.loads(saved[0].read_text())['video_inspection']['passed'])

    def test_lost_stop_response_and_transfer_recovery_never_resend_stop_or_motion(self):
        self.client.stop_recording.side_effect=TimeoutError('lost response')
        self.client.wait_for_transfer.side_effect=[TimeoutError('transfer delayed'),self.result]
        value=self.capture()
        self.assertEqual(value['state'],'FAILED')
        self.assertIn('lost response',value['recording_stop']['stop_error'])
        self.client.stop_recording.assert_called_once()
        self.ready.assert_called_once()
        self.client.start_recording.assert_called_once()

    def test_manifest_access_denied_after_start_recovers_without_restarting_capture(self):
        transfer=sys.modules['pixel_camera.transfer']
        writer=transfer.atomic_json
        denied=False
        def locked_once(path,value):
            nonlocal denied
            if value.get('recording_id') and value['state']=='PREPARING' and not denied:
                denied=True
                error=PermissionError('manifest replace temporarily denied')
                error.winerror=5
                raise error
            writer(path,value)
        with patch.object(transfer,'atomic_json',side_effect=locked_once), patch('pixel_camera_capture.time.sleep'):
            value=self.capture()
        self.assertTrue(denied)
        self.assertEqual(value['state'],'PASSED')
        self.client.start_recording.assert_called_once()
        self.ready.assert_called_once()
        self.client.stop_recording.assert_called_once()

    def test_persistent_manifest_lock_fails_before_motion_and_keeps_recovery_record(self):
        transfer=sys.modules['pixel_camera.transfer']
        writer=transfer.atomic_json
        def locked(path,value):
            if value.get('recording_id') and value['state']=='PREPARING':
                error=PermissionError('manifest remains locked')
                error.winerror=32
                raise error
            writer(path,value)
        with patch.object(transfer,'atomic_json',side_effect=locked) as writes, patch('pixel_camera_capture.time.sleep') as sleep:
            value=self.capture()
        self.assertEqual(value['state'],'FAILED')
        self.assertIn('PermissionError',value['error'])
        self.assertIn('recovered_video_file',value)
        self.assertAlmostEqual(sum(call.args[0] for call in sleep.call_args_list),.35)
        self.assertLess(writes.call_count,10)
        self.ready.assert_not_called()
        self.client.start_recording.assert_called_once()
        self.client.stop_recording.assert_called_once()

    def test_unrelated_save_error_is_not_retried_and_never_starts_capture(self):
        transfer=sys.modules['pixel_camera.transfer']
        with patch.object(transfer,'atomic_json',side_effect=PermissionError('not a Windows sharing error')) as writes, patch('pixel_camera_capture.time.sleep') as sleep:
            with self.assertRaises(PermissionError):
                self.capture()
        writes.assert_called_once()
        sleep.assert_not_called()
        self.ready.assert_not_called()
        self.client.start_recording.assert_not_called()

    def test_final_manifest_lock_still_disconnects_without_repeating_motion(self):
        transfer=sys.modules['pixel_camera.transfer']
        writer=transfer.atomic_json
        def final_locked(path,value):
            if 'finished_at' in value:
                error=PermissionError('final manifest remains locked')
                error.winerror=33
                raise error
            writer(path,value)
        with patch.object(transfer,'atomic_json',side_effect=final_locked), patch('pixel_camera_capture.time.sleep'):
            with self.assertRaises(PermissionError):
                self.capture()
        self.client.disconnect.assert_called_once()
        self.client.start_recording.assert_called_once()
        self.client.stop_recording.assert_called_once()
        self.ready.assert_called_once()

    def test_lost_stop_response_can_be_confirmed_by_finalized_video_without_resend(self):
        self.client.stop_recording.side_effect=TimeoutError('lost response')
        self.assertEqual(self.capture()['state'],'PASSED')
        self.client.stop_recording.assert_called_once()

    def test_owner_changed_between_monitor_and_stop_does_not_send_stop(self):
        def changed(*args):
            self.status['device']['owner']='another'
            return decision_fixture()
        self.policy.side_effect=changed
        value=self.capture()
        self.client.stop_recording.assert_not_called()
        self.assertFalse(value['recording_stop']['stop_write_attempted'])
        self.assertEqual(value['state'],'FAILED')

    def test_cap_fallback_never_calls_stop_and_uses_fixed_duration_validation(self):
        self.policy.return_value=None
        self.policy.side_effect=lambda *args:dict(request_stop=False,fallback_reason='stale_csv')
        metadata=metadata_fixture();metadata['stop_reason']='DURATION'
        self.metadata.write_text(json.dumps(metadata),encoding='utf-8')
        self.inspector.return_value=dict(passed=True,errors=[])
        value=self.capture()
        self.assertEqual(value['state'],'PASSED')
        self.client.stop_recording.assert_not_called()
        self.assertEqual(value['recording_stop']['fallback_reason'],'stale_csv')


if __name__ == '__main__':
    unittest.main()
