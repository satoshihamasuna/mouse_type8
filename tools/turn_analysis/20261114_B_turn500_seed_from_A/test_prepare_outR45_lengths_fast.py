"""機器なしで、B値保持・失敗時停止・斜め配置の誤り検出を確認する。"""
from copy import deepcopy
from datetime import datetime, timedelta, timezone
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch
import numpy as np
import prepare_outR45_lengths_fast as fast
import preflight_outR45_lengths_zoom2 as preflight
BASE = Path(__file__).resolve().parent

class OutR45PreparationGates(unittest.TestCase):
    def setUp(self):
        self.plan = fast.load(BASE/'B_outR45_Lstart32p8_Lend18_plan.json')
        self.baseline = dict(mouse='B', motion_name='out_r45', preset_speed_mm_s=500,
            turn_count=1, parameters=deepcopy(self.plan['expected_compiled_B']),
            pid=deepcopy(self.plan['expected_B_pid']), suction=dict(enabled=False,duty=650),
            feedforward=dict(ff_sp_velo=.617283,ff_om_accel=.000421,ff_om_jerk=1e-7),
            raw_response='B preset baseline')

    def test_fresh_B_control_preserved_without_copying_A(self):
        original = deepcopy(self.baseline)
        candidate = fast.build_candidate(self.baseline,self.plan)
        self.assertEqual(self.baseline,original)
        self.assertEqual(candidate['pid'],self.plan['expected_candidate_pid'])
        for key in ('feedforward','suction'):
            self.assertEqual(candidate[key],original[key])
        self.assertEqual(candidate['parameters'],self.plan['expected_candidate_parameters'])
        self.assertNotIn('raw_response',candidate)

    def test_wrong_turn_or_A_PID_rejected(self):
        for field,value in [('motion_name','in_r135'),('mouse','A')]:
            changed=deepcopy(self.baseline);changed[field]=value
            with self.assertRaises(AssertionError):fast.build_candidate(changed,self.plan)
        changed=deepcopy(self.baseline);changed['pid']['om_ki']=.01
        with self.assertRaises(AssertionError):fast.build_candidate(changed,self.plan)

    def test_apply_changed_B_FF_rejected(self):
        candidate=fast.build_candidate(self.baseline,self.plan)
        actual=deepcopy(candidate);actual['feedforward']['ff_om_accel']+=.0001
        with self.assertRaises(AssertionError):fast.check_candidate(actual,candidate)

    def test_stale_ready_stops_before_connection(self):
        with tempfile.TemporaryDirectory() as directory:
            session=Path(directory)
            meta=dict(session=str(session),operator_ready='USER_CONFIRMED_POSITION_HANDS_OFF_BOTH_ON',capture_request_id='test')
            fast.save(session/'operator_ready.json',dict(placement_stopped_clearance_hands_off_both_on_confirmed=True,
                capture_request_id='test',recorded_at=(datetime.now(timezone.utc)-timedelta(minutes=6)).isoformat()))
            fast.save(session/'meta.json',meta)
            with patch.object(fast,'validate_files',return_value=({},self.plan)),patch.object(fast.subprocess,'Popen') as start:
                with self.assertRaises(AssertionError):fast.prepare(session/'meta.json')
                start.assert_not_called()

    def test_failed_preset_never_applies_or_records(self):
        with tempfile.TemporaryDirectory() as directory:
            session=Path(directory)
            meta=dict(session=str(session),operator_ready='USER_CONFIRMED_POSITION_HANDS_OFF_BOTH_ON',capture_request_id='test',
                config='unused.json',preset_readback=str(session/'preset.json'))
            fast.save(session/'operator_ready.json',dict(placement_stopped_clearance_hands_off_both_on_confirmed=True,
                capture_request_id='test',recorded_at=datetime.now(timezone.utc).isoformat()))
            fast.save(session/'meta.json',meta)
            def connected(*args,**kwargs):
                fast.save(session/'service.json',{})
                return Mock(pid=123,poll=Mock(return_value=None))
            with patch.object(fast,'validate_files',return_value=({},self.plan)),patch.object(fast.subprocess,'Popen',side_effect=connected),patch.object(fast.subprocess,'run',return_value=Mock(returncode=1,stdout='',stderr='simulated failure')) as invoke:
                with self.assertRaises(AssertionError):fast.prepare(session/'meta.json')
                self.assertEqual(invoke.call_count,1)
                self.assertIn('preset',invoke.call_args.args[0])
                self.assertNotIn('capture',invoke.call_args.args[0])
                self.assertNotIn('apply',invoke.call_args.args[0])

    def test_real_firmware_single_LSB_truncation_accepted(self):
        actual = fast.load(BASE/'next_session_B_outR45_Aseed_v1.json')
        actual = fast.load(actual['preset_readback'])
        self.assertEqual(actual['parameters']['lend'],32.009)
        fast.check_baseline(actual,self.plan)
        candidate=fast.build_candidate(actual,self.plan)
        returned=deepcopy(candidate);returned['parameters']['lend']-=.001
        fast.check_candidate(returned,candidate)

    def test_parameter_difference_beyond_one_LSB_rejected(self):
        actual=deepcopy(self.baseline);actual['parameters']['lend']-=.002
        with self.assertRaises(AssertionError):fast.check_baseline(actual,self.plan)

    def test_nonfinite_parameter_rejected(self):
        actual=deepcopy(self.baseline);actual['parameters']['lend']=float('nan')
        with self.assertRaises(AssertionError):fast.check_baseline(actual,self.plan)

    def test_unplanned_PID_change_rejected(self):
        changed=deepcopy(self.plan);changed['expected_candidate_pid']['om_ki']=.01
        with self.assertRaises(AssertionError):fast.build_candidate(self.baseline,changed)

    def inspect_pose(self,centre,forward):
        import video_marker_pose
        centre=np.asarray(centre);forward=np.asarray(forward)
        geometry=dict(marker_detection={},nominal_marker_span_mm=50,marker_span_limits_mm=[40,65])
        with patch.object(video_marker_pose,'marker_registration',return_value=dict(available=True,homography_image_to_board_mm=np.eye(3).tolist())),patch.object(video_marker_pose,'colour_components',return_value=[]),patch.object(video_marker_pose,'select_marker_pair',return_value=(centre+25*forward,centre-25*forward)):
            return preflight.inspect_frame(None,{},geometry,self.plan['expected_candidate_parameters'],self.plan)

    def test_diagonal_placement_and_wrong_heading_or_start(self):
        self.assertTrue(self.inspect_pose([90,45],[2**-.5,2**-.5])['ready'])
        self.assertFalse(self.inspect_pose([90,45],[0,1])['ready'])
        self.assertFalse(self.inspect_pose([225,45],[2**-.5,2**-.5])['ready'])
        self.assertFalse(self.inspect_pose([90,310],[2**-.5,2**-.5])['ready'])

if __name__=='__main__':unittest.main()
