"""準備処理の失敗時に、後続の実機処理へ進まないことを機器なしで検査する。"""
from copy import deepcopy
from datetime import datetime, timedelta, timezone
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import prepare_inR135_fast as fast

BASE = Path(__file__).resolve().parent


class PreparationGates(unittest.TestCase):
    def setUp(self):
        self.source = fast.load(fast.load(BASE/'next_session_B_inR135_r32p3_Lstart44_retry03_v5.json')['settings_source'])
        self.plan = fast.load(BASE/'B_inR135_r32p3_Lstart44_retry03_plan.json')

    def test_changed_B_feedforward_rejected(self):
        actual = deepcopy(self.source)
        actual['parameters'] = self.plan['expected_compiled_B']
        actual['pid'] = self.plan['expected_B_pid']
        actual['feedforward']['ff_om_velo'] += .001
        with self.assertRaises(AssertionError):
            fast.check_baseline(actual, self.source, self.plan)

    def test_incorrect_restored_candidate_rejected(self):
        actual = deepcopy(self.source)
        actual['parameters']['lstart'] += .5
        with self.assertRaises(AssertionError):
            fast.check_candidate(actual, self.source)

    def test_stale_ready_stops_before_connection(self):
        with tempfile.TemporaryDirectory() as directory:
            session = Path(directory)
            meta = dict(session=str(session), operator_ready='USER_CONFIRMED_POSITION_HANDS_OFF_BOTH_ON', capture_request_id='test')
            fast.save(session/'operator_ready.json', dict(
                placement_stopped_clearance_hands_off_both_on_confirmed=True,
                capture_request_id='test', recorded_at=(datetime.now(timezone.utc)-timedelta(minutes=6)).isoformat()))
            fast.save(session/'meta.json', meta)
            with patch.object(fast, 'validate_files', return_value=(self.source, self.plan)), patch.object(fast.subprocess, 'Popen') as start:
                with self.assertRaises(AssertionError):
                    fast.prepare(session/'meta.json')
                start.assert_not_called()

    def test_failed_preset_never_applies_or_records(self):
        with tempfile.TemporaryDirectory() as directory:
            session = Path(directory)
            meta = dict(session=str(session), operator_ready='USER_CONFIRMED_POSITION_HANDS_OFF_BOTH_ON',
                capture_request_id='test', config='unused.json', preset_readback=str(session/'preset.json'))
            fast.save(session/'operator_ready.json', dict(
                placement_stopped_clearance_hands_off_both_on_confirmed=True,
                capture_request_id='test', recorded_at=datetime.now(timezone.utc).isoformat()))
            fast.save(session/'meta.json', meta)

            def connected(*args, **kwargs):
                fast.save(session/'service.json', {})
                return Mock(pid=123, poll=Mock(return_value=None))

            failure = Mock(returncode=1, stdout='', stderr='simulated preset failure')
            with patch.object(fast, 'validate_files', return_value=(self.source, self.plan)), patch.object(fast.subprocess, 'Popen', side_effect=connected), patch.object(fast.subprocess, 'run', return_value=failure) as invoke:
                with self.assertRaises(AssertionError):
                    fast.prepare(session/'meta.json')
                self.assertEqual(invoke.call_count, 1)
                command = invoke.call_args.args[0]
                self.assertIn('preset', command)
                self.assertNotIn('capture', command)
                self.assertNotIn('apply', command)


if __name__ == '__main__':
    unittest.main()
