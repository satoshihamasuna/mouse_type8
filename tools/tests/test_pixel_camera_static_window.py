"""静止窓外の欠落だけを分離し、境界・転送・復号の欠陥を通さないことを確認する。"""
from pathlib import Path
from hashlib import sha256
import json
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'analysis'))
import pixel_camera_static_window as static


def frames_without(indices=()):
    return [dict(pts_s=index/240, corrupt=False, width=1280, height=720)
            for index in range(1201) if index not in indices]


class StaticWindowTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.folder = Path(self.temp.name)
        self.video = self.folder / 'static.mp4'
        self.video.write_bytes(b'transferred fixture; decoding is supplied independently')
        self.manifest_path = self.folder / 'capture.json'
        self.frames = frames_without(range(392, 400))
        digest = sha256(self.video.read_bytes()).hexdigest()
        self.manifest = dict(state='FAILED', requested_seconds=5, label='prepare_trial_static',
            capture_profile='tuning', type8_log_csv=None, video_file=str(self.video),
            sha256=digest, size_bytes=self.video.stat().st_size,
            video_inspection=dict(errors=['DECODE_OR_PTS_ERROR'], passed=False,
                file_path=str(self.video), sha256=digest, sha256_rechecked=True,
                size_bytes=self.video.stat().st_size, decoded_frames=len(self.frames),
                corrupt_frames=0, missing_pts=0, non_increasing_pts=0, wrong_size_frames=0,
                gaps_over_1_5_frames=1))

    def inspect(self, *, frames=None, effect=None):
        self.manifest_path.write_text(json.dumps(self.manifest), encoding='utf-8')
        original = self.manifest_path.read_bytes()
        with patch.object(static, 'decode_video_frames', side_effect=effect,
                          return_value=self.frames if frames is None else frames):
            result = static.inspect_static_window(self.manifest_path)
        self.assertEqual(self.manifest_path.read_bytes(), original)
        self.assertFalse(result['motion_acceptance_replaced'])
        self.assertFalse(result['pose_acceptance_replaced'])
        return result

    def test_startup_gap_does_not_reclassify_failed_whole_video(self):
        result = self.inspect()
        self.assertTrue(result['passed'])
        self.assertEqual(result['source_manifest_state'], 'FAILED')
        self.assertFalse(result['original_manifest_reclassified'])
        self.assertEqual(result['window_s'], [3.3, 4.95])
        self.assertEqual(len(result['decoded_timing']['gaps']), 1)
        self.assertFalse(result['decoded_timing']['gaps'][0]['intersects_static_window'])

    def test_gap_inside_window_is_rejected(self):
        result = self.inspect(frames=frames_without(range(900, 908)))
        self.assertFalse(result['passed'])
        self.assertIn('pts_gap_inside_or_crossing_static_window', result['failure_reasons'])

    def test_gap_crossing_either_window_boundary_is_rejected(self):
        for indices in (range(790, 795), range(1186, 1191)):
            with self.subTest(indices=indices):
                result = self.inspect(frames=frames_without(indices))
                self.assertFalse(result['passed'])
                self.assertIn('pts_gap_inside_or_crossing_static_window', result['failure_reasons'])

    def test_clip_must_cover_both_window_endpoints(self):
        full = frames_without()
        for frames in ([frame for frame in full if frame['pts_s'] > 3.3],
                       [frame for frame in full if frame['pts_s'] < 4.95]):
            result = self.inspect(frames=frames)
            self.assertFalse(result['passed'])
            self.assertIn('static_window_not_fully_covered', result['failure_reasons'])

    def test_invalid_frames_outside_window_are_never_ignored(self):
        defects = [('pts_s', None, 'missing_or_nonfinite_pts'),
                   ('pts_s', float('nan'), 'missing_or_nonfinite_pts'),
                   ('pts_s', -.1, 'non_increasing_pts'),
                   ('corrupt', True, 'corrupt_frames'),
                   ('width', 640, 'wrong_size_frames')]
        for field, value, failure in defects:
            with self.subTest(field=field, value=value):
                frames = [dict(frame) for frame in self.frames]
                frames[20][field] = value
                result = self.inspect(frames=frames)
                self.assertFalse(result['passed'])
                self.assertIn(failure, result['failure_reasons'])

    def test_existing_whole_decode_defects_and_unverified_transfer_are_rejected(self):
        for name in ('corrupt_frames', 'missing_pts', 'non_increasing_pts', 'wrong_size_frames'):
            with self.subTest(name=name):
                self.manifest['video_inspection'][name] = 1
                result = self.inspect()
                self.assertFalse(result['passed'])
                self.assertIn('whole_capture_' + name, result['failure_reasons'])
                self.manifest['video_inspection'][name] = 0
        self.manifest['video_inspection']['sha256_rechecked'] = False
        self.assertIn('transfer_sha_not_verified', self.inspect()['failure_reasons'])

    def test_changed_transferred_bytes_are_rejected(self):
        self.video.write_bytes(b'replacement video')
        result = self.inspect()
        self.assertFalse(result['passed'])
        self.assertIn('video_sha_mismatch', result['failure_reasons'])

    def test_other_whole_capture_errors_are_rejected(self):
        self.manifest['video_inspection']['errors'].append('TRANSFER_ERROR')
        result = self.inspect()
        self.assertFalse(result['passed'])
        self.assertIn('whole_capture_error_not_allowed', result['failure_reasons'])

    def test_manifest_level_error_is_not_masked_by_pts_error(self):
        self.manifest['error'] = 'transfer disconnected'
        result = self.inspect()
        self.assertFalse(result['passed'])
        self.assertIn('whole_manifest_error_not_allowed', result['failure_reasons'])

    def test_dynamic_and_nonfixed_duration_captures_cannot_use_static_exception(self):
        for key, value in (('label', 'motion_trial'), ('requested_seconds', 20), ('type8_log_csv', 'motion.csv')):
            with self.subTest(key=key):
                previous = self.manifest[key]
                self.manifest[key] = value
                result = self.inspect()
                self.assertFalse(result['passed'])
                self.assertIn('not_a_five_second_tuning_static_capture', result['failure_reasons'])
                self.manifest[key] = previous

    def test_decode_error_and_frame_count_mismatch_are_rejected(self):
        result = self.inspect(effect=RuntimeError('corrupt packet'))
        self.assertFalse(result['passed'])
        self.assertIn('video_read_or_decode_error', result['failure_reasons'])
        self.manifest['video_inspection']['decoded_frames'] += 1
        self.assertIn('decoded_frame_count_mismatch', self.inspect()['failure_reasons'])

    def test_file_change_during_decode_is_rejected(self):
        def decode_and_replace(path):
            self.video.write_bytes(b'changed during decode')
            return self.frames
        result = self.inspect(effect=decode_and_replace)
        self.assertFalse(result['passed'])
        self.assertIn('video_changed_during_inspection', result['failure_reasons'])

    def test_exactly_allowed_interval_is_not_rejected_by_float_rounding(self):
        frames = frames_without()
        frames[850]['pts_s'] = frames[849]['pts_s'] + static.MAX_INTERVAL_S
        result = static.assess_decoded_pts(frames)
        self.assertTrue(result['passed'])


if __name__ == '__main__':
    unittest.main()
