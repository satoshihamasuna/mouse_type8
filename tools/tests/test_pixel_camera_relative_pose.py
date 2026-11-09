"""相対解析の動作全区間チェックと、キャッシュの入力対応を確認する。"""
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'analysis'))
from pixel_camera_relative_pose import required_motion_quality, load_pose_cache, file_hash, reference_digest


class RelativePoseTests(unittest.TestCase):
    def test_idle_losses_are_reported_without_discarding_required_motion(self):
        time = np.arange(0, 3.001, .005)
        valid = ~((time > 2.3) & (time < 2.6))
        trial = dict(log_quality=dict(motion_start_time_ms=2000, motion_end_time_ms=3000))
        quality = required_motion_quality(time, valid, valid, [1000], trial, 0)
        self.assertTrue(quality['accepted_for_relative_review'])
        self.assertEqual(quality['observation_rate'], 1)
        self.assertLess(valid.mean(), .95)

    def test_losses_inside_motion_and_missing_stop_margin_do_not_pass(self):
        time = np.arange(0, 3.001, .005)
        valid = ~((time > 1.3) & (time < 1.6))
        trial = dict(log_quality=dict(motion_start_time_ms=2000, motion_end_time_ms=3000))
        self.assertFalse(required_motion_quality(time, valid, valid, [1000], trial, 0)['accepted_for_relative_review'])
        short = time[time <= 2.05]
        ones = np.ones(len(short), bool)
        self.assertFalse(required_motion_quality(short, ones, ones, [1000], trial, 0)['accepted_for_relative_review'])

    def test_motion_boundaries_handle_mcu_tick_wrap(self):
        time = np.arange(0, 2.001, .005)
        valid = np.ones(len(time), bool)
        trial = dict(log_quality=dict(motion_start_time_ms=250, motion_end_time_ms=750))
        quality = required_motion_quality(time, valid, valid, [2**32-500], trial, 0)
        self.assertAlmostEqual(quality['video_motion_start_s'], .75)
        self.assertAlmostEqual(quality['video_motion_end_s'], 1.25)
        self.assertTrue(quality['accepted_for_relative_review'])

    def test_cache_requires_matching_video_and_detector_and_window(self):
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            video, cache = folder/'video.mp4', folder/'pose.csv'
            video.write_bytes(b'video identity used for provenance test')
            pd.DataFrame(dict(time_video_s=[0, .1, .2], marker_detected_raw=[True]*3,
                              calibration_valid=[True]*3)).to_csv(cache, index=False)
            reference, geometry = dict(reference_id='known'), dict(front_colour='red', marker_detection={})
            meta = dict(video_sha256=file_hash(video), cached_pose_sha256=file_hash(cache),
                reference_digest=reference_digest(reference), front_colour='red', marker_detection={},
                video_window=[0, .2], marker_calibration={})
            cache.with_suffix('.provenance.json').write_text(json.dumps(meta), encoding='utf-8')
            trial = dict(video_file=str(video))
            self.assertEqual(len(load_pose_cache(cache, trial, reference, geometry, (0, .2))), 3)
            with self.assertRaises(ValueError):
                load_pose_cache(cache, trial, reference, geometry, (0, .2), frame_stride=4)
            for ref, geo, window in ((dict(reference_id='other'), geometry, (0, .2)),
                                     (reference, dict(front_colour='blue', marker_detection={}), (0, .2)),
                                     (reference, geometry, (0, .3))):
                with self.assertRaises(ValueError):
                    load_pose_cache(cache, trial, ref, geo, window)
            video.write_bytes(b'changed video')
            with self.assertRaises(ValueError):
                load_pose_cache(cache, trial, reference, geometry, (0, .2))

    def test_sampled_quality_is_explicit_and_does_not_change_full_frame_requirements(self):
        time=np.arange(0,3,.0166666667)
        valid=np.ones(len(time),bool)
        trial=dict(log_quality=dict(motion_start_time_ms=2000,motion_end_time_ms=3000))
        quick=required_motion_quality(time,valid,valid,[1000],trial,0,frame_stride=4)
        full=required_motion_quality(time,valid,valid,[1000],trial,0)
        self.assertTrue(quick['accepted_for_relative_review'])
        self.assertEqual(quick['frame_stride'],4)
        self.assertEqual(full['minimum_stationary_observations'],10)
        self.assertFalse(full['accepted_for_relative_review'])


if __name__ == '__main__':
    unittest.main()
