"""用途別撮影条件の分離と、録画前の全項目照合を機器なしで検証する。"""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import pixel_camera_profiles as profiles


class CameraProfileTests(unittest.TestCase):
    def test_saving_and_switching_profiles_restores_each_complete_configuration(self):
        with tempfile.TemporaryDirectory() as folder:
            tuning = dict(ev=-2.5, torch=False, focus_lock=True, zoom_ratio=1.4)
            maze = dict(ev=-2., torch=True, focus_lock=False, zoom_ratio=1.)
            profiles.save_camera_profile('tuning', tuning, folder)
            profiles.save_camera_profile('maze_9x9', maze, folder)
            client = Mock()
            client.get_capabilities.return_value = dict(cameras=[dict(target_fixed_240_supported=True,
                back_facing=True, ev_step='1/6', ev_steps_range='[-24,24]')])
            client.configure.side_effect = lambda **kwargs: kwargs
            for name, expected, steps in (('tuning', tuning, -15), ('maze_9x9', maze, -12), ('tuning', tuning, -15)):
                settings = profiles.load_camera_profile(name, folder)
                self.assertEqual(settings, expected)
                evidence = profiles.configure_camera(client, settings)
                self.assertEqual(evidence['response'], dict(torch=expected['torch'],
                    focus_lock=expected['focus_lock'], zoom_ratio=expected['zoom_ratio'], exposure_compensation_steps=steps))
            self.assertEqual(client.configure.call_count, 3)

    def test_legacy_maze_preferences_do_not_change_tuning_or_overwrite_measurement_evidence(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'explore_capture_preferences.json'
            original = json.dumps(dict(schema=1, camera_settings=dict(ev=-1.5, torch=True), source_report='measured.json'))
            path.write_text(original, encoding='utf-8')
            maze = profiles.load_camera_profile('maze_9x9', folder)
            self.assertEqual(maze['ev'], -1.5)
            self.assertEqual(profiles.load_camera_profile('tuning', folder), profiles.DEFAULTS['tuning'])
            profiles.save_camera_profile('maze_9x9', maze, folder)
            self.assertEqual(path.read_text(encoding='utf-8'), original)

    def test_mismatched_profile_and_invalid_values_are_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            profiles.save_camera_profile('tuning', profiles.DEFAULTS['tuning'], folder)
            path = profiles.profile_path('tuning', folder)
            value = json.loads(path.read_text(encoding='utf-8'))
            value['capture_profile'] = 'maze_9x9'
            path.write_text(json.dumps(value), encoding='utf-8')
            with self.assertRaises(ValueError):
                profiles.load_camera_profile('tuning', folder)
        for settings in ({'zoom_ratio':float('nan')}, {'zoom_ratio':.5}, {'focus_lock':'ON'},
                         {'torch':1}, {'ev':True}, {'ev':-5}, {'exposure_compensation_steps':1.5}):
            with self.subTest(settings=settings), self.assertRaises(ValueError):
                profiles.configure_camera(Mock(), settings)

    def test_missing_or_changed_zoom_and_focus_receipts_are_rejected(self):
        for response in ({}, {'zoom_ratio':1.}, {'zoom_ratio':1.4, 'focus_lock':False}):
            client = Mock()
            client.configure.return_value = response
            with self.subTest(response=response), self.assertRaisesRegex(RuntimeError, '一致'):
                profiles.configure_camera(client, dict(zoom_ratio=1.4, focus_lock=True))


if __name__ == '__main__':
    unittest.main()
