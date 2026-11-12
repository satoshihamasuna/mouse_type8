"""機器へ接続せず、試行一式の保存・元資料保持・動画共有・GUI連携を検証する。"""
import csv
import hashlib
import json
import os
from pathlib import Path
import queue
import sys
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import pixel_camera_archive as archive
from pixel_camera_tuning_panel import PixelTuningPanel


class ArchiveTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        self.source = self.base/'repo/tools/logs/session/run_one'
        self.source.mkdir(parents=True)
        self.remote = self.base/'recordings/remote'
        self.remote.mkdir(parents=True)
        self.root = self.base/'trials'
        self.video = self.remote/'recording_one.mp4'
        self.video.write_bytes(b'synthetic video bytes for integrity checks')
        self.metadata = self.remote/'recording_one.json'
        self.metadata.write_text('{"sensor_csv":"recording_one.sensor.csv"}', encoding='utf-8')
        (self.remote/'recording_one.sensor.csv').write_text('frame,time\n1,0\n', encoding='utf-8')
        self.capture = self.base/'capture.json'
        self.write(self.capture, dict(recording_id='recording_one',sha256=archive.file_hash(self.video),
                                      metadata_file=str(self.metadata)))
        self.csv = self.source/'motion.csv'
        self.csv.write_text('time_ms,ideal.velo\n0,0.5\n5,0\n', encoding='utf-8')
        self.settings = self.source/'motion.settings.json'
        self.write(self.settings, dict(mouse='A',parameters=dict(r_min=44.7,lstart=23.63,lend=17.1)))
        self.trial = self.source/'trial.json'
        self.write(self.trial, dict(mouse='A',motion='long_l180',preset=500,recording_id='recording_one',
            video_file=str(self.video),type8_log_csv=str(self.csv),type8_settings_json=str(self.settings),
            capture_json=str(self.capture),camera_state='PASSED'))
        (self.source/'full').mkdir()
        self.report = self.source/'full/report.json'
        self.write(self.report, dict(trial_json=str(self.trial),input_sha256={str(self.trial):archive.file_hash(self.trial)}))
        self.write(self.source/'full/summary.json', dict(auto_stop_confirmed=True,endpoint_right_mm=-90.2,endpoint_forward_mm=-.8))

    def write(self, path, value):
        path.write_text(json.dumps(value),encoding='utf-8')

    def export(self):
        return archive.archive_trial(self.trial,self.root)

    def test_bundle_is_self_contained_and_original_evidence_remains_unchanged(self):
        originals={path:path.read_bytes() for path in self.source.rglob('*') if path.is_file()}
        originals[self.capture]=self.capture.read_bytes()
        folder=self.export()
        self.assertTrue(os.path.samefile(self.video,folder/'video.mp4'))
        self.assertEqual((folder/'motion.csv').read_bytes(),self.csv.read_bytes())
        self.assertEqual((folder/'motion.settings.json').read_bytes(),self.settings.read_bytes())
        self.assertEqual((folder/'source_trial.json').read_bytes(),self.trial.read_bytes())
        self.assertEqual((folder/'full/report.json').read_bytes(),self.report.read_bytes())
        self.assertTrue((folder/'camera/recording_one.sensor.csv').exists())
        for path,content in originals.items():self.assertEqual(path.read_bytes(),content)
        derived=archive.read_json(folder/'trial.json')
        for key in ('video_file','type8_log_csv','type8_settings_json','capture_json','full_report'):
            self.assertTrue(Path(derived[key]).is_file(),key)
            self.assertTrue(Path(derived[key]).is_relative_to(folder),key)
        manifest=archive.verify_archive(folder)
        self.assertEqual(manifest['files']['video.mp4']['storage'],'hardlink')
        self.assertFalse(manifest['robot_commands_sent'])
        self.assertEqual(archive.archive_for_trial(self.trial),folder)

    def test_repeated_export_verifies_existing_files_without_replacing_them(self):
        folder=self.export()
        first=(folder/'manifest.json').read_bytes()
        self.assertEqual(self.export(),folder)
        self.assertEqual((folder/'manifest.json').read_bytes(),first)
        self.assertFalse(list(self.root.glob('*.partial')))

    def test_tampered_archive_is_not_silently_repaired_or_overwritten(self):
        folder=self.export()
        (folder/'motion.csv').write_text('modified',encoding='utf-8')
        with self.assertRaisesRegex(ValueError,'照合'):self.export()
        self.assertEqual((folder/'motion.csv').read_text(),'modified')
        self.assertTrue(self.csv.read_text().startswith('time_ms'))

    def test_changed_source_for_same_recording_is_rejected_without_overwrite(self):
        folder=self.export()
        original=(folder/'motion.csv').read_bytes()
        self.csv.write_text('new data',encoding='utf-8')
        with self.assertRaisesRegex(ValueError,'異なる入力'):self.export()
        self.assertEqual((folder/'motion.csv').read_bytes(),original)

    def test_video_hash_failure_happens_before_creating_an_archive(self):
        self.video.write_bytes(b'wrong video')
        with self.assertRaisesRegex(ValueError,'SHA-256'):self.export()
        self.assertFalse(self.root.exists())

    def test_changed_analysis_is_not_silently_ignored_on_reexport(self):
        folder=self.export()
        before=(folder/'full/report.json').read_bytes()
        self.report.write_text('{"new_result":true}')
        with self.assertRaisesRegex(ValueError,'元資料'):self.export()
        self.assertEqual((folder/'full/report.json').read_bytes(),before)

    def test_new_source_file_requires_a_separate_snapshot(self):
        folder=self.export()
        (self.source/'new_measurement.csv').write_text('new evidence')
        with self.assertRaisesRegex(ValueError,'ファイル構成'):self.export()
        self.assertFalse((folder/'new_measurement.csv').exists())

    def test_link_failure_keeps_originals_and_never_falls_back_to_a_video_copy(self):
        before=self.trial.read_bytes()
        with patch.object(archive.os,'link',side_effect=OSError('different filesystem')):
            with self.assertRaises(OSError):self.export()
        self.assertEqual(self.trial.read_bytes(),before)
        self.assertTrue(self.video.exists())
        self.assertFalse((self.root/'recording_one').exists())
        self.assertFalse(list(self.root.rglob('video.mp4')))
        self.assertTrue(list(self.root.glob('*.partial')))

    def test_recording_id_cannot_escape_archive_root(self):
        trial=archive.read_json(self.trial);trial['recording_id']='../outside'
        self.write(self.trial,trial)
        with self.assertRaisesRegex(ValueError,'録画ID'):self.export()
        self.assertFalse(self.root.exists())

    def test_root_inside_source_is_rejected_before_recursive_copy(self):
        with self.assertRaisesRegex(ValueError,'再帰'):
            archive.archive_trial(self.trial,self.source/'nested')
        self.assertFalse((self.source/'nested').exists())

    def test_existing_lock_is_preserved_and_prevents_a_second_writer(self):
        self.root.mkdir()
        lock=self.root/'.recording_one.archive.lock'
        lock.write_text('other writer',encoding='ascii')
        with self.assertRaises(FileExistsError):self.export()
        self.assertEqual(lock.read_text(),'other writer')

    def test_original_readme_is_preserved_beside_generated_instructions(self):
        (self.source/'README.md').write_text('original notes',encoding='utf-8')
        folder=self.export()
        self.assertEqual((folder/'source_README.md').read_text(),'original notes')
        self.assertIn('video.mp4',(folder/'README.md').read_text(encoding='utf-8'))

    def test_default_root_and_environment_override(self):
        with patch.dict(os.environ,{},clear=True):
            self.assertEqual(archive.default_root(self.video),self.base/'trials')
        with patch.dict(os.environ,{'TYPE8_TRIALS_DIR':str(self.base/'custom')}):
            self.assertEqual(archive.default_root(self.video),self.base/'custom')

    def test_index_contains_parameters_quality_and_relative_video_csv_paths(self):
        folder=self.export()
        archive.write_index(self.root)
        with (self.root/'index.csv').open(encoding='utf-8-sig',newline='') as stream:
            rows=list(csv.DictReader(stream))
        self.assertEqual(len(rows),1)
        self.assertEqual(rows[0]['lend'],'17.1')
        self.assertEqual(rows[0]['detailed_stop_confirmed'],'True')
        self.assertEqual(self.root/rows[0]['video'],folder/'video.mp4')
        self.assertEqual(self.root/rows[0]['csv'],folder/'motion.csv')


class GuiArchiveTests(unittest.TestCase):
    def panel(self):
        return SimpleNamespace(analysis_lock=threading.Lock(),gui=SimpleNamespace(events=queue.Queue()),
                               analyze=Mock(return_value=0))

    def test_full_analysis_archives_after_completion_without_any_capture(self):
        panel=self.panel()
        with tempfile.TemporaryDirectory() as name:
            folder=Path(name);(folder/'full').mkdir()
            (folder/'full/summary.json').write_text('{"auto_stop_confirmed":true}')
            def export(path):
                panel.analyze.assert_called_once_with(folder,1)
                self.assertEqual(path,folder/'trial.json')
                return folder/'archive'
            with patch('pixel_camera_tuning_panel.archive_trial',side_effect=export) as saved,patch('pixel_camera_tuning_panel.write_index'):
                PixelTuningPanel._full(panel,folder)
            saved.assert_called_once()
            events=list(panel.gui.events.queue)
            self.assertEqual(events[-2][0:3],('pixel_full',str(folder),'要目視確認'))
            self.assertEqual(events[-1],('pixel_archive',str(folder),str(folder/'archive'),None))

    def test_failed_analysis_still_archives_evidence_and_reports_archive_failure_separately(self):
        panel=self.panel();panel.analyze.side_effect=RuntimeError('analysis failed')
        with patch('pixel_camera_tuning_panel.archive_trial',side_effect=OSError('disk unavailable')):
            PixelTuningPanel._full(panel,Path('trial'))
        events=list(panel.gui.events.queue)
        self.assertEqual(events[-2],('pixel_full','trial','解析失敗'))
        self.assertEqual(events[-1],('pixel_archive','trial',None,'disk unavailable'))

    def test_result_button_opens_verified_archive_pointer_instead_of_source(self):
        with tempfile.TemporaryDirectory() as name:
            folder=Path(name);destination=folder/'archive';destination.mkdir()
            (folder/'archive.json').write_text(json.dumps(dict(archive_folder=str(destination))))
            with patch('pixel_camera_tuning_panel.os.startfile',create=True) as opened:
                PixelTuningPanel.open_result(SimpleNamespace(latest_folder=folder))
            opened.assert_called_once_with(str(destination))

    def test_saved_left_turn_review_selects_left_motion_without_device_requests(self):
        with tempfile.TemporaryDirectory() as name:
            folder=Path(name)
            settings=folder/'motion.settings.json'
            settings.write_text(json.dumps(dict(raw_response='saved left parameters',parameters=dict(lend=17.1))))
            trial=folder/'trial.json'
            trial.write_text(json.dumps(dict(mouse='A',motion='long_l180',camera_height_mm=800,type8_settings_json=str(settings))))
            gui=SimpleNamespace(motion=Mock(),turn_type=Mock(),_queue_turn_params=Mock(),_load_defaults=Mock())
            table=Mock();table.insert.return_value='row'
            panel=SimpleNamespace(gui=gui,mouse=Mock(),height=Mock(),rows={},table=table,select_result=Mock(),status=Mock())
            PixelTuningPanel.show_saved(panel,trial)
            gui.motion.set.assert_called_once_with('turn')
            gui.turn_type.set.assert_called_once_with('long_l180')
            gui._load_defaults.assert_not_called()
            gui._queue_turn_params.assert_called_once_with('saved left parameters')


if __name__=='__main__':
    unittest.main()
