"""配置・設定送信・単体走行・自動解析・次試行の順序を実機なしで検証する。"""
from copy import deepcopy
import json
import os
from pathlib import Path
import sys
import tempfile
import tkinter as tk
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from test_pixel_camera_tuning import settings
import pixel_camera_tuning as tuning
import pixel_camera_tuning_panel as panel_module
from turn_tuning_video_setup import body_geometry


class SettingsOnlyTests(unittest.TestCase):
    def test_apply_reads_back_once_and_never_sends_motion_or_permit(self):
        sent, applied = [], [False]
        before, after = settings(20), settings(19)
        class Shell:
            def __init__(self, *args, **kwargs): pass
            def send(self, command):
                sent.append(command)
                if ' set ' in command: applied[0] = True
            def check_battery(self): pass
            def query(self, command, predicate):
                sent.append(command)
                return 'after' if applied[0] else 'before'
            def wait_line(self, predicate, timeout):
                assert predicate('DEBUG_TURN_SET_DONE')
        session = SimpleNamespace(alive=lambda: True, connection=SimpleNamespace(is_open=True))
        with tempfile.TemporaryDirectory() as directory, patch.object(tuning.time, 'sleep'):
            folder = Path(directory)/'settings'
            result = tuning.apply_connected_settings(session, after, tuning.settings_set_command(after), 0,
                folder, lambda line, *args: deepcopy(after if line == 'after' else before), shell_type=Shell)
            self.assertEqual(result['parameters']['lend'], 19)
            self.assertEqual(sum(' set ' in command for command in sent), 1)
            self.assertFalse(any('exe' in command or ' arm ' in command or 'preset' in command for command in sent))
            self.assertFalse(json.loads((folder/'result.json').read_text())['execution_write_attempted'])
            self.assertTrue((folder/'requested.settings.json').exists())
            self.assertTrue((folder/'telemetry.settings.json').exists())

    def test_settings_readback_failure_does_not_execute_or_retry(self):
        shell = Mock()
        shell.query.return_value = 'before'
        shell.wait_line.side_effect = TimeoutError('ambiguous settings write')
        with tempfile.TemporaryDirectory() as directory, self.assertRaises(TimeoutError):
            tuning.verify_and_apply_settings(shell, settings(19), tuning.settings_set_command(settings(19)),
                lambda *args: settings(20), Mock(), directory)
        shell.send.assert_called_once()
        self.assertIn(' set ', shell.send.call_args.args[0])


class WorkflowGuiTests(unittest.TestCase):
    def setUp(self):
        from myshell_debug import DebugGui
        from turn_tuning import configure_tuner
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.folder = Path(self.temp.name)
        self.gui = DebugGui(); self.gui.withdraw(); configure_tuner(self.gui)
        self.addCleanup(self.gui.destroy)
        self.panel = self.gui.camera_panel
        self.panel.record_video.set(True)
        self.panel.analysis_mode.set('位置だけ（盤面XY）')
        self.panel.mouse.set('A')
        value = settings()
        for name, key in dict(distance='velo', acc='r_min', max_velo='lstart', end_velo='lend',
                              degree='degree', degree_correction='correction').items():
            self.gui.values[name].set(value['parameters'][key])
        for group in ('pid', 'feedforward'):
            for name, number in value[group].items(): self.gui.values[name].set(number)
        self.gui.pre_accel.set(6.5)
        self.gui.suction_enable.set(False); self.gui.suction_duty.set(650)
        self.gui.turn_count.set(1)
        self.panel.preferences = self.folder/'tuning_ui.json'
        self.reference = self.folder/'reference.json'
        self.reference.write_text('{"reference_id":"test"}', encoding='utf-8')
        self.geometry = self.folder/'geometry.json'
        self.geometry.write_text(json.dumps(body_geometry('A', dict(reference_id='test'), 'red','yellow',10,8,800)), encoding='utf-8')
        self.panel.reference.set(str(self.reference)); self.panel.geometry.set(str(self.geometry))
        self.panel.height.set(800); self.panel.ready.set(True)
        self.connection = SimpleNamespace(is_open=True)
        self.session = SimpleNamespace(folder=self.folder, connection=self.connection, alive=lambda:True,
            prepare_camera=Mock(), close=Mock(side_effect=lambda:setattr(self.connection,'is_open',False)))
        self.panel.session = self.session; self.gui.ser = self.connection
        self.gui._connection_generation = 7
        self.panel.set_workflow('settings')

    def launch_inline(self, operation, callback):
        result = operation()
        while not self.gui.events.empty():
            item = self.gui.events.get_nowait()
            if isinstance(item, tuple) and item[0].startswith('pixel_'): self.panel.handle(item)
        callback(result)
        return True

    def test_full_loop_disconnects_before_analysis_and_requires_a_new_manual_start(self):
        calls = []
        self.session.close.side_effect = lambda:(calls.append('disconnect'), setattr(self.connection,'is_open',False))
        def apply(*args, **kwargs): calls.append('settings'); return settings()
        def capture(session, request, folder, *args):
            calls.append('capture'); folder.mkdir()
            result = dict(state='CAPTURED_REVIEW_REQUIRED',record_video=True,saved_log=dict(status='saved',period_ms=1))
            (folder/'trial.json').write_text(json.dumps(result),encoding='utf-8')
            return result
        def analyze(folder, stride):
            calls.append('analysis'+str(stride))
            directory=folder/('quick' if stride==4 else 'full');directory.mkdir()
            (directory/'summary.json').write_text(json.dumps(dict(auto_stop_confirmed=True,endpoint_forward_mm=1,endpoint_right_mm=2)))
            return 0
        with patch.object(self.panel, '_launch', side_effect=self.launch_inline), \
                patch.object(panel_module, 'apply_connected_settings', side_effect=apply), \
                patch.object(panel_module, 'capture_connected', side_effect=capture) as captured, \
                patch.object(panel_module, 'render_trial_log', side_effect=lambda folder:calls.append('log')), \
                patch.object(self.panel, 'analyze', side_effect=analyze), \
                patch.object(panel_module, 'archive_trial', side_effect=lambda path:(calls.append('archive') or path.parent/'archive')), \
                patch.object(panel_module, 'write_index'):
            self.panel.apply_settings()
            self.assertEqual(self.panel.workflow_phase, 'ready')
            self.assertTrue(self.panel.ready.get())
            self.panel.run()
            run_folder = Path(captured.call_args.args[2])
            self.assertEqual(json.loads((run_folder/'analysis_plan.json').read_text())['mode'], 'position_only')
            self.assertEqual(calls, ['settings','capture','disconnect','log','analysis4','analysis1','archive'])
            self.assertEqual(self.panel.workflow_phase, 'review')
            self.assertFalse(self.panel.ready.get())
            self.assertIsNone(self.gui.ser)
            self.panel.next_trial()
            self.assertEqual(self.panel.workflow_phase, 'placement')
            self.assertIsNone(self.panel.verified_settings)
            self.assertFalse(self.panel.ready.get())
            self.assertEqual(self.gui.values['end_velo'].get(), settings()['parameters']['lend'])
            captured.assert_called_once()

    def test_run_is_blocked_until_settings_are_sent_and_after_edits(self):
        self.panel.record_video.set(False)
        with patch.object(self.panel, '_launch') as launch:
            self.panel.run()
        launch.assert_not_called()
        self.panel.verified_settings=settings(); self.panel.verified_generation=7
        self.panel.set_workflow('ready')
        self.gui.values['end_velo'].set(18)
        with patch.object(self.panel, '_launch') as launch:
            self.panel.run()
        launch.assert_not_called()
        self.assertIn('設定送信', self.panel.status.get())

    def test_record_only_saves_logs_and_archives_without_any_video_analysis(self):
        self.panel.analysis_mode.set('録画のみ（解析なし）')
        self.panel.geometry.set(str(self.folder/'missing-geometry.json'))
        self.panel.reference.set(str(self.folder/'missing-reference.json'))
        self.panel.height.set('')
        calls=[]
        self.session.close.side_effect=lambda:(calls.append('disconnect'),setattr(self.connection,'is_open',False))
        def capture(session,request,folder,*args):
            request.validate()
            self.assertFalse(request.video_analysis)
            self.assertTrue(request.record_video)
            calls.append('capture'); folder.mkdir()
            result=dict(state='CAPTURED_REVIEW_REQUIRED',record_video=True,
                saved_log=dict(status='saved',period_ms=1),log_stop_evidence=dict(confirmed=True))
            (folder/'trial.json').write_text(json.dumps(result),encoding='utf-8')
            return result
        def render(folder):
            calls.append('log')
            (folder/'log.png').write_bytes(b'log-image-fixture')
            result=json.loads((folder/'trial.json').read_text())
            result['log_image_period_ms']=1
            (folder/'trial.json').write_text(json.dumps(result))
        with patch.object(self.panel,'_launch',side_effect=self.launch_inline), \
                patch.object(panel_module,'apply_connected_settings',return_value=settings()), \
                patch.object(panel_module,'capture_connected',side_effect=capture) as captured, \
                patch.object(panel_module,'render_trial_log',side_effect=render), \
                patch.object(self.panel,'analyze') as analyze, patch.object(self.panel,'_full') as full, \
                patch.object(panel_module,'archive_trial',side_effect=lambda path:(calls.append('archive') or path.parent/'archive')), \
                patch.object(panel_module,'write_index'):
            self.panel.apply_settings()
            self.assertEqual(self.panel.workflow_phase,'ready')
            self.panel.run()
        captured.assert_called_once(); analyze.assert_not_called(); full.assert_not_called()
        self.assertEqual(calls,['capture','disconnect','log','archive'])
        self.assertFalse(self.session.prepare_camera.call_args.kwargs['video_analysis'])
        folder=captured.call_args.args[2]
        trial=json.loads((folder/'trial.json').read_text(encoding='utf-8'))
        self.assertEqual(trial['video_analysis_status'],'SKIPPED_RECORD_ONLY')
        self.assertEqual(trial['log_image_period_ms'],1)
        self.assertEqual(json.loads((folder/'analysis_plan.json').read_text())['mode'],'record_only')
        self.assertFalse((folder/'quick').exists()); self.assertFalse((folder/'full').exists())
        self.assertEqual(self.panel.table.item(self.panel.rows[str(folder)],'values')[4:],('ログで確認','録画のみ（解析なし）'))
        self.assertEqual(self.panel.workflow_phase,'review')
        self.panel.next_trial()
        self.assertEqual(self.panel.workflow_phase,'placement')

    def test_record_only_plan_never_starts_an_analysis_process(self):
        (self.folder/'analysis_plan.json').write_text('{"mode":"record_only"}')
        with patch.object(panel_module,'analyze_trial') as analyze:
            self.assertEqual(self.panel.analyze(self.folder,4),0)
            self.assertEqual(self.panel.analyze(self.folder,1),0)
        analyze.assert_not_called()
        with patch.object(self.panel,'analyze') as analyze:
            self.assertEqual(self.panel._full(self.folder,archive=False),'録画のみ（解析なし）')
        analyze.assert_not_called()

    def test_rectification_only_runs_without_body_profile_or_height(self):
        from test_pixel_camera_rectify import board_and_reference
        _,reference=board_and_reference()
        self.reference.write_text(json.dumps(reference),encoding='utf-8')
        self.panel.analysis_mode.set('AR正対補正のみ')
        self.panel.geometry.set(str(self.folder/'missing-body.json'));self.panel.height.set('')
        calls=[]
        self.session.close.side_effect=lambda:(calls.append('disconnect'),setattr(self.connection,'is_open',False))
        def capture(session,request,folder,*args):
            request.validate();self.assertTrue(request.rectify_only)
            calls.append('capture');folder.mkdir()
            result=dict(state='CAPTURED_REVIEW_REQUIRED',record_video=True,rectification_only=True,
                saved_log=dict(status='saved'),log_stop_evidence=dict(confirmed=True))
            (folder/'trial.json').write_text(json.dumps(result))
            return result
        def analyze(folder,stride):
            calls.append('rectify'+str(stride))
            path=folder/('full' if stride==1 else 'quick');path.mkdir()
            (path/'summary.json').write_text(json.dumps(dict(analysis_mode='rectify_only',
                rectification_available=True,frames=4,valid_frames=3,registration_rate=.75)))
            return 0
        with patch.object(self.panel,'_launch',side_effect=self.launch_inline), \
                patch.object(panel_module,'apply_connected_settings',return_value=settings()), \
                patch.object(panel_module,'capture_connected',side_effect=capture) as captured, \
                patch.object(panel_module,'render_trial_log',side_effect=lambda folder:calls.append('log')), \
                patch.object(self.panel,'analyze',side_effect=analyze), \
                patch.object(self.panel,'_archive_result',side_effect=lambda folder:calls.append('archive')):
            self.panel.apply_settings();self.panel.run()
        self.assertEqual(calls,['capture','disconnect','log','rectify4','rectify1','archive'])
        folder=captured.call_args.args[2]
        values=self.panel.table.item(self.panel.rows[str(folder)],'values')
        self.assertEqual(values[2:5],('—','—','ログで確認'))
        self.assertEqual(values[5],'正対補正完了（欠測あり）')
        self.assertIn('機体位置の推定は行っていません',self.panel.detail_status.get())
        self.assertEqual(self.panel.workflow_phase,'review')

    def test_rectification_dispatch_and_video_button_use_saved_files_only(self):
        (self.folder/'analysis_plan.json').write_text('{"mode":"rectify_only"}')
        with patch.object(panel_module,'analyze_trial',return_value=0) as analyze:
            self.panel.analyze(self.folder,1)
        analyze.assert_called_once_with(self.folder,1,mode='rectify_only')
        (self.folder/'full').mkdir();video=self.folder/'full/rectified.mp4';video.write_bytes(b'fixture')
        self.panel.latest_folder=self.folder
        with patch.object(panel_module.os,'startfile',create=True) as opened:
            self.panel.open_rectified_video()
        opened.assert_called_once_with(str(video))

    def test_rectification_rejects_missing_reference_before_settings_are_sent(self):
        self.panel.analysis_mode.set('AR正対補正のみ')
        self.panel.reference.set(str(self.folder/'missing-reference.json'))
        with patch.object(self.panel,'_launch') as launch:
            self.panel.apply_settings()
        launch.assert_not_called()

    def test_record_only_in_existing_camera_gui_does_not_launch_background_analysis(self):
        self.panel.guided=False
        self.panel.analysis_mode.set('録画のみ（解析なし）')
        def capture(session,request,folder,*args):
            folder.mkdir()
            result=dict(state='CAPTURED_REVIEW_REQUIRED',record_video=True)
            (folder/'trial.json').write_text(json.dumps(result))
            return result
        with patch.object(self.panel,'_launch',side_effect=self.launch_inline), \
                patch.object(panel_module,'capture_connected',side_effect=capture) as captured, \
                patch.object(self.panel,'analyze') as analyze, patch.object(self.panel,'_full') as full, \
                patch.object(self.panel,'_archive_result') as archived, \
                patch.object(panel_module.threading,'Thread') as thread:
            self.panel.run()
        captured.assert_called_once(); archived.assert_called_once()
        analyze.assert_not_called(); full.assert_not_called(); thread.assert_not_called()
        self.session.close.assert_not_called()

    def test_position_analysis_does_not_claim_video_stop_verification(self):
        (self.folder/'analysis_plan.json').write_text('{"mode":"position_only"}')
        with patch.object(panel_module, 'analyze_trial', return_value=0) as analyze:
            self.panel.analyze(self.folder, 4)
        analyze.assert_called_once_with(self.folder, 4, mode='position_only')
        (self.folder/'full').mkdir()
        (self.folder/'full/summary.json').write_text(json.dumps(dict(analysis_mode='position_only',
            position_available=True, observation_rate=.8)))
        with patch.object(self.panel, 'analyze', return_value=0):
            self.assertEqual(self.panel._full(self.folder, archive=False), '位置推定完了（欠測あり）')

    def test_position_completion_labels_board_coordinates_and_uncalibrated_height(self):
        row=self.panel.table.insert('', 'end', values=('position',19,'—','—','未判定','未実施'))
        self.panel.rows[str(self.folder)]=row
        trial=dict(state='CAPTURED_REVIEW_REQUIRED',record_video=True,full_analysis_status='位置推定完了')
        summary=dict(analysis_mode='position_only',endpoint_board_x_mm=12,endpoint_board_y_mm=34,observation_rate=.99)
        self.panel.completed((self.folder,trial,summary))
        values=self.panel.table.item(row,'values')
        self.assertEqual(values[2:5],('12.00','34.00','未判定'))
        self.assertEqual(self.panel.table.heading('forward','text'),'盤面X[mm]')
        self.assertIn('高さ未補正',self.panel.detail_status.get())

    def test_position_failure_displays_analysis_reason_in_the_gui(self):
        row=self.panel.table.insert('', 'end', values=('position',19,'—','—','未判定','未実施'))
        self.panel.rows[str(self.folder)]=row
        trial=dict(state='CAPTURED_REVIEW_REQUIRED',record_video=True,full_analysis_status='位置推定不成立')
        summary=dict(analysis_mode='position_only',error='盤面のAR座標合わせに失敗しました')
        self.panel.completed((self.folder,trial,summary))
        self.assertIn('原因: 盤面のAR座標合わせ',self.panel.detail_status.get())

    def test_learned_mode_requires_registered_models_before_device_operations(self):
        import pixel_camera_detection
        self.panel.analysis_mode.set('位置＋外観学習（試験）')
        with patch.object(pixel_camera_detection,'ROOT',self.folder), \
                patch.object(pixel_camera_detection,'DEFAULT_REGISTRY',self.folder/'missing_registry.json'), \
                patch.object(self.panel,'_launch') as launch:
            self.panel.apply_settings()
        launch.assert_not_called()
        self.assertIn('registry.json',self.panel.status.get())

    def test_reconnection_requires_settings_verification_again(self):
        self.panel.record_video.set(False)
        self.panel.verified_settings=settings(); self.panel.verified_generation=6
        with patch.object(self.panel, '_launch') as launch:
            self.panel.run()
        launch.assert_not_called()

    def test_missing_video_profile_blocks_settings_before_any_device_operation(self):
        self.panel.geometry.set(str(self.folder/'missing.json'))
        with patch.object(self.panel, '_launch') as launch:
            self.panel.apply_settings()
        launch.assert_not_called()
        self.assertIn('映像解析設定', self.panel.status.get())

    def test_first_connection_reads_machine_preset_even_after_an_offline_selection(self):
        self.gui.ser = None
        self.panel.connected_mouse = None
        new_session = SimpleNamespace(connection=SimpleNamespace(is_open=True))
        new_session.prepare = Mock(return_value=new_session)
        with patch.object(self.panel, '_launch', side_effect=self.launch_inline), \
                patch.object(panel_module, 'check_log_environment'), \
                patch.object(panel_module, 'LiveSession', return_value=new_session), \
                patch.object(self.gui, 'after_idle') as scheduled:
            self.panel.prepare()
        scheduled.assert_called_once_with(self.gui._load_defaults)
        self.assertFalse(new_session.prepare.call_args.kwargs['record_video'])
        self.assertEqual(self.panel.workflow_phase, 'settings')

    def test_capture_failure_disconnects_and_returns_to_review_without_retry(self):
        self.panel.record_video.set(False)
        self.panel.verified_settings = settings(); self.panel.verified_generation = 7
        self.panel.set_workflow('ready')
        def launch(operation, callback):
            self.panel._busy(True)
            try:
                result, error = operation(), None
            except Exception as caught:
                result, error = None, str(caught)
            while not self.gui.events.empty():
                item = self.gui.events.get_nowait()
                if isinstance(item, tuple) and item[0].startswith('pixel_'): self.panel.handle(item)
            self.panel.handle(('pixel_done', callback, result, error))
            return True
        with patch.object(self.panel, '_launch', side_effect=launch), \
                patch.object(panel_module, 'capture_connected', side_effect=TimeoutError('run result unknown')) as captured, \
                patch.object(self.panel, 'analyze') as analyze:
            self.panel.run()
        captured.assert_called_once(); self.session.close.assert_called_once(); analyze.assert_not_called()
        self.assertIsNone(self.gui.ser)
        self.assertIsNone(self.panel.verified_settings)
        self.assertFalse(self.panel.ready.get())
        self.assertEqual(self.panel.workflow_phase, 'review')
        self.assertIn('再送しません', self.panel.status.get())


class VideoSetupTests(unittest.TestCase):
    def test_body_profile_is_machine_specific_and_does_not_invent_axle_calibration(self):
        result=body_geometry('B',dict(reference_id='board'),'green','red',9,7,800)
        self.assertEqual(result['mouse'],'B')
        self.assertIsNone(result['axle_offset_from_marker_midpoint_mm'])
        self.assertEqual(result['reference_id'],'board')

    def test_unknown_colours_heights_and_dimensions_are_rejected(self):
        for front,rear,fh,rh,ch in [('red','red',9,7,800),('pink','yellow',9,7,800),
                ('red','yellow',float('nan'),7,800),('red','yellow',9,900,800),('red','yellow',9,7,0)]:
            with self.subTest(front=front,rear=rear,fh=fh,rh=rh,ch=ch),self.assertRaises(ValueError):
                body_geometry('A',dict(reference_id='board'),front,rear,fh,rh,ch)


class CameraEnvironmentTests(unittest.TestCase):
    def test_invalid_optional_camera_config_does_not_prevent_log_only_startup(self):
        from pixel_camera_environment import initialize_camera_environment
        with tempfile.TemporaryDirectory() as directory:
            project = Path(directory)
            config = project/'.tools/pixel_camera_runtime/environment.json'
            config.parent.mkdir(parents=True); config.write_text('{broken', encoding='utf-8')
            with patch.dict(os.environ, {}, clear=True), \
                    patch('pixel_camera_capture.camera_workspace', side_effect=RuntimeError('missing camera')):
                self.assertIsNone(initialize_camera_environment(project))
                with self.assertRaises(RuntimeError): initialize_camera_environment(project, required=True)

    def test_saved_source_and_versioned_dependencies_are_loaded_without_connecting(self):
        from pixel_camera_environment import initialize_camera_environment
        with tempfile.TemporaryDirectory() as directory:
            project = Path(directory); source = project/'camera'
            client = source/'python/pixel_camera/client.py'
            client.parent.mkdir(parents=True); client.write_text('', encoding='utf-8')
            dependencies = project/'.tools'/f'camera_python{sys.version_info.major}{sys.version_info.minor}'
            dependencies.mkdir(parents=True)
            config = project/'.tools/pixel_camera_runtime/environment.json'
            config.parent.mkdir(parents=True); config.write_text(json.dumps(dict(source_dir=str(source))), encoding='utf-8')
            with patch.dict(os.environ, {'PYTHONPATH':'existing'}, clear=True), patch.object(sys, 'path', list(sys.path)):
                self.assertEqual(initialize_camera_environment(project, required=True), source)
                self.assertEqual(os.environ['PIXEL_CAMERA_REPO'], str(source))
                self.assertIn(str(dependencies), sys.path)
                self.assertIn('existing', os.environ['PYTHONPATH'].split(os.pathsep))


if __name__ == '__main__': unittest.main()
