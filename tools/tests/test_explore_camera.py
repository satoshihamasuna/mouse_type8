import csv
import json
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ble'))
import mouse_ble_explore as explore
import explore_camera as camera


class ReadinessTests(unittest.TestCase):
    def setUp(self):
        self.snap = dict(state='waiting_sensor', streaming=True, rows=10, sample_age_seconds=.01)
        self.status = dict(connected=True, device=dict(phase='RECORDING', recording_id='mine', owner='owner'))

    def test_ok_requires_fresh_stream_wait_state_and_owned_recording(self):
        self.assertTrue(camera.ready_for_gesture(self.snap, self.status, 'mine', 'owner'))
        for key, value in [('state', 'running'), ('state', 'starting'), ('streaming', False),
                           ('rows', 0), ('sample_age_seconds', None), ('sample_age_seconds', .6)]:
            self.assertFalse(camera.ready_for_gesture(dict(self.snap, **{key:value}), self.status, 'mine', 'owner'))
        for key, value in [('phase', 'PREPARING'), ('owner', 'other'), ('recording_id', 'other')]:
            changed = dict(self.status, device=dict(self.status['device'], **{key:value}))
            self.assertFalse(camera.ready_for_gesture(self.snap, changed, 'mine', 'owner'))
        self.assertFalse(camera.ready_for_gesture(self.snap, self.status, 'mine', 'owner', True))


class ExposureTests(unittest.TestCase):
    def test_current_maze_reference_is_preferred_and_legacy_remains_available(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            self.assertEqual(camera.default_maze_reference(root),root/'board_9x9_reference.json')
            current=root/'board_9x9_current_reference.json'
            current.write_text('{}',encoding='utf-8')
            self.assertEqual(camera.default_maze_reference(root),current)

    def test_saved_preferences_are_only_loaded_as_valid_display_values(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'preferences.json'
            self.assertEqual(camera.load_camera_preferences(path),{})
            camera.save_json(path,dict(schema=1,camera_settings=dict(ev=-2.,torch=False)))
            self.assertEqual(camera.load_camera_preferences(path),dict(ev=-2.,torch=False))
            camera.save_json(path,dict(schema=1,camera_settings=dict(ev=-20.,torch=False)))
            self.assertEqual(camera.load_camera_preferences(path),{})
            path.write_text('{broken',encoding='utf-8')
            self.assertEqual(camera.load_camera_preferences(path),{})

    def client(self, response=None):
        from unittest.mock import Mock
        client=Mock()
        client.get_capabilities.return_value=dict(cameras=[dict(target_fixed_240_supported=True,
            back_facing=True,ev_step='1/6',ev_steps_range='[-24, 24]')])
        client.configure.side_effect=(lambda **kwargs: kwargs) if response is None else (lambda **kwargs: response)
        return client

    def test_preserve_does_not_send_ev_or_light_or_query_capabilities(self):
        client=self.client()
        result=camera.configure_camera(client,{})
        client.configure.assert_called_once_with()
        client.get_capabilities.assert_not_called()
        self.assertFalse(result['physical_brightness_effect_verified'])

    def test_ev_is_converted_using_device_step_and_receipt_saved(self):
        client=self.client()
        result=camera.configure_camera(client,dict(ev=-.5,torch=False))
        client.configure.assert_called_once_with(exposure_compensation_steps=-3,torch=False)
        self.assertEqual(result['selection']['ev'],-.5)
        self.assertEqual(result['ev_step'],'1/6')
        self.assertEqual(result['response']['exposure_compensation_steps'],-3)

    def test_out_of_range_and_unacknowledged_settings_are_rejected(self):
        client=self.client()
        with self.assertRaises(ValueError):camera.configure_camera(client,dict(ev=-5))
        client.configure.assert_not_called()
        client=self.client(dict(exposure_compensation_steps=0))
        with self.assertRaisesRegex(RuntimeError,'一致'):camera.configure_camera(client,dict(ev=-.5))
        client.configure.assert_called_once()


class CoordinatorTests(unittest.TestCase):
    def exercise(self, mode='complete'):
        actions = []; phases = []; api_calls = []
        with tempfile.TemporaryDirectory() as folder:
            directory = Path(folder)
            class Session:
                state = 'idle'
                def ready(self, options): return self.state == 'idle'
                def snapshot(self):
                    return dict(state=self.state, streaming=self.state != 'idle', rows=12,
                                sample_age_seconds=.01, crc_errors=0, lost=0, time_ms=1000, sequence=10,
                                values={'ideal.velo':.3, 'ideal.rad_velo':0})
                def action(self, action, options=None):
                    actions.append(action)
                    if action == 'start':
                        if mode == 'run_error':
                            coordinator.observe('run_error','start rejected')
                            return
                        self.state = 'waiting_sensor'; coordinator.observe('state', self.state)
                        coordinator.observe('telemetry', self.snapshot())
                        def finish():
                            self.state = 'idle'; coordinator.observe('result', {'result':'complete'})
                        threading.Timer(.05, finish).start()
            session = Session(); session.directory = directory; session._run_settings_path = directory/'run.json'
            class Client:
                client_id = 'owner'
                phase = 'IDLE'
                def get_status(self):
                    return dict(connected=True, device=dict(phase=self.phase, recording_id='mine',owner='owner',
                        elapsed_seconds=.1,max_recording_seconds=30 if mode=='old_camera' else 300))
                def configure(self): api_calls.append('configure')
                def start_recording(self, seconds):
                    api_calls.append('record')
                    if mode == 'lost_start_reply': raise TimeoutError('start reply unknown')
                    if mode == 'lost_start_preparing':
                        self.phase='PREPARING'; raise TimeoutError('start reply unknown')
                    self.phase='RECORDING'; return 'mine'
                def stop_recording(self):
                    api_calls.append('stop'); self.phase='IDLE'; return {'accepted':True}
                def wait_for_transfer(self, **kwargs):
                    api_calls.append('transfer')
                    return SimpleNamespace(file_path=directory/'video.mp4',metadata_path=directory/'video.json',
                        sha256='a'*64,size=10,source_deleted=True,frame_count=240,frame_count_source='decoder')
            coordinator = camera.ExploreCameraRun(session, lambda k,d:phases.append((k,d)),
                client_factory=Client, finalizer=lambda p:(p.parent/'review.html').write_text('review'), interval=.005,
                analysis_config={'enabled':True} if mode=='analysis_error' else None)
            if mode == 'analysis_error':
                reference = dict(markers_cell={'0':[[0,0],[1,0],[1,1],[0,1]]})
                camera.save_json(directory/'board_reference.json', reference)
                coordinator.analysis_config['reference'] = str(directory/'board_reference.json')
            coordinator.start(explore.ShortestOptions(limit_seconds=180) if mode=='shortest'
                              else explore.ExploreOptions(limit_seconds=180))
            coordinator.thread.join(5)
            self.assertFalse(coordinator.active)
            trial = json.loads(next(directory.glob('camera_*/trial.json')).read_text(encoding='utf-8'))
            if mode == 'analysis_error':
                saved_reference = Path(trial['position_analysis']['reference'])
                self.assertEqual(saved_reference.parent, coordinator.folder)
                self.assertEqual(json.loads(saved_reference.read_text(encoding='utf-8')), reference)
        return actions, phases, api_calls, trial

    def test_recording_ready_precedes_ble_start_and_result_precedes_video_stop(self):
        actions, phases, api, trial = self.exercise()
        self.assertEqual(actions, ['start'])
        self.assertEqual(api, ['configure','record','stop','transfer'])
        self.assertIn(('camera_ready',True), phases)
        self.assertEqual(trial['status'],'TRANSFERRED')
        self.assertTrue(trial['controlled_stop'])
        self.assertEqual(trial['frame_count'],240)

    def test_shortest_uses_the_same_recording_guard_and_keeps_run_type(self):
        actions, phases, api, trial=self.exercise('shortest')
        self.assertEqual(actions,['start'])
        self.assertEqual(trial['run_type'],'shortest')
        self.assertEqual(trial['options']['profile'],'plain500')
        self.assertIn(('camera_ready',True),phases)

    def test_position_analysis_failure_preserves_successful_capture_and_basic_review(self):
        from unittest.mock import patch
        with patch.object(camera,'analyze_trial',side_effect=RuntimeError('missing cv2')):
            actions, phases, api, trial=self.exercise('analysis_error')
        self.assertEqual(trial['status'],'TRANSFERRED')
        done=next(d for k,d in phases if k=='camera_done')
        self.assertFalse(done['position_available'])
        self.assertTrue(done['position_requested'])
        self.assertFalse(any(k=='camera_failure' for k,d in phases))

    def test_old_camera_never_receives_start_or_shows_ok(self):
        actions, phases, api, trial = self.exercise('old_camera')
        self.assertEqual(actions,[]); self.assertEqual(api,[])
        self.assertNotIn(('camera_ready',True),phases)
        self.assertEqual(trial['status'],'FAILED')

    def test_unknown_recording_start_is_not_retried_and_never_starts_mouse(self):
        actions, phases, api, trial = self.exercise('lost_start_reply')
        self.assertEqual(actions,[]); self.assertEqual(api,['configure','record'])
        self.assertNotIn(('camera_ready',True),phases)
        self.assertEqual(trial['status'],'FAILED')

    def test_rejected_start_does_not_review_a_previous_run(self):
        actions, phases, api, trial = self.exercise('run_error')
        self.assertEqual(trial['status'],'FAILED')
        self.assertNotIn('run_settings',trial)
        self.assertFalse(any(kind=='camera_done' for kind,_ in phases))

    def test_unknown_start_owned_by_this_client_can_be_stopped_without_retry(self):
        actions, phases, api, trial = self.exercise('lost_start_preparing')
        self.assertEqual(actions,[]); self.assertEqual(api,['configure','record','stop','transfer'])
        self.assertEqual(trial['status'],'FAILED'); self.assertEqual(trial['recording_id'],'mine')

    def test_commit_guard_rejects_wrong_recording_stale_log_and_insufficient_time(self):
        session = SimpleNamespace(snapshot=lambda:dict(rows=5,streaming=True,sample_age_seconds=.01,crc_errors=0,lost=0))
        job = camera.ExploreCameraRun(session,lambda *a:None)
        job.options=explore.ExploreOptions(limit_seconds=180); job.duration=255
        job.baseline_crc=job.baseline_lost=0; job.recording_id='mine'; job.client=SimpleNamespace(client_id='owner')
        job.status=lambda:dict(connected=True,device=dict(phase='RECORDING',recording_id='mine',owner='owner',elapsed_seconds=1))
        job.check_before_commit()
        job.status=lambda:dict(connected=True,device=dict(phase='RECORDING',recording_id='mine',owner='owner',elapsed_seconds=90))
        with self.assertRaisesRegex(RuntimeError,'残り時間'): job.check_before_commit()
        job.cancelled.set()
        with self.assertRaisesRegex(RuntimeError,'取り消'): job.check_before_commit()


class ReviewTests(unittest.TestCase):
    def test_review_preserves_mcu_origin_and_uses_explicitly_coarse_camera_offset(self):
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)
            fields=['time_ms','ideal.velo','ego.velo','ideal.rad_velo','ego.rad_velo','sen_l.distance','sen_r.distance']
            with (p/'telemetry.csv').open('w',newline='') as f:
                w=csv.writer(f); w.writerow(fields)
                for i in range(12): w.writerow([1000+i*5,0 if i==0 else .3,.3,0,0,40,45])
            camera.save_json(p/'run.json',dict(telemetry_start={'time_ms':1000},telemetry_end={'time_ms':1055}))
            trial=dict(run_settings=str(p/'run.json'),ble_directory=str(p),video_file=str(p/'video.mp4'),
                recording_id='mine',motion_anchor={'host_monotonic':100,'time_ms':1005},
                clock_samples=[dict(pc_before=99.99,pc_after=100.01,camera_elapsed_s=2,recording_id='mine')])
            camera.save_json(p/'trial.json',trial); camera.build_review(p/'trial.json')
            sync=json.loads((p/'sync_initial.json').read_text(encoding='utf-8'))
            self.assertEqual(sync['mcu_origin_ms'],1005)
            self.assertAlmostEqual(sync['video_time_at_mcu_origin_s'],2)
            self.assertTrue(sync['manual_alignment_required'])
            self.assertIn('sync_manual.json',(p/'review.html').read_text(encoding='utf-8'))


class CameraReceiptTests(unittest.TestCase):
    def test_duplicate_acknowledgements_do_not_rewrite_a_confirmed_receipt(self):
        from pixel_camera_capture import camera_workspace
        from unittest.mock import patch
        sys.path.insert(0,str(camera_workspace()/'python'))
        import pixel_camera.transfer as transfer
        with tempfile.TemporaryDirectory() as folder:
            store=transfer.ReceiveStore(Path(folder)); path=store.receipts/'receipt.json'; sha='a'*64
            path.write_text(json.dumps({'sha256':sha,'source_deleted':True}),encoding='utf-8')
            original=path.read_bytes()
            with patch.object(transfer,'atomic_json') as write:
                store.mark_source_deleted('receipt',sha)
                write.assert_not_called()
            self.assertEqual(path.read_bytes(),original)


class GuiTests(unittest.TestCase):
    def setUp(self):
        from unittest.mock import patch
        import mouse_ble_explore_camera_gui as gui
        from pixel_camera_profiles import DEFAULTS
        self.addCleanup(patch.stopall)
        patch.object(gui, 'save_camera_profile').start()
        patch.object(gui, 'load_camera_profile', return_value=dict(DEFAULTS['maze_9x9'])).start()

    def test_marker_colours_are_selected_from_saved_run_settings_instead_of_fixed_to_a(self):
        import mouse_ble_explore_camera_gui as gui
        root=gui.tk.Tk(); root.withdraw()
        try:
            app=gui.ExploreCameraApp(root)
            with tempfile.TemporaryDirectory() as folder:
                reference=Path(folder)/'reference.json'; reference.write_text('{}',encoding='utf-8')
                app.reference_path.set(str(reference)); app.analysis_python_path.set(sys.executable)
                config=app.analysis_config()
            self.assertEqual(config['marker_colours_source'],'run_settings.firmware.mouse')
            self.assertNotIn('front_colour',config)
            self.assertNotIn('rear_colour',config)
        finally: root.destroy()

    def test_shortest_camera_run_preserves_profile_maze_goal_and_ekf_choice(self):
        from unittest.mock import patch
        import mouse_ble_explore_camera_gui as gui
        root=gui.tk.Tk(); root.withdraw()
        try:
            app=gui.ExploreCameraApp(root)
            app.session=SimpleNamespace()
            app.analysis_enabled.set(False)
            app.run_mode.set('最短走行'); app.profile.set('turn_600（plain600）')
            app.profiles=['plain500','plain600']; app.maze_source.set('保存済み迷路（Flash）')
            app.goal_x.set(7); app.goal_y.set(6); app.goal_size.set(1); app.limit.set(90)
            app.snapshot={'wall_ekf_supported':True}; app.wall_ekf_mode.set('EKF制御（control）')
            app.capture_ev.set('-0.5'); app.capture_torch.set('OFF')
            with patch.object(gui,'ExploreCameraRun') as coordinator:
                app.start()
                options=coordinator.return_value.start.call_args.args[0]
                self.assertIsInstance(options,explore.ShortestOptions)
                self.assertEqual((options.goal_x,options.goal_y,options.goal_size),(7,6,1))
                self.assertEqual((options.profile,options.maze_source),('plain600','flash'))
                self.assertEqual((options.limit_seconds,options.wall_ekf_mode),(90,'control'))
                self.assertEqual(coordinator.call_args.kwargs['camera_settings'],
                                 dict(ev=-.5,torch=False,zoom_ratio=1.,focus_lock=True))
                gui.save_camera_profile.assert_called_once_with('maze_9x9',
                    dict(ev=-.5,torch=False,zoom_ratio=1.,focus_lock=True))
        finally: root.destroy()

    def test_options_remain_user_selected_and_no_connection_is_started_on_open(self):
        from unittest.mock import Mock, patch
        import mouse_ble_explore_camera_gui as gui
        root=gui.tk.Tk(); root.withdraw()
        try:
            factory=Mock(); app=gui.ExploreCameraApp(root,session_factory=factory)
            factory.assert_not_called()
            app.session=SimpleNamespace()
            app.goal_x.set(5); app.goal_y.set(6); app.goal_size.set(1)
            app.roundtrip.set(False); app.accelerated.set(True); app.limit.set(150)
            app.snapshot={'wall_ekf_supported':True}; app.wall_ekf_mode.set('EKF制御（control）')
            with patch.object(gui,'ExploreCameraRun') as coordinator:
                app.start()
                options=coordinator.return_value.start.call_args.args[0]
                self.assertEqual((options.goal_x,options.goal_y,options.goal_size),(5,6,1))
                self.assertFalse(options.roundtrip); self.assertTrue(options.accelerated)
                self.assertEqual(options.limit_seconds,150); self.assertEqual(options.wall_ekf_mode,'control')
            self.assertEqual(str(app.start_button['state']),'disabled')
        finally: root.destroy()

    def test_gesture_ok_banner_is_large_green_and_revocable(self):
        import mouse_ble_explore_camera_gui as gui
        root=gui.tk.Tk(); root.withdraw()
        try:
            app=gui.ExploreCameraApp(root)
            app.handle_event('camera_ready',True)
            self.assertIn('OK',app.gesture_text.get()); self.assertEqual(app.gesture_sign.cget('bg'),'#15803d')
            app.handle_event('camera_ready',False)
            self.assertNotIn('OK',app.gesture_text.get()); self.assertNotEqual(app.gesture_sign.cget('bg'),'#15803d')
        finally: root.destroy()


if __name__=='__main__': unittest.main()
