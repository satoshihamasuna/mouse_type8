"""実機・Tkウィンドウを開かず、CLI受付・直列化・一回実行を検証する。"""
from contextlib import contextmanager
from copy import deepcopy
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import pixel_camera_tuning_cli as cli
from pixel_camera_tuning import settings_set_command
from test_pixel_camera_tuning import settings


class Variable:
    def __init__(self, value):
        self.value = value
    def get(self):
        return self.value


class CliTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.folder = Path(self.directory.name)
        reference, geometry = self.folder/'reference.json', self.folder/'geometry.json'
        reference.write_text(json.dumps(dict(reference_id='test')), encoding='utf-8')
        geometry.write_text(json.dumps(dict(mouse='A', reference_id='test', front_height_mm=10,
            rear_height_mm=1, axle_offset_from_marker_midpoint_mm=[0, 0])), encoding='utf-8')
        self.config = dict(mouse='A', height_mm=800, reference=str(reference), geometry=str(geometry))
        self.session = SimpleNamespace(folder=self.folder/'transport', alive=lambda:True,
            connection=SimpleNamespace(is_open=True), close=Mock())
        self.service = cli.TuningService(self.folder/'service', self.config, self.session)
        self.settings = settings()
        self.settings_path = self.folder/'settings.json'
        self.settings_path.write_text(json.dumps(self.settings), encoding='utf-8')
        self.descriptor = dict(port=12345, token='test')
        cli.save_json(self.service.folder/'service.json', self.descriptor)

    def wait_job(self, identifier):
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            value = self.service.status(identifier)
            if value['state'] in ('DONE', 'FAILED'):
                return value
            time.sleep(.005)
        self.fail('job did not finish')

    def args(self, *command):
        return cli.parser().parse_args(['--session', str(self.service.folder), *command])

    def test_missing_confirmation_or_invalid_settings_never_reaches_worker(self):
        self.service.perform = Mock()
        for payload in ({'settings':self.settings}, {'settings':self.settings, 'operator_ready':'true'}):
            with self.assertRaises(ValueError):
                self.service.submit('capture', payload, 'missing_confirmation')
        for name, value in (('velo',float('nan')), ('r_min',42), ('lstart',-1)):
            invalid = deepcopy(self.settings)
            invalid['parameters'][name] = value
            with self.assertRaises(ValueError):
                self.service.submit('capture', dict(settings=invalid,operator_ready=True), 'bad_' + name)
        wrong_mouse = dict(self.settings, mouse='B')
        with self.assertRaises(ValueError):
            self.service.submit('apply', dict(settings=wrong_mouse), 'wrong_mouse')
        self.service.perform.assert_not_called()
        self.assertEqual(list((self.service.folder/'jobs').iterdir()), [])

    def test_duplicate_id_remains_consumed_after_new_service_instance(self):
        self.service.perform = Mock(return_value={'ok':True})
        payload = dict(settings=self.settings, operator_ready=True)
        self.service.submit('capture', payload, 'once')
        self.assertEqual(self.wait_job('once')['state'], 'DONE')
        with self.assertRaises(FileExistsError):
            self.service.submit('capture', payload, 'once')
        resumed = cli.TuningService(self.service.folder, self.config, self.session)
        resumed.perform = Mock()
        with self.assertRaises(FileExistsError):
            resumed.submit('capture', payload, 'once')
        resumed.perform.assert_not_called()
        self.assertEqual(self.service.perform.call_count, 1)

    def test_json_save_retries_only_transient_windows_file_conflicts(self):
        path = self.folder/'result.json'
        original_replace = Path.replace
        for code in (5, 32, 33):
            with self.subTest(winerror=code):
                path.write_text('{"old":true}', encoding='utf-8')
                error = PermissionError('temporary file conflict')
                error.winerror = code
                attempts = []
                def replace(temporary, destination):
                    attempts.append(destination)
                    if len(attempts) <= 3:
                        self.assertEqual(json.loads(path.read_text(encoding='utf-8')), {'old':True})
                        raise error
                    return original_replace(temporary, destination)
                with patch.object(Path, 'replace', replace), patch.object(cli.time, 'sleep') as sleep:
                    cli.save_json(path, {'new':code})
                self.assertEqual(len(attempts), 4)
                self.assertEqual([call.args[0] for call in sleep.call_args_list], [.05, .1, .2])
                self.assertEqual(json.loads(path.read_text(encoding='utf-8')), {'new':code})

    def test_json_save_persistent_conflict_is_bounded_and_keeps_previous_result(self):
        path = self.folder/'result.json'
        path.write_text('{"old":true}', encoding='utf-8')
        error = PermissionError('persistent file conflict')
        error.winerror = 5
        with patch.object(Path, 'replace', side_effect=error) as replace, patch.object(cli.time, 'sleep') as sleep:
            with self.assertRaises(PermissionError):
                cli.save_json(path, {'new':True})
        self.assertEqual(replace.call_count, 4)
        self.assertEqual(sleep.call_count, 3)
        self.assertEqual(json.loads(path.read_text(encoding='utf-8')), {'old':True})

    def test_json_save_does_not_retry_other_permission_errors(self):
        for code in (None, 13):
            with self.subTest(winerror=code):
                error = PermissionError('not a transient Windows conflict')
                if code is not None:
                    error.winerror = code
                with patch.object(Path, 'replace', side_effect=error) as replace, patch.object(cli.time, 'sleep') as sleep:
                    with self.assertRaises(PermissionError):
                        cli.save_json(self.folder/'result.json', {'new':True})
                replace.assert_called_once()
                sleep.assert_not_called()

    def test_progress_file_conflict_does_not_repeat_capture_or_release_consumed_id(self):
        original_replace = Path.replace
        failed = []
        def replace(temporary, destination):
            value = json.loads(temporary.read_text(encoding='utf-8'))
            if destination.name == 'progress_conflict.json' and value.get('event') == 'camera_motion_start_confirmed' and not failed:
                failed.append(True)
                error = PermissionError('progress destination is temporarily open')
                error.winerror = 5
                raise error
            return original_replace(temporary, destination)
        def operation(action, payload, identifier, progress):
            progress('camera_motion_start_confirmed', {})
            return {'captured_once':True}
        self.service.perform = Mock(side_effect=operation)
        payload = dict(settings=self.settings, operator_ready=True)
        with patch.object(Path, 'replace', replace):
            self.service.submit('capture', payload, 'progress_conflict')
            value = self.wait_job('progress_conflict')
        self.assertEqual(value['state'], 'DONE')
        self.assertTrue(value['result']['captured_once'])
        self.assertEqual(failed, [True])
        self.service.perform.assert_called_once()
        with self.assertRaises(FileExistsError):
            self.service.submit('capture', payload, 'progress_conflict')
        self.service.perform.assert_called_once()

    def test_busy_worker_rejects_changes_and_close_but_status_remains_read_only(self):
        entered, release = threading.Event(), threading.Event()
        def operation(*args):
            entered.set()
            self.assertTrue(release.wait(3))
            return {'ok':True}
        self.service.perform = Mock(side_effect=operation)
        self.service.submit('capture', dict(settings=self.settings, operator_ready=True), 'busy')
        self.assertTrue(entered.wait(1))
        try:
            self.assertTrue(self.service.status()['busy'])
            self.assertEqual(self.service.status('busy')['state'], 'RUNNING')
            for action, payload in (('read',dict(motion='in_l45')), ('apply',dict(settings=self.settings)), ('close',{})):
                with self.assertRaises(cli.BusyError):
                    self.service.submit(action, payload, action)
            self.assertEqual(self.service.perform.call_count, 1)
        finally:
            release.set()
        self.wait_job('busy')

    def test_ambiguous_capture_failure_is_saved_and_id_cannot_run_again(self):
        self.service.perform = Mock(side_effect=TimeoutError('execution response lost'))
        payload = dict(settings=self.settings, operator_ready=True)
        self.service.submit('capture', payload, 'lost_response')
        value = self.wait_job('lost_response')
        self.assertEqual(value['state'], 'FAILED')
        self.assertIn('response lost', value['error'])
        with self.assertRaises(FileExistsError):
            self.service.submit('capture', payload, 'lost_response')
        self.assertEqual(self.service.perform.call_count, 1)

    def test_capture_commands_reuse_transport_and_forward_confirmation_once_per_request(self):
        def capture(session, request, folder, parse, progress):
            self.assertIs(session, self.session)
            self.assertTrue(request.operator_ready)
            folder.mkdir()
            return dict(state='FAILED', execution_write_attempted=True, camera_run_start_confirmed=False)
        with patch.object(cli,'capture_connected',side_effect=capture) as execute, patch.object(cli,'analyze_trial') as analyze:
            for name in ('one','two'):
                result = self.service.perform('capture',dict(settings=self.settings,operator_ready=True), name, Mock())
                self.assertEqual(result['capture']['state'], 'FAILED')
                self.assertTrue((self.service.folder/('run_'+name)/'operator_confirmation.json').exists())
            self.assertEqual(execute.call_count, 2)
            analyze.assert_not_called()
        self.session.close.assert_not_called()

    def test_apply_sends_one_set_and_mismatched_readback_is_not_retried(self):
        before, after = deepcopy(self.settings), deepcopy(self.settings)
        before['parameters']['lend'] = 24
        after['parameters']['lend'] = 18
        shell = Mock()
        @contextmanager
        def connection(folder):
            yield shell
        with patch.object(self.service,'shell',connection), patch.object(self.service,'read_settings',side_effect=[before,after]) as read:
            with self.assertRaisesRegex(RuntimeError,'読み返し'):
                self.service.perform('apply',dict(settings=self.settings),'apply_mismatch',Mock())
        shell.send.assert_called_once_with(settings_set_command(self.settings))
        self.assertEqual(read.call_count, 2)
        self.assertNotIn('exe', shell.send.call_args.args[0])

    def test_read_and_preset_have_no_set_or_execution(self):
        shell = Mock()
        @contextmanager
        def connection(folder):
            yield shell
        with patch.object(self.service,'shell',connection), patch.object(self.service,'read_settings',return_value=self.settings) as read:
            self.service.perform('read',dict(motion='long_r180'),'read',Mock())
            self.service.perform('preset',dict(motion='long_r180',preset=500),'preset',Mock())
        self.assertEqual(read.call_args_list[0].args[2], None)
        self.assertEqual(read.call_args_list[1].args[2], 500)
        shell.send.assert_not_called()

    def test_wrong_preset_blocks_apply_before_parameter_write(self):
        wrong = deepcopy(self.settings)
        wrong['preset_speed_mm_s'] = 700
        shell = Mock()
        @contextmanager
        def connection(folder):
            yield shell
        with patch.object(self.service,'shell',connection), patch.object(self.service,'read_settings',return_value=wrong):
            with self.assertRaisesRegex(RuntimeError,'preset'):
                self.service.perform('apply',dict(settings=self.settings),'wrong_preset',Mock())
        shell.send.assert_not_called()

    def test_client_post_response_loss_does_not_retry(self):
        args = self.args('capture','--settings',str(self.settings_path),'--operator-ready','--request-id','lost')
        with patch.object(cli,'exchange',side_effect=TimeoutError('lost')) as exchange:
            with self.assertRaises(TimeoutError):
                cli.client(args)
        exchange.assert_called_once()
        self.assertEqual(exchange.call_args.args[1], '/jobs')

    def test_client_capture_needs_new_confirmation_and_explicit_id(self):
        with self.assertRaises(SystemExit):
            self.args('capture','--settings',str(self.settings_path),'--operator-ready')
        args = self.args('capture','--settings',str(self.settings_path),'--request-id','no_confirmation')
        with patch.object(cli,'exchange') as exchange:
            with self.assertRaises(ValueError):
                cli.client(args)
        exchange.assert_not_called()

    def test_client_can_change_one_parameter_without_touching_gains(self):
        args = self.args('apply','--settings',str(self.settings_path),'--set','lend=25','--no-wait')
        with patch.object(cli,'exchange',return_value=dict(state='ACCEPTED')) as exchange:
            cli.client(args)
        expected = deepcopy(self.settings)
        expected['parameters']['lend'] = 25
        self.assertEqual(exchange.call_args.args[2]['payload']['settings'],expected)

    def test_status_client_and_close_do_not_poll_or_resubmit_a_mutation(self):
        with patch.object(cli,'exchange',return_value=dict(state='FAILED')) as exchange:
            cli.client(self.args('status','--request-id','known'))
        exchange.assert_called_once_with(self.descriptor,'/jobs/known')
        with patch.object(cli,'exchange',return_value=dict(state='ACCEPTED')) as exchange:
            cli.client(self.args('close'))
        self.assertEqual(exchange.call_count,1)

    def test_http_status_and_authentication_without_actuator_side_effects(self):
        server = cli.make_http_server(self.service,'token',port=0)
        thread = threading.Thread(target=server.serve_forever,daemon=True)
        thread.start()
        try:
            descriptor = dict(port=server.server_port,token='token')
            self.service.perform = Mock()
            self.assertFalse(cli.exchange(descriptor,'/status')['busy'])
            with self.assertRaises(RuntimeError):
                cli.exchange(dict(descriptor,token='wrong'),'/jobs',dict(id='bad',action='close'))
            self.service.perform.assert_not_called()
            self.assertFalse(list((self.service.folder/'jobs').iterdir()))
        finally:
            server.shutdown()
            server.server_close()
            thread.join(2)

    def test_http_client_roundtrip_and_duplicate_capture_do_not_replay(self):
        server = cli.make_http_server(self.service,'token',port=0)
        thread = threading.Thread(target=server.serve_forever,daemon=True)
        thread.start()
        cli.save_json(self.service.folder/'service.json',dict(port=server.server_port,token='token'))
        self.service.perform = Mock(return_value=self.settings)
        try:
            output = self.folder/'readback.json'
            for unused in range(2):
                value = cli.client(self.args('read','--motion','long_r180','--output',str(output)))
                self.assertEqual(value['state'],'DONE')
                self.assertEqual(json.loads(output.read_text(encoding='utf-8')),self.settings)
            capture = self.args('capture','--settings',str(self.settings_path),'--operator-ready','--request-id','http_once')
            self.assertEqual(cli.client(capture)['state'],'DONE')
            with self.assertRaises(RuntimeError):
                cli.client(capture)
            self.assertEqual(self.service.perform.call_count,3)
            self.assertIs(self.service.session,self.session)
            self.session.close.assert_not_called()
        finally:
            server.shutdown()
            server.server_close()
            thread.join(2)

    def test_startup_failure_closes_transport_and_removes_own_lock(self):
        config = dict(self.config,address='test-address')
        path = self.folder/'config.json'
        path.write_text(json.dumps(config),encoding='utf-8')
        args = self.args('serve','--config',str(path),'--port','0')
        session = Mock()
        session.prepare.side_effect = RuntimeError('device unavailable')
        with patch.object(cli,'LiveSession',return_value=session):
            with self.assertRaisesRegex(RuntimeError,'unavailable'):
                cli.serve(args)
        session.close.assert_called_once()
        self.assertFalse((self.service.folder/'service.lock').exists())

    def test_capture_or_quality_failure_is_not_a_successful_cli_exit(self):
        for result, code in ((dict(capture=dict(state='FAILED')),1),
                             (dict(capture=dict(state='CAPTURED_REVIEW_REQUIRED'),full_returncode=2),2)):
            with patch.object(cli,'client',return_value=dict(state='DONE',result=result)):
                self.assertEqual(cli.main(['--session',str(self.service.folder),'status']),code)

    def test_gui_and_cli_share_identical_turn_command_format(self):
        import myshell_debug as debug
        s = self.settings
        fields = dict(zip([key for _,key in debug.FIELDS],
            [s['parameters'][name] for name in ('velo','r_min','lstart','lend','degree','correction')]
            + [s['pid'][key] for key in ('sp_kp','sp_ki','sp_kd','om_kp','om_ki','om_kd')]))
        fields.update(s['feedforward'])
        gui = SimpleNamespace(values={k:Variable(v) for k,v in fields.items()},
            motion=Variable('turn'),turn_type=Variable('long_r180'),suction_enable=Variable(False),
            _ff_values=lambda:[s['feedforward'][key] for _,key in debug.FF_FIELDS],
            _turn_count_value=lambda:1,_pre_accel_value=lambda:6.5,_suction_duty_value=lambda:650)
        self.assertEqual(debug.DebugGui._set_command(gui),settings_set_command(s))


if __name__ == '__main__':
    unittest.main()
