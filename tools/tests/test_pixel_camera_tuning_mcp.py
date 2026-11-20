"""MCPからの単体走行受付と保存済み解析を実機なしで検証する。"""
from contextlib import nullcontext
import asyncio
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
try:
    import pixel_camera_tuning_mcp as mcp
except ModuleNotFoundError as error:
    if error.name != 'mcp':
        raise
    raise unittest.SkipTest('MCPの依存セットアップが必要です') from error
from test_pixel_camera_tuning import settings
from mcp.server.fastmcp.exceptions import ToolError


class McpTuningTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.folder = Path(directory.name)
        self.service = mcp.McpTuningService(self.folder / 'session', {})
        self.settings = settings()
        reference = self.save('reference.json', dict(reference_id='test'))
        geometry = self.save('geometry.json', dict(mouse='A', reference_id='test',
            front_height_mm=10, rear_height_mm=1))
        self.config = self.save('config.json', dict(mouse='A', address='test-address', height_mm=800,
            reference=str(reference), geometry=str(geometry)))
        self.candidate = self.save('settings.json', dict(actual_settings=self.settings))
        self.placement = dict(x_mm=225, y_mm=45, heading_deg=90, position_tolerance_mm=5, heading_tolerance_deg=2)

    def save(self, name, value):
        path = self.folder / name
        path.write_text(json.dumps(value), encoding='utf-8')
        return path

    def plan(self):
        return self.service.prepare(self.config, self.candidate, self.placement, None)

    def run_capture_job(self, plan, identifier):
        self.assertEqual(self.service.submit('capture', plan, identifier)['state'], 'ACCEPTED')
        with self.service.operation_lock:
            return self.service.status(identifier)['result']

    def test_prepare_is_offline_and_operator_confirmation_is_required(self):
        with patch.object(mcp, 'LiveSession') as live:
            plan = self.plan()
            self.assertEqual(plan['settings'], self.settings)
            with self.assertRaises(ValueError):
                self.service.submit('capture', dict(plan, operator_ready=False), 'unconfirmed')
            self.assertFalse(list((self.service.folder / 'jobs').iterdir()))
            live.assert_not_called()

    def test_camera_health_tool_reports_failure_without_preparing_a_run(self):
        server = mcp.create_server(self.service)
        unhealthy = dict(passed=False, failure_reasons=['battery_below_minimum'],
                         battery=dict(level=1, scale=100, plugged=True))
        with patch('pixel_camera_health.inspect_camera_health', return_value=unhealthy) as health, \
                patch.object(mcp, 'LiveSession') as live, \
                patch.object(self.service, 'prepare') as prepare, \
                patch.object(self.service, 'submit') as submit:
            result = asyncio.run(server.call_tool('check_camera_health',
                dict(adb='test-adb', serial='test-phone', min_level=30)))
            serialized = json.dumps(result, default=lambda value: value.model_dump())
            self.assertIn('battery_below_minimum', serialized)
            health.assert_called_once_with(adb='test-adb', serial='test-phone', min_level=30)
            live.assert_not_called()
            prepare.assert_not_called()
            submit.assert_not_called()
        self.assertFalse(list((self.service.folder / 'jobs').iterdir()))

    def test_camera_health_tool_rejects_invalid_threshold_before_device_read(self):
        server = mcp.create_server(self.service)
        with patch('pixel_camera_health.inspect_camera_health') as health:
            for threshold in (-1, 101, float('nan'), True):
                with self.subTest(threshold=threshold), self.assertRaises(ToolError):
                    asyncio.run(server.call_tool('check_camera_health', dict(min_level=threshold)))
            health.assert_not_called()

    def test_static_mismatch_closes_connection_and_request_survives_restart(self):
        plan = dict(self.plan(), operator_ready=True)
        with patch.object(mcp, 'LiveSession') as live, \
                patch.object(mcp, 'static_check', side_effect=RuntimeError('姿勢不一致')), \
                patch.object(mcp.TuningService, 'perform') as capture:
            result = self.run_capture_job(plan, 'mismatch')
            self.assertEqual(result['state'], 'FAILED')
            self.assertFalse(result['execution_write_attempted'])
            self.assertTrue(result['ble_disconnected'])
            capture.assert_not_called()
            live.return_value.prepare.assert_not_called()
            live.return_value.prepare_camera.assert_called_once()
            live.return_value.close.assert_called_once()
        restarted = mcp.McpTuningService(self.service.folder, {})
        with self.assertRaises(FileExistsError):
            restarted.submit('capture', plan, 'mismatch')

    def test_successful_run_defers_analysis_and_closes_ble(self):
        config=mcp.read_json(self.config)
        mcp.save_json(self.config,dict(config,stream_period_ms=10))
        plan = dict(self.plan(), operator_ready=True, post_straight_accel_m_s2=20)
        original = dict(self.settings, preset_speed_mm_s=700)
        shell = Mock()

        def static_check(*args):
            live.return_value.prepare.assert_not_called()
            return dict(passed=True)

        def capture(session, request, folder, parse, progress):
            self.assertTrue(request.operator_ready)
            live.return_value.prepare.assert_called_once_with('test-address')
            verify.assert_called_once_with(shell, 'straight', 20)
            folder.mkdir()
            state = dict(state='CAPTURED_REVIEW_REQUIRED', execution_write_attempted=True)
            mcp.save_json(folder / 'trial.json', state)
            return state

        with patch.object(mcp, 'LiveSession') as live, \
                patch.object(mcp, 'static_check', side_effect=static_check) as static, \
                patch.object(mcp.TuningService, 'shell', return_value=nullcontext(shell)), \
                patch.object(mcp.TuningService, 'read_settings', side_effect=[original, self.settings]) as read, \
                patch.object(mcp, 'verify_acceleration', return_value=dict(verified={'acc':20})) as verify, \
                patch('pixel_camera_tuning_cli.capture_connected', side_effect=capture) as execute, \
                patch('pixel_camera_tuning_cli.analyze_trial') as analyze:
            result = self.run_capture_job(plan, 'single')
            self.assertEqual(result['state'], 'CAPTURED_REVIEW_REQUIRED')
            self.assertTrue(result['analysis_deferred'])
            self.assertTrue(result['execution_write_attempted'])
            self.assertTrue(result['ble_disconnected'])
            self.assertEqual(live.call_args.kwargs['stream_period_ms'],10)
            self.assertEqual(read.call_args.args[-1], self.settings['preset_speed_mm_s'])
            static.assert_called_once()
            live.return_value.prepare_camera.assert_called_once()
            execute.assert_called_once()
            analyze.assert_not_called()
            live.return_value.close.assert_called_once()
        details = Path(result['trial_json']).parent / 'operator_confirmation.json'
        self.assertEqual(mcp.read_json(details)['source'], 'explicit MCP operator_ready')

    def test_acceleration_only_changes_acc_and_requires_readback(self):
        for motion in ('diagonal', 'straight'):
            with self.subTest(motion=motion):
                response = (f'DEBUG_PARAM {motion} distance:127.279 acc:6.5 max_velo:1.6 end_velo:0 '
                            'sp:2,0.016,0 om:0.12,0.004,0 ff:0.903,0.083,0.081,0.017,0.001,0.002,0.061 '
                            'suction:1 duty:650')
                verified = response.replace('acc:6.5', 'acc:20')
                shell = Mock()
                shell.query.side_effect = [response, verified]
                result = mcp.verify_acceleration(shell, motion, 20)
                self.assertEqual(result['verified'], dict(result['before'], acc=20))
                shell.send.assert_called_once_with(
                    f'debug {motion} set 127.279 20 1.6 0 2 0.016 0 0.12 0.004 0 '
                    '0.903 0.083 0.081 0.017 0.001 0.002 0.061 1 650')
                shell.reset_mock()
                shell.query.side_effect = [response, response]
                with self.assertRaises(RuntimeError):
                    mcp.verify_acceleration(shell, motion, 20)
                shell.send.assert_called_once()

    def test_saved_analysis_and_operator_records_are_outside_of_archive_inputs(self):
        (self.folder / 'run').mkdir()
        trial = self.save('run/trial.json', dict(state='CAPTURED_REVIEW_REQUIRED'))
        marker = self.service.analysis_marker(trial)
        marker.parent.mkdir()
        archive = self.folder / 'archive'

        def analyze(folder, stride):
            report = folder / ('quick' if stride == 4 else 'full') / 'summary.json'
            report.parent.mkdir()
            mcp.save_json(report, dict(auto_stop_confirmed=True))
            return 0

        with patch.object(mcp, 'analyze_trial', side_effect=analyze) as analysis, \
                patch('pixel_camera_archive.archive_for_trial', return_value=None), \
                patch('pixel_camera_archive.archive_trial', return_value=archive) as archive_trial, \
                patch('pixel_camera_archive.write_index'):
            mcp.postprocess(trial, marker)
            self.assertEqual(analysis.call_count, 2)
            archive_trial.assert_called_once_with(trial)
        result = mcp.read_json(marker)
        self.assertEqual(result['state'], 'REVIEW_REQUIRED')
        self.assertEqual(result['archive'], str(archive))
        self.assertFalse(marker.is_relative_to(trial.parent))
        self.assertFalse(self.service.operator_record('single').is_relative_to(trial.parent))


if __name__ == '__main__':
    unittest.main()
