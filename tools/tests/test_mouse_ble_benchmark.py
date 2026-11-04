"""BLE benchmark calculations and bounded failure paths; no physical device is accessed."""
import asyncio
import binascii
import csv
import io
import json
from pathlib import Path
import struct
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import AsyncMock, patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools' / 'ble'))
import mouse_ble_benchmark as benchmark


def frame(sequence, time_ms, period=5):
    values = [0.0] * 55
    values[51:] = (100.0, 20.0, 150.0 + sequence % 100, 1000.0)
    payload = struct.pack('<4sIIH55e', b'L8S2', sequence, time_ms, period, *values)
    return payload + struct.pack('<H', binascii.crc_hqx(payload, 0xffff))


def diagnostics(**overrides):
    result = dict.fromkeys(benchmark.transport.DIAGNOSTICS_FIELDS, 0)
    result.update(version=1, rx_buffer_size=252, rx_buffer_count=4,
                  rx_timeout_us=1000, rx_mode_flags=1)
    result.update(overrides)
    return result


def link_diagnostics(**overrides):
    result = dict.fromkeys(benchmark.transport.LINK_DIAGNOSTICS_FIELDS, 0)
    result.update(version=1, mtu=247, interval_us=15000, supervision_timeout_ms=4000,
                  tx_phy=2, rx_phy=2, tx_max_len=251, tx_max_time_us=1060,
                  rx_max_len=251, rx_max_time_us=1060, ring_capacity=16384,
                  batch_flush_ms=7, sdc_tx_packet_count=10, acl_tx_count=12,
                  acl_tx_size=251, flags=1)
    result.update(overrides)
    return result


class MeasurementTests(unittest.TestCase):
    def test_filter_previous_period_and_measure_clock_or_backlog_slope(self):
        output = io.StringIO()
        capture = benchmark.SegmentCapture(output, 5)
        capture.feed(frame(1, 100, 20), 10.0)
        capture.feed(frame(2, 120), 10.02)
        capture.feed(frame(3, 125), 10.026)
        capture.feed(frame(4, 130), 10.032)
        result = capture.report()
        self.assertEqual(result['rows'], 3)
        self.assertEqual(result['stale_frames_ignored'], 1)
        self.assertEqual(result['effective_hz'], 200)
        self.assertAlmostEqual(result['relative_lag_slope_ms_per_s'], 200)
        self.assertTrue(result['cadence_within_5_percent'])
        self.assertEqual(result['irq_cumulative_max_start_us'], 152)
        self.assertEqual(result['irq_cumulative_max_end_us'], 154)
        self.assertEqual(len(output.getvalue().splitlines()), 4)

    def test_split_notifications_and_device_time_wrap(self):
        capture = benchmark.SegmentCapture(io.StringIO(), 5)
        first = frame(0xffffffff, 0xfffffffd)
        capture.feed(first[:22], 1)
        self.assertEqual(capture.csv.rows, 0)
        capture.feed(first[22:] + frame(0, 2), 1.005)
        self.assertEqual(capture.report()['interval_ms']['mean'], 5)
        self.assertEqual(capture.csv.lost, 0)

    def test_crc_loss_and_period_switch_are_failures(self):
        for suffix, expected in ((frame(2, 110), 'Missing'), (frame(1, 105, 4), 'period changed')):
            capture = benchmark.SegmentCapture(io.StringIO(), 5)
            capture.feed(frame(0, 100), 1)
            with self.assertRaisesRegex(benchmark.BenchmarkError, expected):
                capture.feed(suffix, 1.01)
        broken = bytearray(frame(0, 100))
        broken[30] ^= 1
        with self.assertRaisesRegex(benchmark.BenchmarkError, 'CRC'):
            benchmark.SegmentCapture(io.StringIO(), 5).feed(broken, 1)

    def test_missing_sample_statistics_include_the_last_written_csv_row(self):
        output = io.StringIO()
        capture = benchmark.SegmentCapture(output, 3)
        capture.feed(frame(0, 100, 3), 1.0)
        capture.feed(frame(1, 103, 3), 1.003)
        with self.assertRaisesRegex(benchmark.BenchmarkError, 'Missing 1 log frame'):
            capture.feed(frame(3, 109, 3), 1.009)
        rows = list(csv.DictReader(io.StringIO(output.getvalue())))
        result = capture.report()
        self.assertEqual(result['rows'], len(rows))
        self.assertEqual(result['rows'], 3)
        self.assertEqual(result['lost_frames'], 1)
        csv_span = (int(rows[-1]['time_ms']) - int(rows[0]['time_ms'])) / 1000
        self.assertEqual(result['device_span_s'], csv_span)
        self.assertAlmostEqual(result['effective_hz'], (len(rows) - 1) / csv_span)
        self.assertEqual(capture.intervals, [3, 6])
        self.assertEqual(result['interval_ms']['mean'], 4.5)
        self.assertEqual(result['irq_cumulative_max_end_us'], 153)
        self.assertEqual(len(capture.relative_lag), len(rows))
        self.assertEqual(len(capture.irq['total_us']), len(rows))

    def test_zero_loss_does_not_hide_missed_sample_cadence(self):
        capture = benchmark.SegmentCapture(io.StringIO(), 5)
        capture.feed(frame(0, 100) + frame(1, 106) + frame(2, 112), 1)
        self.assertEqual(capture.csv.lost, 0)
        self.assertFalse(capture.report()['cadence_within_5_percent'])

    def test_idle_identity_and_all_status_counters_are_checked(self):
        benchmark.require_idle('EXPLORE_CAP protocol:1 mouse:A goal_x:7 state:idle\r\n', 'A')
        for text in ('', 'EXPLORE_CAP mouse:B state:idle', 'EXPLORE_CAP mouse:A state:running'):
            with self.assertRaises(benchmark.BenchmarkError):
                benchmark.require_idle(text, 'A')
        for index in range(4):
            after = [0] * 4
            after[index] = 1
            with self.assertRaises(benchmark.BenchmarkError):
                benchmark.require_healthy((0, 0, 0, 0), after)
        self.assertEqual(benchmark.status_delta((0xffffffff, 0, 0, 0), (0, 0, 0, 0))['dropped_uart_bytes'], 1)


    def test_diagnostic_counter_wrap_and_normal_counter_growth(self):
        before = diagnostics(rx_restart_attempts=0xffffffff)
        after = diagnostics(rx_restart_attempts=1, rx_disabled_count=2,
                            rx_buf_request_events=3, last_stop_reason=0x80,
                            stop_reason_or=0x81, rx_buffer_size=128)
        delta = benchmark.diagnostics_delta(before, after)
        self.assertEqual(delta['rx_restart_attempts'], 2)
        self.assertEqual(delta['rx_disabled_count'], 2)
        self.assertNotIn('version', delta)
        self.assertNotIn('last_stop_reason', delta)
        self.assertNotIn('stop_reason_or', delta)
        self.assertNotIn('rx_buffer_size', delta)
        benchmark.require_diagnostics_healthy(before, after)
        for field in benchmark.DIAGNOSTIC_ERROR_COUNTERS:
            with self.subTest(field=field):
                initial = diagnostics(**{field: 0xffffffff})
                current = diagnostics(**{field: 0})
                self.assertEqual(benchmark.diagnostics_delta(initial, current)[field], 1)
                with self.assertRaises(benchmark.BenchmarkError):
                    benchmark.require_diagnostics_healthy(initial, current)

    def test_link_delta_only_counts_notifications_and_allows_retries(self):
        before = link_diagnostics(notify_calls=0xffffffff, notify_bytes=0xfffffffe,
                                  notify_retry_events=0xffffffff, ring_highwater=100,
                                  notify_call_max_ms=10)
        after = link_diagnostics(notify_calls=1, notify_bytes=3, notify_retry_events=2,
                                 ring_highwater=120, notify_call_max_ms=11,
                                 interval_us=7500)
        self.assertEqual(benchmark.link_diagnostics_delta(before, after), {
            'notify_calls': 2, 'notify_bytes': 5, 'notify_failures': 0, 'notify_retry_events': 3})
        benchmark.require_link_healthy(before, after)
        with self.assertRaisesRegex(benchmark.BenchmarkError, 'notification failures'):
            benchmark.require_link_healthy(link_diagnostics(notify_failures=0xffffffff),
                                          link_diagnostics(notify_failures=0))

    def test_link_connection_metrics_use_actual_units_and_keep_unknown_values(self):
        self.assertEqual(benchmark.link_connection_metrics(None), {
            'connection_interval_ms': None, 'phy': None, 'data_length': None,
            'link_metrics_source': 'unknown'})
        result = benchmark.link_connection_metrics(link_diagnostics(interval_us=7500, rx_phy=0,
                                                                   rx_max_len=0, rx_max_time_us=0))
        self.assertEqual(result['connection_interval_ms'], 7.5)
        self.assertEqual(result['phy'], {'tx': 2, 'rx': None})
        self.assertEqual(result['data_length'], {'tx_octets': 251, 'tx_time_us': 1060,
                                               'rx_octets': None, 'rx_time_us': None})
        unknown = benchmark.link_connection_metrics(dict.fromkeys(benchmark.transport.LINK_DIAGNOSTICS_FIELDS, 0))
        self.assertIsNone(unknown['connection_interval_ms'])
        self.assertIsNone(unknown['phy'])
        self.assertIsNone(unknown['data_length'])


class SafetyTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        # Legacy firmware has no optional diagnostic characteristic.
        for name in ('read_diagnostics', 'read_link_diagnostics'):
            patcher = patch.object(benchmark, name, AsyncMock(return_value=None))
            patcher.start()
            self.addCleanup(patcher.stop)

    async def diagnostic_segment(self, reader, *, expected_support=None,
                                 link_reader=None, expected_link_support=None):
        clock = [0.0]
        state = {'failure': None, 'highwater': 0}
        if expected_support is not None:
            state['diagnostics_supported'] = expected_support
        if expected_link_support is not None:
            state['link_supported'] = expected_link_support
        class Incoming:
            sequence = 0
            def qsize(self):
                return 0
            async def get(self):
                clock[0] += 0.005
                packet = frame(self.sequence, 100 + self.sequence * 5)
                self.sequence += 1
                return clock[0], packet
        with tempfile.TemporaryDirectory(dir=ROOT / 'build') as directory, \
                patch.object(benchmark, 'time', SimpleNamespace(monotonic=lambda: clock[0])), \
                patch.object(benchmark, 'stream_control', AsyncMock()), \
                patch.object(benchmark, 'read_status', AsyncMock(return_value=(0, 0, 0, 0))), \
                patch.object(benchmark, 'read_diagnostics', reader), \
                patch.object(benchmark, 'read_link_diagnostics',
                             link_reader if link_reader is not None else AsyncMock(return_value=None)), \
                patch.object(benchmark, 'drain', AsyncMock(return_value=0)):
            result = await benchmark.capture_segment(None, None, Incoming(), state,
                                                     Path(directory), 1, 5, 0.02)
            stored = json.loads((Path(directory) / '01_5ms.json').read_text(encoding='utf-8'))
        return result, stored

    async def test_diagnostics_record_before_after_and_allow_normal_activity(self):
        calls = 0
        async def reader(*args):
            nonlocal calls
            calls += 1
            return diagnostics(rx_disabled_count=calls, rx_restart_attempts=calls * 2,
                               rx_buf_request_events=calls * 3)
        result, stored = await self.diagnostic_segment(reader)
        self.assertTrue(result['passed'], result)
        self.assertTrue(result['diagnostics_supported'])
        self.assertTrue(result['diagnostics_final_read'])
        self.assertEqual(result['diagnostics_before']['rx_disabled_count'], 1)
        self.assertEqual(result['diagnostics_after']['rx_disabled_count'], 3)
        self.assertEqual(result['diagnostics_delta']['rx_restart_attempts'], 4)
        self.assertEqual(result['diagnostics_last_observed'], result['diagnostics_after'])
        self.assertEqual(stored['diagnostics_after'], result['diagnostics_after'])

    async def test_diagnostic_error_growth_fails_even_when_legacy_status_is_healthy(self):
        calls = 0
        async def reader(*args):
            nonlocal calls
            calls += 1
            return diagnostics(framing_count=0 if calls == 1 else 1,
                               last_stop_reason=0 if calls == 1 else 4)
        result, stored = await self.diagnostic_segment(reader)
        self.assertFalse(result['passed'])
        self.assertIn('framing_count', result['error'])
        self.assertEqual(result['bridge_delta'], dict.fromkeys(benchmark.STATUS_NAMES, 0))
        self.assertTrue(result['diagnostics_final_read'])
        self.assertEqual(result['diagnostics_delta']['framing_count'], 1)
        self.assertEqual(stored['diagnostics_last_observed']['last_stop_reason'], 4)

    async def test_diagnostic_read_failure_is_not_unsupported(self):
        result, stored = await self.diagnostic_segment(AsyncMock(side_effect=TimeoutError('diagnostic timeout')))
        self.assertFalse(result['passed'])
        self.assertIsNone(result['diagnostics_supported'])
        self.assertIn('TimeoutError', result['diagnostics_read_error'])
        self.assertIsNone(stored['diagnostics_after'])
        self.assertFalse(stored['diagnostics_final_read'])

    async def test_diagnostic_read_failure_keeps_the_last_valid_snapshot(self):
        reader = AsyncMock(side_effect=[diagnostics(), TimeoutError('read failed'), TimeoutError('read failed')])
        result, stored = await self.diagnostic_segment(reader)
        self.assertFalse(result['passed'])
        self.assertTrue(result['diagnostics_supported'])
        self.assertEqual(result['diagnostics_last_observed'], diagnostics())
        self.assertFalse(result['diagnostics_final_read'])
        self.assertIsNone(result['diagnostics_after'])
        self.assertIsNone(stored['diagnostics_delta'])
        self.assertIn('TimeoutError', stored['diagnostics_read_error'])

    async def test_failed_baseline_read_can_keep_a_later_final_snapshot_without_delta(self):
        reader = AsyncMock(side_effect=[TimeoutError('baseline read failed'), diagnostics()])
        result, stored = await self.diagnostic_segment(reader, expected_support=True)
        self.assertFalse(result['passed'])
        self.assertIn('TimeoutError', result['error'])
        self.assertNotIn('cleanup_error', result)
        self.assertIsNone(result['diagnostics_before'])
        self.assertTrue(result['diagnostics_final_read'])
        self.assertEqual(result['diagnostics_after'], diagnostics())
        self.assertIsNone(stored['diagnostics_delta'])

    async def test_diagnostics_cannot_disappear_after_support_was_confirmed(self):
        result, stored = await self.diagnostic_segment(AsyncMock(return_value=None), expected_support=True)
        self.assertFalse(result['passed'])
        self.assertTrue(result['diagnostics_supported'])
        self.assertIn('disappeared', result['diagnostics_read_error'])
        self.assertIsNone(stored['diagnostics_after'])

    async def test_link_snapshots_keep_peaks_and_configs_without_counter_deltas(self):
        calls = 0
        async def reader(*args):
            nonlocal calls
            calls += 1
            return link_diagnostics(notify_calls=calls * 10, notify_bytes=calls * 2440,
                                    notify_retry_events=calls, ring_highwater=calls * 500,
                                    notify_call_max_ms=calls, interval_us=30000 if calls == 1 else 15000)
        result, stored = await self.diagnostic_segment(AsyncMock(return_value=diagnostics()), link_reader=reader)
        self.assertTrue(result['passed'], result)
        self.assertTrue(result['diagnostics_supported'])
        self.assertTrue(result['link_supported'])
        self.assertTrue(result['link_final_read'])
        self.assertEqual(result['link_before']['interval_us'], 30000)
        self.assertEqual(result['link_after']['ring_highwater'], 1500)
        self.assertEqual(result['link_delta'], {'notify_calls': 20, 'notify_bytes': 4880,
                                              'notify_failures': 0, 'notify_retry_events': 2})
        self.assertEqual(result['link_last_observed'], result['link_after'])
        self.assertEqual(result['link_last_observed_metrics']['connection_interval_ms'], 15)
        self.assertEqual(stored['link_after'], result['link_after'])

    async def test_link_notify_failure_fails_with_healthy_legacy_and_rx_diagnostics(self):
        reader = AsyncMock(side_effect=[link_diagnostics(), link_diagnostics(notify_failures=1),
                                        link_diagnostics(notify_failures=1)])
        result, stored = await self.diagnostic_segment(AsyncMock(return_value=diagnostics()), link_reader=reader)
        self.assertFalse(result['passed'])
        self.assertIn('notification failures', result['error'])
        self.assertEqual(stored['link_delta']['notify_failures'], 1)
        self.assertEqual(stored['bridge_delta'], dict.fromkeys(benchmark.STATUS_NAMES, 0))

    async def test_link_unsupported_does_not_disable_existing_rx_diagnostics(self):
        result, stored = await self.diagnostic_segment(AsyncMock(return_value=diagnostics()))
        self.assertTrue(result['passed'], result)
        self.assertTrue(result['diagnostics_supported'])
        self.assertFalse(result['link_supported'])
        self.assertFalse(result['link_final_read'])
        self.assertIsNone(stored['link_before'])
        self.assertIsNone(stored['link_after'])
        self.assertIsNone(stored['link_delta'])
        self.assertEqual(result['link_last_observed_metrics']['link_metrics_source'], 'unknown')

    async def test_link_read_failure_is_distinct_from_unsupported_and_preserves_last_snapshot(self):
        for responses in ([TimeoutError('baseline failure')],
                          [link_diagnostics(), TimeoutError('final failure'), ConnectionError('disconnected')]):
            with self.subTest(responses=len(responses)):
                result, stored = await self.diagnostic_segment(AsyncMock(return_value=None),
                    link_reader=AsyncMock(side_effect=responses))
                self.assertFalse(result['passed'])
                self.assertIn('Error', result['link_read_error'])
                self.assertFalse(result['link_final_read'])
                self.assertIsNone(stored['link_after'])
                self.assertIsNone(stored['link_delta'])
                if len(responses) == 1:
                    self.assertIsNone(result['link_supported'])
                    self.assertIsNone(stored['link_last_observed'])
                else:
                    self.assertTrue(result['link_supported'])
                    self.assertEqual(stored['link_last_observed'], link_diagnostics())

    async def test_link_cannot_disappear_at_baseline_or_later_in_one_connection(self):
        for responses, supported in (([None, None], True),
                                     ([link_diagnostics(), None, None], None)):
            with self.subTest(expected_support=supported):
                result, stored = await self.diagnostic_segment(AsyncMock(return_value=None),
                    link_reader=AsyncMock(side_effect=responses), expected_link_support=supported)
                self.assertFalse(result['passed'])
                self.assertTrue(result['link_supported'])
                self.assertIn('disappeared', result['link_read_error'])
                self.assertIsNone(stored['link_after'])

    async def test_link_failed_baseline_can_save_final_snapshot_without_delta(self):
        result, stored = await self.diagnostic_segment(AsyncMock(return_value=None),
            link_reader=AsyncMock(side_effect=[TimeoutError('baseline failure'), link_diagnostics()]),
            expected_link_support=True)
        self.assertFalse(result['passed'])
        self.assertNotIn('cleanup_error', result)
        self.assertIsNone(stored['link_before'])
        self.assertTrue(stored['link_final_read'])
        self.assertEqual(stored['link_after'], link_diagnostics())
        self.assertIsNone(stored['link_delta'])

    async def test_control_packets_only_use_stream_actions(self):
        sent = []
        async def write(client, data, lock):
            sent.append(data)
        with patch.object(benchmark.transport, 'write_bytes', write):
            await benchmark.stream_control(None, None, 5)
            await benchmark.stream_control(None, None, stop=True)
        self.assertEqual([packet[3] for packet in sent], [1, 0])

    async def test_mock_stream_records_csv_raw_boundaries_and_healthy_metadata(self):
        incoming = asyncio.Queue()
        producing = False
        producer = None
        async def produce():
            sequence = 0
            while producing:
                await asyncio.sleep(0.005)
                if producing:
                    incoming.put_nowait((time.monotonic(), frame(sequence, 100 + sequence * 5)))
                    sequence += 1
        async def control(client, lock, period_ms=20, *, stop=False):
            nonlocal producing, producer
            producing = not stop
            if not stop and producer is None:
                producer = asyncio.create_task(produce())
        async def status(*args):
            return (0, 0, 0, 0)
        async def drain(*args):
            return 0
        with tempfile.TemporaryDirectory(dir=ROOT / 'build') as directory, \
                patch.object(benchmark, 'stream_control', control), \
                patch.object(benchmark, 'read_status', status), \
                patch.object(benchmark, 'drain', drain):
            result = await benchmark.capture_segment(None, None, incoming,
                {'failure': None, 'highwater': 0}, Path(directory), 1, 5, 0.05)
            await producer
            self.assertTrue(result['passed'], result)
            self.assertAlmostEqual(result['sample']['effective_hz'], 200)
            raw = (Path(directory) / '01_5ms.notifications.bin').read_bytes()
            self.assertEqual(raw[:4], benchmark.RAW_MAGIC)
            received_at, size = benchmark.RAW_HEADER.unpack_from(raw, 4)
            self.assertGreaterEqual(received_at, 0)
            self.assertEqual(size, 126)
            self.assertEqual(raw[4 + benchmark.RAW_HEADER.size:][:4], b'L8S2')
            self.assertEqual(result['bridge_delta'], dict.fromkeys(benchmark.STATUS_NAMES, 0))
            self.assertTrue(result['bridge_final_status_read'])
            self.assertFalse(result['diagnostics_supported'])
            self.assertIsNone(result['diagnostics_before'])
            self.assertIsNone(result['diagnostics_after'])
            self.assertFalse(result['diagnostics_final_read'])

    async def test_disconnect_does_not_publish_a_stale_status_as_final(self):
        clock = [0.0]
        state = {'failure': None, 'highwater': 0}
        reads = 0
        class Incoming:
            sequence = 0
            def qsize(self):
                return 0
            async def get(self):
                clock[0] += 0.6
                packet = frame(self.sequence, 100 + self.sequence * 5)
                self.sequence += 1
                return clock[0], packet
        async def control(*args, **kwargs):
            pass
        async def status(*args):
            nonlocal reads
            reads += 1
            if reads == 2:
                state['failure'] = 'BLE disconnected'
            if reads >= 3:
                raise RuntimeError('Not connected')
            return (0, 0, 0, 0)
        async def drain(*args):
            return 0
        with tempfile.TemporaryDirectory(dir=ROOT / 'build') as directory, \
                patch.object(benchmark, 'time', SimpleNamespace(monotonic=lambda: clock[0])), \
                patch.object(benchmark, 'read_diagnostics', AsyncMock(return_value=diagnostics(rx_restart_attempts=2))), \
                patch.object(benchmark, 'read_link_diagnostics', AsyncMock(return_value=link_diagnostics(notify_calls=2))), \
                patch.object(benchmark, 'stream_control', control), \
                patch.object(benchmark, 'read_status', status), \
                patch.object(benchmark, 'drain', drain):
            result = await benchmark.capture_segment(None, None, Incoming(), state,
                                                     Path(directory), 1, 5, 5)
            self.assertFalse(result['passed'])
            self.assertIn('BLE disconnected', result['error'])
            self.assertIn('Not connected', result['cleanup_error'])
            self.assertFalse(result['bridge_final_status_read'])
            self.assertIsNone(result['bridge_after'])
            self.assertIsNone(result['bridge_delta'])
            self.assertEqual(result['bridge_last_observed'], dict.fromkeys(benchmark.STATUS_NAMES, 0))
            self.assertTrue(result['diagnostics_supported'])
            self.assertFalse(result['diagnostics_final_read'])
            self.assertIsNone(result['diagnostics_after'])
            self.assertIsNone(result['diagnostics_delta'])
            self.assertEqual(result['diagnostics_last_observed'], diagnostics(rx_restart_attempts=2))
            self.assertTrue(result['link_supported'])
            self.assertFalse(result['link_final_read'])
            self.assertIsNone(result['link_after'])
            self.assertIsNone(result['link_delta'])
            self.assertEqual(result['link_last_observed'], link_diagnostics(notify_calls=2))
            stored = json.loads((Path(directory) / '01_5ms.json').read_text(encoding='utf-8'))
            self.assertIsNone(stored['bridge_after'])

    async def test_failure_still_stops_and_preserves_report(self):
        actions = []
        async def control(client, lock, period_ms=20, *, stop=False):
            actions.append(0 if stop else 1)
        async def status(client, lock):
            return (0, 0, 0, 0)
        async def drain(incoming, seconds=0.3):
            return 0
        state = {'failure': 'PC receive queue overflow', 'highwater': 512}
        with tempfile.TemporaryDirectory(dir=ROOT / 'build') as directory, \
                patch.object(benchmark, 'stream_control', control), \
                patch.object(benchmark, 'read_status', status), \
                patch.object(benchmark, 'drain', drain):
            result = await benchmark.capture_segment(None, None, asyncio.Queue(), state,
                                                     Path(directory), 1, 5, 0.1)
            self.assertFalse(result['passed'])
            self.assertEqual(actions, [0, 1, 0])
            stored = json.loads((Path(directory) / '01_5ms.json').read_text(encoding='utf-8'))
            self.assertIn('PC receive queue overflow', stored['error'])
            self.assertEqual((Path(directory) / '01_5ms.notifications.bin').read_bytes(), b'M8N1')

    async def test_summary_uses_latest_actual_link_metrics_and_inherits_support(self):
        connected = link_diagnostics(interval_us=30000, tx_phy=1, rx_phy=1)
        negotiated = link_diagnostics(interval_us=15000, notify_calls=10)
        client = SimpleNamespace(is_connected=True, mtu_size=247, start_notify=AsyncMock())
        context = AsyncMock()
        context.__aenter__.return_value = client
        async def segment(client, lock, incoming, state, directory, index, period, duration):
            self.assertTrue(state['link_supported'])
            return {'passed': True, 'link_last_observed': negotiated}
        fake_bleak = SimpleNamespace(BleakClient=lambda *args, **kwargs: context,
            BleakScanner=SimpleNamespace(find_device_by_name=AsyncMock(return_value=SimpleNamespace(address='test'))))
        with tempfile.TemporaryDirectory(dir=ROOT / 'build') as directory, \
                patch.dict(sys.modules, {'bleak': fake_bleak}), \
                patch.object(benchmark, 'read_status', AsyncMock(return_value=(0, 0, 0, 0))), \
                patch.object(benchmark, 'read_link_diagnostics', AsyncMock(return_value=connected)), \
                patch.object(benchmark, 'stream_control', AsyncMock()), \
                patch.object(benchmark, 'drain', AsyncMock(return_value=0)), \
                patch.object(benchmark, 'query_idle', AsyncMock(return_value='EXPLORE_CAP mouse:A state:idle\r\n')), \
                patch.object(benchmark, 'capture_segment', segment):
            args = SimpleNamespace(output_root=Path(directory), mouse='A', periods=(20,),
                duration=0.1, address=None, name='test', enter_shell=False, settle_seconds=0)
            result = await benchmark.run(args)
        self.assertTrue(result['passed'], result)
        self.assertEqual(result['link_at_connection'], connected)
        self.assertEqual(result['link_last_observed'], negotiated)
        self.assertEqual(result['connection_interval_ms'], 15)
        self.assertEqual(result['phy'], {'tx': 2, 'rx': 2})
        self.assertEqual(result['data_length']['tx_octets'], 251)
        self.assertEqual(result['link_observation_segment'], 1)

    async def test_failed_segment_prevents_faster_tests(self):
        tested = []
        class Client:
            is_connected = True
            mtu_size = 247
            def __init__(self, *args, **kwargs):
                pass
            async def __aenter__(self):
                return self
            async def __aexit__(self, *args):
                pass
            async def start_notify(self, *args):
                pass
        async def find(*args, **kwargs):
            return SimpleNamespace(address='test-device')
        async def control(*args, **kwargs):
            pass
        async def drain(*args):
            return 0
        async def status(*args):
            return (0, 0, 0, 0)
        async def idle(*args):
            return 'EXPLORE_CAP mouse:A state:idle\r\n'
        async def segment(client, lock, incoming, state, directory, index, period, duration):
            tested.append(period)
            return {'passed': False, 'error': 'test failure'}
        fake_bleak = SimpleNamespace(BleakClient=Client,
                                    BleakScanner=SimpleNamespace(find_device_by_name=find))
        with tempfile.TemporaryDirectory(dir=ROOT / 'build') as directory, \
                patch.dict(sys.modules, {'bleak': fake_bleak}), \
                patch.object(benchmark, 'stream_control', control), \
                patch.object(benchmark, 'read_status', status), \
                patch.object(benchmark, 'drain', drain), \
                patch.object(benchmark, 'query_idle', idle), \
                patch.object(benchmark, 'capture_segment', segment):
            args = SimpleNamespace(output_root=Path(directory), mouse='A', periods=(20, 10, 5),
                                   duration=0.1, address=None, name='test', enter_shell=False, settle_seconds=0)
            result = await benchmark.run(args)
            self.assertEqual(tested, [20])
            self.assertFalse(result['passed'])


if __name__ == '__main__':
    unittest.main()
