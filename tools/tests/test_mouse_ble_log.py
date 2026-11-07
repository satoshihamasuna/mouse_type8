"""Streaming protocol tests; no Bluetooth or robot is accessed."""
import binascii
import csv
import io
import os
from pathlib import Path
import re
import struct
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools' / 'ble'))
import mouse_ble_log as log
from test_side_wall_stability import compiler_command


def frame(sequence=0, time_ms=100, period_ms=20, *, legacy=False):
    count = 51 if legacy else len(log.LOG_LABELS)
    payload = struct.pack('<4sIIH{}e'.format(count), b'L8S1' if legacy else log.MAGIC,
                          sequence, time_ms, period_ms, *[i / 4 for i in range(count)])
    return payload + struct.pack('<H', binascii.crc_hqx(payload, 0xffff))


class DecoderTests(unittest.TestCase):
    def test_mixed_stream_preserves_text_split_by_binary_frames(self):
        text = []
        decoder = log.StreamDecoder()
        wire = b'EXPLORE_WA' + frame() + b'IT_SENSOR\r\nEXPLORE_END result:complete\r\n'
        samples = []
        for byte in wire:
            samples += decoder.feed(bytes([byte]), text.append)
        self.assertEqual(len(samples), 1)
        self.assertEqual(b''.join(text), b'EXPLORE_WAIT_SENSOR\r\nEXPLORE_END result:complete\r\n')

    def test_all_notification_splits_and_text_are_supported(self):
        wire = b'boot\r\n' + frame() + frame(1, 120)
        for split in range(len(wire) + 1):
            decoder = log.StreamDecoder()
            samples = decoder.feed(wire[:split]) + decoder.feed(wire[split:])
            self.assertEqual([sample.sequence for sample in samples], [0, 1])
            self.assertEqual(samples[0].values, tuple(i / 4 for i in range(len(log.LOG_LABELS))))

    def test_crc_resync_and_bounded_noise_buffer(self):
        decoder = log.StreamDecoder()
        broken = bytearray(frame())
        broken[25] ^= 1
        self.assertEqual(decoder.feed(b'x' * 10000), [])
        self.assertLessEqual(len(decoder.buffer), 3)
        samples = decoder.feed(broken + b'LOG,text\r\n' + frame(2, 140))
        self.assertEqual([sample.sequence for sample in samples], [2])
        self.assertEqual(decoder.crc_errors, 1)

    def test_invalid_frame_magic_inside_other_binary_is_preserved(self):
        decoder = log.StreamDecoder()
        text = []
        binary = b'BINARY\r\n' + log.MAGIC + bytes(log.FRAME.size) + b'\r\n> '
        samples = decoder.feed(binary + frame(), text.append)
        self.assertEqual(len(samples), 1)
        self.assertEqual(b''.join(text), binary)
        self.assertEqual(decoder.crc_errors, 1)

    def test_csv_counts_missing_frames_and_timestamp_and_sequence_wrap(self):
        output = io.StringIO()
        capture = log.CsvCapture(output)
        capture.feed(frame(0xfffffffe, 0xfffffff0) + frame(1, 44))
        rows = list(csv.DictReader(io.StringIO(output.getvalue())))
        self.assertEqual(rows[0]['lost_frames'], '0')
        self.assertEqual(rows[1]['lost_frames'], '2')
        self.assertEqual(rows[1]['cnt'], '60')
        self.assertEqual(rows[1]['Battery'], '8.0')
        self.assertEqual(len(rows[1]), 60)

    def test_legacy_frames_keep_the_original_csv_column_alignment(self):
        output = io.StringIO()
        capture = log.CsvCapture(output)
        wire = frame(2, 100, legacy=True) + frame(3, 120, legacy=True)
        for byte in wire:
            capture.feed(bytes([byte]))
        rows = list(csv.DictReader(io.StringIO(output.getvalue())))
        self.assertEqual(len(rows[0]), 56)
        self.assertEqual(rows[0]['Battery'], '8.0')
        self.assertEqual(rows[0]['sequence'], '2')
        self.assertNotIn('irq.total_us', rows[0])

    def test_csv_rejects_a_format_change_in_the_same_recording(self):
        capture = log.CsvCapture(io.StringIO())
        capture.feed(frame(legacy=True))
        with self.assertRaisesRegex(RuntimeError, 'field count changed'):
            capture.feed(frame(1, 120))

    def test_restart_is_reported_instead_of_becoming_a_huge_gap(self):
        capture = log.CsvCapture(io.StringIO())
        capture.feed(frame(4, 200))
        with self.assertRaises(RuntimeError):
            capture.feed(frame(0, 300))

    def test_csv_labels_match_the_saved_log(self):
        source = (ROOT / 'Module' / 'Src' / 'log_data.cpp').read_text(encoding='utf-8')
        labels = source.split('static const char *labels[(LOG_DATA_NUM+1)] = {')[1].split('};')[0]
        self.assertEqual(tuple(re.findall(r'"([^"]+)"', labels)), ('cnt', *log.LOG_LABELS))

    def test_control_period_limits_and_checksum(self):
        for period in (0, 2, 1001):
            with self.subTest(period=period), self.assertRaises(ValueError):
                log.control_packet(period)
        for period in (3, 4, 5, 10, 20, 1000):
            with self.subTest(period=period):
                packet = log.control_packet(period)
                self.assertEqual(struct.unpack_from('<H', packet, 4)[0], period)
                self.assertEqual(struct.unpack_from('<H', packet, 6)[0],
                                 binascii.crc_hqx(packet[:6], 0xffff))
        packet = log.control_packet(20)
        self.assertEqual(packet[:6], b'\x00L8\x01\x14\x00')
        self.assertEqual(binascii.crc_hqx(b'123456789', 0xffff), 0x29b1)

    def test_short_period_crc_resync_preserves_csv_times_and_missing_frames(self):
        output = io.StringIO()
        capture = log.CsvCapture(output)
        broken = bytearray(frame(period_ms=3))
        broken[25] ^= 1
        wire = broken + frame(0, 100, 3) + frame(1, 103, 3) + frame(3, 109, 3)
        for offset in range(0, len(wire), 17):
            capture.feed(wire[offset:offset + 17])
        rows = list(csv.DictReader(io.StringIO(output.getvalue())))
        self.assertEqual(capture.decoder.crc_errors, 1)
        self.assertEqual([row['period_ms'] for row in rows], ['3', '3', '3'])
        self.assertEqual([row['sequence'] for row in rows], ['0', '1', '3'])
        self.assertEqual([row['time_ms'] for row in rows], ['100', '103', '109'])
        self.assertEqual([row['cnt'] for row in rows], ['0', '3', '9'])
        self.assertEqual([row['lost_frames'] for row in rows], ['0', '0', '1'])
        self.assertEqual(capture.lost, 1)
        self.assertEqual(rows[0]['Battery'], '8.0')
        self.assertTrue(all(len(row) == 60 for row in rows))

    def test_decoder_rejects_below_minimum_and_above_maximum_period(self):
        decoder = log.StreamDecoder()
        samples = decoder.feed(frame(0, 100, 2) + frame(1, 101, 1001) + frame(2, 103, 3))
        self.assertEqual([sample.sequence for sample in samples], [2])
        self.assertEqual(samples[0].period_ms, 3)
        self.assertEqual(decoder.crc_errors, 0)


class FirmwareTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        command = compiler_command()
        if command is None:
            raise unittest.SkipTest('No host C++ compiler; set CXX or install local Zig')
        build_dir = ROOT / 'build'
        build_dir.mkdir(exist_ok=True)
        cls.work = tempfile.TemporaryDirectory(prefix='log-stream-test-', dir=build_dir)
        cls.addClassCleanup(cls.work.cleanup)
        cls.executable = Path(cls.work.name) / ('log-stream.exe' if os.name == 'nt' else 'log-stream')
        environment = os.environ.copy()
        if Path(command[0]).stem == 'zig':
            environment.setdefault('ZIG_LOCAL_CACHE_DIR', str(build_dir / 'ir-confidence-zig-cache'))
            environment.setdefault('ZIG_GLOBAL_CACHE_DIR', str(build_dir / 'ir-confidence-zig-global'))
            if os.name == 'nt':
                command += ['-target', 'x86_64-windows-gnu']
        result = subprocess.run(command + [
            '-std=c++14', '-O0', '-Wall', '-Wextra', '-fno-exceptions', '-fno-rtti',
            '-I', str(ROOT / 'tools/tests/log_stream_stubs'), '-I', str(ROOT),
            str(ROOT / 'tools/tests/log_stream_harness.cpp'),
            str(ROOT / 'Module/Src/log_stream.cpp'), str(ROOT / 'Peripheral/Src/communicate.cpp'),
            str(ROOT / 'Module/Src/remote_explore.cpp'),
            '-o', str(cls.executable),
        ], capture_output=True, env=environment, timeout=240)
        if result.returncode:
            raise AssertionError(result.stdout.decode(errors='replace') + result.stderr.decode(errors='replace'))

    def run_case(self, case, packet=None, period=20):
        result = subprocess.run([str(self.executable), case,
                                 (packet or log.control_packet(period)).hex(),
                                 log.control_packet(period, stop=True).hex(),
                                 log.control_packet(period, action=3).hex(),
                                 log.control_packet(period, action=2).hex()],
                                capture_output=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr.decode(errors='replace'))
        return result.stdout

    def test_real_uart_queue_is_nonblocking_atomic_and_preserves_irq_mask(self):
        self.run_case('queue')

    def test_remote_exploration_guard_arm_stop_timeout_and_link_loss(self):
        self.run_case('explore-guard')

    def test_dma_wrap_preserves_inflight_memory_and_byte_order(self):
        self.run_case('dma-wrap')

    def test_failed_dma_start_is_retried_without_losing_bytes(self):
        self.run_case('dma-retry')

    def test_log_ring_keeps_latest_400_samples_in_order_after_multiple_wraps(self):
        self.run_case('log-ring')

    def test_stop_discards_log_pending_but_drains_frames_already_in_uart_queue(self):
        for period in (3, 4, 5, 10, 20):
            with self.subTest(period=period):
                samples = log.StreamDecoder().feed(self.run_case('stream-stop', period=period))
                self.assertEqual([sample.sequence for sample in samples], [0, 1, 2, 3])

    def test_shell_and_noncontrol_binary_rx_are_preserved(self):
        self.run_case('rx')

    def test_keepalive_extends_the_lease_without_resetting_sequence(self):
        self.run_case('keepalive')

    def test_control_is_received_even_when_the_foreground_rx_queue_is_full(self):
        self.run_case('fullrx')

    def test_firmware_crc_format_drops_lease_stop_and_tick_wrap(self):
        for period in (3, 4, 5, 10, 20):
            for case in ('stream', 'wrap'):
                with self.subTest(case=case, period=period):
                    samples = log.StreamDecoder().feed(self.run_case(case, period=period))
                    self.assertEqual([sample.sequence for sample in samples], list(range(13)) + [14, 15])
                    count = len(log.LOG_LABELS)
                    expected = struct.unpack('<{}e'.format(count),
                                             struct.pack('<{}H'.format(count), *range(0x3c00, 0x3c00 + count)))
                    self.assertEqual(samples[0].values, expected)
                    self.assertTrue(all(sample.period_ms == period for sample in samples))
                    self.assertEqual((samples[1].time_ms - samples[0].time_ms) & 0xffffffff, period)

    def test_firmware_period_deadlines_include_the_minimum_and_maximum(self):
        for period in (3, 4, 5, 10, 20, 1000):
            with self.subTest(period=period):
                samples = log.StreamDecoder().feed(self.run_case('deadline', period=period))
                self.assertEqual([sample.sequence for sample in samples], [0, 1])
                self.assertTrue(all(sample.period_ms == period for sample in samples))
                self.assertEqual((samples[1].time_ms - samples[0].time_ms) & 0xffffffff, period)

    def test_invalid_period_and_corrupt_request_do_not_start_stream(self):
        for period in (0, 2, 1001):
            with self.subTest(period=period):
                bad = struct.pack('<3sBH', b'\x00L8', 1, period)
                bad += struct.pack('<H', binascii.crc_hqx(bad, 0xffff))
                self.run_case('invalid', bad)
        bad = bytearray(log.control_packet(20))
        bad[-1] ^= 1
        self.run_case('invalid', bad)


class FirmwareTimingTests(unittest.TestCase):
    """Virtual time reproduces scheduling loss, not BLE or real CPU performance."""

    @classmethod
    def setUpClass(cls):
        command = compiler_command()
        if command is None:
            raise unittest.SkipTest('No host C++ compiler; set CXX or install local Zig')
        build_dir = ROOT / 'build'
        build_dir.mkdir(exist_ok=True)
        cls.work = tempfile.TemporaryDirectory(prefix='log-timing-test-', dir=build_dir)
        cls.addClassCleanup(cls.work.cleanup)
        work = Path(cls.work.name)
        current = (ROOT / 'Peripheral/Src/communicate.cpp').read_text(encoding='utf-8')
        legacy = current.replace('#define TX_BUFFER_SIZE 512', '#define TX_BUFFER_SIZE 128', 1)
        if legacy == current:
            raise AssertionError('Cannot construct the old 128-byte TX control')
        (work / 'communicate_legacy.cpp').write_text(legacy, encoding='utf-8')
        environment = os.environ.copy()
        if Path(command[0]).stem == 'zig':
            environment.setdefault('ZIG_LOCAL_CACHE_DIR', str(build_dir / 'ir-confidence-zig-cache'))
            environment.setdefault('ZIG_GLOBAL_CACHE_DIR', str(build_dir / 'ir-confidence-zig-global'))
            if os.name == 'nt':
                command += ['-target', 'x86_64-windows-gnu']
        cls.executables = {}
        for capacity in (128, 512):
            executable = work / ('timing-{}{}'.format(capacity, '.exe' if os.name == 'nt' else ''))
            arguments = command + ['-std=c++14', '-O2', '-Wall', '-Wextra',
                                   '-fno-exceptions', '-fno-rtti',
                                   '-I', str(ROOT / 'tools/tests/log_stream_stubs'),
                                   '-I', str(ROOT), '-I', str(work),
                                   str(ROOT / 'tools/tests/log_stream_timing_harness.cpp'),
                                   '-o', str(executable)]
            if capacity == 128:
                arguments.append('-DLEGACY_TX128')
            result = subprocess.run(arguments, capture_output=True, env=environment, timeout=240)
            if result.returncode:
                raise AssertionError(result.stdout.decode(errors='replace') +
                                     result.stderr.decode(errors='replace'))
            cls.executables[capacity] = executable

    def run_model(self, capacity, *, busy=300, period=3, seconds=15, step=25, baud=460800):
        result = subprocess.run([str(self.executables[capacity]), str(seconds), str(busy),
                                 str(period), str(step), str(baud)], capture_output=True,
                                text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr)
        rows = list(csv.DictReader(io.StringIO(result.stdout)))
        self.assertEqual(len(rows), len(range(0, 1018, step)))
        for row in rows:
            self.assertEqual(int(row['crc_errors']), 0)
            self.assertEqual(int(row['gaps']), int(row['dropped']))
            self.assertEqual(int(row['systick_coalesced']), 0)
        return rows

    def test_three_ms_adc_phase_scan_reproduces_old_loss_and_keeps_new_tx_draining(self):
        control = self.run_model(128, busy=0, step=250)
        self.assertTrue(all(int(row['dropped']) == 0 for row in control))
        old = self.run_model(128)
        current = self.run_model(512)
        self.assertTrue(all(int(row['dropped']) > 0 for row in old))
        self.assertTrue(all(int(row['max_pending']) == 8 for row in old))
        self.assertTrue(all(int(row['dropped']) == 0 for row in current))
        self.assertTrue(all(int(row['attempted']) == int(row['received_after_drain'])
                            for row in current))
        self.assertTrue(any(int(row['max_tx_used']) >= 2 * 126 for row in current))

    def test_longer_control_occupancy_preserves_dma_wrap_bytes_and_frame_order(self):
        rows = self.run_model(512, busy=550, step=250)
        self.assertTrue(all(int(row['dropped']) == 0 for row in rows))
        self.assertGreater(sum(int(row['split_transfers']) for row in rows), 0)

    def test_real_uart_bandwidth_overload_still_drops_whole_frames(self):
        rows = self.run_model(512, seconds=3, step=250, baud=230400)
        self.assertTrue(all(int(row['dropped']) > 0 for row in rows))


if __name__ == '__main__':
    unittest.main()
