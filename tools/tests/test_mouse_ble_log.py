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
        for period in (0, 19, 1001):
            with self.assertRaises(ValueError):
                log.control_packet(period)
        packet = log.control_packet(20)
        self.assertEqual(packet[:6], b'\x00L8\x01\x14\x00')
        self.assertEqual(binascii.crc_hqx(b'123456789', 0xffff), 0x29b1)


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

    def run_case(self, case, packet=None):
        result = subprocess.run([str(self.executable), case,
                                 (packet or log.control_packet(20)).hex(),
                                 log.control_packet(20, stop=True).hex(),
                                 log.control_packet(20, action=3).hex(),
                                 log.control_packet(20, action=2).hex()],
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

    def test_stop_discards_prepared_and_queued_frames(self):
        self.run_case('stream-stop')

    def test_shell_and_noncontrol_binary_rx_are_preserved(self):
        self.run_case('rx')

    def test_keepalive_extends_the_lease_without_resetting_sequence(self):
        self.run_case('keepalive')

    def test_control_is_received_even_when_the_foreground_rx_queue_is_full(self):
        self.run_case('fullrx')

    def test_firmware_crc_format_drops_lease_stop_and_tick_wrap(self):
        for case in ('stream', 'wrap'):
            with self.subTest(case=case):
                samples = log.StreamDecoder().feed(self.run_case(case))
                self.assertEqual([sample.sequence for sample in samples], list(range(10)) + [11, 12])
                count = len(log.LOG_LABELS)
                expected = struct.unpack('<{}e'.format(count),
                                         struct.pack('<{}H'.format(count), *range(0x3c00, 0x3c00 + count)))
                self.assertEqual(samples[0].values, expected)
                self.assertEqual((samples[1].time_ms - samples[0].time_ms) & 0xffffffff, 20)

    def test_invalid_period_and_corrupt_request_do_not_start_stream(self):
        bad = b'\x00L8\x01\x01\x00'
        bad += struct.pack('<H', binascii.crc_hqx(bad, 0xffff))
        self.run_case('invalid', bad)
        bad = bytearray(log.control_packet(20))
        bad[-1] ^= 1
        self.run_case('invalid', bad)


if __name__ == '__main__':
    unittest.main()
