"""実際の割り込み時間集計と、新旧保存ログのフレーム長を検証する。"""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from test_side_wall_stability import compiler_command

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
from log_protocol import saved_log_frame


class InterruptTimingTests(unittest.TestCase):
    def test_cycles_wrap_previous_sample_maximum_reset_and_unavailable_counter(self):
        command = compiler_command()
        if command is None:
            self.skipTest('Host C++ compiler is required')
        environment = os.environ.copy()
        if Path(command[0]).stem == 'zig':
            environment.setdefault('ZIG_LOCAL_CACHE_DIR', str(ROOT / 'build/ir-confidence-zig-cache'))
            environment.setdefault('ZIG_GLOBAL_CACHE_DIR', str(ROOT / 'build/ir-confidence-zig-global'))
            if os.name == 'nt':
                command += ['-target', 'x86_64-windows-gnu']
        with tempfile.TemporaryDirectory(prefix='interrupt-timing-', dir=ROOT / 'build') as directory:
            executable = Path(directory) / ('timing.exe' if os.name == 'nt' else 'timing')
            result = subprocess.run(command + [
                '-std=c++14', '-Wall', '-Wextra', '-fno-exceptions', '-fno-rtti',
                '-I', str(ROOT / 'tools/tests/interrupt_timing_stubs'),
                '-I', str(ROOT / 'tools/tests/log_stream_stubs'), '-I', str(ROOT),
                str(ROOT / 'Module/Src/interrupt_timing.cpp'),
                str(ROOT / 'tools/tests/interrupt_timing_harness.cpp'), '-o', str(executable),
            ], capture_output=True, env=environment, timeout=240)
            self.assertEqual(result.returncode, 0, result.stderr.decode(errors='replace'))
            result = subprocess.run([str(executable)], capture_output=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stderr.decode(errors='replace'))

    def test_saved_log_header_selects_old_and_new_frame_lengths(self):
        for count, size in ((51, 106), (55, 114)):
            frame = saved_log_frame(['cnt'] + ['field'] * count)
            self.assertEqual(frame.size, size)
            packet = frame.pack(0xa55a, 7, *range(count))
            self.assertEqual(frame.unpack(packet)[2:], tuple(range(count)))
        for labels in (None, [], ['field'] * 56, ['cnt', 'field']):
            with self.assertRaises(ValueError):
                saved_log_frame(labels)


if __name__ == '__main__':
    unittest.main()
