"""Exercise the firmware's bounded BLE notification batching policy on the host."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

from test_side_wall_stability import compiler_command

ROOT = Path(__file__).resolve().parents[2]


class BatchPolicyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        command = compiler_command()
        if command is None:
            raise unittest.SkipTest('No host C++ compiler; set CXX or install local Zig')
        build_dir = ROOT / 'build'
        build_dir.mkdir(exist_ok=True)
        cls.work = tempfile.TemporaryDirectory(prefix='hy0020-batch-', dir=build_dir)
        cls.addClassCleanup(cls.work.cleanup)
        cls.executable = Path(cls.work.name) / ('batch.exe' if os.name == 'nt' else 'batch')
        environment = os.environ.copy()
        if Path(command[0]).stem == 'zig':
            environment.setdefault('ZIG_LOCAL_CACHE_DIR', str(build_dir / 'ir-confidence-zig-cache'))
            environment.setdefault('ZIG_GLOBAL_CACHE_DIR', str(build_dir / 'ir-confidence-zig-global'))
            if os.name == 'nt':
                command += ['-target', 'x86_64-windows-gnu']
        result = subprocess.run(command + [
            '-std=c++14', '-O0', '-Wall', '-Wextra', '-fno-exceptions', '-fno-rtti',
            '-I', str(ROOT), str(ROOT / 'tools/tests/hy0020_batch_harness.cpp'),
            '-o', str(cls.executable),
        ], capture_output=True, env=environment, timeout=240)
        if result.returncode:
            raise AssertionError(result.stdout.decode(errors='replace') + result.stderr.decode(errors='replace'))

    def run_case(self, name):
        result = subprocess.run([str(self.executable), name], capture_output=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr.decode(errors='replace'))

    def test_short_ascii_flush_has_a_deadline_despite_new_arrivals(self):
        self.run_case('ascii')

    def test_dma_chunks_fill_payloads_and_stop_flushes_the_final_tail(self):
        self.run_case('stream')

    def test_empty_and_new_session_discard_the_previous_deadline(self):
        self.run_case('reset')

    def test_small_mtu_and_later_mtu_growth_keep_the_flush_bound(self):
        self.run_case('mtu')

    def test_long_uptime_and_session_wrap_keep_the_deadline(self):
        self.run_case('uptime')


if __name__ == '__main__':
    unittest.main()
