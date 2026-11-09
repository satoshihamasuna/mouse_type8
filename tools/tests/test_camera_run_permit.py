"""実際の撮影専用許可をホストC++で検証する。実機は使用しない。"""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

from test_side_wall_stability import compiler_command

ROOT = Path(__file__).resolve().parents[2]


class CameraRunPermitTests(unittest.TestCase):
    def test_one_shot_token_expiry_wrap_and_invalid_arguments(self):
        command = compiler_command()
        if command is None:
            self.skipTest('Host C++ compiler is required')
        environment = os.environ.copy()
        if Path(command[0]).stem == 'zig':
            environment.setdefault('ZIG_LOCAL_CACHE_DIR', str(ROOT / 'build/ir-confidence-zig-cache'))
            environment.setdefault('ZIG_GLOBAL_CACHE_DIR', str(ROOT / 'build/ir-confidence-zig-global'))
            if os.name == 'nt':
                command += ['-target', 'x86_64-windows-gnu']
        with tempfile.TemporaryDirectory(prefix='camera-permit-', dir=ROOT / 'build') as directory:
            executable = Path(directory) / ('permit.exe' if os.name == 'nt' else 'permit')
            result = subprocess.run(command + [
                '-std=c++14', '-Wall', '-Wextra', '-fno-exceptions', '-fno-rtti',
                '-I', str(ROOT), str(ROOT / 'tools/tests/camera_run_permit_harness.cpp'),
                '-o', str(executable),
            ], capture_output=True, env=environment, timeout=240)
            self.assertEqual(result.returncode, 0, result.stderr.decode(errors='replace'))
            result = subprocess.run([str(executable)], capture_output=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stderr.decode(errors='replace'))


if __name__ == '__main__':
    unittest.main()
