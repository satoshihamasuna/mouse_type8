"""未接続の角加速度ベース壁制御を、合成運動で検証する。"""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

from test_side_wall_stability import ROOT, compiler_command


class StraightWallAccelControlTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = compiler_command()
        if compiler is None:
            raise unittest.SkipTest('Host C++ compiler is required')
        build = ROOT / 'build'
        build.mkdir(exist_ok=True)
        environment = os.environ.copy()
        if Path(compiler[0]).stem == 'zig':
            environment.setdefault('ZIG_LOCAL_CACHE_DIR', str(build / 'ir-confidence-zig-cache'))
            environment.setdefault('ZIG_GLOBAL_CACHE_DIR', str(build / 'ir-confidence-zig-global'))
            if os.name == 'nt':
                compiler += ['-target', 'x86_64-windows-gnu']
        cls.work = tempfile.TemporaryDirectory(prefix='straight-wall-accel-', dir=build)
        cls.addClassCleanup(cls.work.cleanup)
        cls.compiler, cls.environment = compiler, environment
        cls.executables = {}
        for mouse in ('A', 'B'):
            executable = Path(cls.work.name) / ('control-' + mouse + ('.exe' if os.name == 'nt' else ''))
            result = subprocess.run(compiler + ['-std=c++14', '-O0', '-Wall', '-Wextra', '-Werror',
                '-fno-exceptions', '-fno-rtti', '-DMOUSE_' + mouse, '-I', str(ROOT),
                str(ROOT / 'tools/tests/straight_wall_accel_control_harness.cpp'),
                str(ROOT / 'Task/Src/straight_wall_control.cpp'), '-o', str(executable)],
                capture_output=True, text=True, errors='replace', env=environment, timeout=240)
            if result.returncode:
                raise AssertionError(result.stdout + result.stderr)
            cls.executables[mouse] = executable

    def run_case(self, case):
        for mouse, executable in self.executables.items():
            with self.subTest(mouse=mouse):
                result = subprocess.run([str(executable), case], capture_output=True, text=True,
                                        errors='replace', timeout=15)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_nonlinear_error_dynamics_during_acceleration_and_deceleration(self):
        self.run_case('model')

    def test_measured_gyro_command_seed_and_fixed_configuration(self):
        self.run_case('feedback')

    def test_mirrored_limits_command_consistency_and_saturation_release(self):
        self.run_case('limits')

    def test_disable_low_speed_and_explicit_command_reset(self):
        self.run_case('lifecycle')

    def test_nonfinite_overflow_invalid_period_and_unset_parameters(self):
        self.run_case('invalid')

    def test_closed_loop_acceleration_cruise_deceleration_and_sample_intervals(self):
        self.run_case('trajectory')

    def test_straight_wall_adapter_preserves_wall_quality_phase_and_output_units(self):
        self.run_case('adapter-observations')

    def test_straight_wall_adapter_blends_and_returns_to_fallback_with_actual_previous_command(self):
        self.run_case('adapter-handover')

    def test_straight_wall_adapter_reset_missing_kinematics_and_unset_configuration(self):
        self.run_case('adapter-guards')

    def test_actual_ir_task_compiles_with_test_only_type_substitution(self):
        # 型の置換は一時ヘッダーだけで行い、実走行ソース・採用パラメータは編集しない。
        shim = Path(self.work.name) / 'replace_wall_control.h'
        shim.write_text('#include "Task/Inc/straight_wall_accel_adapter.h"\n'
                        '#define StraightWallControl StraightWallAccelAdapter\n', encoding='utf-8')
        for mouse in ('A', 'B'):
            with self.subTest(mouse=mouse):
                result = subprocess.run(self.compiler + ['-std=c++14', '-O0', '-fno-exceptions',
                    '-fno-rtti', '-DMOUSE_' + mouse, '-I', str(ROOT / 'tools/tests/side_wall_stubs'),
                    '-I', str(ROOT), '-include', str(shim), '-c', str(ROOT / 'Task/Src/ir_sens_task.cpp'),
                    '-o', str(Path(self.work.name) / ('ir-' + mouse + '.o'))], capture_output=True,
                    text=True, errors='replace', env=self.environment, timeout=240)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == '__main__':
    unittest.main()
