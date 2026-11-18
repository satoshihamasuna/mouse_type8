"""実ファームのローカル/全体座標と、IRQ・モード・シェルの接続を検証する。"""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

from test_side_wall_stability import ROOT, compiler_command


def function(text, signature):
    begin = text.index(signature)
    opening = text.index('{', begin)
    depth = 1
    end = opening + 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[begin:end]


class PoseFramesTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = compiler_command()
        if compiler is None:
            raise unittest.SkipTest('Host C++ compiler is required')
        environment = os.environ.copy()
        if Path(compiler[0]).stem == 'zig':
            environment.setdefault('ZIG_LOCAL_CACHE_DIR', str(ROOT / 'build/ir-confidence-zig-cache'))
            environment.setdefault('ZIG_GLOBAL_CACHE_DIR', str(ROOT / 'build/ir-confidence-zig-global'))
            if os.name == 'nt':
                compiler += ['-target', 'x86_64-windows-gnu']
        cls.work = tempfile.TemporaryDirectory(prefix='pose-frames-', dir=ROOT / 'build')
        cls.addClassCleanup(cls.work.cleanup)
        directory = Path(cls.work.name)
        peripheral = directory / 'Peripheral/Inc/peripheral.h'
        peripheral.parent.mkdir(parents=True)
        peripheral.write_text('''#pragma once
#include <stdint.h>
inline void Motor_SetDuty_Right(int) {}
inline void Motor_SetDuty_Left(int) {}
inline void FAN_Motor_SetDuty(int) {}
''', encoding='utf-8')
        header = (ROOT / 'Task/Inc/ctrl_task.h').read_text(encoding='utf-8')
        (directory / 'pose_setters.inc').write_text('\n'.join(function(header, signature) for signature in
            ('inline void motion_control_set(', 'inline void motion_exeStatus_set(')), encoding='utf-8')
        shell = (ROOT / 'System/Src/myshell.cpp').read_text(encoding='utf-8')
        (directory / 'pose_shell.inc').write_text(function(shell, 'static int usrcmd_pose(int argc, char **argv)\n{'), encoding='utf-8')
        irq = (ROOT / 'Module/Src/interrupt.cpp').read_text(encoding='utf-8')
        begin = irq.index('auto &frames=vehicle.pose;')
        end = irq.index('else frames.predict_body(', begin)
        end = irq.index(';', end) + 1
        (directory / 'pose_prediction.inc').write_text(irq[begin:end], encoding='utf-8')
        logger = (ROOT / 'Module/Src/log_data.cpp').read_text(encoding='utf-8')
        begin = logger.index('const auto turn_position =')
        end = logger.index('sample[15] =', begin)
        end = logger.index(';', end) + 1
        (directory / 'pose_log.inc').write_text(logger[begin:end], encoding='utf-8')
        init = (ROOT / 'Task/Src/init_motion.cpp').read_text(encoding='utf-8')
        for method, operation in [('Motion_start', 'start'), ('Motion_end', 'stop'), ('Motion_error_handling', 'stop')]:
            source = function(init, 'void Motion::' + method + '()')
            call = 'vehicle->pose.' + operation + '();'
            # The coordinate lifecycle must precede the legacy local resets.
            assert source.index(call) < source.index('vehicle->ego_initialize();')
            (directory / (method + '_pose.inc')).write_text(call, encoding='utf-8')
        cls.executables = {}
        for mouse in ('A', 'B'):
            exe = directory / ('pose-' + mouse + ('.exe' if os.name == 'nt' else ''))
            result = subprocess.run(compiler + ['-std=c++14', '-O0', '-Wall', '-Wextra', '-fno-exceptions', '-fno-rtti',
                '-DMOUSE_' + mouse, '-I', str(directory), '-I', str(ROOT),
                str(ROOT / 'tools/tests/pose_frames_harness.cpp'), '-o', str(exe)],
                capture_output=True, text=True, errors='replace', env=environment, timeout=240)
            if result.returncode:
                raise AssertionError(result.stdout + result.stderr)
            cls.executables[mouse] = exe

    def run_case(self, case):
        for mouse, executable in self.executables.items():
            with self.subTest(mouse=mouse):
                result = subprocess.run([str(executable), case], capture_output=True, text=True, errors='replace', timeout=15)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_se2_roundtrip_and_covariance_cross_correlations(self):
        self.run_case('geometry')

    def test_world_continuity_closed_route_diagonal_reverse_and_pivot(self):
        self.run_case('route')

    def test_actual_motion_setters_stop_error_and_next_start(self):
        self.run_case('lifecycle')

    def test_each_vehicle_owns_its_pose_and_legacy_resets_preserve_coordinates(self):
        self.run_case('ownership')

    def test_log_positions_derive_from_single_pose_and_preserve_axes_and_column_slots(self):
        self.run_case('log-view')

    def test_actual_irq_adopts_turn_increment_once_in_the_correct_frame(self):
        self.run_case('irq')

    def test_shell_origin_units_busy_invalid_values_and_irq_mask(self):
        self.run_case('shell')


if __name__ == '__main__':
    unittest.main()
