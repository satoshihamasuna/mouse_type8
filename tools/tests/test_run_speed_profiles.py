"""実calcRunTime・走行呼出し・モーション初期化の速度条件をA/Bで確認する。"""
import os
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

from test_side_wall_stability import compiler_command, parameter_sources

ROOT = Path(__file__).resolve().parents[2]


class RunSpeedProfileTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        command = compiler_command()
        if command is None:
            raise unittest.SkipTest('Host C++ compiler is required')
        cls.work = tempfile.TemporaryDirectory(prefix='run-speed-', dir=ROOT/'build')
        cls.addClassCleanup(cls.work.cleanup)
        folder = Path(cls.work.name)
        environment = os.environ.copy()
        if Path(command[0]).stem == 'zig':
            environment.setdefault('ZIG_LOCAL_CACHE_DIR', str(ROOT/'build/ir-confidence-zig-cache'))
            environment.setdefault('ZIG_GLOBAL_CACHE_DIR', str(ROOT/'build/ir-confidence-zig-global'))
            if os.name == 'nt':
                command += ['-target', 'x86_64-windows-gnu']
        header = (ROOT/'Subsys/Inc/make_path.h').read_text(encoding='utf-8')
        begin = header.index('class calcRunTime\n')
        end = header.index('\nclass Dijkstra:', begin)
        stub = folder/'Subsys/Inc/make_path.h'
        stub.parent.mkdir(parents=True)
        stub.write_text('#pragma once\n#include "Params/run_config.h"\n' + header[begin:end], encoding='utf-8')
        for name, content in {
            'Module/Inc/vehicle.h': '#pragma once\n#include "Task/Inc/run_typedef.h"\n',
            'Subsys/Inc/wall_class.h': '#pragma once\n',
        }.items():
            path = folder/name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content, encoding='utf-8')
        # 実宣言・inlineラッパー・初期化の距離計算を使用。ハードウェア状態遷移は模擬しない。
        ctrl = (ROOT/'Task/Inc/ctrl_task.h').read_text(encoding='utf-8')
        init = (ROOT/'Task/Src/init_motion.cpp').read_text(encoding='utf-8')
        declarations, definitions, wrappers = [], [], []
        for kind in ('straight', 'diagonal'):
            marker = 'void Init_Motion_' + kind
            start = ctrl.index(marker)
            declarations.append(ctrl[start:ctrl.index(';', start)+1])
            marker = 'inline t_exeStatus exe_Motion_' + kind
            start = ctrl.index(marker)
            wrappers.append(ctrl[start:ctrl.index('\n\t\t}', start)+4])
            start = init.index('void Motion::Init_Motion_' + kind)
            brace = init.index('{', start)
            prefix = init.index('motion_plan.velo.set', brace)
            stop = init.index('\n\tmotion_plan.rad_accel.init', prefix)
            definitions.append(init[start:brace+1] + '\n' + init[prefix:stop]
                               + '\nlast_sp=sp_gain;last_om=om_gain;last_ff=ff_gain;\n}\n')
        (folder/'motion_declarations.inc').write_text('\n'.join(declarations+wrappers), encoding='utf-8')
        (folder/'motion_initialization.inc').write_text('\n'.join(definitions), encoding='utf-8')
        # 全速度設定を保持済みの機体別校正表と照合する。
        names = re.findall(r'extern const t_straight_param \*const ((?:st|di)_mode_\w+)\[(\d+)\];',
                           (ROOT/'Params/Param_A/run_param_A.h').read_text(encoding='utf-8'))
        assert len(names) == 38
        cases = []
        for name, count in names:
            kind = 'straight' if name.startswith('st_') else 'diagonal'
            suffix = name.split('_mode_',1)[1]
            profile = kind+'_speed_'+('plain500' if suffix == '500_v0' else suffix)
            cases.append('{&'+profile+','+name+','+count+'},')
        (folder/'mode_profile_cases.inc').write_text('\n'.join(cases),encoding='utf-8')
        run = (ROOT/'Subsys/Src/make_path.cpp').read_text(encoding='utf-8')
        constant = next(line for line in run.splitlines() if line.startswith('static constexpr float RUN_START_LENGTH_MM'))
        (folder/'run_start_constants.inc').write_text(constant+'\n', encoding='utf-8')
        begin = run.index('void Dijkstra::run_Dijkstra(')
        for kind, case in (('straight', 'Straight'), ('diagonal', 'Diagonal')):
            section = run.index('case '+case+':', begin)
            start = run.index('section_end = ', section)
            stop = run.index('\n\t\t\t\tbreak;', start)
            (folder/('run_'+kind+'.inc')).write_text(run[start:stop], encoding='utf-8')
        for method, label in (('run_Dijkstra_suction','suction'), ('run_Dijkstra_suction_acc','variable')):
            begin = run.index('void Dijkstra::'+method+'(')
            for kind, case in (('straight','Straight'),('diagonal','Diagonal')):
                section = run.index('case '+case+':',begin)
                start = run.index('section_end = ',section)
                stop = run.index('\n\t\t\t\tbreak;',start)
                (folder/('run_'+label+'_'+kind+'.inc')).write_text(run[start:stop],encoding='utf-8')
        cls.executables = {}
        for mouse in ('A', 'B'):
            executable = folder/('speed-'+mouse+('.exe' if os.name == 'nt' else ''))
            result = subprocess.run(command + ['-std=c++14', '-O0', '-Wall', '-Wextra',
                '-fno-exceptions', '-fno-rtti', '-DMOUSE_'+mouse,
                '-I', str(folder), '-I', str(ROOT),
                str(ROOT/'tools/tests/run_speed_profiles_harness.cpp'),
                *parameter_sources(mouse), '-o', str(executable)],
                capture_output=True, text=True, errors='replace', env=environment, timeout=240)
            if result.returncode:
                raise AssertionError('Mouse '+mouse+':\n'+result.stdout+result.stderr)
            cls.executables[mouse] = executable

    def run_case(self, case):
        for mouse, executable in self.executables.items():
            with self.subTest(mouse=mouse):
                result = subprocess.run([str(executable), case], capture_output=True,
                                        text=True, errors='replace', timeout=20)
                self.assertEqual(result.returncode, 0, result.stdout+result.stderr)

    def test_current_profiles_and_machine_specific_control_bands(self):
        self.run_case('current')

    def test_changed_base_step_and_nonmultiple_max_generate_candidates(self):
        self.run_case('generation')

    def test_straight_and_diagonal_can_be_changed_independently(self):
        self.run_case('independent')

    def test_asymmetric_braking_matches_analytic_time_and_initialization(self):
        self.run_case('braking')

    def test_short_sections_stops_and_unreachable_endpoints(self):
        self.run_case('short')

    def test_invalid_profiles_and_nonfinite_inputs_are_rejected(self):
        self.run_case('invalid')

    def test_switching_to_legacy_restores_explicit_candidate_policy(self):
        self.run_case('legacy')

    def test_base_and_diagonal_queries_preserve_straight_selected_values(self):
        self.run_case('lifetime')

    def test_actual_run_dispatch_forwards_next_turn_speed_gains_and_braking(self):
        self.run_case('dispatch')

    def test_all_38_profiles_reproduce_the_original_354_candidates(self):
        self.run_case('all-profiles')

    def test_segmented_candidates_apply_their_own_acceleration_and_braking(self):
        self.run_case('segmented-physics')

    def test_overlapping_unordered_and_inconsistent_ranges_are_rejected(self):
        self.run_case('invalid-ranges')

    def test_actual_suction_and_variable_turn_dispatch_use_profile_rates(self):
        self.run_case('high-speed-dispatch')


if __name__ == '__main__':
    unittest.main()
