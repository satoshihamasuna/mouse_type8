"""実Dijkstra走行処理の指令・吸引・ログ・中止順序をA/Bで確認する。"""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

from test_side_wall_stability import compiler_command, parameter_sources
from test_suction_start import function_source

ROOT = Path(__file__).resolve().parents[2]
CASES = ('profiles', 'turns', 'combined', 'combined-missing', 'initial-selection',
         'cancel', 'invalid', 'unreachable', 'motion-error', 'initial-error', 'long-route')


class RunDijkstraTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        command = compiler_command()
        if command is None:
            raise unittest.SkipTest('Host C++ compiler is required')
        cls.work = tempfile.TemporaryDirectory(prefix='run-dijkstra-', dir=ROOT/'build')
        cls.addClassCleanup(cls.work.cleanup)
        folder = Path(cls.work.name)
        environment = os.environ.copy()
        if Path(command[0]).stem == 'zig':
            environment.setdefault('ZIG_LOCAL_CACHE_DIR', str(ROOT/'build/ir-confidence-zig-cache'))
            environment.setdefault('ZIG_GLOBAL_CACHE_DIR', str(ROOT/'build/ir-confidence-zig-global'))
            if os.name == 'nt':
                command += ['-target', 'x86_64-windows-gnu']
        header = (ROOT/'Subsys/Inc/make_path.h').read_text(encoding='utf-8')
        stub = folder/'Subsys/Inc/make_path.h'
        stub.parent.mkdir(parents=True)
        stub.write_text('#pragma once\n#include "Params/run_config.h"\n'
            + header[header.index('#define DIJKSTRA_PATH_MAX'):header.index('\nclass Dijkstra:')], encoding='utf-8')
        vehicle = folder/'Module/Inc/vehicle.h'
        vehicle.parent.mkdir(parents=True)
        vehicle.write_text('#pragma once\n#include "Task/Inc/run_typedef.h"\n', encoding='utf-8')
        (folder/'Subsys/Inc/wall_class.h').write_text('#pragma once\n', encoding='utf-8')
        source = (ROOT/'Subsys/Src/make_path.cpp').read_text(encoding='utf-8')
        support = source[source.index('static constexpr float RUN_START_LENGTH_MM'):source.index('#define DIJKSTRA_MAX_TIME')]
        (folder/'run_support.inc').write_text(support, encoding='utf-8')
        methods = ['run_Dijkstra', 'run_Dijkstra_suction', 'run_Dijkstra_suction_acc']
        if 'void Dijkstra::run_Dijkstra_common(' in source:
            methods.append('run_Dijkstra_common')
        (folder/'run_methods.inc').write_text('\n'.join(function_source(source, 'void Dijkstra::'+name+'(')
            for name in methods), encoding='utf-8')
        cls.executables = {}
        for mouse in 'AB':
            executable = folder/('dijkstra-'+mouse+('.exe' if os.name == 'nt' else ''))
            result = subprocess.run(command+['-std=c++14', '-O0', '-Wall', '-Wextra',
                '-fno-exceptions', '-fno-rtti', '-DMOUSE_'+mouse, '-I', str(folder), '-I', str(ROOT),
                str(ROOT/'tools/tests/run_dijkstra_harness.cpp'), *parameter_sources(mouse), '-o', str(executable)],
                capture_output=True, text=True, errors='replace', env=environment, timeout=240)
            if result.returncode:
                raise AssertionError('Mouse '+mouse+':\n'+result.stdout+result.stderr)
            cls.executables[mouse] = executable

    def test_run_commands_and_lifecycle(self):
        for mouse, executable in self.executables.items():
            for case in CASES:
                with self.subTest(mouse=mouse, case=case):
                    result = subprocess.run([str(executable), case], capture_output=True,
                        text=True, errors='replace', timeout=20)
                    self.assertEqual(result.returncode, 0, result.stdout+result.stderr)


if __name__ == '__main__':
    unittest.main()
