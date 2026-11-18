"""実際のファーム内ターンEKFをA/B両定義でホストコンパイルする。"""
from pathlib import Path
import os
import subprocess
import tempfile
import unittest

from test_side_wall_stability import compiler_command,ROOT


def build_harness(directory,mouse):
    compiler=compiler_command()
    if compiler is None:
        raise unittest.SkipTest("No host C++ compiler")
    environment=os.environ.copy()
    if Path(compiler[0]).stem=='zig':
        compiler += ['-target','x86_64-windows-gnu'] if os.name=='nt' else []
        environment.setdefault('ZIG_LOCAL_CACHE_DIR',str(ROOT/'build/ir-confidence-zig-cache'))
        environment.setdefault('ZIG_GLOBAL_CACHE_DIR',str(ROOT/'build/ir-confidence-zig-global'))
    executable=Path(directory)/('turn-pose-'+mouse+('.exe' if os.name=='nt' else ''))
    shell=(ROOT/'System/Src/myshell.cpp').read_text(encoding='utf-8')
    begin=shell.index('static int usrcmd_turn_ekf(int argc, char **argv)\n{')
    end=shell.index('\n}\n',begin)+3
    (Path(directory)/'turn_pose_shell.inc').write_text(shell[begin:end],encoding='utf-8')
    invocation=compiler+['-std=c++14','-O1','-Wall','-Wextra','-fno-exceptions','-fno-rtti',
                         '-DMOUSE_'+mouse,'-I',str(ROOT),'-I',str(directory),str(ROOT/'tools/tests/turn_pose_ekf_harness.cpp'),'-o',str(executable)]
    result=subprocess.run(invocation,capture_output=True,text=True,errors='replace',env=environment,timeout=240)
    if result.returncode:
        raise AssertionError(result.stdout+result.stderr)
    return executable


class TurnPoseFirmwareTests(unittest.TestCase):
    def test_real_filter_gates_profiles_modes_resets_and_faults_for_A_and_B(self):
        with tempfile.TemporaryDirectory(prefix='turn-pose-',dir=ROOT/'build') as directory:
            for mouse in ('A','B'):
                with self.subTest(mouse=mouse):
                    executable=build_harness(directory,mouse)
                    result=subprocess.run([str(executable)],capture_output=True,text=True,errors='replace',timeout=30)
                    self.assertEqual(result.returncode,0,result.stdout+result.stderr)
                    self.assertIn('checks passed',result.stdout)


if __name__=='__main__':
    unittest.main()
