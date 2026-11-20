"""実demoメニューをホスト実行し、項目と制御モードの切替を確認する。"""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

from test_side_wall_stability import compiler_command

ROOT = Path(__file__).resolve().parents[2]


class DemoModesTest(unittest.TestCase):
    def test_all_menu_items_modes_errors_and_restoration(self):
        command = compiler_command()
        if command is None:
            self.skipTest('Host C++ compiler is required')
        build = ROOT / 'build'
        build.mkdir(exist_ok=True)
        environment = os.environ.copy()
        if Path(command[0]).stem == 'zig':
            environment.setdefault('ZIG_LOCAL_CACHE_DIR', str(build / 'ir-confidence-zig-cache'))
            environment.setdefault('ZIG_GLOBAL_CACHE_DIR', str(build / 'ir-confidence-zig-global'))
            if os.name == 'nt':
                command += ['-target', 'x86_64-windows-gnu']
        with tempfile.TemporaryDirectory(prefix='demo-modes-', dir=build) as directory:
            work = Path(directory)
            # 実メニューと両エントリを使い、走行・Flash・エンコーダだけ模擬する。
            source = (ROOT / 'System/Src/demo.cpp').read_text(encoding='utf-8')
            (work / 'demo_menu.inc').write_text(source[source.index('#define ENABLE'):], encoding='utf-8')
            source = (ROOT / 'System/Src/demo2.cpp').read_text(encoding='utf-8')
            (work / 'demo2_entry.inc').write_text(source[source.index('namespace Mode'):], encoding='utf-8')
            header = (ROOT / 'System/Inc/demo_util.h').read_text(encoding='utf-8')
            (work / 'demo_util.inc').write_text(header[header.index('namespace DemoUtil'):header.index('#endif')], encoding='utf-8')
            for mouse in ('A', 'B'):
                executable = work / ('demo-' + mouse + ('.exe' if os.name == 'nt' else ''))
                invocations = [command + ['-std=c++14', '-Wall', '-Wextra', '-fno-exceptions',
                    '-fno-rtti', '-DMOUSE_' + mouse, '-I', str(work),
                    str(ROOT / 'tools/tests/demo_modes_harness.cpp'), '-o', str(executable)],
                    [str(executable)]]
                for invocation in invocations:
                    with self.subTest(mouse=mouse, executable=invocation[0]):
                        result = subprocess.run(invocation, capture_output=True, text=True,
                            errors='replace', env=environment, timeout=240)
                        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == '__main__':
    unittest.main()
