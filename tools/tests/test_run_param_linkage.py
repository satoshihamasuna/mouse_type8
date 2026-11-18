"""機体別パラメータの実体・既定FFが翻訳単位をまたいで共有されることを確認する。"""
import os
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

from test_side_wall_stability import compiler_command, parameter_sources


ROOT = Path(__file__).resolve().parents[2]


class RunParamLinkageTests(unittest.TestCase):
    def test_all_exported_parameters_share_addresses_between_translation_units(self):
        command = compiler_command()
        if command is None:
            self.skipTest('Host C++ compiler is required')
        environment = os.environ.copy()
        if Path(command[0]).stem == 'zig':
            environment.setdefault('ZIG_LOCAL_CACHE_DIR', str(ROOT/'build/ir-confidence-zig-cache'))
            environment.setdefault('ZIG_GLOBAL_CACHE_DIR', str(ROOT/'build/ir-confidence-zig-global'))
            if os.name == 'nt':
                command += ['-target', 'x86_64-windows-gnu']
        with tempfile.TemporaryDirectory(prefix='param-linkage-', dir=ROOT/'build') as directory:
            folder = Path(directory)
            stub = folder/'Module/Inc/vehicle.h'
            stub.parent.mkdir(parents=True)
            stub.write_text('#pragma once\n#include "Task/Inc/run_typedef.h"\n', encoding='utf-8')
            for mouse in ('A', 'B'):
                with self.subTest(mouse=mouse):
                    headers = list((ROOT/'Params'/('Param_'+mouse)).rglob('*.h'))
                    headers.append(ROOT/'Params/typedef_run_param.h')
                    names = set()
                    for header in headers:
                        text = header.read_text(encoding='utf-8')
                        for match in re.finditer(r'^extern const [^;]+?\b(\w+)\s*(?:\[\d+\])?;', text, re.M):
                            names.add(match[1])
                    self.assertTrue({'ff_gain_default', 'basic_sp_gain', 'basic_om_gain', 'mode_500'} <= names)
                    addresses = ',\n'.join('&'+name for name in sorted(names))
                    prefix = '#include <cstddef>\n#include <cassert>\n#include "Params/run_param.h"\n'
                    peer = folder/'peer.cpp'
                    peer.write_text(prefix + 'const void* const peer_values[]={\n'+addresses+'\n};\n'
                                    'const void* peer_address(unsigned i){return peer_values[i];}\n'
                                    'unsigned peer_count(){return sizeof(peer_values)/sizeof(peer_values[0]);}\n'
                                    'const t_ff_gain* peer_default_ff(){t_param p{};return p.ff_gain;}\n', encoding='utf-8')
                    main = folder/'main.cpp'
                    main.write_text(prefix + 'const void* const values[]={\n'+addresses+'\n};\n'
                                    'const void* peer_address(unsigned);unsigned peer_count();\n'
                                    'const t_ff_gain* peer_default_ff();\n'
                                    'int main(){assert(peer_count()==sizeof(values)/sizeof(values[0]));\n'
                                    'for(unsigned i=0;i<peer_count();++i)assert(values[i]==peer_address(i));\n'
                                    'assert(peer_default_ff()==&ff_gain_default);\n'
                                    'assert(mode_500[Long_turnR90]==&param_R90_500);\n'
                                    'assert(mode_500[Long_turnL90]==&param_L90_500);}\n', encoding='utf-8')
                    executable = folder/('check'+mouse+('.exe' if os.name=='nt' else ''))
                    result = subprocess.run(command + ['-std=c++14', '-O0', '-Wall', '-Wextra',
                        '-fno-exceptions', '-fno-rtti', '-DMOUSE_'+mouse,
                        '-I', str(folder), '-I', str(ROOT), str(main), str(peer),
                        *parameter_sources(mouse), '-o', str(executable)],
                        capture_output=True, text=True, errors='replace', env=environment, timeout=240)
                    self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
                    result = subprocess.run([str(executable)], capture_output=True, text=True, errors='replace', timeout=15)
                    self.assertEqual(result.returncode, 0, result.stdout+result.stderr)


if __name__ == '__main__':
    unittest.main()
