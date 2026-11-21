"""生成ELFの左右135退出値・実参照と、今回の変更範囲を確認する。"""
from pathlib import Path
import difflib
import hashlib
import json
import math
import re
import struct
import subprocess

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'tools/turn_analysis/20261121_A_turn1800_OUT135_adoption'
ELF = ROOT / 'build/Debug-MouseA/mouse_type8.elf'
cache = (ELF.parent / 'CMakeCache.txt').read_text(encoding='utf-8')
assert re.search(r'^MOUSE_TARGET:STRING=A$', cache, re.M)
ar = re.search(r'^CMAKE_C_COMPILER_AR:FILEPATH=(.+)$', cache, re.M)[1].strip()
nm = Path(ar).with_name('arm-none-eabi-nm.exe')
symbols = {}
for line in subprocess.check_output([str(nm), '-S', '--defined-only', '--demangle', str(ELF)], text=True).splitlines():
    fields = line.split(maxsplit=3)
    if len(fields) == 4:
        try:
            symbols[fields[3]] = (int(fields[0], 16), int(fields[1], 16))
        except ValueError:
            pass
data = ELF.read_bytes()
assert data[:6] == b'\x7fELF\x01\x01'
phoff = struct.unpack_from('<I', data, 28)[0]
phsize, phcount = struct.unpack_from('<HH', data, 42)
segments = [struct.unpack_from('<IIIIIIII', data, phoff + index*phsize) for index in range(phcount)]

def contents(name):
    address, size = symbols[name]
    for kind, offset, vaddr, _, filesz, _, _, _ in segments:
        if kind == 1 and vaddr <= address and address+size <= vaddr+filesz:
            return data[offset+address-vaddr:offset+address-vaddr+size]
    raise AssertionError((name, hex(address), size))

rows = []
modes = ('mode_1800','mode_1800_acc_v1','mode_1800_acc_v2')
for side, sign, turn_dir in (('R',-1,1),('L',1,2)):
    table_name = f'slalom_out{side}135_1800_table'
    payload = contents(table_name)
    assert len(payload) == 28
    values = struct.unpack('<fffffif', payload)
    velo, radius, lstart, lend, degree, direction, correction = values
    expected = (1.8,sign*41.,0.,35.2,sign*135.,turn_dir,-sign*2.1)
    assert all(math.isclose(have,want,abs_tol=1e-5) for have,want in zip(values,expected)), (side,values)
    parameter_name = f'param_out{side}135_1800'
    pointers = struct.unpack('<6I', contents(parameter_name))
    names = [table_name,'sp_gain_turnOut135_1800','om_gain_turnOut135_1800',f'ff_gain_out135_{side}_1800']
    assert list(pointers[:4]) == [symbols[name][0] for name in names]
    assert math.isclose(degree+correction,sign*132.9,abs_tol=1e-5)
    for mode in modes:
        mode_values = struct.unpack('<'+'I'*(len(contents(mode))//4),contents(mode))
        assert mode_values[13 if side == 'R' else 14] == symbols[parameter_name][0], (mode,side)
    rows.append(dict(side=side,velocity_m_s=velo,radius_mm=radius,Lstart_mm=lstart,Lend_mm=lend,
        logical_degree=degree,degree_correction=correction,physical_degree=degree+correction,
        direction_enum=direction,table_address=hex(symbols[table_name][0]),
        parameter_address=hex(symbols[parameter_name][0]),gain_references=names[1:]))

files = ['Params/Param_A/turn_1800/turn_1800.cpp','Params/Param_A/turn_1800/turn_1800.h','README.md']
texts, diff = {}, []
for name in files:
    before = (OUT/'before'/name).read_text(encoding='utf-8')
    after = (ROOT/name).read_text(encoding='utf-8')
    texts[name] = before,after
    diff.extend(difflib.unified_diff(before.splitlines(True),after.splitlines(True),
        fromfile='before/'+name,tofile='after/'+name,n=3))
before, after = texts[files[0]]
start = '//k = 250\nconst t_pid_gain sp_gain_turnOut135_1800'
end = 'const t_param *const mode_1800[]'
assert before[:before.index(start)] == after[:after.index(start)]
assert before[before.index(end):] == after[after.index(end):]
assert '#if' not in after[after.index(start):after.index(end)]
for kind in ('t_pid_gain','t_ff_gain'):
    pattern = r'const '+kind+r'\s+\w+\s*=\s*\{[^}]*\};'
    assert re.findall(pattern,before) == re.findall(pattern,after)
before, after = texts[files[1]]
start = '//k = 250\nextern const t_pid_gain sp_gain_turnOut135_1800'
end = 'extern const t_param *const mode_1800[17];'
assert before[:before.index(start)] == after[:after.index(start)]
assert before[before.index(end):] == after[after.index(end):]
for side in ('L','R'):
    assert after.count(f'extern const t_turn_param_table slalom_out{side}135_1800_table;') == 1
before, after = texts['README.md']
start = after.index('### 機体A・turn1800の左右135度退出への暫定値反映\n')
end = after.index('### 機体B・turn1600への形状テーブル転用\n',start)
assert after[:start]+after[end:] == before

def log_text(name):
    content = (OUT/name).read_bytes()
    return content.decode('utf-16' if content.startswith((b'\xff\xfe',b'\xfe\xff')) else 'utf-8-sig')

host = log_text('host_linkage.stdout.txt')
assert 'Ran 1 test' in host and host.rstrip().endswith('OK') and 'skipped=' not in host
assert 'Linking CXX executable mouse_type8.elf' in log_text('build.stdout.txt')
with (OUT/'patch_from_this_task.patch').open('x',encoding='utf-8',newline='\n') as f:
    f.writelines(diff)
report = dict(passed=True,target='A',preset='Debug-MouseA',status='USER_AUTHORIZED_PROVISIONAL_SOURCE_APPLICATION',
    artifact=str(ELF),artifact_sha256=hashlib.sha256(data).hexdigest(),compiled_tables=rows,
    modes_verified=list(modes),physical_angle_mirrored=True,logical_angles_preserved=True,
    directional_pid_ff_references_preserved=True,existing_unrelated_working_changes_preserved=True,
    legacy_marker_switch_does_not_select_out135_tables=True,
    configure_and_build='PASS: bundled CMake --preset/--build --preset Debug-MouseA',
    host_parameter_linkage_test='PASS: test_run_param_linkage.py; A/B parameter sources; no skip',
    source_files={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in files},
    source_candidate=str(OUT/'candidate.settings.json'),physical_flash_performed=False,
    right_repeatability_confirmed=False,left_physical_run_verified=False,
    limitations=['右の完全な測定1本による暫定反映。充電後の再現性と左実走・連続／最短接続は未確認。'])
with (OUT/'firmware_verification.json').open('x',encoding='utf-8') as f:
    json.dump(report,f,ensure_ascii=False,indent=2,allow_nan=False)
    f.write('\n')
print(json.dumps(dict(passed=True,compiled_tables=rows,existing_changes_preserved=True),ensure_ascii=False,indent=2))
