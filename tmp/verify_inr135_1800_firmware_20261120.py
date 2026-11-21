"""生成ELFの左右135入口テーブル・PID/FF参照と、今回の編集範囲を確認する。"""
from pathlib import Path
import difflib
import hashlib
import json
import math
import re
import struct
import subprocess

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'tools/logs/pixel_camera/turn1800_A_inr135_20261120_v1/Ls2p5_Le26_c2p4_review_v1/firmware_application_20261120_v1'
ELF = ROOT / 'build/Debug-MouseA/mouse_type8.elf'
cache = (ELF.parent / 'CMakeCache.txt').read_text(encoding='utf-8')
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
segments = [struct.unpack_from('<IIIIIIII', data, phoff + index * phsize) for index in range(phcount)]


def contents(name):
    address, size = symbols[name]
    for kind, offset, vaddr, _, filesz, _, _, _ in segments:
        if kind == 1 and vaddr <= address and address + size <= vaddr + filesz:
            return data[offset + address - vaddr:offset + address - vaddr + size]
    raise AssertionError((name, hex(address), size))


rows = []
for side, sign, turn_dir in (('R', -1, 1), ('L', 1, 2)):
    table_name = f'slalom_in{side}135_1800_table'
    payload = contents(table_name)
    assert len(payload) == 28
    velo, radius, lstart, lend, degree, direction, correction = struct.unpack('<fffffif', payload)
    expected = (1.8, sign * 40., 2.5, 26., sign * 135., turn_dir, -sign * 2.4)
    for have, want in zip((velo, radius, lstart, lend, degree, direction, correction), expected):
        assert math.isclose(have, want, abs_tol=1e-5), (side, have, want)
    parameter_name = f'param_in{side}135_1800'
    pointers = struct.unpack('<6I', contents(parameter_name))
    pointer_names = [table_name, 'sp_gain_turnIn135_1800', 'om_gain_turnIn135_1800', f'ff_gain_in135_{side}_1800']
    assert list(pointers[:4]) == [symbols[name][0] for name in pointer_names]
    assert math.isclose(degree + correction, sign * 132.6, abs_tol=1e-5)
    for mode in ('mode_1800', 'mode_1800_acc_v1', 'mode_1800_acc_v2'):
        mode_values = struct.unpack('<' + 'I' * (len(contents(mode)) // 4), contents(mode))
        assert mode_values[11 if side == 'R' else 12] == symbols[parameter_name][0], (mode, side)
    rows.append(dict(side=side, velocity_m_s=velo, radius_mm=radius, Lstart_mm=lstart, Lend_mm=lend,
        logical_degree=degree, degree_correction=correction, physical_degree=degree + correction,
        direction_enum=direction, table_address=hex(symbols[table_name][0]),
        parameter_address=hex(symbols[parameter_name][0]), gain_references=pointer_names[1:]))

files = ['Params/Param_A/turn_1800/turn_1800.cpp', 'Params/Param_A/turn_1800/turn_1800.h', 'README.md']
text = {}
diff = []
for name in files:
    before = (OUT / 'before' / name).read_text(encoding='utf-8')
    after = (ROOT / name).read_text(encoding='utf-8')
    text[name] = (before, after)
    diff.extend(difflib.unified_diff(before.splitlines(True), after.splitlines(True),
        fromfile='before/' + name, tofile='after/' + name, n=3))
before, after = text[files[0]]
start = '//k = 250\nconst t_pid_gain sp_gain_turnIn135_1800'
end = '//k = 250\nconst t_pid_gain sp_gain_turnOut135_1800'
assert before[:before.index(start)] == after[:after.index(start)]
assert before[before.index(end):] == after[after.index(end):]
for kind in ('t_pid_gain', 't_ff_gain'):
    pattern = r'const ' + kind + r'\s+\w+\s*=\s*\{[^}]*\};'
    assert re.findall(pattern, before) == re.findall(pattern, after)
before, after = text[files[1]]
start = '//k = 250\nextern const t_pid_gain sp_gain_turnIn135_1800'
end = '//k = 250\nextern const t_pid_gain sp_gain_turnOut135_1800'
assert before[:before.index(start)] == after[:after.index(start)]
assert before[before.index(end):] == after[after.index(end):]
before, after = text['README.md']
start = after.index('### 機体A・turn1800の左右135度進入調整値\n')
end = after.index('### 機体B・turn1600への形状テーブル転用\n', start)
assert after[:start] + after[end:] == before
with (OUT / 'patch_from_this_task.patch').open('x', encoding='utf-8', newline='\n') as file:
    file.writelines(diff)
report = dict(passed=True, target='A', preset='Debug-MouseA', artifact=str(ELF),
    artifact_sha256=hashlib.sha256(data).hexdigest(), compiled_tables=rows,
    physical_angle_mirrored=True, logical_angles_preserved=True,
    modes_verified=['mode_1800', 'mode_1800_acc_v1', 'mode_1800_acc_v2'],
    directional_pid_ff_references_preserved=True, existing_unrelated_working_changes_preserved=True,
    legacy_marker_switch_does_not_select_in135_tables=True,
    source_files={name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest() for name in files},
    host_parameter_linkage_test='PASS: unittest test_run_param_linkage.py; A/B real parameter definitions; no skip',
    configure_and_build='PASS: bundled CMake 4.3.1+st.1 --preset/--build --preset Debug-MouseA',
    physical_flash_performed=False, left_physical_run_verified=False,
    source_candidate='tools/logs/pixel_camera/turn1800_A_inr135_20261120_v1/Ls2p5_Le26_c2p4_review_v1/candidate.settings.json')
with (OUT / 'verification.json').open('x', encoding='utf-8') as file:
    json.dump(report, file, ensure_ascii=False, indent=2, allow_nan=False)
    file.write('\n')
print(json.dumps(dict(passed=True, compiled_tables=rows, existing_changes_preserved=True), ensure_ascii=False, indent=2))
