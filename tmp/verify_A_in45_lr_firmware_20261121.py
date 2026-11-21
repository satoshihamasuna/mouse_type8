"""生成したARM ELFの左右45度の定数と参照、および変更範囲を照合する。"""
from pathlib import Path
import difflib
import hashlib
import json
import re
import shutil
import struct

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT/'tools/logs/pixel_camera/turn1800_A_inr45_20261120_v1'
OUT = BASE/'firmware_in45_lr_adoption_20261121_v1'
elf = ROOT/'build/Debug-MouseA/mouse_type8.elf'
data = elf.read_bytes()
assert data[:6] == b'\x7fELF\x01\x01'
header = struct.unpack_from('<HHIIIIIHHHHHH', data, 16)
assert header[1] == 40
sections = [struct.unpack_from('<10I', data, header[5]+i*header[10]) for i in range(header[11])]
symbols = {}
for section in sections:
    if section[1] != 2:
        continue
    strings = sections[section[6]]
    names = data[strings[4]:strings[4]+strings[5]]
    for offset in range(section[4], section[4]+section[5], section[9]):
        name, address, size, info, other, index = struct.unpack_from('<IIIBBH', data, offset)
        if name and 0 < index < len(sections):
            symbols[names[name:].split(b'\0', 1)[0].decode('ascii')] = (address, size, index)

def values(name, fmt):
    address, size, index = symbols[name]
    section = sections[index]
    offset = section[4]+address-section[3]
    return struct.unpack_from(fmt, data, offset)

def f32(value):
    return struct.unpack('<f', struct.pack('<f', value))[0]

def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

source = ROOT/'Params/Param_A/turn_1800/turn_1800.cpp'
before = (OUT/'Param_A_turn_1800.cpp.original.before').read_text(encoding='utf-8')
after = source.read_text(encoding='utf-8')
start = '//k = 250\nconst t_pid_gain sp_gain_turnIn45_1800'
end = '\n//k = 250\nconst t_pid_gain sp_gain_turnOut45_1800'
def outside_in45(text):
    prefix, tail = text.split(start)
    block, suffix = tail.split(end)
    return prefix, suffix
assert outside_in45(before) == outside_in45(after)
for name in ('sp_gain_turnIn45_1800', 'om_gain_turnIn45_1800', 'param_inL45_1800', 'param_inR45_1800'):
    pattern = r'^const [^\n]*\b'+name+r' = [^\n]+$'
    assert re.findall(pattern, before, re.M) == re.findall(pattern, after, re.M)

expected = read(BASE/'refinement_postcharge_final20261121_v1/recommended_candidate.settings.json')
right = values('slalom_inR45_1800_table', '<5fif')
left = values('slalom_inL45_1800_table', '<5fif')
assert right == tuple(map(f32, [1.8, -53., 2., 40., -45.]))+(1, f32(0.))
assert left == tuple(map(f32, [1.8, 53., 2., 40., 45.]))+(2, f32(0.))
sp = values('sp_gain_turnIn45_1800', '<3f')
om = values('om_gain_turnIn45_1800', '<3f')
assert sp == tuple(f32(expected['pid'][k]) for k in ('sp_kp', 'sp_ki', 'sp_kd'))
assert om == tuple(f32(expected['pid'][k]) for k in ('om_kp', 'om_ki', 'om_kd'))
keys = ('ff_sp_velo', 'ff_sp_accel', 'ff_sp_bias', 'ff_om_velo', 'ff_om_accel', 'ff_om_decel',
        'ff_om_bias', 'ff_om_jerk', 'ff_sp_turn_accel_mag', 'ff_sp_turn_velo_sq')
ff_right = values('ff_gain_in45_R_1800', '<10f')
assert ff_right == tuple(f32(expected['feedforward'][k]) for k in keys)
ff_left = values('ff_gain_in45_L_1800', '<10f')
original_ff = re.search(r'const t_ff_gain ff_gain_in45_L_1800 = \{([^}]+)\}', before)[1]
assert ff_left == tuple(f32(float(v.strip().rstrip('f'))) for v in original_ff.split(',') if v.strip())
for direction, index in (('R', 7), ('L', 8)):
    param_name = 'param_in'+direction+'45_1800'
    assert values(param_name, '<4I') == tuple(symbols[n][0] for n in (
        'slalom_in'+direction+'45_1800_table', 'sp_gain_turnIn45_1800',
        'om_gain_turnIn45_1800', 'ff_gain_in45_'+direction+'_1800'))
    for mode in ('mode_1800', 'mode_1800_acc_v2'):
        assert values(mode, '<17I')[index] == symbols[param_name][0]
build = read(OUT/'build_result.json')
assert build['passed'] and sha(source) == build['source_sha256_after']
assert sha(elf) == build['elf_sha256']
preparation = read(OUT/'preparation.json')
assert sha(ROOT/'Params/Param_B/turn_1800/turn_1800.cpp') == preparation['B_sha256']
shutil.copy2(source, OUT/'Param_A_turn_1800.cpp.after')
shutil.copy2(elf, OUT/'mouse_type8_Debug-MouseA.elf')
shutil.copy2(elf.with_suffix('.map'), OUT/'mouse_type8_Debug-MouseA.map')
(OUT/'this_task.diff').write_text(''.join(difflib.unified_diff(
    before.splitlines(keepends=True), after.splitlines(keepends=True),
    fromfile='Param_A_turn_1800.cpp.original.before', tofile='Param_A_turn_1800.cpp.after')), encoding='utf-8')
result = dict(passed=True, preset='Debug-MouseA', right_table=right, left_table=left,
    speed_pid=sp, omega_pid=om, feedforward_right=ff_right, feedforward_left=ff_left,
    parameter_references_verified=True, mode_1800_indices7_8_verified=True,
    mode_1800_acc_v2_indices7_8_verified=True, B_source_unchanged=True,
    only_in45_block_changed=True, gain_definitions_and_references_unchanged=True,
    elf_sha256=sha(elf), source_sha256=sha(source),
    firmware_flashing_performed=False, new_firmware_hardware_test_performed=False)
with (OUT/'elf_verification.json').open('x', encoding='utf-8') as f:
    json.dump(result, f, indent=2)
    f.write('\n')
print(json.dumps(result, indent=2))
