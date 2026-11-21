"""生成ARM ELFの左右180度の値・符号・参照と限定差分を照合する。"""
from pathlib import Path
import hashlib
import json
import re
import shutil
import struct

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_r180_20261120_v1'
OUT=BASE/'firmware_RL180_adoption_20261121_v1'
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
build=read(OUT/'build_result.json');host=read(OUT/'host_linkage_result.json')
assert build['passed'] and host['passed'] and host['skips']==0
prep=read(OUT/'preparation.json')
for name in ('turn_1800.cpp','turn_1800.h'):
    path=ROOT/'Params/Param_A/turn_1800'/name
    assert sha(path)==prep['sources'][str(path)]['after_sha256']
    before=(OUT/(name+'.before')).read_text(encoding='utf-8')
    after=path.read_text(encoding='utf-8')
    kind='const' if name.endswith('.cpp') else 'extern const'
    start='//k = 250\n'+kind+' t_pid_gain sp_gain_turn180_1800'
    end='\n//k = 300\n'
    def outside(text):
        prefix,tail=text.split(start,1);block,suffix=tail.split(end,1);return prefix,suffix
    assert outside(before)==outside(after),name
    for symbol in ('sp_gain_turn180_1800','om_gain_turn180_1800','param_L180_1800','param_R180_1800'):
        pattern=r'^'+kind+r' [^\n]*\b'+symbol+r'[^\n]*$'
        assert re.findall(pattern,before,re.M)==re.findall(pattern,after,re.M),symbol
assert sha(ROOT/'Params/Param_B/turn_1800/turn_1800.cpp')==prep['B_sha256']
elf=ROOT/'build/Debug-MouseA/mouse_type8.elf';data=elf.read_bytes()
assert sha(elf)==build['elf_sha256'] and data[:6]==b'\x7fELF\x01\x01'
header=struct.unpack_from('<HHIIIIIHHHHHH',data,16);assert header[1]==40
sections=[struct.unpack_from('<10I',data,header[5]+i*header[10]) for i in range(header[11])]
symbols={}
for section in sections:
    if section[1]!=2:continue
    strings=sections[section[6]];names=data[strings[4]:strings[4]+strings[5]]
    for offset in range(section[4],section[4]+section[5],section[9]):
        name,address,size,info,other,index=struct.unpack_from('<IIIBBH',data,offset)
        if name and 0<index<len(sections):symbols[names[name:].split(b'\0',1)[0].decode('ascii')]=(address,size,index)
def values(name,fmt):
    address,size,index=symbols[name];section=sections[index]
    return struct.unpack_from(fmt,data,section[4]+address-section[3])
def f32(v):return struct.unpack('<f',struct.pack('<f',v))[0]
right=values('slalom_R180_1800_table','<5fif')
left=values('slalom_L180_1800_table','<5fif')
assert right==tuple(map(f32,[1.8,-48.2,7.52,31.5,-180.]))+(1,f32(2.))
assert left==tuple(map(f32,[1.8,48.2,7.52,31.5,180.]))+(2,f32(-2.))
assert right[4]+right[6]==-178. and left[4]+left[6]==178.
selected=read(BASE/'selection_20261121_v1/selected.settings.json')
sp=values('sp_gain_turn180_1800','<3f');om=values('om_gain_turn180_1800','<3f')
assert sp==tuple(f32(selected['pid'][k]) for k in ('sp_kp','sp_ki','sp_kd'))
assert om==tuple(f32(selected['pid'][k]) for k in ('om_kp','om_ki','om_kd'))
before=(OUT/'turn_1800.cpp.before').read_text(encoding='utf-8');ff={}
for direction,index in (('R',5),('L',6)):
    ffname='ff_gain_long_turn_180_'+direction+'_1800'
    literal=re.search(r'const t_ff_gain '+ffname+r' = \{([^}]+)\}',before)[1]
    expected=tuple(f32(float(v.strip().rstrip('f'))) for v in literal.split(',') if v.strip())
    ff[direction]=values(ffname,'<10f');assert ff[direction]==expected
    param='param_'+direction+'180_1800'
    assert values(param,'<4I')==tuple(symbols[n][0] for n in
        ('slalom_'+direction+'180_1800_table','sp_gain_turn180_1800','om_gain_turn180_1800',ffname))
    for mode in ('mode_1800','mode_1800_acc_v1','mode_1800_acc_v2'):
        assert values(mode,'<17I')[index]==symbols[param][0],(mode,index)
shutil.copy2(elf,OUT/'mouse_type8_Debug-MouseA.elf')
shutil.copy2(elf.with_suffix('.map'),OUT/'mouse_type8_Debug-MouseA.map')
result=dict(passed=True,preset='Debug-MouseA',right_table=right,left_table=left,
    physical_command_angles_deg=dict(right=-178.,left=178.),speed_pid=sp,omega_pid=om,
    direction_specific_feedforward=ff,parameter_references_verified=True,
    all_three_1800_modes_indices5_6_verified=True,outside_RL180_blocks_unchanged=True,
    gains_and_parameter_references_unchanged=True,B_source_unchanged=True,
    elf_sha256=sha(elf),map_sha256=sha(elf.with_suffix('.map')),
    source_sha256={str(ROOT/'Params/Param_A/turn_1800'/n):sha(ROOT/'Params/Param_A/turn_1800'/n) for n in ('turn_1800.cpp','turn_1800.h')},
    firmware_flashing_performed=False,left_physical_test_performed=False,continuous_run_verified=False)
with (OUT/'elf_verification.json').open('x',encoding='utf-8') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
