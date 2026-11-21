"""生成したARM ELFから右45度の定数と参照を読み、採用設定へ照合する。"""
from pathlib import Path
import hashlib
import json
import shutil
import struct

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_inr45_20261120_v1'
OUT=BASE/'firmware_inr45_adoption_20261121_v1'
elf=ROOT/'build/Debug-MouseA/mouse_type8.elf'
data=elf.read_bytes()
assert data[:6]==b'\x7fELF\x01\x01'
header=struct.unpack_from('<HHIIIIIHHHHHH',data,16)
assert header[1]==40  # EM_ARM.
sections=[struct.unpack_from('<10I',data,header[5]+i*header[10]) for i in range(header[11])]
symbols={}
for section in sections:
    if section[1]!=2:continue
    strings=sections[section[6]]
    names=data[strings[4]:strings[4]+strings[5]]
    for offset in range(section[4],section[4]+section[5],section[9]):
        name,address,size,info,other,index=struct.unpack_from('<IIIBBH',data,offset)
        if name and 0<index<len(sections):
            text=names[name:].split(b'\0',1)[0].decode('ascii')
            symbols[text]=(address,size,index)

def raw(name):
    address,size,index=symbols[name]
    section=sections[index]
    offset=section[4]+address-section[3]
    return data[offset:offset+size]

def values(name,fmt):return struct.unpack_from(fmt,raw(name))
def f32(value):return struct.unpack('<f',struct.pack('<f',value))[0]
def read(path):return json.loads(path.read_text(encoding='utf-8-sig'))
expected=read(BASE/'refinement_postcharge_final20261121_v1/recommended_candidate.settings.json')
right=values('slalom_inR45_1800_table','<5fif')
left=values('slalom_inL45_1800_table','<5fif')
assert right==tuple(map(f32,[1.8,-53.,2.,40.,-45.]))+(1,f32(0.))
assert left==tuple(map(f32,[1.8,53.,1.33,46.18,45.]))+(2,f32(0.))
sp=values('sp_gain_turnIn45_1800','<3f')
om=values('om_gain_turnIn45_1800','<3f')
ff=values('ff_gain_in45_R_1800','<10f')
assert sp==tuple(f32(expected['pid'][k]) for k in ('sp_kp','sp_ki','sp_kd'))
assert om==tuple(f32(expected['pid'][k]) for k in ('om_kp','om_ki','om_kd'))
keys=('ff_sp_velo','ff_sp_accel','ff_sp_bias','ff_om_velo','ff_om_accel','ff_om_decel',
      'ff_om_bias','ff_om_jerk','ff_sp_turn_accel_mag','ff_sp_turn_velo_sq')
assert ff==tuple(f32(expected['feedforward'][k]) for k in keys)
param=values('param_inR45_1800','<4I')
assert param==tuple(symbols[n][0] for n in ('slalom_inR45_1800_table',
    'sp_gain_turnIn45_1800','om_gain_turnIn45_1800','ff_gain_in45_R_1800'))
mode=values('mode_1800','<17I')
assert mode[7]==symbols['param_inR45_1800'][0]
mode_v2=values('mode_1800_acc_v2','<17I')
assert mode_v2[7]==symbols['param_inR45_1800'][0]
build=read(OUT/'build_result.json')
source=ROOT/'Params/Param_A/turn_1800/turn_1800.cpp'
source_sha=hashlib.sha256(source.read_bytes()).hexdigest()
assert build['passed'] and source_sha==build['source_sha256_after']
assert hashlib.sha256(data).hexdigest()==build['elf_sha256']
preparation=read(OUT/'preparation.json')
b_source=ROOT/'Params/Param_B/turn_1800/turn_1800.cpp'
assert hashlib.sha256(b_source.read_bytes()).hexdigest()==preparation['before_sha256']['Params/Param_B/turn_1800/turn_1800.cpp']
shutil.copy2(source,OUT/'Param_A_turn_1800.cpp.after')
shutil.copy2(elf,OUT/'mouse_type8_Debug-MouseA.elf')
shutil.copy2(elf.with_suffix('.map'),OUT/'mouse_type8_Debug-MouseA.map')
result=dict(passed=True,preset='Debug-MouseA',right_table=right,left_table=left,
    speed_pid=sp,omega_pid=om,feedforward=ff,parameter_references_verified=True,
    mode_1800_index7_verified=True,mode_1800_acc_v2_index7_verified=True,
    B_source_unchanged=True,elf_sha256=build['elf_sha256'],source_sha256=source_sha,
    firmware_flashing_performed=False,new_firmware_hardware_test_performed=False)
with (OUT/'elf_verification.json').open('x',encoding='utf-8') as f:json.dump(result,f,indent=2);f.write('\n')
print(json.dumps(result,indent=2))
