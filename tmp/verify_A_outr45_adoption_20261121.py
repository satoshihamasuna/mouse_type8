from pathlib import Path
import difflib
import hashlib
import json
import re
import shutil
import struct

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_outr45_20261120_v1'
AUDIT=BASE/'firmware_outR45_adoption_20261121_v1'
OUT=ROOT/'tools/turn_analysis/20261121_A_turn1800_OUTR45_adoption'
ELF=ROOT/'build/Debug-MouseA/mouse_type8.elf'
def read(path):return json.loads(path.read_text(encoding='utf-8'))
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def save(path,value):
    with path.open('x',encoding='utf-8') as file:json.dump(value,file,ensure_ascii=False,indent=2);file.write('\n')
data=ELF.read_bytes();assert data[:6]==b'\x7fELF\x01\x01'
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
    assert struct.calcsize(fmt)<=size,(name,size,fmt)
    return struct.unpack_from(fmt,data,section[4]+address-section[3])
def f32(value):return struct.unpack('<f',struct.pack('<f',value))[0]
right=values('slalom_outR45_1800_table','<5fif');left=values('slalom_outL45_1800_table','<5fif')
assert right==tuple(map(f32,[1.8,-55.,15.4,15.9,-45.]))+(1,f32(-1.7))
assert left==tuple(map(f32,[1.8,55.,22.66,26.,45.]))+(2,f32(0))
expected=read(BASE/'lengths_cminus1p7_Ls15p4_Le15p9_20261121_01/candidate.settings.json')
sp=values('sp_gain_turnOut45_1800','<3f');om=values('om_gain_turnOut45_1800','<3f')
assert sp==tuple(f32(expected['pid'][k]) for k in ('sp_kp','sp_ki','sp_kd'))
assert om==tuple(f32(expected['pid'][k]) for k in ('om_kp','om_ki','om_kd'))
keys=('ff_sp_velo','ff_sp_accel','ff_sp_bias','ff_om_velo','ff_om_accel','ff_om_decel','ff_om_bias','ff_om_jerk','ff_sp_turn_accel_mag','ff_sp_turn_velo_sq')
ff=values('ff_gain_out45_R_1800','<10f');assert ff==tuple(f32(expected['feedforward'][key]) for key in keys)
for side,index in (('R',9),('L',10)):
    names=(f'slalom_out{side}45_1800_table','sp_gain_turnOut45_1800','om_gain_turnOut45_1800',f'ff_gain_out45_{side}_1800')
    assert values(f'param_out{side}45_1800','<6I')==tuple(symbols[name][0] for name in names)+(0,0)
    for mode,count in (('mode_1800',17),('mode_1800_acc_v2',19)):
        assert values(mode,'<'+str(count)+'I')[index]==symbols[f'param_out{side}45_1800'][0]
    assert values('mode_1800_acc_v1','<19I')[index]==0

diff=[];texts={}
for name in ('Params/Param_A/turn_1800/turn_1800.cpp','Params/Param_A/turn_1800/turn_1800.h','README.md'):
    before=(AUDIT/'before'/name).read_text(encoding='utf-8');after=(ROOT/name).read_text(encoding='utf-8');texts[name]=(before,after)
    diff.extend(difflib.unified_diff(before.splitlines(True),after.splitlines(True),fromfile='before/'+name,tofile='after/'+name))
    target=AUDIT/'after'/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(ROOT/name,target)
for name,prefix in (('Params/Param_A/turn_1800/turn_1800.cpp','const'),('Params/Param_A/turn_1800/turn_1800.h','extern const')):
    before,after=texts[name]
    start='//k = 250\n'+prefix+' t_pid_gain sp_gain_turnOut45_1800'
    end='\n//k = 250\n'+prefix+' t_pid_gain sp_gain_turnIn135_1800'
    assert before[:before.index(start)]==after[:after.index(start)]
    assert before[before.index(end):]==after[after.index(end):]
    if prefix=='const':
        for pattern in (r'^const t_turn_param_table slalom_outL45_1800_table = .+$',r'^const t_param param_out[LR]45_1800 = .+$',r'const t_(?:ff|pid)_gain\s+\w+\s*=\s*\{[^}]*\};'):
            assert re.findall(pattern,before,re.M)==re.findall(pattern,after,re.M)
    else:assert after.count('extern const t_turn_param_table slalom_outR45_1800_table;')==1
before,after=texts['README.md'];start=after.index('### 機体A・turn1800の右45度退出の暫定採用値\n');end=after.index('### 機体A・turn1800の左右135度進入調整値\n',start)
assert after[:start]+after[end:]==before
hashes=read(AUDIT/'before_sha256.json')
for name in ('Params/Param_B/turn_1800/turn_1800.cpp','Params/Param_B/turn_1800/turn_1800.h'):assert sha(ROOT/name)==hashes[name]
build=read(AUDIT/'build_result.json');assert build['passed'] and sha(ELF)==build['elf_sha256']
assert sha(ROOT/'Params/Param_A/turn_1800/turn_1800.cpp')==build['source_sha256_after']
for suffix in ('.elf','.map'):shutil.copy2(ELF.with_suffix(suffix),AUDIT/('mouse_type8_Debug-MouseA'+suffix))
(AUDIT/'this_task.diff').write_text(''.join(diff),encoding='utf-8')
result=dict(passed=True,preset='Debug-MouseA',right_table=right,left_table_unchanged=left,speed_pid=sp,omega_pid=om,
    right_feedforward=ff,parameter_pointer_references_verified=True,mode_1800_and_acc_v2_indices9_10_verified=True,
    acc_v1_OUT45_remains_disabled=True,only_A_right_OUT45_block_and_its_header_changed=True,
    all_left_definitions_PID_FF_and_parameter_references_preserved=True,B_sources_unchanged=True,
    source_sha256=build['source_sha256_after'],elf_sha256=sha(ELF),firmware_written=False,
    host_test=dict(command='python -m unittest discover -s tools/tests -p test_run_param_linkage.py -v',passed=1,skipped=0,source='Observed successful tool output; not rerun.'))
save(AUDIT/'firmware_verification.json',result);save(OUT/'firmware_verification.json',result)
shutil.copy2(AUDIT/'build_result.json',OUT/'build_result.json')
adoption=read(OUT/'adoption.json');adoption.update(status='PROVISIONAL_SOURCE_UPDATED_DEBUG_BUILD_AND_ELF_VERIFIED_NOT_FLASHED',validation=result)
(OUT/'adoption.json').write_text(json.dumps(adoption,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
path=OUT/'README.md';text=path.read_text(encoding='utf-8');old='機体AのDebugビルド・既存リンクテスト・ELF内の値と参照を確認中。結果は追記する。本体Flashへの書き込みは未実施。'
new='Debug-MouseAのConfigure/Buildは成功。cube-cmakeがPATHにないため、既存CMakeCacheと同じSTM32CubeのCMake/Ninja/Arm GCCを使用した。既存のtest_run_param_linkage.pyは1件成功・スキップ0で、A/Bの実パラメータをホストでコンパイルして参照共有を確認した。生成ARM ELFの右形状・補正符号・PID/FF・mode_1800とmode_1800_acc_v2の参照を照合し、左定義・Bソース・他ターンが変更前と同じことも確認した。[ELF検証記録](firmware_verification.json)を参照。本体Flashへの書き込みと更新後の実走は未実施。'
assert text.count(old)==1;path.write_text(text.replace(old,new),encoding='utf-8')
print(json.dumps(result,ensure_ascii=False),flush=True)
