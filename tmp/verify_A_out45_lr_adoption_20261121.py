from pathlib import Path
from datetime import datetime,timezone
import difflib
import hashlib
import json
import re
import shutil
import struct

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_outr45_20261120_v1'
AUDIT=BASE/'firmware_out45_lr_adoption_20261121_v1'
DOC=ROOT/'tools/turn_analysis/20261121_A_turn1800_OUTR45_adoption'
ELF=ROOT/'build/Debug-MouseA/mouse_type8.elf'
def read(path):return json.loads(path.read_text(encoding='utf-8'))
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def save(path,value):
    with path.open('x',encoding='utf-8') as file:json.dump(value,file,ensure_ascii=False,indent=2);file.write('\n')
build=read(AUDIT/'build_result.json');tests=read(AUDIT/'host_test_result.json')
assert build['passed'] and tests['passed'] and tests['skipped']==0
assert sha(ELF)==build['elf_sha256']
data=ELF.read_bytes();assert data[:6]==b'\x7fELF\x01\x01'
header=struct.unpack_from('<HHIIIIIHHHHHH',data,16);assert header[1]==40
sections=[struct.unpack_from('<10I',data,header[5]+i*header[10]) for i in range(header[11])];symbols={}
for section in sections:
    if section[1]!=2:continue
    string_section=sections[section[6]];strings=data[string_section[4]:string_section[4]+string_section[5]]
    for offset in range(section[4],section[4]+section[5],section[9]):
        name,address,size,info,other,index=struct.unpack_from('<IIIBBH',data,offset)
        if name and 0<index<len(sections):symbols[strings[name:].split(b'\0',1)[0].decode('ascii')]=(address,size,index)
def values(name,fmt):
    address,size,index=symbols[name];section=sections[index];assert struct.calcsize(fmt)<=size
    return struct.unpack_from(fmt,data,section[4]+address-section[3])
def f32(value):return struct.unpack('<f',struct.pack('<f',value))[0]
texts={};diff=[]
for name in ('Params/Param_A/turn_1800/turn_1800.cpp','Params/Param_A/turn_1800/turn_1800.h','README.md'):
    before=(AUDIT/'before'/name).read_text(encoding='utf-8');after=(ROOT/name).read_text(encoding='utf-8');texts[name]=(before,after)
    diff.extend(difflib.unified_diff(before.splitlines(True),after.splitlines(True),fromfile='before/'+name,tofile='after/'+name))
for name,prefix in (('Params/Param_A/turn_1800/turn_1800.cpp','const'),('Params/Param_A/turn_1800/turn_1800.h','extern const')):
    before,after=texts[name];start='//k = 250\n'+prefix+' t_pid_gain sp_gain_turnOut45_1800';end='\n//k = 250\n'+prefix+' t_pid_gain sp_gain_turnIn135_1800'
    assert before[:before.index(start)]==after[:after.index(start)] and before[before.index(end):]==after[after.index(end):]
    if prefix=='const':
        for pattern in (r'^const t_turn_param_table slalom_outR45_1800_table = .+$',r'^const t_param param_out[LR]45_1800 = .+$',r'const t_(?:pid|ff)_gain\s+\w+\s*=\s*\{[^}]*\};'):
            assert re.findall(pattern,before,re.M)==re.findall(pattern,after,re.M)
        assert '#if' not in after[after.index(start):after.index(end)]
    else:
        for side in ('L','R'):assert after.count(f'extern const t_turn_param_table slalom_out{side}45_1800_table;')==1
rows=[];before_cpp=texts['Params/Param_A/turn_1800/turn_1800.cpp'][0]
for side,sign,direction,index in (('R',-1,1,9),('L',1,2,10)):
    table=values(f'slalom_out{side}45_1800_table','<5fif')
    assert table==tuple(map(f32,[1.8,sign*55.,15.4,15.9,sign*45.]))+(direction,f32(sign*1.7))
    gain_names=('sp_gain_turnOut45_1800','om_gain_turnOut45_1800',f'ff_gain_out45_{side}_1800')
    names=(f'slalom_out{side}45_1800_table',)+gain_names
    assert values(f'param_out{side}45_1800','<6I')==tuple(symbols[name][0] for name in names)+(0,0)
    ff_values=[float(item.strip().rstrip('f')) for item in re.search(r'const t_ff_gain ff_gain_out45_'+side+r'_1800 = \{([^}]+)\}',before_cpp)[1].split(',') if item.strip()]
    assert values(gain_names[-1],'<10f')==tuple(f32(value) for value in ff_values+[0.]*(10-len(ff_values)))
    for mode,count in (('mode_1800',17),('mode_1800_acc_v2',19)):
        assert values(mode,'<'+str(count)+'I')[index]==symbols[f'param_out{side}45_1800'][0]
    assert values('mode_1800_acc_v1','<19I')[index]==0
    rows.append(dict(side=side,table=table,physical_degree=table[4]+table[6],directional_feedforward_preserved=True))
preparation=read(AUDIT/'preparation.json')
for name in ('Params/Param_B/turn_1800/turn_1800.cpp','Params/Param_B/turn_1800/turn_1800.h'):
    assert sha(ROOT/name)==preparation['before_sha256'][name]
for name,expected in preparation['source_after_application'].items():assert sha(ROOT/name)==expected
assert sha(ROOT/'Params/Param_A/turn_1800/turn_1800.cpp')==build['source_sha256_after']
result=dict(passed=True,preset='Debug-MouseA',tables=rows,all_directional_PID_FF_and_parameter_references_preserved=True,
    mode_1800_and_acc_v2_OUT45_references_verified=True,acc_v1_OUT45_remains_disabled=True,
    right_geometry_preserved=True,other_turns_and_B_sources_preserved=True,
    source_sha256=build['source_sha256_after'],ELF_sha256=sha(ELF),host_test=tests,
    left_hardware_verified=False,firmware_written=False)
save(AUDIT/'firmware_lr_verification.json',result);save(DOC/'firmware_lr_verification.json',result)
shutil.copy2(AUDIT/'build_result.json',DOC/'build_lr_result.json')
shutil.copy2(AUDIT/'host_test_result.json',DOC/'host_test_lr_result.json')
for suffix in ('.elf','.map'):shutil.copy2(ELF.with_suffix(suffix),AUDIT/('mouse_type8_Debug-MouseA'+suffix))
(AUDIT/'this_task.diff').write_text(''.join(diff),encoding='utf-8')
adoption=read(DOC/'adoption.json');adoption.update(status='PROVISIONAL_LR_SOURCE_UPDATED_DEBUG_BUILD_ELF_VERIFIED_NOT_FLASHED',
    validation=result,left_mirror_verified_at_utc=datetime.now(timezone.utc).isoformat(),left_hardware_verified=False)
(DOC/'adoption.json').write_text(json.dumps(adoption,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
path=DOC/'README.md';text=path.read_text(encoding='utf-8');old='機体AのDebugビルド・参照テスト・左右のELF値と補正符号を確認中。左の実走および本体書き込みは未実施。'
new='Debug-MouseAのConfigure/Buildが成功し、既存の参照共有テストも1件成功・スキップ0。ARM ELFの左右の形状・反対符号の補正・方向別PID/FF参照・mode_1800/acc_v2の参照を照合した。右の形状・他ターン・Bソースを保持したことも変更前との差分で確認済み。[左右追加のELF検証](firmware_lr_verification.json)、[ビルド結果](build_lr_result.json)、[参照テスト結果](host_test_lr_result.json)を参照。左の実走および本体書き込みは未実施。'
assert text.count(old)==1;path.write_text(text.replace(old,new),encoding='utf-8')
for name in preparation['before_sha256']:
    target=AUDIT/'after'/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(ROOT/name,target)
state=read(BASE/'session_outr45_status_20261121_v15.json')
state.update(recorded_at_utc=datetime.now(timezone.utc).isoformat(),state='PROVISIONAL_LR_PARAMETERS_SOURCE_BUILD_ELF_VERIFIED_OFF',
    selected_left_parameters=adoption['left_parameters'],left_mirror_requested=True,left_hardware_verified=False,
    firmware_source_changed=True,Debug_MouseA_build_passed=True,ELF_values_and_references_verified=True)
save(BASE/'session_outr45_status_20261121_v16.json',state)
print(json.dumps(result,ensure_ascii=False),flush=True)
