"""生成ARM ELFの左右in45・専用PID/FF・モード参照と、この作業の差分範囲を検証。"""
from pathlib import Path
from datetime import datetime,timezone
import difflib
import hashlib
import json
import re
import shutil
import struct
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'tools/turn_analysis/20261121_B_turn1600_IN45_adoption'
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr45_20261120_v1';ELF=ROOT/'build/Debug-MouseB/mouse_type8.elf'
read=lambda p:json.loads(Path(p).read_text(encoding='utf-8-sig'))
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
data=ELF.read_bytes();assert data[:6]==b'\x7fELF\x01\x01'
header=struct.unpack_from('<HHIIIIIHHHHHH',data,16);assert header[1]==40
sections=[struct.unpack_from('<10I',data,header[5]+i*header[10]) for i in range(header[11])];symbols={}
for section in sections:
 if section[1]!=2:continue
 strings=sections[section[6]];names=data[strings[4]:strings[4]+strings[5]]
 for offset in range(section[4],section[4]+section[5],section[9]):
  name,address,size,info,other,index=struct.unpack_from('<IIIBBH',data,offset)
  if name and 0<index<len(sections):symbols[names[name:].split(b'\0',1)[0].decode('ascii')]=(address,size,index)
def values(name,fmt):
 address,size,index=symbols[name];section=sections[index]
 assert struct.calcsize(fmt)<=size
 return struct.unpack_from(fmt,data,section[4]+address-section[3])
def f32(value):return struct.unpack('<f',struct.pack('<f',value))[0]
expected=read(BASE/'provisional_decision_20261121_v1/selected.settings.json');p=expected['parameters']
right=values('slalom_inR45_1600_table','<5fIf');left=values('slalom_inL45_1600_table','<5fIf')
assert right==tuple(map(f32,[p['velo'],p['r_min'],p['lstart'],p['lend'],p['degree']]))+(1,f32(p['correction']))
assert left==tuple(map(f32,[p['velo'],-p['r_min'],p['lstart'],p['lend'],-p['degree']]))+(2,f32(-p['correction']))
sp=values('sp_gain_turnIn45_1600','<3f');om=values('om_gain_turnIn45_1600','<3f')
assert sp==tuple(f32(expected['pid'][k]) for k in ('sp_kp','sp_ki','sp_kd'))
assert om==tuple(f32(expected['pid'][k]) for k in ('om_kp','om_ki','om_kd'))
keys=('ff_sp_velo','ff_sp_accel','ff_sp_bias','ff_om_velo','ff_om_accel','ff_om_decel','ff_om_bias','ff_om_jerk','ff_sp_turn_accel_mag','ff_sp_turn_velo_sq')
ff=values('ff_gain_turnIn45_1600','<10f');assert ff==tuple(f32(expected['feedforward'][k]) for k in keys)
assert symbols['ff_gain_turnIn45_1600'][0]!=symbols['ff_gain_default'][0]
for direction,index in (('R',7),('L',8)):
 name='param_in'+direction+'45_1600'
 assert values(name,'<6I')==tuple(symbols[n][0] for n in ('slalom_in'+direction+'45_1600_table','sp_gain_turnIn45_1600','om_gain_turnIn45_1600','ff_gain_turnIn45_1600'))+(0,0)
 for mode in ('mode_1600','mode_1600_acc_v2'):assert values(mode,'<17I')[index]==symbols[name][0]
 if 'mode_1600_acc_v1' in symbols:assert values('mode_1600_acc_v1','<19I')[index]==0
for name in ('param_R90_1600','param_L90_1600','param_R180_1600','param_L180_1600','param_inR135_1600','param_inL135_1600'):
 assert values(name,'<4I')[3]==symbols['ff_gain_default'][0]
cpp='Params/Param_B/turn_1600/turn_1600.cpp';hdr='Params/Param_B/turn_1600/turn_1600.h';files=[cpp,hdr,'README.md'];diff=[]
before=(OUT/'before'/cpp).read_text(encoding='utf-8');after=(ROOT/cpp).read_text(encoding='utf-8')
def outside_in45(text):
 start=text.index('\n',text.index('const t_param param_RV90_1600 ='))+1
 end=text.index('//k = 250\nconst t_pid_gain sp_gain_turnOut45_1600')
 return text[:start],text[end:]
assert outside_in45(before)==outside_in45(after)
v1=re.search(r'const t_param \*const mode_1600_acc_v1\[\]\s*=\s*\{(.*?)\};',after,re.S)
assert v1 and [v.strip() for v in v1[1].split(',')][7:9]==['NULL','NULL']
assert (ROOT/hdr).read_text(encoding='utf-8').replace('extern const t_ff_gain ff_gain_turnIn45_1600;\n','')==(OUT/'before'/hdr).read_text(encoding='utf-8')
readme=(ROOT/'README.md').read_text(encoding='utf-8');begin=readme.index('### 機体B・turn1600の左右45度進入への暫定値反映\n');end=readme.index('### 最短走行開始時の予備加速\n',begin)
assert readme[:begin]+readme[end:]==(OUT/'before/README.md').read_text(encoding='utf-8')
for rel in files:
 original=(OUT/'before'/rel).read_text(encoding='utf-8');current=(ROOT/rel).read_text(encoding='utf-8')
 diff.extend(difflib.unified_diff(original.splitlines(True),current.splitlines(True),fromfile='before/'+rel,tofile=rel))
 target=OUT/'after'/rel;target.parent.mkdir(parents=True,exist_ok=True);assert not target.exists();shutil.copyfile(ROOT/rel,target)
(OUT/'this_task.diff').write_text(''.join(diff),encoding='utf-8')
build=read(OUT/'build_status.json');assert build['configure']['returncode']==build['build']['returncode']==0
cache=(ELF.parent/'CMakeCache.txt').read_text(encoding='utf-8');assert re.search(r'^MOUSE_TARGET:\w+=B$',cache,re.M)
for path,name in ((ELF,'mouse_type8_Debug-MouseB.elf'),(ELF.with_suffix('.map'),'mouse_type8_Debug-MouseB.map')):
 assert not (OUT/name).exists();shutil.copyfile(path,OUT/name)
result=dict(verified_at_utc=datetime.now(timezone.utc).isoformat(),passed=True,preset='Debug-MouseB',
 right_table=right,left_table=left,speed_pid=sp,omega_pid=om,in45_feedforward=ff,
 left_mirrored_at_user_request=True,left_physical_run_verified=False,parameter_references_verified=True,
 mode1600_and_acc_v2_indices7_8_verified=True,acc_v1_in45_still_unavailable=True,
 acc_v1_symbol_linked='mode_1600_acc_v1' in symbols,
 non_in45_ff_references_unchanged=True,only_in45_source_block_changed=True,existing_other_edits_preserved=True,
 host_validation=dict(command='python -m unittest discover -s tools/tests -p test_run_param_linkage.py -v',
  returncode=0,tests=1,mice=['A','B'],skips=0,observed_seconds=16.145),
 build=build,elf=str(ELF),elf_sha256=sha(ELF),archived_elf_sha256=sha(OUT/'mouse_type8_Debug-MouseB.elf'),
 source_sha256={rel:sha(ROOT/rel) for rel in files},input_settings_sha256=sha(BASE/'provisional_decision_20261121_v1/selected.settings.json'),
 hardware_connections_started=0,execution_commands_sent_by_application=0,firmware_write_performed=False,
 updated_firmware_hardware_test_performed=False,physical_power_state='BOTH_OFF_USER_CONFIRMED')
assert result['elf_sha256']==result['archived_elf_sha256']
with (OUT/'verification.json').open('x',encoding='utf-8') as f:json.dump(result,f,ensure_ascii=False,indent=2)
state_path=BASE/'resume_state.json';state=read(state_path);state.update(state='IN45_LR_FIRMWARE_SOURCE_APPLIED_DEBUG_B_BUILT_BOTH_OFF',
 source_params_changed=True,source_adoption_user_requested=True,firmware_adoption_record=str(OUT/'verification.json'),
 firmware_adoption_readme=str(OUT/'README.md'),updated_firmware_elf=str(OUT/'mouse_type8_Debug-MouseB.elf'),
 updated_firmware_elf_sha256=result['elf_sha256'],firmware_write_performed=False,left_physical_run_verified=False,
 left_value_source='right candidate mirrored at explicit user request',execution_pending=False,next_request_id=None,next_stage=None,
 next_action='機体B左右in45の暫定値・専用PID/FFをソース反映しDebug-MouseBビルド/ELF照合完了。実機書込み・左/連続走行は未実施。両方OFF。')
state_path.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
with (OUT/'README.md').open('a',encoding='utf-8') as f:f.write('\nConfigure/Debug-MouseBビルドは成功。PATHにcube-cmakeがないため既存CMakeCacheのSTM32同梱CMake4.3.1+st.1/Ninja/GNU14.3.1+st.2を使用（コマンドと出力はbuild_status.json/configure.stdout.txt/build.stdout.txt）。既存test_run_param_linkage.pyはA/Bとも実行、1テスト成功・skip0。生成ELFの左右形状・PID/FF全係数・左右t_param参照・mode_1600/acc_v2の7/8番を照合し、acc_v1の未対応とほかのターンの既定FFを保持していることを確認。値・参照・ハッシュはverification.json。ビルドの警告はin45変更外の既存書式/未使用変数。実機書込み・追加走行は0回。\n')
print(json.dumps(dict(passed=True,right_correction=right[-1],left_correction=left[-1],host_tests='A/B PASS, skip0',elf_sha256=result['elf_sha256'],firmware_written=False),ensure_ascii=False))
