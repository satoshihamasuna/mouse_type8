"""ARM ELFのin/out45共通FF参照と、出口の形状/PID保持を確認する。"""
from pathlib import Path
from datetime import datetime,timezone
import difflib
import hashlib
import json
import re
import shutil
import struct
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'tools/turn_analysis/20261121_B_turn1600_OUT45_FF_adoption'
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
 address,size,index=symbols[name];section=sections[index];assert struct.calcsize(fmt)<=size
 return struct.unpack_from(fmt,data,section[4]+address-section[3])
def f32(value):return struct.unpack('<f',struct.pack('<f',value))[0]
expected=read(BASE/'provisional_decision_20261121_v1/selected.settings.json')
keys=('ff_sp_velo','ff_sp_accel','ff_sp_bias','ff_om_velo','ff_om_accel','ff_om_decel','ff_om_bias','ff_om_jerk','ff_sp_turn_accel_mag','ff_sp_turn_velo_sq')
ff=values('ff_gain_turnIn45_1600','<10f');assert ff==tuple(f32(expected['feedforward'][k]) for k in keys)
ff_address=symbols['ff_gain_turnIn45_1600'][0];references={}
for kind,indices in (('in',{'R':7,'L':8}),('out',{'R':9,'L':10})):
 for direction,index in indices.items():
  name=f'param_{kind}{direction}45_1600';refs=values(name,'<4I')
  assert refs==tuple(symbols[n][0] for n in (f'slalom_{kind}{direction}45_1600_table',f'sp_gain_turn{kind.title()}45_1600',f'om_gain_turn{kind.title()}45_1600','ff_gain_turnIn45_1600'))
  for mode in ('mode_1600','mode_1600_acc_v2'):assert values(mode,'<17I')[index]==symbols[name][0]
  references[name]=[hex(value) for value in refs]
out_sp=values('sp_gain_turnOut45_1600','<3f');out_om=values('om_gain_turnOut45_1600','<3f')
assert out_sp==tuple(map(f32,[4.,.05,0.])) and out_om==tuple(map(f32,[.2,.01,0.]))
assert values('slalom_outR45_1600_table','<5fIf')==tuple(map(f32,[1.6,-60.,21.8,16.5,-45.]))+(1,f32(.3))
assert values('slalom_outL45_1600_table','<5fIf')==tuple(map(f32,[1.6,60.,21.8,16.5,45.]))+(2,f32(-.3))
for name in ('param_outR135_1600','param_outL135_1600','param_R90_1600','param_L90_1600'):
 assert values(name,'<4I')[3]==symbols['ff_gain_default'][0]
cpp='Params/Param_B/turn_1600/turn_1600.cpp';before=(OUT/'before'/cpp).read_text(encoding='utf-8');after=(ROOT/cpp).read_text(encoding='utf-8')
def retained_code(text):
 return [line for line in text.splitlines() if not line.lstrip().startswith('//') and not re.match(r'^const t_param param_out[LR]45_1600 = ',line)]
assert retained_code(before)==retained_code(after)
assert len(re.findall(r'^const t_param param_out[LR]45_1600 = ',after,re.M))==2
build=read(OUT/'build_status.json');assert build['configure']['returncode']==build['build']['returncode']==0
assert re.search(r'^MOUSE_TARGET:\w+=B$',(ELF.parent/'CMakeCache.txt').read_text(encoding='utf-8'),re.M)
diff=[]
for rel in (cpp,'README.md'):
 old=(OUT/'before'/rel).read_text(encoding='utf-8');new=(ROOT/rel).read_text(encoding='utf-8')
 diff.extend(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile='before/'+rel,tofile=rel))
 target=OUT/'after'/rel;target.parent.mkdir(parents=True,exist_ok=True);assert not target.exists();shutil.copyfile(ROOT/rel,target)
(OUT/'this_task.diff').write_text(''.join(diff),encoding='utf-8')
for path,name in ((ELF,'mouse_type8_Debug-MouseB.elf'),(ELF.with_suffix('.map'),'mouse_type8_Debug-MouseB.map')):
 assert not (OUT/name).exists();shutil.copyfile(path,OUT/name)
result=dict(verified_at_utc=datetime.now(timezone.utc).isoformat(),passed=True,mouse='B',preset='Debug-MouseB',
 source_changed_only_out45_ff_references_and_comments=True,all_shape_pid_and_other_code_preserved=True,
 shared_in_out45_feedforward=ff,compiled_parameter_references=references,mode_indices7_to10_verified=True,
 out45_speed_pid=out_sp,out45_omega_pid=out_om,out45_shape_values_verified=True,
 host_validation=dict(command='python -m unittest discover -s tools/tests -p test_run_param_linkage.py -v',returncode=0,tests=1,mice=['A','B'],skips=0,observed_seconds=5.777),
 build=build,elf_sha256=sha(ELF),archived_elf_sha256=sha(OUT/'mouse_type8_Debug-MouseB.elf'),
 source_sha256=sha(ROOT/cpp),firmware_write_performed=False,new_hardware_test_performed=False,additional_execution_commands=0,
 input_sha256={str(Path(__file__)):sha(Path(__file__)),str(BASE/'provisional_decision_20261121_v1/selected.settings.json'):sha(BASE/'provisional_decision_20261121_v1/selected.settings.json')})
assert result['elf_sha256']==result['archived_elf_sha256']
with (OUT/'verification.json').open('x',encoding='utf-8') as f:json.dump(result,f,ensure_ascii=False,indent=2)
with (OUT/'README.md').open('a',encoding='utf-8') as f:f.write('\nDebug-MouseBのConfigure/ビルド成功。既存パラメータリンク検証はA/B実行・1テスト成功・skip0。生成ELFでin45/out45左右4定義のFF参照一致、角加速/角減速0.000750、出口形状/PID、mode_1600/acc_v2の7〜10番を確認。ソースの変更は左右out45のFF参照とコメントだけ。値・コマンド・ハッシュはverification.json、実行出力はconfigure.stdout.txt/build.stdout.txt。実機接続・書込み・追加走行なし。\n')
state_path=BASE/'resume_state.json';state=read(state_path);state.update(state='IN45_LR_AND_OUT45_FF_SOURCE_APPLIED_DEBUG_B_BUILT_BOTH_OFF',
 firmware_out45_ff_adoption_record=str(OUT/'verification.json'),updated_firmware_elf=str(OUT/'mouse_type8_Debug-MouseB.elf'),
 updated_firmware_elf_sha256=result['elf_sha256'],firmware_write_performed=False,execution_pending=False,
 next_action='機体B左右in45に加え、左右out45にも同じFF一式をソース反映。Bビルド/ELF照合完了。実機書込み・追加走行なし、両方OFF。')
state_path.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(passed=True,shared_by=['inR45','inL45','outR45','outL45'],angular_accel_decel=[ff[4],ff[5]],shape_pid_preserved=True,firmware_written=False),ensure_ascii=False))
