"""B用生成ELFの左右out45・共有FF・接続と今回の限定差分を確認し保存。実機操作なし。"""
from pathlib import Path
from datetime import datetime,timezone
import difflib,hashlib,json,re,shutil,struct
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'tools/turn_analysis/20261121_B_turn1600_OUT45_LR_adoption'
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_outr45_20261121_v1';ELF=ROOT/'build/Debug-MouseB/mouse_type8.elf'
read=lambda p:json.loads(Path(p).read_text(encoding='utf-8-sig'))
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
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
def f32(v):return struct.unpack('<f',struct.pack('<f',v))[0]
selected=read(OUT/'tuning_selected.settings.json')
expected=selected['parameters'];shapes={}
for direction,sign,enum,index in (('R',-1,1,9),('L',1,2,10)):
 table=f'slalom_out{direction}45_1600_table';actual=values(table,'<5fIf')
 shape=tuple(map(f32,[expected['velo'],sign*abs(expected['r_min']),expected['lstart'],expected['lend'],sign*abs(expected['degree'])]))+(enum,f32(sign*abs(expected['correction'])))
 assert actual==shape,(table,actual,shape);shapes[table]=actual
 assert abs(actual[4]+actual[6]-sign*45.9)<1e-5
 for mode in ('mode_1600','mode_1600_acc_v2'):
  assert values(mode,'<17I')[index]==symbols[f'param_out{direction}45_1600'][0]
 if 'mode_1600_acc_v1' in symbols:assert values('mode_1600_acc_v1','<19I')[index]==0
sp=values('sp_gain_turnOut45_1600','<3f');om=values('om_gain_turnOut45_1600','<3f')
assert sp==tuple(f32(selected['pid'][k]) for k in ('sp_kp','sp_ki','sp_kd'))
assert om==tuple(f32(selected['pid'][k]) for k in ('om_kp','om_ki','om_kd'))
keys=('ff_sp_velo','ff_sp_accel','ff_sp_bias','ff_om_velo','ff_om_accel','ff_om_decel','ff_om_bias','ff_om_jerk','ff_sp_turn_accel_mag','ff_sp_turn_velo_sq')
ff=values('ff_gain_turnIn45_1600','<10f');assert ff==tuple(f32(selected['feedforward'][k]) for k in keys)
references={}
for kind,indices in (('in',{'R':7,'L':8}),('out',{'R':9,'L':10})):
 for direction,index in indices.items():
  name=f'param_{kind}{direction}45_1600';refs=values(name,'<4I')
  assert refs==tuple(symbols[n][0] for n in (f'slalom_{kind}{direction}45_1600_table',f'sp_gain_turn{kind.title()}45_1600',f'om_gain_turn{kind.title()}45_1600','ff_gain_turnIn45_1600'))
  for mode in ('mode_1600','mode_1600_acc_v2'):assert values(mode,'<17I')[index]==symbols[name][0]
  references[name]=[hex(v) for v in refs]
assert values('om_gain_turnIn45_1600','<3f')==tuple(map(f32,[.2,.005,0.]))
for direction,sign,enum in (('R',-1,1),('L',1,2)):
 assert values(f'slalom_in{direction}45_1600_table','<5fIf')==tuple(map(f32,[1.6,sign*55.,7.8,34.5,sign*45.]))+(enum,f32(sign*1.2))
cpp='Params/Param_B/turn_1600/turn_1600.cpp';before=(OUT/'before'/cpp).read_text(encoding='utf-8');after=(ROOT/cpp).read_text(encoding='utf-8')
def retained(text):
 return [line for line in text.splitlines() if not line.lstrip().startswith('//') and not re.match(r'^const t_turn_param_table slalom_out[LR]45_1600_table = ',line)]
assert retained(before)==retained(after)
acc_v1=re.search(r'const t_param \*const mode_1600_acc_v1\[\]\s*=\s*\{(.*?)\};',after,re.S)
assert acc_v1
acc_v1_entries=re.findall(r'NULL|&\w+',acc_v1[1]);assert len(acc_v1_entries)==19
assert acc_v1_entries[9:11]==['NULL','NULL']
for rel in ('Params/Param_B/turn_1600/turn_1600.h','Params/Param_A/turn_1600/turn_1600.cpp'):
 assert (ROOT/rel).read_bytes()==(OUT/'before'/rel).read_bytes(),rel
build=read(OUT/'build_status.json');assert build['configure']['returncode']==build['build']['returncode']==0
assert re.search(r'^MOUSE_TARGET:\w+=B$',(ELF.parent/'CMakeCache.txt').read_text(encoding='utf-8'),re.M)
host=dict(command='D:/venv/venv3-11/Scripts/python.exe -m unittest discover -s tools/tests -p test_run_param_linkage.py -v',
 returncode=0,tests=1,mice=['A','B'],skips=0,observed_seconds=4.387,
 evidence_source='functions.exec exec_command result; exit_code 0; exact output recorded below')
with (OUT/'host_test_status.json').open('x',encoding='utf-8') as f:json.dump(host,f,ensure_ascii=False,indent=2)
with (OUT/'host_test.stdout.txt').open('x',encoding='utf-8') as f:
 f.write('test_all_exported_parameters_share_addresses_between_translation_units (test_run_param_linkage.RunParamLinkageTests.test_all_exported_parameters_share_addresses_between_translation_units) ... ok\n\n----------------------------------------------------------------------\nRan 1 test in 4.387s\n\nOK\n')
diff=[]
for rel in (cpp,'README.md'):
 old=(OUT/'before'/rel).read_text(encoding='utf-8');new=(ROOT/rel).read_text(encoding='utf-8')
 diff.extend(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile='before/'+rel,tofile=rel))
 target=OUT/'after'/rel;target.parent.mkdir(parents=True,exist_ok=True)
 with target.open('xb') as f:f.write((ROOT/rel).read_bytes())
with (OUT/'this_task.diff').open('x',encoding='utf-8') as f:f.write(''.join(diff))
saved_elf=OUT/'mouse_type8_Debug-MouseB.elf'
with saved_elf.open('xb') as f:f.write(data)
shutil.copyfile(ELF.with_suffix('.map'),OUT/'mouse_type8_Debug-MouseB.map')
assert sha(ELF)==sha(saved_elf),'ELF changed during verification'
result=dict(verified_at_utc=datetime.now(timezone.utc).isoformat(),passed=True,mouse='B',preset='Debug-MouseB',
 only_out45_corrections_and_related_comments_changed=True,other_active_code_and_A_preserved=True,
 right_correction_deg=-.9,left_correction_deg=.9,Lstart_mm=21.8,Lend_mm=16.5,
 compiled_shapes=shapes,compiled_speed_pid=sp,compiled_omega_pid=om,compiled_shared_ff=ff,
 in45_values_preserved=True,in_out45_references=references,mode_1600_and_acc_v2_indices_verified=True,
 acc_v1_null_out45_entries_preserved_in_source=True,acc_v1_linked_into_elf='mode_1600_acc_v1' in symbols,
 acc_v1_note='未参照のためgcされた場合はソースの既存NULL枠と不変差分を照合。',host_validation=host,build=build,
 elf_sha256=sha(saved_elf),source_sha256=sha(ROOT/cpp),firmware_write_performed=False,
 new_hardware_test_performed=False,additional_execution_commands=0,left_hardware_tested=False,
 input_sha256={str(p):sha(p) for p in (OUT/'tuning_selected.settings.json',OUT/'tuning_decision.json',Path(__file__))})
with (OUT/'verification.json').open('x',encoding='utf-8') as f:json.dump(result,f,ensure_ascii=False,indent=2)
with (OUT/'README.md').open('a',encoding='utf-8') as f:
 f.write('\nDebug-MouseBのConfigure/ビルド成功。既存パラメータリンク検証はA/B実行・1テスト成功・skip0。生成ELFで左右out45の補正−0.9/+0.9度、半径±60mm、Lstart21.8/Lend16.5mm、PID/FF保持、mode_1600/acc_v2の接続を確認。in45の値/参照も維持。未参照のacc_v1はELFから除去されており、ソース比較でout45のNULL枠維持を確認。ELF/mapを一式保存し、this_task.diffは今回だけの差分。実機接続・書込み・追加走行なし。\n')
state_path=BASE/'resume_state.json';state=read(state_path)
assert state['physical_power_state']=='BOTH_OFF_USER_CONFIRMED' and state['execution_commands_sent']==3
assert state['next_request_id'] is None and not state['execution_pending']
state.update(state='OUT45_LR_SOURCE_APPLIED_DEBUG_B_BUILT_BOTH_OFF',firmware_out45_lr_adoption_record=str(OUT/'verification.json'),
 updated_firmware_elf=str(saved_elf),updated_firmware_elf_sha256=result['elf_sha256'],
 source_params_changed=True,selected_correction_applied_to_firmware_source=True,firmware_write_performed=False,
 next_action='B1600左右out45の暫定値をソース反映し、Bビルド/ELF/リンク検証を完了。実機書込みと左/連続走行は未実施。両方OFF、追加走行予約なし。')
state_path.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(passed=True,mouse='B',right_correction=-.9,left_correction=.9,lengths=[21.8,16.5],
 build='PASS',host_tests='A/B PASS skip0',ELF_refs_verified=True,firmware_written=False,elf_sha256=result['elf_sha256']),ensure_ascii=False))
