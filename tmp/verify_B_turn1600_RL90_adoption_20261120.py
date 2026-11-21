"""生成ELFの左右90度定数を実機設定と照合し、反映・検証状態を保存する。書込みなし。"""
from pathlib import Path
import hashlib
import json
import math
import re
import struct
import subprocess

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_r90_20261120_v1'
OUT=BASE/'firmware_RL90_adoption_20261120_v1'
ELF=ROOT/'build/Debug-MouseB/mouse_type8.elf'
TOOLCHAIN=Path('C:/Users/sato1/AppData/Local/stm32cube/bundles/gnu-tools-for-stm32/14.3.1+st.2/bin')
source=ROOT/'Params/Param_B/turn_1600/turn_1600.cpp'
record=json.loads((OUT/'adoption.json').read_text(encoding='utf-8'))
source_bytes=source.read_bytes()
source_digest=hashlib.sha256(source_bytes).hexdigest()
table_pattern=rb'^const t_turn_param_table (\w+) = ([^\r\n]+);'
before_tables=dict(re.findall(table_pattern,(OUT/'turn_1600.cpp.before').read_bytes(),re.M))
current_tables=dict(re.findall(table_pattern,source_bytes,re.M))
own_names={name.encode() for name in record['changed_tables']}
concurrent_updates={name.decode():dict(before=before_tables[name].decode(),current=value.decode())
    for name,value in current_tables.items() if name not in own_names and before_tables.get(name)!=value}
cache=(ELF.parent/'CMakeCache.txt').read_text(encoding='utf-8')
assert re.search(r'^MOUSE_TARGET:\w+=B$',cache,re.M)
data=ELF.read_bytes()
assert data[:6]==b'\x7fELF\x01\x01'
section_offset=struct.unpack_from('<I',data,32)[0]
section_size,section_count=struct.unpack_from('<HH',data,46)
assert section_size>=40
sections=[struct.unpack_from('<10I',data,section_offset+index*section_size) for index in range(section_count)]
names=subprocess.check_output([str(TOOLCHAIN/'arm-none-eabi-nm.exe'),'-C','--defined-only',str(ELF)],text=True)
settings=json.loads(Path(record['right_input_settings']).read_text(encoding='utf-8'))['parameters']
tables={}
for direction in ('L','R'):
    symbol=f'slalom_{direction}90_1600_table'
    address=int(re.search(r'^([0-9a-fA-F]+) R '+symbol+r'$',names,re.M)[1],16)
    section=next(section for section in sections if section[3]<=address and address+28<=section[3]+section[5] and section[1]==1)
    values=struct.unpack_from('<5fIf',data,section[4]+address-section[3])
    sign=1 if direction=='R' else -1
    expected=[settings['velo'],settings['r_min']*sign,settings['lstart'],settings['lend'],settings['degree']*sign]
    assert all(math.isclose(value,want,rel_tol=0,abs_tol=1e-5) for value,want in zip(values[:5],expected))
    assert math.isclose(values[6],settings['correction']*sign,abs_tol=1e-6)
    tables[symbol]=dict(address=hex(address),velo=values[0],r_min=values[1],Lstart=values[2],Lend=values[3],
        degree=values[4],turn_direction_enum=values[5],degree_correction=values[6],motion_degree=values[4]+values[6])
record.update(build_status='PASS',configure_preset='Debug-MouseB',build_preset='Debug-MouseB',
    cmake='4.3.1+st.1 (bundled executable; wrapper unavailable)',arm_gcc='14.3.1+st.2',
    host_validation=dict(test='test_run_param_linkage.py',status='PASS',tests=1,mice=['A','B'],skips=0),
    elf=str(ELF),elf_sha256=hashlib.sha256(data).hexdigest(),elf_size_bytes=len(data),
    compiled_tables_verified_against_recorded_candidate=True,compiled_tables=tables,
    source_sha256_verified_at_build=source_digest,concurrent_active_table_changes=concurrent_updates,
    concurrent_changes_preserved=True,
    firmware_write_performed=False,left_physical_run_verified=False)
assert hashlib.sha256(source.read_bytes()).hexdigest()==source_digest
assert hashlib.sha256(ELF.read_bytes()).hexdigest()==record['elf_sha256']
(OUT/'turn_1600.cpp.after_combined').write_bytes(source_bytes)
(OUT/'adoption.json').write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
state_path=BASE/'resume_state.json'
state=json.loads(state_path.read_text(encoding='utf-8'))
state.update(state='FIRMWARE_RL90_SOURCE_ADOPTED_BUILT_BOTH_OFF',source_params_changed=True,
    source_adoption_user_requested=True,source_adoption_record=str(OUT/'adoption.json'),
    source_adoption_readme=str(ROOT/'tools/turn_analysis/20261120_B_turn1600_R90_adoption/README.md'),
    firmware_write_performed=False,updated_firmware_elf=str(ELF),updated_firmware_elf_sha256=record['elf_sha256'],
    left_value_source='mirrored from right at user request; left physical run unverified',
    execution_pending=False,next_request_sent=False,pending_request_id=None,pending_request_sent=False,
    next_action='B R/L90 source values adopted and Debug-MouseB ELF built. Both remain OFF; no pending connection or motion. MCU write and left run not performed.')
state['firmware_source_hashes_unchanged']['Params/Param_B/turn_1600/turn_1600.cpp']=False
state['firmware_source_hashes_note']='B turn1600 intentionally changed after the three-run review at explicit user request; archived trial-source evidence remains untouched.'
state_path.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(build='PASS',host_validation=record['host_validation'],elf_sha256=record['elf_sha256'],tables=tables),ensure_ascii=False))
