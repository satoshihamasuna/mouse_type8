"""機体Bの左右180度を保存候補・生成ELFと照合し、ソース採用を記録する。"""
from pathlib import Path
from datetime import datetime, timezone, timedelta
import hashlib
import json
import math
import re
import struct
import subprocess

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT/'tools/logs/pixel_camera/turn1600_B_r180_20261120_v1'
OUT = BASE/'firmware_RL180_adoption_20261120_v1'
SOURCE = ROOT/'Params/Param_B/turn_1600/turn_1600.cpp'
ELF = ROOT/'build/Debug-MouseB/mouse_type8.elf'
NM = Path('C:/Users/sato1/AppData/Local/stm32cube/bundles/gnu-tools-for-stm32/14.3.1+st.2/bin/arm-none-eabi-nm.exe')
before = (Path(__file__).parent/'turn_1600.cpp.before').read_bytes()
after = SOURCE.read_bytes()
assert after.count(b'\n') == after.count(b'\r\n'), 'Source CRLF changed'
pattern = rb'^const t_turn_param_table (\w+) = ([^\r\n]+);'
old_tables = dict(re.findall(pattern, before, re.M))
new_tables = dict(re.findall(pattern, after, re.M))
changed = {name for name in new_tables if old_tables.get(name) != new_tables[name]}
assert changed == {b'slalom_L180_1600_table', b'slalom_R180_1600_table'}
# コメントを除いたコードでも、この2定義だけが変更されていることを確認する。
expected = before
for name in changed:
    expected = expected.replace(old_tables[name], new_tables[name])
def code_only(data):
    return re.sub(rb'\s+', b'', re.sub(rb'//[^\r\n]*', b'', data))
assert code_only(expected) == code_only(after), 'Unrelated source change'
cache = (ELF.parent/'CMakeCache.txt').read_text(encoding='utf-8')
assert re.search(r'^MOUSE_TARGET:\w+=B$', cache, re.M)
settings_path = BASE/'final_single_turn_candidate/B_R180_1600_single_turn_candidate.settings.json'
settings = json.loads(settings_path.read_text(encoding='utf-8'))['parameters']
data = ELF.read_bytes()
assert data[:6] == b'\x7fELF\x01\x01'
section_offset = struct.unpack_from('<I', data, 32)[0]
section_size, section_count = struct.unpack_from('<HH', data, 46)
sections = [struct.unpack_from('<10I', data, section_offset+i*section_size) for i in range(section_count)]
names = subprocess.check_output([str(NM), '-C', '--defined-only', str(ELF)], text=True)
tables = {}
for direction, sign, enum in (('R', 1, 1), ('L', -1, 2)):
    symbol = f'slalom_{direction}180_1600_table'
    address = int(re.search(r'^([0-9a-fA-F]+) R '+symbol+r'$', names, re.M)[1], 16)
    section = next(s for s in sections if s[1] == 1 and s[3] <= address and address+28 <= s[3]+s[5])
    values = struct.unpack_from('<5fIf', data, section[4]+address-section[3])
    expected_values = [settings['velo'], settings['r_min']*sign, settings['lstart'], settings['lend'], settings['degree']*sign]
    assert all(math.isclose(v, e, rel_tol=0, abs_tol=1e-5) for v, e in zip(values[:5], expected_values))
    assert values[5] == enum
    assert math.isclose(values[6], settings['correction']*sign, rel_tol=0, abs_tol=1e-6)
    tables[symbol] = dict(zip(('velo_m_s', 'r_min_mm', 'Lstart_mm', 'Lend_mm', 'degree_deg', 'turn_dir_enum', 'correction_deg'), values))
    tables[symbol]['physical_command_degree_deg'] = values[4]+values[6]
record = dict(
    status='SOURCE_ADOPTED_AND_DEBUG_MOUSE_B_BUILD_VERIFIED',
    recorded_at=datetime.now(timezone(timedelta(hours=9))).isoformat(),
    mouse='B', speed_mm_s=1600,
    user_request='ファームに反映，左にもよろしくお願いします．',
    source=str(SOURCE), source_sha256=hashlib.sha256(after).hexdigest(),
    source_before_sha256=hashlib.sha256(before).hexdigest(),
    changed_tables=sorted(n.decode() for n in changed),
    other_active_code_unchanged=True, pid_and_feedforward_unchanged=True,
    right_input_settings=str(settings_path),
    right_input_settings_sha256=hashlib.sha256(settings_path.read_bytes()).hexdigest(),
    measurement_record=str(BASE/'final_single_turn_candidate/candidate.json'),
    left_value_source='右の測定候補からユーザー指定で鏡像展開。左実走は未確認。',
    configure_preset='Debug-MouseB', configure_status='PASS',
    build_preset='Debug-MouseB', build_status='PASS', build_output='ninja: no work to do.',
    build_tool='STM32 bundled CMake 4.3.1+st.1; cube-cmake wrapper unavailable (ecosystem lookup failed)',
    arm_gcc='14.3.1+st.2',
    host_validation=dict(test='test_run_param_linkage.py', tests=1, mice=['A', 'B'], status='PASS', skips=0),
    elf=str(ELF), elf_sha256=hashlib.sha256(data).hexdigest(),
    compiled_tables_verified_against_saved_candidate=True, compiled_tables=tables,
    firmware_write_performed=False, left_physical_run_verified=False,
    continuous_and_maze_run_verified=False,
)
OUT.mkdir(parents=True, exist_ok=True)
assert not (OUT/'adoption.json').exists(), 'Existing adoption record must be preserved'
(OUT/'turn_1600.cpp.before').write_bytes(before)
(OUT/'turn_1600.cpp.after').write_bytes(after)
(OUT/'adoption.json').write_text(json.dumps(record, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
state_path = BASE/'resume_state.json'
state = json.loads(state_path.read_text(encoding='utf-8'))
state.update(state='FIRMWARE_RL180_SOURCE_ADOPTED_BUILT_BOTH_OFF', source_params_changed=True,
    source_adoption_user_requested=True, source_adoption_record=str(OUT/'adoption.json'),
    source_adoption_readme=str(ROOT/'tools/turn_analysis/20261120_B_turn1600_R180_adoption/README.md'),
    firmware_write_performed=False, updated_firmware_elf=str(ELF), updated_firmware_elf_sha256=record['elf_sha256'],
    left_value_source=record['left_value_source'], execution_pending=False,
    next_request_sent=False, pending_request_id=None, pending_request_sent=False,
    next_action='左右180度のソース反映・Debug-MouseBビルド・ELF値照合済み。両方OFF・BLE未接続・保留指令なし。実機書き込み・左実走・連続/迷路検証は未実施。')
hashes = state.get('firmware_source_hashes_unchanged')
if isinstance(hashes, dict):
    hashes['Params/Param_B/turn_1600/turn_1600.cpp'] = False
state['firmware_source_hashes_note'] = '3本の測定後、ユーザー指定でB左右180度をソースへ採用。元の測定時ソース証跡は保持。'
state_path.write_text(json.dumps(state, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
assert SOURCE.read_bytes() == after
assert ELF.read_bytes() == data
print(json.dumps(dict(build='PASS', host_tests=record['host_validation'], compiled_tables=tables, elf_sha256=record['elf_sha256'])))
