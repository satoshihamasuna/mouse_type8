"""右V90の暫定反映に検証結果と保存前の根拠を追加する。実機操作なし。"""
from pathlib import Path
from datetime import datetime, timezone
import hashlib, json, re, shutil, struct, subprocess

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT/'tools/logs/pixel_camera/turn1600_B_rv90_20261120_v1'
OUT = ROOT/'tools/turn_analysis/20261121_B_turn1600_RV90_adoption'
CHECK = BASE/'source_validation_20261121'
read = lambda p: json.loads(p.read_text(encoding='utf-8'))
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
def save(path, value, new=True):
    with path.open('x' if new else 'w', encoding='utf-8') as f:
        json.dump(value, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.write('\n')

evidence = read(OUT/'evidence.json')
run = Path(evidence['source_trial']).parent
assert not (run/'archive.json').exists(), '保存済みの試行は変更しない'
records = {action: read(CHECK/(action+'.json')) for action in ('configure','build','linkage')}
assert all(r['returncode']==0 and r['source_unchanged_during_check'] for r in records.values())
assert 'Ran 1 test' in (CHECK/'linkage.log').read_text(encoding='utf-8')
assert 'skipped' not in (CHECK/'linkage.log').read_text(encoding='utf-8').lower()
for path, values in evidence['source_changes'].items():
    assert sha(ROOT/path)==values['after_sha256']

gcc = Path('C:/Users/sato1/AppData/Local/stm32cube/bundles/gnu-tools-for-stm32/14.3.1+st.2/bin')
elf = ROOT/'build/Debug-MouseB/mouse_type8.elf'
symbols = subprocess.check_output([str(gcc/'arm-none-eabi-nm.exe'),'-C',str(elf)],text=True)
def address(name):
    matches = re.findall(r'^([0-9a-f]+) R '+name+r'$', symbols, re.M)
    assert len(matches)==1, name
    return int(matches[0],16)
ffaddr, tableaddr, paramaddr = [address(n) for n in ('ff_gain_turnRV90_1600','slalom_RV90_1600_table','param_RV90_1600')]
def bytes_at(addr, size):
    output = subprocess.check_output([str(gcc/'arm-none-eabi-objdump.exe'),'-s','-j','.rodata',
        f'--start-address={addr}',f'--stop-address={addr+size}',str(elf)],text=True)
    memory = {}
    for line in output.splitlines():
        parts = line.split()
        if not parts or not re.fullmatch('[0-9a-f]{7,8}',parts[0]): continue
        at = int(parts[0],16)
        for part in parts[1:5]:
            if not re.fullmatch('[0-9a-f]{8}',part): break
            for byte in bytes.fromhex(part): memory[at]=byte; at+=1
    return bytes(memory[i] for i in range(addr,addr+size))
ff = list(struct.unpack('<10f',bytes_at(ffaddr,40)))
expected = [0.661202,0.052929,0,0.009256,0.000800,0.000700,0,0,0,0]
assert all(abs(a-b)<1e-7 for a,b in zip(ff,expected))
velo,radius,ls,le,degree,direction,correction = struct.unpack('<5fIf',bytes_at(tableaddr,28))
assert all(abs(a-b)<1e-5 for a,b in zip([velo,radius,ls,le,degree,correction],[1.6,-40,6,17.4,-90,-1.2]))
assert struct.unpack('<4I',bytes_at(paramaddr,16))[3]==ffaddr
validation = dict(recorded_at=datetime.now(timezone.utc).isoformat(),status='PASSED',
    configure=records['configure'],build=records['build'],linkage=records['linkage'],
    linkage_tests=1,linkage_skipped=0,firmware_write_performed=False,
    build_note='STM32バンドルのnative CMakeで同じDebug-MouseBプリセットを検証。ninjaはno work。ELF内の右V90値・専用FF・参照先も照合した。',
    elf=dict(path=str(elf),sha256=sha(elf),mtime_utc=datetime.fromtimestamp(elf.stat().st_mtime,timezone.utc).isoformat(),
        ff_address=ffaddr,table_address=tableaddr,param_address=paramaddr,ff_values=ff,
        table=dict(velo=velo,radius=radius,lstart=ls,lend=le,degree=degree,direction=direction,correction=correction),
        right_v90_ff_pointer_verified=True))
save(OUT/'validation.json',validation)
evidence.update(validation_state='PASSED',validation=str(OUT/'validation.json'))
save(OUT/'evidence.json',evidence,new=False)
readme = OUT/'README.md'
text = readme.read_text(encoding='utf-8')
old = '反映後のB向けDebugビルドと、実パラメータの共有リンク検証を実行する。結果は`validation.json`へ記録する。'
assert text.count(old)==1
text = text.replace(old,'反映後の`Debug-MouseB` Configure/Buildと、A/B実パラメータの共有リンク検証1件が成功（スキップ0）。Buildは`ninja: no work to do`で、ELF内の右V90テーブル・専用FF値・参照先も照合した。結果は[validation.json](validation.json)に記録した。実機へのFlash書込みは行っていない。')
readme.write_text(text,encoding='utf-8')

snapshot = read(run/'analysis_sources/module_hashes_before_analysis.json')
post = {name:sha(ROOT/name) for name in snapshot['modules']}
save(run/'analysis_sources/module_hashes_after_analysis.json',dict(checked_at=datetime.now(timezone.utc).isoformat(),
    modules=post,unchanged=post==snapshot['modules'],before=snapshot['modules']))
assert post==snapshot['modules'], '解析中の共有実装変更は再確認が必要'
save(run/'relative_geometry_review.json',dict(status=evidence['status'],
    independent_optical_geometry_quality_passed=True,relative_geometry_goal_passed=False,
    entry_relative_exit_deg=evidence['entry_relative_exit_deg'],Lend_residual_norm_mm=evidence['Lend_residual_norm_mm'],
    Lend_residual_mm=evidence['Lend_residual_mm'],entry_board_direction_deg=evidence['entry_board_direction_deg'],
    exit_board_direction_deg=evidence['exit_board_direction_deg'],
    coordinate_scope='height-corrected marker midpoint aligned to measured entry; board fixed angles retained separately',
    operator_contact_detected=False,operator_both_powered_off=True,
    automatic_precise_adoption_allowed=False,manual_provisional_reflection=True,firmware_write_performed=False))
dest=run/'analysis_sources/provisional_source_reflection'
dest.mkdir(exist_ok=False)
for name in ('README.md','evidence.json','validation.json','source_patch.diff','source_changes.json','candidate.settings.json','midpoint_review.json','tracking_review.json','correction_bracket.json'):
    shutil.copy2(OUT/name,dest/name)
for action in records:
    for suffix in ('.json','.log'): shutil.copy2(CHECK/(action+suffix),dest/(action+suffix))
state=read(BASE/'resume_state.json')
state.update(state='RV90_PROVISIONAL_SOURCE_VALIDATED_BOTH_OFF',validation=str(OUT/'validation.json'),
    power_state='BOTH_OFF_USER_CONFIRMED',latest_power_state_report='接触なし・BとB用HY0020の両方OFF',
    current_settings=str(run/'saved_1ms.settings.json'),current_archive_folder=None,current_archive_verified=False,
    next_action='試行一式の保存・照合後、同じ値の独立再測定。新しい配置とON・手離しの確認を待つ。')
save(BASE/'resume_state.json',state,new=False)
print(json.dumps(dict(validation='PASSED',elf_parameters_verified=True,analysis_modules_unchanged=True,run=str(run),hardware_operations=0)))
