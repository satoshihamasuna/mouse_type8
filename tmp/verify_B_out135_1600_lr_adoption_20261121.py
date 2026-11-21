"""B左右out135のELF/参照先を検証し、実測根拠付きの暫定反映記録を保存。"""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json
import math
import re
import struct
import subprocess

root=Path(__file__).resolve().parents[1]
base=root/'tools/logs/pixel_camera/turn1600_B_outr135_20261121_resume_v1'
out=base/'firmware_OUT135_adoption_20261121_v1'
source=root/'Params/Param_B/turn_1600/turn_1600.cpp'
doc=root/'tools/turn_analysis/20261121_B_turn1600_OUT135_adoption'
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def save(path,value):
    with path.open('x',encoding='utf-8') as stream:
        json.dump(value,stream,ensure_ascii=False,indent=2,allow_nan=False);stream.write('\n')

request=read(out/'request.json')
assert not (out/'adoption.json').exists()
assert sha(source)==request['source_after_sha256']
assert sha(Path(request['right_measurement_record']))==request['right_measurement_record_sha256']
measurement=read(Path(request['right_measurement_record']))
build=read(out/'build_status.json')
assert build['configure']['returncode']==build['build']['returncode']==0
for filename in ('mouse_type8.elf','mouse_type8.map'):
    source_artifact=root/'build/Debug-MouseB'/filename
    destination=out/filename
    if not destination.exists():
        destination.write_bytes(source_artifact.read_bytes())
    assert sha(destination)==sha(source_artifact)
elf=out/'mouse_type8.elf'
cache=(root/'build/Debug-MouseB/CMakeCache.txt').read_text(encoding='utf-8')
entry=lambda key:re.search(r'^'+key+r':[^=]*=(.*)$',cache,re.M)[1]
assert entry('MOUSE_TARGET')=='B' and entry('CMAKE_BUILD_TYPE')=='Debug'
nm=Path(entry('CMAKE_NM'));objdump=Path(entry('CMAKE_OBJDUMP'))
output=subprocess.check_output([str(nm),'-C','--defined-only',str(elf)],text=True,encoding='utf-8',errors='replace')
symbols={m[3]:int(m[1],16) for line in output.splitlines() if (m:=re.match(r'^([0-9a-fA-F]+)\s+(\w)\s+(.+)$',line))}

def data_at(address,length):
    text=subprocess.check_output([str(objdump),'-s',f'--start-address={address}',f'--stop-address={address+length}',str(elf)],text=True,encoding='utf-8',errors='replace')
    result=bytearray()
    for line in text.splitlines():
        match=re.match(r'^\s*[0-9a-fA-F]+\s+((?:[0-9a-fA-F]{8}(?:\s+|$)){1,4})',line)
        if match:result.extend(bytes.fromhex(''.join(match[1].split())))
    assert len(result)==length,(address,length,len(result))
    return bytes(result)

compiled={}
fields=('velo_m_s','r_min_mm','Lstart_mm','Lend_mm','degree_deg','turn_dir_enum','correction_deg')
expected={'slalom_outR135_1600_table':[1.6,-41.,5.3,31.,-135.,1,1.],
          'slalom_outL135_1600_table':[1.6,41.,5.3,31.,135.,2,-1.]}
for name,wanted in expected.items():
    actual=struct.unpack('<5fIf',data_at(symbols[name],28))
    assert all(abs(a-b)<1e-5 for a,b in zip(actual,wanted)),(name,actual)
    compiled[name]=dict(zip(fields,actual),physical_command_degree_deg=actual[4]+actual[6])
gain_values={}
for name,wanted in [('sp_gain_turnOut135_1600',[4.,.05,0.]),('om_gain_turnOut135_1600',[.2,.01,0.])]:
    actual=struct.unpack('<3f',data_at(symbols[name],12))
    assert all(abs(a-b)<1e-6 for a,b in zip(actual,wanted))
    gain_values[name]=list(actual)
ff=struct.unpack('<10f',data_at(symbols['ff_gain_default'],40))
expected_ff=list(measurement['settings']['feedforward'].values())
assert len(expected_ff)==10 and all(abs(a-b)<2e-6 for a,b in zip(ff,expected_ff)),(ff,expected_ff)
mode_results={}
for mode_name in ('mode_1600','mode_1600_acc_v1','mode_1600_acc_v2'):
    source_match=re.search(r'const t_param \*const '+re.escape(mode_name)+r'\[\]\s*=\s*\{([^}]+)\}',source.read_text(encoding='utf-8-sig'))
    assert source_match is not None
    source_entries=[item.strip() for item in source_match[1].split(',') if item.strip()]
    assert source_entries[13:15]==['&param_outR135_1600','&param_outL135_1600']
    if mode_name not in symbols:
        assert mode_name!='mode_1600'
        mode_results[mode_name]=dict(right_index=13,left_index=14,source_entries_verified=True,
            elf_symbol_present=False,note='未使用テーブルはリンク時gc-sectionsで除去。ELFでの参照検証は対象外。')
        continue
    mode=struct.unpack('<17I',data_at(symbols[mode_name],68))
    for index,param,table in [(13,'param_outR135_1600','slalom_outR135_1600_table'),(14,'param_outL135_1600','slalom_outL135_1600_table')]:
        assert mode[index]==symbols[param]
        pointers=struct.unpack('<4I',data_at(symbols[param],16))
        assert pointers==(symbols[table],symbols['sp_gain_turnOut135_1600'],symbols['om_gain_turnOut135_1600'],symbols['ff_gain_default'])
    mode_results[mode_name]=dict(right_index=13,left_index=14,source_entries_verified=True,
        elf_symbol_present=True,table_pid_ff_pointers_verified=True)
assert sha(source)==request['source_after_sha256']
result=dict(status='SOURCE_PROVISIONALLY_ADOPTED_DEBUG_MOUSE_B_BUILD_AND_ELF_VERIFIED',
    record_date='2026-11-21',recorded_at=datetime.now(timezone.utc).isoformat(),
    mouse='B',preset_mm_s=1600,user_request=request['user_request'],
    source=str(source),source_sha256=sha(source),source_before_sha256=request['source_before_sha256'],
    changed_tables=request['changed_tables'],other_active_code_unchanged=True,pid_and_ff_unchanged=True,
    right_measurement_record=request['right_measurement_record'],right_measurement_record_sha256=request['right_measurement_record_sha256'],
    adoption_scope='右は同設定3本の入口相対マーカー中点による単体候補。左はユーザー指定の鏡映。暫定ソース反映。',
    left_physical_run_verified=False,continuous_and_maze_run_verified=False,
    measured_statistics=measurement['statistics'],build=build,
    build_tool='既存Debug-MouseBのCMakeCacheに登録されたSTM32同梱CMake/Ninja/Arm GCCで同じpresetを実行。cube-cmakeはPATHにない。',
    host_validation=dict(command='D:/venv/venv3-11/Scripts/python.exe -m unittest discover -s tools/tests -p test_run_param_linkage.py -v',
        tests=1,status='PASS',mice=['A','B'],skips=0,observed_exit_code=0,seconds=4.224),
    elf=str(elf),elf_sha256=sha(elf),compiled_tables=compiled,compiled_pid=gain_values,
    compiled_ff_default=list(ff),compiled_mode_pointer_verification=mode_results,
    firmware_write_performed=False,current_ram_firmware_updated=False,
    unused_refinement_Lend_mm=29.2,unused_refinement_executed=False,
    remaining=['停止点の7.63〜8.57mm差','左の実走','更新FWの実機書込み後の確認','連続接続','BのEKF係数の同定','絶対車軸/機軸の校正'])
save(out/'adoption.json',result)
assert not doc.exists();doc.mkdir()
lines=['# 機体B・1600 mm/s・左右135°出口への暫定反映\n\n',
    '2026-11-21、ユーザーの「反映して，左もね」に従い、右の同設定3本を確認した形状を左右out135へ反映した。左は右測定値の鏡映で、未実走。準備だけだったLend29.2 mm候補は走行せず、測定済み31.0 mmを採った。\n\n',
    '|方向|速度 [m/s]|半径 [mm]|Lstart [mm]|Lend [mm]|論理角 [°]|補正 [°]|\n|---|---:|---:|---:|---:|---:|---:|\n',
    '|右out135|1.60|−41.0|5.3|31.0|−135|＋1.0|\n|左out135|1.60|＋41.0|5.3|31.0|＋135|−1.0|\n\n',
    '反映先は[turn_1600.cpp](../../../Params/Param_B/turn_1600/turn_1600.cpp)。指令角は右−134°・左＋134°。変更前の定義は直前コメントと保存スナップショットに保持した。今回の有効コード差分は左右out135の2定義のみ。既存のB用PID/FFを用いる。\n\n',
    '右の測定条件は速度1.6 m/s、吸引ON duty650、前走加速度指定6.5 m/s²（距離制約で有効約10.057）、斜め前走127.279 mm・直進後走90 mm、後走加速度20 m/s²。壁EKF shadow、壁切れlegacy、ターンEKF control（照会時inactive）。カメラ1500 mm、前緑/後赤、EV−1.5・ズーム1.8倍。\n\n',
    '|試行|入口相対の出口角 [°]|Lend目標差 [mm]|必要区間の実マーカー観測率|最低電圧 [V]|\n|---|---:|---:|---:|---:|\n']
for i,row in enumerate(measurement['runs'],1):
    lines.append(f'|{i}|{row["exit_angle_deg"]:.3f}|{row["Lend_error_mm"]:.3f}|{row["required_observation_fraction"]*100:.3f}%|{row["battery_min_v"]:.3f}|\n')
lines+=['\n平均出口角135.232°、角度の最大目標差0.627°、Lend目標差最大2.339 mm。前後マーカー中点を実測入口方向へ合わせた相対評価で、固定盤面・車軸/機軸の絶対精度や連続接続の合格ではない。全3本で操作者が接触なし・両方OFFを確認。走行中の配信欠落/CRC0・出力飽和0。\n\n',
    '全必要区間479コマを目視し、3本目の床模様/基板への誤対応2点を別解析で無効化した。入口時計・出口直線・Lend補間・停止窓へ影響せず、数値は不変。除外後も観測率95%以上、各フレーム実ArUco登録100%。元の検出不合格、公式summaryのDETAILED_REVIEW_REQUIRED、自動停止未合格や採用制限は保持した。ユーザー指定による暫定ソース反映である。\n\n',
    '停止点の目標差7.63〜8.57 mm、Lend後の実測中点移動95.97〜96.32 mm（後走指令90 mm）は別の残課題として保持する。左実走・連続接続・更新FWの実機確認・BのEKF係数は未確認。今回、実機書込み・BLE接続・追加走行は行っていない。\n\n',
    '`Debug-MouseB` Configure/Buildに成功。既存のSTM32同梱ツールをCMakeCacheから選び、同じpresetを実行した。`test_run_param_linkage.py`は1件PASS・skip0で、A/Bの実パラメータを複数翻訳単位からコンパイル/リンクした。ELFの全7項目、PID/FFと通常mode_1600の左右参照先も照合した。acc_v1/v2はソースの左右参照を確認し、未使用によるリンク時除去のためELF参照検証の対象外とした。生成ELF/mapは採用記録フォルダへ複製しハッシュを保存した。\n\n',
    '- [測定3本と実マーカー照合](../../logs/pixel_camera/turn1600_B_outr135_20261121_resume_v1/geometry_same_candidate_three_review_v2_manual_pairs/README.md)\n',
    '- [ソース差分・ビルド・ELF検証記録](../../logs/pixel_camera/turn1600_B_outr135_20261121_resume_v1/firmware_OUT135_adoption_20261121_v1/adoption.json)\n']
(doc/'README.md').write_text(''.join(lines),encoding='utf-8')
readme=root/'README.md';before=readme.read_bytes();newline=b'\r\n' if before.count(b'\r\n')==before.count(b'\n') else b'\n'
anchor='### 最短走行開始時の予備加速'.encode()
assert before.count(anchor)==1
paragraph=('### 機体B・turn1600の左右135度出口への暫定値反映\n\n'
    '2026-11-21のユーザー指定で、右out135の単体3本を確認した形状を左右へ暫定反映しました。速度1600 mm/s、半径は右−41 mm・左＋41 mm、Lstart 5.3 mm・Lend 31.0 mm、角度補正は右＋1.0度・左−1.0度です。論理角は左右±135度、指令角は右−134度・左＋134度。既存のB用PID・FFを使います。左は鏡映で実走未確認です。\n\n'
    '右の入口相対マーカー中点で平均出口角135.232度、Lend目標差は最大2.339 mm。停止点の7.63〜8.57 mm差、左実走・連続接続・更新FWの実機確認は残っています。ソース反映と実機書込みを区別し、[採用根拠と検証記録](tools/turn_analysis/20261121_B_turn1600_OUT135_adoption/README.md)へ記録しました。\n\n').encode().replace(b'\n',newline)
(out/'README.before.md').write_bytes(before)
assert readme.read_bytes()==before
readme.write_bytes(before.replace(anchor,paragraph+anchor))
state_path=base/'resume_state.json';state=read(state_path)
state.update(state='OUT135_LR_PROVISIONALLY_SOURCE_ADOPTED_BUILD_VERIFIED',
    firmware_adoption_record=str(out/'adoption.json'),source_adopted=True,
    left_source_adopted_by_user_requested_mirror=True,left_physical_run_verified=False,
    out135_debug_mouse_b_build_verified=True,firmware_write_performed=False,current_ram_firmware_updated=False,
    candidate_status='PROVISIONAL_SOURCE_ADOPTION_BY_USER',next_stage=None,next_request_id=None,
    next_action_kind='no_automatic_run_after_source_adoption',next_action='左右out135の暫定ソース反映とBビルド/ELF検証が完了。追加走行・書込みは予定しない。')
state_path.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
with (base/'README.md').open('a',encoding='utf-8') as stream:
    stream.write('\n2026-11-21: ユーザー「反映して，左もね」により、測定済み5.3/31.0mm・補正+1.0°をB1600の右out135へ、鏡映値を左へ暫定ソース反映した。Debug-MouseBビルド・ホスト参照テスト・ELF全項目/参照先を検証済み。29.2mm候補は未走行のまま保留。書込み・追加走行なし。最新状態はfirmware_OUT135_adoption_20261121_v1/adoption.jsonとtools/turn_analysis/20261121_B_turn1600_OUT135_adoption/README.mdを参照。\n')
print(json.dumps(dict(status=result['status'],compiled_tables=compiled,elf_sha256=result['elf_sha256'],
    mode_pointers_verified=list(mode_results),host_tests=1,host_skips=0,firmware_write_performed=False),ensure_ascii=False),flush=True)
