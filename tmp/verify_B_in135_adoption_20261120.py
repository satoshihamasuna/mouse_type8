"""B左右in135の差分範囲と実ELFを照合し、採用根拠を保存する。実機操作なし。"""
from pathlib import Path
import difflib
import hashlib
import json
import re
import struct
import subprocess

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr135_20261120_v1'
OUT=BASE/'firmware_IN135_adoption_20261120_v1'
SOURCE=ROOT/'Params/Param_B/turn_1600/turn_1600.cpp'
ELF=ROOT/'build/Debug-MouseB/mouse_type8.elf'
DOC=ROOT/'tools/turn_analysis/20261120_B_turn1600_IN135_adoption'
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
measurement=read(BASE/'same_candidate_three_review_v1/review.json')
assert measurement['valid_same_candidate_runs']==3
settings=measurement['settings'];assert settings['mouse']=='B' and settings['motion_name']=='in_r135'
assert settings['parameters']==dict(velo=1.6,r_min=-42.5,lstart=10.5,lend=17.2,degree=-135.,correction=.4,pre_accel=6.5)
before=(OUT/'turn_1600.before.cpp').read_text(encoding='utf-8-sig');after=SOURCE.read_text(encoding='utf-8-sig')
names=['slalom_inR135_1600_table','slalom_inL135_1600_table']


def other_code(text):
    text=re.sub(r'/\*.*?\*/|//[^\n]*','',text,flags=re.S)
    for name in names:
        text,count=re.subn(r'const\s+t_turn_param_table\s+'+name+r'\s*=\s*\{[^}]+\};','',text)
        assert count==1,name
    return re.sub(r'\s+',' ',text).strip()


assert other_code(before)==other_code(after),'左右in135以外の有効コードに差分があります'
with (OUT/'our_source_change.patch').open('x',encoding='utf-8') as file:
    file.writelines(difflib.unified_diff(before.splitlines(keepends=True),after.splitlines(keepends=True),
        fromfile='turn_1600.before.cpp',tofile=str(SOURCE.relative_to(ROOT))))
with (OUT/'turn_1600.after.cpp').open('x',encoding='utf-8') as file:file.write(after)
cache=(ROOT/'build/Debug-MouseB/CMakeCache.txt').read_text(encoding='utf-8')
entry=lambda key:re.search(r'^'+key+r':[^=]*=(.*)$',cache,re.M)[1]
assert entry('MOUSE_TARGET')=='B' and entry('CMAKE_BUILD_TYPE')=='Debug'
nm=Path(entry('CMAKE_NM'));objdump=Path(entry('CMAKE_OBJDUMP'))
text=subprocess.check_output([str(nm),'-C','--defined-only',str(ELF)],text=True,encoding='utf-8',errors='replace')
symbols={m[3]:int(m[1],16) for line in text.splitlines() if (m:=re.match(r'^([0-9a-fA-F]+)\s+(\w)\s+(.+)$',line))}


def data_at(address,length):
    output=subprocess.check_output([str(objdump),'-s',f'--start-address={address}',f'--stop-address={address+length}',str(ELF)],
        text=True,encoding='utf-8',errors='replace')
    result=bytearray()
    for line in output.splitlines():
        m=re.match(r'^\s*[0-9a-fA-F]+\s+((?:[0-9a-fA-F]{8}(?:\s+|$)){1,4})',line)
        if m:result.extend(bytes.fromhex(''.join(m[1].split())))
    assert len(result)==length,(address,length,len(result),output)
    return bytes(result)


expected={'slalom_inR135_1600_table':[1.6,-42.5,10.5,17.2,-135.,1,.4],
          'slalom_inL135_1600_table':[1.6,42.5,10.5,17.2,135.,2,-.4]}
compiled={}
fields=('velo_m_s','r_min_mm','Lstart_mm','Lend_mm','degree_deg','turn_dir_enum','correction_deg')
for name,wanted in expected.items():
    actual=struct.unpack('<5fIf',data_at(symbols[name],28))
    assert all(abs(a-b)<1e-5 for a,b in zip(actual,wanted)),(name,actual,wanted)
    compiled[name]=dict(zip(fields,actual));compiled[name]['physical_command_degree_deg']=actual[4]+actual[6]
mode=struct.unpack('<17I',data_at(symbols['mode_1600'],68))
for index,param,table in [(11,'param_inR135_1600','slalom_inR135_1600_table'),(12,'param_inL135_1600','slalom_inL135_1600_table')]:
    assert mode[index]==symbols[param]
    assert struct.unpack('<I',data_at(symbols[param],4))[0]==symbols[table]
build=read(OUT/'build_status.json');assert build['configure']['returncode']==build['build']['returncode']==0
result=dict(status='SOURCE_ADOPTED_AND_DEBUG_MOUSE_B_BUILD_AND_ELF_VERIFIED',record_date='2026-11-20',
    mouse='B',speed_mm_s=1600,user_request=read(OUT/'request.json')['user_request'],
    source=str(SOURCE),source_sha256=sha(SOURCE),source_before_sha256=sha(OUT/'turn_1600.before.cpp'),
    changed_tables=names,other_active_code_unchanged=True,pid_and_feedforward_unchanged=True,
    right_measurement_record=str(BASE/'same_candidate_three_review_v1/review.json'),
    right_measurement_record_sha256=sha(BASE/'same_candidate_three_review_v1/review.json'),
    left_value_source='ユーザー指定により右の半径・論理角・補正を鏡像展開。左の実走は未確認。',
    left_physical_run_verified=False,continuous_and_maze_run_verified=False,
    configure_preset='Debug-MouseB',build_preset='Debug-MouseB',build=build,
    build_tool='既存B CMakeCacheに記録されたSTM32同梱CMake/Ninja/Arm GCC。PATH上にcube-cmakeがないため同じpresetで代用。',
    host_validation=dict(test='test_run_param_linkage.py',tests=1,status='PASS',mice=['A','B'],skips=0),
    elf=str(ELF),elf_sha256=sha(ELF),compiled_tables=compiled,compiled_tables_verified_against_saved_candidate=True,
    compiled_mode_1600_entries_verified=True,firmware_write_performed=False,
    target_scope='B1600左右in135のみ。進行中のout135候補は未反映。')
with (OUT/'adoption.json').open('x',encoding='utf-8') as file:json.dump(result,file,ensure_ascii=False,indent=2,allow_nan=False)
assert not DOC.exists();DOC.mkdir()
lines=['# 機体B・1600 mm/s・左右135度入口のソース反映\n\n',
    '2026-11-20、ユーザーの「in135を反映しておいて，ひだりもね」に従い、右の単体3本で確認した値を機体Bへ反映しました。左はユーザー指定の鏡像展開です。\n\n',
    '| 方向 | 速度[m/s] | 半径[mm] | Lstart[mm] | Lend[mm] | 論理角[deg] | 角度補正[deg] |\n',
    '| --- | ---: | ---: | ---: | ---: | ---: | ---: |\n',
    '| 右135度入口 | 1.60 | −42.5 | 10.5 | 17.2 | −135.0 | ＋0.40 |\n',
    '| 左135度入口 | 1.60 | ＋42.5 | 10.5 | 17.2 | ＋135.0 | −0.40 |\n\n',
    '反映先は[turn_1600.cpp](../../../Params/Param_B/turn_1600/turn_1600.cpp)。物理指令角は右−134.6度・左＋134.6度です。変更前の値は直前コメントと採用記録のソーススナップショットに保持しました。BのPID/FFと他のターン定義は維持しています。\n\n',
    '## 右の採用根拠\n\n',
    '機体B、1600 mm/s、吸引duty650、前走加速度指定6.5 m/s²、後斜め走の加減速度20 m/s²。45 mm格子盤、カメラ高さ1500 mm、前緑・後赤です。各動画の実測入口直線に合わせた相対マーカー中点で評価しました。\n\n',
    '| 試行 | 相対回転量[deg] | Lend右[mm] | Lend前[mm] | 目標位置からの差[mm] | 最低電圧[V] |\n',
    '| --- | ---: | ---: | ---: | ---: | ---: |\n']
for i,row in enumerate(measurement['runs']):
    geom=row['geometry'];point=geom['midpoint_Lend_right_forward_mm'];error=measurement['Lend']['each_residual_norm_mm'][i]
    lines.append(f'| {i+1} | {geom["exit_path_direction_change_deg"]:.3f} | {point[0]:.3f} | {point[1]:.3f} | {error:.3f} | {row["tracking"]["battery_min_v"]:.3f} |\n')
lines += ['\n目標は右90/前135 mm。平均回転量134.747度、最大角度差0.621度、Lend位置差は最大2.194 mmです。全3本で操作者が穴・壁への接触なしと両方OFFを確認しています。1本目は充電前、2/3本目は充電後で、電圧差を記録しています。\n\n',
    '3本目はCSV鮮度判定により録画上限へ退避した元結果を保持し、全2696フレームと録画末尾の静止窓を別に検証しました。元の自動停止summaryの未合格を上書きせず、撮影制御の停止証拠、長い連続CSV・後半動画と操作者確認を区別しています。\n\n',
    '- [同一候補3本の全記録](../../logs/pixel_camera/turn1600_B_inr135_20261120_v1/same_candidate_three_review_v1/review.json)\n',
    '- [3本の相対軌跡](../../logs/pixel_camera/turn1600_B_inr135_20261120_v1/same_candidate_three_review_v1/three_trajectories.png)\n',
    '- [採用値・ソース差分・ビルド・ELF照合の記録](../../logs/pixel_camera/turn1600_B_inr135_20261120_v1/firmware_IN135_adoption_20261120_v1/adoption.json)\n\n',
    '## 検証と未確認事項\n\n',
    '`Debug-MouseB`のConfigure/Buildに成功しました。PATH上に`cube-cmake`がないため、既存BビルドのCMakeCacheに記録された導入済みSTM32同梱CMake/Ninja/Arm GCCを使い、同じpresetを実行しました。\n\n',
    '`python -m unittest discover -s tools/tests -p test_run_param_linkage.py -v`は1件成功、スキップ0件。A/Bそれぞれの実パラメータを複数翻訳単位からコンパイル・リンクしています。生成ELFの左右in135の全7項目と、`mode_1600`の左右参照先も照合しました。今回の有効コード差分が左右in135の2定義に限定されることを確認しています。\n\n',
    '左の実走、更新ファームの実機書き込み後の評価、連続ターン・迷路走行、固定盤面や車軸中心の絶対校正は未確認です。BのターンEKF係数は今回同定・反映していません。進行中のout135候補はこの反映に含めていません。\n']
with (DOC/'README.md').open('x',encoding='utf-8') as file:file.write(''.join(lines))
with (BASE/'same_candidate_three_review_v1/README.md').open('a',encoding='utf-8') as file:
    file.write('\n2026-11-20追記: ユーザー指定でBの左右in135へソース反映しました。左は鏡像・未実走。採用後の状態は[採用記録](../../../../turn_analysis/20261120_B_turn1600_IN135_adoption/README.md)を参照。元の測定時点の未反映記録は履歴として保持しています。\n')
state=read(BASE/'resume_state.json');state.update(source_adopted=True,left_source_adopted_by_user_requested_mirror=True,
    left_physical_run_verified=False,firmware_write_performed=False,firmware_adoption_record=str(OUT/'adoption.json'),
    current_ram_firmware_updated=False,in135_debug_mouse_b_build_verified=True)
(BASE/'resume_state.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(status=result['status'],compiled_tables=compiled,other_active_code_unchanged=True,
    elf_sha256=result['elf_sha256'],host_skips=0,firmware_write_performed=False),ensure_ascii=False),flush=True)
