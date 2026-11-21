"""ユーザー指定でBの左右90度へ保存済み候補を反映。測定資料と既存差分を保持する。"""
from pathlib import Path
import hashlib
import json
import re
from datetime import datetime,timezone

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_r90_20261120_v1'
OUT=BASE/'firmware_RL90_adoption_20261120_v1'
SOURCE=ROOT/'Params/Param_B/turn_1600/turn_1600.cpp'
README=ROOT/'README.md'
settings=json.loads((BASE/'device_prepare_green_corr1p0_04/actual.settings.json').read_text(encoding='utf-8'))
comparison=json.loads((BASE/'green_correction1p0_three_video_review_v1/video_comparison.json').read_text(encoding='utf-8'))
assert settings['mouse']=='B' and settings['parameters']['lstart']==13.6
assert settings['parameters']['lend']==26.31 and settings['parameters']['correction']==1.
assert comparison['repeat_count']==3 and comparison['all_relative_reviews_eligible']
original=SOURCE.read_bytes()
left=b'const t_turn_param_table slalom_L90_1600_table = {1.60f, 52.0f,12.0f,26.31f, 90.0f,Turn_L,-1.5f};'
right=b'const t_turn_param_table slalom_R90_1600_table = {1.60f,-52.0f,12.0f,26.31f,-90.0f,Turn_R, 1.5f};'
assert original.count(b'\n'+left)==1 and original.count(b'\n'+right)==1
newline=b'\r\n' if b'\r\n' in original else b'\n'
OUT.mkdir(exist_ok=False)
for path in (SOURCE,README):
    (OUT/(path.name+'.before')).write_bytes(path.read_bytes())
updated=original.replace(left,b'// '+left+newline+left.replace(b'12.0f',b'13.6f').replace(b'-1.5f',b'-1.0f'))
updated=updated.replace(right,b'// '+right+newline+right.replace(b'12.0f',b'13.6f').replace(b' 1.5f',b' 1.0f'))
old='// 速度[m/s]、半径・Lstart・Lend[mm]、角度・角度補正[deg]。Bでの実走は未確認。'.encode('utf-8')
new='// 速度[m/s]、半径・Lstart・Lend[mm]、角度・角度補正[deg]。Bでの追加調整は各定義の記録を参照。'.encode('utf-8')
assert updated.count(old)==1
updated=updated.replace(old,new)
anchor=b'//k = 250'+newline+b'const t_pid_gain sp_gain_turn90_1600'
comment=newline.join(line.encode('utf-8') for line in [
    '// 2026-11-20 B右90度・1600 mm/s・吸引duty650の単体3本を反映。Lstart/Lend = 13.6/26.31 mm。',
    '// 補正は右+1.0度、左はユーザー指定の鏡像-1.0度（左の実走・連続接続は未確認）。',
    '// 根拠: tools/turn_analysis/20261120_B_turn1600_R90_adoption/README.md'])
assert updated.count(anchor)==1
updated=updated.replace(anchor,comment+newline+anchor)
pattern=rb'^const t_turn_param_table (\w+) = ([^\r\n]+);'
before_tables=dict(re.findall(pattern,original,re.M))
after_tables=dict(re.findall(pattern,updated,re.M))
changed=[name.decode() for name in before_tables if before_tables[name]!=after_tables[name]]
assert set(changed)=={'slalom_L90_1600_table','slalom_R90_1600_table'}
SOURCE.write_bytes(updated)
readme=README.read_bytes()
end=b'\r\n' if b'\r\n' in readme else b'\n'
anchor='### 最短走行開始時の予備加速'.encode('utf-8')
section=['### 機体B・turn1600の左右90度への調整値反映','',
    '2026-11-20のユーザー依頼で、右90度の単体調整値を機体Bの形状テーブルへ反映し、左90度にも鏡像展開しました。',
    '旧定義はコメントで保持しています。PID・FFは機体Bの既存設定です。','',
    '| 方向 | 速度[m/s] | 半径[mm] | Lstart[mm] | Lend[mm] | 論理角[deg] | 角度補正[deg] |',
    '| --- | ---: | ---: | ---: | ---: | ---: | ---: |',
    '| 右90度 | 1.60 | −52.0 | 13.6 | 26.31 | −90.0 | ＋1.0 |',
    '| 左90度 | 1.60 | ＋52.0 | 13.6 | 26.31 | ＋90.0 | −1.0 |','',
    '右は吸引duty650の単体3本で相対旋回角88.90～89.97度、出口直線ずれ−0.31～＋1.08 mm。',
    '開始位置を厳密にそろえず、各動画の入口直線を基準にした比較です。左はユーザー指定の同値展開で実走未確認です。',
    'ソース反映と更新ファームの書き込み後の確認・連続接続は区別します。[反映根拠と検証](tools/turn_analysis/20261120_B_turn1600_R90_adoption/README.md)。','']
assert readme.count(anchor)==1
README.write_bytes(readme.replace(anchor,end.join(line.encode('utf-8') for line in section)+end+anchor))
record=dict(at=datetime.now(timezone.utc).isoformat(),mouse='B',changed_tables=changed,
    user_authorized_source_adoption=True,left_value_source='mirrored from right at user request; not measured',
    source_sha256_before=hashlib.sha256(original).hexdigest(),source_sha256_after=hashlib.sha256(updated).hexdigest(),
    right_input_settings=str(BASE/'device_prepare_green_corr1p0_04/actual.settings.json'),
    right_comparison=str(BASE/'green_correction1p0_three_video_review_v1/video_comparison.json'),
    other_turn_tables_unchanged=True,firmware_write_performed=False,build_status='pending')
(OUT/'adoption.json').write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(record,ensure_ascii=False))
