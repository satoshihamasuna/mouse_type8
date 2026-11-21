"""依頼された通電待機の運用だけを既存スキル・READMEへ追記し、変更前を保持する。"""
from pathlib import Path
from datetime import datetime,timezone
import hashlib
import json

root=Path(__file__).resolve().parents[1]
backup=root/'tools/logs/pixel_camera/cli_out_r45_aftercharge_20261111_144206/skill_power_idle_update'
backup.mkdir(exist_ok=False)
section='''## 電源を入れたまま待たせない

- 通電は接続・設定照合・短い撮影や走行確認に必要な間に絞る。全フレーム動画解析、長いビルド、コード・スキル編集、原因調査や長い返答待ちへ移る前に、走行と手かざし待機の終了、設定・ログ・再開条件の保存を確認し、ユーザーに機体とHY0020の電源OFFを案内する。接続も閉じられる時点で閉じる。
- 撮影や接続の確認に失敗しても、通電したまま再試写と解析を延々と繰り返さない。実機が不要な作業へ切り替える時点でOFFを案内し、進捗報告だけを送りながら電源ONで放置しない。保存済みデータだけを処理する解析の終了まで通電を維持させない。
- BLE切断やIR LED消灯を本体電源OFFと同一視しない。物理的なON/OFFはユーザーに行ってもらい、返答まではOFF未確認として記録する。OFFの返答待ち中も、電源不要の解析・編集は進めてよい。
- 次の実機操作に必要な準備を終えてからONを依頼する。再投入後は電圧と実機値を読み返し、消失するRAM設定を保存済み候補から復元・照合して、開始位置・向き・手離しなどの現在の設置状態を確認する。終了・中断時もOFFを案内し、電源の確認状態、未送信／送信済み指令、復元する設定と次の再開条件を残す。電源再投入を実行指令の再送理由にしない。

'''
analysis_note='''実機取得の後に時間のかかる解析へ移る場合は、[電源を入れたまま待たせない手順](../mouse-turn-tuning/SKILL.md#電源を入れたまま待たせない)に従い、停止・資料保存を確認して電源OFFを案内する。保存済み動画やCSVの解析のために通電を維持しない。

'''
readme_note='''長時間の解析・編集や返答待ちは、機体とHY0020の電源をOFFにして行います。走行・手かざし待機の終了と設定・CSV・動画の保存を確認した時点でOFFを案内し、接続も閉じられる時点で閉じます。`--no-wait`やBLE切断だけでは本体の電源は切れません。保存済みデータの解析が続いていても、そのために通電を維持する必要はありません。電源を再投入するとRAM設定は失われるため、次の測定準備を終えてからONにし、保存済み設定の復元・実機値照合・設置確認を行ってください。

'''
edits=[
    ('.agents/skills/mouse-turn-tuning/SKILL.md','## 作業範囲\n',section+'## 作業範囲\n'),
    ('.agents/skills/mouse-log-analysis/SKILL.md','## 入力を確定する\n',analysis_note+'## 入力を確定する\n'),
    ('tools/pixel_camera_README.md','### 動画・CSV・設定をDドライブにまとめる\n',readme_note+'### 動画・CSV・設定をDドライブにまとめる\n'),
]
records=[]
for relative,old,new in edits:
    target=(root/relative).resolve()
    assert target.is_relative_to(root)
    before=target.read_bytes()
    newline=b'\r\n' if b'\r\n' in before else b'\n'
    anchor=old.encode('utf-8').replace(b'\n',newline)
    replacement=new.encode('utf-8').replace(b'\n',newline)
    assert before.count(anchor)==1,relative
    after=before.replace(anchor,replacement,1)
    before_path=backup/'before'/relative
    before_path.parent.mkdir(parents=True,exist_ok=True)
    before_path.write_bytes(before)
    target.write_bytes(after)
    after_path=backup/'after'/relative
    after_path.parent.mkdir(parents=True,exist_ok=True)
    after_path.write_bytes(after)
    assert after.replace(replacement,anchor,1)==before
    assert after.count(b'\r\n')==after.count(b'\n') if newline==b'\r\n' else b'\r\n' not in after
    records.append(dict(path=relative,before_sha256=hashlib.sha256(before).hexdigest(),after_sha256=hashlib.sha256(after).hexdigest(),newline='CRLF' if newline==b'\r\n' else 'LF',only_requested_insertion=True))
(backup/'changes.json').write_text(json.dumps(dict(recorded_at=datetime.now(timezone.utc).isoformat(),files=records),ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(records,ensure_ascii=False))
