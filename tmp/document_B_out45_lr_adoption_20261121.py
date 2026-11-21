"""今回の差分の限定と、反映根拠・未確認範囲を記録する。"""
from pathlib import Path
import json,re
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'tools/turn_analysis/20261121_B_turn1600_OUT45_LR_adoption'
cpp='Params/Param_B/turn_1600/turn_1600.cpp'
before=(OUT/'before'/cpp).read_text(encoding='utf-8');after=(ROOT/cpp).read_text(encoding='utf-8')
def retained(text):
 return [line for line in text.splitlines() if not line.lstrip().startswith('//') and
  not re.match(r'^const t_turn_param_table slalom_out[LR]45_1600_table = ',line)]
assert retained(before)==retained(after),'Other active code changed during this task'
for name in ('Params/Param_B/turn_1600/turn_1600.h','Params/Param_A/turn_1600/turn_1600.cpp'):
 assert (ROOT/name).read_bytes()==(OUT/'before'/name).read_bytes(),name
text='''# 機体B・turn1600：左右out45への暫定値反映

2026-11-21のユーザー指示「右と左にはんえいして」に基づき、右out45の実走済み暫定値をソースへ反映。左は右の符号を反転して展開し、実走未確認。機体A・他ターンは今回変更していない。

| 項目 | 右out45 | 左out45 |
|---|---:|---:|
| 速度 m/s | 1.60 | 1.60 |
| 半径 mm | −60 | ＋60 |
| Lstart mm | 21.8 | 21.8 |
| Lend mm | 16.5 | 16.5 |
| 論理角 deg | −45 | ＋45 |
| 角度補正 deg | −0.9 | ＋0.9 |
| 指令角 deg | −45.9 | ＋45.9 |

速度PID4.0/0.05/0、角速度PID0.20/0.01/0を維持。in/out45共通の `ff_gain_turnIn45_1600` を左右out45も継続参照し、FFは並進速度0.661202・並進加速0.052929・並進バイアス0、角速度0.009256・角加速0.000750・角減速0.000750、角バイアス/jerk/並進ターン加速/速度二乗は0。in45の角速度Ki0.005とは区別する。

変更ファイルは `Params/Param_B/turn_1600/turn_1600.cpp` とルートREADME。制御コードの変更は左右out45の角度補正2値のみで、関連コメントを更新。変更前の作業差分をbefore/に保存し、無関係な実装と既存のコメント履歴を保持している。

## 測定根拠

[三回の暫定決定](../../logs/pixel_camera/turn1600_B_outr45_20261121_v1/selected_m0p9_three_review_v1/README.md)と、その設定・decision・verificationを `tuning_*.json` に保存。B専用BLE E1:7F:86:21:84:73 / 8767、高さ1500mm、前緑/後赤、EV−1.5/1.8倍、吸引650、斜め前走127.279mm・後直進90mm/減速20m/s²。前走加速指定6.5m/s²はシェルで実効10.057m/s²になる。これらの単体取得条件を最短テーブルへ新しい固定条件として追加していない。

右は合計三回（基準＋0.3度、−0.9度二回）、全て再送0、記録欠落0、操作者が両方OFF・接触なしを確認。003は必要区間実観測・実ArUco登録100%、時計半窓差0.491msで相対幾何判定通過。出口45.828度、Lend中点目標差＋1.254/＋2.467mm（ノルム2.767mm）。002の時計半窓差2.134ms>2msは不合格のまま保持し、採用設定の有効反復二本を確認済みとはしない。

## 適用範囲と未確認

入口相対のマーカー中点と進行方向による単体暫定値。減速・制動後の停止点残差約6.93mm、角速度の振れ、絶対車軸/機体yaw/盤面固定の精度、左の実走、連続接続、BのEKF同定は未確認/未解決。ソース反映・生成ELFと実機書込みを区別し、今回の実機接続・書込み・追加走行は行っていない。

## 検証

Debug-MouseBでConfigureとビルド、既存 `test_run_param_linkage.py` のA/Bホスト検証、生成ARM ELFの左右形状・PID/FF参照・mode_1600/acc_v2接続を確認する。結果はbuild_status.json・host_test_status.json・verification.jsonと各stdoutへ保存する。
'''
with (OUT/'README.md').open('x',encoding='utf-8') as f:f.write(text)
print('Only out45 correction values and related comments changed; A/header/other active code preserved')
