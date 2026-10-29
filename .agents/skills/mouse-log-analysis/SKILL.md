---
name: mouse-log-analysis
description: "mouse_type8 の保存済み走行 CSV・設定 JSON・動画から、FF 係数やターンの Lstart/Lend を解析し、品質条件と根拠を残す。走行ログの比較、FF 調整候補、動画によるターン長測定の依頼で使用する。"
---

# mouse_type8 の走行ログ解析

既存ツールで入力条件を確認してから解析し、推定値と採用できる範囲を示す。
解析依頼だけで `Params/` を変更しない。反映も依頼された場合は対象機体と反映範囲を確認する。

## 入力を確定する

- [解析ツール一覧](../../../tools/analysis/README.md) と対象スクリプトの実装・引数を読む。コマンドはリポジトリルートで実行する。
- CSV、同名の `.settings.json`、機体 A/B、取得時のファームウェア、速度・motion・PID・FF・吸引条件を照合する。
- 古いモータドライバ、異なる機体・設定のログを無条件で混ぜない。設定 JSON に機体の証拠がなければ既存記録を探し、不明と記載する。
- 必須列、数値化後の欠損、行数、`cnt` の連続性、ログ末尾と走行区間を確認する。行番号を実時刻と決めつけない。
- ログ周期は取得時設定・ロギング実装から確定する。制御の積分周期が 1 ms であることだけではログ周期を確定できない。
- `ff_analyzer.py` の既定は 2 ms、`search_turn_sp_ff.py` は 1 ms。指定可能なツールには実際の `--period-ms` を渡す。
- `turn_ff_report.py` と `turn_sp_voltage_report.py` は 1 ms 固定、`video_marker_pose.py` は `cnt * 0.001`。異なる周期の入力を扱う場合は、時間軸を合わせる対応を先に行う。
- 出力は機体・取得日・解析条件を区別できる保存先に置き、入力ログを保持する。

## ツールを選ぶ

| 解析したい内容 | 起点 |
| --- | --- |
| 直進・旋回の基本 FF 係数 | `tools/analysis/ff_analyzer.py` |
| 探索ターンの並進 FF | `tools/analysis/search_turn_sp_ff.py` |
| 探索ターンの電圧と応答 | `tools/analysis/search_turn_voltage_report.py` |
| 高速ターンの旋回 FF・追従 | `tools/analysis/turn_ff_report.py` |
| 高速ターン中の並進電圧 | `tools/analysis/turn_sp_voltage_report.py` |
| 目標・実測・軌跡やオドメトリ比較 | `tools/analysis/log_compare.py`、`tools/analysis/odometry_compare.py` |
| 動画から新しい Lstart/Lend を測定 | `tools/analysis/lzero_turn_lengths.py` |

各ツールの `--help` で引数を確認し、入力と保存先を明示する。
最新ログの自動選択を使う場合も、実際に選ばれたファイルと設定を確認する。
`turn_ff_report.py` の設定別グループ選択は機体を識別しないため、A/B の混在を別途確認する。

## FF 係数を評価する

- [FF 解析手順](../../../tools/analysis/ff_analyzer_README.md) を読む。`ff_analyzer.py` は明示した CSV と `--period-ms`、結果保存用の `--json` を使う。
- 基本モデルの必要電圧は `feedforward + feedback` の近似。旋回モデルの加速・減速の分け方は、対象スクリプトと取得時の制御実装を確認する。
- `ideal.rad_accel` がなければ角速度から微分するため、周期の誤りが係数を直接変える。
- 飽和除外と MAD 外れ値除去の条件、除外数、対象区間を確認する。`V_r`・`V_l`・`Battery` がない場合、既定の飽和除外は働かない。
- RMSE・R² に加え、条件数、励起不足の警告、各相の残留 feedback、加減速・左右の追従と飽和を確認する。
- 条件数が 30 を大きく超える場合や励起不足では、分離できない係数を確定しない。速度一定と独立した加減速の追加ログを提案する。
- `unavailable`、`SKIPPED`、除外・欠損を結果へ残す。直進だけのログから旋回係数を確定しない。
- バイアス候補と `DEAD_V` の役割の重複、壁制御・スリップ・閉ループ同定による偏りを含めて候補値を説明する。

## 動画から Lstart/Lend を求める

[ターン長の条件](references/turn-lengths.md) を読んでから `lzero_turn_lengths.py` を使う。
測定条件・動画との対応・品質条件を満たす動画解を集計し、ログ解と比較する。
`encoder_only_turn_lengths.py`、`turn1600_variable_speed_lengths.py` など Legacy 方式を新しい採用値の決定に使わない。
README の `Param_A` への反映や右から左への複製を、機体 B や別測定へ無条件に適用しない。

## 根拠と結果を保存する

- 入力一覧、機体、取得条件、ログ周期の根拠、実行コマンド、ツールの版・作業差分、除外理由をレポートへ残す。
- 係数 JSON・集計 CSV・比較図と、その値に使ったログや設定を対応付ける。再解析できる保存先を示す。
- 採用可能な候補、識別不能な項目、実機で未確認の事項を分けて示す。解析だけで追従改善や走行安定性を確認済みとしない。
