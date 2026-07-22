# 解析ツール

保存済みログを解析するスクリプトをまとめています。コマンドはリポジトリのルートディレクトリで実行してください。

| スクリプト | 用途 |
| --- | --- |
| `ff_analyzer.py` | 走行ログからフィードフォワード係数を推定 |
| `log_compare.py` | 目標値・測定値・位置軌跡を比較 |
| `search_turn_sp_ff.py` | 探索ターンの並進フィードフォワードを推定 |
| `search_turn_voltage_report.py` | 探索ターンの電圧と応答を解析 |
| `turn_ff_report.py` | ターン別フィードフォワードレポートを生成 |
| `turn_sp_voltage_report.py` | 高速ターン中の並進電圧を解析 |

各コマンドの引数は、次の形式で確認できます。

```powershell
python tools\analysis\ff_analyzer.py --help
```

`ff_analyzer.py` の詳細は [ff_analyzer_README.md](ff_analyzer_README.md) を参照してください。
