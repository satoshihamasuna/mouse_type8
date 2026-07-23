# 解析ツール

## Lstart/Lendの基本手法

1600 mm/sターンのLstart/Lendは、`Lstart=Lend=0`で取得した
マーカー動画と対応ログから求める。

- `lzero_turn_lengths.py`: manifestを読み、全ランを解析・集計する
- `trim_motion_video.py`: 元動画から実走行区間を切り出す
- `video_marker_pose.py`: ArUco較正、赤・黄マーカー追跡、ログ同期を行う

手順、座標、採用条件は
[`../turn_analysis/1600/README.md`](../turn_analysis/1600/README.md)を参照。

## その他の現行ツール

| スクリプト | 用途 |
| --- | --- |
| `ff_analyzer.py` | 走行ログからフィードフォワード係数を推定 |
| `search_turn_sp_ff.py` | 探索ターンの並進フィードフォワードを推定 |
| `search_turn_voltage_report.py` | 探索ターンの電圧と応答を解析 |
| `turn_ff_report.py` | 高速ターンのフィードフォワードレポートを生成 |
| `turn_sp_voltage_report.py` | 高速ターン中の並進電圧を解析 |
| `log_compare.py` | 目標値、実測値、位置軌跡を比較 |
| `odometry_compare.py` | オドメトリ方式を比較 |
| `generate_aruco_print.py` | 動画較正用ArUcoを生成 |

各コマンドの引数は次の形式で確認できる。

```powershell
python tools\analysis\<script>.py --help
```

## Legacy

次のコードは旧Lstart/Lend方式との比較または回帰テストのために残している。
新しいターン長の決定には使用しない。

- `encoder_only_turn_lengths.py`
- `turn1600_variable_speed_lengths.py`
- `video_turn_overlay.py`
- `video_turn_simulator_compare.py`
