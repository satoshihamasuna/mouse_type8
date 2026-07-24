# 解析ツール

## Lstart/Lendを新しく測定する

### 測定条件

1. 対象速度・対象ターンの`Lstart`と`Lend`を左右とも`0.0`にする。
2. 赤色を機体前、黄色を機体後ろのマーカーとして取り付ける。
3. 90 mm角のArUco較正領域が映る固定カメラで撮影する。
4. 1ターンにつき動画と`myshell_debug`ログを3本ずつ取得する。
5. 動画とログは同じ順番で取得し、時刻とターン名の両方で対応を確認する。
6. スローモーション倍率を記録する。倍率が既知なら必ず
   `--slow-factor`で固定する。

測定時の`.settings.json`も保存する。解析スクリプトは設定内の
`lstart`または`lend`が0でなければ停止する。

### manifestを作る

解析結果のディレクトリに`manifest.csv`を作る。

```csv
video,log_stem,motion,target_lateral_mm,target_forward_mm
PXL_..._turn90.mp4,20260725_..._myshell_debug_log_long_r90,long_r90,-90,90
```

- `video`: 動画ディレクトリからのファイル名
- `log_stem`: `tools/logs`内の`.csv`と`.settings.json`から拡張子を除いた名前
- `motion`: ログのターン名
- `target_lateral_mm`: 撮影盤面上で左が正、右が負
- `target_forward_mm`: 初期進行方向が正

標準的な目標座標は次のとおり。

| motion | target_lateral_mm | target_forward_mm |
| --- | ---: | ---: |
| long_r90 | -90 | 90 |
| long_l90 | 90 | 90 |
| long_r180 | -90 | 0 |
| long_l180 | 90 | 0 |
| in_r45 | -45 | 90 |
| in_r135 | -90 | 45 |
| out_r45 | -90 | 45 |
| out_r135 | -90 | -45 |
| r_v90 | -90 | 0 |

右ターンだけを測定する場合、採用したLstart/Lendを左ターンにも使う。
左右を別々に測定した場合は、manifestでも`r`と`l`を分けて集計する。

### 全ターンを解析する

PowerShellでリポジトリルートから実行する。

```powershell
python tools/analysis/lzero_turn_lengths.py `
  --manifest tools/turn_analysis/1800/video_YYYYMMDD_lzero/manifest.csv `
  --video-root C:\path\to\video_turn1800 `
  --output tools/turn_analysis/1800/video_YYYYMMDD_lzero `
  --speed 1800 `
  --workers 3
```

倍率が4倍など既知の場合は固定する。

```powershell
python tools/analysis/lzero_turn_lengths.py `
  --manifest tools/turn_analysis/1400/video_YYYYMMDD_lzero/manifest.csv `
  --video-root C:\path\to\video_turn1400 `
  --output tools/turn_analysis/1400/video_YYYYMMDD_lzero `
  --speed 1400 `
  --slow-factor 4 `
  --workers 3
```

`--slow-factor`を省略すると6.5–9.5倍の範囲で同期倍率を探索する。
採用条件の既定値はマーカー検出率95%以上、採用倍率7.5–9.0倍、
動画―ログ軌跡RMSE 8 mm以下である。撮影条件や高速時の滑りに合わせて
変更する場合は、結果の再現性を確認したうえで次を使う。

```text
--min-detection-pct 90
--min-slow-factor 7 --max-slow-factor 9
--max-rmse-mm 10
```

### ターンをひとつずつ解析する

`--motion`を指定すると、manifest内の該当ターンだけを処理する。
出力をターン別ディレクトリに分ければ、全体の集計結果を上書きしない。

```powershell
python tools/analysis/lzero_turn_lengths.py `
  --manifest tools/turn_analysis/1800/video_YYYYMMDD_lzero/manifest.csv `
  --video-root C:\path\to\video_turn1800 `
  --output tools/turn_analysis/1800/video_YYYYMMDD_lzero/by_motion/long_r90 `
  --speed 1800 `
  --motion long_r90 `
  --workers 3
```

`--motion`は複数回指定できる。

```text
--motion long_r90 --motion long_r180
```

追跡済みの結果を意図的に再計算するときは`--force-motion long_r90`、
全ランを再計算するときは`--force`を付ける。

### 出力と採用値

- `README.md`: 採用数、中央値、動画範囲、品質条件
- `adopted_summary.csv`: パラメータへ反映するLstart/Lend
- `run_results.csv`: 全ランの倍率、検出率、RMSE、動画解、ログ解
- 各ランの`trajectory_overlay.jpg`: 動画軌跡とログ軌跡
- 各ランの`marker_log_comparison.png`: 同期、速度、位置誤差
- 各ランの`metrics.json`: 詳細な同期・検出指標

採用値は品質条件を通過した動画解の中央値である。負の解は実装できないため、
`adopted_summary.csv`では0 mmへ制約され、READMEにも記録される。
long180だけは始終方向が平行で連立方程式からLstart/Lendを一意に分離できない。
旋回軌跡の前進頂点を95 mmに置いてLstartを決め、終点からLendを決める。

`Params/Param_A/turn_<speed>.h`へ小数第2位で反映し、右だけを測定した速度は
左右を同値にする。左右を測定した速度は方向別に反映する。最後にSTM32ビルドを行う。

### 使用するスクリプト

- `lzero_turn_lengths.py`: manifestの検証、対象ターンの選択、全ランの解析・集計
- `trim_motion_video.py`: 元動画から実走行区間を切り出す
- `video_marker_pose.py`: ArUco較正、赤・黄マーカー追跡、ログ同期

実例は次を参照。

- [`../turn_analysis/1400/video_20260725_lzero/README.md`](../turn_analysis/1400/video_20260725_lzero/README.md)
- [`../turn_analysis/1600/video_20260724_lzero/README.md`](../turn_analysis/1600/video_20260724_lzero/README.md)
- [`../turn_analysis/1800/video_20260725_lzero/README.md`](../turn_analysis/1800/video_20260725_lzero/README.md)
- [`../turn_analysis/2000/video_20260725_lzero/README.md`](../turn_analysis/2000/video_20260725_lzero/README.md)

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
