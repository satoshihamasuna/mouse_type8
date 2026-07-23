# 1600 mm/s Lstart/Lend実測

## 基本方針

`Lstart=0`、`Lend=0`で走行し、動画で測定した旋回中の接地軌跡から
必要な前後直進距離を求める。速度・横滑りを仮定した軌跡モデルは
Lstart/Lendの決定には使用しない。

動画の赤・黄マーカーで機体中心と姿勢を測定し、ログの
`ideal.rad_velo`から角速度プロファイルの開始・終了を特定する。
動画とログを時間同期したうえで、旋回本体の実変位にLstartとLendを加え、
指定終点へ到達する連立方程式を解く。

long180は始終方向が平行でLstartとLendを一意に分離できないため、
旋回軌跡の頂点を開始点から95 mmに置く条件を追加する。

## 測定手順

1. 対象ターンのLstartとLendを左右とも0 mmにする。
2. 4隅のArUcoと、機体の赤・黄マーカーが見える状態で右旋回を3回撮影する。
3. 各走行のログCSVと`.settings.json`を保存する。
4. `manifest.csv`へ動画、ログ、ターン種別、目標座標を記入する。
5. `lzero_turn_lengths.py`を実行する。
6. `adopted_summary.csv`の中央値を左右共通値としてパラメータへ反映する。
7. 設定後に実走し、終点と壁距離を確認する。

`.settings.json`のLstartまたはLendが0でない場合、解析スクリプトは停止する。

## 座標と開始姿勢

manifestの目標座標は、開始点を`(0, 0)`とした
`(横方向, 前方向)` [mm]で記述する。右が負、左が正。

| ターン | 右旋回の目標 | 左旋回の目標 | 右旋回の開始姿勢 |
| --- | ---: | ---: | ---: |
| long90 | (-90, 90) | (90, 90) | 0° |
| long180 | (-90, 0) | (90, 0) | 0° |
| in45 | (-45, 90) | (45, 90) | 0° |
| out45 | (-90, 45) | (90, 45) | -45° |
| in135 | (-90, 45) | (90, 45) | 0° |
| out135 | (-90, -45) | (90, -45) | -45° |
| V90 | (-90, 0) | (90, 0) | -45° |

## 実行方法

現在の2026-07-24データを再集計する場合:

```powershell
python tools\analysis\lzero_turn_lengths.py
```

別の動画セットを解析する場合:

```powershell
python tools\analysis\lzero_turn_lengths.py `
  --manifest path\to\manifest.csv `
  --video-root path\to\videos `
  --output tools\turn_analysis\1600\video_YYYYMMDD_lzero
```

既存の個別結果を使わず動画追跡からやり直す場合は`--force`を付ける。
特定ターンだけなら、例えば`--force-motion in_r45`を使用する。

manifestに必要な列:

```text
video,log_stem,motion,target_lateral_mm,target_forward_mm
```

`log_stem`には`.csv`を除いたログ名を指定する。

## 採用判定

次をすべて満たすランを有効とする。

- マーカー検出率95%以上
- 推定スローモーション倍率7.5～9.0
- 動画―ログ軌跡RMSE 8 mm以下

各ターンで有効ランが2本未満の場合は自動結果をそのまま採用せず、
動画、マーカー、ログ対応を確認して再測定する。

## 現在の採用値

2026-07-24の右旋回動画から求め、左右共通値として
`Params/Param_A/turn_1600.h`へ反映した値。

| ターン | Lstart [mm] | Lend [mm] |
| --- | ---: | ---: |
| long90 | 18.42 | 26.31 |
| long180 | 22.68 | 26.33 |
| in45 | 6.46 | 39.86 |
| out45 | 21.74 | 20.46 |
| in135 | 17.43 | 20.56 |
| out135 | 11.13 | 36.50 |
| V90 | 6.49 | 20.83 |

詳細は[`video_20260724_lzero/README.md`](video_20260724_lzero/README.md)、
全ランは[`video_20260724_lzero/run_results.csv`](video_20260724_lzero/run_results.csv)、
動画とログの対応は
[`video_20260724_lzero/manifest.csv`](video_20260724_lzero/manifest.csv)にある。

## 主なコード

- `tools/analysis/lzero_turn_lengths.py`: manifest読込、動画・ログ同期、長さ算出、集計
- `tools/analysis/trim_motion_video.py`: 実走行区間の抽出
- `tools/analysis/video_marker_pose.py`: ArUco較正、マーカー追跡、ログ同期

## Legacy

次は旧方式との比較と回帰テストのためコードだけを残している。
新しいLstart/Lendの決定には使用しない。

- `tools/analysis/encoder_only_turn_lengths.py`
- `tools/analysis/turn1600_variable_speed_lengths.py`
- `tools/analysis/video_turn_overlay.py`
- `tools/analysis/video_turn_simulator_compare.py`

旧方式の生成結果は削除済みで、必要な場合はGit履歴から復元できる。
