# 動画からの Lstart/Lend 測定条件

詳細な座標表と実行例は [解析ツール README](../../../../tools/analysis/README.md) を参照する。

## 入力と対応

- 対象速度・ターンで Lstart=Lend=0 として取得した、赤が機体前・黄が後ろの動画を使う。
- 固定カメラに 90 mm 角の ArUco 較正領域を映し、1 ターンにつき動画と myshell_debug ログを 3 本ずつ取得する手順が基本。
- 保存済み動画を解析する場合は測定条件を記録から確認する。実機の設定変更や撮影は解析から独立した作業として扱う。
- `.settings.json` の `parameters.lstart` / `lend` がゼロであることを確認する。現実装は設定ファイル欠落時に停止しないため、正常終了だけを条件確認の証拠にしない。
- `manifest.csv` の列は `video,log_stem,motion,target_lateral_mm,target_forward_mm`。
- 動画と CSV・設定 JSON を時刻・ターン名で対応付ける。`log_stem` は一意で、CSV・設定 JSON は現在の実装では `tools/logs/` を参照する。
- 目標座標は撮影盤面の左が正、右が負、初期進行方向が正。斜め開始では初期方向への変換も確認する。
- 対応 motion は現状 `long_r90`、`long_l90`、`long_r180`、`long_l180`、`in_r45`、`in_r135`、`out_r45`、`out_r135`、`r_v90`。左斜め系を名前の置換だけで処理しない。

## 実行と品質

- `--manifest`、`--video-root`、`--output`、`--speed` を明示する。スクリプト内の過去実験用既定パスを流用しない。
- 対象を絞る場合は `--motion` を使い、出力も分ける。キャッシュを再利用する場合は入力・設定・解析条件の一致を確認する。
- スローモーション倍率が既知なら `--slow-factor` で固定する。省略時は 6.5–9.5 倍を探索し、既定の採用範囲は 7.5–9.0 倍。
- 既定の品質条件はマーカー検出率 95% 以上、動画―ログ軌跡 RMSE 8 mm 以下。固定倍率の場合は指定倍率との一致で判定する。
- 1 motion につき採用ランが 2 本未満ならスクリプトは停止する。閾値を変更する場合は変更理由・感度・再現性を残す。
- OpenCV の `cv2.aruco.ArucoDetector` が利用できる環境を確認する。較正領域、マーカー追跡、同期、動画とログの軌跡を目視でも確認する。

## 解の解釈と成果物

- 採用値は品質を満たした動画解の中央値。ログ解は比較用で、同じ真値の独立測定とは扱わない。
- 負の解は `adopted_summary.csv` で 0 mm へ制約される。生の中央値・範囲と制約の有無を併記する。
- long180 は始終方向が平行で、終点だけから Lstart/Lend を一意に分離できない。現方式は旋回軌跡の前進頂点を 95 mm に置く追加条件を使う。
- 右だけの測定値を左にも使う場合は対称性の仮定であると明記する。左右を実測した場合は方向別に集計し、`--side-policy` を実際の方針へ合わせる。
- `manifest.csv`、`run_results.csv`、`adopted_summary.csv`、README、各ランの `metrics.json`、`trajectory_overlay.jpg`、`marker_log_comparison.png` を対応付けて保存する。
- Params への反映も依頼されている場合は、機体に応じた `Params/Param_A/turn_<speed>.h` または `Params/Param_B/turn_<speed>.h` を確認する。A の採用値を B へコピーしない。
