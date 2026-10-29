---
name: mouse-wall-control
description: "mouse_type8 の直進側壁制御、IR センサの採用判定、壁制御の状態遷移を変更・検証する。一般的な速度 PID 調整や保存済みログの解析だけには使用しない。"
---

# 直進側壁制御の変更

コマンドはリポジトリルートで実行する。変更する現象に関係する実装とテストを選んで読む。

## 実装の起点

| 対象 | 読むファイル |
| --- | --- |
| 制御則・信頼度・phase | [zirconia_wall_control.cpp](../../../Task/Src/zirconia_wall_control.cpp)、[入出力型](../../../Task/Inc/zirconia_wall_control.h) |
| ADC 平均・距離校正・機体状態との接続 | [ir_sens_task.cpp](../../../Task/Src/ir_sens_task.cpp)、[sensing_task.h](../../../Task/Inc/sensing_task.h) |
| 閾値・ゲイン・出力上限 | [zirconia_wall_param.h](../../../Params/zirconia_wall_param.h) |
| 新規直進・モード切替・ターン後の復帰 | `Task/Src/init_motion.cpp`、`init_motion2.cpp`、`motion.cpp`、`turn_motion.cpp` |
| 回帰ケースと実装ハーネス | [test_side_wall_stability.py](../../../tools/tests/test_side_wall_stability.py)、[side_wall_stability_harness.cpp](../../../tools/tests/side_wall_stability_harness.cpp) |

センサ入力を追う場合は `IrSensTask_type8::IrSensorSet()` を確認する。基底クラスの同名関数には固定値があり、実機の ADC 経路とは異なる。

## 接続時に維持する関係

- type8 の `ego.x_point` は左正の mm、Zirconia の横偏差は右正の m。入力は `-x_point / 1000`、推定位置への同期は `-raw_error_m * 1000`。
- `delta_tms` は ms、制御器の `dt_s` は s。姿勢は rad、角速度は rad/s、角加速度は rad/s²。
- 直進壁制御は区間専用のジャイロ積分 `straight_heading_rad` を使う。壁切れ処理による `ego.radian` のリセットへ再結合しない。
- `output.omega_radps` と `alpha_radps2` は直進の目標へ代入する。前周期の目標角速度へ補正を加えると二重積分になる。
- `sample_updated` と移動距離は新規観測に対応させる。観測のない周期に同じ距離を再投入しない。
- 直進制御用の 10 点 ADC 平均と、従来の壁切れ検出用 20 サンプル距離履歴を分離する。迷路用 `is_wall` と制御用 `trusted_mask` を同一視しない。
- 無効化、新規直進、モード切替の `ResetStraightWallControl()` と、ターン後の `PrimeStraightWallControl()` の経路を確認する。制御状態と観測平均のリセット範囲を混同しない。
- 現在は目標速度 0.20 m/s 以下、または平均未準備では直進補正を無効化する。この条件を変更する依頼でなければ維持する。

## 変更と検証

現象が距離校正、観測採用、phase 遷移、制御出力、内側の追従のどこで発生するかを切り分ける。
設定だけの調整と制御則の変更を区別し、必要な箇所へ変更を絞る。

不具合の再現ケースは、実装本体を使う既存ハーネスへ追加・更新する。
関連するケースを選び、左右反転、姿勢制動、角速度・角加速度制限、片側柱と反対壁の継続、
全壁喪失、停止中の未確認、サンプル間隔の違い、推定位置同期、モード復帰を確認する。
ADC 経路を変える場合は機体別校正を通るケース、共通 IR 処理を変える場合は斜め制御の回帰も確認する。

```powershell
python -m unittest discover -s tools/tests -p test_side_wall_stability.py -v
```

このテストは機体 A / B の実際の IR タスクと制御器をホストでコンパイルする。
コンパイラ未検出のスキップと実行成功を区別する。ファームウェア変更後は A / B の Debug ビルドも確認する。
実機評価の進め方とホスト検証の限界は [tools/tests/README.md](../../../tools/tests/README.md) を参照する。

## 結果の読み方

- 直進中の `control_ir` は左正の横偏差 mm、`control_ir_dot` は左向き正の区間姿勢 rad。斜め中は従来の意味。
- 詳細は `straight_wall_output` の phase、採用マスク、transition、飽和状態で確認する。マスクは 1=左、2=右、3=両側。
- `Params/zirconia_wall_param.h` の値は移植元由来の初期値。ホストテスト成功を機体 A / B の実機調整済みという意味にしない。
