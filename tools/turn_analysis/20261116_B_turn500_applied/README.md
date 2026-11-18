# 機体B turn500の試験反映（2026-11-16）

ユーザー依頼により、機体Bの右7種類の保存済み単体候補を`Params/Param_B/turn_500.h`へ反映した。
対応する左右の前距離（Lstart）・後距離（Lend）・旋回PIDを共通とし、左は右の半径・論理角・角度補正の符号を反転した。
左右14種類は`mode_500`から参照され、単体GUIの500プリセット、turnpattern、最短`plain500`がこのテーブルを使用する。

## 適用値

速度は全て0.50 m/s（500 mm/s）。半径は大きさを記し、右は負、左は正。距離はmm、角度はdeg。
角度補正は論理角に加算する値で、右の＋1度に対応する左の値は−1度となる。

| ターン（左右共通） | 半径の大きさ [mm] | Lstart [mm] | Lend [mm] | 論理角の大きさ [deg] | 右 / 左の角度補正 [deg] |
| --- | ---: | ---: | ---: | ---: | ---: |
| 90度 | 37.5 | 40.00 | 44.00 | 90 | ＋1.0 / −1.0 |
| 180度 | 44.5 | 23.63 | 24.47 | 180 | 0 / 0 |
| V90 | 41.0 | 9.00 | 11.60 | 90 | ＋1.0 / −1.0 |
| 45度入口 | 50.0 | 14.00 | 36.00 | 45 | ＋0.5 / −0.5 |
| 45度出口 | 50.0 | 32.80 | 18.00 | 45 | ＋0.5 / −0.5 |
| 135度入口 | 32.3 | 44.00 | 38.35 | 135 | ＋0.5 / −0.5 |
| 135度出口 | 32.3 | 33.70 | 46.07 | 135 | ＋1.5 / −1.5 |

旋回の速度PIDは全種類・左右でKp=1.50、Ki=0.02、Kd=0、角速度PIDはKp=0.10、Ki=0.002、Kd=0。
既存のターン種類別ゲイン定義を使用し、対応する左右の`t_param`は同じゲインを参照する。
FFはBの`ff_gain_default`を継続する。保存済み候補のFFは、表示の整数丸めを除いてBの既定係数と一致する。
機体AのFF・PID、他速度、探索ターン、直進プロファイル、ターン内の前後直進ゲインを選ぶ既存処理は変更していない。

## 入力資料と測定条件

入力は`../20261114_B_turn500_seed_from_A/`の各`B_*_single_turn_candidate.settings.json`と以下の集計。
設定JSONの`mouse=B`、preset=500、回数1、吸引OFF、前走加速度6.5 m/s²を確認した。
これらは過去の測定条件であり、最短の全直進を同じ速度・加速度へ固定する意味ではない。

| 右の候補設定JSON | 測定根拠 |
| --- | --- |
| `B_R90_single_turn_candidate.settings.json` | [右90度・3本](../20261114_B_turn500_seed_from_A/R90_corr1_Lend44_three_review/README.md) |
| `B_R180_single_turn_candidate.settings.json` | [右180度・3本](../20261114_B_turn500_seed_from_A/R180_radius44p5_three_v5/README.md) |
| `B_rV90_single_turn_candidate.settings.json` | [右V90・3本・検出v8](../20261114_B_turn500_seed_from_A/rV90_Ls9_Le11p6_c1_three_v8_review/README.md) |
| `B_inR45_single_turn_candidate.settings.json` | [右45度入口・3本](../20261114_B_turn500_seed_from_A/inR45_kp01_Lstart14_three_review/README.md) |
| `B_outR45_single_turn_candidate.settings.json` | [右45度出口・3本](../20261114_B_turn500_seed_from_A/outR45_lengths_three_review/README.md) |
| `B_inR135_single_turn_candidate.settings.json` | [右135度入口・3本](../20261114_B_turn500_seed_from_A/inR135_r32p3_Lstart44_three_review/README.md) |
| `B_outR135_single_turn_candidate.settings.json` | [右135度出口・3本・検出v8](../20261114_B_turn500_seed_from_A/outR135_corr1p5_Lstart33p7_three_v8_review/README.md) |

## 評価範囲

右の測定は実測入口方向へ合わせた相対マーカー中点の評価。盤面固定の停止点・絶対車軸位置の精度を保証するものではない。
左側はユーザー指定の左右共通化による対称値で、左の実走測定値ではない。
右135度入口は1倍2本・2倍1本の混在条件で、旧2倍映像の基板上の黄色を前マーカーと誤認した例がある。
保存済みメタ情報の`measurement_detector_followup_required=true`を維持し、連続走行や盤面固定での採用を主張する前に検出v8で再点検する。
元の測定・候補・除外資料は変更せず保持する。

2026-11-16に機体Bへの書き込み・内容検証・リセットまで実施した。反映後の単体走行・左右の連続接続・壁あり最短走行は未実施。
ビルドとホストで確認するのは、コンパイル、14種類の参照、候補値との一致、左右の符号・共通ゲイン・Bの既定FFの対応である。

## 検証結果

- `Debug-MouseB`のConfigure・Build成功。成果物は`build/Debug-MouseB/mouse_type8.elf`。
- 実際の`Params/run_param.h`を`MOUSE_B`でホストC++コンパイルし、保存済み候補JSONと`mode_500`の14参照を照合して成功。距離・半径・論理角・角度補正、左右の共通PID参照、B既定FFとの対応を確認した。
- `python -m unittest discover -s tools/tests -p test_search_turn_params.py -v`成功（1件、スキップ0）。実装本体を使ったA/Bの探索パラメータ・前後直進PID選択の既存検証も通過。
- 記録はローカルの`build/B-turn500-applied-Debug-MouseB.log`と`build/B-turn500-applied-iolaalat/`。生成物はGit管理対象外。

## 実機書き込み（2026-11-16）

- ユーザーがB本体のST-LINK/SWD配線と電源の準備完了を確認後に実施。
- `Debug-MouseB`をクリーンビルドし、コンパイル定義が`MOUSE_B`であることを確認。
- 書き込み用に固定したELF: `build/flash-B-20261116-032605/mouse_type8.elf`。
- ELF SHA256: `d96608a57c46be9082a6d187636c2f0b8c2432804c667c70a8f42e96c2b6450b`。
- STM32CubeProgrammer v2.23.0、STLINK-V3MINIE、SWD。接続先はDevice ID `0x455`、STM32U535/STM32U545、512 KB、電圧3.27 V。
- CLIの`-d ELF -v -rst`で書き込み、`Download verified successfully`とソフトウェアリセット完了を確認。
- 詳細は同フォルダーの`manifest.json`、`connect.log`、`program-verify-reset.log`。実機の走行・BLE通信は未確認で、書き込み後の電源OFFも未確認。
