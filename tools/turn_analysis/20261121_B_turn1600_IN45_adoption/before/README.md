# mouse_type8

STM32U535CEU6 を対象にしたマイクロマウス用ファームウェアです。STM32CubeIDE for Visual Studio Code で生成された CMake/Ninja 構成を使い、アプリケーション層は C/C++ で実装されています。

## 開発環境

- VS Code
- STM32CubeIDE for Visual Studio Code
- CMake Tools
- Ninja
- Arm GNU Toolchain (`arm-none-eabi-gcc`, `arm-none-eabi-g++`)

このプロジェクトでは `.vscode/settings.json` で CMake 実行コマンドに `cube-cmake`、生成器に `Ninja` を指定しています。

## プロジェクト構成

| ディレクトリ / ファイル | 内容 |
| --- | --- |
| `Core/` | STM32CubeMX が生成する初期化コード。`main.c`、割り込み、GPIO、ADC、SPI、TIM、USART など。 |
| `Drivers/` | STM32 HAL Driver と CMSIS。 |
| `Peripheral/` | モータ、エンコーダ、IMU、赤外線センサ、バッテリ、通信、Flash などの低レベル入出力。 |
| `Module/` | 割り込み処理、ログ、Flash 管理などの共通モジュール。 |
| `Task/` | 走行制御、姿勢制御、センサ処理、モーション生成などのタスク層。 |
| `Subsys/` | 探索、地図生成、経路生成、壁情報管理などの迷路処理。 |
| `System/` | モード選択、デバッグ、デモ、シェル、システムユーティリティ。 |
| `Component/` | コントローラ、キュー、経路追従、数学ユーティリティ、ntshell などの汎用部品。 |
| `Params/` | 走行パラメータ、ターンテーブル、センサ閾値など。 |
| `firmware/hy0020/` | mouse_type8 の UART を BLE へ中継する HY0020 用ファームウェアと SWD 設定。 |
| `tools/ble/` | HY0020 用の PC シェル、既存 GUI の BLE 接続、通信診断ツール。 |
| `cmake/` | STM32CubeIDE for VS Code が生成した CMake 設定とツールチェーン設定。 |
| `CMakeLists.txt` | プロジェクト本体の CMake エントリ。 |
| `CMakePresets.json` | MOUSE_A / MOUSE_B、それぞれの `Debug` / `Release` CMake preset。 |
| `mouse_type8.ioc` | STM32CubeMX の設定ファイル。 |
| `open-ioc.cmd` | 同じフォルダ内の `.ioc` ファイルを開くための Windows 用スクリプト。 |
| `STM32U535CEUX_FLASH.ld` | Flash 用リンカスクリプト。 |
| `STM32U535CEUX_RAM.ld` | RAM 用リンカスクリプト。 |

## 実行の流れ

吸引開始の停止中は、機体別の`om_gain_suction_start`で開始方向を保持します。
機体Aはduty650の保存動画3本で右へ約0.6～1.8°の変化を確認し、停止用P/Iを基本ゲインの3倍とする比較用候補を設定しました。
停止中の旋回フィードバックは±0.5 Vに制限し、飽和周期は積分を抑制します。立上げ完了直後の待機でも角度偏差と保持出力を引き継ぎます。
停止中のI項は実測角度から再構成し、出力制限によって開始方向を失わないようにしています。
機体Bの専用ゲイン初期値はB自身の基本値です。新ゲインでの実機効果・振動・IRQ時間は未確認です。
測定根拠は[吸引開始の動画・ログ照合](tools/logs/suction_start_A_20261119_baseline_v1/README.md)、ホスト検証は[テスト手順](tools/tests/README.md)を参照してください。

直進・探索・ターン前後の直進壁制御は、ローカルの
`program_zirconia/zirconia/software/Control/Src/wall_control.c` と
`Params/control_param.h` の `independent_side_pose_v3` をtype8へ移植しています。
実装は `Task/Src/straight_wall_control.cpp`、初期ゲインは
`Params/wall_param_A.h` / `wall_param_B.h` の `wall_control_param`、検証方法は [tools/tests/README.md](tools/tests/README.md) を参照してください。
左右独立の信頼度評価、3 mmの新規壁確認、10 mmの補正立上げ、予測横偏差と姿勢制動を使用します。
姿勢制御時の上限は角速度1.2 rad/s・角加速度20 rad/s²、その他は0.2 rad/s・4 rad/s²です。
これらはzirconiaの初期値であり、Mouse A/Bでの実機調整は未実施です。

移植範囲は横壁制御です。前壁合わせ、内側の速度・角速度PID、機体別距離校正は既存実装を使用します。
斜め直線は従来制御を基本とし、`control`では下記の接近観測を使う斜めEKF制御へ切り替えます。
直進の壁切れイベントと位置補正は、下記の確認・遅れ補償を使います。
直進壁制御の観測だけに10点ADC平均と65/70 mmの壁有無ヒステリシスを使用し、
従来の壁切れ・迷路用判定の履歴から独立させています。前壁の存在だけでは側壁制御を抑制しません。
type8の横位置推定値（左正・mm）を直進壁制御の偏差（右正・m）に換算し、安定壁が得られた場合だけ同期します。
姿勢偏差は壁制御区間のジャイロ積分を別に保持し、従来の壁切れ処理による姿勢リセットの影響を避けます。
無効化・モード切替・別通路で制御状態を初期化し、同じ通路の連続直進・ターン前直進では制御phase・姿勢・推定を維持します。
探索ターン後は旋回の残留角を新しい通路の姿勢として引き継ぎ、新しい10点のADC平均を待って壁制御を再開します。
探索ターンの出口直進長には、入口の左正の横偏差を符号を保って±10 mmに制限し、右ターンで加算・左ターンで減算します。
従来の0.20 m/s以下の測定採用制限を維持し、新直進制御ではこの範囲の補正出力を0にします。

直進壁制御には、通路内の横偏差・姿勢偏差・進行距離誤差を推定する固定3状態EKFを組み込んでいます。
起動時は`shadow`（従来制御で走り、EKFを並行計算）です。停止中の`wall_ekf control`で
壁の品質・予測残差・共分散に基づく壁採用と、EKFの横偏差・姿勢を使う制御へ切り替えます。
EKF姿勢は区間ジャイロ積分との差が3°以内で、共分散の条件も満たす場合に採用します。
不整合・観測不採用・確認中は並行計算中の従来制御へ戻し、EKF側の古い旋回指令を破棄します。
全壁喪失時は信頼できる姿勢だけを短距離保持し、それも不確かな場合は補正を0にします。
壁なしでは予測を継続し、壁切れの横位置補正は観測として一度だけ取り込みます。
最短の直進→90/180度ターン→直進では、A/Bとも`control`時に入口姿勢＋実測ジャイロ旋回量−論理ターン角を出口の姿勢初期値へ引き継ぎます。
直線→45/135度進入→斜め→V90（Long V90を含む）→斜め→45/135度退出→直線でも、`wall_ekf control`で姿勢と分散を引き継ぎます。斜め側には`wall_edge control`も必要です。
各EKFは独立したまま、共通の姿勢・分散だけを`Task/Inc/motion_heading.h`で受け渡します。横位置・進行位置・滑り角・バイアスは各推定器に保持し、既存の斜め同士の座標・共分散変換も維持します。
ターンEKFの入口には通路の姿勢・分散を一度だけ渡します。校正・品質条件を満たす`turn_ekf control`の最終周期では、その姿勢・分散を出口へ返します。その他は入口姿勢に実測旋回量とジャイロ雑音を加えてつなぎ、相関する二つの推定を重ねて観測更新しません。
斜めEKFの新規開始と接近観測EKFにも出口の姿勢・分散を渡します。柱観測がない間の引継ぎは220制御周期未満・斜め1.5区画未満に制限し、姿勢・分散も確認します。Bでは未校正の柱から姿勢を生成せず、直前区間から引き継いだ姿勢をジャイロで伝播する場合だけ接続できます。
姿勢分散も伝播し、区間の原点リセットで残留角を失わないようにします。入口/出口と同じ割込みのジャイロを二重加算せず、出口では回転中のADC平均を捨てて新しい10点を待ちます。
入口姿勢なし・非有限のジャイロ・残留角15°超は引継ぎを棄却します。`legacy/shadow`、スタート直後の縦位置基準は今回の適用対象外です。実機の高速走行性能は未評価です。
機体Aの90 mm探索直進、最短の直進ターン入口、およびターン出口から続く直進では、`wall_ekf control`選択で壁切れと独立柱もEKFの進行距離観測として実適用します。
探索はWall→Open=49.5 mm・Open→Open=39.5 mmです。最短の直進入口は経路のセル中心基準へ45 mm移し、壁切れ4.5 mm・独立柱−5.5 mmを試験基準にします。Legacyの壁切れ7 mmとは分けています。
各原点の基準へ確定遅れを加算し、同一区間で一度だけ観測します。独立柱の極小が入口原点より手前なら負になります。
ターン後は90/180度ターンと45/135度の斜めからの退出が対象です。出口内では`Lend+fix_post_run`を加えた基準を使い、直後の直進へ原点を移すと壁切れ`4.5+90n mm`・柱極小`−5.5+90n mm`（n=0,1,…）になります。
検出履歴・採用済み区画を出口から直進へ引き継ぎ、両側や両特徴点で同じ区画を二度補正しません。次モーションの設定待ちでも同じ通路のEKFを継続します。
EKFの縦補正が柱の極小と確定の間に入った場合は、保留中の観測座標へ既知の補正量を反映します。地図の参照位置は動かさず、未知の座標ジャンプは引き続き棄却します。
補正量は最大半量・±4 mm、残差窓は±8 mmです。出口の基準も試験値であり、補正ONの実機精度は未確認です。Bの未校正の位置基準・斜め極小・ターン出口からの連続性が不明な直進へは適用しません。
斜めの進行距離補正には、入口・直線・出口、左右、差分による壁端・柱の距離極小を分けたShadow専用の枠組みを用意しています。
A/Bともこの進行距離モデルは未校正で、制御へは反映しません。
`Params/wall_param_A.h` / `wall_param_B.h`の`diagonal_feature_shadow_param`で基準・周期・横ずれ/角度の感度を機体別に保持し、前壁・飽和・姿勢・世代の棄却理由を診断へ残します。
入口/出口をまたぐ原点移動と、旋回後の論理方向に対する角度残差もShadow側へ引き継ぎます。詳細は[操作・診断仕様](tools/myshell_README.md#直進壁ekf)を参照してください。
斜め柱の姿勢推定EKFは、最短経路の各区画境界から柱の側と接続壁を求めます。柱の数は経路に応じて変わり、欠落や割り当てが曖昧な候補は飛ばせます。
前壁センサを使わず、柱の極小・エンコーダ・ジャイロから進行位置・横位置・角度を推定します。斜め進入ターンの出口から斜め直線、次のターン入口まで同じ座標を維持します。
V90や連続する斜め退出・進入ターンでは、直前の斜めで信頼できる推定を得た場合に限り、旋回中もエンコーダとジャイロで予測します。
次の斜めの地図原点と進行方向へ状態・共分散を変換し、出口で座標と向きの整合を確認します。引き継いだ推定を操舵に使うには新しい区間で左右両側の更新と既存の信頼度条件を満たす必要があります。
進行位置は経路座標を初期値とし、標準偏差20 mmで開始します。柱の照合は斜め区画長の3/4（約47.7 mm）を残差の上限とする共通ルールです。機体Aは映像照合で確認し、機体Bのセンサ校正は別途必要です。`wall_edge shadow/control`の走行後ログで`diag_pose_*`を確認できます。
機体Aで`wall_ekf control`と`wall_edge control`を選ぶと、信頼できる横位置・角度を斜め走行中の目標角速度へ反映します。時間と移動距離で切り替えを制限し、鮮度条件を外れたら従来制御へ戻します。進行位置は診断に記録します。
探索の前壁左右差判定は、`control`時だけ推定と観測履歴による確認を加えます。
機体A/Bの校正を維持し、係数の実機調整は未実施です。
操作・ログの意味は[直進壁EKF](tools/myshell_README.md#直進壁ekf)を参照してください。

旋回中の位置更新には、機体Aの検証範囲で固定6状態のターンEKFを採用し、位置増分を`Vehicle::pose`へ一度だけ反映します。
起動時は`turn_ekf control`で、非吸引500の右90°、吸引1600・duty600～700の右90°/右180°を対象にします。
吸引状態から滑り係数と雑音を選び、静止校正・観測残差・共分散・処理時間の条件を満たす区間だけ位置増分と滑り角を反映します。
悪化があった非吸引の右180°、未検証の左旋回・別速度、未校正のBは従来の積分を使います。
ジャイロによる制御角、速度・長さ、側壁制御の`x_point`は既存の処理を維持します。通路とターンの姿勢原点の差は内部で変換します。旧`ego.turn_x/y`ログ列は、`pose`をログ用の固定原点へ変換した左/前位置です。
パラメータは`Params/turn_pose_param.h`、操作・適用範囲は[ターン自己位置EKF](tools/myshell_README.md#ターン自己位置ekf)を参照してください。
実C++実装の因果的ログ再生で確認しましたが、変更後ファームの実機周期・走行評価は未実施です。

1. `Core/Src/main.c` で HAL と各ペリフェラルを初期化します。
2. `CPP_Main()` を呼び出し、C++ 側の初期化へ入ります。
3. `System/Src/mode.cpp` の `Mode::Select_Mode()` でエンコーダとボタンによるモード選択を行います。
4. 選択したモードに応じて、インターフェース確認、デモ走行、デバッグ走行、シェル実行などを呼び出します。

### 走行・探索ターンパラメータの配置

共通の入口は`Params/run_param.h`です。選択された機体の走行設定と探索ターン設定を次の配置から読み込みます。

| 機体 | 走行設定 | 探索ターン設定 |
| --- | --- | --- |
| A | [Params/Param_A/run_param_A.h](Params/Param_A/run_param_A.h) | [Params/Param_A/turn_search/turn_search.h](Params/Param_A/turn_search/turn_search.h) |
| B | [Params/Param_B/run_param_B.h](Params/Param_B/run_param_B.h) | [Params/Param_B/turn_search/turn_search.h](Params/Param_B/turn_search/turn_search.h) |

各ヘッダーは`extern const`による宣言と設定の根拠を保持し、実際の値は同じ名前の`.cpp`へ分けています。

| 用途 | 機体Aの値の定義・編集先 | 機体Bの値の定義・編集先 |
| --- | --- | --- |
| 直進・共通設定 | [run_param_A.cpp](Params/Param_A/run_param_A.cpp) | [run_param_B.cpp](Params/Param_B/run_param_B.cpp) |
| 探索ターン | [turn_search.cpp](Params/Param_A/turn_search/turn_search.cpp) | [turn_search.cpp](Params/Param_B/turn_search/turn_search.cpp) |
| 最短ターン（500の例） | [turn_500.cpp](Params/Param_A/turn_500/turn_500.cpp) | [turn_500.cpp](Params/Param_B/turn_500/turn_500.cpp) |

速度ごとにフォルダを分け、宣言の`.h`と値の`.cpp`を同じ場所へ置きます。A/Bとも同じ構成です。

```text
Params/Param_A/                 # Param_Bも同じ構成
├── run_param_A.h
├── run_param_A.cpp
├── turn_search/
│   ├── turn_search.h
│   └── turn_search.cpp
├── turn_500/
│   ├── turn_500.h
│   └── turn_500.cpp
└── ...                        # 他速度もturn_<speed>/へ配置
```

PID・FF・形状・配列の順序を保ち、各定数は対応する`.cpp`で一度だけ定義します。CMakeは選択した機体の定義ファイル群だけをリンクします。
`turn_search.h`には探索90度ターンの左右・速度別テーブル、専用PID、FFの宣言と調整根拠をまとめています。
呼出し側は引き続き`Params/run_param.h`を使用できます。数値を調整する場合は対応する`.cpp`の定義を編集してください。
モード配列の要素数を変更する場合は、対応するヘッダーの配列要素数も合わせて更新します。
`PARAM_A_USE_PINK_LED_TURN_LENGTHS`の切替は`run_param_A.h`、定数初期化に使う探索FFの3つのスカラー値は`turn_search.h`に保持しています。

走行モードの速度設定は、各機体の`run_param_A.cpp` / `run_param_B.cpp`の前半にまとめています。A/Bそれぞれ直進・斜めの全38設定を同じ型で定義し、`Params/run_config.h`で全17の走行モードへ対応付けています。`plain500`は次の2つで調整します。

```cpp
// 順序: ベース速度、加速度、減速度の大きさ、速度の刻み、最高速度
// 単位: 速度はm/s、加減速度はm/s^2
const t_speed_profile straight_speed_plain500 = {0.50f, 6.0f, 6.0f, 0.10f, 0.70f};
const t_speed_profile diagonal_speed_plain500 = {0.50f, 6.0f, 6.0f, 0.10f, 0.70f};
```

この設定では500・600・700 mm/sを候補として生成します。刻みや最高速度の変更に伴う候補配列の編集は不要で、刻みで割り切れない最高速度も最後の候補へ含めます。ベースと最高速度が同じなら定速上限1候補になり、刻み0も指定できます。それ以外は正の刻みが必要で、候補数は256以下です。

高速側で刻みや加速度が途中で変わる場合は、`t_speed_range`を昇順に並べます。たとえば`straight_speed_1200_v0`は次の定義で、1200・1300 mm/sでは10 m/s²、1400・1500 mm/sでは12 m/s²を指定します。

```cpp
constexpr t_speed_range straight_speed_ranges_1200_v0[] = {
    {1.20f, 10.0f, 10.0f, 0.10f, 1.30f},
    {1.40f, 12.0f, 12.0f, 0.10f, 1.50f},
};
const t_speed_profile straight_speed_1200_v0 = make_speed_profile(straight_speed_ranges_1200_v0);
```

編集先は各範囲の5項目です。`make_speed_profile`が全体のベース・最高速度などを求めるため、全体側への二重記入は不要です。範囲間の空きを使って、1500から2000 mm/sへ飛ぶような不等間隔候補も表現します。範囲同士の重複・逆順は許可しません。加減速度は選んだ候補の値を区間全体へ適用します。走行中に速度域を通過するたびに加速度を切り替える設定ではありません。

直進と斜めは別設定です。ベースは通常の接続基準速度で、区間の入口には直前の目標速度、出口には次ターンの校正済み入口速度（ゴールは0）を使います。経路の時間計算と実走行は同じ候補選択処理を使い、従来の10 mmの選択余裕を保ちます。短区間でベースまで届かない場合は到達できる三角速度波形を計算し、入口から出口へ加減速できない区間は採用しません。加速度と減速度はモーション初期化まで個別に渡します。

PID・FFは速度候補から独立した新形式で設定します。`t_control_gain`に速度PID・角速度PID・FF参照をまとめ、`t_control_band`にその組を使い始める速度[m/s]を指定します。`straight_control_*` / `diagonal_control_*`が速度帯の表を参照し、`Params/run_config.h`で速度プロファイルと組み合わせます。同じPID・FFの組は共有するため、加速度別・速度別に同じ値を書く必要はありません。

たとえば機体Aの`plain500`のゲイン帯は次の設定です。選択速度以下の最も近い帯を使い、先頭より低い速度は先頭の組、最後より高い速度は最後の組を使います。直進・斜めの表は個別に指定できます。速度条件の変更と新速度でのゲイン校正は別作業です。

```cpp
constexpr t_control_band control_bands_straight_plain500[] = {
    {0.5f, &straight_gain_low},
    {0.7f, &straight_gain_700},
    {2.2f, &straight_control_high},
};
const t_control_profile straight_control_plain500 = make_control_profile(control_bands_straight_plain500);
```

2200 mm/s以上の速度帯は、各機体の`straight_control_high`を参照します。2026-11-18に2200の既存値へ統一し、Aは速度PID=2.0/0.05/0、角速度PID=0.2/0.01/0、Bは速度PID=4.0/0.05/0、角速度PID=0.2/0.01/0としました（順序はKp/Ki/Kd）。FFは各機体の`ff_gain_default`です。Aの旧3000以上の角速度PIDと旧3200の速度D項もこの値へ統一しています。

速度を数字で付けた旧`param_*`・`st_param_*`・`sp_gain_*`・`om_gain_*`と、旧候補配列`st_mode_*` / `di_mode_*`の宣言/定義は全速度でコメントアウトしました。探索の固定速度は`search_straight` / `search_straight_slow`、デバッグ走行は`debug_straight_*`の`t_motion_profile`で設定し、`.parameter()`でモーションAPIへ渡します。機体Aの探索370 mm/s・低速320 mm/s、機体Bの探索320 mm/sと既存のPID・FFは維持しています。最短の旧候補配列APIとプロファイル未指定の互換経路は廃止し、速度またはゲイン設定が欠けた走行設定は不正として扱います。ターン形状・ターン速度は引き続き`turn_<speed>/`の機体別設定を使います。変更後の実機走行は未確認です。

2026-11-17の移行では、A/Bそれぞれ全38設定・354候補の速度・加減速度・PID/FF参照を保持しました。全17の走行モードで新形式を使います。直進と斜めで加速度が異なる1800設定（直進30、斜め20 m/s²）や、可変ターン1600 v1の直進50・斜め30 m/s²も維持しています。実際の入口/出口速度を考慮するため、選択される候補や経路の時間見積りが変わる場合があります。2026-11-18の旧定義廃止でも、速度・加減速度の全354候補と低速側の機体別PID・FF数値をホストテストで照合しています。

### 機体A・turn1600の左右90度調整値

2026-11-18のユーザー依頼で、`Params/Param_A/turn_1600/turn_1600.cpp`の左右90度へ単体調整値を反映しました。以前の定義は撮影方式フラグの分岐ごとコメントで保持しています。
Lstartは左右12.0 mm、Lendは左右26.31 mm、角度補正は右＋1.5度・左−1.5度です。論理角は左右±90度を保ち、物理指令角は補正の加算により右−88.5度・左＋88.5度になります。
左右90度では`PARAM_A_USE_PINK_LED_TURN_LENGTHS`によらずこの採用値を使います。方向別FFとPIDは既存値を参照します。

根拠は[機体A・右90度の3本比較](tools/logs/pixel_camera/turn1600_A_r90_20261118_frame18_v1/repeatability_corr1p5_Ls12p0_03/review_ja.md)です。
速度1600 mm/s・吸引650・前走加速度6.5 m/s²で、入口直線基準の相対評価は平均直線角90.12度、出口線の入口方向位置差平均＋0.58 mmでした。
左はユーザー指定による同値展開で未走行確認です。盤面固定の絶対精度、連続・最短走行への接続、更新ファームウェアの書き込み後の動作は未確認です。

### 機体A・turn1600の左右180度調整値

2026-11-19、右180度の単体調整値を左右180度に反映しました。半径は右−45 mm・左＋45 mm、角度補正は右＋2.0度・左−2.0度です。Lstart 13.12 mmとLend 27.38 mmは左右共通です。論理角は右−180度・左＋180度、物理指令角は右−178度・左＋178度です。左右180度は `PARAM_A_USE_PINK_LED_TURN_LENGTHS` によらずこの値を使い、以前の定義は分岐ごとコメントで保持しています。左はユーザー指定による展開で、実走は未確認です。

速度1600 mm/s・吸引duty 650の右180度単体走行3本で、進入直線基準の旋回角は179.73～179.92度、折返し幅は89.39～91.54 mmでした。動画の横/前速度比は25.5～26.7%で、滑り係数Kの検証候補191.34 m/s²とともに設定のコメントへ記録しています。吸引時の実行値はK = 254.90725 m/s²です。EKFの継続適用と連続走行への接続は実機で要確認です。[3本の測定比較](tools/logs/pixel_camera/turn1600_A_r180_20261118_v1/corr2p0_radius45_repeatability_03/compare.html)・[滑り係数の再生比較](tools/logs/pixel_camera/turn1600_A_r180_20261118_v1/corr2p0_radius45_slip_candidates_20261119/compare.html)

### 機体A・turn1800の左右45度進入調整値

2026-11-21のユーザー依頼で、右45度進入のLstart 2.0 mm・Lend 40.0 mmを左右へ反映しました。速度1.80 m/s、半径は右−53 mm・左＋53 mm、論理角は右−45度・左＋45度、角度補正は左右とも0度です。左右とも`PARAM_A_USE_PINK_LED_TURN_LENGTHS`によらずこの値を使い、方向別PID・FFは既存の参照を使います。左は依頼による鏡映で実走未確認です。

機体A・吸引duty650の右単体の有効5本で、入口相対のマーカー中点のLend残差距離は平均1.69 mm・最大2.32 mm、出口方向は44.56〜45.30度でした。[採用値・測定根拠・検証結果](tools/turn_analysis/20261121_A_turn1800_inr45_adoption/README.md)へ記録しています。機体AのDebugビルドとELF内の左右の値・参照を確認済みで、更新ファームウェアの実機書き込み・連続／最短走行は未実施です。

### 機体A・turn1800の左右135度進入調整値

2026-11-20のユーザー依頼で、右135度進入の単体調整候補を左右へ反映しました。速度1800 mm/s、半径は右−40 mm・左＋40 mm、Lstart 2.5 mm・Lend 26.0 mm、角度補正は右＋2.4度・左−2.4度です。論理角は左右±135度を維持し、物理指令角は右−132.6度・左＋132.6度になります。この左右の形状値は`PARAM_A_USE_PINK_LED_TURN_LENGTHS`によらず使用し、方向別のPID・FFは既存値を参照します。

機体A・吸引duty 650の右単体走行3本で、入口方向基準の出口方向は134.82～135.51度でした。動画と1 msログがそろった2本の出口位置差は約0.40～1.25 mmです。通信切断・電源再投入で1 msログを失った1本は、動画の方向確認だけに使用し、位置差の統計へ含めていません。[候補設定](tools/logs/pixel_camera/turn1800_A_inr135_20261120_v1/Ls2p5_Le26_c2p4_review_v1/candidate.settings.json)・[測定比較](tools/logs/pixel_camera/turn1800_A_inr135_20261120_v1/Ls2p5_Le26_c2p4_review_v1/review.json)に根拠を保存しています。左は依頼による鏡映で実走未確認です。盤面固定の絶対精度、連続・最短接続、書き込み後の動作は未確認です。

### 機体B・turn1600への形状テーブル転用

2026-11-20のユーザー指定で、`Params/Param_B/turn_1600/turn_1600.cpp`の
全14種類の`t_turn_param_table`を、同日時点の機体Aの現行値へ置き換えました。
参照元は`Params/Param_A/turn_1600/turn_1600.cpp`です。速度[m/s]、半径・Lstart・Lend[mm]、
論理角・角度補正[deg]をコピーし、変更前のB値は各定義の直前にコメントで保持しています。
BのPID・FFはBの既存設定を使います。これは初期値としてAの設定値を指定により転用したものです。Bでの追加調整と実走確認範囲は、以下の各ターンの記録を参照してください。

### 機体B・turn1600の左右90度への調整値反映

2026-11-20のユーザー依頼で、右90度の単体調整値を機体Bの形状テーブルへ反映し、左90度にも鏡像展開しました。
旧定義はコメントで保持しています。PID・FFは機体Bの既存設定です。

| 方向 | 速度[m/s] | 半径[mm] | Lstart[mm] | Lend[mm] | 論理角[deg] | 角度補正[deg] |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 右90度 | 1.60 | −52.0 | 13.6 | 26.31 | −90.0 | ＋1.0 |
| 左90度 | 1.60 | ＋52.0 | 13.6 | 26.31 | ＋90.0 | −1.0 |

右は吸引duty650の単体3本で相対旋回角88.90～89.97度、出口直線ずれ−0.31～＋1.08 mm。
開始位置を厳密にそろえず、各動画の入口直線を基準にした比較です。左はユーザー指定の同値展開で実走未確認です。
ソース反映と更新ファームの書き込み後の確認・連続接続は区別します。[反映根拠と検証](tools/turn_analysis/20261120_B_turn1600_R90_adoption/README.md)。

### 機体B・turn1600の左右180度への調整値反映

2026-11-20のユーザー依頼で、右180度の単体調整値を機体Bの形状テーブルへ反映し、左180度にも鏡像展開しました。
旧定義はコメントで保持しています。PID・FFは機体Bの既存設定です。

| 方向 | 速度[m/s] | 半径[mm] | Lstart[mm] | Lend[mm] | 論理角[deg] | 角度補正[deg] |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 右180度 | 1.60 | −47.5 | 13.12 | 27.38 | −180.0 | ＋0.5 |
| 左180度 | 1.60 | ＋47.5 | 13.12 | 27.38 | ＋180.0 | −0.5 |

右は吸引duty650の単体3本で相対旋回角179.25～180.44度、折返し幅90.47～91.56 mm。
各動画の入口直線と走行前マーカー中点を基準に比較しています。左はユーザー指定の鏡像展開で実走未確認です。
更新ファームの書き込み後の確認・連続接続は未実施です。[反映根拠と検証](tools/turn_analysis/20261120_B_turn1600_R180_adoption/README.md)。

### 最短走行開始時の予備加速

最短の通常・吸引・可変ターン走行では、13.1 mmの予備区間を先頭の直線またはターン前直進へ含めます。
短い独立モーションを完了させないため、予備区間の終端で目標加速度が一瞬0になる切替をなくしています。
先頭がターンの場合も、停止状態からターン速度へ加速する初期化を使います。可変ターンの先頭右90度に残っていた定速用の呼出しも同じ扱いです。

走行開始時の進行距離原点を−13.1 mmとし、直線の速度候補を選ぶ際もこの予備距離を含めます。
物理的な直線終端・ターン開始位置は従来と同じです。予備区間中の進行距離ログは負の値になり、区間を通過すると0になります。
先頭直線の速度・加速度・PID/FFは、予備距離を含めて選んだ候補を最初から使います。ターン形状と機体別の設定値は従来値です。
2026-11-17にA/Bの全17走行設定をホストで検証しました。実機での連続加速・到達速度・走行精度は未確認です。

### 吸引立上げ中の停止制御

`suction_start`は機体A/B共通で、目標速度・加速度・角速度・角加速度を0に保って吸引電圧を立ち上げます。
この間と完了直後の次モーション待ちでは、速度PIDへ左右エンコーダの平均速度だけを渡します。
通常の走行速度推定に含まれる加速度センサの補正値が、吸引振動や加速度の偏りによって停止中の駆動指令になることを防ぎます。
実際の車輪の移動に対する速度PIDと、ジャイロによる角速度PIDは継続します。走行用の壁操舵は呼び出しません。
次の直進・ターンへ移ると通常の速度推定を使う制御へ戻ります。PIDゲインと吸引電圧・立上げ時間の設定は従来値です。

吸引I/Oと`suction_start`から滑り係数の固定代入を外し、実測・目標の滑り計算は
`Vehicle::turn_slip_k_m_s2()`から同じ基本値を参照します。
設定元は`Params/turn_pose_param.h`で、機体Aは以前推定した非吸引41.28055 m/s²・吸引254.90725 m/s²を採用します。
根拠は2026-11-18の[条件別解析](tools/logs/turn_slip_conditions_A_20261118_v1/README.md)で、
500 mm/s・非吸引6動画と1600 mm/s・duty650の13動画から求めた実効係数です。
機体Bは対応する校正がないため、Bの非吸引75・吸引250 m/s²を維持します。
係数は直前のファン出力で選び、出力0では非吸引、正の出力では吸引の値を使います。
吸引フラグだけをONにした時点では切り替わらず、停止時は出力の記録も0へ戻します。
ターンEKFの校正値と共通の設定を使いますが、EKFの速度・duty・ターン種別の採用範囲は変更しません。

2026-11-17にA/Bの実パラメータ・実PID・実モーション関数を使うホストテストで、停止中の加速度由来の速度偏り、前後移動の抑制、吸引立上げ完了後の待機、直進への復帰を検証しました。
実機での吸引振動・停止の安定性は未確認です。テスト手順は[ホストテスト](tools/tests/README.md)を参照してください。

### 機体A・探索370 mm/sの単体調整値

2026-11-16のユーザー依頼で、`Params/Param_A/turn_search/turn_search.h`の左右370 mm/sへ単体候補を試験用に反映しました。
速度PIDは5.0/0.04/0、Lstartは左右9.46 mm、Lendは右8.50 mm・左9.00 mm、角度補正は左右0度です。
半径±26 mm、角速度PID、方向別FFは従来値を維持しています。共有の探索ターンゲインは変更せず、370専用の速度ゲインを使います。

単体試験とゲイン供給を揃えるため、370のターン内の入口・出口直進にも同じ速度/角速度PIDを指定します。
`t_param.search_straight_sp_gain` / `search_straight_om_gain`は探索ターン専用の任意設定です。
`Init_Motion_search_turn` / `exe_Motion_search_turn`の明示引数を優先し、省略時はこの設定、未設定なら従来の`basic_*_gain`を使います。
他速度・機体Bと単体GUIの明示指定は従来のゲインを維持します。ターン外の直進や探索全体の加速度設定は今回の変更対象ではありません。

根拠は[左右370の測定資料](tools/logs/pixel_camera/A_search370_summary_20261116/README.md)です。
吸引OFF・前走加速度6.5 m/s²、右3本・左5本の相対マーカー中点評価で、盤面固定の絶対精度を保証するものではありません。
左の角度補正＋0.7度は比較のみで不採用です。2026-11-16に機体Aへ書き込み・照合済みです。
反映後の実機再現性・連続接続・壁あり探索は未検証です。

### 探索ターン後の復帰と補正判定

探索スラロームは、旋回開始時の通路姿勢＋旋回中のジャイロ積分－論理ターン角（左右90°）を、
出口の姿勢として渡します。`degree_correction`は指令側の補償であり、通路の基準角へ加えません。
`ego.radian`の区間リセットから独立して保持し、引継ぎ周期の二重積分を避けます。
入口姿勢の分散にも旋回中のジャイロ雑音を加えて引き継ぎます。採用可能なEKF姿勢を使う場合はその分散、
それ以外は区間ジャイロ積分の分散を使い、出口で一律に標準偏差8°へ戻しません。
入口の分散が不明・不正、または残留角が不正な場合は既存の初期分散を使います。
新通路の横位置は、旋回中の10点平均を捨て、新しい側壁観測で確認してからEKFへ採用します。
観測待ちでも姿勢と分散の予測は継続します。新通路の横位置分散は従来の初期値に戻し、入口の横位置との相関は持ち越しません。
この分散伝播は既存のジャイロ雑音モデルによる近似で、実機での姿勢精度の校正値ではありません。
ターン寸法・半径・Lstart/Lend・FF係数は変更していません。

探索の側壁偏差15 mm、前壁左右差10 mm、前壁平均距離80 mm未満の補正条件を基本とします。
`control`では、以下のすべてを満たす場合だけ、10～20 mm未満の未確認な前壁左右差を補正対象から外します。

- EKFの位置・姿勢が採用可能で、現在の側壁観測を採用し、壁制御が追従中。
- 横偏差5 mm以内・位置標準偏差2 mm以下、姿勢偏差と姿勢標準偏差がともに2°以内、実測角速度0.5 rad/s以下。
- 前センサ双方が55～115 mmで、3 mm以上の前進観測の変化が同じ面と整合。
- 左右差の候補は10 mmで開始・8 mm未満で解除し、同じ符号のまま前進3 mmの確認にはまだ達していない。

3 mm以上続く候補、20 mm以上の左右差、前壁への接近、側壁の大偏差、信頼度・履歴不足では補正します。
`legacy/shadow`は従来の瞬時距離による判定を保ちます。設定は機体別の`search_turn_guard_param`で、実機調整前の試作値です。
この条件は同じ面との整合を確認する近似で、同じ実壁を観測したという実位置の保証ではありません。
補正回数と判断点は停止後にBLE GUIへ出力し、[設定JSONへ保存](tools/ble/README.md#探索の補正理由と判断点を記録する)します。
実機での中心走行・補正回数の改善量と処理時間は未確認です。

### 壁関連コードの配置

壁関連は、各層の責務ごとにまとめています。Task側の検出器・補助クラスを個別ヘッダへ分けず、次の5ファイルを起点に追えます。

| 責務 | 主な配置 | 内容 |
| --- | --- | --- |
| 検出 | [Task/Inc/wall_detection.h](Task/Inc/wall_detection.h) | 側壁の安定判定、Wall→Open、Open→Open、斜めの差分交差・極小区間 |
| 推定 | [Task/Inc/straight_wall_ekf.h](Task/Inc/straight_wall_ekf.h) | 直進・斜めで使う通路内の横位置/姿勢EKF |
| 制御 | [Task/Inc/straight_wall_control.h](Task/Inc/straight_wall_control.h)、[Task/Src/straight_wall_control.cpp](Task/Src/straight_wall_control.cpp) | 直進/斜めの壁制御と制御用ADC平均 |
| 補正判断 | [Task/Inc/wall_position_correction.h](Task/Inc/wall_position_correction.h) | 探索ターン前の採否、縦位置補正の候補選択・Shadow判定 |
| 診断 | [Task/Inc/wall_diagnostics.h](Task/Inc/wall_diagnostics.h) | 壁補正と柱候補の記録、停止後の出力 |

センサ値の接続と状態遷移は既存の`Task/Src/ir_sens_task.cpp`、モーションからの呼出しは`motion.cpp` / `turn_motion.cpp`で扱います。
前壁にも共通のIR範囲状態、旋回のジャイロ引継ぎ、迷路の壁データは、それぞれ既存の担当層に置きます。
PC側の壁・柱診断の受信/CSV出力は`tools/ble/wall_diagnostics.py`、動画時刻との対応は`tools/analysis/wall_event_review.py`です。
ファイルを統合してもクラス・数値・判定処理・ログ形式は維持しています。

壁・柱の診断バッファは各64件の記録枠をゼロ初期化領域（`.bss`）に置き、
記録する1件だけを元のイベント既定値で初期化します。
非ゼロの初期値による全件分のFlash消費を避け、記録上限・ログ形式・RAM使用量を維持します。
placement newは既存の記録枠を使い、ヒープ確保や記録時の全件初期化は行いません。

### 未接続の角加速度ベース直進壁制御

`Task/Inc/straight_wall_accel_control.h`に、固定の先読み距離`L`を使う独立した制御候補を置いています。
ヘッダー単独で使用でき、既存制御と同じ呼出しで試すための`Task/Inc/straight_wall_accel_adapter.h`も用意しています。
現在のIR処理は`StraightWallControl`を使い、候補を呼び出す変更はしていません。既存の`legacy/shadow/control`とA/Bの採用値は維持します。
既存実装の名前は`StraightWallControl` / `StraightWallAverage`、ファイルは`straight_wall_control.h/.cpp`へ統一しています。

直線通路・横滑りなしのモデルで、左正の横偏差`e`と姿勢`psi`を重み付けした合成誤差`s`を作ります。
`L`を先読み距離とみなす解釈は小角近似ですが、横位置の運動式には三角関数をそのまま使い、速度一定は仮定しません。

```text
s       = e + L * psi
s_dot   = v * sin(psi) + L * omega_measured
alpha   = -(Kp * s + Kd * s_dot + a * sin(psi)
            + v * cos(psi) * omega_measured) / L
omega_command_next = omega_command_previous + limited_alpha * dt
```

これにより、飽和がなくモデルどおりに追従する場合に`s_ddot + Kd*s_dot + Kp*s = 0`となります。
`L`・ゲイン・上限は構築時にコピーして固定します。`L`や速度からジャークを計算する処理はありません。
ゲインは`Kp = wn^2` [1/s²]、`Kd = 2*zeta*wn` [1/s]を基準に設計できます。

入力は横偏差[m]、姿勢[rad]、前進速度[m/s]、その速度と整合する並進加速度[m/s²]、
実測角速度[rad/s]、周期[s]です。位置・姿勢・角速度は同じ通路基準・時刻を使います。
`ego.x_point`を使う将来の接続ではmmからmへ換算し、姿勢は区間専用値または採用可能なEKF値を渡します。
ジャイロ実測角速度と、内部で積分・保持する指令角速度は区別します。

以下は合成モデル用の使用例です。数値はA/Bの調整済み・採用値ではありません。
67 mmは現行直進の姿勢/横偏差ゲイン比`20/300 m`を参考にした値です。

```cpp
#include "Task/Inc/straight_wall_accel_control.h"

StraightWallAccelControl::Parameters trial;
trial.lookahead_m = 0.067f;
trial.error_kp_per_s2 = 400.0f;
trial.error_kd_per_s = 40.0f;
trial.omega_limit_radps = 1.2f;
trial.alpha_limit_radps2 = 20.0f;
StraightWallAccelControl candidate(trial);

StraightWallAccelControl::Input sample;
sample.enabled = true;
sample.lateral_m = 0.002f;
sample.heading_rad = 0.01f;
sample.velocity_mps = 0.8f;
sample.accel_mps2 = 6.0f;
sample.measured_omega_radps = 0.0f;
sample.dt_s = 0.001f;
const auto command = candidate.update(sample); // 候補値の計算だけを行う。
```

角加速度を制限して指令角速度を陽的Euler積分し、角速度も制限します。
出力`alpha_radps2`は最終的な指令角速度の増分/dtへ合わせ、飽和中の未適用分は蓄積しません。
`requested_alpha_radps2`は制限前の式の値で、`error_m/error_rate_mps`は`s/s_dot`です。
指令を引き継ぐ検討では`reset(previous_command)`を明示的に呼びます。非有限・上限外の初期指令は拒否し0へ戻します。

`enabled`は既定false、`L`・ゲイン・上限の既定値0は未設定として拒否します。
無効化、不正入力・パラメータ、`dt <= 0`または`dt >= 20 ms`、
速度が`minimum_velocity_mps`（既定0.20 m/s）以下の場合は状態と出力を0へ戻します。
指令の変化量制限は`Active`中に適用し、停止・異常時の0へのリセットとは区別します。
壁の確認・信頼度・壁なし時の姿勢保持・立ち上げ率はこの部品に含めません。
将来接続する場合も既存の目標速度0.20 m/s以下の無効化と観測採用条件は呼出し側で維持する必要があります。

差し替え用の`StraightWallAccelAdapter`は、`StraightWallControl`と同じ`Input`・`Output`・`Phase`を公開し、
`reset()`・`prime_tracking(left_m, right_m)`・`update(input)`で呼び出せます。
左右壁の品質・採用マスク・phase・推定位置同期は既存実装を使い、`pose_control`が成立する通常追従だけを候補へ切り替えます。
10 mmの立上げ中は、同じ前回指令から作った従来指令と候補指令を`transition`で混合します。
確認中・AwayOnly・壁なしの姿勢保持は既存分岐を使い、実採用した指令を次周期へ同期して復帰時の連続性を保ちます。
上限は機体別の`wall_control_param.tracking_omega_limit/tracking_alpha_limit`を参照します。

将来試すときは、`sensing_task.h`の2つの制御器の型と初期化を次のように差し替えます。
以下も合成モデル用の例で、現在の走行処理には適用していません。

```cpp
#include "Task/Inc/straight_wall_accel_adapter.h"

StraightWallAccelAdapter straight_wall_control{{0.067f, 400.0f, 40.0f}};
StraightWallAccelAdapter ekf_wall_control{{0.067f, 400.0f, 40.0f}};
```

さらに、`ir_sens_task.cpp`で共通の`input`を作る箇所に、次の2入力を追加します。`filtered = input`にも引き継がれます。

```cpp
input.accel_mps2 = vehicle->ego.accel.get();
input.measured_omega_radps = vehicle->ego.rad_velo.get();
```

共通`Input`に追加した2フィールドは既定NaNです。現行の`StraightWallControl`では使用せず、
候補アダプタでは未供給・非有限の値を`InvalidInput`として出力を0へ戻します。既定構築も未設定として拒否するため、試験値を明示します。
アダプタの`Output.error_m/raw_error_m`は従来どおり右正の横偏差[m]を保ち、既存のIRログ・位置同期へそのまま渡せます。
`status()`と`using_acceleration_control()`で状態と候補の採用を、`acceleration_output()`で混合前の候補と左正の合成誤差を確認できます。
既存実装を再利用するため、試験時も`Task/Src/straight_wall_control.cpp`はビルド対象に残します。

検証は[ホストテスト](tools/tests/README.md)を参照してください。
機体別の実機調整、内側PIDの追従、横滑り・IR誤差・割込み時間は未評価です。

### 壁関連パラメータの配置

既存の走行パラメータと同様に、`Params/wall_param.h`で機体を選択し、
`wall_param_A.h` / `wall_param_B.h`に`const static`の構造体で値を定義します。
型は`typedef_wall_param.h`、直進制御は`wall_control_param`、EKFは`wall_ekf_param`、
斜め制御は`diagonal_wall_param`、直進壁切れは`wall_edge_param`、斜め柱通過は`diagonal_edge_param`、探索補正判定は`search_turn_guard_param`です。
壁切れの試験条件`wall_edge_trial_param`と探索基準、柱の`straight_pillar_param` / `diagonal_pillar_param`もA/B別ファイルへ統合しています。
柱の設定型`PillarPeakParam`も既存の`typedef_wall_param.h`で管理します。
距離の単位は構造体のフィールド名と定義側のコメントに記載しています。
既存の係数は維持し、A/Bで値が同じ場合も機体別に保持します。Bへの実測値の転用ではありません。

### BLEのturn_500最短走行と斜めEKF

BLE GUIの最短プロファイルは`turn_500（plain500）`が既定です。
既存の`mode_500`（機体別`turn_500.h`）と直進/斜めの500・600・700 mm/s設定を使います。
全区間を500 mm/sへ固定する設定ではありません。[操作・記録方法](tools/ble/README.md#斜め区間のekfを比較する)を参照してください。

機体Aの`Params/Param_A/turn_500/turn_500.h`は、2026-11-12の単体調整14種類を試験用に有効化しています。
半径・Lstart・Lend・角度補正を候補コメントと対応させ、PID・FFは既存値を維持しています。
論理角は経路判定用のまま保持し、物理的な旋回角の補正は`degree_correction`へ設定しています。
測定条件・入力資料・ばらつきは各テーブルのコメントを参照してください。連続走行・壁あり経路の検証は未実施です。

機体Bの`Params/Param_B/turn_500/turn_500.h`は、2026-11-16のユーザー依頼で右7種類の単体候補を左右14ターンへ試験反映しました。
対応する左右でLstart・Lend・旋回PIDを共通とし、半径・論理角・角度補正は符号を反転しています。
速度PIDは1.50 / 0.02 / 0、角速度PIDは0.10 / 0.002 / 0です。Bの既定FFを使用します。
直進プロファイル・ターン前後直進のゲイン選択・機体A・他速度のテーブルは今回の変更対象ではありません。
適用値と入力資料は[機体B turn500の反映記録](tools/turn_analysis/20261116_B_turn500_applied/README.md)を参照してください。
右の相対マーカー中点評価に基づく試験値で、左側・連続接続・壁あり経路は未検証です。
135度入口には旧動画の検出再点検が残っています。2026-11-16に機体BへDebug版を書き込み・検証・リセット済みです。反映後の走行確認は未実施です。

`legacy`は従来の斜め制御、`shadow`は同じ制御を保ってEKFを並行計算します。
`control`も従来の斜め制御則・速度依存ゲイン・軌道の制動を使い、信頼できるEKF接近偏差がある周期だけ横偏差入力を置き換えます。
EKF未確認・不採用・柱のない区間は、従来のIR/オドメトリ入力へ戻します。観測がないことを理由に従来の補正を0にしません。
斜めは断続的な柱を見るため、直進の45 mm平行壁モデルへ流し込みません。
既存の32 mm基準より近い25〜32 mm未満の観測を、前進3 mmの確認後、10 ms以上の間隔で横偏差の近似観測へ使います。
片側の前センサだけが近い場合は柱の観測候補を残し、両前センサが66 mm以下の場合は近接観測を除外します。
柱の変化から姿勢を補正・過信せず、姿勢は新しい斜め軸を基準としたジャイロ積分で予測します。
相対ジャイロ姿勢は診断用であり、絶対的な斜め方向として制御へ使いません。角速度・角加速度は従来の斜め制御則で計算します。
連続する同方向の斜め区間は状態を保ち、旋回・停止・軸の変更で初期化します。
斜め用の状態を機体の`ego.x_point`へ直接同期せず、前壁判定・壁切れ・旋回のPID/FFとは分けています。

これは32 mmルールに基づく限定的な接近モデルです。斜めの絶対位置・姿勢や最適な制御ゲインを実機同定したものではありません。
モータ停止後に斜め対象周期・EKF適用周期・観測更新数・従来入力周期・前方除外周期をGUI/設定JSONへ保存します。
`diagonal_controller:2`がこの併用方式の識別子です。既存55項目のバイナリ列順は維持します。

### 直進の壁切れ検知

`Task/Inc/wall_detection.h`は、迷路用`is_wall`と独立して壁切れイベントを生成します。
機体A/Bの距離校正を通った瞬時距離と、各制御回の符号付きエンコーダ移動量を使います。
設定は機体別の`wall_edge_param`です。試作値で、実機での検出位置精度は未評価です。
この検出は`wall_ekf legacy/shadow/control`の各モードに共通で、切替対象はEKFと壁制御への使用方法です。

- 見逃しを減らすため、65 mm以下の壁を0.5 mmの前進で確認してから検出待ちに入り、65 mm通過を候補にします。
  60 mm以内・3 mmの確認を要求していた方式より、遠めの壁や短い壁を候補にしやすくしています。
- 次以降の前進観測で66 mm以上・通過後0.5 mm以上を確認して、1周期だけイベントを出します。
  閾値へ戻る単発スパイクは取り消し、次の壁を再確認するまでは二重検出しません。
- 65 mmを通過した位置を前後2観測で補間し、確認までの前進距離を`r/l_corner_delay_mm`へ保持します。
  直近4制御回以内に前進があれば、ゼロカウント周期での通過も保持し、後続の前進で確認します。
  この場合の通過位置はその時点のカウント位置で、エンコーダ分解能より細かい位置は推定しません。
  探索直進・ターン前後の直進位置補正にはその距離を加え、左右の壁切れ位置記録では差し引きます。
  左右位置差による横位置補正は、その区間で両側のイベントを記録してから行います。
- 後退、非数値、1回6 mm超の移動、2 rad/s超の旋回では候補を破棄します。
  停止中は確定せず、短いゼロカウントは許容します。候補は8 mmまたは32制御回で失効します。
  `r/l_corner_time`は通過位置からの経過制御回数で、65535で飽和します。
- 迷路への壁登録条件は維持します。直進EKFは確定前の候補から壁端ガードを開始し、10点平均に残る過去の壁を観測へ再採用しません。

既存CSVの列数は維持しており、壁切れイベント・遅れ距離は収録していません。
5 ms間引きの距離だけでは1 msごとの誤検出率や実位置精度を確定できません。
確認条件を緩めた分、持続するノイズや短い柱の反応を拾う可能性があります。
65 mm以下の壁観測が一度も得られない壁は、この65 mm基準の検出対象には入りません。
デバッガでは`IrSensTask_type8::getInstance()`の`r/l_wall_corner`、`r/l_corner_delay_mm`、
`r/l_corner_length`を確認できます。検証方法は[ホストテスト](tools/tests/README.md)を参照してください。

### 壁切れ縦位置補正の試験切替

停止中の `wall_edge show|legacy|shadow|control` で切り替えます。起動時は `legacy`、Flashへは保存しません。
横壁制御の `wall_ekf` とは独立した、縦位置補正の試験です。

| wall_edge | 直進のアーム条件 | 縦位置補正 |
| --- | --- | --- |
| `legacy` | 従来の65 mm以下・0.5 mm。Open起点の独立柱は分離 | 従来の入口・出口・探索補正。Aの直進独立柱は最大半量で直接補正 |
| `shadow` | 62 mm以下を12 mm前進 | 候補と補正予定量を記録し、縦位置へ反映しない |
| `control` | 62 mm以下を12 mm前進 | 90 mm探索直進と直進からのターン入口だけ反映。斜め入口と両出口はShadow |

試験値は `Params/wall_param_A.h` / `wall_param_B.h` にまとめ、`wall_param.h`で機体を選択します。機体Aの探索直進の65 mm交差基準は
2026-11-16の探索2走行の映像照合を根拠とする試験候補49.5 mmです。機体Bは未測定のため45 mmを維持します。
ターン入口の7 mm基準は別の座標系なので変更しません。既存の残差窓・補正上限・融合率も維持し、
探索は許容残差8 mm・最大4 mm・半量、直進入口は許容残差8 mm・最大4 mmです。

12 mmの連続壁確認は独立柱の短いかすりを減らす条件です。短い壁・62 mmより遠い壁は候補から外れ、
長く見える柱まで完全に識別できる条件ではありません。Open→Openと斜め柱の追加検出器は、下記のShadow記録で比較します。
最短の一般直進途中の縦位置基準、角度・横位置に応じた基準位置係数は未確定です。

全モードで、左右候補を同じ補正前位置から評価し、有効な候補を選んだ後だけ境界を使用済みにします。
wrong_side・窓外・古い世代の候補は、後続の正しい側をブロックしません。同時に有効なら残差が小さい側を1回だけ使用します。
旋回本体では即座に `TURN_IRSENS` へ切り替え、同一IRQ内で出口へ進んでも旋回中の候補を持ち越しません。
探索スラロームの専用経路でも同じ切替を行います。出口は直進IRモードへ戻してから旋回残留角を引き継ぎ、モード変更による推定値の初期化で消えない順序にします。
同じ方向の直進→ターン前直進はアーム履歴を保ち、補正区間を更新すると過去のイベント・確定待ち候補を破棄します。
これらの候補管理修正は `legacy` にも適用するため、旧バイナリと完全同一の比較モードではありません。

`WALL_CORR` は `mode`、`feature`、`generation`、`event_generation` と、適用前の
`proposed_reason` / `proposed_delta_mm` / `proposed_after_mm` を追加記録します。
Shadowでは `reason:shadow`、`delta_mm:0`、`after_mm=before_mm` となります。
Shadowでは従来の横壁制御と差分交差ペアを使用します。Legacyでは追加の柱極値ペアも横位置補正へ採用し、斜めの差分交差ペアによる二重補正を止めます。Controlで斜め柱姿勢EKFを使う間も、従来の柱ペアから横位置を代入する補正を止めます。
実機評価は未実施です。まず同じ条件でShadowの候補・残差・映像対応を確認し、その後Controlとの差を比較します。

### 直進独立柱・斜めの極小区間を扱う追加検出器

追加の2検出器は`wall_edge legacy` / `shadow` / `control`の全モードで動作します。
LegacyではA/Bとも、左右の直進ADC極大・斜め距離極小をそれぞれ別のペアとして横位置補正へ採用します。
確認時点からの移動距離を引いて極値位置をそろえ、直進は左右差8 mm未満（探索は5 mm未満）、斜めは従来の区画間隔と±20 mmの窓で評価します。
単独の柱、品質棄却、停止・後退・欠損・旋回、補正区間の変更をまたぐ古いペアは使いません。従来の壁切れ・差分交差とは履歴を混ぜません。
斜めLegacyの差分交差はターン入口・出口の従来の縦位置補正に残し、横位置補正は距離極小のペアへ切り替えます。
機体Aの90 mm探索直進と最短の直進ターン入口の独立柱は、LegacyではEKFを使わず最大半量・±4 mmで縦位置も直接補正します。
`wall_ekf control`かつ`wall_edge control`では縦位置誤差のEKF観測になります。
同じ区間のWall→Openとラッチを共有し、同時なら壁切れを優先して二重補正を防ぎます。Bの未校正の縦位置と、斜め極小からの縦位置直接補正は無効です。
停止中の`wall_edge show`で`pillar_active`（全モード1）、`pillar_pair_apply`（Legacy時1）、`pillar_apply/pillar_entry_apply`（縦位置補正）を確認できます。

- `StraightPillarEdge`（Open→Open）は、距離78 mm以上のOpenを12 mm前進して確認し、生ADCの100以上の山を検出します。
  ADCが50以上減少し、Openかつ手前基線に近い値へ戻ることを確認します。距離80 mmへ張り付く弱い柱も生ADCで扱います。
  その後に62 mm以下の連続壁を12 mm確認した場合はOpen資格と柱候補を破棄し、壁切れを独立柱として扱いません。この壁確認には機体別`wall_edge_trial_param`の条件を共用します。
  連続壁後の壁切れ、壁へ入り続ける波形、1周期のスパイクは別扱いです。
- `DiagonalPillarEdge`は60 mm以内で接近を確認し、距離極小・極小付近0.25 mm幅の底の区間・3 mm以上の離反を追います。
  初期10 mmの平均窓充填は不要です。長い接近と平坦な底を分け、極小からの保持距離で失効させます。
  検出後20 mm以上の前進と十分な離反の両方を要求し、解除条件による短距離の二重発火を抑えます。
- 途中で|yaw|>2 rad/sとなっても生波形の追跡は続け、該当候補を`quality_rejected`として記録します。
  その姿勢変化をまたいだ候補を、有効な位置観測として確定しません。後退・非数値の距離・無効ADC・6 mm超のサンプル欠損は追跡を初期化します。

設定は`Params/wall_param_A.h` / `wall_param_B.h`の試験初期値です。直進と斜めで、信号の単位・Open条件・離反条件を分けています。
Bは2026-11-14の録画探索`run_153203_022162`の再生を根拠に、Open後の接近・離反の保持距離を40 mmとします（Aは32 mm）。
Bの広い独立柱を壁切れへ混同しないための条件で、縦補正の位置基準は未校正のためBの実適用は無効です。
特徴点型は`straight_pillar_adc_maximum`と`diagonal_pillar_minimum`です。
既存の`wall_edge_crossing`や`diagonal_difference_crossing`とは基準位置が違うため、既存の左右ペア補正に混ぜません。

`PILLAR_EDGE`に極小サンプルの実MCU時刻と確認時刻、底の区間幅、特徴点座標・確認遅れを保存します。
極小時の生ADC4本・距離4本・範囲状態、機体の`ego.x_point/ego.radian/gyro`、制御側の横偏差・姿勢も対応付けます。
制御側推定はIR採取前の最新制御周期の値で、絶対姿勢の校正値ではありません。機体の角度0・横位置0を中心/正対の証拠には使いません。
前壁は片側/両側の範囲状態と応答を保存し、追跡窓内で両前壁が有効範囲内かつ66 mm以下になった候補には品質フラグを付けます。
片側前壁だけで候補を消さず、範囲外の125 mmを角度観測とみなしません。

座標世代の不一致、極小から確認までの座標ジャンプ、大きな回転、前壁近接対、斜め側壁の近距離限界、無効な前壁/姿勢は品質フラグで区別します。
同方向の補正区間更新では接近履歴を残し、極小サンプルの世代が古ければそのまま不採用の証拠として記録します。
実機Aの90 mm探索直進は39.5 mmを基準に補正予定量を記録し、Legacyでは直接、両control時には3状態EKFで実適用します。残差8 mm以内・最大半量・最大4 mmです。
Bおよび斜め・ターン出口には、この39.5 mm基準を転用しません。最短の直進ターン入口は別原点の−3 mmを使います。

走行後JSONの`pillar_diagnostics`、`run_*.pillar_candidates.csv`へ保存します。各軸の候補数・品質棄却数と先頭64件を保持し、超過は`dropped`で示します。
動画比較は極小サンプルの時刻へ移動し、確認時刻も別列に残します。5/6 ms保存波形での再生結果は1 msの実機検出率ではありません。
既存`DiagonalWallEdge`も並行して残し、新旧の特徴点・位置補正効果を区別して試験します。

### 壁切れ・柱検出のLED表示

Wall→Open、従来の斜め差分交差、追加の直進Open→Open、追加の斜め極小区間は、有効な検出が確定すると右はLED6（`0x10`）、左はLED1（`0x20`）を約50 ms点灯します。
追加2検出器は診断ログが無効・記録上限に到達した場合も表示し、`quality_rejected`候補では点灯を開始しません。
アーム成立ではなく離反などの確認が完了した時点の表示で、極小サンプル時刻とは確認遅れ分だけ異なります。
Shadowでも表示するため、点灯は位置補正の実適用を意味しません。実適用は診断の`delta_mm`と`PILLAR_STATS.applied`で確認します。

点灯保持は`Peripheral/Src/interface.c`で通常のLED要求へ重ね、1 ms制御割り込みで消灯時間を更新します。
区画開始や従来の消灯要求があっても保持時間を維持し、他のLEDは通常の要求どおりに表示します。
`Return_LED_Status()`は一時点灯を除いた通常要求を返すため、読んでORする処理で検出LEDが点灯したまま残りません。

### 斜め走行中の柱通過・壁切れ検知（既存方式）

`Task/Inc/wall_detection.h`は、柱の距離が近づいてから遠ざかる形を検出します。
直進の65 mm通過とは異なる基準で、`legacy/shadow/control`共通です。設定は機体別`diagonal_edge_param`です。

- 20制御回の時間平均をやめ、1 mm間隔の11点・10 mmの距離履歴を使います。
  60 mm未満で負の距離差を前進0.5 mm確認し、差が0を経由して正へ変わる場合も候補にします。25 mmの飽和区間の後も対象です。
- 以後の前進で、差0.5 mm以上・候補位置から距離値1 mm以上の上昇・前進0.5 mm以上を確認して1回通知します。
  0.25 mmを超えて戻る周期は確定を見送り、負の差や候補時の距離へ戻れば取り消します。候補は8 mmまたは32制御回で失効します。
- 65 mm以上を前進1 mm確認するか、検出後20 mm以上進んで候補時の距離値より5 mm以上離れてから、次の近づく柱を待ちます。
  柱の間で65 mmに達しない場合も再検出でき、同じ柱の近傍の二重検出を抑えます。
- 0差の通過位置を補間し、確認までの前進距離を`r/l_corner_delay_mm`へ保持します。
  ターン前後の斜め距離補正へ加え、左右のイベント位置記録では差し引きます。左右両方の有効な記録が揃うまで横位置を補正しません。
- 停止指令、後退、非数値、1回6 mm超の移動、2 rad/s超の旋回、座標軸変更では履歴を破棄します。
  4制御回以内のゼロカウントは許容しますが、前進なしでは確定しません。

`SHORTEST_CAP/SHORTEST_WALL_STATS`の`diagonal_edge:1`が新方式の識別子です。
走行後のGUIに左右の検出回数、設定JSONに検出回数と最大確認遅れ[mm]を保存します。CSVの列順は変更しません。
検出回数は補正を採用した回数ではありません。5 msログから欠けた1 msの検出状態や実位置を復元することもできません。
10 mm基準は従来の0.5 m/s・約20 ms相当ですが、速度依存の検出位置を変更するため、実機で位置精度を再確認してください。
ターン表・Lstart/Lend・`DIAGONAL_CORRECTION`は変更していません。

## demo / demo2

demoとdemo2は、探索・経路確認・最短走行・時間設定・迷路履歴の全16項目を共用します。
項目番号と走行設定は従来のdemoに合わせ、demo2も同じ手かざし閾値で開始します。
demoでは`wall_ekf`・`wall_edge`・`turn_ekf`をすべて`control`、demo2ではすべて`legacy`にします。
EKFの機体別校正・速度・品質による採用条件は各推定器の設定に従います。
メニュー終了時は、入る前の各制御設定に戻します。

| 項目 | 動作 |
| --- | --- |
| `0`～`3` | 往路・復路の探索（従来demoの各探索方式） |
| `4` | 経路確認（`run_config_uniform1000`） |
| `5` | 非吸引の最短走行（`run_config_plain500`） |
| `6` | 探索時間設定 |
| `7` | 迷路履歴の確認・読込み |
| `8`～`E` | 従来demoの各設定による吸引最短走行 |
| `F` | メニュー終了 |

メインのdemo選択ではdemo → demo2の順に開きます。demoの`F`でlegacyのdemo2へ移り、
demo2の`F`でメインのモード選択に戻ります。

## ローカル座標と全体座標

図と操作例を含む説明は [Moduleの座標系ガイド（HTML）](Module/coordinate_systems.html) を参照してください。外部ライブラリ不要で、ローカルファイルとして開けます。

`Module/Inc/vehicle.h`の`Vehicle::pose`で、各動作のローカル座標と走行全体の座標を機体ごとに保持します。
`ego`は実測・制御用の区間状態、`ideal`は目標状態、`pose`は共通の座標と変換原点です。`machine_params`の項目は前後運動・横運動・上下加速度・姿勢・滑りにまとめ、初期化を共通化しています。
幾何変換と更新処理の型は`Module/Inc/pose_frames.h`に置き、独立したsingletonや共有状態を持たせません。`local`は`world`と`origin`から参照時に算出し、内部への重複保存を避けています。
各EKFと既存の`ego.length / x_point / radian`は固有の座標・単位を維持します。位置の二重管理を避けるため、`turn_x/y`と`turn_x/y_dash`の保存・独立積分は廃止しました。滑り角`turn_slip_theta`は姿勢と異なる物理状態として残し、読み出しのない`turn_slip_dot`は削除しています。

| 座標 | 原点・軸 | 更新・保持 |
| --- | --- | --- |
| 全体`world` | 既定は走行開始位置(0,0)、開始方向が+X、左が+Y。姿勢は左旋回正 | センサ移動量と採用した推定補正を一度ずつ反映。動作切替・旧egoの原点変更で消さない |
| ローカル`local` | 動作開始点が位置原点、経路方向が+s、左が+e。旋回中は入口の軸に固定 | 新モーションまたは直進/旋回/斜め等のphase変更時に位置原点を更新。姿勢偏差は残す |
| 変換`origin` | ローカル原点の全体位置と軸方向 | `world = origin × local`。回転と平行移動で相互変換 |

位置はmm、内部姿勢はradで、`vehicle.pose.snapshot()`（ポインタの場合は`vehicle->pose.snapshot()`）から3者を同時に取得できます。
モーション・IRタスク・シェルも対象の`Vehicle`が所有する状態を使います。`ego_initialize()`と区間積分の初期化は全体位置・動作原点を保持してログ用原点だけを更新し、`ideal_initialize()`は`pose`を変更しません。走行開始/終了で`pose.start()` / `pose.stop()`を明示します。
`motion`はモーション番号、`frame`は座標系番号です。同じphaseの制御呼出しや次設定待ちでは原点を繰り返し更新しません。
90/180°・45/135°・V90・探索旋回・その場旋回の完了時は、論理角だけ経路方向を更新します。角度補正を論理角へ混ぜず、実際の姿勢は測定した旋回量から保持します。

通常は符号付きエンコーダ移動量とジャイロから更新し、ターンEKF採用中はその位置・姿勢増分を直接回転して使います。
直進では固定した通路座標から横位置・採用姿勢を置き換え、適用した縦補正だけ進行方向へ反映します。
斜め柱EKFは開始時に全体座標へ結び付け、校正・品質条件を満たす採用状態を変換して反映します。各EKFの同じ観測を全体側で再び融合しません。
共分散は各EKFに保持し、固定原点での回転用関数は`rotate_covariance()`です。全体座標の新しいEKFや、原点不確かさを含む全体共分散は追加していません。

`ego.turn_x/y`は従来の列位置・左/前のmm単位を保ち、`pose.log_position()`から算出します。
`reset_turn_position_for_log()`は従来の位置ゼロ化箇所でログ用原点だけを変更し、`pose`の全体位置と動作ローカル位置は保持します。固定原点の向きはその時点のセンサ姿勢を考慮するため、制御phaseの原点とは独立です。
値の生成元は単一の推定位置へ移り、エンコーダと採用したEKF・壁補正を反映します。以前の速度による独立積分との数値一致を前提にしないでください。
旧`ideal.turn_x/y`の空き2枠は`estimate.heading_rad` / `estimate.sigma_heading_rad`へ変更しました。
直進・ターン・斜めで引継ぎに採用する姿勢と標準偏差をradで記録し、有効な推定がない場合は両方NaNです。
`wall_ekf control`が必要で、斜めは`wall_edge control`も必要です。ターンEKF不採用時は接続用のジャイロ予測を記録します。
旋回中は入口軸、出口からは論理角を引いた出口軸での姿勢です。詳しくは[姿勢ログの仕様](tools/ble/README.md#採用姿勢の2項目)を参照してください。

停止・エラー後も最後の座標を参照できます。次の`Motion_start()`は設定済み開始座標から始めます。
停止中の`pose reset [x_mm y_mm yaw_deg]`で開始座標を指定でき、`pose reset`単独は(0,0,0)へ戻します。Flashには保存しません。
たとえば迷路の実際の開始位置が(45,45) mmで北向きなら`pose reset 45 45 90`です。
`pose show`で両座標と変換原点・経路方向・有効性を確認します。詳細は[座標の操作](tools/myshell_README.md#ローカル全体座標)を参照してください。
センサ入力が非有限または1周期の移動範囲外なら、その増分を捨てて`valid:0`と`rejected`を残します。原点を再設定するまで欠測の印を保持します。
周期ログは55項目を維持し、意味の変わった2枠を区別するためBLEは`L8S5`、保存magicは`0xA55D`です。更新後のPCツールは旧形式も読み分けます。全体位置の実機精度と追加処理のIRQ時間は未測定です。

## クロック設定

機体A/B共通で、PLLのSYSCLK出力100 MHzをHCLKへ分周せずに渡し、コアを100 MHzで動かします。
タイマ入力クロックと既存のPWM・センサ周期は維持します。

| 対象 | 従来 | 現在 |
| --- | --- | --- |
| SYSCLK（PLL出力） | 100 MHz | 100 MHz |
| コア / HCLK | 50 MHz（SYSCLK / 2） | 100 MHz（SYSCLK / 1） |
| APB1 / APB2バス | 50 MHz（HCLK / 1） | 25 MHz（HCLK / 4） |
| TIM1/2/3/4/8入力 | 50 MHz | 50 MHz（APB分周ありのためPCLK × 2） |
| APB3バス | 50 MHz | 50 MHz（HCLK / 2） |
| IR用LPTIM1 / ADC4トリガ用LPTIM3 | HSI16 / MSIK | 同じクロック源・分周・周期を維持 |
| ADC1 / ADC4変換クロック | HCLK 50 MHz / 2 = 25 MHz | HCLK 100 MHz / 4 = 25 MHz |
| SPI2 SCK | SYSCLK 100 MHz / 32 = 3.125 MHz | 同じ |
| USART1ボーレート生成用クロック | PCLK2 50 MHz / 1 | SYSCLK 100 MHz / 2 = 50 MHz |

TIMのPSC・ARR・比較値とエンコーダ設定は変更しません。モータPWM 200 kHz、ファンPWM 100 kHz、
既存の約1 ms制御周期、UART設定460800 bpsを維持します。
`Peripheral/Src/motor.c`の既存の`PCLK1`/`PCLK2`定数はタイマ入力50 MHzとして使われており、
今回のAPBバスクロック25 MHzへ置き換えないでください。
電圧スケールはRange 2を維持し、Flash待ち時間を1 WSから3 WSへ変更します。
100 MHz・Range 2での待ち時間は[STのFlash解説](https://www.st.com/resource/en/product_training/stm32u5-memory-flash.pdf)を参照してください。

`SystemClock_Config()`、ADC・USARTの初期化と`mouse_type8.ioc`を対応させています。
単にAHB分周だけを外すとPWM・ADC・通信速度も変わるため、周辺設定も同時に扱ってください。
HALのSysTickはHCLKに追従し、割り込み時間計測のDWT換算も`SystemCoreClock`を使用します。
実機では更新後ログの`irq.total_us`・`irq.max_us`・`irq.period_us`で処理余裕と周期を再確認します。

## VS Code コマンド集

### CMake preset の選択

コマンドパレットを開きます。

```text
Ctrl + Shift + P
```

以下を実行します。

```text
CMake: Select Configure Preset
```

機体とビルド種別に合わせて、以下の preset を選択します。

| Preset | 機体 | パラメータ | IMU |
| --- | --- | --- | --- |
| `Debug-MouseA` | MOUSE_A | Param_A | LSM6DSV16X |
| `Debug-MouseB` | MOUSE_B | Param_B | LSM6DSV80X |
| `Release-MouseA` | MOUSE_A | Param_A | LSM6DSV16X |
| `Release-MouseB` | MOUSE_B | Param_B | LSM6DSV80X |

従来の `Debug` / `Release` preset は互換性のためMOUSE_Aとして扱われます。MOUSE_Bの開発では、必ず名前に `MouseB` が付いたpresetを選択してください。

### Configure

```text
CMake: Configure
```

ターミナルからMOUSE_Aを設定する場合:

```powershell
cube-cmake --preset Debug-MouseA
```

MOUSE_Bを設定する場合:

```powershell
cube-cmake --preset Debug-MouseB
```

### Build

```text
CMake: Build
```

ターミナルからMOUSE_Aをビルドする場合:

```powershell
cube-cmake --build --preset Debug-MouseA
```

MOUSE_Bをビルドする場合:

```powershell
cube-cmake --build --preset Debug-MouseB
```

Debugビルドの成果物は機体別のディレクトリに出力されます。

```text
build/Debug-MouseA/mouse_type8.elf
build/Debug-MouseA/mouse_type8.map
build/Debug-MouseB/mouse_type8.elf
build/Debug-MouseB/mouse_type8.map
```

### Clean

VS Code から実行する場合:

```text
CMake: Clean
```

ターミナルから Ninja の clean を実行する場合:

```powershell
cube-cmake --build --preset Debug-MouseA --target clean
```

### Rebuild

```text
CMake: Clean Rebuild
```

ターミナルから実行する場合:

```powershell
cube-cmake --build --preset Debug-MouseA --clean-first
```

MOUSE_Bの場合は、presetを `Debug-MouseB` に置き換えます。

### compile_commands.json の更新

補完や clangd の参照情報を更新したい場合は Configure を実行します。

```text
CMake: Configure
```

Debug preset の場合、機体ごとに以下のファイルが使われます。

```text
build/Debug-MouseA/compile_commands.json
build/Debug-MouseB/compile_commands.json
```

### STM32CubeMX 設定を開く

Windows のターミナルから:

```powershell
.\open-ioc.cmd
```

または `mouse_type8.ioc` を VS Code / Explorer から開きます。

## よく使うターミナルコマンド

```powershell
# MOUSE_A Debug configure
cube-cmake --preset Debug-MouseA

# MOUSE_A Debug clean build
cube-cmake --build --preset Debug-MouseA --clean-first

# MOUSE_B Debug configure
cube-cmake --preset Debug-MouseB

# MOUSE_B Debug clean build
cube-cmake --build --preset Debug-MouseB --clean-first

# MOUSE_A Release build
cube-cmake --preset Release-MouseA
cube-cmake --build --preset Release-MouseA

# MOUSE_B Release build
cube-cmake --preset Release-MouseB
cube-cmake --build --preset Release-MouseB

# .ioc を開く
.\open-ioc.cmd
```

## 注意点

- `cmake/vscode_generated.cmake` は STM32CubeIDE for VS Code が管理する生成ファイルです。ソース追加や CubeMX 設定変更後に再生成される可能性があります。
- ユーザー定義の CMake 設定を追加する場合は、まず `CMakeLists.txt` の `User defined` セクションを使います。
- `.ioc` を変更した後は、生成されたソースや CMake 設定に差分が出るため、ビルド前に `CMake: Configure` を実行してください。
- 機体別のビルドディレクトリは `build/Debug-MouseA`、`build/Debug-MouseB`、`build/Release-MouseA`、`build/Release-MouseB` です。

### Flash / Write to MCU

Debugビルド後、ST-LINKを接続してSWD経由で書き込みます。ビルドした機体と書き込み対象を必ず一致させてください。

MOUSE_A:

```powershell
cube-cmake --preset Debug-MouseA
cube-cmake --build --preset Debug-MouseA --clean-first
cube programmer -c port=SWD -d .\build\Debug-MouseA\mouse_type8.elf -v -rst
```

MOUSE_B:

```powershell
cube-cmake --preset Debug-MouseB
cube-cmake --build --preset Debug-MouseB --clean-first
cube programmer -c port=SWD -d .\build\Debug-MouseB\mouse_type8.elf -v -rst
```

`cube programmer` が見つからない場合は、STM32CubeProgrammer の CLI を直接指定します:

```powershell
& "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" -c port=SWD -d .\build\Debug-MouseA\mouse_type8.elf -v -rst
```

MOUSE_Bの場合は、書き込みファイルを `.\build\Debug-MouseB\mouse_type8.elf` に変更します。

## HY0020 による BLE 通信

USART1 を HY0020 経由で BLE に中継するプログラムと PC ツールを同梱しています。
配線・ビルド・書き込みは [firmware/hy0020/README.md](firmware/hy0020/README.md)、
専用シェルと GUI の使い方は [tools/ble/README.md](tools/ble/README.md) を参照してください。
対象は Mouse A / B 共通の UART インターフェースです。HY0020 は STM32 とは別にビルドします。
HY0020は従来品をA、新品をBへ割り当て、機体別の広告名を`HY0020-MOUSE-A` / `HY0020-MOUSE-B`とします。
BLEアドレスと機体別HEXの作り方は[HY0020の対応表](firmware/hy0020/README.md#機体とhy0020の対応)を参照してください。

USART1 と HY0020 の UART は **460800 bps / 8N1** です。115200 bps 版から変更する場合は、
STM32 本体と HY0020 の両方に対応するファームウェアを書き込んでください。
PC を有線で直結するツールも 460800 bps に合わせます。旧ファームウェアには 115200 bps を指定します。
書き込み時の電源操作・配線切替を終えてから BLE へ接続します。

`python tools/ble/mouse_ble_log.py` で全55項目を既定20 ms周期のバイナリとして連続受信し、
CSVへ逐次保存できます。ストリーム対応STM32ファームウェアが必要です。
短周期対応版では `--period-ms` で3～1000 msを指定できます。20 ms未満にはSTM32本体の更新が必要です。
使い方と帯域・欠落の扱いは [連続ログ取得](tools/ble/README.md#ログを連続取得する) を参照してください。

`tools/ble/mouse_ble_explore_gui.py` は、BLE接続確認・手かざし待ち付きの実機探索開始・
ライブ表示・探索中の連続ログ保存を一画面で行います。
接続後はログ配信を停止して待機し、探索開始（手かざし待ちを含む）から終了・中断まで配信します。
新しい `explore` コマンド対応ファームウェアとPython 3.10以上・Tkが必要です。
[探索GUIの使い方](tools/ble/README.md#guiで実機探索と逐次ログを操作する) を参照してください。
ログにはADC制御割り込みの処理時間・ログ処理時間・最大時間・開始間隔の4列を追加しています。
[割り込み時間の確認方法](tools/ble/README.md#割り込み時間をログで確認する) を参照してください。

## 赤外線センサ表示

赤外LEDは起動時・通常のメニュー／シェル待機時に消灯し、走行中と手かざし開始待機中だけ発光します。
手かざし検出後のカウントダウン、待機の解除、走行終了・異常停止では消灯します。表示用LEDの動作とは別です。
demoから開く迷路データの読込み・履歴確認と時間設定も、実行を選んで手かざしを待つ間だけ発光します。
ボタン操作も必要な項目では、手かざしとボタンが両方そろうまで待機を維持します。
異常停止後に手かざし待機を選び直すと再点灯します。待機だけではモータ制御やエラー状態を復帰させず、
次の走行開始処理で前回のエラー状態とカウンタをリセットしてから制御を有効にします。
ADCと電池電圧の取得は消灯中も継続します。再点灯後は2回のADC完了までIR差分値を無効にし、
走行開始時には3 ms待ってから初期の壁情報を利用できるようにしています。

センサ値と換算距離の表示は、shellモードで `irsens 100 100`（100回、行間に100 ms待機）を送信します。
この表示だけでは赤外LEDを点灯しません。消灯中・点灯直後のIR差分値は0で、換算距離は反射光の測定値として扱いません。
使い方は [myshell シリアル操作ガイド](tools/myshell_README.md#irセンサ表示) を参照してください。

## 左右の壁制御の使用判定

左右センサの安定性は、エンコーダの前進距離に合わせて1mm刻みで測定値を補間し、直近6mmの7点平均と現在値との差で判定します。差の絶対値が5.0mm未満の状態で10mm進むと、制御に使う壁までの距離上限を基準距離（直進45mm・斜め32mm）から65mmへ広げます。履歴が空の場合は、最初に6mm分の履歴を集めます。

停止中は確認距離を増やさず、壁の消失・対応する前センサによる前壁接近・壁切れ検出・後退でその側の判定履歴をクリアします。判定用の定数は `Task/Inc/wall_detection.h` にあります。判定差の閾値は実機の走行ログに合わせて調整してください。

基準距離以内の壁をすぐ制御に使う動作、診断用の20サンプル距離履歴、低速時の制御抑制、制御ゲインと制御式は従来どおりです。

## turnpattern GUI

最大20個のターンを組み合わせて本体へ送信するGUIは、次のコマンドで起動します。

```powershell
python .\tools\myshell_turnpattern.py
```

初回のみ `pyserial` が必要です。

```powershell
pip install pyserial
```

GUIでは走行パターンをJSONファイルとして保存・再読込でき、ターン速度、前走加速度、後走減速度、吸引を設定できます。`Apply & exe` は設定とパターンを送信した後、前センサ待ちで走行を開始します。ログは `tools/logs`、保存したパターンは既定で `tools/turnpatterns` に出力されます。
