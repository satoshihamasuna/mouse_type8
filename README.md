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

直進・探索・ターン前後の直進壁制御は、ローカルの
`program_zirconia/zirconia/software/Control/Src/wall_control.c` と
`Params/control_param.h` の `independent_side_pose_v3` をtype8へ移植しています。
実装は `Task/Src/zirconia_wall_control.cpp`、初期ゲインは
`Params/wall_param_A.h` / `wall_param_B.h` の `wall_control_param`、検証方法は [tools/tests/README.md](tools/tests/README.md) を参照してください。
左右独立の信頼度評価、3 mmの新規壁確認、10 mmの補正立上げ、予測横偏差と姿勢制動を使用します。
姿勢制御時の上限は角速度1.2 rad/s・角加速度20 rad/s²、その他は0.2 rad/s・4 rad/s²です。
これらはzirconiaの初期値であり、Mouse A/Bでの実機調整は未実施です。

移植範囲は横壁制御です。前壁合わせ、内側の速度・角速度PID、機体別距離校正は既存実装を使用します。
斜め直線は従来制御を基本とし、`control`では下記の接近観測を使う斜めEKF制御へ切り替えます。
直進の壁切れイベントと位置補正は、下記の確認・遅れ補償を使います。
直進壁制御の観測だけに10点ADC平均と65/70 mmの壁有無ヒステリシスを使用し、
従来の壁切れ・迷路用判定の履歴から独立させています。前壁の存在だけでは側壁制御を抑制しません。
type8の横位置推定値（左正・mm）をzirconiaの偏差（右正・m）に換算し、安定壁が得られた場合だけ同期します。
姿勢偏差は壁制御区間のジャイロ積分を別に保持し、従来の壁切れ処理による姿勢リセットの影響を避けます。
無効化・モード切替・別通路で制御状態を初期化し、同じ通路の連続直進・ターン前直進では制御phase・姿勢・推定を維持します。
探索ターン後は旋回の残留角を新しい通路の姿勢として引き継ぎ、新しい10点のADC平均を待って壁制御を再開します。
探索ターンの出口直進長には、入口の左正の横偏差を符号を保って±10 mmに制限し、右ターンで加算・左ターンで減算します。
従来の0.20 m/s以下の測定採用制限を維持し、新直進制御ではこの範囲の補正出力を0にします。

直進壁制御には、通路内の横偏差・姿勢偏差を推定する固定2状態EKFを組み込んでいます。
起動時は`shadow`（従来制御で走り、EKFを並行計算）です。停止中の`wall_ekf control`で
壁の品質・予測残差・共分散に基づく壁採用と、EKFの横偏差・姿勢を使う制御へ切り替えます。
EKF姿勢は区間ジャイロ積分との差が3°以内で、共分散の条件も満たす場合に採用します。
不整合・観測不採用・確認中は並行計算中の従来制御へ戻し、EKF側の古い旋回指令を破棄します。
全壁喪失時は信頼できる姿勢だけを短距離保持し、それも不確かな場合は補正を0にします。
壁なしでは予測を継続し、壁切れの横位置補正は観測として一度だけ取り込みます。
進行距離の壁切れ補正は既存処理と併用します。探索の前壁左右差判定は、`control`時だけ推定と観測履歴による確認を加えます。
機体A/Bの校正を維持し、係数の実機調整は未実施です。
操作・ログの意味は[直進壁EKF](tools/myshell_README.md#直進壁ekf)を参照してください。

1. `Core/Src/main.c` で HAL と各ペリフェラルを初期化します。
2. `CPP_Main()` を呼び出し、C++ 側の初期化へ入ります。
3. `System/Src/mode.cpp` の `Mode::Select_Mode()` でエンコーダとボタンによるモード選択を行います。
4. 選択したモードに応じて、インターフェース確認、デモ走行、デバッグ走行、シェル実行などを呼び出します。

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

### 壁関連パラメータの配置

既存の走行パラメータと同様に、`Params/wall_param.h`で機体を選択し、
`wall_param_A.h` / `wall_param_B.h`に`const static`の構造体で値を定義します。
型は`typedef_wall_param.h`、直進制御は`wall_control_param`、EKFは`wall_ekf_param`、
斜め制御は`diagonal_wall_param`、直進壁切れは`wall_edge_param`、斜め柱通過は`diagonal_edge_param`、探索補正判定は`search_turn_guard_param`です。
距離の単位は構造体のフィールド名と定義側のコメントに記載しています。
既存の係数は維持し、A/Bで値が同じ場合も機体別に保持します。Bへの実測値の転用ではありません。

### BLEのturn_500最短走行と斜めEKF

BLE GUIの最短プロファイルは`turn_500（plain500）`が既定です。
既存の`mode_500`（機体別`turn_500.h`）と直進/斜めの500・600・700 mm/s設定を使います。
全区間を500 mm/sへ固定する設定ではありません。[操作・記録方法](tools/ble/README.md#斜め区間のekfを比較する)を参照してください。

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

`Task/Inc/straight_wall_edge.h`は、迷路用`is_wall`と独立して壁切れイベントを生成します。
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

### 斜め走行中の柱通過・壁切れ検知

`Task/Inc/diagonal_wall_edge.h`は、柱の距離が近づいてから遠ざかる形を検出します。
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

センサ値と換算距離の表示は、shellモードで `irsens 100 100`（100回、行間に100 ms待機）を送信します。
使い方は [myshell シリアル操作ガイド](tools/myshell_README.md#irセンサ表示) を参照してください。

## 左右の壁制御の使用判定

左右センサの安定性は、エンコーダの前進距離に合わせて1mm刻みで測定値を補間し、直近6mmの7点平均と現在値との差で判定します。差の絶対値が5.0mm未満の状態で10mm進むと、制御に使う壁までの距離上限を基準距離（直進45mm・斜め32mm）から65mmへ広げます。履歴が空の場合は、最初に6mm分の履歴を集めます。

停止中は確認距離を増やさず、壁の消失・対応する前センサによる前壁接近・壁切れ検出・後退でその側の判定履歴をクリアします。判定用の定数は `Task/Inc/side_wall_stability.h` にあります。判定差の閾値は実機の走行ログに合わせて調整してください。

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
