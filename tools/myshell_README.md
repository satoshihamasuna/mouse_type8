# myshell シリアル操作ガイド

このメモは、STM32 側の `myshell` コマンドと、PC 側の Python GUI
`tools/myshell_gui.py` の使い方をまとめたものです。

## セットアップ

### Pixel録画中だけ使う撮影専用ターン

通常の`debug turn type exe`は従来どおり前センサへの手かざしを待ちます。
Pixel撮影と連動する`tools/pixel_camera_turn_capture.py --execute`と、既存debug GUIの「Pixel撮影・結果」タブは、録画ID・所有者・RECORDING状態を照合してから、
次の撮影専用経路で手かざしなしの単体1回を実行します。詳しくは[Pixelカメラ撮影](pixel_camera_README.md)を参照してください。

| コマンド | 動作 |
| --- | --- |
| `debug camera show` | 撮影専用プロトコルと許可の有効時間を照会する |
| `debug camera arm token` | 撮影中のPCが送る1〜99999999のトークンを、15秒間だけ一回分保存する。単独では動かない |
| `debug turn type exe_camera token` | 一致する有効な許可を消費し、電圧と回数1を確認して前センサ待ちなしで実行する |

許可の不一致・期限切れ・再利用は拒否します。通常の`exe`に入る場合も撮影用許可を消去します。
この撮影専用経路はカメラ側の録画状態を確認するPC入口と一緒に使い、動画を撮らない通常操作には使いません。
失敗や応答なしで実行コマンドを自動再送しません。
撮影タブは`tools/Start_Pixel_Tuning.cmd`から開けます。接続を保持してGUI値の反映・照合を行い、「撮影して1回走行」で記録と結果をまとめて保存します。
以前のCLIへGUIのRAM調整値を渡す場合は、`Apply`と`Show device values`で確認後に切断し、
撮影スクリプトの`--use-device-settings --expected-settings`で実機値を照合して引き継ぎます。
`Apply and execute`では撮影準備を確認しません。詳細は上記のPixel撮影手順を参照してください。

リポジトリのルートで Python 依存パッケージを入れます。

```powershell
pip install pyserial
```

GUI は Tkinter を使います。通常の Windows 版 Python には標準で含まれています。

## myshell を起動する

マウス側を shell モードにしてから、PC からシリアル接続します。

GUI の初期設定は以下です。

```text
Port: COM8
Baud: 460800
Line ending: CR
Char delay: 0.08 s
```

Windows で別の COM 番号が割り当てられている場合は、GUI の `Port` を変更してください。
今回の確認では `COM6` を使いました。

## GUI の起動

```powershell
python tools\myshell_gui.py
```

## HY0020 経由の BLE 接続

BLE の専用シェルは `tools/ble/mouse_ble_shell.py`、既存 GUI 用の接続は
`mouse_ble_gateway.py` と `mouse_gui_ble.py` を使います。
依存パッケージ、シェル移行、GUI での `BLE` ポートの選択は
[BLE 通信ツールの説明](ble/README.md) を参照してください。
マウス側のコマンドとバイナリ形式は有線接続と共通です。
仮想ポート `BLE` のBaud欄は物理USARTの通信速度を変更しません。STM32とHY0020側をともに460800 bpsへ合わせてください。

## GUI で maze_data をアップロードして Dijkstra を走らせる手順

1. `Port` に接続先の COM ポートを入れます。
   例: `COM6`

2. `Connect` を押します。
   接続できるとステータスが `Connected: ...` になります。

3. `Load maze_data` を押して、`tools/maze_data/*.txt` の迷路ファイルを選びます。
   読み込まれた迷路が右側のキャンバスに表示されます。迷路ファイル内の `G`
   セルからゴール位置も読み取り、右上に `Goal x,y size n` と表示します。

4. `Upload maze_data` を押します。
   GUI が `load maze_bin` を送信し、迷路データ 1024 bytes をマイコン側の RAM 上の
   `wall_data` に書き込みます。続けて `load goal x y size` を送信し、Dijkstra 用の
   ゴール位置も更新します。flash には保存しません。

5. `path dijkstra` を押します。
   直前にアップロードした迷路を使って `check_run_Dijkstra()` が実行され、左側の
   `Serial output` に経路が表示されます。さらに、右側の迷路キャンバス上に、シミュレータの
   走行軌跡に近い形で Dijkstra 経路が重ね描きされます。

6. 別の迷路で試す場合は、もう一度 `Load maze_data` -> `Upload maze_data` ->
   `path dijkstra` の順に操作します。

注意:

- `Upload maze_data` は RAM 上の `wall_data` とゴール設定だけを更新します。
- `load save` を押すと、flash に保存されている迷路を RAM に読み直します。
- マイコンを再起動すると、アップロードした RAM 上の迷路は消えます。
- アップロード後に `disp maze_bin` を押すと、現在の myshell 実装では flash 側の保存迷路を読み直して表示します。アップロード済み迷路の確認には `path dijkstra` を使うのが確実です。
- GUI の経路表示は、Dijkstra 出力の `x,y,d` から中心位置と壁位置を判定して描きます。

## GUI ボタン

| Button | 動作 |
| --- | --- |
| `Connect` | 選択したシリアルポートを開きます。 |
| `Send` | 入力欄のコマンドを送信します。 |
| `load save` | flash に保存されている迷路を RAM 上の `wall_data` に読み込みます。 |
| `disp maze` | 保存迷路を ASCII 表示します。 |
| `disp maze_bin` | 保存迷路をバイナリ受信し、キャンバスに描画します。 |
| `Save binary` | 最後に受信または読み込んだ迷路バイナリを `.bin` として保存します。 |
| `Save maze_data` | 最後に受信または読み込んだ迷路を `tools/maze_data/` 形式の `.txt` として保存します。 |
| `Load maze_data` | シミュレータ用の迷路 `.txt` を読み込み、キャンバスに描画します。 |
| `Upload maze_data` | 読み込んだ迷路とゴール位置をマイコン側 RAM に送ります。 |
| `disp history` | flash 側の壁履歴を表示します。 |
| `path dijkstra` | 現在 RAM 上にある迷路で Dijkstra の経路確認を実行し、迷路上にシミュレータ風の走行軌跡を表示します。 |
| `path queue` | 同じ条件で優先度付きキュー版の Dijkstra を実行し、送信完了から応答完了までの時間を表示します。 |
| `compare path time` | 通常版と優先度付きキュー版を連続実行し、シリアル往復時間と比率を表示します。 |
| `disp log` | ログをテキスト表示します。 |
| `disp log_bin -> CSV` | バイナリログを受信して `tools/logs/` に CSV 保存します。 |
| `end exe` | myshell を終了します。 |
| `Clear output` | 左側のシリアル出力欄を消します。 |
| `Save output` | 左側のシリアル出力欄をテキスト保存します。 |

## myshell コマンド

| Command | 説明 |
| --- | --- |
| `help` | コマンド一覧を表示します。 |
| `info sys` | システム名を表示します。 |
| `info ver` | ファームウェアのバージョン文字列を表示します。 |
| `irsens count [interval_ms]` | 指定回数、4方向のIRセンサ値と換算距離[mm]を表示し、最後に平均値を出力します。 |
| `ir_sensor count [interval_ms]` | `irsens` と同じコマンドです。 |
| `interface ir [count] [interval_ms]` | IR差分値と発光ON/OFF時のADC値を表示します（`photo` も同じ）。 |
| `load save` | flash に保存されている壁データを shell 側の `wall_data` に読み込みます。 |
| `load maze_bin` | GUI からバイナリ迷路を受け取り、shell 側の `wall_data` に読み込みます。flash には書きません。 |
| `load goal x y size` | Dijkstra 用のゴール左下座標 `(x, y)` と正方形サイズを RAM に設定します。 |
| `disp maze` | 保存迷路を読み込んで ASCII 表示します。 |
| `disp maze_bin` | 保存迷路を読み込んで、1 cell 1 byte のバイナリとして出力します。 |
| `disp history` | 保存迷路を読み込んで壁履歴を表示します。 |
| `disp log` | ログデータを CSV 風テキストで表示します。 |
| `disp log_bin` | ログデータを高速受信用のバイナリフレームで出力します。 |
| `path dijkstra` | 現在 RAM 上にある迷路とゴール設定で `check_run_Dijkstra()` を実行します。未ロードの場合は flash の保存迷路と既定ゴールを読みます。 |
| `path dijkstra_queue` | 同じ迷路・開始地点・ゴールで優先度付きキュー版を実行します。 |
| `search run start_x start_y N\|E\|S\|W goal\|full\|full_prune first\|second mask max_steps` | モーションを実行せず、実機C++の仮想壁・歩数マップ・足立法を連続再生します。`full_prune`はUNKNOWN開放仮定の枝刈り全面探索です。 |
| `explore show` | 実機探索の対応プロトコル・機体A/B・既定ゴールを返します。動きません。 |
| `explore start gx gy size plain\|acc outward\|roundtrip limit_seconds` | ログ配信と許可パケットが有効な場合、手かざし待ち後に実際の探索走行を開始します。専用GUIから操作します。 |
| `shortest show` | 最短走行の対応機体・プロファイル一覧・正常完了したBLE探索のRAM迷路の有無を返します。動きません。 |
| `shortest start gx gy size profile ram\|flash limit_seconds` | ログ配信・許可パケット・電圧・経路を確認し、手かざし後に最短走行します。BLE探索GUIの「最短走行」から操作します。 |
| `wall_ekf show` | 直進壁EKFのモード、最新の横偏差・姿勢・共分散・壁採用マスク・更新数を表示します。 |
| `wall_ekf legacy\|shadow\|control` | 停止中に直進・斜め壁制御の推定・制御モードを変更します。走行は開始しません。 |
| `search replay reset\|keep plain\|acc start_x start_y dir goal_x goal_y goal_size goal\|full\|full_prune first\|second mask max_steps` | `Search`と同じ壁観測・マップ更新順で、モーションだけをno-opにして逐次再生します。 |
| `log mode0` | ログヘッダ出力を有効にします。 |
| `log mode1` | ログヘッダ出力を無効にします。 |
| `end exe` | myshell を終了します。 |

## 直進壁EKF

通路内の左正の横偏差`e` [m]と姿勢偏差`psi` [rad]を持つ2状態EKFです。
`Task/Inc/straight_wall_ekf.h`、設定は`Params/wall_param_A.h` / `wall_param_B.h`の`wall_ekf_param`です。
各制御割り込みのエンコーダ移動量と、現在のジャイロ・制御の`delta_tms`で予測します。
壁切れで変化する`ego.length`の差分や、リセットされる`ego.radian`を積分入力に使いません。
現行制御と時間刻みをそろえており、物理周期との整合・処理時間は実機IRQログでの確認が必要です。

```text
wall_ekf show
wall_ekf shadow
wall_ekf control
wall_ekf legacy
```

| モード | 動作 |
| --- | --- |
| `legacy` | 従来のZirconia制御を使用し、EKFを停止します。 |
| `shadow` | 起動時の既定値。従来制御を保ち、EKFと候補の制御出力を並行計算します。 |
| `control` | EKFの横偏差・姿勢と信頼度を、壁の採用判定・直進側壁制御へ使用します。 |

モードはRAM保持で、電源投入時は`shadow`です。変更はモーション未実行かつ制御無効時のみ許可し、
走行中は`WALL_EKF_ERROR busy`を返します。コマンドは推定器を初期化し、モータを始動しません。
BLE探索GUIでは「壁制御」から選択でき、走行開始前の設定・読み返しと、終了後の状態保存を自動で行います。
手入力での切替は不要です。比較手順と記録項目は[GUIから壁制御を比較する](ble/README.md#guiから壁制御を比較する)を参照してください。

| 観測の状態 | EKF・制御の動作 |
| --- | --- |
| 両壁・片壁 | 既存の左右別品質判定を通し、更新前の予測に対する残差/NISで採用を判定します。左右とも採用した場合は距離差、片壁の場合は45 mm基準で横偏差を更新します。 |
| 壁なし・不採用 | 横偏差・姿勢を予測し、共分散を増やします。観測していない通路中心へ横位置を引き戻しません。姿勢が十分確かで最終観測から90 mm以内なら低い出力上限で姿勢を保持します。 |
| 壁切れ | 影響側の観測を直ちに外し、8 mmの再確認距離を設けます。反対側の壁観測は継続できます。 |
| 壁の再出現 | 既存の距離・信号安定判定とEKF残差判定を経て、制御の確認・立上げを再開します。 |

10点ADC移動平均を毎周期独立観測として過信しないよう、EKFの壁更新間隔は10 ms以上です。
横偏差の標準偏差5 mm以下・絶対値30 mm以下で位置を制御に採用し、
姿勢の標準偏差3°以下・絶対値15°以下に加え、独立した区間ジャイロ姿勢との差3°以内を要求します。
満たさない場合や壁観測が不採用の間は、並行計算している従来制御へ戻します。
位置観測の採用と姿勢制御の採用を分け、採用した距離による横位置更新は継続します。
EKF側の出力を保留する間はその制御器を初期化し、古い飽和旋回指令を持ち越しません。
壁が全く見えず姿勢も信頼できない場合は、古い横位置に向けた補正を出しません。
新しい壁の確認中は、従来の「近い壁から離れる向きだけ」の小さい補正を維持します。
これらの雑音・信頼度設定は機体別構造体で保持する試作値で、A/Bの実機測定による校正済み値ではありません。

連続する直進区間と探索ターン前直進では、EKF・生ジャイロの通路姿勢・壁制御phaseを維持します。
停止・無効化・旋回/斜めへの切替・別通路開始では初期化します。
探索ターン出口では、旋回開始時の姿勢＋独立した旋回ジャイロ積分－論理ターン角を、
新通路の姿勢へ引き継ぎます。非数値や15°超の残留角はこの引継ぎへ採用しません。
姿勢分散も入口の値＋旋回時間×既存のジャイロ雑音分散率で伝播します。
採用したEKF姿勢にはその分散、それ以外は独立した区間ジャイロ積分の分散を対応させます。
不明・非数値・0以下の分散は標準偏差8°の初期値へ戻し、確かな入口姿勢まで一律に8°へ戻す処理を避けます。
旋回中のADC平均を捨て、直進の新しい10点を待つ間もジャイロ姿勢・分散を更新します。
引継ぎ周期の雑音を二重加算せず、横位置分散と横位置/姿勢の相関は新しい通路で取り直します。
3°の姿勢整合条件、壁観測の確認、従来制御へ戻す条件は維持します。
横位置のゼロリセットを中央の実測とは扱わず、従来の初期共分散から新しい側壁観測で更新します。
`control`だけ、安定した側壁EKFと前壁の変化履歴を使い、未確認な小さい前壁左右差を除外します。
大偏差・接近・継続する候補・観測や推定の信頼不足では従来どおり補正します。
具体的な条件は[探索ターン後の復帰と補正判定](../README.md#探索ターン後の復帰と補正判定)を参照してください。
既存の左右壁端から得る横位置補正は、`control`では分散付き観測として一度だけ更新し、
現在の移動量・角速度で予測した後に取り込みます。同時に`x_point`へ値を直接上書きしません。
壁切れの進行距離補正は既存の`ego.length`処理を維持します。
壁端が示す位置・左右対応・機体別校正は従来の判定を使うため、誤対応を完全には排除できません。
姿勢を0にする既存の区間リセットは、姿勢を実測した観測とは扱いません。

`WALL_EKF`行の`active`は現在の推定有効状態です。停止後も`e_mm`、`psi_rad`、
`var_e_mm2`、`var_psi_rad2`、`updates`（壁更新数）、`corners`（壁端補正数）には最新値を保持します。
`candidate`は品質・壁端除外後、`accepted`は更新前のEKF残差判定後の左右マスク（1=左、2=右）です。
`nis_l/r`は各側の正規化残差²です。`show`はIRQを短時間マスクして固定サイズの状態をコピーし、
表示はマスク解除後に行います。割り込み内で通信・待機・動的メモリ確保は追加していません。

CSVの列数・バイナリ形式は維持します。`control_ir`は使用した横偏差[mm]、`control_ir_dot`は
使用した姿勢[rad]で、両方左正です。`control`では採用可能なEKF出力、`legacy/shadow`では従来値です。
全EKF状態の逐次CSV配信は追加しておらず、詳細状態は`show`またはデバッガで確認します。
`straight_wall_estimate`、`straight_wall_candidate_mask`、`straight_wall_ekf_control_output`も参照できます。

### 斜め区間への適用

`wall_ekf`は斜め直線にも適用します。`legacy/shadow`の斜め制御とCSVの意味は従来どおりです。
`control`は`Task/Inc/diagonal_wall_control.h`の32 mm接近モデルを使い、採用可能なEKF偏差だけを従来の斜め制御則の横偏差入力へ渡します。
未確認・不採用時は従来のIR/オドメトリ入力、速度依存ゲイン、軌道制動を維持します。柱がない区間で従来の補正を0へ置き換えません。
片側前センサが近い場合は柱候補を残し、両前センサ66 mm以下の場合は近接観測を除外します。
柱のない区間を中心位置0の観測にせず、断続的な柱から姿勢の確信度を上げません。
新しい斜め軸で姿勢平均0・広い初期分散から始め、ジャイロで相対姿勢を予測します。
旋回中の制御は既存のターンPID/FFです。今回のEKF接続はその前後を含む斜め直線区間が対象です。
相対姿勢は診断用で、絶対的な斜め方向の基準として補正に使いません。
斜めの`control_ir`は制御に使った左正の横偏差入力[mm]（EKF不採用時は従来入力）、`control_ir_dot`は斜め開始からの相対姿勢[rad]です。
直進と異なり絶対的な通路中心からの距離とみなさないでください。斜めの`legacy/shadow`の`control_ir_dot`とは単位・意味が異なります。
`WALL_EKF`行の`e_mm/psi_rad/...`は直進側の状態です。斜めの詳細状態は`diagonal_wall_output`をデバッガで確認します。
BLE最短走行の終了時には`SHORTEST_WALL_STATS`で斜め対象周期・EKF適用周期・更新数を出します。
併用方式は`diagonal_controller:2`で識別し、従来入力周期・両前センサによる除外周期もGUIとJSONへ保存します。
操作とJSONの読み方は[斜めEKF比較](ble/README.md#斜め区間のekfを比較する)を参照してください。

ホストでは機体A/Bの実装・校正を通して検証します。書込み・走行・制御周期内の余裕は別途の実機検証です。

## IRセンサ表示

マウスをshellモードにして、GUIのコマンド入力欄またはシリアルターミナルから送信します。
100回表示する場合（行間の待機は既定で10 ms）:

```text
irsens 100
```

20回表示し、行間に100 ms待機する場合:

```text
irsens 20 100
```

`ir_sensor 20 100` でも同じ動作になります。回数は1〜1000、待機時間は0〜1000 msです。
待機時間にはシリアル送信の時間が別途加わります。0を指定すると、同じ最新標本を繰り返し表示する場合があります。

`IRSENS_START` に続けて、指定回数の `IRSENS index:...` 行、平均値の `IRSENS_AVG`、`IRSENS_END` を出力します。
各行の `value_fl/fr/sl/sr` は発光ON−OFFのADC差分、`distance_fl/fr/sl/sr_mm` は換算距離です。
平均値は表示した全標本について小数点以下3桁で出力します。距離は各標本の換算距離を平均します。
FLは前左、FRは前右、SLは左、SRは右です。連続取得中の値を読み取ります。

## Search診断

`search run` はモータを動かさず、完成済みのRAM迷路を使って次の実装をマイコン上で実行します。

```text
virtual_wall_class → make_map → adachi::select_next_direction
```

例:

```text
search run 0 0 N goal first 1 256
search run 7 8 E full first 1 256
search run 7 8 E full_prune first 1 256
```

各ステップで次を `SEARCH_STEP` として出力します。

- 現在座標・姿勢
- 現在セルと4近傍の歩数値
- 観測壁4辺
- 仮想壁4辺
- 選択された次座標・方向
- 次座標用の仮想壁更新が、選択済み辺を閉じたか
- 迷路全体の仮想壁辺数

終了後、`SEARCH_MAP` で32行の歩数マップ、`SEARCH_VIRTUAL` で仮想壁一覧を出力します。

`search run` は完成迷路の静的診断であり、`Search::run_goal_search()`の壁観測手順そのものではありません。
実探索と同じ逐次壁観測を検証する場合は、次の`search replay`を使用します。

### Search逐次再生

`load maze_bin`で読み込んだ迷路をセンサの真値として保持し、探索用の壁は`reset`時に
`init_maze()`から開始します。到着セルごとに真値から4辺を観測し、次の順番で進めます。

```text
壁観測 → 方向選択 → Init_Motion相当 → Search_UpdateMap → execute_Motion(no-op) → 到着
```

`Search_UpdateMap`は実走行の`Search::update_map()`と共通です。`keep`を指定すると、直前の往路で
観測した壁を復路へ引き継ぎます。各`REPLAY_STEP`には選択辺の真値、仮想壁更新後の閉鎖状態、
観測壁・仮想壁・歩数値を出力します。

仮想壁のprotected領域は探索レッグのstart/goalではなく、迷路固有のstart/goalです。
`reset`時の開始座標と`load goal`のゴール領域を保持するため、復路でも元の2×2ゴール領域全体を
保護し、歩数マップの復路目的地だけをスタートへ切り替えます。

仮想壁更新には移動前の現在座標、歩数マップ更新には選択した移動先を別々に渡します。
このため、到着セルに接する仮想壁を到着前に誤って解除しません。

Search専用GUIは次のコマンドで起動します。

```powershell
python tools\myshell_search_gui.py
```

GUIはCOM接続を保持し、迷路送信、静的Search、往路再生、往復再生、32シナリオ検証を
実行できます。Shell log、ステップ表、実壁・仮想壁・経路の迷路表示、シナリオ一覧を
切り替えて確認できます。迷路タブではマウス位置を黒丸で表示し、結果の座標列を
再生・一時停止・前後1ステップ・速度変更できます。スペースキーでも一時停止・再開できます。
診断結果の受信後は「迷路 / 経路」タブへ自動的に切り替わります。

全組み合わせを実行する例:

```powershell
python tools\myshell_search.py --port COM8 `
  --maze tools\maze_data\logs_maze.bin `
  --goal 7,7,2 --return-goal 0,0,1 --start 0,0,N `
  --replay-matrix
```

このマトリクスは次の32シナリオ、合計64レッグを実行します。

- 往路: `plain/acc × first/second`の4種類
- 復路: `goal/full × plain/acc × first/second`の8種類
- 各組み合わせで往路を`reset`し、その復路だけを`keep`で実行

シナリオごとに生ログとステップCSVを保存し、全体結果を`*_summary.csv`へ保存します。

PCから迷路をアップロードして自動検証する場合は次を実行します。

```powershell
python tools\myshell_search.py `
  --port COM8 `
  --maze tools\maze_data\logs_maze.bin `
  --goal 7,7,2 `
  --start 0,0,N
```

探索経路を迷路上で再生する場合は`--animate`を追加します。マウスは黒丸、走行済み軌跡は水色、
ゴールは黄色、仮想壁は赤破線で表示されます。

```powershell
python tools\myshell_search.py `
  --port COM8 `
  --maze tools\maze_data\logs_maze.bin `
  --goal 7,7,2 --start 0,0,N `
  --animate --animation-interval 100
```

全面探索の復路確認例:

```powershell
python tools\myshell_search.py `
  --port COM8 `
  --maze tools\maze_data\logs_maze.bin `
  --goal 0,0,1 `
  --start 7,8,E `
  --mode full
```

入力は1024-byte `.bin`、壁履歴テキスト、標準 `maze_data` テキストに対応します。
拡張子が `.txt` でも内容が1024-byteバイナリなら、そのまま迷路payloadとして扱います。

結果は `tools/logs/` に次の形式で保存します。

- 生の `SEARCH_*` 応答テキスト
- ステップCSV
- 歩数マップCSV
- 仮想壁CSV
- 歩数マップ・経路・仮想壁を重ねたPNG

`verification=FAILED` になる主な条件は次のとおりです。

- ゴールへ到達できない
- 選択候補がない
- 最小歩数でない方向を選択した
- 選択済みの辺を直後の仮想壁更新が閉じた
- スタート・ゴールに仮想壁が接している
- 仮想壁数または歩数マップ行数が出力と一致しない

## 迷路バイナリ形式

`disp maze_bin` は以下の形式で出力します。

```text
MAZE_BIN_START
<width * height raw bytes>
MAZE_BIN_END
```

標準の迷路サイズは 32 x 32 なので、payload は 1024 bytes です。

各 byte は 1 cell の壁情報です。

```text
bit 0-1: north
bit 2-3: east
bit 4-5: south
bit 6-7: west
```

壁状態は以下です。

```text
0: NOWALL
1: WALL
2: UNKNOWN
3: VWALL
```

payload の順序は、上段 (`y = height - 1`) から下段へ向かう row-major です。

`load maze_bin` は同じ 1024 bytes の payload を逆方向に送ります。
これは RAM 上の `wall_data` だけを更新し、flash の保存迷路は上書きしません。

```text
load maze_bin
MAZE_BIN_READY 1024
<width * height raw bytes>
MAZE_BIN_LOAD_DONE
```

ゴール位置は別コマンドで送ります。

```text
load goal 7 7 2
GOAL_SET_DONE x:7 y:7 size:2
```

`path dijkstra` 実行時には、使われるゴール設定が先に表示されます。

```text
DIJKSTRA_GOAL x:7 y:7 size:2
DIJKSTRA_START
...
DIJKSTRA_END
```

## Dijkstra 経路表示

GUI の迷路表示では、Dijkstra のノード種別をそのまま座標に変換して経路を描きます。

| motion | 表示上の移動 |
| --- | --- |
| `Straight` | 区画中心から区画中心 |
| `Diagonal` | 壁のない壁位置から壁のない壁位置 |
| `Turn_in_*` | 区画中心から壁のない壁位置 |
| `Turn_out_*` | 壁のない壁位置から区画中心 |
| `Long_turn*` | 区画中心から区画中心 |
| `Turn_RV90`, `Turn_LV90` | 壁のない壁位置から壁のない壁位置 |

## ログバイナリ形式

`disp log_bin` は ASCII のメタ情報を出した後、バイナリフレームを出力します。

保存ログは最新400サンプルのリングです。満杯になると最古のサンプルを上書きします。
`disp log` / `disp log_bin` は有効なサンプルだけを古い順に出力します。
保存周期は `LOG_DATA_PRIOD` ms（既定1 ms）で、CSVの `cnt` は出力の先頭を0とする経過時間[ms]です。
表示中は保存を一時停止して整合を保ち、表示後に元の保存状態へ戻します。
走行開始時の `reset_log()` は読み書き位置と有効件数を初期化し、走行停止時は保存内容を残します。
以前の保存領域400件に対して1000件を出力していた範囲外アクセスも解消しています。

```text
START
HEADER,cnt,...
BINARY
<binary log_frame_t frames>
<LOG_MAGIC_END frame>
END
```

通常フレームは以下です。

```c
uint16_t magic;       // 0xA55A
uint16_t index;
half_float data[55];
```

終了フレームは `magic = 0xFFFF` です。

末尾4列は `irq.total_us,irq.logging_us,irq.max_us,irq.period_us`（いずれもµs）です。
従来の51列の並びは維持し、新しい通常フレームは114 byteです。
更新後のメイン・debug・turnpattern GUIは `log mode0` を指定し、HEADERの列数から新55項目・旧51項目のフレーム長を選びます。
`Serial_binary.py` もHEADERから選択するため、取得前に `log mode0` にしてください。
計測範囲と前回値の扱いは [割り込み時間のログ](ble/README.md#割り込み時間をログで確認する) を参照してください。

## トラブルシュート

- GUI が timeout する場合は、`Timeout` を長めにするか、マウス側 myshell をリセットしてから再接続してください。
- コマンド文字が欠けて echo される場合は、`Char delay` を大きくしてください。
- `Upload maze_data` が途中で止まる場合は、マイコン側が `load maze_bin` の受信待ちになっていることがあります。その場合は一度リセットして myshell に入り直してください。

## 既存のコマンドラインツール

古いコマンドラインツールも残しています。

```powershell
python tools\Serial_maze_view.py --port COM8 --baud 460800 --binary
python tools\Serial_binary.py
```

手動確認は GUI、スクリプト化したい受信処理はコマンドラインツール、という使い分けが便利です。
