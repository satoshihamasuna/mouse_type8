# myshell シリアル操作ガイド

このメモは、STM32 側の `myshell` コマンドと、PC 側の Python GUI
`tools/myshell_gui.py` の使い方をまとめたものです。

## セットアップ

### 配置後のGUIボタン開始と本体ログ周期

`tools/Start_Turn_Tuning.cmd`で既存GUIを起動します。録画OFFではPixelなしでBLE接続し、
「配置完了・1回走行開始」から単体1回を実行できます。[操作と保存先](turn_tuning_README.md)を参照してください。

| コマンド | 動作 |
| --- | --- |
| `debug gui show` | GUI専用実行・最大周期・保存済みログの実際の周期を照会 |
| `debug gui period N` | 次のGUI／撮影ターンの本体ログ周期を1～100 msで指定 |
| `debug gui arm TOKEN` | ボタン開始用の使い切り許可を15秒間発行 |
| `debug turn TYPE exe_gui TOKEN` | 手かざしなしの単体1回。完了は`DEBUG_GUI_RUN_DONE` |
| `debug search_turn right\|left exe_gui TOKEN` | 探索ターンの同じボタン開始経路 |

GUI用と撮影用の許可は別です。周期は実行開始時に消費し、次回の既定を1 msへ戻します。
本体ログ400件は設定周期のまま読み出せ、`cnt`は経過msです。周期の受付だけでは走行しません。

### Pixel録画中だけ使う撮影専用ターン

通常の`debug turn type exe`は従来どおり前センサへの手かざしを待ちます。
Pixel撮影と連動する`tools/pixel_camera_turn_capture.py --execute`と、既存debug GUIの「ターン調整・ログ・録画」タブの録画ONは、録画ID・所有者・RECORDING状態を照合してから、
次の撮影専用経路で手かざしなしの単体1回を実行します。詳しくは[Pixelカメラ撮影](pixel_camera_README.md)を参照してください。

| コマンド | 動作 |
| --- | --- |
| `debug camera show` | 撮影専用プロトコルと許可の有効時間を照会する |
| `debug camera arm token` | 撮影中のPCが送る1〜99999999のトークンを、15秒間だけ一回分保存する。単独では動かない |
| `debug turn type exe_camera token` | 一致する有効な許可を消費し、電圧と回数1を確認して前センサ待ちなしで実行する |
| `debug search_turn right\|left exe_camera token` | 探索ターンを同じ一回許可・電圧・回数1の条件で実行する。前後45 mmの加減速を含む |

許可の不一致・期限切れ・再利用は拒否します。通常の`exe`に入る場合も撮影用許可を消去します。
探索ターンの撮影対応は`DEBUG_CAMERA_CAP protocol:1 ttl_ms:15000 search_turn:1`で識別します。GUI・CLIは未対応の機体で探索ターンの撮影を開始しません。
`search_turn_log_hold:1`も返す版では、`search_turn ... exe_camera`の本体1 ms保存リングを探索ターン直後に確定します。後続45 mm直進に含まれる400 ms制動で旋回区間が上書きされることを防ぎ、`disp log_bin`で前走・ターン側の最新400件を取り出せます。連続配信CSVは停止まで継続します。通常の`exe`の保存タイミングは従来どおりです。
`turn_log_hold:1`に対応する版では、14種類の`turn`もLendまで終えた直後にログを確定します。通常の`debug turn type exe`と撮影用`exe_camera`の両方が対象です。後段の減速・制動を含めず、`Get log CSV`でターン側の最新400件（1 ms間隔）を取得できます。回数を増やしても保存上限は400件です。
撮影GUI・CLIは、`saved_log_period_ms:1 saved_log_capacity:400`も確認できた場合に、`DEBUG_CAMERA_RUN_DONE`の後で`saved_1ms.csv`と同名設定JSONを自動保存します。動画同期用の5 ms連続CSVとは別ファイルです。
この撮影専用経路はカメラ側の録画状態を確認するPC入口と一緒に使い、動画を撮らない通常操作には使いません。
失敗や応答なしで実行コマンドを自動再送しません。
撮影タブは`tools/Start_Pixel_Tuning.cmd`から開けます。接続を保持してGUI値の反映・照合を行い、「配置完了・1回走行開始」で記録と結果をまとめて保存します。
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

探索ターンは`debug search_turn right|left preset 速度`で機体別の初期値を読み込みます。
対応速度はAが280 / 300 / 320 / 350 / 370 / 400 mm/s、Bが280 / 300 / 320 / 350 / 370 mm/sです。
例: `debug search_turn right preset 350`、`debug search_turn left preset 370`。プリセット読込みでは走行しません。
Bの350はBの320から速度のみ変更した未調整の初期値です。370は2026-11-21のB左右各3本の入口相対単体候補を反映しています。条件と値は[採用根拠](turn_analysis/20261121_B_search370_adoption/README.md)を参照してください。A/Bとも通常探索の直進・ターンの既定速度は320 mm/sです。時間経過時の低速設定も320 mm/sです。

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
| `turn_ekf show` | ターン局所自己位置EKFのモード、適用可否・理由、位置・滑り角・標準偏差・更新数を表示します。 |
| `pose show` | 動作ローカル座標、全体座標、変換原点、座標系番号と有効性を表示します。走行中も参照できます。 |
| `pose reset [x_mm y_mm yaw_deg]` | 停止中に全体の開始位置・方向を設定します。省略時は(0,0,0)。次の走行にも使い、Flashには保存しません。 |
| `wall_edge show` | 壁切れ縦位置補正の試験モード、アーム条件、探索基準位置と適用範囲を表示します。 |
| `wall_detect show` | 停止中に最新の壁切れ検出を表示します。単体ターン・探索ターン・turnpatternとBLE探索/最短で自動記録。`phase:before/after`の区間集計は未検出も残し、ターン前の`expected_side`（曲がる側）と`expected_seen`で確認できます。詳細は[検出ログ](ble/README.md#ターン前後の壁切れ検出wall_detect-schema-1)を参照してください。 |
| `wall_edge legacy\|shadow\|control` | 停止中に壁切れ試験を切り替えます。起動時legacy。Shadowは縦位置へ適用せず、Controlは90 mm探索直進と直進ターン入口だけ適用します。 |
| `wall_edge legacy` / `shadow` / `control` の柱候補 | 全モードで直進独立柱と斜め極小を検出します。LegacyはA/Bの柱極値ペアを横位置補正に採用し、Aの90 mm探索直進・最短の直進入口では縦位置も最大半量・±4 mmで直接補正します。両control時はAの独立柱をEKF観測にします。`pillar_active`は検出、`pillar_pair_apply`はLegacyの横位置採用、`pillar_apply/pillar_entry_apply`は縦位置の適用設定です。Bと斜め極小の未校正の縦位置は直接補正しません。 |
| `wall_ekf legacy\|shadow\|control` | 停止中に直進・斜め壁制御の推定・制御モードを変更します。走行は開始しません。 |
| `search replay reset\|keep plain\|acc start_x start_y dir goal_x goal_y goal_size goal\|full\|full_prune first\|second mask max_steps` | `Search`と同じ壁観測・マップ更新順で、モーションだけをno-opにして逐次再生します。 |
| `log mode0` | ログヘッダ出力を有効にします。 |
| `log mode1` | ログヘッダ出力を無効にします。 |
| `end exe` | myshell を終了します。 |

壁切れ・柱の有効検出は右LED6／左LED1を約50 ms点灯します。追加柱検出の品質棄却候補は点灯を開始せず、Shadowでも有効候補は表示します。点灯は補正適用やアーム状態の表示ではありません。

直進の`shadow/control`は、壁切れと独立柱で共通のOpen履歴判定を使います。
78 mm以上を12 mm確認した後の短い接近は、62 mm以下が12 mm続いても柱候補を保持し、65 mm交差を壁切れとして出しません。
Openを離れた観測区間は最大32 mmで期限切れにし、連続壁の後まで古いOpenを持ち越さないようにします。
再びOpenを12 mm確認すると次の観測区間へ移り、停止・無効値・後退・IRモード変更で履歴を破棄します。同じIRモードの直進→ターン入口では履歴を保持します。
設定は`Params/wall_param_A.h` / `wall_param_B.h`の`straight_feature_history_param`です。
32 mmは2026-11-17の機体A探索で見つかった約26 mm幅の近い柱からの試験値で、機体Bの実機確認は未実施です。
起動時は`legacy`です。Legacyでも追加柱を検出・採用し、Shadow記録・品質判定・LEDの意味は維持します。実機の補正精度・閉ループ動作は録画付き評価が未実施です。

## ローカル・全体座標

```text
pose show
pose reset
pose reset 45 45 90
```

座標は`vehicle.h`の`Vehicle::pose`に機体ごとの状態として保持します。シェルは制御タスクが使う`Vehicle`を参照します。
全体座標は既定で走行開始位置が原点、初期進行方向が+X、左が+Yです。位置はmm、表示姿勢はradです。
`reset`の3値だけは`X [mm] Y [mm] 開始方向 [deg]`で、北向きの90°は+Yを向く意味です。実際の開始位置に合わせて指定してください。
設定は次の`Motion_start()`にも使います。`Motion_end()`とエラー停止では最後の位置を残し、無効化後のセンサ値で書き換えません。

| `POSE`項目 | 意味 |
| --- | --- |
| `world_x_mm/world_y_mm/world_yaw_rad` | 全体の位置・姿勢 |
| `local_s_mm/local_e_mm/local_yaw_rad` | 現在の動作開始点・経路方向から見た前方位置・左位置・姿勢偏差 |
| `origin_x_mm/origin_y_mm/origin_yaw_rad` | ローカル座標を全体座標へ変換する固定原点と軸方向 |
| `reference_yaw_rad` | 論理ターン角を積算した経路方向。計測姿勢とは独立 |
| `motion/frame/phase` | モーション番号・座標系番号・制御phase（0=NOP、1=直進、2=斜め、3=旋回、4=その場旋回、5=制動） |
| `active/valid` | 走行全体の追跡中か、欠測・異常増分なく更新できているか。停止しても`valid`と座標は保持 |
| `samples/corrections/rejected` | センサ予測・採用状態の座標反映・異常増分の拒否回数。観測件数とは異なる |

ローカル位置は動作・phaseの境界で原点を移しますが、全体位置は保持します。残留姿勢はゼロにしません。
`world位置 = origin位置 + R(origin姿勢) × local位置`、`world姿勢 = origin姿勢 + local姿勢`です（姿勢は±πへ正規化）。
既存の`ego`や各EKFの固有座標を読む場合は、これらと原点・軸が一致するとは限りません。
直進・斜めの採用補正とターンEKF増分を幾何変換で反映し、全体側に独立したEKFは追加していません。共分散は各推定器で保持します。
不正サンプルを一度でも拒否すると`valid:0`を保持します。`reset`は制御中・モーション実行中に拒否します。走行は開始しません。
周期CSV・バイナリの列は増やしていないため、この座標は`pose show`またはC++の`vehicle->pose.snapshot()`で参照します。
旧ログの`ego.turn_x/y`列は`vehicle->pose.log_position()`の左/前成分です。独立した位置積分は持たず、ログ用原点だけを従来のゼロ化箇所で更新します。
ログ用原点の変更は`pose show`の全体位置・動作ローカル位置を変えません。`ego.turn_x/y`はエンコーダと採用した推定補正を反映する値になり、従来の速度積分値と数値が変わる場合があります。
旧`ideal.turn_x/y`列は`estimate.heading_rad` / `estimate.sigma_heading_rad`へ置き換えました。
`wall_ekf control`時の引継ぎに採用する姿勢・標準偏差をradで記録し、無効時は両方NaNです。斜めは`wall_edge control`も必要です。
旋回中は入口軸、出口では論理角を引いた新しい経路軸での姿勢になります。全体のyawや旧`ego.radian`とは原点が異なります。
55項目・転送サイズを保ち、識別子を`L8S5` / `0xA55D`へ更新しています。[参照元と無効条件](ble/README.md#採用姿勢の2項目)も確認してください。

## ターン自己位置EKF

`turn_ekf`は旋回中に`Vehicle::pose`へ渡す位置増分と滑り角の推定を選びます。
起動時は`control`です。停止中に次のコマンドを使えます。

```text
turn_ekf show
turn_ekf legacy
turn_ekf shadow
turn_ekf control
turn_ekf reset
```

`legacy`は従来の位置積分、`shadow`は並行計算、`control`は適用条件を満たす増分だけを自己位置へ反映します。
`show`は走行中も参照でき、変更・リセットは停止中に限定します。`reset`は静止校正も取り直します。
Flashへは保存しません。`control`は自己位置の選択であり、新しい操舵指令を出すモードではありません。

| 機体・条件 | 自己位置への採用 |
| --- | --- |
| A・非吸引・500mm/s・右90° | 品質条件を満たす間だけ採用 |
| A・吸引・1600mm/s・PWM duty600～700・右90°/右180° | 品質条件を満たす間だけ採用。校正時はduty650 |
| A・非吸引・右180° | 比較で悪化したため従来積分 |
| 左旋回、斜め進入/退出、探索ターン、別速度、機体B | 未検証・未校正のため従来積分 |

状態は`[前方位置,左位置,姿勢,滑り角,ジャイロ零点,横加速度零点]`です。
`Params/turn_pose_param.h`の吸引別係数を使い、未来のIMUを参照せず予測・更新します。
横加速度は20点平均の実測値を20ms以上離して観測し、モデル不一致を雑音へ含めています。
車輪停止・指令停止中の100ms窓で50ms以上の有効サンプルを得て、ジャイロ/横加速度の静止零点を確認します。
吸引の立上がり中でPWMが安定しない窓、姿勢変化が大きい窓は採用しません。校正は吸引別で保持し、30秒で期限切れになります。

共分散、NIS、観測の鮮度、区間ジャイロ角との整合、位置差と1回の補正量を確認し、
条件を外れた周期は従来の増分を使います。実測周期が不明、前回IRQの処理時間が周期の85%以上、
非有限値、ターン途中の吸引切替はその旋回のEKF適用を中止します。
方法を切り替える際に絶対座標を置き換えず、位置増分を選ぶため座標を突然飛ばしません。
壁制御の`x_point`、`ego.radian`、速度/距離、PID・FFは既存の座標・入出力を保ちます。
`wall_ekf control`では入口通路の姿勢・分散をターンEKFへ渡し、最終周期も`turn_ekf control`で採用可能なら、出口の姿勢・分散へ置き換えます。最終周期が不採用、またはターンEKFがLegacy/Shadow・校正範囲外なら、入口姿勢＋実測ジャイロ旋回量−論理角と伝播分散を使います。
両者は同じジャイロを含むため、独立観測として二重に融合しません。入口姿勢なし・非有限値・残留角15°超では引継ぎを行いません。
旋回中の未校正IR距離を壁観測として加えることはしません。

`TURN_EKF`の`active`は旋回中、`applied`は直前周期での適用、`eligible`は機体/速度/ターンの対象判定です。
`reason`は`applied / shadow / legacy / uncalibrated / unsupported / bias_not_ready / invalid /
profile_changed / innovation / covariance / disagreement / inactive`を表示します。
`applied_ticks`は直近のEKF区間で採用した周期数、`updates/rejected`は横加速度観測の採否数です。
`x_left_mm/y_forward_mm`は並行EKFの区間位置、`yaw_rad/beta_rad`と各標準偏差はradです。姿勢を引き継いだ区間のEKF座標と`yaw_rad`は入口通路基準で、採用する位置・姿勢増分を全体座標へ変換して反映します。
ログの`ego.turn_x/y`は`pose`をログ用原点へ変換した左/前のmmで、適用中の`ego.turn_slip_theta`はEKFのradになります。`turn_x/y`、`turn_x/y_dash`、`turn_slip_dot`の保存変数はありません。
CSVの列・順序・バイナリ形式は変更していません。

保存ログでの適用判断は[因果的C++再生結果](logs/turn_pose_firmware_A_20261118_v4/README.md)を参照してください。
実機への書込み・変更後の走行・割込み処理時間の測定は未実施です。

## 直進壁EKF

通路内の左正の横偏差`e` [m]、姿勢偏差`psi` [rad]、encoder進行距離に対する誤差`ds` [m]を持つ3状態EKFです。
`Task/Inc/straight_wall_ekf.h`、設定は`Params/wall_param_A.h` / `wall_param_B.h`の`wall_ekf_param`です。
各制御割り込みのエンコーダ移動量と、現在のジャイロ・制御の`delta_tms`で予測します。
壁切れで変化する`ego.length`の差分や、リセットされる`ego.radian`を積分入力に使いません。
現行制御と時間刻みをそろえており、物理周期との整合・処理時間は実機IRQログでの確認が必要です。

最短走行の姿勢引継ぎは90/180°だけでなく、45/135°進入→斜め→V90/Long V90→斜め→45/135°退出にも対応します。
`wall_ekf control`を選び、斜め側では`wall_edge control`も選びます。起動時のShadowは従来どおりです。
出口の姿勢・分散は斜め柱EKFの初期値と接近観測EKFに渡し、横位置や壁・柱の観測済みフラグは新しく確認します。
柱が得られない短い斜めは、入口からのジャイロ姿勢だけを220制御周期未満かつ斜め1.5区画未満で次のターンへ渡せます。角度±8°・既存の姿勢分散上限も満たす必要があります。
未校正のBでもこの伝播は使えますが、未校正の柱観測やターンモデルを姿勢の根拠にしません。停止・無効化・モード切替・不正サンプルで引継ぎを破棄します。
各EKFの位置・滑り・バイアスを単一状態へ統合する処理ではなく、共通の姿勢とその周辺分散を区間境界で渡します。既存の斜め同士の位置・共分散変換は継続します。

機体Aでは`wall_ekf legacy/shadow/control`が`wall_edge`も同じモードへ切り替えます。
録画付きGUIの壁制御を「EKF制御（control）」にすれば、追加の手動コマンドなしでAの探索特徴点補正も有効になります。
`wall_ekf show`の`states:3 feature_apply:1`、`wall_edge show`の`pillar_apply:1`で設定を照合します。
起動時は従来どおりEKF Shadow・壁切れLegacyなので、走行前にモードを選択してください。走行中の変更は禁止です。

縦位置の実適用はAの90 mm探索直進、最短の直進ターン入口、ターン出口から連続する直進です。探索のWall→Openは49.5 mm、Open→Openは39.5 mmに確認遅れを加えます。
最短の`Prev_Turn`は経路のセル中心原点で、Aの特徴点EKFでは探索のセル入口線基準から45 mm移し、壁切れ`49.5−45=4.5 mm`、独立柱`39.5−45=−5.5 mm`を試験基準にします。Legacyの`STRAIGHT_CORRECTION=7 mm`は変更しません。
入口の対象は90/180度・45/135度進入ターンです。出口の対象は90/180度・45/135度退出ターンの直進側です。
出口内は壁切れ`Lend+fix_post_run+4.5 mm`、柱極小はそこから10 mm手前を試験基準とし、直後の`Straight`では`4.5+90n mm` / `−5.5+90n mm`へ移します。
柱の極小後に別の特徴点のEKF補正が入るときは、保留中の観測座標だけを同量補正し、地図参照と消費済み境界は保持します。`minimum_rebase_mm`には原点移動と既知のEKF補正が含まれます。未知の1 mm超の座標ジャンプは棄却します。
交差・極小の位置から区画を選ぶため、柱の確定が遅れても次区画と混同しません。原点変更とモーション世代変更をまたいで採用済み区画を保持し、左右・壁切れ/柱の重複を抑止します。
次の直進を設定するまでの`Run_Pause`でも、連続性が確認できる場合だけEKFの予測・観測を1 IRQに一度進めます。
従来の出口補正とは別の`straight_run`モデル（phase=5、窓±8 mm、最大±4 mm・半量）を使います。旋回・斜め入口/出口・出口との連続性が不明な直進には適用しません。
直進完了時にplanned距離だけ引いた分を、追跡中の極小サンプル座標にも反映します。同じ通路のターン初期化では横・角度推定を維持し、原点移動済みの候補を新世代へ引き継ぎます。
明示的な座標移動を伴わない世代不一致・座標ジャンプは引き続き棄却します。逆側候補は使用済みラッチを消費せず、旋回開始時に候補を破棄します。
入口・出口の−3 mmはAの特徴点間の差から求めた試験値で、補正ONでの最短走行精度は未検証です。`wall_edge show`の`straight_run_apply:1`が出口からの直進モデルの実適用有効を示します（`exit_apply`は従来の出口補正の表示です）。
`wall_feature_ekf_param`の初期標準偏差4 mm、距離当たりの予測雑音5 mm/√m、観測標準偏差は壁切れ3 mm・独立柱4 mmの試験初期値です。
残差±8 mm、NIS 9以下、Kalman gain最大0.5、実補正±4 mmかつ半量以内を維持します。
同一IRQの予測後に一度だけ観測し、egoへ注入した量を誤差状態から差し引きます。世代変更では縦誤差・その共分散だけを再初期化します。
同じ区間の壁切れと独立柱は一つの使用済みラッチを共有し、同時候補では壁切れを優先します。棄却はラッチを消費しません。
ログが無効・満杯でも実適用は動作します。Bの未校正の進行距離補正・斜め極小は実適用しません。
ホスト試験とA/Bビルドを確認した実装で、補正ONの実機精度・IRQ時間の検証は別途必要です。

斜めの進行距離補正はA/BともShadow専用の枠組みです。`wall_edge show`は`diagonal_feature_framework:1 diagonal_feature_shadow:1 diagonal_feature_apply:0`を返します。
`diagonal_feature_shadow_param.reference[phase][kind][side]`の順序は、phase=入口/直線/出口、kind=差分による壁端/柱の距離極小、side=左/右です。
基準位置mm、同じ側の周期mm、横ずれ感度mm/mm、角度感度mm/rad、校正済みフラグを各モデルで保持します。全12モデルの`calibrated`はfalseで、0 mmを校正済みの基準とは扱いません。
予測基準は`reference + n*period + 座標移動 + lateral_sensitivity*e + heading_sensitivity*psi`です。極小の確定遅れを加えて進行位置と比較します。
柱は極小時の姿勢、従来の差分エッジは確定時の姿勢を使うため、両特徴点を別に校正します。周期の選択も確定遅れと姿勢による移動分を除いた特徴点位置で行います。
前壁の両側接近（近限界を含む）、無効な前壁値、斜め側壁の近限界、旋回中、世代不整合を品質条件とし、横ずれ±8 mm・角度±5°・角速度±2 rad/sを試験ゲートにします。
斜めから直進に出るターン・V90の入口、斜め直線、斜めへ入るターン・V90の出口へ接続しています。同一通路の既知の原点変更では極小位置・推定用の横位置と角度を維持し、旋回/別通路では履歴を破棄します。
長いV90の入口初期化には既存のIRモード切替・リセットがあるため、その境界をまたぐ極小の保持は対象外です。通常V90・45/135度退出ターンの同一斜め通路を引き継ぐ処理とは区別します。
最短経路の斜め柱姿勢EKFは`wall_edge shadow/control`で`PILLAR_EDGE`の`diag_pose_*`に出力します。経路上の柱の左右・接続壁と、柱極小時のエンコーダ・ジャイロ履歴を使います。両方が`control`のとき、機体Aでは採用できた横位置・角度を斜めの目標角速度へ反映します。Bは未校正です。項目は[BLE走行ログの説明](ble/README.md)を参照してください。
旋回終端では測定角度と論理ターン角の残差をShadow姿勢へ渡します。この姿勢・横位置は機体内の推定であり、映像の中心線や車軸の真値とは区別してください。
校正済みモデルを仮定する試験では、独立した進行距離誤差の1状態Kalman評価でNIS・gain・補正案を算出します。横ずれと角度は入力として扱い、走行用の3状態EKFへ融合しません。
初期標準偏差4 mm、距離雑音0.15 mm/√mm、観測標準偏差4 mm、残差±8 mm・NIS≤9・gain≤0.5・補正案±3 mm以内は枠組みの試験初期値です。
同じ側の30 mm以内の観測は同じ柱の候補として重複を抑止します。左右は斜めで交互に現れるため、別々に管理します。
パラメータを校正済みに変更しても実補正量は常に0です。機体位置・横位置・走行指令へ書き戻す経路や、シェルで有効化する操作は用意していません。

```text
wall_ekf show
wall_ekf shadow
wall_ekf control
wall_ekf legacy
```

| モード | 動作 |
| --- | --- |
| `legacy` | 従来の直進壁制御を使用し、EKFを停止します。 |
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
最短の直進から90/180度ターンを経て直進へ戻る場合、A/Bとも`control`では入口姿勢に実測ジャイロの旋回量を足し、論理ターン角を引いて出口へ渡します。
`degree_correction`を含む目標旋回角ではなく、通路の向きが変わる論理角を基準にします。分散は入口から伝播し、原点リセットや同じIRQの二重積分を避けます。
回転中のADC平均を破棄して新しい10サンプルを待ちます。入口姿勢なし・異常なジャイロ・15°を超す残留角は引き継ぎません。
斜め→45/135度退出→直進も、機体Aで両方のモードが`control`かつ斜めEKFの姿勢が制御に採用できる場合に引き継ぎます。入口はセンサ処理済みの姿勢・分散を使い、その周期のジャイロを再加算しません。
斜めの採用条件（左右の観測・横位置と姿勢の分散・観測から220 ms未満かつ1.5斜め区画未満・横位置15 mm以内・姿勢8°以内）を満たさない場合や、未校正の機体Bは引き継ぎません。
`legacy/shadow`は従来の出口処理です。スタート直後の距離基準への拡張、実機での高速走行評価は未実施です。
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
`control`は`Task/Inc/straight_wall_control.h`の32 mm接近モデルを使い、採用可能なEKF偏差だけを従来の斜め制御則の横偏差入力へ渡します。
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

赤外LEDは走行中・手かざし開始待機中に加え、停止中の`irsens`／`ir_sensor`と`interface ir`／`interface photo`の取得中だけ発光します。測定コマンドはモータ制御を有効化せず、点灯後2回のADC完了を待ってから取得し、終了後に測定用の発光要求を解除します。
通常のシェル待機中・測定外および点灯直後の2 ADC周期はIR差分値を0にするため、そのときの換算距離は反射光の測定値ではありません。
電池電圧のADC取得は消灯中も継続します。

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

校正には`IRSENS_AVG`の`value_*`と、別途実測した壁までの距離を対応付けます。`distance_*_mm`は現在の機体別校正表による換算値です。発光ON/OFFそれぞれのADCを見る場合は`interface ir 100 20`を使います（平均値は出力しません）。

| 応答 | 意味 |
| --- | --- |
| `IRSENS_ERROR busy` / `PHOTO_ERROR busy` | モータ制御・実行状態・手かざし待機・他の測定が使用中。現在の発光状態を変更せず拒否 |
| `IRSENS_ERROR adc_timeout` / `PHOTO_ERROR adc_timeout` | 点灯後50 ms以内にADCが準備できず終了。測定用の発光要求は解除済み |
| `IRSENS_ERROR adc_not_ready` / `PHOTO_ERROR adc_not_ready` | 取得中にADCが無効化され中断。測定用の発光要求を解除し、平均値・正常終了行を出力しない |

BLEからの機体Bへの呼出しは[専用シェルのIR測定例](ble/README.md#停止中のir測定と平均値)を参照してください。

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
uint16_t magic;       // 0xA55D
uint16_t index;
half_float data[49];  // 動作/制御出力0～34、IRQ35～38、壁距離/壁制御39～48
int16_t adc[4];       // 左/右/前左/前右のLED点灯ADC−消灯ADC
uint16_t ir_range_states;
uint16_t motion_pattern;
```

終了フレームは `magic = 0xFFFF` です。

周期ログのPIDゲイン6列をADC4列・範囲状態・走行パターンへ置き換えました。
速度・姿勢、位置・滑り、モーター・制御出力、IRQ時間、壁距離・壁制御、ADC整数、範囲状態・走行パターンの順にまとめました。
ADCをhalfへ変換しないため、2049や負値もそのまま保存できます。通常フレーム長は114 byteのままです。
更新後のメイン・debug・turnpattern GUIは`log mode0`を指定し、HEADERの列数・列名・並びから新形式と旧55/51項目を区別します。
新形式のmagicは`0xA55D`、旧形式は`0xA55A`/`0xA55B`/`0xA55C`です。機体とPC受信ツールを両方更新してください。
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
