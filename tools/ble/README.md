# HY0020 用 PC 通信ツール

mouse_type8 の UART を HY0020 経由で BLE 接続するツールです。
すべてのコマンドは mouse_type8 リポジトリのルートから実行します。
HY0020 側のプログラム・配線・書き込みは [firmware/hy0020/README.md](../../firmware/hy0020/README.md) を参照してください。
現在のUARTは460800 bps / 8N1です。STM32とHY0020の両方へ同じ速度のファームウェアを書き込んでください。
115200 bpsの旧ファームウェアが片側に残っていると通信できません。

## ツールの選び方

| ファイル | 用途 |
| --- | --- |
| `mouse_ble_shell.py` | 専用の対話シェル・単発コマンド送受信（最初の動作確認用） |
| `mouse_ble_log.py` | 全55項目（従来51項目＋割り込み計測4項目）を連続受信し、逐次CSVへ保存 |
| `mouse_ble_benchmark.py` | 停止状態で配信周期を段階測定し、欠落・実効Hz・受信遅延傾向・IRQ時間を保存 |
| `mouse_ble_explore_gui.py` | BLE接続・実機探索/最短走行・ライブ表示・連続ログ保存を一画面で操作 |
| `mouse_ble_gateway.py` | BLE と localhost TCP をつなぐ、既存 GUI 用の中継 |
| `mouse_gui_ble.py` | 既存 GUI に `BLE` ポートを追加して起動 |
| `ble_monitor.py` | NUS デバイスの検索・通知の表示 |
| `uart_monitor.py` | FT232H / WinUSB を使う有線 UART の受信診断 |
| `requirements-ble.txt` | Python 3.10 以上用の BLE ライブラリ（Bleak） |
| `requirements-uart.txt` | FT232H 有線受信用ライブラリ |

コピー元は `HY0020_test` です。コミットと配置の対応は
[ファームウェアの説明](../../firmware/hy0020/README.md#配置とコピー元) に記載しています。
Hello 送信版のファームウェアは含めていません。既定の接続名は `HY0020-MOUSE` です。

## BLE 用 Python の準備

Python 3.10 以上と PC の Bluetooth アダプターが必要です。
Python 3.10 以上を通常の `python` で起動できる場合:

```powershell
python -m pip install --target .tools/ble_python -r tools/ble/requirements-ble.txt
python tools/ble/mouse_ble_shell.py --help
```

この PC では通常の Windows 版 Python 3.12 を使います。
次の `$blePython` は使用する PowerShell ごとに設定してください。

```powershell
$blePython = Join-Path $env:LOCALAPPDATA 'Programs\Python\Python312\python.exe'
& $blePython --version
& $blePython -m pip install --target .tools/ble_python -r tools/ble/requirements-ble.txt pyserial==3.5
```

専用シェルと中継は SDK 同梱の Python 3.10 以上でも使えます。
GUI ランチャーを含む BLE ツールは `.tools/ble_python` の依存パッケージを自動で参照します。
`.tools/` は Git 管理対象外です。別の環境では依存パッケージを導入してください。
PC の Bluetooth をオンにし、スマホ・別の BLE モニターからは切断しておきます。
同時に接続できるのは 1 台です。

## GUIで実機探索と逐次ログを操作する

対象機体A/Bへ、`explore show` に応答する探索対応STM32ファームウェアを書き込みます。
ログ配信だけに対応した以前の版では、GUIのログ表示は動きますが探索開始は無効です。
HY0020のファームウェアは既存の中継版を使います。

Python 3.10以上・Tk・Bleakが必要です。このPCでは通常のPython 3.11で起動します。

```powershell
& "$env:LOCALAPPDATA/Programs/Python/Python311/python.exe" tools/ble/mouse_ble_explore_gui.py

# 既存ランチャーから起動する場合も、中継プロセスは不要
& "$env:LOCALAPPDATA/Programs/Python/Python311/python.exe" tools/ble/mouse_gui_ble.py --gui explore
```

同時にBLEへ接続するのはこのGUIだけにします。
`mouse_ble_log.py`・専用シェル・`mouse_ble_gateway.py`などは終了しておきます。
SDK同梱PythonではTkのDLLが読み込めない場合があるため、このPCでは上記の通常版Pythonを使います。
WinRTのバイナリ依存にはPythonのバージョンを合わせる必要があります。
Python 3.11用は次のように別ディレクトリへ導入します。各ツールは実行中のPythonに対応する
`.tools/ble_python311` 等を優先し、従来の `.tools/ble_python` も検索します。

```powershell
& "$env:LOCALAPPDATA/Programs/Python/Python311/python.exe" -m pip install --target .tools/ble_python311 -r tools/ble/requirements-ble.txt pyserial==3.5
```

1. マウスをアイドルのモード選択状態またはシェルにし、「BLE接続」を押します。
   シェル応答で機体を確認し、探索開始まではログ配信を停止したまま接続を維持します。
   名前検索で見つからない場合は `ble_monitor.py --scan --timeout 10` で確認し、
   検出した機体のアドレスをGUIの「アドレス（任意）」へ入力します。
   広告の名前が空でも、アドレスによる接続は可能です。
2. 探索コマンド対応・バッテリ電圧が確認できると、「走行開始」が有効になります。
   接続だけでは走行しません。
3. ゴール座標・サイズ、往復/往路、既知区間の加速、制限時間を設定します。
   開始位置は `(0, 0)`、北向きです。ゴール初期値は機体の設定から取得します。
   「壁制御」で従来・EKF・従来＋EKF並行計算を選びます。選択値は走行開始前に機体へ適用します。
4. モードを「探索」にして「走行開始 → 手かざし待ち」を押すと、ログ配信・CSV記録を開始します。
   ログの受信を確認してから探索コマンドを送り、前センサへの手かざしを待ちます。
   ログを受信できない場合は走行要求を送りません。前センサに手をかざすと、
   既存走行と同じLED点滅の後、実際の `Search` による探索が始まります。
5. 手かざし待ち・往路・復路を通してログを記録します。
   最新55項目、選択項目の波形、受信件数・欠落・CRCエラーを画面で確認できます。
6. 探索終了・中断・開始拒否の応答を受けるとログ配信を停止し、BLE接続は維持します。
   「走行を停止」も停止応答までは記録し、その後は送信を止めます。次の走行開始で配信を再開します。
   停止前に送信済みのデータが少量届く場合があります。待機中の画面は最後の受信値を表示します。
   「切断」またはウィンドウを閉じる場合は、停止応答を待って保存を終えます。

画面の更新は約10 Hz、保存するログは指定周期（既定20 ms / 50 Hz）です。
シェル送信は既存GUIと同じ80 ms/文字です。開始時は、ログを止めた状態でコマンドの文字列だけを送ります。
続いてログ配信を有効にし、最初のログを受信してから許可パケットと実行確定のCRを送ります。
長い文字列の送信とログ配信を重ねず、ログを受信できない場合はCRを送らず入力行を取り消します。
CRの送信結果が不明な場合も再送しません。継続要求と受信・CSV保存は独立して進み、
継続要求は探索開始手順の配信開始から終了応答までの間だけ送り、待機中は送りません。
接続時に既存の停止要求で前回のログ配信を止めます。最短走行には後述の対応STM32ファームウェアが必要です。
保存先には接続ごとに `<日時>_ble_explore/` を作成し、既存ファイルを上書きしません。

| ファイル | 内容 |
| --- | --- |
| `telemetry.csv` | 探索/最短走行開始から終了までの全55項目と時刻・連番・欠落数。最初の受信で列数を確定し、受信ごとにflush。開始前はデータ行なし |
| `shell.txt` | バイナリログを除いたシェル応答・走行イベント |
| `events.jsonl` | 状態遷移、開始要求と実行確定要求、停止、終了結果、受信集計、中継カウンター、異常時の詳細 |
| `run_<時刻>.json` | 各走行の種別（run_type）・ゴール・方式/プロファイル・迷路・周期・機体情報、選択/確認した壁制御モード、走行前後のEKF状態・終了結果・ログ連番とMCU時刻 |

探索は新しいRAM迷路から開始し、往復時は往路で得た壁情報を復路で引き継ぎます。
制限時間は手かざし待ちから往復終了までの通算秒数です。
機体別の既存探索パラメータ（Aは370、Bは320）と探索アルゴリズムを使い、Flashへ迷路を自動保存しません。
既存の `--gui search` はモータを動かさない再生用です。実機探索には `--gui explore` を選びます。

### GUIから壁制御を比較する

「壁制御」は探索・最短走行の両方に適用します。

| 選択 | 動作 |
| --- | --- |
| 従来制御（legacy） | Zirconiaの従来出力を使用し、EKF計算を止めます。 |
| EKF制御（control） | EKFの横偏差・姿勢・信頼度を直進側壁制御に使用します。 |
| 従来制御＋EKF並行計算（shadow） | GUIの初期値。操舵は従来出力で、EKFを並行計算します。 |

接続時に機体の現在値を読み取り、選択欄の横へ表示します。接続だけではモードを変更しません。
「走行開始」を押すと、配信停止中に選択モードを設定し、さらに `wall_ekf show` で読み返します。
拒否・応答なし・確認値の不一致では、走行コマンドと走行許可を送りません。走行中は選択欄を無効化します。
終了後はログ配信を止めてから再び状態を取得し、次の開始を有効にします。

同じ機体・迷路・開始位置・速度・吸引条件・ログ周期で、まず「従来制御」、次に「EKF制御」を選び、
それぞれ「走行開始」で記録します。置き直し・手かざしは各回に必要です。
同じBLE接続でも走行ごとにモードを適用し直し、電源再投入後もGUIの選択値を開始前に適用します。
計算負荷だけを確認する場合は「従来制御」と「従来制御＋EKF並行計算」を比較します。

設定JSONの `options.wall_ekf_mode` と `wall_ekf.requested_mode` が選択値、
`wall_ekf.confirmed_mode` と `before.mode` が走行直前の機体確認値です。
`wall_ekf.after` に終了後の横偏差・姿勢・共分散・壁採用・更新数などの応答を保存します。
終了後の取得失敗は `after: null` と `after_error`（取得を試みた場合）で示します。
`telemetry_start` は開始許可前に受信を確認した最新点、`telemetry_end` は終了応答を解釈した時点の最新受信点です。
応答と同じBLE通知に含まれるログや停止後の残送信があるため、厳密なモータ駆動区間は指令列でも確認してください。
応答原文と確認失敗は `shell.txt`・`events.jsonl` にも残ります。CSVの列順とバイナリ形式は変えていません。

`wall_ekf` 非対応の旧ファームウェアでは選択欄を無効化し、機体設定を維持して従来どおり取得できます。
接続時にコマンド非対応の明示応答がなく、応答なし・不正応答となった場合は開始を無効にします。
通信を確認して再接続してください。
この場合の `options.wall_ekf_mode` は `keep`、`confirmed_mode` は `null` で、
GUIの選択値や取得日時を根拠にEKF/従来の比較対象へ割り当てません。
従来の保存ログにもモードは追加できません。今回追加した状態取得は走行前後のスナップショットであり、
走行中の全EKF状態・壁採用マスクを連続記録するものではありません。

### 探索の補正理由と判断点を記録する

`EXPLORE_CAP` の `turn_diag:1` に対応するSTM32ファームウェアでは、探索終了後にGUIへ
側壁偏差・前壁左右差・前壁接近の理由別回数と、未確認の閾値超えを除外した回数を表示します。
GUIを更新しただけでは機体側の処理は変わりません。非対応のファームウェアは未対応と表示し、従来どおり取得できます。
`EXPLORE_CAP`の`turn_cov:1`は、ターン入口の姿勢分散を旋回中・出口へ引き継ぐ処理への対応を示します。
設定JSONの`firmware.turn_cov`にも保存され、次回のログで対応ファームウェアを識別できます。

`run_<時刻>.json` の `turn_diagnostics.summary` に全判断回数、理由別回数、除外回数、保存/省略件数を保存します。
`events` には先頭64件の右左折判断を、MCU時刻`ms`・区画`x/y`・機体方位`heading`・旋回方向`turn`・
実際に選択した`reason`・判定フラグ・左右/前壁距離[mm]・EKF横偏差[mm]・姿勢[rad]・標準偏差とともに保存します。
フラグは1=前壁変化の整合、2=候補を前進3 mmで確認、4=この判定に使えるEKF姿勢、8=未確認候補を除外、
16=20 mm以上の左右差、32=従来判定（legacy/shadow）、64=control選択中だがEKF出力を不採用です。
64のときは従来制御を使用し、無壁で姿勢も不確かな場合は補正0になります。
4がないEKF値を確かな位置・姿勢として評価しないでください。64はこの復帰処理に対応するファームウェアから追加されます。
同じ右左折でも側壁合わせは2本、前壁合わせは1本のその場旋回を行います。カウンターはその場旋回の本数ではなく判断の回数です。
Uターンや探索脚の最初の方向合わせは、この右左折判定のカウンターに含みません。

実機側は固定長メモリで先頭64件を保持し、超過分も集計には数えます。モータを停止してから
`EXPLORE_TURN_STATS` と `EXPLORE_TURN` を出力し、その後に `EXPLORE_END` を送ります。
走行中・制御割り込み内のprintfや動的メモリ確保は追加していません。
`complete` は保存対象の件数と連番が届き、終了応答まで受信した意味です。
`summary.dropped` が0以外なら65件目以降の個別記録は省略されています。通信中断で集計/個別行が不足した場合は完全な記録としません。
生の応答は `shell.txt`、受信イベントは `events.jsonl` にも残ります。既存55項目のCSV・バイナリ形式は維持します。

### 開始・停止の通信

```text
explore show
explore start goal_x goal_y goal_size plain|acc outward|roundtrip limit_seconds
```

`explore show` は `EXPLORE_CAP protocol:1 mouse:A|B ...` を返します。
探索中は `EXPLORE_WAIT_SENSOR`、`EXPLORE_PHASE name:outward|return`、
`EXPLORE_END result:complete|stopped|link_timeout|time_limit|motion_error` を通知します。
引数や電圧・取得状態に問題がある場合は、動作を始めず `EXPLORE_ERROR` を返します。

ログ取得要求と同じCRC付き8 byteパケットの操作2で探索を中断し、操作3で次の探索を一度だけ許可します。
停止要求はシェル処理を待たずRX割り込みで受け、次の制御周期でモータ出力を0にします。
探索中にログ継続要求が3秒途絶えた場合や制限時間超過でも停止します。
GUIは切断後の探索開始要求を自動再送しません。再接続後に新たに開始してください。

### 同じBLE接続で最短走行する

対象機体A/Bへ、`shortest show` に応答する本変更以降のSTM32ファームウェアを書き込みます。
HY0020の中継ファームウェア・ログのバイナリ形式は共通です。旧STM32でも探索は利用できますが、最短走行の開始は無効です。

1. 上記GUIでBLE接続し、モードを「最短走行」に切り替えます。探索完了後も接続を維持したまま切り替えられます。
2. 迷路を「直近のBLE探索（RAM）」または「保存済み迷路（Flash）」から選びます。
   RAMは同じ電源投入中に正常完了したBLE探索の壁情報です。新たな探索を始めると作り直し、
   中断した探索の迷路は採用しません。Flashを選んでもRAMの探索結果は保持します。自動でFlashへ保存はしません。
3. ゴール・サイズ・制限時間・最短プロファイルを選択します。プロファイル一覧は機体から取得します。
   既定は `turn_500（plain500）`。`turn_700（plain700）`、吸引走行 `suction...`、可変ターン `acc...` も選択でき、
   対象機体の `Params/run_config.h` と機体別パラメータをそのまま使用します。
4. 機体を `(0, 0)` 北向きへ置き直し、「走行開始 → 手かざし待ち」を押します。
   ログの受信と走行許可を確認後、既知の壁情報でDijkstra経路が作れるかを確認します。
   RAM迷路なし・経路なしの場合は走行しません。手かざしとLED点滅の後、ゴールまで片道の最短走行を行います。
5. 経路計算・手かざし待ち・吸引立上げ・走行・終了応答まで逐次CSVを保存します。
   終了後はログ配信だけを止め、同じBLE接続で次の探索/最短走行を開始できます。

制限時間は経路計算・手かざし待ちを含む通算時間です。「走行を停止」、継続要求が3秒途絶えた場合、
制限時間超過では、探索と同じ制御割り込みの停止処理を使います。開始要求は切断後も自動再送しません。
最短走行中も受信・継続要求は別タスクで動き、既定20 ms周期のログを同じ記録フォルダへ追記します。
経路の再計算中に停止要求を受けた場合も、モータ開始前に停止判定を再確認します。

```text
shortest show
shortest start goal_x goal_y goal_size profile ram|flash limit_seconds
```

`SHORTEST_CAP protocol:1 mouse:A|B ... ram_maze:0|1 profiles:...` が対応機能とプロファイルを返します。
走行時は `SHORTEST_PHASE name:planning|running`、`SHORTEST_WAIT_SENSOR`、`SHORTEST_END result:...` を返します。
結果は `complete` / `missing_maze` / `no_path` / `stopped` / `link_timeout` / `time_limit` / `motion_error` です。
`complete` はモーション処理がエラーなく終了した意味で、実測のゴール到達精度を保証しません。
`goal_x` 等は指令先であり、自己位置の測定値ではありません。
引数・電圧・許可/ログ配信の条件を満たさない場合は `SHORTEST_ERROR` を返します。
手入力だけでは開始条件を満たせないため、走行には専用GUIを使用します。

本変更はA/BのDebugビルドと模擬BLE・ホストC++・非表示Tkによる検証対象です。
実機の最短走行中の受信安定性・停止・制御周期への影響は未確認です。

### 斜め区間のEKFを比較する

1. 更新したSTM32ファームウェアとGUIを使い、最短プロファイルに `turn_500（plain500）` を選びます。
   実際の送信コマンドと設定JSONのプロファイル名は `plain500` です。
   機体別の `turn_500.h` と既存の直進/斜め速度設定を使い、ターンは500 mm/s、直線は経路に応じて500〜700 mm/sとなります。
2. 「壁制御」を `従来制御（legacy）`、`従来制御＋EKF並行計算（shadow）`、`EKF制御（control）` で切り替え、
   同じ迷路・ゴール・プロファイルで比較します。選択したモードは開始前に読み返し確認します。
3. `SHORTEST_CAP diagonal_ekf:1` の機体では、斜め対応をGUIへ表示します。対応機体の `control` は直進と斜め直線の両方へ作用します。
   旧機体の機能は変更されず、斜めEKF未対応と表示します。
4. 走行後、GUIの「斜め S区間: EKF適用 N/M周期 ／ 観測更新 K回」を確認します。
   Sは速度0.20 m/s超の連続した斜め制御区間数です。
   Mは速度0.20 m/s超で斜め制御を処理した回数、NはそのうちEKFの推定偏差を実際に使った回数、Kは観測更新数です。
   `shadow`はKが増えてもNは0、`legacy`はN/Kが0です。Mが0なら斜め区間の制御を評価できていません。
   `diagonal_controller:2`では、control時の「従来入力」はEKF偏差を採用せず従来入力を使った周期数で、Nと合わせてMになります。
   「前方除外」は近い側壁候補があり、両前センサ66 mm以下で近接観測を除外した周期数です。shadowでも集計します。
   `diagonal_edge:1`では「壁切れ 右 R回・左 L回」も表示します。斜めの柱通過検出数で、位置補正の採用回数ではありません。

実機はモータを停止してから、次の応答を出します。

```text
SHORTEST_WALL_STATS diagonal_cycles:100 diagonal_ekf_cycles:60 diagonal_updates:4 diagonal_segments:2 diagonal_controller:2 diagonal_fallback_cycles:40 diagonal_front_blocked_cycles:3 diagonal_edge:1 diagonal_edges_right:12 diagonal_edges_left:13 diagonal_edge_delay_max_mm:2.500
SHORTEST_END result:complete ...
```

値は走行ごとに初期化し、`run_<時刻>.json` の `wall_control_stats` と `events.jsonl`へ保存します。
対応ファームウェアから応答が届かなければ `wall_control_stats` は `null` のままで、前の走行値を流用しません。
これらは制御周期の集計で、ターン回数や中心ずれの精度を表す値ではありません。
`diagonal_edges_right/left`は検出回数、`diagonal_edge_delay_max_mm`は0差通過から確認までの最大エンコーダ移動量[mm]です。
柱通過は10 mmの距離履歴で判定し、確認遅れを斜め距離補正へ反映します。方式と条件は[斜めの壁切れ](../../README.md#斜め走行中の柱通過壁切れ検知)を参照してください。
斜めControlではCSVの `control_ir` が実際に制御へ使った横偏差入力[mm]、`control_ir_dot` が相対ジャイロ姿勢[rad]になります。
`diagonal_controller:2`では前者はEKF採用時の接近偏差、その他は従来のIR/オドメトリ入力です。
斜めLegacy/Shadowの同列は従来の意味です。既存55項目の列順・バイナリ形式は変えません。

斜め用EKFは32 mm基準の近い柱を横方向の近似観測として使います。
柱が見えない間の値0を中心線の観測にせず、柱から姿勢の確信度も上げません。
新しい斜め軸では広い初期分散から始めます。状態は直進側と独立し、斜めの旋回自体は既存PID/FFを使います。
斜め直線も従来の制御則・速度依存ゲイン・軌道制動を保ち、採用可能なEKF偏差だけを横偏差入力へ使います。
片側前センサが近くても柱候補を残し、両前センサ66 mm以下では観測を除外します。
未確認・不採用・柱のない区間では従来入力を保ち、相対姿勢を新しい絶対方向の基準にはしません。
この近接モデルはA/Bの試作値で、修正後の実機での中心走行・安定性・処理時間は未確認です。

## ログを連続取得する

ストリーム対応の STM32 ファームウェアを対象機体 A / B に書き込んだ後、次を実行します。
HY0020 は460800 bpsに対応した UART–BLE 中継ファームウェアを使います。
通常の GUI、中継、専用シェルなど、ほかの BLE クライアントは切断してください。

```powershell
python tools/ble/mouse_ble_log.py

# 100 ms周期で60秒だけ取得し、新しいCSVへ保存
python tools/ble/mouse_ble_log.py --period-ms 100 --duration 60 --output tools/logs/live_100ms.csv
```

この PC では、準備した Python 3.10 以上の `$blePython` で起動します。

```powershell
& $blePython tools/ble/mouse_ble_log.py
```

接続と通知購読の後に取得を自動開始し、既定では20 ms周期（50 Hz）で取得します。
終了は Ctrl+C です。CSV の既定保存先は `tools/logs/<日時>_ble_live.csv` です。
ファイルは受信ごとに flush し、既存ファイルは上書きしません。
`--period-ms` の要求許可範囲は3～1000 msで、既定は20 msです。`--duration` は秒単位で0なら継続取得です。
20 ms未満の要求には、最短3 ms対応のSTM32ファームウェアへの更新が必要です。旧ファームウェアは20 ms未満の要求を無視します。

シェルモードへの移行や `disp log_bin` は不要です。アイドル、Interface / Demo / Debug、
シェルで取得でき、`log_enable` が無効な間も現在の55項目を送ります。
保存ログと同じ取得処理・half float・列順・単位を使用します。
内部の保存ログは最新400件を保持するリングです。満杯になると最古のデータを更新します。
保存周期は既定1 msで、停止後に `disp log` / `disp log_bin` で古い順に取り出せます。
配信だけを有効にしても保存リングは更新しません。
GUI・専用シェルだけを接続した場合は自動配信を開始しません。

STM32 には BLE 接続状態が直接届かないため、受信ツールが0.5秒ごとに取得要求を送ります。
通常終了時は停止要求を送り、切断・PC 停止などで取得要求が3秒途絶えた場合も自動停止します。
旧 STM32 ファームウェアなどで有効なフレームが5秒届かない場合はエラー終了します。
切断後はツールを起動し直してください。

全55項目を送るフレームは126 byteです。460800 bps / 8N1のUARTで1フレームの送信時間は約2.734 ms、
20 ms周期では約6300 byte/sでUART帯域の約13.7%を使います。1 ms周期は帯域不足です。
3 ms周期の要求は約333 Hz・42000 byte/sでUART帯域の約91.1%を使い、送信待ち時間の余裕は小さくなります。
要求周期は実効受信周期や走行中の安定性を保証しません。`time_ms` の間隔、欠落、CRC、割り込み時間を確認してください。
制御割り込みはサンプルを8件の送信用リングへコピーし、
CRC計算・DMA送信は低優先度のSysTickから進めます。UARTが混雑している間は準備済みフレームを保持し、
送信用リングも満杯になったときだけ新しいサンプルを見送ります。
シェルの大量出力などが重なると欠落が増えます。
連番の飛びを `lost_frames`、破損を画面の `crc_errors` で確認できます。
HY0020 側のバッファあふれ・UART/BLE異常ではエラー終了します。

新形式のCSVは `cnt` と55項目に `sequence,time_ms,period_ms,lost_frames` を追加した60列です。
先頭51項目の並び・単位は維持しています。受信ツールは旧51項目の `L8S1` も読め、旧形式は56列で保存します。
途中で項目数が変わった場合は列ずれを防ぐため終了し、再接続で新しいCSVを作成します。
`cnt` は最初の受信サンプルからの実経過時間[ms]で、行番号ではありません。
`time_ms` はSTM32起動後の32 bit時刻[ms]、`sequence` は取得試行の32 bit連番（取得再開でリセットしない）、
`lost_frames` は直前の受信から欠けたフレーム数（初回は0）です。時刻・連番のラップも処理します。
フレームは1 msの内部ログの間引きであり、送られなかったサンプルの再送はありません。
走行の実行や設定の変更は、この受信ツールからは行いません。

### 停止状態で配信周期を段階測定する

STM32 は最短3 ms対応版、HY0020 は460800 bps版へ更新し、UART 配線と電源投入を済ませます。
ほかの BLE クライアントを切断し、マウスを停止したシェル状態にして実行します。

```powershell
# 20 → 10 → 5 → 4 → 3 msを各15秒測定する例
& $blePython tools/ble/mouse_ble_benchmark.py --periods 20,10,5,4,3 --duration 15

# 候補周期だけを60秒測定する例。別の実行で繰り返す
& $blePython tools/ble/mouse_ble_benchmark.py --periods 5 --duration 60
```

引数なしの既定は20 / 10 / 5 / 4 / 3 msを各10秒です。`--duration` は1段あたり0より大きく60秒以下です。
モード選択状態からシェルへ入る場合だけ `--enter-shell` を付けます。機体は既定Aで、Bは `--mouse B` を指定します。
配信停止中に読み取り専用の `explore show` で対象機体と `state:idle` を確認し、同じ BLE 接続で全段を測定します。
送るバイナリ要求はログ開始・継続の操作1とログ停止の操作0だけで、探索開始・許可・遠隔停止は送りません。
各段の前後に配信を停止して残留受信を流し、最初の要求周期のフレームを受けてから集計します。

各段はCRC・連番欠落・HY0020の4カウンター増加・途中の周期不一致・受信キューあふれ・5秒の有効フレーム未受信で失敗します。
診断対応HY0020では、受信停止理由・RX再開失敗・受信バッファ割当/応答失敗のカウンター増加も失敗として記録します。
リンク診断対応時は通知の最終失敗数の増加も失敗とし、資源不足による通常の再試行回数だけでは失敗にしません。
欠落が0でも、2件以上のサンプルで時刻間隔が要求以上、平均間隔が要求の105%以下でなければ失敗です。
画面の `effective_hz` と `mean_dt_ms` で実効周期を確認します。失敗時はログ停止を要求し、以降の段は実行しません。
短い段階試験後、採用候補を60秒で複数回確認します。要求下限3 msは実機での安定受信を保証する値ではありません。

保存先は `tools/logs/<日時>_ble_benchmark/` で、すべて新規作成します。

| ファイル・項目 | 内容 |
| --- | --- |
| `NN_Pms.csv` | 要求周期に一致する55項目と連番・STM32時刻等の60列。列の意味は通常の連続取得と同じ |
| `NN_Pms.notifications.bin` | 先頭ASCII `M8N1`。以後はlittle endianの `<dI>`（段開始からのPC単調時計の秒数double、通知長uint32）と、その長さの通知生バイト列を繰り返す |
| `NN_Pms.json` | 合否、`sample.interval_ms` / `effective_hz`、通知byte数・通知間隔・受信キュー最大、HYカウンター前後差分、IRQ分布と累積最大値の開始・終了値 |
| `summary.json` | 全段の結果、機体応答、取得できたATT MTU。リンク診断対応時は実conn interval・PHY・data lengthを保存し、未取得は `null` |

`bridge_final_status_read` が偽の場合、切断などで終了時のカウンターを取得できていません。
この場合 `bridge_after` / `bridge_delta` は `null`、直前の観測値は `bridge_last_observed` に分けて記録します。
診断対応時は `diagnostics_before` / `diagnostics_after` / `diagnostics_delta` と `diagnostics_last_observed` も保存します。
`diagnostics_final_read` が偽なら最終値を取得できておらず、`diagnostics_after` / `diagnostics_delta` は `null` です。
診断特性のない旧HY0020は `diagnostics_supported:false` として通常測定を続けます。読取り失敗や不正な形式は未対応とせず、
`diagnostics_read_error` に記録して測定を失敗にします。同じ接続で対応確認後に特性が消えた場合も失敗です。
リンク診断は同様に `link_before` / `link_after` / `link_delta` / `link_last_observed` を保存します。
`link_final_read:false` の場合、`link_after` / `link_delta` は `null` で、最後の観測値を最終値として扱いません。
`link_supported:false` は明示的な特性欠如だけを示します。読取り失敗・不正形式・接続途中の特性消失は
`link_read_error` に記録して失敗します。リンク診断がない旧版でも、既存Statusと受信停止診断の検証は続けます。

`relative_lag_slope_ms_per_s` はPC受信時刻とSTM32時刻の相対的なずれの傾きで、バッファ滞留と両時計のドリフトが混ざります。
絶対遅延やFIFO残量の直接測定ではありません。通知は複数フレームの一部を含められ、通知間隔とサンプル間隔は異なります。
Windows/WinRTのATT MTUだけから実転送量やPHYは断定できません。キュー滞留はcallbackで記録する `connection_queue_highwater` も確認します。
IRQ列は前回完了した制御処理の値で、累積最大値は各段の開始時にリセットしません。
この合否は停止状態でのデータ取得の基準です。保存ログ有効時や走行中の制御負荷・壁判定の安定性は別途検証します。

### HY0020の受信停止理由

旧Status特性 `6e400004-b5a3-f393-e0a9-e50e24dcca9e` は、従来どおり4項目のlittle-endian uint32、16 byteです。
診断対応ファームウェアは読取り専用特性 `6e400005-b5a3-f393-e0a9-e50e24dcca9e` を追加します。
v1はlittle-endianの `<20I>`、80 byteで、順序は次のとおりです。

| word（0始まり） | 項目 |
| --- | --- |
| 0 | `version`（1） |
| 1～2 | `last_stop_reason`, `stop_reason_or` |
| 3～9 | `overrun_count`, `parity_count`, `framing_count`, `break_count`, `collision_count`, `noise_count`, `other_reason_count` |
| 10～15 | `rx_disabled_count`, `rx_restart_attempts`, `rx_restart_failures`, `rx_buffer_alloc_failures`, `rx_buffer_rsp_failures`, `rx_buf_request_events` |
| 16～19 | `rx_buffer_size`, `rx_buffer_count`, `rx_timeout_us`, `rx_mode_flags` |

理由bitmapはZephyrの `UART_RX_STOPPED` の生値で、bit0～5は順にoverrun / parity / framing / break / collision / noiseです。
複数bitがあれば各理由カウンターへ加算します。理由0または未知bitを含む場合は `other_reason_count` も増えます。
理由カウンターは起動からの全受信停止を累積し、旧Statusの `uart_errors`（接続・通知有効中の停止）とは集計条件が異なります。
SDK v3.4.1のnRFドライバはERRORSRCの複数エラーから優先する1理由だけを返すため、全ハードウェアエラーの同時記録ではありません。
`rx_mode_flags` のbit0はlegacy CBWT使用、バッファサイズ・個数・タイムアウトは実行時設定の確認用です。

ベンチは13項目のカウンターだけをuint32 wrap対応で差分計算し、理由bitmapや設定値はそのまま保存します。
正常動作でも増える受信無効化・再開試行・バッファ要求の回数だけでは失敗にしません。
MTU23などで80 byteの読取りが複数のRead Blobに分かれた場合、全項目が厳密に同時取得されたことは保証しません。
理由別カウンターの合計を受信停止回数や欠落byte数と同一視しないでください。

### HY0020のBLEリンク・中継診断

リンク診断対応版は読取り専用特性 `6e400006-b5a3-f393-e0a9-e50e24dcca9e` を追加します。
v1はlittle-endianの `<24I>`、96 byteです。既存0004/0005の形式と動作は維持します。

| word（0始まり） | 項目・単位 |
| --- | --- |
| 0～4 | `version`（1）, `mtu` [byte], `interval_us` [µs], `latency` [接続イベント数], `supervision_timeout_ms` [ms] |
| 5～10 | `tx_phy`, `rx_phy`, `tx_max_len` [byte], `tx_max_time_us` [µs], `rx_max_len` [byte], `rx_max_time_us` [µs] |
| 11～17 | `ring_bytes`, `ring_highwater` [byte], `notify_calls`, `notify_bytes`, `notify_failures`, `notify_call_max_ms` [ms], `notify_retry_events` |
| 18～23 | `batch_flush_ms` [ms], `sdc_tx_packet_count`, `acl_tx_count`, `acl_tx_size` [byte], `ring_capacity` [byte], `flags` |

PHY値は1=LE 1M、2=LE 2M、4=LE Codedです。実リンクの間隔・PHY・DLEを取得できない項目は0です。
`flags` bit0は通知のバッチ化を示します。SDC/ACLの個数・サイズはビルド設定であり、実際の無線パケット数や実効帯域ではありません。
`ring_bytes` はUART→BLEリングだけの残量で、送信待ちのローカル通知やBluetoothスタック内の待機分を含みません。
`ring_highwater` と `notify_call_max_ms` は起動からの最大値です。各段の最大値へリセットしません。

`notify_calls` は `bt_gatt_notify` の呼出数（再試行を含む）、`notify_bytes` はスタックに受理された通知byte数です。
受理されたことだけでは無線到達を保証しません。`notify_failures` は再試行を打ち切った最終失敗数、
`notify_retry_events` は資源不足などで再試行した回数です。この4項目だけをuint32 wrap対応で差分計算します。
設定値・現在残量・累積ピークには差分計算を適用しません。複数項目の厳密な同時取得は保証しません。

ベンチの `summary.link_at_connection` は接続直後、`link_last_observed` は最新の成功した観測です。
`connection_interval_ms` / `phy` / `data_length` は最新の実リンク観測から更新します。
取得元は `link_metrics_source` に保存し、特性がない場合は `unknown`、個々の未取得値は `null` とします。
段階ごとの `link_last_observed_metrics` も最後の観測に対応し、最終値として使えるかは `link_final_read` で区別します。

### 旧送受信構成での短周期確認（2026年11月4日）

このPC、機体A、UART 460800 bps、ATT MTU報告値247、停止状態で、5 msは60秒×3回・計36,176件を正常受信しました。
実効頻度は約199.4 Hz、欠落・CRCエラー・HY0020カウンター増分はすべて0です。既定20 msの約4倍の配信頻度です。
4 msは2回ともBLE切断、3 msの単独試験も切断し、再接続時にHY0020のUART受信エラー累積値の増加を確認しました。
試験当時のHY0020は128 byte×3の受信バッファで、停止理由を記録しておらず、overrunなどの内訳は未特定です。
最初の受信対処版ではlegacy CBWTと252 byte×4を導入してUART受信停止は0になりましたが、4 ms長期では送信リングがあふれ、
3 msではSTM32送信待機キューの見送りとBLE排出待ちが残りました。経緯は `tools/logs/20261104_hy0020_rxfix_ef0c2671/report.md` に保存しています。
この旧構成での最短安定候補は5 msでした。現在の送受信対処版の結果は次節を参照してください。
結果・raw・CSV・書き込み条件は `tools/logs/20261104_ble_fastlog_5b67da98/report.md` に保存しています。

### 送受信対処版での3 ms確認（2026年11月4日）

機体AとHY0020を更新し、停止状態で3 msを60秒×3回、計180秒・60,366件正常受信しました。
実効332.6～332.7 Hzで、連番欠落・CRC・UART/BLEエラー・破棄カウンター増分はすべて0です。
STM32のTX512/RX128、HY0020の最大244 byteまとめ送り（部分データの追加待ち最大7 ms）とSDC送信6/ACL送信12を使用しています。

実リンクはCI15 ms・送受信PHY2 Mbps・DLE251 byte・MTU247でした。送信リングの累積最大は3470 / 16384 byte、
通知APIの累積最大待ちは47 ms、IRQ累積最大は521 µsでした。終了時リング0で、持続的な到着遅れ増大はありませんでした。
独立したraw/CSV照合と、ログ停止後のASCII応答・機体A idle・通常切断も確認しています。

3 msは機体側の取得周期で、BLE通知はまとめ送りです。既定20 msは維持し、CLIの`--period-ms 3`やGUIの周期3 msで明示指定します。
126 byte・8N1を名目460800 bpsで送るだけで約2.734 ms必要なため、同じ速度・フレームで2 msの持続配信はUART容量を超えます。
走行中の探索負荷・壁判定と、機体Bの実機通信は未確認です。データと条件は
`tools/logs/20261104_ble_fastlog_v2_9dc817ac/report.md`と`results.json`に保存しています。

### DMAと計算時間への影響

UART TXは512 byteのリングから連続した領域をまとめてDMAに渡します。RXは128 byteを維持します。
DMA送信中の領域はUARTの送信完了まで確保し、リングの折り返しがあれば2回に分けます。
複数のログフレームを予約して、1 msの送信サービス時刻とUART完了時刻のずれによる送信の空白を減らします。
ログ停止時もUARTへ予約済みの最大512 byte（線上の送信時間は460800 bpsで約11.1 ms）は送出し、未予約のログだけを破棄します。
通常の126 byteフレームを20 ms周期で送る場合、DMA起動と送信完了コールバックは各50回/秒です。
不要なDMA半転送割り込みも無効にしています。
RXはシェル互換の1 byte DMAを継続使用します。継続要求は8 byteを0.5秒ごとに送るため、16 byte/sです。

CRC処理は優先度15のSysTickに置き、優先度0のADC制御割り込みが先に動ける構成です。
迷路計算中でも配信は進みますが、サンプル取得・CRC・送信完了処理・DMAのバス使用による負荷は残ります。
迷路計算時間への影響がゼロとは保証しません。実機で配信有無による計算時間と制御周期を測定してください。
SysTick・ADCの優先度は既存の `.ioc` / HAL設定と一致させています。

### 連続取得のバイナリ形式

`disp log_bin`（新55項目は114 byte、magic `0xA55A`）とは別の形式です。
数値はすべてlittle endian、CRCは CRC-16/CCITT-FALSE（poly `0x1021`、初期値 `0xFFFF`）です。

| フレーム内位置 | 型 | 内容 |
| --- | --- | --- |
| 0 | 4 byte | ASCII `L8S2`（55項目。旧 `L8S1` は51項目） |
| 4 | uint32 | 取得試行連番（送信見送りも加算） |
| 8 | uint32 | 取得時刻[ms] |
| 12 | uint16 | 取得周期[ms] |
| 14 | binary16 × 55 | 保存ログと同じ列順のデータ |
| 124 | uint16 | 先頭124 byteのCRC（旧形式は位置116） |

取得要求は8 byte（`00 4C 38`、操作 uint8、周期 uint16、先頭6 byteのCRC uint16）です。
操作1はログ開始・継続、0はログ停止、2は遠隔探索停止、3は次の遠隔探索を許可します。
操作2はログ配信を止めません。要求の文字間隔は100 ms以内にします。
STM32のRX割り込みで要求を取り出すため、シェル入力・迷路アップロードへ要求は渡しません。
通知の境界とフレームの境界は一致する必要がなく、受信ツールはCRCを確認しながら再同期します。

### 割り込み時間をログで確認する

機体A/Bに計測対応ファームウェアを書き込み、更新後のGUIを起動し直します。
ライブ画面の「表示項目」で次の4列を選ぶと、波形とCSVで確認できます。別途の計測コマンドは不要です。

| 列 | 内容・単位 |
| --- | --- |
| `irq.total_us` | 前回のADC1制御コールバック本体の処理時間 [µs] |
| `irq.logging_us` | その回の `LogData::logging()` の処理時間 [µs] |
| `irq.max_us` | 起動または `reset_log()` / `log init` 以後の最大処理時間 [µs] |
| `irq.period_us` | 前回と、その1回前の制御コールバック開始時刻の間隔 [µs] |

CPUのDWTサイクルカウンタを使い、割り込み内のprintfやHAL_GetTickによる1 ms単位の計時は行いません。
処理時間はコールバック終了後に確定するため、各ログには前回完了した割り込みの値を載せます。
`irq.max_us` は送信間隔内に発生した遅い回も保持します。GUI探索の走行開始時には保存ログとともに最大値をリセットします。
Demoでも同じ計測処理が働き、連続取得CLIで比較できます。Demo探索は保存ログを自動有効化しない点に注意してください。

通常の周期は約1000 µsです。`irq.total_us` / `irq.max_us` が1000 µsに近い場合は余裕が少なく、
超える場合は制御コールバックだけで1周期を使っています。`irq.period_us` で周期の乱れも確認します。
計測対象はADCコールバック内のpreprocess・main・postprocessで、HALの前後処理、例外の入退出、計測値の集計部分は含みません。
カウンタ読出しなどの計測負荷は加わるため、この値だけで締切順守を保証するものではありません。
binary16の丸め（約1000 µs付近では0.5 µs刻み）があり、65504 µsを超える値は無限大になります。
カウンタが使えない場合、初期化直後、またはリセット直後の未計測値はNaNです。
デバッガで停止した期間やスリープ時の実時間を測る用途には使いません。

## 専用シェルで通信する

マウスをアイドルのモード選択状態にして実行します。

```powershell
& $blePython tools/ble/mouse_ble_shell.py --enter-shell --command 'info ver'
& $blePython tools/ble/mouse_ble_shell.py --command 'info battery'
& $blePython tools/ble/mouse_ble_shell.py --command 'help'
```

`--enter-shell` は `shell` + CR を送り、0.5 秒待ってから本コマンドを送ります。
マウスがすでにシェルに入っている場合は省略します。
すでにシェル内で `shell` を送ると `Unknown command found.` が返ることがあります。

対話モード:

```powershell
& $blePython tools/ble/mouse_ble_shell.py --enter-shell
```

コマンドを入力して Enter で送信します。`/quit` または Ctrl+C は PC 側の接続を終了します。
マウス側のシェルも終了する場合は、先に `end exe` を送ります。

- ASCII コマンドの末尾に CR を 1 個付けます。既定の文字間隔は `--char-delay 0.08` 秒です。
- `--command` を複数指定すると順番に送信します。コマンド内に改行は含めません。
- 単発モードでは最後の受信から 1 秒で応答を区切り、最大 60 秒待ちます。
  長い処理は `--idle-timeout` / `--response-timeout` を調整するか対話モードを使います。
- `--raw-output .tools/response.bin` で受信バイトをそのまま新規保存します。既存ファイルは上書きしません。
  画面表示はテキスト用なので、バイナリの確認には保存ファイルを使います。
- コマンドを自動で繰り返したり、再接続して再実行したりしません。

マウスが何も出力していなければ、接続しただけでは文字列は届きません。
応答がない場合は TX / RX、共通 GND、電源、シェルのモードを確認します。

## 既存の GUI を BLE で使う

まず専用シェルを終了します。2 つのターミナルをリポジトリのルートで開きます。

ターミナル 1（Python 3.10 以上）:

```powershell
$blePython = Join-Path $env:LOCALAPPDATA 'Programs\Python\Python312\python.exe'
& $blePython tools/ble/mouse_ble_gateway.py
```

`Ready: socket://127.0.0.1:8765` が表示されたら、ターミナル 2 で GUI を起動します。
GUI には既存の [tools/README.md](../README.md#セットアップ) の依存パッケージと Tkinter が必要です。
SDK 同梱 Python で Tk の DLL が不足する環境では、GUI は通常の Windows 版 Python で起動します。

```powershell
$blePython = Join-Path $env:LOCALAPPDATA 'Programs\Python\Python312\python.exe'
& $blePython tools/ble/mouse_gui_ble.py --gui main
```

GUI の Port で `BLE`、Baud は既定の `460800` で Connect します。
`--gui debug`の走行調整GUIでは、続けて`Show device values`を押して設定値と電圧を確認します。
`Char delay`は保守的な既定`0.08`秒で、明示した文字間隔も使用できます。
最初の照会時にshellモードを自動準備するため、マウスが停止したモード選択状態でも接続後に取得できます。
低電圧・照会の未応答時は設定変更や走行コマンドを送りません。画面の入力値は接続し直しても保持します。
仮想シリアルのBaud欄は物理USARTの速度を変更しません。STM32とHY0020間の速度は双方のファームウェア設定で決まります。
`--gui` は `main` / `debug` / `turnpattern` / `search` を選べます。
タイムアウトを設定できる GUI は大きめ（例: 120 秒）にします。
シェルの操作方法は [myshell シリアル操作ガイド](../myshell_README.md) と共通です。

リポジトリ位置はスクリプトから自動判定します。別のチェックアウトを使う場合だけ
`--mouse-repo <パス>` を指定します。ポート番号を変える場合は両プロセスに同じ `--port` を指定します。

中継は `127.0.0.1` のみで待ち受け、クライアントは 1 台です。
GUI を切断すると中継も終了するので、再接続時は中継を起動し直します。
`BLE` はこのランチャーから起動した GUI 内の接続先です。通常の COM ポートも利用できます。
迷路・ログはバイナリのまま転送し、転送エラーが検出されると接続を終了します。

## 診断ツール

BLE デバイスの検索と通知表示:

```powershell
& $blePython tools/ble/ble_monitor.py --scan
& $blePython tools/ble/ble_monitor.py --seconds 10
```

`ble_monitor.py` の `--send` は CR 付加や文字間隔の調整をしない汎用 NUS 送信です。
mouse_type8 のコマンド送信には `mouse_ble_shell.py` を使います。
Windows でサービス無効のエラーが出る場合は Bluetooth Support Service（`bthserv`）と
Windows 設定の Bluetooth を確認します。

FT232H を使う有線受信はオプションです。

```powershell
& $blePython -m pip install --target .tools/python -r tools/ble/requirements-uart.txt
& $blePython tools/ble/uart_monitor.py --seconds 10
```

WinUSB のまま、HY0020 の P0.06 / TX → FT232H の D1 / RX、GND 同士を接続します。
460800 bps、8N1、フロー制御なしです。配線変更時は電源を切り、SWD 用の配線を外します。
このモニターは受信専用です。中継版の UART には独立した定期 Hello 出力はありません。

## ホストテスト

Python 3.10 以上、Bleak と pyserial が必要です。Bleak は上の手順で、
pyserial は GUI のセットアップまたは次のコマンドで導入します。

```powershell
& $blePython -m pip install --target .tools/ble_python pyserial==3.5
& $blePython -m unittest discover -s tools/tests -p test_mouse_ble_transport.py -v
```

実機と通信せず、localhost と模擬 BLE 相手で次を確認します。

- ASCII + CR、文字単位の送信、シェル移行と生データ保存。
- 20 byte 分割、キュー満杯時の再試行、タイムアウト時の再送抑止、ステータス異常検出。
- NUL を含む 1024 byte の迷路と 106 byte のログの保持、模擬 BLE との双方向転送。
- 既存 COM ポートの引き継ぎ、このリポジトリの GUI の自動選択と明示指定。

Python 3.10 未満または依存パッケージがない場合はスキップします。
BLE の検証としては、上記コマンドでスキップなしの成功を確認してください。
実機での電波条件、バイナリ転送、走行の確認は別途必要です。

## 機体 A での動作確認（2026年11月3日）

機体 A を給電し、HY0020 と UART 接続した状態で、このリポジトリの PC ツールを確認しました。

- `ble_monitor.py --scan --timeout 10` で `HY0020-MOUSE` を 1 台検出しました。
- `mouse_ble_shell.py` で `info ver` / `info battery` / `help` の送受信が成功しました。
  バージョンは `0.0.0`、電圧は `7620 mV`（下限 `7000 mV`、`OK`）でした。
  生応答 899 byte を `build/hy0020/tool-check-20261103-77ac8d57/shell.bin` に保存し、各応答を確認しました。
- すでにシェルに入っていたため、開始時の `shell` には `Unknown command found.` が返りました。
  続くコマンドは正常応答しており、この状態では次回の `--enter-shell` を省略できます。
- `mouse_ble_gateway.py` と `mouse_gui_ble.py --gui main` で `BLE @ 115200` への接続が成功しました。
  GUI の走行設定 17 件の自動読み込み、`help`、入力欄からの `info battery` 送信を確認しました。
  GUI に表示された電圧は `7566 mV`、状態は `OK` でした（CLI とは別時点の測定値）。

今回の確認は BLE 検出とシェル・メイン GUI の基本通信です。
実機の迷路・ログのバイナリ転送、debug / turnpattern / search GUI の操作、
FT232H 有線モニター、機体 B は未確認です。ファームウェアの書き込みや走行は行っていません。

## 現在の PC への環境導入（2026年11月3日）

- Windows 版 Python 3.12.6 を使用し、`.tools/ble_python` に Bleak 3.0.2 / pyserial 3.5 と依存パッケージを導入しました。
- `.tools/python` に `requirements-uart.txt` の FT232H 有線受信用パッケージを導入し、libusb backend の読込みを確認しました。
- `test_mouse_ble_transport.py` は、GUI ランチャーの独立起動の回帰ケースを含む 12 件がすべて成功し、スキップはありません。
- CLI 5 種の `--help` と GUI 4 種のモジュール読込みを確認しました。
- Windows のアダプター情報から BLE 対応と Bluetooth が ON であることを確認しました。

今回の環境導入では BLE のスキャン・接続・送信、実機への書き込み・走行は行っていません。
FT232H は接続デバイス一覧にないため、WinUSB ドライバーの割当ては未確認です。

## 現環境での実機通信確認（2026年11月3日）

- ユーザーが Zadig で切り替えた FT232H（`0403:6014`）は、Service `WinUSB`、状態 `OK`、問題コード `0` を確認しました。
- PC の Bluetooth アダプターから `HY0020-MOUSE`（`C1:B5:60:87:91:73`）の NUS 広告を検出しました。
- 通常の Windows ユーザー環境で専用シェルを実行し、`info ver` / `info battery` の送受信が成功しました。
  応答はマウス側の `Version 0.0.0` と `7515 mV`（下限 `7000 mV`、状態 `OK`）です。
  生応答 122 byte を `build/hy0020/ble-check-20261103-175416.bin` に保存しています。
- すでにシェル中だったため、開始時の `shell` には `Unknown command found.` が返りました。続く 2 コマンドは正常応答しました。
- サンドボックス内では接続時に `BleakDeviceNotFoundError` となり、通常の Windows ユーザー環境で再実行して成功しました。

搭載機体の A / B は今回未照合です。走行・Flash 書き込みは行わず、試験後に BLE 接続を終了しています。

## 逐次ログの実機確認（2026年11月4日）

`HY0020-MOUSE` に `mouse_ble_log.py` で接続し、次を確認しました。

| 指定周期 | 指定取得時間 | 保存件数 | サンプル間隔 | 欠落 / CRCエラー |
| --- | --- | --- | --- | --- |
| 20 ms | 10秒 | 504件 | 20 ms × 493回、21 ms × 10回 | 0 / 0 |
| 100 ms | 5秒 | 51件 | 100 ms × 49回、101 ms × 1回 | 0 / 0 |

CSVはいずれも56列（51項目と時刻・連番など）で、連番は連続しています。
保存先は `tools/logs/20261104_062706_182206_ble_live.csv` と
`tools/logs/20261104_062821_330838_ble_live.csv` です。
前者の受信後、取得要求を送らず `ble_monitor.py --seconds 4` で再接続したところ、
4秒間の受信は0 byteでした。さらに100 ms周期での取得再開に成功しました。
検証の詳細は `tools/logs/20261104_062706_182206_ble_live_validation.json` に保存しています。

停止中の取得では `speed_ctrl.get_D_peration` と `omega_ctrl.get_D_peration` が全行 `NaN` でした。
ほかの49項目は有限値です。`Component/Inc/controller.h` のD項表示は `dt` で除算しており、
起動後にPIDの `Control()` が未実行なら静的オブジェクトの `dt` が0のままとなる実装に整合します。
この2列の値の問題と、通信の欠落・CRCエラーが0件だったことを区別してください。

搭載機体のA/Bと書き込み済みイメージの厳密な照合は未実施です。
今回はログ取得・停止・再接続を確認し、Flash書き込み・走行指令の送信は行っていません。
迷路計算時間や制御割り込みの実行時間は測定していません。

## 探索GUIの実機接続確認（2026年11月4日・機体A）

以下は、接続中に常時ログを配信していた版での確認記録です。
現在のGUIは上記手順のとおり、探索開始から終了応答までだけ配信します。

探索コマンド対応の `Debug-MouseA` を書き込み・照合し、ユーザーによるバッテリー交換と
電源再投入後に、Python 3.11の探索GUIで確認しました。

- `explore show` は `protocol:1 mouse:A`、`info battery` は7364 mV・下限7000 mV・`OK`でした。
- 20 ms周期で72.4秒・3619件を取得した時点で、欠落0・GUIのCRCエラー0・セッションのエラー記録0でした。
  ログ中の電圧は約7.34～7.37 Vで、「探索開始 → 手かざし待ち」が有効になりました。
- 保存先は `tools/logs/20261104_073544_212653_ble_explore/` です。上記は継続記録中の確認時点の値です。
  書き込み・ホスト検証の記録は `build/ble-explore-check-20261104-49928a8d/validation.json`、
  交換後の確認記録は同ディレクトリの `post_battery_gui_validation.json` にあります。
- 交換直後の別セッションでは短時間の切断があり、原因は未確定です。
  上記の継続受信と区別し、長時間運用や走行中の安定性を確認済みとは扱いません。

この確認では走行開始コマンドを送っていません。実際の手かざし待ち・探索走行中のログ取得は未確認です。

## 割り込み時間4列の実機確認（2026年11月4日・機体A）

計測対応 `Debug-MouseA` を書き込み・照合し、書き込み前後で保存用Flash末尾8 KiBの一致を確認しました。
停止中・保存ログ無効・BLE 50 ms周期で102件を取得し、60列すべての列対応と計測4列の有限値を確認しました。
欠落・CRCエラーは0件です。取得サンプルの `irq.total_us` は平均302.63 µs、`irq.logging_us` は平均3.86 µs、
`irq.max_us` の最大は529.5 µs、`irq.period_us` は1017.5～1018.0 µsでした。
これらは停止中の値で、走行中の制御負荷や保存ログ有効時のログ処理時間を確認した結果ではありません。

CSVは `tools/logs_20261104/20261104_092118_irq_timing_idle.csv`、集計は同名の `.validation.json` に保存しました。
書き込み・保存領域の確認記録は `build/irq-timing-check-20261104-090936-87b01108/post_flash_validation.json` です。
