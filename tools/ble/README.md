# HY0020 用 PC 通信ツール

mouse_type8 の UART を HY0020 経由で BLE 接続するツールです。
すべてのコマンドは mouse_type8 リポジトリのルートから実行します。
HY0020 側のプログラム・配線・書き込みは [firmware/hy0020/README.md](../../firmware/hy0020/README.md) を参照してください。

## ツールの選び方

| ファイル | 用途 |
| --- | --- |
| `mouse_ble_shell.py` | 専用の対話シェル・単発コマンド送受信（最初の動作確認用） |
| `mouse_ble_log.py` | 全55項目（従来51項目＋割り込み計測4項目）を連続受信し、逐次CSVへ保存 |
| `mouse_ble_explore_gui.py` | BLE接続・実機探索開始・ライブ表示・連続ログ保存を一画面で操作 |
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
2. 探索コマンド対応・バッテリ電圧が確認できると、「探索開始」が有効になります。
   接続だけでは走行しません。
3. ゴール座標・サイズ、往復/往路、既知区間の加速、制限時間を設定します。
   開始位置は `(0, 0)`、北向きです。ゴール初期値は機体の設定から取得します。
4. 「探索開始 → 手かざし待ち」を押すと、ログ配信・CSV記録を開始します。
   ログの受信を確認してから探索コマンドを送り、前センサへの手かざしを待ちます。
   ログを受信できない場合は走行要求を送りません。前センサに手をかざすと、
   既存走行と同じLED点滅の後、実際の `Search` による探索が始まります。
5. 手かざし待ち・往路・復路を通してログを記録します。
   最新55項目、選択項目の波形、受信件数・欠落・CRCエラーを画面で確認できます。
6. 探索終了・中断・開始拒否の応答を受けるとログ配信を停止し、BLE接続は維持します。
   「探索を停止」も停止応答までは記録し、その後は送信を止めます。次の探索開始で配信を再開します。
   停止前に送信済みのデータが少量届く場合があります。待機中の画面は最後の受信値を表示します。
   「切断」またはウィンドウを閉じる場合は、停止応答を待って保存を終えます。

画面の更新は約10 Hz、保存するログは指定周期（既定20 ms / 50 Hz）です。
シェル送信は既存GUIと同じ80 ms/文字です。開始時は、ログを止めた状態でコマンドの文字列だけを送ります。
続いてログ配信を有効にし、最初のログを受信してから許可パケットと実行確定のCRを送ります。
長い文字列の送信とログ配信を重ねず、ログを受信できない場合はCRを送らず入力行を取り消します。
CRの送信結果が不明な場合も再送しません。継続要求と受信・CSV保存は独立して進み、
継続要求は探索開始手順の配信開始から終了応答までの間だけ送り、待機中は送りません。
接続時に既存の停止要求で前回のログ配信を止めるため、この動作変更によるSTM32の更新は不要です。
保存先には接続ごとに `<日時>_ble_explore/` を作成し、既存ファイルを上書きしません。

| ファイル | 内容 |
| --- | --- |
| `telemetry.csv` | 探索開始から終了までの全55項目と時刻・連番・欠落数。最初の受信で列数を確定し、受信ごとにflush。探索前はデータ行なし |
| `shell.txt` | バイナリログを除いたシェル応答・探索イベント |
| `events.jsonl` | 状態遷移、開始要求と実行確定要求、停止、終了結果、受信集計、中継カウンター、異常時の詳細 |
| `run_<時刻>.json` | 各探索のゴール・方式・周期と機体情報 |

探索は新しいRAM迷路から開始し、往復時は往路で得た壁情報を復路で引き継ぎます。
制限時間は手かざし待ちから往復終了までの通算秒数です。
機体別の既存探索パラメータ（Aは370、Bは320）と探索アルゴリズムを使い、Flashへ迷路を自動保存しません。
既存の `--gui search` はモータを動かさない再生用です。実機探索には `--gui explore` を選びます。

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

## ログを連続取得する

ストリーム対応の STM32 ファームウェアを対象機体 A / B に書き込んだ後、次を実行します。
HY0020 は既存の UART–BLE 中継ファームウェアをそのまま使います。
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
`--period-ms` は20～1000、`--duration` は秒単位で0なら継続取得です。

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

全55項目を送るフレームは126 byteです。115200 bps / 8N1 のUARTで1 ms周期は帯域不足のため、
最短周期は20 msです（約6300 byte/s）。制御割り込みはサンプルを8件の送信用リングへコピーし、
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

### DMAと計算時間への影響

UART TXは128 byteのリングから連続した領域をまとめてDMAに渡します。
DMA送信中の領域はUARTの送信完了まで確保し、リングの折り返しがあれば2回に分けます。
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

GUI の Port で `BLE`、Baud で `115200` を選んで Connect します。
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
115200 bps、8N1、フロー制御なしです。配線変更時は電源を切り、SWD 用の配線を外します。
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
