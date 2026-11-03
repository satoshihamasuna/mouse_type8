# HY0020 用 PC 通信ツール

mouse_type8 の UART を HY0020 経由で BLE 接続するツールです。
すべてのコマンドは mouse_type8 リポジトリのルートから実行します。
HY0020 側のプログラム・配線・書き込みは [firmware/hy0020/README.md](../../firmware/hy0020/README.md) を参照してください。

## ツールの選び方

| ファイル | 用途 |
| --- | --- |
| `mouse_ble_shell.py` | 専用の対話シェル・単発コマンド送受信（最初の動作確認用） |
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
