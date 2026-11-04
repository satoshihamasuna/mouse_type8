# HY0020 UART–BLE 通信モジュール

mouse_type8 の USART1 を BLE（Nordic UART Service、NUS）へ双方向に中継する、
HY0020 / AE-HY0020-DIP（nRF52832）用のファームウェアです。
広告名は `HY0020-MOUSE` です。PC 側の使い方は [tools/ble/README.md](../../tools/ble/README.md) を参照してください。

```text
PC の BLE ツール ⇄ Bluetooth / NUS ⇄ HY0020 ⇄ UART 460800 8N1 ⇄ mouse_type8
```

## 配置とコピー元

2026年11月3日に `C:\Users\sato1\Documents\git\HY0020_test` の
コミット `a25ab0349cadfac3980f415464c5c3ed99d4f7e6` の作業ツリーからコピーしました。
コピー対象に未コミットの変更がないことを確認しています。

| コピー元（HY0020_test 内） | このリポジトリの配置 | 内容 |
| --- | --- | --- |
| `sketches/mouse_uart_ble/` | `firmware/hy0020/` | 中継ファームウェア、ボード定義、ビルド設定 |
| `openocd/ft232h-hy0020.cfg` | `firmware/hy0020/openocd/` | FT232H の SWD 設定 |
| `tools/` の BLE 通信・UART 受信ツール | `tools/ble/` | 専用シェル、GUI 中継、診断ツール |
| `tests/test_mouse_ble_transport.py` | `tools/tests/` | PC 側の模擬通信テスト |

```text
firmware/hy0020/
├─ README.md
├─ build.ps1                         nRF Connect SDK を使ったビルド
├─ CMakeLists.txt / Kconfig / prj.conf
├─ src/main.c                        UART–BLE 中継本体
├─ src/batch_policy.h                部分通知の待機期限判定（ホスト検証共用）
├─ boards/fdk/hy0020_dip/             HY0020 のボード・UART 端子設定
└─ openocd/ft232h-hy0020.cfg          FT232H の書き込み設定
```

UART 単独版・Hello 文字列送信版、元リポジトリの `.git`・`.vscode`・SDK・
導入済み Python ライブラリ・ビルド生成物はコピーしていません。
コピー時点では中継本体、ボード定義、通信設定はコピー元と同じでした。
現在は本リポジトリの UART 速度を 115200 bps から 460800 bps へ変更しています。
HY0020 → マウスの最低 2 ms / byte の間隔はコピー元の設定を維持しています。
短周期ログへの対処として、UART受信のタイムアウト通知方式とDMAバッファを変更し、受信診断を追加しています。
コピー先に合わせてビルド出力、PC ツールの参照先、GUI のリポジトリ既定値を調整しています。
STM32 用の CMake からは独立したプロジェクトです。

## mouse_type8 との配線

対象は Mouse A / B 共通の USART1 インターフェースです。
`Core/Src/usart.c` と HY0020 の設定は 460800 bps、8 bit、パリティなし、ストップ 1 bit、フロー制御なしです。
PA9 / PA10 は MCU 端子名です。基板コネクターの位置は対象機体の回路図で確認してください。

460800 bps へ移行するときは、対象機体 A / B の STM32 本体と HY0020 の両方に新しいファームウェアを書き込みます。
片側だけを更新すると UART 速度が一致せず、BLE が接続できてもマウスとのコマンド・ログ通信はできません。
STM32 本体のビルドと書き込みは [README.md](../../README.md#flash--write-to-mcu) を参照してください。
両方の更新後、電源を切って UART 配線へ戻し、配線切替と電源投入が完了してから BLE の疎通を確認します。

電源を切ってから接続し、HY0020 には安定した 3.3 V を供給します。

| mouse_type8 | HY0020 |
| --- | --- |
| USART1 TX / PA9 | P0.08 / RX |
| USART1 RX / PA10 | P0.06 / TX |
| GND | GND |

信号も 3.3 V 系で、TX / RX を交差させます。5 V や RS-232 信号を直接入力しません。
給電元は一つにします。通常の BLE 接続には PC の Bluetooth アダプターを使います。

## ビルド

以下のコマンドは mouse_type8 リポジトリのルートで実行します。
nRF Connect SDK v3.4.1 と対応ツールチェーンが必要です。
Zephyr / BLE を含むため、STM32 用の `cube-cmake` ではビルドしません。

この PC では [Nordic 公式の導入手順](https://nrfconnectdocs.nordicsemi.com/ncs/latest/nrf/installation/install_ncs.html) に従い、
nRF Util と SDK Manager を `.tools/nrfutil/`、SDK とツールチェーンを `C:\ncs` に配置しています。
導入済み構成は次のコマンドで確認できます。`NRFUTIL_HOME` はこの PowerShell 内だけで設定します。

```powershell
$env:NRFUTIL_HOME = Join-Path (Get-Location).Path '.tools\nrfutil\home'
& .\.tools\nrfutil\nrfutil.exe sdk-manager list --install-dir C:\ncs
```

```powershell
powershell -ExecutionPolicy Bypass -File .\firmware\hy0020\build.ps1
```

既定の SDK は `C:\ncs\v3.4.1`、ツールチェーンは `C:\ncs\toolchains\4f5b6ad6dd` です。
導入先が異なる場合は指定します。

```powershell
powershell -ExecutionPolicy Bypass -File .\firmware\hy0020\build.ps1 -SdkRoot 'C:\ncs\v3.4.1' -ToolchainRoot 'C:\ncs\toolchains\使用するツールチェーン'
```

`-Pristine` は HY0020 のビルドディレクトリを再構成する場合に使います。
出力は `build/hy0020/mouse_uart_ble/zephyr/zephyr.hex`、
キャッシュは `build/hy0020/zephyr-cache/` です。STM32 の機体別ビルド出力と分けています。
ツールチェーンの環境設定はスクリプトの実行中だけ適用します。

UART 速度は [ボード設定](boards/fdk/hy0020_dip/hy0020_dip_nrf52832.dts) の `current-speed` で指定します。
460800 bps は nRF Connect SDK v3.4.1 内の Zephyr の `dts/bindings/serial/nordic,nrf-uart-common.yaml` で選択可能な値であり、
`drivers/serial/uart_nrfx_uarte.c` と nrfx の `hal/nrf_uarte.h` にも対応する設定が定義されています。

## FT232H による SWD 書き込み

OpenOCD と FT232H の WinUSB ドライバー（USB ID `0403:6014`）が必要です。
電源を切り、マウスとの UART 配線を外してから次の接続に変更します。

| FT232H | HY0020 |
| --- | --- |
| D0 / ADBUS0 | SWDCLK |
| D1 / ADBUS1 | 220～470 Ω の抵抗を介して SWDIO |
| D2 / ADBUS2 | 同じ SWDIO へ直結 |
| GND | GND |

HY0020 へ 3.3 V を供給します。D1 と D2 は抵抗なしで短絡しません。
FT232H の UART モニターなど、同じアダプターを使うプログラムは終了しておきます。

```powershell
$openocdExe = Join-Path (Get-Location).Path '.tools\openocd\xpack-openocd-0.12.0-7\bin\openocd.exe'

# 接続確認（消去・書き込みなし）
& $openocdExe -f firmware/hy0020/openocd/ft232h-hy0020.cfg -c 'init; shutdown'

# ビルド済み HEX の書き込み・照合・リセット
& $openocdExe -f firmware/hy0020/openocd/ft232h-hy0020.cfg -c 'adapter speed 100; program build/hy0020/mouse_uart_ble/zephyr/zephyr.hex verify reset exit'
```

この PC には [xPack OpenOCD v0.12.0-7](https://github.com/xpack-dev-tools/openocd-xpack/releases/tag/v0.12.0-7) を上記の場所へ導入しています。
別の環境では OpenOCD の配置に合わせて変更してください。設定の既定速度は 10 kHz、
中継版の書き込みはコピー元で確認済みの 100 kHz を明示します。
終了コード 0、`Verified OK`、リセット完了を確認します。
APPROTECT で保護されている場合の `nrf52_recover` は Flash 全域・UICR を消去するため、
通常の書き込み手順には含めていません。
書き込み後は電源を切って mouse_type8 用の UART 配線に戻します。

稼働中のZephyrを単に`halt`して書く場合、MPUのRAM実行禁止によりRAM上のFlashアルゴリズムが
タイムアウトすることがあります。2026年11月4日の対処版書き込みでは、タイムアウト後にMPU有効と
命令アクセス違反を確認しました。対象Flashは消去済みだったため、照合が完了するまで給電と停止状態を保ちました。
この場合は設定読込み後、`init`の前に`nrf52.cpu configure -work-area-size 0`を指定し、通常の
`flash write_image erase`と`verify_image`を使うことでCPUを動かさずに書き込み・照合できます。
OpenOCDの[nRF5ドライバ](https://raw.githubusercontent.com/openocd-org/openocd/e5888bda3/src/flash/nor/nrf5.c)が
直接書き込みへ切り替える経路です。今回の100 kHzでは書き込み約94秒、照合約16秒でした。
保護設定を維持したまま対象ページだけを書き直し、全Flash読み戻しの一致を確認してから再起動しました。

## 中継の動作と制限

- CR / LF / NUL を含むバイト列を変換せずに転送します。UART にデバッグ文や起動バナーを混ぜません。
- HY0020 → マウスは既定で最低 2 ms / byte の間隔を入れます。設定は `CONFIG_MOUSE_UART_BYTE_GAP_US` です。
- UART の DMA 受信バッファは252 byte×4個です。nRF52832のUARTEの255 byte上限をビルド時に確認します。
  4個のプールから現在・次のバッファを供給します。4個すべてをハードウェアへ同時に予約する構成ではありません。
- UARTの受信byte数をTIMER2で数えるlegacy CBWTを使います。1 msの無通信タイムアウトでは受信を停止せず、
  到着済みの末尾だけを通知します。短いASCII応答もバッファ満了を待たずに転送します。
  DTSの`uart0.timer`でTIMER2を専有し、Bluetooth/MPSLが使うTIMER0は変更しません。
- マウス → PC は16 KiBのリングから、`min(244, MTU-3)` byteが揃えば即通知します。MTUは247を推奨します。
  部分データは最初に待ち始めてから最大7 msで通知を試み、新着データでは期限を延ばしません。
  満杯分の送信後に残る部分データは新しい待機として扱います。待機にはセマフォを使います。
  7 msはまとめ送りの追加待機であり、Bluetoothの資源待ち・無線到達までの時間を保証する値ではありません。
- 送信バッファはSDCが6個、ホストACLが12個です。SDKの整合条件に合わせEvent受信バッファを13個にします。実際の接続間隔・PHY・データ長は接続相手との交渉で決まり、
  `USER_PHY_UPDATE` / `USER_DATA_LEN_UPDATE`は観測のために有効化しています。自動DLEを明示的に維持し、
  PHYの自動要求方針は変更していません。
- 無接続時・通知無効時の UART 出力は保存しません。接続後にコマンドを送ります。
- BLE → UART は Write Request を使い、満杯のキューは ATT エラー `0x11` で拒否します。
  PC ツールは拒否が明確なチャンクだけを再試行し、結果不明のタイムアウトでは再送しません。
- UART のあふれ・エラー、BLE 通知の継続失敗を検出すると切断します。切断時の未送信キューは破棄します。
- 接続は 1 台です。ペアリング・暗号化を要求しない開発用の設定です。

| 特性 | UUID | 用途 |
| --- | --- | --- |
| NUS Service | `6e400001-b5a3-f393-e0a9-e50e24dcca9e` | サービス |
| TX | `6e400003-b5a3-f393-e0a9-e50e24dcca9e` | HY0020 → PC の通知 |
| RX | `6e400002-b5a3-f393-e0a9-e50e24dcca9e` | PC → HY0020 の書き込み |
| Status | `6e400004-b5a3-f393-e0a9-e50e24dcca9e` | 4 個の little-endian uint32 |
| Diagnostics | `6e400005-b5a3-f393-e0a9-e50e24dcca9e` | v1・20個のlittle-endian uint32（80 byte）、読取り専用 |
| Link Diagnostics | `6e400006-b5a3-f393-e0a9-e50e24dcca9e` | v1・24個のlittle-endian uint32（96 byte）、読取り専用 |

Status は UART 破棄バイト数、UART エラー数、キュー満杯回数、BLE 送信エラー数の順です。
既存Statusの16 byte形式と集計条件は維持します。Diagnosticsは起動からの全`UART_RX_STOPPED`の理由、
受信無効化・再開試行・バッファ要求と各失敗の回数、受信設定を保持します。読取りや再接続ではリセットしません。
再開失敗は`uart_rx_enable`の失敗（`-EBUSY`を含む）、割当失敗は初期受信・再開・次バッファ要求の合計です。
`UART_RX_STOPPED`の理由はZephyrが通知した値です。SDK v3.4.1のnRFドライバはハードウェアの複数エラーから
優先する1理由を返すため、全ERRORSRCの同時記録とは扱いません。
Read Blobの断片にはoffset 0で採った同じ値を使いますが、複数カウンターの厳密な同時取得は保証しません。
項目順とPCの合否判定は[受信停止理由](../../tools/ble/README.md#hy0020の受信停止理由)を参照してください。

Link Diagnosticsの形式は`<24I>`で、既存Status / Diagnosticsの形式は変更しません。
項目順は次のとおりです。リンク値は`bt_conn_get_info()`から取得し、取得できないリンク値は0です。

| word | 項目（順序どおり） |
| --- | --- |
| 0–4 | `version=1`, `mtu`, `interval_us`, `latency`, `supervision_timeout_ms` |
| 5–10 | `tx_phy`, `rx_phy`, `tx_max_len`, `tx_max_time_us`, `rx_max_len`, `rx_max_time_us` |
| 11–17 | `ring_bytes`, `ring_highwater`, `notify_calls`, `notify_bytes`, `notify_failures`, `notify_call_max_ms`, `notify_retry_events` |
| 18–23 | `batch_flush_ms=7`, `sdc_tx_packet_count=6`, `acl_tx_count=12`, `acl_tx_size=251`, `ring_capacity=16384`, `flags` |

PHYはZephyrのbitmask（1=1M、2=2M、4=Coded）、`latency`は接続イベント数、`flags`のbit0はまとめ送り有効です。
`ring_bytes`は読取時点のリング内byte数で、取出し済み・送信待ちの1通知分やBluetoothスタック内の待機分を含みません。
`ring_highwater`と`notify_call_max_ms`は起動からの最大値です。通知カウンターも切断・CCC変更ではリセットしません。
`notify_calls`は再試行を含むAPI呼出し回数、`notify_bytes`はenqueue成功分だけで、無線到達を表しません。
`notify_retry_events`は`-ENOMEM` / `-EAGAIN`で再試行待機へ入った回数です。`notify_failures`は最終的に再試行せず
終了した非0の回数（`-ENOTCONN`を除く）で、これらの2秒期限切れも含みます。
Read Blobはoffset 0の値を保持しますが、リンク値と個別atomicカウンターの厳密な同時snapshotではありません。
まとめ送りやバッファ数の変更後の持続帯域は、末尾の「送受信対処版の機体A検証」に実測結果を記載しています。
この構成をSDK v3.4.1でビルドし、Flash 118872 byte、RAM 57232 byte / 65536 byteで成功しました。
実ヘッダのホスト検証5件も成功しています。スタックの実使用量は未確認です。

この変更は、4/3 msログで観測した受信停止に対し、タイムアウト時のSTOPRXとバッファ切替の負担を減らす対処です。
旧版は停止理由を保存していないため、バッファ不足やタイムアウトだけが原因だったとは確定していません。
書き込み後、機体Aの停止状態で20/5/4/3 ms配信と、配信停止後のASCII応答・再接続を確認しました。結果は末尾の検証記録を参照してください。

## コピー元での検証記録

以下はコピー元 README に記録された結果です。今回のコピー作業で実機確認をやり直した結果ではありません。

- nRF Connect SDK v3.4.1 でビルド成功。Flash 116776 byte / 512 KiB、RAM 50792 byte / 64 KiB。
- 2026年11月3日に DMA バッファ修正版の書き込み・照合・リセットと `HY0020-MOUSE` の広告を確認。
- マウス給電・UART 接続で `info ver` / `info battery` / `help` の BLE 送受信を確認。
  応答は `Version 0.0.0`、バッテリー 7672 mV、899 byte の生データで、4 個のエラーカウンターはすべて 0。
- 実マウスでの迷路・ログのバイナリ転送と走行コマンドは未確認。Mouse A / B ごとの実機確認の区別は記録されていません。

コピー後の PC 側テスト手順は [tools/ble/README.md](../../tools/ble/README.md#ホストテスト) を参照してください。

## コピー後の検証（2026年11月3日）

- この配置から nRF Connect SDK v3.4.1 / GCC 14.3.0 でビルド成功、終了コード 0。
  `build/hy0020/mouse_uart_ble/zephyr/zephyr.hex` を生成し、Flash 116776 byte、RAM 50792 byte を確認しました。
- SDK 同梱 Python 3.12.4 / Bleak 3.0.2 で `test_mouse_ble_transport.py` の 11 件がスキップなしで成功しました。
- 中継本体、ボード定義、通信設定、SWD 設定、専用シェル、BLE requirements の計 11 ファイルで、
  コピー元との SHA-256 の一致を確認しました。PC ツールの構文と主要 4 コマンドの `--help` も確認しました。
- 今回の作業では実機への書き込み、UART / BLE の実接続、走行は実施していません。
  FT232H 有線モニターの実受信と Mouse A / B それぞれでの通信は未確認です。

## 現在の PC への環境導入（2026年11月3日）

- nRF Util 8.2.1 / SDK Manager 1.16.1 で nRF Connect SDK v3.4.1 とツールチェーン `4f5b6ad6dd` を `C:\ncs` に導入しました。
  SDK Manager の一覧で SDK / Toolchain の両方が `Installed` であることを確認しました。
- 上記の `build.ps1` を GCC 14.3.0 で実行し、終了コード 0 で `zephyr.hex` / `zephyr.elf` を生成しました。
  Flash 116776 byte、RAM 50792 byte で、コピー元の記録と同じ使用量です。対象は Mouse A / B 共通の HY0020 中継ファームウェアです。
- xPack OpenOCD v0.12.0-7 を `.tools/openocd/` に導入し、公式配布の SHA-256 との一致、起動と既存 SWD 設定の読込みを確認しました。
  `init` を実行せず、`ftdi` / `swd` / `nrf52.cpu` の設定認識を確認しています。
- SDK 展開時に一部のシンボリックリンクが作成されませんでした。対象一覧は `.tools/nrfutil/sdk-symlink-skipped.txt` に保存しています。
  HY0020 のビルドは上記のとおり成功しています。
- Codex の通常サンドボックス内では SDK の Git 所有者チェックにより west の拡張読込みが止まったため、
  SDK を導入したユーザー環境でビルドを確認しました。Git のグローバル設定は変更していません。

今回の環境導入では実機への接続・書き込み・走行は行っていません。
FT232H は接続デバイス一覧にないため、WinUSB ドライバーの割当てと SWD 実接続は未確認です。

## UART 高速化のビルド確認（2026年11月4日）

- `build.ps1` を nRF Connect SDK v3.4.1 / GCC 14.3.0 で実行し、460800 bps 版の `zephyr.hex` / `zephyr.elf` を生成しました。
  終了コードは 0、Flash は 116776 byte、RAM は 50792 byte です。Mouse A / B 共通の HY0020 用です。
- 生成された `zephyr.dts` の `current-speed = <0x70800>`（460800）と、
  `.config` の `CONFIG_MOUSE_UART_BYTE_GAP_US=2000` を確認しました。
- 実機への書き込み・UART / BLE 接続・走行は行っていません。460800 bps での実通信と探索への影響は未確認です。

## 短周期受信対処の事前検証（2026年11月4日）

- legacy CBWT・252 byte×4・Diagnostics v1追加版をnRF Connect SDK v3.4.1 / GCC 14.3.0でビルドし、終了コード0を確認しました。
  Flashは117624 byte、RAMは51568 byte / 65536 byteです。スタックの実使用量は実機では未確認です。
- 生成DTSのTIMER2専有、生成configの`CONFIG_UARTE_NRFX_UARTE_COUNT_BYTES_WITH_TIMER_LEGACY=y`、
  UART 460800 bpsとHY0020→マウスの2000 µs / byte維持を確認しました。
- ホスト検証は通信20件・ベンチ17件・ログ23件、計60件がスキップなしで成功しました。
  旧Status互換、診断非対応の旧版、読取り異常・特性消失、切断時の最終値の扱い、CRC・分割受信を含みます。
- Mouse A/B共通のHY0020向け変更で、今回の対処にSTM32側の追加変更はありません。
- 旧版と対処版のHEX/ELF、生成設定、書き込み前検証は`build/hy0020-rxfix-20261104-ef0c2671/`へ保存しています。
  対処版HEXのSHA-256は`44d953e7761d6cb2d9fd7586c745e31624809a0fdb2312269d5a5d42a5354397`です。
- この事前検証時点では対処版の実機書き込みと4/3 msの再試験は未実施です。配線・給電の準備完了を確認してから進めます。

## 短周期受信対処版のSWD書き込み確認（2026年11月4日）

- ユーザーによるSWD配線・3.3 V給電の準備完了後、HY0020へ上記SHA-256の対処版を書き込みました。
  マウス本体は電源OFFで、STM32への追加書き込みは行っていません。
- 書き込み前にFlash 512 KiBとUICR 4 KiBを保存し、旧HEXの全116772 byteとの一致を確認しました。
  初回のRAMアルゴリズムはタイムアウトしたため、上記の直接書き込みへ切り替え、書き込み・照合を完了しました。
- 新HEXの全117616 byte、消去ページ内の余白、0x1D000以降の未消去ページ保持、UICR全4 KiB保持を
  独立したPython検証で確認しました。全消去やUICRへの書き込みは行っていません。
- 再起動後、`UARTE0_BAUDRATE=0x07400000`、`ENABLE=8`、`TIMER2_MODE=1`、`BITMODE=3`を実機で読み出しました。
  460800 bps設定と32 bitの受信byteカウンタ設定を確認し、CPUを動作状態に戻しました。
- 記録は`build/hy0020-rxfix-20261104-ef0c2671/flash_session.json`と`validation.json`です。
  この段階ではUART配線へ戻す作業の完了待ちで、対処版のBLE通信と4/3 msの再試験は未実施です。

## 送信対処追加版のHY0020書き込み確認（2026年11月4日）

- 最大244 byteへのまとめ送り・部分データの追加待ち最大7 ms、SDC送信6個・ACL送信12個・Event受信13個、
  UUID0006の実リンク診断を追加した版をビルドしました。Flash 118872 byte、RAM 57232 / 65536 byteです。
  静的配置の余りは8304 byteで、実行時のスタック使用量は未測定です。
- STM32 TX512/RX128版のA/B Debugビルドと、通信26・ベンチ27・ログ26・HY batch 5の計84件がスキップなしで成功しました。
- ユーザーがHY0020だけのSWD・3.3 V準備完了を確認した後、同機だけを書き込みました。
  HEX SHA-256は`741734777d575384b90eba844623587d125f16a3ff62f27144f1d8ec2980d55a`です。
- 旧版の全Flash 512 KiBとUICR 4 KiBを保存し、旧HEX 117616 byteとの一致を確認後、直接書き込みで更新しました。
  新HEX全118864 byte、消去ページ内余白、0x1E000以降の保持、UICR全4 KiBの保持を独立照合しました。
- 再起動後のUART 460800設定、TIMER2の32 bitカウンタ設定、CPU再開も実機確認しました。
  記録は`build/ble-fastlog-v2-20261104-9dc817ac/hy0020/flash_session.json`と`validation.json`です。
- この段階では機体AのSWD準備完了待ちです。機体AのTX512版書き込みと、UART復帰後のBLE短周期試験は未実施です。

## 送受信対処版の機体A検証（2026年11月4日）

- 後続で機体AのTX512版も書き込み、全Flash読み戻し一致、保存迷路8 KiBと未消去ページの保持、再起動を確認しました。
  記録は`build/ble-fastlog-v2-20261104-9dc817ac/mouse_A/flash_session.json`です。
- ユーザーによるUART配線・通常給電への復帰確認後、20/5/4/3 msの各15秒試験と、3 msの60秒×3回を実施しました。
  長期試験は計180秒・60,366件、実効332.6～332.7 Hzで欠落・CRC・UART/BLEエラー・破棄増分0でした。
- UUID0006で実CI15 ms、送受信PHY2 Mbps、DLE251 byte、MTU247を確認しました。
  長期試験のraw通知は244 byte、リング最大3470 / 16384 byte、終了残量0、通知APIの累積最大待ち47 msでした。
- rawのCRC・連番・時刻とCSV全60列を独立照合し、モータ出力・目標速度0、IRQ累積最大521 µsを確認しました。
  最後はログ停止後のASCII応答・7355 mV/status:OK・機体A idleを確認し、BLEを切断しました。
- 停止状態・当該PC/電波条件での検証です。走行中の探索負荷や壁判定、機体Bの実機通信、実行時スタック最大使用量は未確認です。
  結果とraw/CSVは`tools/logs/20261104_ble_fastlog_v2_9dc817ac/report.md`と`results.json`を参照してください。
