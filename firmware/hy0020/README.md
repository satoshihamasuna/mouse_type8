# HY0020 UART–BLE 通信モジュール

mouse_type8 の USART1 を BLE（Nordic UART Service、NUS）へ双方向に中継する、
HY0020 / AE-HY0020-DIP（nRF52832）用のファームウェアです。
広告名は `HY0020-MOUSE` です。PC 側の使い方は [tools/ble/README.md](../../tools/ble/README.md) を参照してください。

```text
PC の BLE ツール ⇄ Bluetooth / NUS ⇄ HY0020 ⇄ UART 115200 8N1 ⇄ mouse_type8
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
├─ boards/fdk/hy0020_dip/             HY0020 のボード・UART 端子設定
└─ openocd/ft232h-hy0020.cfg          FT232H の書き込み設定
```

UART 単独版・Hello 文字列送信版、元リポジトリの `.git`・`.vscode`・SDK・
導入済み Python ライブラリ・ビルド生成物はコピーしていません。
中継本体、ボード定義、通信設定はコピー元と同じです。
コピー先に合わせてビルド出力、PC ツールの参照先、GUI のリポジトリ既定値を調整しています。
STM32 用の CMake からは独立したプロジェクトです。

## mouse_type8 との配線

対象は Mouse A / B 共通の USART1 インターフェースです。
`Core/Src/usart.c` の設定は 115200 bps、8 bit、パリティなし、ストップ 1 bit、フロー制御なしです。
PA9 / PA10 は MCU 端子名です。基板コネクターの位置は対象機体の回路図で確認してください。

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
$openocdExe = Join-Path $env:USERPROFILE 'Downloads\xpack-openocd-0.12.0-7-win32-x64\xpack-openocd-0.12.0-7\bin\openocd.exe'

# 接続確認（消去・書き込みなし）
& $openocdExe -f firmware/hy0020/openocd/ft232h-hy0020.cfg -c 'init; shutdown'

# ビルド済み HEX の書き込み・照合・リセット
& $openocdExe -f firmware/hy0020/openocd/ft232h-hy0020.cfg -c 'adapter speed 100; program build/hy0020/mouse_uart_ble/zephyr/zephyr.hex verify reset exit'
```

OpenOCD の配置は環境に合わせて変更してください。設定の既定速度は 10 kHz、
中継版の書き込みはコピー元で確認済みの 100 kHz を明示します。
終了コード 0、`Verified OK`、リセット完了を確認します。
APPROTECT で保護されている場合の `nrf52_recover` は Flash 全域・UICR を消去するため、
通常の書き込み手順には含めていません。
書き込み後は電源を切って mouse_type8 用の UART 配線に戻します。

## 中継の動作と制限

- CR / LF / NUL を含むバイト列を変換せずに転送します。UART にデバッグ文や起動バナーを混ぜません。
- HY0020 → マウスは既定で最低 2 ms / byte の間隔を入れます。設定は `CONFIG_MOUSE_UART_BYTE_GAP_US` です。
- UART の DMA 受信バッファは各 128 byte です。nRF52832 の UARTE の 255 byte 上限をビルド時に確認します。
- マウス → PC は 16 KiB のリングバッファから MTU に合わせて通知します。MTU は 247 を推奨します。
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

Status は UART 破棄バイト数、UART エラー数、キュー満杯回数、BLE 送信エラー数の順です。

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
