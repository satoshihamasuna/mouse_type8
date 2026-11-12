# 既存ターン調整GUIの操作と参照先

コマンドはリポジトリのルートから実行する。以下は起動例で、この資料を読むだけで実機へ送信するものではない。

## 読む実装・説明

| 内容 | 参照先 |
| --- | --- |
| 単体GUI、単位、前後の走行区間、ログ保存 | [tools/README.md](../../../../tools/README.md)の「myshellデバッグ・パラメータ調整GUI」 |
| 単体GUIのコマンド生成・実機値取得 | [myshell_debug.py](../../../../tools/myshell_debug.py)の`_set_command`、`_query_device_settings`、`_log_binary_worker` |
| 連続パターンの編集・JSON・送信 | [myshell_turnpattern.py](../../../../tools/myshell_turnpattern.py)の`_apply_worker`、`_log_worker` |
| 初期値、引数、RAM設定、実行の実体 | [myshell.cpp](../../../../System/Src/myshell.cpp)の`shell_debug_turn_get`、`shell_debug_turn_command`、`shell_turnpattern_execute` |
| BLE中継・GUIランチャー | [BLE README](../../../../tools/ble/README.md)の「既存の GUI を BLE で使う」、[mouse_gui_ble.py](../../../../tools/ble/mouse_gui_ble.py) |
| 連続パターンGUIの概要 | [README.md](../../../../README.md)の「turnpattern GUI」 |

## BLEと有線の起動

BLEはPython 3.10以上、Tkinter、Bleak/pyserialが使えるPythonを選ぶ。既存の導入済み環境を先に確認する。
Windowsの導入場所が以下と異なる場合は`$mousePython`だけを実際の環境へ合わせる。
バージョン別の`.tools/ble_python*`依存パッケージはランチャーが参照する。

ターミナル1で中継を起動する。

```powershell
$mousePython = Join-Path $env:LOCALAPPDATA 'Programs\Python\Python311\python.exe'
& $mousePython tools/ble/mouse_ble_gateway.py
```

`Ready: socket://127.0.0.1:8765`を確認して、ターミナル2で単体GUIを起動する。

```powershell
$mousePython = Join-Path $env:LOCALAPPDATA 'Programs\Python\Python311\python.exe'
& $mousePython tools/ble/mouse_gui_ble.py --gui debug
```

GUIのPortで`BLE`を選ぶ。中継のクライアントは1台なので、探索GUI・専用シェル・別の接続が残っていれば切り替える。
GUIを切断すると中継も終了するため、再接続では中継を起動し直す。
連続パターンGUIは同じランチャーの`--gui turnpattern`。ポート番号を変える場合は中継・ランチャー双方へ同じ`--port`を渡す。
有線なら`python tools/myshell_debug.py`または`python tools/myshell_turnpattern.py`を使い、実際のCOMとUSART設定を照合する。

## 単体の500調整

1. `Show device values`でshell準備・電圧・実機値を確認する。ターン名ごとに`Turn preset speed`を500へ合わせ、実機の`preset`と速度を読み返す。
2. 半径、Lstart/Lend、Degree correction、前走加速度、吸引、必要なPID/FFを編集する。GUIは表示単位へ変換するので、シェル応答の整数スケーリングを画面値へ二重に掛けない。
3. `Apply`は設定変更、`Apply and execute`は設定後の走行。実行は前加速と後減速も含む。直列の連続試験では各ターンの入出力軸を確認する。
4. 実行後の`Get log CSV`で`tools/logs/*_myshell_debug_log_<motion>.csv`と`.settings.json`を取得する。
   実機から取得したPID/FF・吸引・preset・回数・元の応答を確認する。設定照会失敗でもCSV保存が進むのでJSONの`error`を確認する。

GUI・ファームウェアの現実装にはdegree correction、pre_accel、FF、countなどの引数がある。
READMEの短い`debug turn ... set`例を手書きで流用する前に、`_set_command`とファームウェアの引数仕様を照合する。
`preset`と`reset`はRAMを初期値へ戻す操作なので、採用候補のログ・設定を保存してから使う。

| ターン名（右 / 左） | 入力 → 出力 |
| --- | --- |
| `long_r90` / `long_l90`、`long_r180` / `long_l180` | 直進 → 直進 |
| `in_r45` / `in_l45`、`in_r135` / `in_l135` | 直進 → 斜め |
| `out_r45` / `out_l45`、`out_r135` / `out_l135` | 斜め → 直進 |
| `r_v90` / `l_v90` | 斜め → 斜め |

GUIの`Turn count`を増やすことは、任意の異なるターン列を組むことと同じではない。
混在列はturnpatternの接続検証を使い、単体のRAM編集値が連続実行で読み直される点にも注意する。

## ログと反映の区別

- 単体はCSVと同名`.settings.json`を取得する。turnpatternのパターンJSONとCSVを、単体の実機パラメータ付きJSONと同一視しない。
- `debug/turnpattern`に探索GUIと同じ`wall_ekf`選択・自動適用・状態保存があるとは限らない。モードと壁切れ方式は実機応答で記録する。
- 保存ログの周期はロギング実装と設定から確認する。BLE探索CSVの`period_ms=5`をデバッグ保存ログへ流用しない。
- ゼロ設定を前提に専用解析ツールでLstart/Lendを逆算する場合は、[mouse-log-analysisの測定条件](../../mouse-log-analysis/references/turn-lengths.md)を使う。
  特にLstart=Lend=0、動画・ログ対応、対応motion、スロー倍率、品質条件を確認し、未対応の左斜め系を単純に名前置換しない。
  現在の非ゼロ設定での比較調整と、この逆算の入力条件は区別する。全体の進め方は[turn_500の調整方針](turn500-workflow.md)を参照する。
- 最短への反映は機体別`Params/Param_A/turn_500.h`または`Param_B/turn_500.h`、選択は`run_config_plain500`と`mode_500`を確認する。

GUI・送受信を修正した場合は変更に応じて`test_myshell_debug_session.py`、`test_myshell_gui.py`（pytest）、
BLEランチャーの`test_mouse_ble_transport.py`を選ぶ。ファームウェアも変更した場合はmouse-build-checkへ進む。
