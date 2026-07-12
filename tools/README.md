# ツール

シリアルログの取得、迷路表示、ログ解析などに使用するPythonツールです。

以下のコマンドはリポジトリのルートディレクトリで実行してください。

## セットアップ

ツールが使用するPythonパッケージをインストールします。

```powershell
pip install pyserial pandas numpy matplotlib
```

## myshell GUI

myshellの一般的な操作を行うGUIを起動します。

```powershell
python tools\myshell_gui.py
```

GUIからコマンドを送信できるほか、`disp maze_bin` の受信結果を迷路キャンバスへ表示し、
`disp log_bin` の受信結果をCSVとして保存できます。

詳しいコマンドと通信プロトコルについては、次のファイルを参照してください。

```text
tools/myshell_README.md
```

## myshellデバッグ・パラメータ調整GUI

myshellを経由して、直進・斜め走行の走行パラメータとPIDゲインを調整し、実走行できます。

入力画面は「動作固有パラメータ」「PIDゲイン」「吸引設定」に分かれています。Motionを変更すると、その動作で使用する項目だけが表示されます。
吸引設定は動作パラメータから独立しており、Motion・ターン種別・速度プリセットを変更してもチェック状態と吸引値を保持します。

```powershell
python tools\myshell_debug.py
```

GUIが使用するファームウェア側のコマンドは次のとおりです。

```text
debug straight show
debug straight set distance acc max_velo end_velo sp_kp sp_ki sp_kd om_kp om_ki om_kd suction_enable suction_duty
debug straight exe
debug diagonal show
debug diagonal set distance acc max_velo end_velo sp_kp sp_ki sp_kd om_kp om_ki om_kd suction_enable suction_duty
debug diagonal exe
debug turn type show
debug turn type reset
debug turn type preset speed
debug turn type set velo r_min Lstart Lend degree sp_kp sp_ki sp_kd om_kp om_ki om_kd suction_enable suction_duty
debug turn type exe
```

ターンの `type` には、次の14種類を指定できます。

```text
long_r90, long_l90, long_r180, long_l180
in_r45, in_l45, out_r45, out_l45
in_r135, in_l135, out_r135, out_l135
r_v90, l_v90
```

`reset` は選択したターンを、現在選択されている速度プリセットの初期値へ戻します。

`preset` では、`Params` に定義された速度別ターンテーブルを読み込みます。指定可能な速度は次のとおりです。

```text
300, 500, 700, 1000, 1200, 1400, 1500, 1600, 1800, 2000
```

GUIの `Turn preset speed` を変更すると、マイコンから該当速度の値を読み込み、パラメータ入力欄へ反映します。

ターンを実行すると、`debug.cpp` の確認走行と同様に前加速区間と後減速区間も走行します。

| ターン種別 | 前加速 | 後減速 |
| --- | --- | --- |
| `long_r90`, `long_l90` | 直進2区画 | 直進2区画 |
| `long_r180`, `long_l180` | 直進1区画 | 直進1区画 |
| `in_r45`, `in_l45`, `in_r135`, `in_l135` | 直進1区画 | 斜め2区画 |
| `out_r45`, `out_l45`, `out_r135`, `out_l135` | 斜め2区画 | 直進1区画 |
| `r_v90`, `l_v90` | 斜め2区画 | 斜め1区画 |

前加速の終端速度と後減速の開始速度には、選択したターンの `velo` が使用されます。

`suction_enable` は吸引なしが `0`、吸引ありが `1` です。`suction_duty` は `0～990` の範囲で指定します。
吸引は走行直前に徐々に立ち上がり、走行終了時の `Motion_end()` で停止します。

### Pivot turn／Search turn

```text
debug pivot_turn right|left show|reset|set|exe
debug pivot_turn right|left set degree rad_acc rad_velo sp_kp sp_ki sp_kd om_kp om_ki om_kd suction_enable suction_duty

debug search_turn right|left show|reset|set|exe
debug search_turn right|left set velo r_min Lstart Lend degree sp_kp sp_ki sp_kd om_kp om_ki om_kd suction_enable suction_duty turn_count
```

`pivot_turn` の角度は度、角加速度と角速度はそれぞれrad/s²、rad/sで指定します。
`search_turn` の実行時は、探索スラロームの前後に45 mmの直進加速・減速区間が入ります。
`turn_count` は1～100で指定します。前加速と後減速はそれぞれ1回だけ実行し、その間で探索ターンを指定回数繰り返します。
右旋回の角度・角加速度・角速度・旋回半径は負、左旋回は正で指定します。

### デバッグログのCSV保存

`Get log CSV` を押すと `disp log_bin` を送信し、バイナリログを受信して次のディレクトリへ保存します。

```text
tools/logs/YYYYMMDD_HHMMSS_myshell_debug_log_動作名.csv
```

動作名には、最後に `Apply and execute` で実行した内容が入ります。

```text
20260712_153000_myshell_debug_log_straight.csv
20260712_153100_myshell_debug_log_long_r90.csv
20260712_153200_myshell_debug_log_pivot_turn_right.csv
20260712_153300_myshell_debug_log_search_turn_left.csv
```

GUI起動後にまだ動作を実行していない場合は、動作名が `unknown` になります。

CSVの列名はファームウェアが送信する `HEADER` を使用します。ログフレームは半精度浮動小数点から通常の数値へ変換して保存されます。

debug走行のログは、吸引の立ち上げ完了直後から開始します。ターンでは前加速・ターン本体・後減速がログに含まれ、吸引の立ち上げ区間は含まれません。

`ログ初期化` ボタンを押すと、確認後に次のコマンドを送信します。

```text
log init
```

このコマンドはログ記録を停止し、ログ件数とマイコン上のログデータをすべてゼロクリアします。

### exe実行時のSerial取り外し

`exe` を送信した後、GUIは完了応答を待ちません。前センサ待機に入ったらSerialを物理的に取り外せます。
前センサが反応してから動作が完了するまで、ファームウェアは通常の完了メッセージをSerialへ送信しません。

### パラメータ送信速度

コマンドは受信の確実性を優先して1文字ずつ送信します。既定の `Char delay` は `0.005` 秒です。
以前の `0.08` 秒より約16倍高速で、マイコン側の受信処理にも文字間隔を確保できます。
文字欠けが発生する環境では `Char delay` を `0.01`、`0.02` の順に大きくしてください。

ターンの一括設定コマンドに対応するため、マイコン側NTShellのコマンド最大長は192文字です。

`exe` を送信すると前センサの反応待ちになり、前センサをかざすと走行を開始します。
設定したパラメータはRAM上に保持され、マイコンをリセットすると初期値へ戻ります。

## バイナリログの受信

STM32からバイナリ形式のログを受信し、`tools/logs/` 以下へCSVとして保存します。

```powershell
python tools\Serial_binary.py
```

出力例:

```text
tools/logs/20260708_2340_type8a_MPQ.csv
```

`tools/logs/` はgitの管理対象外であるため、生成されたCSVファイルはpushされません。

## ログ解析

### 目標値・測定値・位置軌跡の比較

`tools/logs/` 内の最新CSVを解析します。

```powershell
python tools\log_compare.py
```

CSVを指定する場合:

```powershell
python tools\log_compare.py tools\logs\20260712_141937_myshell_debug_log_long_r90.csv
```

画面を開かずPNGへ保存する場合:

```powershell
python tools\log_compare.py --no-show --save tools\logs\log_compare.png
```

次の目標値・測定値・誤差を時系列で比較します。

- 速度
- 角速度
- 走行距離
- 角度
- 加速度
- 横位置

さらに速度・角速度をログ周期で積分し、目標位置と測定位置を算出します。算出した2次元軌跡を比較し、ログに保存された `turn_x / turn_y` も破線で重ねます。各比較グラフにはRMSEも表示されます。

次のNotebookを開きます。

```text
tools/Serial_indicate.ipynb
```

Notebookは `tools/logs/` 内の最新CSVを自動的に読み込みます。
CSVに次の列が含まれている場合は、PIDゲインも表示します。

```text
speed_ctrl.Kp, speed_ctrl.Ki, speed_ctrl.Kd
omega_ctrl.Kp, omega_ctrl.Ki, omega_ctrl.Kd
```

## 迷路ビューア

myshellから受信した迷路データを表示します。

内部で使用するSTM32側のコマンド:

```text
disp maze
```

Python側のコマンド:

```powershell
python tools\Serial_maze_view.py --port COM8 --baud 115200
```

シリアル受信が不安定な場合は、文字の送信間隔を長くします。

```powershell
python tools\Serial_maze_view.py --port COM8 --baud 115200 --char-delay 0.05
```

受信したシリアルテキストをデバッグ表示する場合:

```powershell
python tools\Serial_maze_view.py --port COM8 --baud 115200 --debug
```

迷路画像を保存する場合:

```powershell
python tools\Serial_maze_view.py --port COM8 --baud 115200 --save tools\logs\maze.png
```

シリアル通信の代わりに、保存済みのASCII迷路を読み込む場合:

```powershell
python tools\Serial_maze_view.py --file tools\logs\maze_sample.txt --width 32 --height 32
```

## 迷路エディタ／ビューア本体

`tools/maze.py` には、`Serial_maze_view.py` が使用する `Maze` クラスが定義されています。
直接実行すると、簡易的な対話式迷路ビューア／エディタを起動できます。

```powershell
python tools\maze.py
```
