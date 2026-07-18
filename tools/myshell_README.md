# myshell シリアル操作ガイド

このメモは、STM32 側の `myshell` コマンドと、PC 側の Python GUI
`tools/myshell_gui.py` の使い方をまとめたものです。

## セットアップ

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
Baud: 115200
Line ending: CR
Char delay: 0.08 s
```

Windows で別の COM 番号が割り当てられている場合は、GUI の `Port` を変更してください。
今回の確認では `COM6` を使いました。

## GUI の起動

```powershell
python tools\myshell_gui.py
```

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
| `search replay reset\|keep plain\|acc start_x start_y dir goal_x goal_y goal_size goal\|full\|full_prune first\|second mask max_steps` | `Search`と同じ壁観測・マップ更新順で、モーションだけをno-opにして逐次再生します。 |
| `log mode0` | ログヘッダ出力を有効にします。 |
| `log mode1` | ログヘッダ出力を無効にします。 |
| `end exe` | myshell を終了します。 |

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
再生・一時停止・前後1ステップ・速度変更できます。

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
half_float data[51];
```

終了フレームは `magic = 0xFFFF` です。

## トラブルシュート

- GUI が timeout する場合は、`Timeout` を長めにするか、マウス側 myshell をリセットしてから再接続してください。
- コマンド文字が欠けて echo される場合は、`Char delay` を大きくしてください。
- `Upload maze_data` が途中で止まる場合は、マイコン側が `load maze_bin` の受信待ちになっていることがあります。その場合は一度リセットして myshell に入り直してください。

## 既存のコマンドラインツール

古いコマンドラインツールも残しています。

```powershell
python tools\Serial_maze_view.py --port COM8 --baud 115200 --binary
python tools\Serial_binary.py
```

手動確認は GUI、スクリプト化したい受信処理はコマンドラインツール、という使い分けが便利です。
