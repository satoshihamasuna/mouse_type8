# 迷路探索シミュレータ

`search_simulator.py` は、マイコン側の探索処理を動作させずに検証するためのツールです。

## 再現する処理

- `wall_class` の `UNKNOWN` / `NOWALL` / `WALL` と仮想壁
- `add_pillar_walls`
- `add_dead_end_walls`
- `add_explored_branch_walls`
- `make_map_queue` / `make_map_queue_zenmen`
- `adachi::get_next_dir`
- マイコンと同じ更新順序（仮想壁更新 → マップ作成 → 移動実行）

テキスト形式の迷路と、マイコンが出力する1024バイトの壁スナップショットを読み込めます。

## 基本実行

リポジトリのルートから実行します。

```powershell
python tools/search_simulator.py tools/maze_data/MM2011MM.txt
```

仮想壁を無効にする場合：

```powershell
python tools/search_simulator.py tools/maze_data/MM2011MM.txt --virtual off
```

## 探索モード

`--map-mode goal` はゴール到達を優先します。`--map-mode full` は全面探索用のマップを使用します。

通常の枝閉鎖は、枝の壁・区画が観測済みであることを要求します。

```powershell
python tools/search_simulator.py tools/maze_data/MM2011MM.txt `
  --map-mode full --branch-mode observed
```

PRUNE形式では、UNKNOWNを開放と仮定して単一入口の枝を閉じます。
全区画を実際に踏む保証はなく、主に復路短縮の検証用です。

```powershell
python tools/search_simulator.py tools/maze_data/MM2011MM.txt `
  --map-mode full --branch-mode unknown_open --max-steps 4096
```

## 仮想壁の仕様

- 柱則：3方向の壁が確定した柱の残り1方向を閉じる
- 袋小路：3方向が閉じた区画の入口を閉じる
- 枝閉鎖：入口が1本で、枝側にゴール・スタート・現在地がない枝を閉じる
- 通常モードではUNKNOWNを未確定として枝閉鎖しない
- PRUNEモードではUNKNOWNを開放として枝閉鎖する
- スタート、現在地、ゴール領域は保護する

## 連続往復の比較

往路終了時の既知壁と仮想壁を復路へ引き継いで比較します。

```powershell
python tools/compare_virtual_walls.py --max-steps 4096 `
  --return-map-mode full tools/maze_data/MM2011MM.txt
```

比較対象は `full_observed` と `full_prune` です。`goal+fallback` は全面マップが通常マップへフォールバックした後にゴールへ到達したことを示します。

## 診断

```powershell
python tools/search_simulator.py tools/maze_data/logs_maze.bin --compare
python tools/search_simulator.py tools/maze_data/logs_maze.bin `
  --audit-contexts --compare --features branch
```

テストは次で実行します。

```powershell
python -m unittest tools/test_search_simulator.py
```
