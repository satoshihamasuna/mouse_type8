# Pythonテスト

リポジトリのルートディレクトリから、次のコマンドで全テストを実行します。

```powershell
python -m unittest discover -s tools/tests -v
```

側壁の空間安定判定と実際の IR センサ処理だけを検証する場合:

```powershell
python -m unittest discover -s tools/tests -p test_side_wall_stability.py -v
```

このテストは MouseA/B の両方をホスト向けにコンパイルします。`CXX`、PATH 上の `c++` / `g++` / `clang++`、または `build/ir-confidence-tools/ziglang/zig.exe` の順に使用し、コンパイラが見つからない場合はスキップします。Vehicle とハードウェア境界だけを stub 化し、安定判定・距離変換・壁制御の採用判定は実装本体を使用します。
