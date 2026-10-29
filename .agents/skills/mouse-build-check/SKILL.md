---
name: mouse-build-check
description: "mouse_type8 の STM32 ファームウェアを機体 A/B 向けにビルドし、変更に対応するホストテストを選んで検証する。ビルド不具合や検証依頼に使う。ドキュメントだけの編集には不要。"
---

# Mouse ビルド・検証

対象機体、変更した層、Debug / Release の必要範囲を決め、実行した検証と未実行の検証を区別して報告する。
以降のコマンドはリポジトリルートで実行する。

## ビルド対象の選択

[CMakePresets.json](../../../CMakePresets.json) と [CMakeLists.txt](../../../CMakeLists.txt) を確認する。
`MOUSE_TARGET` は A / B のどちらか一つをコンパイル定義へ変換する。既存の `Debug` / `Release` は A を選ぶため、機体名付き preset を使う。

| 対象 | Debug | Release |
| --- | --- | --- |
| A | `Debug-MouseA` | `Release-MouseA` |
| B | `Debug-MouseB` | `Release-MouseB` |

共通ソースは A / B を確認し、機体専用パラメータだけなら対象機体を確認する。最適化・リンク・ビルド設定を変えた場合は対応する Release も確認する。

```powershell
Get-Command cube-cmake, ninja, arm-none-eabi-gcc, arm-none-eabi-g++

# B または Release の確認では preset 名を置き換える
cube-cmake --preset Debug-MouseA
cube-cmake --build --preset Debug-MouseA
```

Configure が成功してから Build を実行する。各 preset の成果物は `build/<preset>/mouse_type8.elf` と `mouse_type8.map`。
ツールが PATH にない場合は、既存の STM32 拡張機能の環境や指定済みのツールパスを調べる。ローカルの導入場所をスキルへ固定しない。

## ビルド不具合の切り分け

- Configure 失敗: [ツールチェーン設定](../../../cmake/gcc-arm-none-eabi.cmake)、preset、利用可能な Ninja / Arm GCC を確認する。
- ソース追加後のリンクエラー: ルート `CMakeLists.txt` と `cmake/vscode_generated.cmake` の登録を確認する。独自ソースはルートの `target_sources` へ追加し、重複登録を避ける。
- 一方の機体だけの失敗: `Params/mouse_config.h`、`run_param.h`、`sens_table.h` から機体別 include と定義を追う。A / B の両定義を有効にして回避しない。
- メモリやリンク領域の問題: 該当 preset の `.map` と使用するリンカスクリプトを確認する。リンク成功だけでスタック余裕を確認済みとしない。

## 変更に対応するホストテスト

詳細は [テスト手順](../../../tools/tests/README.md) と [Python ツールの依存関係](../../../tools/README.md) を読む。
下表のファイルを起点に、変更内容を直接確認できる既存ケースを選ぶ。

| 変更対象 | テスト |
| --- | --- |
| 直進側壁制御、ADC から距離校正、IR タスク | `test_side_wall_stability.py` |
| 探索シミュレータ、仮想壁、探索結果パーサ | `test_search_simulator.py`、`test_myshell_search.py` |
| オドメトリ、ターンシミュレータ | `test_odometry_compare.py`、`test_turn_simulator.py` |
| myshell GUI のコマンド生成・結果パーサ | `test_myshell_gui.py`（pytest 形式） |

```powershell
# 対象を絞る例
python -m unittest discover -s tools/tests -p test_side_wall_stability.py -v

# unittest 形式のテストをまとめて実行する場合
python -m unittest discover -s tools/tests -v

# GUI パーサの関数形式のテストは別途実行する
python -m pytest tools/tests/test_myshell_gui.py -q
```

`unittest discover` は pytest の関数形式のケースを実行しない。
側壁テストはホスト C++ コンパイラがなければスキップされる。スキップ件数と理由を結果へ残す。
探索シミュレータの Python テストは、ファームウェア側の探索 C++ を直接実行するテストとは区別する。

## 完了時

実行コマンド、対象機体と preset、成否、スキップ・未実行の理由を簡潔に示す。
ビルド・ホスト検証の結果と実機評価を区別し、ビルド依頼へ書き込みや走行を自動的に追加しない。
