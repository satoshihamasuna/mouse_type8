# AGENTS.md

## 対象と基本方針

- このファイルはリポジトリ全体で作業するエージェント向けの指針です。
- ユーザーへの説明と追加ドキュメントは、原則として日本語で記述してください。
- 変更前に関連する実装と README を確認し、依頼に必要な範囲に変更を絞ってください。既存の作業差分や測定データを保持してください。
- 既存ファイルの命名、インデント、改行に合わせ、無関係な整形やリネームを避けてください。

## プロジェクト概要と配置

STM32U535CEU6（Cortex-M33）向けのマイクロマウス用ファームウェアです。STM32CubeIDE for Visual Studio Code の CMake/Ninja 構成を使用します。
ファームウェアは C11 / C++14 で、C++ は例外・RTTI を無効にしています。ホスト側ツールは Python です。

| 配置 | 主な責務 |
| --- | --- |
| `Core/` | CubeMX 生成の初期化、割り込みハンドラ、起動コード |
| `Drivers/` | STM32 HAL、CMSIS |
| `Peripheral/` | モータ、エンコーダ、IMU、IR センサ、通信などの入出力 |
| `Module/` | 制御割り込み、機体状態、ログ、Flash 管理 |
| `Task/` | モーション生成、走行制御、センサ処理、壁制御 |
| `Subsys/` | 探索、迷路、仮想壁、地図・経路生成 |
| `System/` | モード選択、シェル、デバッグ・デモ走行 |
| `Component/` | コントローラ、数学処理、キューなどの共通部品 |
| `Params/` | 機体別設定、走行・ターンパラメータ、センサ校正 |
| `tools/` | Python のシリアル通信、GUI、シミュレータ、解析ツール |
| `tools/tests/` | Python テストと実装本体を使ったホスト C++ テスト |

起動経路は `Core/Src/main.c` → `CPP_Main()`（`System/Src/system_util.cpp`）→ `Mode::Select_Mode()` です。
制御処理は `Module/Src/interrupt.cpp` の ADC1 完了コールバックから `preprocess()` → `main()` → `postprocess()` の順に呼ばれます。

## 機体 A / B の扱い

- 機体は `CMakePresets.json` の preset と、`CMakeLists.txt` の `MOUSE_TARGET` で選択します。ソースに `MOUSE_A` / `MOUSE_B` を直接追加して切り替えないでください。
- 対応関係は `Params/mouse_config.h`、走行設定は `Params/run_param.h`、センサ校正は `Params/sens_table.h` を起点に確認してください。

| 機体 | Debug preset | Release preset | ターン設定 | IMU |
| --- | --- | --- | --- | --- |
| A | `Debug-MouseA` | `Release-MouseA` | `Params/Param_A/` | LSM6DSV16X |
| B | `Debug-MouseB` | `Release-MouseB` | `Params/Param_B/` | LSM6DSV80X |

- 互換用の `Debug` / `Release` preset は機体 A です。機体を明示した preset を使用してください。
- 校正値、制御ゲイン、フィードフォワード係数は機体ごとに扱い、一方の測定値を他方へ無条件で適用しないでください。

## ビルド

コマンドはリポジトリのルートから PowerShell で実行します。`cube-cmake`、Ninja、Arm GNU Toolchain が必要です。詳細は [README.md](README.md) を参照してください。

```powershell
# 機体 A
cube-cmake --preset Debug-MouseA
cube-cmake --build --preset Debug-MouseA

# 機体 B
cube-cmake --preset Debug-MouseB
cube-cmake --build --preset Debug-MouseB
```

- 共有ファームウェアを変更した場合は、A / B の両方をビルドしてください。機体専用の変更では対象機体を確認してください。
- 最適化やビルド設定に関わる変更では、対応する `Release-MouseA` / `Release-MouseB` も確認してください。
- 成果物は `build/<preset>/mouse_type8.elf`、`mouse_type8.map` です。補完用の `compile_commands.json` も各ビルドディレクトリに生成されます。
- ツールが利用できない場合は、未実行の検証と不足するツールを報告してください。

## 生成コードとソース追加

- `Core/` を変更する際は CubeMX の `USER CODE BEGIN` / `USER CODE END` 区間を優先し、再生成での保持を考慮してください。
- ペリフェラル設定を変更する場合は `mouse_type8.ioc` と生成コードの整合を確認し、再生成後に Configure と差分確認を行ってください。
- `cmake/vscode_generated.cmake` は拡張機能が管理する生成ファイルです。独自のソース・include・オプションの追加は、原則としてルート `CMakeLists.txt` の `User defined` セクションを使用してください。ソースの二重登録を避けてください。
- `Drivers/`、`Component/ntshell-v0.3.1/`、`Component/codegen/` の変更は必要な範囲に限定し、既存の著作権・ライセンス表記を保持してください。

## 制御コードと通信の変更

- 距離の mm / m、角度の deg / rad、速度・加速度の単位、左右の符号、時間刻みを呼び出し元まで確認してください。制御処理には 1 ms を前提とする積分があります。
- 制御割り込みへ待機、長時間の通信・ログ出力、動的メモリ確保などを新たに追加する際は、周期への影響を検討してください。
- 直進壁制御は `Task/Src/zirconia_wall_control.cpp`、その接続は `Task/Src/ir_sens_task.cpp`、設定は `Params/zirconia_wall_param.h` を参照してください。壁切れ検出、迷路用壁判定、斜め制御との役割の違いを保ってください。
- シェルやログ形式を変更する場合は、`System/Src/myshell.cpp`、`Module/Src/log_data.cpp` と対応する Python 側の送受信・解析を併せて確認してください。CSV の列順・単位、バイナリ形式、整数スケーリングの互換性を考慮してください。
- 調整値や解析結果を更新するときは、対象機体、入力ログ、実行条件、単位、判断根拠を残してください。

## ホスト側の検証

手順の詳細は [tools/tests/README.md](tools/tests/README.md)、ツールと依存パッケージは [tools/README.md](tools/README.md) を参照してください。

```powershell
# unittest 形式のテスト
python -m unittest discover -s tools/tests -v

# 側壁判定・IR タスク・直進壁制御を変更した場合の対象テスト
python -m unittest discover -s tools/tests -p test_side_wall_stability.py -v

# myshell GUI のパーサ等を変更した場合（pytest が必要）
python -m pytest tools/tests/test_myshell_gui.py -q
```

- `test_myshell_gui.py` は pytest 形式のため、`unittest discover` だけでは実行されません。
- 側壁関連テストは実際の C++ 実装を機体 A / B 向けにホストでコンパイルします。`CXX`、PATH 上の `c++` / `g++` / `clang++`、ローカル Zig の順に探索し、コンパイラがなければスキップされます。スキップを成功と扱わないでください。
- 変更に対応するテストを選び、不具合修正では再現ケースを追加・更新してください。ドキュメントのみの変更では、記載内容と参照先の確認を行ってください。
- ホストテストとビルドで確認した事項、実機で確認した事項を区別してください。書き込みや走行は、依頼された実機作業の範囲で行ってください。

## データと作業報告

- `build/` はビルド生成物、`tools/logs/` は Git 管理対象外の取得ログ・解析結果です。通常のソース変更へ生成物を混在させないでください。
- `tools/turn_analysis/`、`tools/maze_data/`、`output/`、`tmp/` には保存済みの測定資料や成果物があります。名前だけで不要と判断して削除・上書きしないでください。
- 操作方法・コマンド・設定を変更した場合は、関連する README も更新してください。シリアル操作は [tools/myshell_README.md](tools/myshell_README.md)、解析は [tools/analysis/README.md](tools/analysis/README.md) を参照してください。
- 完了報告には、変更内容、対象機体、実施した検証と結果、未確認事項を簡潔に記載してください。

## プロジェクト専用スキル

繰り返す作業の手順は `.agents/skills/` に配置しています。依頼に対応するスキルを使用してください。

| スキル | 用途 |
| --- | --- |
| [mouse-build-check](.agents/skills/mouse-build-check/SKILL.md) | 機体 A / B のビルド、変更に対応するホストテスト、ビルド不具合の切り分け |
| [mouse-wall-control](.agents/skills/mouse-wall-control/SKILL.md) | 直進側壁制御、IR 観測の採用判定、状態遷移の変更と回帰検証 |
| [mouse-log-analysis](.agents/skills/mouse-log-analysis/SKILL.md) | 保存済みログの FF 解析、動画からの Lstart/Lend 測定、採用根拠の整理 |

明示的に指定する場合は、依頼に `$mouse-build-check` などのスキル名を含めてください。
