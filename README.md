# mouse_type8

STM32U535CEU6 を対象にしたマイクロマウス用ファームウェアです。STM32CubeIDE for Visual Studio Code で生成された CMake/Ninja 構成を使い、アプリケーション層は C/C++ で実装されています。

## 開発環境

- VS Code
- STM32CubeIDE for Visual Studio Code
- CMake Tools
- Ninja
- Arm GNU Toolchain (`arm-none-eabi-gcc`, `arm-none-eabi-g++`)

このプロジェクトでは `.vscode/settings.json` で CMake 実行コマンドに `cube-cmake`、生成器に `Ninja` を指定しています。

## プロジェクト構成

| ディレクトリ / ファイル | 内容 |
| --- | --- |
| `Core/` | STM32CubeMX が生成する初期化コード。`main.c`、割り込み、GPIO、ADC、SPI、TIM、USART など。 |
| `Drivers/` | STM32 HAL Driver と CMSIS。 |
| `Peripheral/` | モータ、エンコーダ、IMU、赤外線センサ、バッテリ、通信、Flash などの低レベル入出力。 |
| `Module/` | 割り込み処理、ログ、Flash 管理などの共通モジュール。 |
| `Task/` | 走行制御、姿勢制御、センサ処理、モーション生成などのタスク層。 |
| `Subsys/` | 探索、地図生成、経路生成、壁情報管理などの迷路処理。 |
| `System/` | モード選択、デバッグ、デモ、シェル、システムユーティリティ。 |
| `Component/` | コントローラ、キュー、経路追従、数学ユーティリティ、ntshell などの汎用部品。 |
| `Params/` | 走行パラメータ、ターンテーブル、センサ閾値など。 |
| `cmake/` | STM32CubeIDE for VS Code が生成した CMake 設定とツールチェーン設定。 |
| `CMakeLists.txt` | プロジェクト本体の CMake エントリ。 |
| `CMakePresets.json` | MOUSE_A / MOUSE_B、それぞれの `Debug` / `Release` CMake preset。 |
| `mouse_type8.ioc` | STM32CubeMX の設定ファイル。 |
| `open-ioc.cmd` | 同じフォルダ内の `.ioc` ファイルを開くための Windows 用スクリプト。 |
| `STM32U535CEUX_FLASH.ld` | Flash 用リンカスクリプト。 |
| `STM32U535CEUX_RAM.ld` | RAM 用リンカスクリプト。 |

## 実行の流れ

1. `Core/Src/main.c` で HAL と各ペリフェラルを初期化します。
2. `CPP_Main()` を呼び出し、C++ 側の初期化へ入ります。
3. `System/Src/mode.cpp` の `Mode::Select_Mode()` でエンコーダとボタンによるモード選択を行います。
4. 選択したモードに応じて、インターフェース確認、デモ走行、デバッグ走行、シェル実行などを呼び出します。

## VS Code コマンド集

### CMake preset の選択

コマンドパレットを開きます。

```text
Ctrl + Shift + P
```

以下を実行します。

```text
CMake: Select Configure Preset
```

機体とビルド種別に合わせて、以下の preset を選択します。

| Preset | 機体 | パラメータ | IMU |
| --- | --- | --- | --- |
| `Debug-MouseA` | MOUSE_A | Param_A | LSM6DSV16X |
| `Debug-MouseB` | MOUSE_B | Param_B | LSM6DSV80X |
| `Release-MouseA` | MOUSE_A | Param_A | LSM6DSV16X |
| `Release-MouseB` | MOUSE_B | Param_B | LSM6DSV80X |

従来の `Debug` / `Release` preset は互換性のためMOUSE_Aとして扱われます。MOUSE_Bの開発では、必ず名前に `MouseB` が付いたpresetを選択してください。

### Configure

```text
CMake: Configure
```

ターミナルからMOUSE_Aを設定する場合:

```powershell
cube-cmake --preset Debug-MouseA
```

MOUSE_Bを設定する場合:

```powershell
cube-cmake --preset Debug-MouseB
```

### Build

```text
CMake: Build
```

ターミナルからMOUSE_Aをビルドする場合:

```powershell
cube-cmake --build --preset Debug-MouseA
```

MOUSE_Bをビルドする場合:

```powershell
cube-cmake --build --preset Debug-MouseB
```

Debugビルドの成果物は機体別のディレクトリに出力されます。

```text
build/Debug-MouseA/mouse_type8.elf
build/Debug-MouseA/mouse_type8.map
build/Debug-MouseB/mouse_type8.elf
build/Debug-MouseB/mouse_type8.map
```

### Clean

VS Code から実行する場合:

```text
CMake: Clean
```

ターミナルから Ninja の clean を実行する場合:

```powershell
cube-cmake --build --preset Debug-MouseA --target clean
```

### Rebuild

```text
CMake: Clean Rebuild
```

ターミナルから実行する場合:

```powershell
cube-cmake --build --preset Debug-MouseA --clean-first
```

MOUSE_Bの場合は、presetを `Debug-MouseB` に置き換えます。

### compile_commands.json の更新

補完や clangd の参照情報を更新したい場合は Configure を実行します。

```text
CMake: Configure
```

Debug preset の場合、機体ごとに以下のファイルが使われます。

```text
build/Debug-MouseA/compile_commands.json
build/Debug-MouseB/compile_commands.json
```

### STM32CubeMX 設定を開く

Windows のターミナルから:

```powershell
.\open-ioc.cmd
```

または `mouse_type8.ioc` を VS Code / Explorer から開きます。

## よく使うターミナルコマンド

```powershell
# MOUSE_A Debug configure
cube-cmake --preset Debug-MouseA

# MOUSE_A Debug clean build
cube-cmake --build --preset Debug-MouseA --clean-first

# MOUSE_B Debug configure
cube-cmake --preset Debug-MouseB

# MOUSE_B Debug clean build
cube-cmake --build --preset Debug-MouseB --clean-first

# MOUSE_A Release build
cube-cmake --preset Release-MouseA
cube-cmake --build --preset Release-MouseA

# MOUSE_B Release build
cube-cmake --preset Release-MouseB
cube-cmake --build --preset Release-MouseB

# .ioc を開く
.\open-ioc.cmd
```

## 注意点

- `cmake/vscode_generated.cmake` は STM32CubeIDE for VS Code が管理する生成ファイルです。ソース追加や CubeMX 設定変更後に再生成される可能性があります。
- ユーザー定義の CMake 設定を追加する場合は、まず `CMakeLists.txt` の `User defined` セクションを使います。
- `.ioc` を変更した後は、生成されたソースや CMake 設定に差分が出るため、ビルド前に `CMake: Configure` を実行してください。
- 機体別のビルドディレクトリは `build/Debug-MouseA`、`build/Debug-MouseB`、`build/Release-MouseA`、`build/Release-MouseB` です。

### Flash / Write to MCU

Debugビルド後、ST-LINKを接続してSWD経由で書き込みます。ビルドした機体と書き込み対象を必ず一致させてください。

MOUSE_A:

```powershell
cube-cmake --preset Debug-MouseA
cube-cmake --build --preset Debug-MouseA --clean-first
cube programmer -c port=SWD -d .\build\Debug-MouseA\mouse_type8.elf -v -rst
```

MOUSE_B:

```powershell
cube-cmake --preset Debug-MouseB
cube-cmake --build --preset Debug-MouseB --clean-first
cube programmer -c port=SWD -d .\build\Debug-MouseB\mouse_type8.elf -v -rst
```

`cube programmer` が見つからない場合は、STM32CubeProgrammer の CLI を直接指定します:

```powershell
& "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" -c port=SWD -d .\build\Debug-MouseA\mouse_type8.elf -v -rst
```

MOUSE_Bの場合は、書き込みファイルを `.\build\Debug-MouseB\mouse_type8.elf` に変更します。

## turnpattern GUI

最大20個のターンを組み合わせて本体へ送信するGUIは、次のコマンドで起動します。

```powershell
python .\tools\myshell_turnpattern.py
```

初回のみ `pyserial` が必要です。

```powershell
pip install pyserial
```

GUIでは走行パターンをJSONファイルとして保存・再読込でき、ターン速度、前走加速度、後走減速度、吸引を設定できます。`Apply & exe` は設定とパターンを送信した後、前センサ待ちで走行を開始します。ログは `tools/logs`、保存したパターンは既定で `tools/turnpatterns` に出力されます。
