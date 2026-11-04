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

## Zirconia v3直進壁制御

`zirconia_wall_control.cpp` と実際のIRタスクを同時にコンパイルし、左右対称性、姿勢制動、角速度・角加速度上限、片側柱の拒否と反対壁の継続、全壁喪失、距離基準の確認、停止中の未採用、姿勢に整合するセンサ変化、推定位置同期、無効化・モード切替・ターン後の再開を検証します。ADC→機体別距離校正→制御出力までを確認するケースも含みます。斜め走行は従来の制御則を回帰確認します。

実機では、まず低速（0.3～0.5 m/s）の連続両壁・中央開始で、`ideal.rad_velo` と `ego.rad_velo` の追従を確認します。その後、左右への小さなオフセット、左連続壁＋右柱、左右反転、壁なしへの移行を別々に確認します。新ゲインはzirconia由来の初期値であり、type8での収束距離や安定性をホストテストは保証しません。高速走行の前に、持続的な飽和・蛇行・過度な壁への接近がないことを実測してください。今回、書き込みやモーター動作は実施しません。

既存CSVの列数は変更していません。直進中の `control_ir` は制御に使った横偏差[mm]（左寄りが正）、`control_ir_dot` は新制御の姿勢偏差[rad]（左向きが正）です。斜め中は両列とも従来の意味です。詳細な採用マスク・phase・transition・飽和状態は `IrSensTask::straight_wall_output` をデバッガで参照できます。マスクは1=左、2=右、3=両側、phaseは0=無効、1=推定、2=確認、3=壁から離れる補正のみ、4=追従です。

## HY0020 BLE 通信

`test_mouse_ble_transport.py` は Python 3.10 以上と Bleak / pyserial を使用します。
セットアップは [BLE 通信ツール](../ble/README.md#ホストテスト) を参照してください。

```powershell
$blePython = Join-Path $env:LOCALAPPDATA 'Programs\Python\Python312\python.exe'
& $blePython -m unittest discover -s tools/tests -p test_mouse_ble_transport.py -v
```

localhost と模擬 BLE 相手を使い、実機には接続しません。シェルの改行・文字間隔、
バイナリ保持、再送条件、GUI の自動検出と COM ポート互換性、独立起動時のローカル依存の読込みを確認します。
Python 3.10 未満または依存パッケージ不足ではスキップされるため、BLE の確認では
上記を対応する Python で実行し、スキップなしの成功を確認してください。

連続バイナリログは、次のテストも実行します。

```powershell
python -m unittest discover -s tools/tests -p test_mouse_ble_log.py -v
```

`log_stream.cpp` と `communicate.cpp` の実装本体をホストでコンパイルし、
要求の検証、周期、停止・リース切れ、時刻ラップ、混雑のリング保持と満杯時の見送り、
割り込みマスクの保持、シェル・通常バイナリ入力の維持を確認します。
DMAは開始時にコピーせず送信完了時にメモリを読む模擬を使い、送信中の領域の保護、
リング折り返しでの分割と順序、1フレーム1回のDMA起動、開始失敗時の再試行を確認します。
保存用 `LogBuffer` も1234件書き込んで最新400件の全55列が古い順に残ることと、リセットを検証します。
コンパイラの探索は側壁テストと共通で、見つからなければC++のケースはスキップします。
Python側は実装本体が出力したフレームを復号し、通知分割、CRC破損後の再同期、
CSV列・時刻・連番欠落を検証します。上記の `test_mouse_ble_transport.py` には、
模擬BLE接続による自動開始・継続要求・逐次CSV保存・終了時の停止も含まれます。
実機での持続転送速度、制御周期への影響、電波条件は別途確認が必要です。

## BLE実機探索GUI

Python 3.10以上・Tk・Bleakがある環境で、次を実行します。

```powershell
python -m unittest discover -s tools/tests -p test_ble_explore.py -v
```

実機には接続しません。A/B向けに実際の `System/Src/explore.cpp` と遠隔停止状態をコンパイルし、
Searchとモータの境界だけを模擬して、手かざし待ちの中断、往復、途中停止、電圧・引数・ログ配信の開始条件を確認します。
模擬BLEでは1接続でのログとASCII応答の分離、探索中の継続保存、停止応答を待つ切断、
待機中の配信停止と受信タイムアウト抑止、終了・開始拒否後の配信停止、同じ接続での再探索、
ログ未受信・旧ファームウェアへの走行要求抑止を確認します。
開始コマンドの文字列を配信停止中に送り、最初のログ受信後に許可・CRを送る順序と、
ログ未受信時に未実行の入力行を取り消すことも確認します。
非表示のTk画面で開始ボタンの有効化条件とライブ値も確認します。
このPCのPython 3.11では `.tools/ble_python311` の依存を使用します。

## 割り込み時間のログ

`python -m unittest discover -s tools/tests -p test_interrupt_timing.py -v` は実際の集計実装をホストで実行し、
サイクルからµsへの変換、32 bitカウンタの折り返し、前回完了値、最大値、リセット、カウンタ非対応時のNaNを確認します。
保存ログのHEADERによる旧51項目・新55項目のフレーム長選択も検証します。
BLEログテストは `L8S2` の55項目・60列CSVと旧 `L8S1` の51項目・56列CSVの列対応、形式変更時の終了も確認します。
DWTレジスタの実動作や、実機の割り込み時間はホストテストでは確認できません。
