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
