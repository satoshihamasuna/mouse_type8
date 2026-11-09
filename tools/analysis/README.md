# 解析ツール

Pixel撮影タブの解析は`pixel_camera_tuning_report.py --trial PATH --stride 4`（速報）／`--stride 1`（全フレーム詳細）です。
録画・走行はせず、保存済みの動画・CSV・実機設定・プロファイルだけを使います。出力は試行内のquick/とfull/で区別し、既存出力を上書きしません。
速報は元のPTS・フレーム番号を保った約60fpsで、PROVISIONALとして扱います。全フレーム用キャッシュと混用しません。
停止確認は指令後の実測速度・角速度と、映像の停止後の安定性・必須区間の観測率を併用します。採用パラメータは自動更新しません。
UIの操作と保存先は[Pixel撮影・調整](../pixel_camera_README.md#調整guiで撮影と単体走行をまとめる)を参照してください。

実機用の通路内2状態EKFは`Task/Inc/straight_wall_ekf.h`です。横偏差・姿勢偏差の運動式を
共有し、取得済みCSVの間引き値ではなく各制御回の移動量を使います。
壁制御への接続と`legacy/shadow/control`の切替は[直進壁EKF](../myshell_README.md#直進壁ekf)を参照してください。
以下のPython解析結果と実機の走行性能は区別して評価します。

## 速度も状態に含めて経路を比較する逐次センサEKF

`sensor_path_ekf.py` は、位置・姿勢・前進速度・角速度・ジャイロバイアス・
前後加速度バイアス・現在の通路の未知中心位置を、8状態で逐次推定します。
既定のエンコーダ版では`ego.velo`を推定器へ渡しません。
積分済みの角度・座標、routeは両方式とも推定器へ渡しません。
IMUの前後加速度で予測し、右車輪・左車輪・ジャイロ・採用できる壁距離の順に観測更新します。
車輪速度の観測モデルは`v_R = v + tread*omega/2`、`v_L = v - tread*omega/2`です。

`--velocity-source estimated_velocity`で左右車輪の観測を記録推定速度`ego.velo`に置き換えます。
IMU・壁の逐次更新、8状態、運動予測は共通です。推定器本体はこのモードで車輪パルスと
`irq.period_us`を読みません。CLIは積分比較線の生成用に左右パルスも必要です。
直接車輪更新を除くので、左右差からの角速度情報も除かれます。

```powershell
python tools/analysis/sensor_path_ekf.py `
  --log tools/logs/20261106_100618_review/explore/telemetry.csv `
  --settings tools/logs/20261106_100618_review/explore/run_100635_157989.json `
  --output tools/logs/20261106_sensor_velocity_ekf/explore
```

推定速度を観測に使う場合:

```powershell
python tools/analysis/sensor_path_ekf.py `
  --log tools/logs/20261106_100618_review/explore/telemetry.csv `
  --settings tools/logs/20261106_100618_review/explore/run_100635_157989.json `
  --velocity-source estimated_velocity `
  --output tools/logs/estimated_velocity_trial/explore
```

`ego.velo`は現ソースでは車輪速度の20制御回平均と`0.010 * ego.accel`から作られます。
このため独立センサとは扱わず、観測モデルを`z_v = v + 0.010*b_accel + n_v`とし、
加速度バイアスと予測に共有する現時刻の加速度雑音相関を近似して更新します。
速度観測の独立雑音標準偏差は固定の試作値0.08 m/s、共有加速度込みの観測分散は
0.0068 (m/s)²です。移動平均の時間相関・遅れを完全に同定したモデルではありません。
相関雑音を持つモデルの一般形は[MathWorks公式解説](https://www.mathworks.com/help/control/ref/kalmanfilter.html)
も参照しています。

受信辞書を直接使う場合は`SensorPathEkf(velocity_source="estimated_velocity")`を生成します。
`replay(..., calibration_override=..., wall_holdout=...)`で共通の静止校正・壁除外区間を与え、
速度入力だけを変える比較もできます。除外区間の壁値は推定・採用判定に使いません。

確認済みの符号・校正を使う機体Aのみ対応します。異なる走行を含むCSVは、
通信休止・連番欠落で先に分割してください。指令列はCLIで共通走行区間を切り出すためだけに使います。
左右パルスは配信区間の累積ではなく1制御回分の値で、`irq.period_us`をその時間幅の近似に使います。
走行前の時刻計測リセットによる欠損は校正から除外し、走行中の欠損はエラーにします。
静止校正では、エラー処理による疑いのあるジャイロ・加速度の同時ゼロを除外します。
ログ上のジャイロは符号・スケール校正済み、加速度は20制御回の平均値で、どちらもセンサ由来です。

壁は直進・側壁30～60 mm・前壁112 mm超・6 mm以上の連続確認で採用します。
通路方向には初期北向きからの直交軸を仮定し、壁距離は姿勢偏差が小さいときの横距離で近似します。
片壁は45 mm基準、両壁は左右差を使います。通路の最初の観測で未知中心位置を現在姿勢と
相関を持たせて初期化するため、絶対XYの格子合わせや最初の壁観測での位置合わせは行いません。
旋回後は別の未知壁として扱い、同じ壁への再訪やループ閉じ込みは使いません。

`paths_overlay.png`で、記録速度の積分、車輪パルス＋IMUの積分、車輪＋IMUのEKF、
車輪＋IMU＋壁のEKFを同じMCU時間軸・始点で比較し、壁更新を採用した点も表示します。
推定速度版ではEKFの2本が推定速度＋IMU・推定速度＋IMU＋壁になります。
`sensor_signals.png`、`paths_and_states.csv`、`path_metrics.csv`、入力・設定・実装ハッシュを持つ
`report.json`も保存します。指標は推定経路同士の差であり、位置精度ではありません。

雑音設定は固定の試作値です。間引きで欠けたパルス、MCU時刻と制御積分の時間差、
加速度平均の遅れ・相関、横滑り・距離校正の影響は復元できません。
バイアス状態が時間や運動モデルの誤差を吸収する可能性もあり、推定終点が近づくだけでは
改善とは判断できません。実位置の正解値、機体B、実機周期内での動作は未検証です。

検証: `python -m unittest discover -s tools/tests -p test_sensor_path_ekf.py -v`。
一般的な速度補正付きEKFは[PythonRoboticsの著者解説](https://atsushisakai.github.io/PythonRobotics/modules/2_localization/extended_kalman_filter_localization_files/extended_kalman_filter_localization.html)、
差動二輪の速度関係は[WPILib公式解説](https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/differential-drive-kinematics.html)
を参照しています。本ツールの通路壁・校正モデルは保存ログ向けの独自近似です。

## 左右エンコーダ・IMU・壁を直接使う逐次2状態EKF

`encoder_wall_imu_ekf.py` は左右のエンコーダパルス、IMU角速度、壁距離を1行ずつ読み、
通路中心からの横偏差`e`と通路方向に対する角度`psi`を推定します。内部状態と共分散は2次元です。
`ego.velo`、`ego.radian`、`ego.turn_x/y`、4状態EKFの結果は入力に使いません。
IMUには実装の符号・スケール補正済みジャイロ値`ego.rad_velo`を使います。

```powershell
python tools/analysis/encoder_wall_imu_ekf.py `
  --log tools/logs/20261105_094850_965524_ble_explore/telemetry.csv `
  --settings tools/logs/20261105_094850_965524_ble_explore/run_094901_849463.json `
  --output tools/logs/20261105_094850_965524_ble_explore/encoder_wall_imu_ekf_trial
```

`--follow`で取得中CSVの完成した行に追従し、推定をCSVへ逐次保存します。
Ctrl+Cで処理済みの範囲を評価・描画します。`--max-rows N`も使用できます。
機体Aの符号と設定で検証したため、CLIは設定JSONが機体Aのときだけ実行します。
機体Bへ適用する前にエンコーダ・ジャイロ・壁距離校正を確認してください。

左右の車輪速度を前進正にそろえ、`v=(v_R+v_L)/2`、`omega_enc=(v_R-v_L)/tread`を求めます。
CSVの左パルスは前進時に負なので反転します。現在の設定は1パルス約0.27434 mm、車輪間隔28 mmです。
根拠は`Params/mouse_config.h`、`Peripheral/Src/encoder.c`、`Module/Src/interrupt.cpp`です。
差動二輪の車輪速度と機体速度の関係は
[WPILibの公式解説](https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/differential-drive-kinematics.html)
も参照してください。

**保存CSVの`sp_pulse`は1 ms制御1回分のパルス数で、5 ms配信区間の累積値ではありません。**
したがってCSVの時刻差でパルス数を割ると、速度を約1/5にする誤りになります。
1 msで速度へ換算して過去5配信点の平均を使います。欠けた4回分程度のパルスは復元できません。
位置・姿勢の積分時間にはMCU時刻差を使います。以前の局所座標から時間を復元した経路とは条件が異なります。

予測角速度は、走行前に零点を引いたジャイロと左右差による角速度を分散の逆数で統合します。
入力の速度・角速度の共分散をヤコビアンで2状態の共分散へ伝播します。
エンコーダのパルス量子化・スリップの仮定とジャイロ雑音下限を明示し、
大きな旋回速度の不一致はエンコーダ側の分散を増やします。この判断はIMUを優先する仮定です。
量子化分散はカウント両端の誤差から1/6パルス²とし、重なる移動平均でも保守的に元の分散を維持します。
これらは固定の試作値で、帰還点へ合わせて調整した値ではありません。
ジャイロ零点は動き出す前の最大500点から別途校正し、EKFの状態には追加しません。

```python
from encoder_wall_imu_ekf import EncoderWallImuEkf, fused_controls

filter = EncoderWallImuEkf()
u = fused_controls(v_right, v_left, gyro_minus_bias,
                   (variance_right, variance_left), variance_gyro, tread_m)
filter.predict_sensors(u["velocity"], u["omega"], dt_s, u["control_covariance"])
filter.observe_lateral(wall_lateral_m, wall_variance_m2)  # 採用できる壁があるときだけ
```

受信辞書を直接渡す場合は`StreamingEncoderFusion.update(sample)`を使います。
`EncoderWallImuEkf`単体では、実際に測定した左右車輪速度・時間幅を入力できます。
壁距離の選別、片壁/両壁からの横偏差、直進中の状態維持と旋回・停止後の再初期化は下記の逐次版と同じです。
壁距離で横偏差を更新し、時間方向の結合を通して角度も補正します。
横滑り・変動するジャイロバイアスを独立に推定する状態は含めていません。

比較は旋回予測入力を「IMUのみ」「ENC＋IMU」「ENCのみ」に変えた3条件です。
前進速度、壁採用条件、18 mmの評価区間は共通にし、1/3を丸ごと壁更新から除いて3分割評価します。
ENCのみ条件も評価をそろえるためIMUによる壁品質ゲートを残します。IMUを完全に除いた条件ではありません。

`fusion_states.csv`へ各条件の横偏差・角度・観測済みフラグ・標準偏差と外側の推定経路を保存します。
`sensor_fusion_states.png`、`sensor_fusion_paths.png`、`report.json`、`validation_blocks.json`も生成します。
現在の間引かれたパルス値では旋回速度の不確かさが大きく、ENC旋回の統合比率が非常に小さくなり得ます。
寄与を評価する追加ログとして、左右の累積カウントまたは配信区間全体のカウント差が有効です。
ホスト上の試作であり、実機の中心走行改善やSTM32の1 ms周期での動作を確認したものではありません。

テストは`python -m unittest discover -s tools/tests -p test_encoder_wall_imu_ekf.py -v`。

## 1サンプルずつ更新する壁EKF

`streaming_wall_ekf.py` はCSVを1行ずつ読み、現在と過去の入力だけで推定を更新します。
`StreamingWallEstimator.update(sample)` を受信処理から直接呼ぶこともできます。
BLE接続やファームウェアの変更は行いません。NumPy・pandas・Matplotlibを使用します。

```powershell
python tools/analysis/streaming_wall_ekf.py `
  --log tools/logs/20261105_083528_350069_ble_explore/telemetry.csv `
  --settings tools/logs/20261105_083528_350069_ble_explore/run_083539_607469.json `
  --output tools/logs/20261105_083528_350069_ble_explore/streaming_ekf_trial
```

取得中のCSVへ追従する場合は、同じコマンドに`--follow`を追加します。
完成した行だけ読み、追記途中の行は改行まで待ちます。Ctrl+Cで読み込みを終了し、
処理済みの範囲について評価・グラフを保存します。`--max-rows N`で処理する行数も指定できます。
新たなBLE接続は作らず、既存のログ取得プログラムが書くCSVを読みます。
途中の連番欠落や100 ms以上の時刻の飛びはエラーにします。単一の連続した配信を入力してください。

同じ受信データで次の推定を動かします。

- 全体位置の4状態EKF：東・北・姿勢・ジャイロバイアス。
- 通路内の2状態EKF：横偏差`e`と通路方向に対する姿勢偏差`psi`。左が正、内部はm・radです。
- 比較用の1状態KF：横偏差だけのランダムウォーク。角度の追加効果を比較します。

2状態の予測は`e_next = e + ds*sin(psi + delta_psi/2)`、
`psi_next = psi + delta_psi`で、`delta_psi`はジャイロの区間平均から静止時バイアスを引いた積分です。
壁距離は小さい姿勢偏差での横距離と近似し、`L = 45 mm - e`、`R = 45 mm + e`とします。
両壁時は`(R-L)/2`、片壁時は45 mm基準の横偏差を観測します。
側壁30–60 mm、前方112 mm超、距離変化の上限、連続3点・6 mmの確認で観測を選別します。
観測ノイズ等は固定の仮定値で、帰還点やrouteへ合わせた値ではありません。
センサの取付角・校正・横滑りの影響は未検証です。

この2状態は現在の通路軸を基準にした誤差状態です。旋回・停止中は横位置を評価せず、
次の直進で大きめの初期共分散から推定を再開します。連続直進では、
ログ内の局所角度・座標が区画境界でリセットされてもEKFを初期化しません。
`local_active`は直進候補、`local_observed`はその区間で壁観測を採用済みかを示します。
観測前の横偏差0・角度0は初期値であり、中心を観測できたことを示しません。
移動量や安定した壁がない場合、角度は十分に観測できません。標準偏差もCSVへ保存します。

```python
from streaming_wall_ekf import StreamingWallEstimator

estimator = StreamingWallEstimator()
for sample in received_samples:  # CSVと同じキーを持つ、時刻順の辞書
    estimate = estimator.update(sample)
    if estimate["started"] and estimate["local_active"]:
        lateral_mm = estimate["local.e_mm"]
        heading_deg = estimate["local.psi_deg"]
```

独立した2状態の数値処理だけなら`LateralHeadingEkf.predict(ds, delta_psi, dt)`と
`observe_lateral(e, variance)`を使用できます。ジャイロバイアスはこの2状態へ追加せず、
走行開始前の直近最大500点の静止候補から別途校正します。30点未満なら未校正を明示します。
全体位置の復元も未来側の補間を使わず、現在と過去21点の局所積分間隔から時間幅を推定します。
区間リセットの補間や配信時刻と制御周期の差の不確かさは残ります。
実機へ組み込む際は実際の周期とモーション状態を直接入力する必要があります。

検証では、移動量から逐次決める独立した18 mm区間の1/3を丸ごと観測更新から除外し、
除外する区間を3通りに変えて壁距離変化の予測誤差を集計します。境界サンプルも共有しません。
入力CSVの処理済み範囲が各再生で一致することをハッシュで確認します。
全体位置版は迷路軸との対応が確認できる区間だけ採点するため、2状態版と評価数が異なる場合があります。
後述のオフライン版とは前処理・観測区間が異なり、以前のRMSEと直接比較する指標ではありません。

出力は`streaming_states.csv`、`report.json`、`validation_blocks.json`、
`streaming_global.png`、`streaming_local.png`です。
推定器本体の履歴サイズは固定です。CLIのみ描画用の出力履歴を保持します。
実位置の正解値は未取得のため、壁距離との整合性・推定の改善と、実際の中心走行の改善を区別してください。

検証コマンドは`python -m unittest discover -s tools/tests -p test_streaming_wall_ekf.py -v`。
EKFの予測・線形化・観測更新の一般形は
[PythonRoboticsの著者による解説](https://atsushisakai.github.io/PythonRobotics/modules/2_localization/extended_kalman_filter_localization_files/extended_kalman_filter_localization.html)
も参照してください。本ツールの壁・通路モデルは上記の近似を独自に使用しています。

### 補正ごとの経路を比較する

`compare_ekf_paths.py` は上記の`streaming_states.csv`から、補正なし・ゼロ点補正のみ・
4状態EKF・2状態EKF＋外側の経路積分を比較します。

```powershell
python tools/analysis/compare_ekf_paths.py `
  --states tools/logs/20261105_083528_350069_ble_explore/streaming_ekf_trial/streaming_states.csv `
  --output tools/logs/20261105_083528_350069_ble_explore/path_correction_comparison
```

任意の`--route route.json`は表示用です。推定器・経路積分へ入力しません。
`paths_overlay.png`に全条件の重ね合わせと帰還点の拡大、`paths_separate.png`に
全条件を同じ縮尺で個別表示します。共通始点・初期北向き以外の位置・角度・縮尺合わせはしません。
`compared_paths.csv`と`report.json`へ座標・条件・ハッシュ・推定帰還点を保存します。

2状態EKF自体が推定するのは横偏差・姿勢偏差です。全体経路を描くため、
`LocalPosePath`が前進・旋回中の車輪/ジャイロの位置積分を別に保持します。
直進中で壁を採用済み、姿勢標準偏差3°以下・姿勢偏差15°以内のときに2状態出力を使います。
外側の姿勢から25°以内に直交する通路軸を対応付け、その軸と`psi`で進行方向を更新します。
連続して採用できる区間は`e`の差分を横移動、測定移動量と`psi`を前進に使います。
採用開始時の`e`は相対偏差の原点とし、絶対XYや区画中心へ位置を飛ばす処理はしません。
不採用時と旋回中は固定のゼロ点補正を使うオドメトリに戻ります。

この接続方法と通路軸の仮定を含む比較であり、2状態フィルタ単独から全体位置が得られるわけではありません。
4状態EKFの出力は2状態版の積分へ使いません。2状態出力を使わない場合に
ゼロ点補正のみの経路と一致することを検証します。
帰還点の始点からの距離は推定の閉じ具合の指標であり、実際の帰還位置誤差とは区別してください。

## 壁観測によるオフラインEKFの試作

`wall_heading_ekf.py` は保存済みBLEログと連続復元済みオドメトリを使い、
位置・姿勢・ジャイロ積分の残差バイアスを推定します。機体Aの保存データでの試作であり、
ファームウェアや走行制御は変更しません。NumPy・pandas・Matplotlibを使用します。

```powershell
python tools/analysis/wall_heading_ekf.py `
  --log tools/logs/20261104_221513_567719_ble_explore/telemetry.csv `
  --odometry tools/logs/20261104_221513_567719_ble_explore/route_overlay/continuous_odometry.csv `
  --settings tools/logs/20261104_221513_567719_ble_explore/run_221644_948095.json `
  --route tools/logs/20261104_221513_567719_ble_explore/route_analysis/route.json `
  --output tools/logs/20261104_221513_567719_ble_explore/kalman_trial
```

元ログとの連番・MCU時刻が一致する復元CSVを明示します。復元CSVの必須列は
`sequence`, `mcu_time_ms`, `east_mm`, `north_mm`, `heading_rad`,
`estimated_control_interval_ms`, `interpolated_interval` です。位置はmm、角度は左正rad、
初期姿勢は原点・北向きです。時間幅は制御積分に対応するmsで、行番号や一定の配信周期から置換しません。
生ログのファイル選択やオドメトリ復元は自動では行いません。

予測は元の移動増分、補正は直進時の安定した側壁距離の変化から求める姿勢観測です。
平行な壁と直交する迷路軸を仮定し、壁切れ・距離上限/下限・前壁近傍・急な変化を除外します。
routeと帰還座標はフィルタへ入力せず、係数の調整にも使いません。`--route`は表示用です。
壁観測の1/3を更新に使わずに残し、未使用観測の距離変化予測誤差を評価します。
さらに、その1/3の選び方を3通り切り替えて集計します。ブロックは境界の1サンプルだけ共有します。
観測ノイズ等はコード内の仮定値であり、このデータの終点に合わせて最適化した値ではありません。

`report.json`に元の推定・静止時バイアス補正のみ・壁EKFの3条件と評価結果を保存します。
`trajectories.csv`, `wall_observations.csv`, `ekf_comparison.png`も出力します。
元オドメトリに含まれるリセット・時間復元の誤差、横滑り、距離スケールの誤差は残ります。
壁観測との整合性や帰還整合性の改善は、外部測位による実位置の精度改善とは区別してください。
測定後の推定軌跡の比較であり、記録された実走行が改善したことを示すものではありません。

検証は `python -m unittest discover -s tools/tests -p test_wall_heading_ekf.py -v`。
非線形予測の線形化と観測更新の構成は
[PythonRoboticsのEKF解説](https://atsushisakai.github.io/PythonRobotics/modules/2_localization/extended_kalman_filter_localization_files/extended_kalman_filter_localization.html)
も参照してください。本ツールの壁観測モデルは、同資料のGNSS観測とは異なります。

## Lstart/Lendを新しく測定する

### 測定条件

1. 対象速度・対象ターンの`Lstart`と`Lend`を左右とも`0.0`にする。
2. 赤色を機体前、黄色を機体後ろのマーカーとして取り付ける。
3. 360 mm角（45 mm方眼8×8）のArUco較正領域が映る固定カメラで撮影する。
   四隅の内側頂点間の寸法を`video_marker_pose.py`の`GRID_MM`と一致させる。
4. 1ターンにつき動画と`myshell_debug`ログを3本ずつ取得する。
5. 動画とログは同じ順番で取得し、時刻とターン名の両方で対応を確認する。
6. スローモーション倍率を記録する。倍率が既知なら必ず
   `--slow-factor`で固定する。

四隅のArUcoはID 5・7・6・4の配置を維持し、各マーカーの盤面内側の頂点を較正に使う。
印刷マーカー自体の回転には依存しない。固定した頂点番号を使うと、マーカーの貼付方向が違う場合に
外側の頂点を選び、位置・角度へ誤差が入る。検証は
`python -m unittest discover -s tools/tests -p test_video_marker_pose.py -v`で行う。

動画との同期と旋回区間の切出しには、CSVの`time_ms`（なければミリ秒単位の`cnt`）を使う。
切出しログは先頭時刻を0へ揃える。5 ms配信や不等間隔のログを1行1 msとして扱わない。
新しい解析JSONは`log_omega_start_time_s`・`log_omega_end_time_s`を記録し、
過去のJSONの1 ms行番号形式も読み込める。等速のUSB録画では`--slow-factor 1`を指定する。

測定時の`.settings.json`も保存する。解析スクリプトは設定内の
`lstart`または`lend`が0でなければ停止する。

### manifestを作る

解析結果のディレクトリに`manifest.csv`を作る。

```csv
video,log_stem,motion,target_lateral_mm,target_forward_mm
PXL_..._turn90.mp4,20260725_..._myshell_debug_log_long_r90,long_r90,-90,90
```

- `video`: 動画ディレクトリからのファイル名
- `log_stem`: `tools/logs`内の`.csv`と`.settings.json`から拡張子を除いた名前
- `motion`: ログのターン名
- `target_lateral_mm`: 撮影盤面上で左が正、右が負
- `target_forward_mm`: 初期進行方向が正

標準的な目標座標は次のとおり。

| motion | target_lateral_mm | target_forward_mm |
| --- | ---: | ---: |
| long_r90 | -90 | 90 |
| long_l90 | 90 | 90 |
| long_r180 | -90 | 0 |
| long_l180 | 90 | 0 |
| in_r45 | -45 | 90 |
| in_r135 | -90 | 45 |
| out_r45 | -90 | 45 |
| out_r135 | -90 | -45 |
| r_v90 | -90 | 0 |

右ターンだけを測定する場合、採用したLstart/Lendを左ターンにも使う。
左右を別々に測定した場合は、manifestでも`r`と`l`を分けて集計する。

### 全ターンを解析する

PowerShellでリポジトリルートから実行する。

```powershell
python tools/analysis/lzero_turn_lengths.py `
  --manifest tools/turn_analysis/1800/video_YYYYMMDD_lzero/manifest.csv `
  --video-root C:\path\to\video_turn1800 `
  --output tools/turn_analysis/1800/video_YYYYMMDD_lzero `
  --speed 1800 `
  --workers 3
```

倍率が4倍など既知の場合は固定する。

```powershell
python tools/analysis/lzero_turn_lengths.py `
  --manifest tools/turn_analysis/1400/video_YYYYMMDD_lzero/manifest.csv `
  --video-root C:\path\to\video_turn1400 `
  --output tools/turn_analysis/1400/video_YYYYMMDD_lzero `
  --speed 1400 `
  --slow-factor 4 `
  --workers 3
```

`--slow-factor`を省略すると6.5–9.5倍の範囲で同期倍率を探索する。
採用条件の既定値はマーカー検出率95%以上、採用倍率7.5–9.0倍、
動画―ログ軌跡RMSE 8 mm以下である。撮影条件や高速時の滑りに合わせて
変更する場合は、結果の再現性を確認したうえで次を使う。

```text
--min-detection-pct 90
--min-slow-factor 7 --max-slow-factor 9
--max-rmse-mm 10
```

### ターンをひとつずつ解析する

`--motion`を指定すると、manifest内の該当ターンだけを処理する。
出力をターン別ディレクトリに分ければ、全体の集計結果を上書きしない。

```powershell
python tools/analysis/lzero_turn_lengths.py `
  --manifest tools/turn_analysis/1800/video_YYYYMMDD_lzero/manifest.csv `
  --video-root C:\path\to\video_turn1800 `
  --output tools/turn_analysis/1800/video_YYYYMMDD_lzero/by_motion/long_r90 `
  --speed 1800 `
  --motion long_r90 `
  --workers 3
```

`--motion`は複数回指定できる。

```text
--motion long_r90 --motion long_r180
```

追跡済みの結果を意図的に再計算するときは`--force-motion long_r90`、
全ランを再計算するときは`--force`を付ける。

### 出力と採用値

- `README.md`: 採用数、中央値、動画範囲、品質条件
- `adopted_summary.csv`: パラメータへ反映するLstart/Lend
- `run_results.csv`: 全ランの倍率、検出率、RMSE、動画解、ログ解
- 各ランの`trajectory_overlay.jpg`: 動画軌跡とログ軌跡
- 各ランの`marker_log_comparison.png`: 同期、速度、位置誤差
- 各ランの`metrics.json`: 詳細な同期・検出指標

採用値は品質条件を通過した動画解の中央値である。負の解は実装できないため、
`adopted_summary.csv`では0 mmへ制約され、READMEにも記録される。
long180だけは始終方向が平行で連立方程式からLstart/Lendを一意に分離できない。
旋回軌跡の前進頂点を95 mmに置いてLstartを決め、終点からLendを決める。

`Params/Param_A/turn_<speed>.h`へ小数第2位で反映し、右だけを測定した速度は
左右を同値にする。左右を測定した速度は方向別に反映する。最後にSTM32ビルドを行う。

### 使用するスクリプト

- `lzero_turn_lengths.py`: manifestの検証、対象ターンの選択、全ランの解析・集計
- `trim_motion_video.py`: 元動画から実走行区間を切り出す
- `video_marker_pose.py`: ArUco較正、赤・黄マーカー追跡、ログ同期

実例は次を参照。

- [`../turn_analysis/1400/video_20260725_lzero/README.md`](../turn_analysis/1400/video_20260725_lzero/README.md)
- [`../turn_analysis/1600/video_20260724_lzero/README.md`](../turn_analysis/1600/video_20260724_lzero/README.md)
- [`../turn_analysis/1800/video_20260725_lzero/README.md`](../turn_analysis/1800/video_20260725_lzero/README.md)
- [`../turn_analysis/2000/video_20260725_lzero/README.md`](../turn_analysis/2000/video_20260725_lzero/README.md)

## その他の現行ツール

### 登録済みマーカー2枚以上での盤面補正

`video_marker_pose.py`へ`--marker-reference`を指定すると、基準画像と共通する任意の2枚以上のArUcoの各4頂点を照合し、射影変換で較正します。
3枚読めた場合は12頂点を使用します。基礎方式は特定の隅の検出を必要としません。
調整用基準の`grid_refinement.enabled`がONの場合は、45 mm格子も使ってフレームごとに再校正し、未使用の格子点で検査します。詳細は[Pixel撮影手順](../pixel_camera_README.md)を参照してください。

```powershell
& D:/venv/venv3-11/Scripts/python.exe tools/analysis/video_marker_pose.py `
  --video C:/path/to/video.mp4 --log C:/path/to/motion.csv --output tools/logs/video_analysis/new_run `
  --front-color red --marker-reference tools/logs/pixel_camera/marker_reference.json
```

出力の`marker_reference_used.json`へ使った基準を保存し、`metrics.json`の`marker_calibration`へ基準ID・使用ID・較正成立率を記録します。
`aruco_complete_frame_rate`は4枚を実検出した率のままで、2～3枚による較正成立率と混同しません。
登録基準指定時はMP4自動回転を無効にし、TYPE8の横向き画像座標を使います。
指定がなければ従来の4枚による較正を維持します。基準の作成と撮影側の設定は[Pixel撮影手順](../pixel_camera_README.md)を参照してください。
絶対mm精度は元の基準校正に依存します。現在の盤面での[45 mm格子による検査結果](../logs/pixel_camera/20261109_accuracy_final/README.md)を確認してください。Lstart/Lendの採用値への自動反映は行いません。

### 機体マーカーの高さと動画の時間軸

`video_marker_pose.py --body-geometry tools/logs/pixel_camera/body_geometry_A.json`で、赤10 mm・黄約1 mmを個別に補正できます。
カメラ中心の盤面座標x/y/z（mm）が必要で、未確定なら解析開始前に停止します。
機体Aの運用設定は`camera_height_policy=per_measurement`です。解析ごとに`--camera-height-mm`（例: 800）を指定し、前回の高さは自動流用しません。
レンズから鉛直に下ろした点の盤面座標は`--camera-foot-mm X Y`で指定するか、設定の`camera_foot_board_mm`へ保存します。
高さ・位置を撮影ごとに確定したコピーへ展開し、`body_geometry_used.json`と`metrics.json`に、その動画のパスと一緒に保存します。
車軸中心のオフセットがなければ、`metrics.json`の`body_geometry.center_kind`は`marker_midpoint`です。
指定した設定は`body_geometry_used.json`にも保存します。赤を前にした場合も、互換性のため前マーカーのCSV列名は従来の`blue_x_mm`等を保持します。

画像を全フレーム分メモリに蓄積せず読み直し、動画の実PTSとCSVの実`time_ms`を使います。
`--video-start 6.5 --video-end 9.5`等で解析範囲を限定できます。PTSとフレーム番号は元動画のままです。
今回の独自240 fps動画は実時間記録なので、同期探索には`--slow-factor 1`を指定します。
補間した未検出点は動画・ログの位置合わせから除外します。軌跡から推定した時刻合わせは、LED等による独立した同期検証とは区別します。
Lstart/Lendの採用値を求める場合は、上記のゼロ設定・対応ターン・品質条件も満たす必要があります。

### 補正に使っていない格子点での精度検査

真下位置が不明でも固定カメラでの相対変位は`pixel_camera_relative_pose.py`で求められます。
未知の真下位置を仮定せず、赤・黄の異なる高さで重み付けした中点の変位を使います。
機体の絶対位置・高さ補正済みの機体角度は出力しません。`trial.json`で動画・CSV・設定と対応付け、撮影ごとの高さとその根拠を指定します。
動作全区間を含む比較と、MCU動作区間前後の有効率を検査します。[手順](../pixel_camera_README.md)と[保存動画による確認](../logs/pixel_camera/20261109_unknown_camera_foot/README.md)を参照してください。

`pixel_camera_accuracy.py`は、45 mm格子の内部交点を画像の白線中心から検出し、マーカー補正行列で求めた座標・距離・直角の誤差を調べます。
マーカーだけの基準では、格子点をマーカー補正行列の推定に使用しません。
格子併用の基準では、校正用に指定された格子点を検査から除外し、残りの点だけで評価します。検査点へ行列を再フィットしません。
機体などの色のある遮蔽物や、線の検出品質を満たさない点は除外理由付きで保存します。

```powershell
& D:/venv/venv3-11/Scripts/python.exe tools/analysis/pixel_camera_accuracy.py `
  --capture tools/logs/pixel_camera/20261109_060259_97d7ae24.capture.json `
  --reference tools/logs/pixel_camera/20261109_marker_registration/reference.json `
  --roi-settings tools/logs/pixel_camera/framing_settings.json `
  --output tools/logs/pixel_camera/accuracy_NEW --pitch-mm 45 --frames 12
```

出力はaccuracy_report.json、checkpoints.csv、distances.csv、angles.csv、検出品質・使用フレームのCSV、誤差図、入力基準のコピーです。
位置のRMS・95％点・最大観測誤差と、45 mm区間の距離誤差を分けて確認します。2枚の各組み合わせも同じ検査点で比較します。
盤面上の点の検査であり、外周・隠れた点・機体上の色マーカーの高さによる視差は未検証です。基準寸法自体の誤差も別途考慮する必要があります。

| スクリプト | 用途 |
| --- | --- |
| `ff_analyzer.py` | 走行ログからフィードフォワード係数を推定 |
| `search_turn_sp_ff.py` | 探索ターンの並進フィードフォワードを推定 |
| `search_turn_voltage_report.py` | 探索ターンの電圧と応答を解析 |
| `turn_ff_report.py` | 高速ターンのフィードフォワードレポートを生成 |
| `turn_sp_voltage_report.py` | 高速ターン中の並進電圧を解析 |
| `log_compare.py` | 目標値、実測値、位置軌跡を比較 |
| `odometry_compare.py` | オドメトリ方式を比較 |
| `generate_aruco_print.py` | 動画較正用ArUcoを生成 |

各コマンドの引数は次の形式で確認できる。

```powershell
python tools\analysis\<script>.py --help
```

## Legacy

次のコードは旧Lstart/Lend方式との比較または回帰テストのために残している。
新しいターン長の決定には使用しない。

- `encoder_only_turn_lengths.py`
- `turn1600_variable_speed_lengths.py`
- `video_turn_overlay.py`
- `video_turn_simulator_compare.py`
