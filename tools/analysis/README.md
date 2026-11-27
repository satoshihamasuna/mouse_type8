# 解析ツール

単体ターン調整GUIは`tools/turn_tuning_log.py`で本体ログ画像を自動保存します。
`python tools/turn_tuning_log.py <trial.json>`で再作成できます。設定JSONの実際の記録周期とCSVの経過msを照合し、
角加速度・FF・軌跡の計算にもその周期を使います。[出力ファイル](../turn_tuning_README.md)を参照してください。

単体撮影の停止評価は、`pixel_camera_tuning_report.py`が同じ試行の連続`telemetry.csv`を参照し、直進指令に続く`run_brake`が終わってから0.2〜0.6秒の画像を確認する。短い`motion.csv`だけでは200 ms以上の停止記録が切れていたり、指令終了後もブレーキが動作している場合があるため。旧試行で連続CSVがない場合は従来の入力へ戻る。観測点数・静止時間・画像散らばり・ログの速度/角速度・軌跡RMSEの条件は緩めない。元の解析結果を保持し、再評価は別出力先へ保存する。

周期ログの現行形式は`L8S5` / 保存magic `0xA55D`です。旧`ideal.turn_x/y`の2枠は採用姿勢・標準偏差のrad値で、無効時はNaNになります。
`log_compare.py`は旧理想軌跡列がない場合も`ego.turn_x/y`の実測軌跡を表示します。[姿勢ログの仕様](../ble/README.md#採用姿勢の2項目)を参照してください。

## ファーム内ターンEKFを保存ログで因果的に再生する

`turn_pose_firmware_replay.py`は実際のC++実装と機体Aのパラメータをホストでコンパイルし、
未来の加速度補間なしで保存CSVを再生します。出力ディレクトリは新規に限定します。

```powershell
python tools/analysis/turn_pose_firmware_replay.py --review tools/logs/turn_trajectory_overlay_A_20261118_v1 --output PATH_TO_NEW_OUTPUT
```

先に作成した軌跡比較の同じ動画座標を使い、採用された周期の増分だけを反映してRMSEを比較します。
旧ログに制御stateがないため、旋回区間は角速度指令ピークの2%を超える区間で近似します。
5ms配信点の因果再生で、欠けた1ms観測や実機IRQ負荷を復元するものではありません。
出力は実行ファイル・入力/出力・各試行の軌跡・集計CSV・実装ハッシュ付きJSONです。
採用範囲と結果は[ファーム採用時の検証](../logs/turn_pose_firmware_A_20261118_v4/README.md)を参照してください。

## 単純積分・滑りモデル・動画の軌跡をHTMLで重ねる

`turn_trajectory_overlay.py`は条件別滑り解析の`report.json`と有効動画観測から、
単純積分・吸引別滑りEKF・動画の相対軌跡を同じ入口/時刻で比較するHTMLを生成します。
入力ハッシュを照合し、入口位置と入口直線の向きだけを共通化します。
推定方法ごとの回転/尺度/終点合わせは行わず、欠測は20msを超えて補間しません。

```powershell
python tools/analysis/turn_trajectory_overlay.py --review tools/logs/turn_slip_conditions_A_20261118_v1 --output PATH_TO_NEW_OUTPUT
python -m unittest discover -s tools/tests -p test_turn_trajectory_overlay.py -v
```

`compare.html`に19試行の切替、3軌跡の表示切替、時刻スライダー、元動画の同期再生、
位置誤差とRMSEの比較一覧を含みます。グラフのデータはHTML内に保存し、動画は元の保存先を参照します。
動画が別の場所にある場合はHTML内で該当動画を選べます。
出力は新規ディレクトリに限定し、`summary.csv`・各試行の誤差CSV・`manifest.json`も保存します。
今回の成果物は[比較HTML](../logs/turn_trajectory_overlay_A_20261118_v1/compare.html)。
動画のマーカー中点に対する照合であり、Kの同定に同じ試行を含むため独立な精度保証ではありません。

## ターン中のヨー角・横滑り・平面位置をEKFで推定する

`turn_pose_ekf.py`は機体AのMCU時刻付き保存ログを使う8状態のオフライン試作です。
状態は入口固定座標の位置`x,y`、左正のヨー角、固定座標の速度`vx,vy`、
ジャイロバイアス、前後・左右の加速度バイアスです。車輪平均の前進速度を観測し、
ジャイロと前後・左右加速度で予測します。横滑り角は機体座標の速度から求め、
`ego.turn_slip_theta`、画像、出口目標、旋回中の横滑りゼロ仮定は入力にしません。
NumPy・pandasが必要です。

吸引ON/OFFで[turn_pose_profiles_A.json](turn_pose_profiles_A.json)の雑音設定を自動選択します。
選択元は設定JSONの`suction.enabled`で、速度からは推測しません。不明な状態は拒否します。
非吸引の校正域は500mm/s、吸引は1600mm/s・duty650です。他の速度・吸引量への適用は外挿になります。

中心付近のIMUというユーザー情報とターン動画から同定した滑りモデルを使う場合は、
下のコマンドへ`--slip-model`を追加します。この場合は6状態`x,y,psi,beta,b_gyro,b_acc_left`に切り替わり、
吸引別の`K`で`beta_dot=-K/u*beta-omega`を予測、横加速度`ay=-K*beta+b`で更新します。
横加速度を予測入力と観測へ二重に使用しません。20点平均の車輪速度は
`ego.velo-.010*ego.accel`から復元し、記録された周期の中央値と平均遅れを使います。
途中でIRQ診断が1行欠けても平均周期が十分に測れていれば、この方式では利用可能です。
`--slip-model`は中心IMUと校正済み滑りを仮定する事後解析で、`--geometry`とは併用できません。
既存の8状態版は壁の幾何観測に対応し、吸引別の雑音設定を使います。
条件別同定・走行単位の除外検証・実走行適用は
[500非吸引と1600吸引の比較](../logs/turn_slip_conditions_A_20261118_v1/README.md)を参照してください。

```powershell
python tools/analysis/turn_pose_ekf.py `
  --log PATH_TO_MCU_TIMESTAMPED_MOTION_CSV `
  --settings PATH_TO_MOTION_SETTINGS_JSON `
  --output PATH_TO_NEW_OUTPUT
python -m unittest discover -s tools/tests -p test_turn_pose_ekf.py -v
```

出力は`states.csv`と入力ハッシュ付き`report.json`です。積分には`time_ms`差を使い、
1制御回分の`sp_pulse`の速度換算には`irq.period_us`を使います。
行番号や配信周期で車輪パルスを割りません。静止サンプル30点以上、連続sequence、
有限な入力が必要です。先頭行のIRQ診断欠損だけはその行を省き、途中の欠損は拒否します。
実機の20点平均加速度を約9.5制御周期だけ先へ補間して遅れを合わせるため、
これは事後解析です。`--accel-delay-ms 0`で遅れ未補正の感度を比較できます。
`--imu-forward-mm` / `--imu-left-mm`は車軸基準のIMU取付位置で、既定0は未測定の仮定です。
角加速度と向心加速度のレバーアーム補正を行い、角加速度の算出にも未来点を使います。
平均前の加速度や間引きで欠けたエンコーダ値は復元しません。

壁なしでは入口を基準とする相対推定です。初期位置・方位、横加速度バイアス、
傾きによる重力投影、IMU位置の誤差が残ります。共分散の数値だけで実精度を保証しません。
ロール・ピッチは推定せず、固定バイアスで表せない傾きの変化はモデル外です。

`--geometry PATH_TO_JSON`で既知の有限壁線分への光線観測を追加できます。
JSONには`mouse:"A"`、`calibrated:true`、`range_model:"calibrated_beam_length"`、
`walls:[{a_m:[x,y],b_m:[x,y]}]`、`sensors`が必要です。
各sensorは`name,offset_m:[前,左],angle_rad,min_m,max_m,sigma_m,column,valid_column,valid_value`を持ちます。
`column`はmm単位の校正済み光線距離、`valid_column`は有効状態の明示的な列です。
壁・センサ位置は同じ入口固定座標を使い、全てm・radへ換算して計算します。
斜入射・端点近傍・対応曖昧・範囲外・残差過大は棄却し、同じセンサの更新を20ms以上離します。
壁を使わない間も同じ状態・共分散で予測を継続します。
**現行の`sen_*.distance`を、そのまま旋回中の光線距離とみなしてはいけません。**
ADC距離変換の基準、光軸・取付位置、入射角・反射率、壁面対応の校正が必要です。
片壁では壁に沿う位置が拘束されず、壁切れ・柱や非平行壁など追加の幾何情報が必要です。

機体A・500mm/sの20本と模擬壁の検討は
[2026-11-18の解析結果](../logs/turn_pose_ekf_A_20261118_v1/README.md)を参照してください。
実ログの旋回中は全センサが遠距離側の上限値で、壁ありの実精度は未検証です。
機体Bの校正、ファームへの組込み、実機制御への採用は行っていません。

## 左右の斜め特徴点から位置・角度をShadow検証する

`diagonal_edge_geometry.py`は、物理柱との対応を確認した`pillar_comparison.csv`、
`cases.json`、`map_verified.json`、`deep_provenance.json`を含む解析フォルダーと、
センサ取付方向の`photo_geometry.json`を入力にします。NumPy・SciPy・Matplotlibが必要です。
機体Aの今回の6柱×7走行に対応し、未校正の角度・取付位置を機体Bへ適用しません。

```powershell
python tools/analysis/diagonal_edge_geometry.py --review PATH_TO_REVIEW --photo-geometry PATH_TO_PHOTO_GEOMETRY_JSON --output PATH_TO_NEW_OUTPUT
python -m unittest discover -s tools/tests -p test_diagonal_edge_geometry.py -v
```

画像座標を推定へ入力せず、前方・横・角度の幾何推定とPythonのEKFで再生します。
極小と差分交差を分け、飽和・品質不良を除外し、左右交互の柱を世代・実時刻で扱います。
確認時刻までに得たエッジだけを採用し、事後解と確認済みのみの解を分離します。
高速では5ms角速度の再積分誤差も照合し、確認済みの1ms積算角スナップショットを使う別解も出します。
運動データの時刻補間には次の5msサンプルが入ることがあり、実機の1msオンライン動作を再現した保証にはなりません。

`report.html`、各柱の`predictions.csv`、`geometry_sweep.csv`、`fits.json`、
`ekf_audits.json`、入力SHA256付き`provenance.json`を保存します。出力先は新規にし、入力を保持します。
これはモデルの検証であり、ファームのEKFへは接続せず、Paramsを変更しません。
写真の部品取付角とIR波形の実効特徴点方向、車軸と画像マーカー中点は別の校正項目です。

周期ログの`L8S4`には`sen_l.value` / `sen_r.value` / `sen_fl.value` / `sen_fr.value`が追加されています。
距離変換前の点灯/消灯ADC差分を整数のまま保存するため、距離値が25/80 mmなどへ制限される区間でも波形を確認できます。
`ir.range_states`と`motion.pattern`も同時刻で保存します。定義は [ADC周期ログ](../ble/README.md#adcを含む周期ログl8s4) を参照してください。
旧ログにこれらの列がない場合は復元しません。PIDゲイン6列は新しい周期ログには含まれません。

## 壁の補正判断と動画の対応

位置解析の`review.html`に、MCUが保存した探索ターン判断と壁切れ補正の表を追加しました。
各行をクリックすると、その判断の約0.3秒前へ動画を移動します。画面のoffset・時計比率を調整すると移動先も追従します。
壁距離・ADC差分・範囲状態・走行パターンの波形は、CSVに保存された全サンプルと実`time_ms`を使います。
`wall_edge`試験の`reason:shadow`は灰色の線で表示し、実適用の緑と区別します。
`proposed_delta_mm/proposed_after_mm`は補正予定、`delta_mm/after_mm`は実際の縦位置への適用結果です。
追加の柱候補は`pillar_candidate`として同じ比較表に表示します。動画と波形の線は`minimum_ms`の実観測時刻へ合わせ、
確認時刻`ms`と`confirmation_video_s`を別に残します。姿勢・横位置と前壁ADCは極小時の組で比較します。
「波形の表示範囲」で全体／現在位置の前後1秒／前後0.25秒を選べます。波形をクリックして動画を移動できます。
左＝青、右＝赤、前左＝緑、前右＝紫です。縦線は緑＝補正採用、赤＝棄却、紫＝探索時の補正動作判断で、壁端の真の通過時刻とは区別します。
ADC等の旧ログ未記録項目は欠測のままとし、数値欄は12 ms以内に観測がある場合のみ表示します。
対応時刻は `動画秒 = offset + clock_scale * (ms - mcu_origin_ms) / 1000` です。
青LEDや旋回波形で同期を確認し、手動同期を変更した場合は再解析してCSVへ反映してください。

`wall_events.csv`にはMCU時刻、動画時刻、同期採用フラグ、受信完備フラグ、理由、距離、ADC、範囲状態、補正前後を出力します。
通常の旋回表の番号は角速度指令区間の順序、壁の判断表の番号はファームウェアの判断順序です。
旧ファームウェアの未記録項目は空欄とし、距離の端点や近傍テレメトリからADC・状態を推測しません。
記録形式と各フラグは [BLEログの仕様](../ble/README.md#壁制御を改善するためのログwall_diag-schema-1) を参照してください。

2026-11-14の既存成功走行の62判断は、`tools/logs/20261114_A_wall_diagnostics_review/review.html`でも確認できます。
既存の追跡・同期を再利用した表示で、変更後ファームウェアの走行結果ではありません。元の動画・測定資料は保持しています。

## 探索・最短の動画から位置・向きと制御応答を比較する

`maze_video_pose.py`は録画付き走行GUIの`trial.json`から、動画・BLE CSV・実機設定を照合します。
OpenCV（aruco入り）・NumPyが必要です。カメラ・BLE・機体へは接続しません。

```powershell
python tools/analysis/maze_video_pose.py --trial PATH_TO_TRIAL_JSON --reference PATH_TO_9X9_REFERENCE_JSON --cell-mm 90 --stride 4 --output tools/logs/POSITION_NEW
```

標準では今回の動画の最大16フレームをそれぞれArUcoで登録し、黒い床上の赤・白の壁候補を検出します。登録JSONに残る過去の壁配置は自動適用しません。
同じ盤面・ArUco配置でも、壁配置は動画ごとに独立して検出します。過去の壁・開口で今回の未確認辺を補完しません。GUIの手動壁配置も1回分だけで、次の試行へは引き継ぎません。
180辺ごとの観測数と壁・開口・未確認の判断を`wall_map_used.json`へ保存します。遮蔽や反射は未確認になり得るため、HTMLと元動画で照合してください。候補を本体の地図へ書き戻す処理はありません。
確認済みの配置を使う場合は、`--wall-map PATH_TO_VERIFIED_WALL_MAP_JSON`で走行時の物理壁配置を指定します。
ArUco座標登録は`--reference`を使い、表示する壁180辺だけを`--wall-map`から選びます。
HTMLの上部に壁配置の版・入力を表示し、`analysis.json`の`wall_map`へSHA256を記録します。
旧動画へ現在の壁配置を一律に使わず、元動画の壁と照合してください。
座標登録が成立しても、赤い壁や機体内部の黄色部品を前後マーカーと誤認する場合があります。
姿勢の飛びを同期の調整で隠さず、元フレームでマーカーの位置を確認して除外します。

出力先は未作成のフォルダーを指定します。240fpsの速報は`--stride 4`、詳細は`--stride 1`です。
録画付きGUIは高速走行の短い動作を落とさないよう、解析間隔を既定1にします。間隔2・4も選択できます。
9×9の色検出は既存の一意な組を優先し、組が見つからない場合だけ小さい・淡い目印を再検出します。曖昧な組と盤面登録の欠測は補完しません。
`--marker-detection PATH_TO_BODY_JSON`で機体別の`marker_detection`、公称間隔・許容間隔を指定できます。`mouse`・前後色は実機設定と照合します。この指定から調整盤の高さや座標登録は流用しません。
GUIでは「色検出・壁配置の詳細」から同じJSONを選べます。録画時に選んだ設定は試行内へコピーします。
動画の元フレーム番号・実PTSと、CSVの実`time_ms`を使います。行番号を1 msへ変換しません。
前後マーカーの色は、走行時の設定JSONの`firmware.mouse`から自動選択します。機体Aは前＝赤・後＝黄、機体Bは前＝黄・後＝赤です。
機体名がA/Bと確認できない場合は位置解析を止めます。保存済み試行の再解析も、その試行の機体設定を使います。
選択した色は`analysis.json`と比較画面に記録し、元動画への重畳色も前後の実際の色に合わせます。
9×9盤面用の登録済みArUco基準と、組の選択用の公称間隔約50 mmを使います。これは機体Bの寸法・高さを実測校正した値ではありません。
1マス寸法を省略するとセル単位です（組の選択には今回の90 mm盤面に対する約0.56セルの間隔を使います）。
毎フレームで2個以上の登録IDを実検出し、RANSAC残差・分布を確認します。
未読IDは基準近傍で拡大して再復号しますが、前フレームの位置で補完しません。
姿勢候補が複数、色の組がない、過大な移動・回転、盤面登録がない箇所は欠測です。

- `review.html`: 検出位置を重ねた元動画、盤面の軌跡・前向き、速度・角速度・青LED画素数、旋回別表。
- `pose.csv`: 観測位置・向き・品質・元フレーム/PTS。欠測を保持します。
- `matched.csv`: 同期オフセットを適用した動画と直近BLE観測（12 ms以内のみ）。
- `turns.csv`: 指令旋回区間ごとの角度、角速度追従RMS、映像観測率、BLE最大間隔、側壁距離。
- `analysis.json` / `sync.json`: 機体・入力SHA256・解析版・条件・品質・同期候補と制約。

同期はPC時刻による初期値の±2秒で、動画の符号付き角速度とジャイロを照合します。
励起・観測数・相関・候補の一意性が不足すると自動採用しません。
比較画面で青LEDと旋回を照合し、手動同期JSONを保存した場合は上記コマンドに`--sync PATH_TO_SYNC_JSON`を追加して新しい出力先へ再解析します。
30秒以上の走行では5区間の旋回一致も照合します。3区間以上で一意に一致し、
時間方向へ十分に分布し、整合する場合だけ、時計比率を含む
`video_time = offset + clock_scale * MCU経過秒`へ進みます。空間軌跡へ合わせて誤差を消す処理はありません。
定数オフセットより相関が十分改善しない場合は採用しません。露光時刻・物理的な時計精度の校正とは区別します。
比較画面の`video/MCU`で比率も確認でき、手動同期JSONに`clock_scale`も保存します。

今回の9×9用基準は`tools/logs/pixel_camera/board_9x9_reference.json`です。旧8×8・360 mmの基準を使い回しません。
確認済み格子レポートと同じ画像（SHA256一致）から、次の形式で登録できます。

```powershell
python tools/analysis/maze_video_pose.py --register-frame PATH_TO_FRAME_1_5S_PNG --grid-report PATH_TO_VISIBILITY_REPORT_JSON --output PATH_TO_9X9_REFERENCE_JSON
```

盤面左上原点、x右・y下の座標で、マーカー中点を追跡します。機体の迷路原点・方位は自動決定しません。
1マス90 mm・黒枠40 mmはユーザー確認値ですが、レンズ歪み・高さ・真下位置は未校正です。
mm出力も基準平面へ投影した見かけの位置です。真の横滑り角・車軸位置・Lstart/Lendの採用値にはそのまま使いません。
まず画像重畳・欠測・同期を確認し、旋回の入口／出口のずれと角速度追従・壁応答を照合して、改善対象を決めます。
採用Params・制御ゲインの自動変更はありません。

Pixel撮影タブの解析は`pixel_camera_tuning_report.py --trial PATH --stride 4`（速報）／`--stride 1`（全フレーム詳細）です。
録画・走行はせず、保存済みの動画・CSV・実機設定・プロファイルだけを使います。出力は試行内のquick/とfull/で区別し、既存出力を上書きしません。
速報は元のPTS・フレーム番号を保った約60fpsで、PROVISIONALとして扱います。全フレーム用キャッシュと混用しません。
解析範囲は走行開始応答からの推定に加え、実際の録画停止要求時刻まで含めます。吸引の立ち上がり待ちがある高速ターンでも旋回・停止を切り落としません。
保存済み試行の再解析は`--output PATH_TO_NEW_FOLDER`で別の未作成フォルダーへ出力できます。必要なら`--video-start` / `--video-end`で範囲を明示します。
停止確認は指令後の実測速度・角速度と、映像の停止後の安定性・必須区間の観測率を併用します。採用パラメータは自動更新しません。
UIの操作と保存先は[Pixel撮影・調整](../pixel_camera_README.md#調整guiで撮影と単体走行をまとめる)を参照してください。

実機用の通路内2状態EKFは`Task/Inc/straight_wall_ekf.h`です。横偏差・姿勢偏差の運動式を
共有し、取得済みCSVの間引き値ではなく各制御回の移動量を使います。
壁制御への接続と`legacy/shadow/control`の切替は[直進壁EKF](../myshell_README.md#直進壁ekf)を参照してください。
以下のPython解析結果と実機の走行性能は区別して評価します。

## turn500から300／400／600の候補を予測する

既存測定から滑りを観測し直す場合は`turn_slip_observation.py`を使用します。
前回の予測モデルの入力ハッシュを照合し、動画接線とジャイロの比較、
同一動画内の剛体マーカー姿勢による再評価、横加速度との整合、
目印中点の車軸からのずれとの識別可能性を保存します。

```powershell
python tools/analysis/turn_slip_observation.py --output tools/turn_analysis/A_slip_review_NEW
python -m unittest discover -s tools/tests -p test_turn_slip_observation.py -v
```

元の前後マーカー追跡CSVと全フレーム解析が必要です。動画の画素からの追跡は再実行しません。
機体やカメラへは接続しません。元の予測モデルを再生成する前に、
[2026-11-12の滑り再評価](../turn_analysis/20261112_A_slip_analysis/README.md)の識別上の制約を確認してください。

`turn_speed_prediction.py`は、機体Aの`Params/Param_A/turn_500/turn_500.h`に記載された
14種類×3本の保存測定から有効な速度・旋回応答と横滑りを同定し、
300／400／600 mm/sの候補ヘッダー、設定JSON、軌道CSV、比較図を作成します。
Python 3.10以上、NumPy、pandas、SciPy、Matplotlibを使用します。

```powershell
python tools/analysis/turn_speed_prediction.py --output tools/turn_analysis/A_speed_prediction_NEW
python -m unittest discover -s tools/tests -p test_turn_speed_prediction.py -v
```

保存先は未作成のディレクトリを指定します。元動画・ログ・設定JSONと、
各試行の全フレーム解析が必要です。既存の絶対パスを含むため、別PCへ移す場合は入力の対応を確認してください。
別速度での実測精度は未検証です。500の試行除外交差検証と速度依存性の感度を区別し、
既存の採用Paramsへは自動反映しません。PID/FFは測定時のRAM係数を継承します。
モデル・候補値・検証結果・適用範囲は[2026-11-12の解析記録](../turn_analysis/20261112_A_speed_prediction_v2/README.md)を参照してください。

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

`Params/Param_A/turn_<speed>/turn_<speed>.cpp`へ小数第2位で反映し、右だけを測定した速度は
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

### 動画だけで盤面上の位置を推定する

`pixel_camera_position.py`は走行CSVを使わず、盤面ArUcoと機体の前後の色目印からXYを出力します。機体にArUcoを貼る必要はありませんが、盤面マーカーだけでは機体自体を検出できないため、前後の色目印も必要です。

```powershell
python tools/analysis/pixel_camera_position.py `
  --video C:/path/to/video.mp4 --reference C:/path/to/marker_reference.json `
  --geometry C:/path/to/body_geometry.json --output tools/logs/video_position/new_run
```

`--stride 4`は速報、既定の`1`は全フレームです。`--start`・`--end`で動画PTSの範囲を指定できます。`--trial trial.json`では保存済み試行の動画・盤面参照・目印設定だけを読み、CSVや停止判定には依存しません。出力先は新規フォルダーに限ります。

`position.csv`は元フレーム番号・動画PTS・フレームごとの登録マーカー数・有効マスクと、前後目印・中点のXYを記録します。盤面参照の左上が原点、Xは右・Yは下、単位mmです。各評価フレームで実復号した登録済みArUcoが2個以上必要で、欠測に補間位置を出力しません。`position_trajectory.png`と`summary.json`に軌跡・観測率・元動画と設定のハッシュを保存します。

位置は高さ未補正の見かけの目印中点です。車軸の絶対位置、高さ補正済みの機体yaw、停止合否、ターンの合格を示しません。追跡には既存の中央値フィルターを使い、欠測フレームの値は出力前にNaNへ戻します。

解析前に現在の動画で盤面登録を検査します。古い探索ROIが原因で不成立の場合は、3個以上の登録があり、全登録IDを全画面で一意に実復号でき、同じ盤面・格子品質条件を満たすときだけ探索ROIを更新します。これは探索範囲の補正で、後段の各フレームのAR登録や欠測除外は省きません。重複ID、未読ID、格子品質不足を自動救済しません。`marker_reference_input.json`に入力、`marker_reference_used.json`に使用した参照、`summary.json`の`reference_search`に試行PTS・元の失敗理由・更新の有無を保存します。旧解析と入力資料は上書きしません。

### 登録済みマーカー2枚以上での盤面補正

`video_marker_pose.py`へ`--marker-reference`を指定すると、基準画像と共通する任意の2枚以上のArUcoの各4頂点を照合し、射影変換で較正します。
3枚読めた場合は12頂点を使用します。基礎方式は特定の隅の検出を必要としません。
登録参照を指定する調整用・9×9用の位置評価では、各フレームで登録済みArUcoを2個以上実検出することが必須です。格子補正OFFでも、別フレームの中央値や別動画の変換で欠測を補いません。
調整解析では`calibration_valid=False`のフレームを評価から除外し、必要動作区間のArUco較正成立率も95%以上を要求します。フレームごとの参照を証明できない旧キャッシュは再解析が必要です。
調整用基準の`grid_refinement.enabled`がONの場合は、45 mm格子も使ってフレームごとに再校正し、未使用の格子点で検査します。詳細は[Pixel撮影手順](../pixel_camera_README.md)を参照してください。

色マーカーの候補点は一括で盤面へ射影し、高さ補正を使わない相対解析では間隔・盤内条件を配列で検査します。候補の色条件・間隔範囲・得点・同点時の順序は維持し、色ノイズが多い画像での重複計算を減らしています。

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
指定した設定は`body_geometry_used.json`にも保存します。互換性のためCSV列名は前マーカーが`blue_x_mm`等、後マーカーが`yellow_x_mm`等のままです。列名を実際の色と読み替えないでください。
機体Bの前緑・後赤は`--front-color green --rear-color red`で指定できます。
`pixel_camera_relative_pose.py`は機体JSONの`front_colour`・`rear_colour`を使い、追跡キャッシュも前後両方の色を照合します。
`nominal_marker_span_mm`と`marker_span_limits_mm`は前後の組の選択に使います。尺度の補正には使いません。
小さい目印では色別`marker_detection`に`minimum_area_px`と`opening_kernel_size`（0または3）を指定できます。
未指定時は従来の面積15画素・3×3 openingを使い、キャッシュは色・検出設定・間隔条件が変われば拒否します。
`opening_kernel_shape`は`rect`（既定）または`ellipse`です。後者は小さい丸を残しながら細い連結を除くための設定で、
同じ動画を変更前後で検査し、実際の目印の検出・誤認・必要区間の観測率を確認します。
照明で黄色が黄緑に映る場合などは、色別`hue_ranges: [[25,65]]`で色相の範囲も明示できます。
OpenCV HSVの0〜179の整数・両端を含む範囲を1〜4組指定します。赤の境界は`[[0,10],[170,179]]`のように分けます。
未指定時の色相範囲は従来どおりです。検出設定を変えたら別の保存映像でも誤認・静止安定性を確認し、旧解析を保持して新しい出力先へ再解析します。品質基準を緩めて有効本数を増やしません。
盤面参照の`board_side_mm`を使うため、810 mmの盤面を旧360 mmへ縮尺変更しません。

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
遮蔽の色マスクは、8近傍で連結した9画素以上の領域を25×25画素へ広げます。
孤立した色ノイズで見える格子点を除外せず、小さな色領域や細長い配線の遮蔽は維持します。白線の検出条件、補正点・未使用検査点の必要数と精度条件は維持します。

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

`turn_ff_report.py --project-header`には引き続き`Params/Param_A/turn_500/turn_500.h`などのヘッダーを指定できます。
FFレポートの区間長は実機と同じ`degree + correction`から見積もります。`correction`がない旧設定は0として扱います。
該当する区間がない場合は、予想長と観測長を表示して終了します。入力ログの1 ms周期と角度補正を確認してください。
共有定義へ移行したヘッダーでは、同じフォルダの同じ名前の`.cpp`を読みます（例：`turn_500/turn_500.h`→`turn_500/turn_500.cpp`）。値の調整先もこの`.cpp`です。
以前の一括定義版は、同じ機体の`run_param_A.cpp` / `run_param_B.cpp`内の該当区間を読みます。
従来の値を含むヘッダーも読めます。読み分けは`python -m unittest discover -s tools/tests -p test_turn_ff_profiles.py -v`で確認します。
解析が出力する候補ヘッダーを採用する場合も、実機で使用する定義の反映先は機体別の`.cpp`です。
| `generate_aruco_print.py` | 動画較正用ArUcoを生成 |

各コマンドの引数は次の形式で確認できる。

```powershell
python tools\analysis\<script>.py --help
```

## 斜めの柱特徴による姿勢推定の試作

`diagonal_feature_localization.py` は、既知の柱の通過位置から進行位置 `s` [mm]、左横位置 `e` [mm]、左向き角 `psi` [rad] を共同推定するオフライン参照実装です。ファームウェアへは接続していません。
左右・接続壁の型による実効基準、離反確認後の遅延更新、飽和の区間観測、回転中の時刻不確かさ、校正済み前センサ、座標軸変更時の平均・共分散変換を扱います。必要に応じて共通基準ずれの不確かさも保持し、横・角度と進行位置の採用判定を分けます。

機体Aの13走行に対する走行単位の外部検証は [新斜め補正の検討結果](../logs/20261118_diagonal_correction_design_v1/report.html) に保存しています。原画像の誤検出・欠測、同期不一致、変更後迷路、候補の対応付け、校正対象と全残差を追えます。
画像は他走行の校正と評価にのみ使い、当該走行の推定へ入力しません。前センサは取付・壁面対応の校正と測距状態が有効な場合だけ使います。機体Bの校正、実機適用、走行改善の確認は別途必要です。

```powershell
python -m unittest discover -s tools/tests -p 'test_diagonal_*.py' -v
```

保存データの再評価手順と前提は上記レポート末尾を参照してください。再生結果だけで補正定数を確定しないでください。

## ARによる盤面の正対補正だけを保存する

`pixel_camera_rectify.py`は機体の色や学習モデルを使わず、各フレームで実復号した登録済みArUco 2個以上から盤面を正方形へ射影補正します。登録基準の盤面寸法・軸・品質条件を使い、検索ROIの調整も既存と同じ検証を通した場合だけ行います。位置追跡やログ比較は呼びません。

```powershell
python tools/analysis/pixel_camera_rectify.py --trial <trial.json>
# 録画のみで保存した動画等は参照を明示する
python tools/analysis/pixel_camera_rectify.py --video <元動画.mp4> --reference <登録参照.json> --output <新しい出力先>
```

出力は`rectified.mp4`、最初に補正できたフレームの`rectified_preview.png`、各フレームの登録ID・元PTS・有効フラグ・失敗理由・変換行列を記録する`rectification.csv`、入力のコピーと`summary.json`です。既存の出力先や元動画を上書きしません。GUIでも「AR正対補正のみ」を選べます。

既定は720×720画素です。`--size`に128〜2160の偶数、`--start`・`--end`に元動画の秒、`--stride 4`に4フレーム間引きを指定できます。動画はH.264 MP4で音声を含みません。PyAVを使い、出力先頭を0秒として元PTSの時間差を保ち、保存後に再復号して枚数・時刻を照合します。`output_pts_origin_video_s`を足すと元動画の時刻へ戻せます。[PyAVの時刻の説明](https://pyav.basswood.io/docs/stable/api/time.html)も参照してください。

補正できないフレームは黒背景の`AR UNAVAILABLE`とし、フレーム自体を削除したり、前回の変換・補間で隠したりしません。全フレームが不成立なら失敗として理由と動画を残します。盤面の平面補正なので、機体の高さによる視差やレンズ歪みの校正、機体座標の精度検証とは区別してください。

## 機体外観学習と小型CNNの試験

`train_turn_appearance.py`は保存動画から、同フレームの盤面ARと前後の色ペアを観測できる画像を抽出し、機体外観モデル・64×64画像・前後点の擬似正解・入力動画のハッシュを保存します。A/B・前後色・盤面基準が違う試行は混ぜません。色検出から作った擬似正解なので、画像中の点が本当に目印に合っているかも確認してください。

```powershell
python tools/analysis/train_turn_appearance.py --trial <学習用trial.json> --geometry <機体JSON> --output <新しい学習フォルダー> --samples-per-trial 24
python -m pip install --target .tools/vision_python312 --index-url https://download.pytorch.org/whl/cpu -r tools/requirements-vision-cnn.txt
python tools/body_keypoint_cnn.py --patches <学習フォルダー> --output <新しいkeypoints.pt> --steps 400 --robust
```

CPU版PyTorchはCNNを使うときだけ必要です。上記はPython 3.12の例で、保存先末尾は実行するPythonの版に合わせます。`--robust`は回転、減光、方向の異なるブレ、片側の遮蔽を加え、隠れた点の推定も学習します。学習・解析はPC内で行い、BLEやカメラの操作はありません。モデル・既存の出力先は上書きしません。

独立動画の比較には`evaluate_body_detectors.py`を使います。manifestの`variants`に`["legacy","learned_assist"]`、`cases`に`label`・`kind`（`static`/`motion`）・`video`・`geometry`・`reference`・`window: [開始秒,終了秒]`・`stride: 4`を指定します。必要なら`warmup_start`へ報告区間より前の開始秒を指定し、両方式に同じ追跡準備区間を与えます。`review_frames`は元動画のフレーム番号です。

```powershell
python tools/analysis/evaluate_body_detectors.py --manifest <比較条件.json> --appearance-model <appearance.npz> --cnn-model <keypoints.pt> --output <新しい比較フォルダー>
```

学習動画を評価へ使うとハッシュ照合で拒否します。各フレームのAR登録、色ペアの観測数、補った数、静止時の散らばり、位置・角度の飛び、処理時間、比較画像を保存します。被覆率だけでは精度を証明できません。旋回や欠測を含む別動画の目印位置と照合し、誤認があるモデルを通常用へ登録しないでください。CNN単独では盤面座標を決めず、直前姿勢は画像を切り出す位置にだけ使い、現在画像の機体外観・前後間隔・変位の条件も検査します。盤面登録の品質条件は変えません。

GUI用の一覧は次の形式です。`model_sha256`等には実ファイルのSHA-256、`validation`には独立評価結果へのパスと採否理由を入れます。ファイルパスはリポジトリ相対、または絶対パスです。登録しても通常の解析方式は変わりません。

```json
{
  "schema_version": 1,
  "models": {
    "B:green:red": {
      "identity": ["B", "green", "red"],
      "model_file": "tools/logs/pixel_camera/models/B/appearance.npz",
      "model_sha256": "<SHA-256>",
      "keypoint_model_file": "tools/logs/pixel_camera/models/B/keypoints.pt",
      "keypoint_model_sha256": "<SHA-256>",
      "require_colour_pair": false,
      "fallback_only": true,
      "center_margin_mm": 0,
      "validation": {"report": "<独立動画の評価JSON>", "decision": "<判断根拠>"}
    }
  }
}
```

`keypoint_model_file`とそのハッシュはCNNを使わなければ省略できます。`require_colour_pair: true`では片側や両側の欠測を補いません。測定用の実観測値と学習推定を区別し、学習点・補間点を観測率へ加算しません。機体Bは前緑・後赤の動画に後黄の設定を使うと追跡できないため、学習前に実際の色と設定を照合してください。

### Google Colab で認識率を比較・GPU 学習する

`evaluate_body_detectors.py` は `--base-dir` と `--device`（`cpu`/`cuda`/`auto`）に対応し、保存済みの `video_sha256` を使って Linux / Google Colab 環境でも学習動画と評価動画の独立分割チェックを行います。`body_keypoint_cnn.py` も `--device cuda`（保存時はローカル CPU 互換テンソル）に対応しています。

Colab へ持ち出す際は `export_colab_recognition_bundle.py` で評価 manifest・動画・設定・任意モデル・学習パッチを相対パス構成（または `.zip`）へ出力し、[tools/colab/mouse_recognition_colab.ipynb](../colab/mouse_recognition_colab.ipynb) で既存方式と AR 正対盤面グローバル CNN（`colab_board_detector.py`）を同じ AR 登録条件で横並び検証できます。手順とボトルネック整理は [tools/colab/README.md](../colab/README.md) を参照してください。

## Legacy

次のコードは旧Lstart/Lend方式との比較または回帰テストのために残している。
新しいターン長の決定には使用しない。

- `encoder_only_turn_lengths.py`
- `turn1600_variable_speed_lengths.py`
- `video_turn_overlay.py`
- `video_turn_simulator_compare.py`
