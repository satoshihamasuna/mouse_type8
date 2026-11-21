# Pixel Camera System — USB遠隔録画

Pixel 4aの背面Camera2とH.264エンコーダを調査し、1280×720・固定240 fpsの実時間MP4を試験するAndroid Studioプロジェクトです。

**USBでのPC遠隔録画、自動転送、サイズ・SHA-256照合後の端末動画削除、共通Python API、PySide6 GUIを実装しました。** Pixel 4a (5G) / Android 14で往復の実機動作を確認しています。物理的な240 fpsの時間分解能とPixel 4a (4G)は未検証です。Wi-Fiは今回の開発対象から外しました。

PCでは **Start_Pixel_Camera.cmd** をダブルクリックしてGUIを開けます。通常の手順とPython APIは **[USB遠隔録画の使い方](docs/USB_REMOTE.md)** を参照してください。接続済みなら「録画開始」から操作できます。既定保存先はrecordings/remote/です。

10秒の起点を最初の映像PTSに修正し、GUIとCLIから繰り返し撮影・全フレーム復号・自動レポートに対応しました。[連続試験の使い方](docs/USB_REMOTE.md#連続撮影試験)を参照してください。

保存済み動画のコマ送り・PTS表示・24コマの一覧画像保存も利用できます。**Start_Video_Review.cmd** で検査画面を開くか、録画GUIの履歴から「選択した動画をコマ送りで検査」を押します。[映像の検査手順](docs/USB_REMOTE.md#コマ送りと連続フレームの保存)を参照してください。

[初回のカメラ実機試験](docs/DEVICE_TEST_20261108.md)と[USB遠隔操作の実機試験](docs/USB_TEST_20261108.md)を記録しています。以下は端末内の手動試験と測定の詳細です。

## 実装済みの範囲

- OSリリース・APIレベル、機種、ビルドfingerprint、セキュリティパッチを実機から取得。
- Camera2の全カメラの対応レベル、高速撮影サイズ・fps範囲、通常時のISO・露光・EV・AF・AWB情報をJSON表示・保存。
- 背面カメラとH.264エンコーダを選択。ハードウェアアクセラレーション、Surface入力、サイズ/fpsおよびフォーマット対応、ビットレート範囲、profile/level、性能推定値を調査。
- SESSION_HIGH_SPEEDとCameraConstrainedHighSpeedCaptureSessionを使う試験。createHighSpeedRequestList → setRepeatingBurst。
- 720p・[240,240]の申告がないカメラでは試験を開始しない。暗黙に120 fpsや30 fpsへ変更しない。
- MediaCodecのSurfaceに直接出力し、MediaMuxerでMP4保存。音声、ImageReader、プレビュー、画像のCPUコピーを使用しない。
- エンコーダに240 fpsを指定し、実際の出力PTSから絶対時刻の原点だけを除去。フレーム間隔を30 fpsへ引き伸ばさない。
- 最初の映像フレームから1〜30秒の自動停止と手動停止。指定時間以上のPTSを持つ末尾フレームだけを保存対象から除外し、EOSまで出力を回収。除外数はrecording_windowに記録。時刻の引き伸ばしや補間は行わない。2秒未満はタイミング評価には短すぎるため判定保留。
- センサー時刻、撮影開始時刻、エンコード済みPTS、フレーム数、撮影失敗、バッファ欠落、熱状態、ファイルサイズ、SHA-256を保存。
- ストレージ残量から録画開始を制限し、録画中に64 MiB未満になれば停止。
- 停止時にカメラの残り出力とエンコーダEOSを待機。MP4確定とMediaExtractor検査に成功したファイルだけを正式な.mp4へ変更。異常時は.partial.mp4とレポートを保持。
- 権限拒否時の説明・アプリ設定への導線。画面回転で状態を保持。端末内の手動試験はホーム移動・画面消灯で停止し、USB遠隔録画はcamera型Foreground Serviceで管理。

高速撮影セッションはAE・AF・AWBが自動制御に上書きされます。端末内のPhase 1試験UIではISO、露光時間、手動フォーカス、WB、EVの変更を提供していません。2026-11-09版のPC遠隔操作では自動露出の補正値（EV）を変更できます。Pixel 4a (5G)で−2/−1/＋1 EVの要求と撮影結果メタデータの一致を確認しました。手動ISO・露光時間の指定とは別です。調整用の任意AF固定は、連続動画AFの合焦後にトリガーを1回だけ送り、CaptureResultの固定確認までRECORDINGを通知しません。[AF固定の手順と制約](docs/USB_REMOTE.md#合焦後のaf固定調整用任意)を参照してください。通常モードの対応範囲を高速撮影でも利用可能とは扱いません。AE等の実測値はsensor.csvに残します。設定と確認方法は[USB遠隔録画](docs/USB_REMOTE.md#露出補正ev)を参照してください。

## Gitで管理する範囲

Android・Python・GUI・TYPE8連携用のソース、テスト、Gradle Wrapper、手順書を管理します。録画・検査レポートの`recordings/`、接続コードを含む`.runtime/`、仮想環境`.venv/`、SDKパスの`local.properties`、IDE設定、ビルド生成物は`.gitignore`で除外しています。録画データはGitとは別にバックアップしてください。

別のPCにcloneした場合はAndroid SDKとPython環境を用意し、[USB遠隔録画のセットアップ](docs/USB_REMOTE.md)に従って仮想環境と接続コードを作り直します。

変更を保存するときは、ルートで内容を確認してからコミットとpushを行います。

```powershell
git status
git diff
git add <変更したファイル>
git commit -m "変更内容の説明"
git push
```

## ビルド

既存のGradle/Compose構成を維持し、minSdkを30に変更しました。Android 13（API 33）とAndroid 14（API 34）でインストール可能な設定です。compileSdk/targetSdkは既存プロジェクトの37です。機種名からOSやROMの種類を推定せず、取得した値を確認してください。

Android Studioでルートを開き、Gradle Sync後に次を実行します。

```powershell
.\gradlew.bat :app:assembleDebug :app:testDebugUnitTest :app:lintDebug
```

デバッグAPK: app/build/outputs/apk/debug/app-debug.apk

Android Studioに設定済みのGradle JDKとSDKを使用してください。CLIのJAVA_HOMEに別のJDKが設定されている場合は、Android StudioのGradleツールウィンドウから実行する方法もあります。

## 実機で最初に行うこと

1. Pixel 4aで開発者向けオプションとUSBデバッグを有効にする。
2. USBでPCへ接続し、端末のRSA許可ダイアログでこのPCを許可する。
3. adb devices -lで状態がdeviceになることを確認する。unauthorizedの場合は端末の許可操作が必要。
4. Android StudioからappをRunする。アプリ名は「Pixel Camera Probe」。
5. 「権限を許可・機能を調査」を押し、カメラ権限を許可する。
6. 機種、Androidバージョン、背面カメラの720p240固定の申告、H.264エンコーダの申告を確認する。
7. 720p240を申告する背面カメラとSurface対応のハードウェアエンコーダを選択する。
8. 明るく、フリッカーが少ない環境で、独立した既知の時間基準を持つ動く被写体を向け、10秒の試験録画を行う。安全に固定した回転マーカーや計測済みのLEDパターンが候補。60 Hz画面のストップウォッチだけでは約4.17 ms間隔の証明にはならない。
9. 「停止処理・MP4確定中」が終わるまで画面を開いたままにし、結果とエラーを確認する。
10. 以下の手順でPCに取り出し、デコードして検査する。明所・暗所で比較し、10秒を複数回、その後30秒でも確認する。

ここまでの手順は端末内の手動試験です。通常のUSB遠隔録画では、アプリが表示された状態で「PC接続を開始」してcamera型Foreground Serviceを起動し、その後PCから操作します。Android 14での継続処理は実装済みです。

## PCへ検証データを取り出す

Python 3.10以降を使用します。取り出しツールとffprobe検証ツールに外部pip依存はありません。補助的なOpenCVデコードツールのみopencv-pythonを必要とします。Android SDKのplatform-toolsをPATHに追加するか、--adbでadb.exeを指定してください。

```powershell
py -3 tools/pull_probe.py --out recordings/test01
```

PATHにadbがない場合:

```powershell
py -3 tools/pull_probe.py --adb "$env:LOCALAPPDATA/Android/Sdk/platform-tools/adb.exe" --out recordings/test01
```

複数端末がある場合は--serial SERIALを追加してください。**録画・確定処理が完了してから実行します。** デバッグAPKのrun-asでアプリ内部領域を読み出します。端末上のファイルは削除しません。既存の異なる録画ファイルは上書きせずエラーにし、更新されたcapabilities.jsonはハッシュ付きの別名で保持します。

Windows PowerShell 5のバイナリリダイレクトによる破損を避けるため、adb exec-outの出力はPythonがバイナリのまま受け取ります。取り出しの終了だけで転送の整合性を保証せず、次の検証も行ってください。run-asによるこの取り出しはデバッグビルド専用です。

保存先は端末のアプリ専用内部領域 files/probe/ です。

| ファイル | 内容 |
|---|---|
| capabilities.json | 最後に調査した機能と端末情報 |
| 日時_240fps_ID.mp4 | EOS・MP4検査まで成功した動画 |
| 日時_240fps_ID.partial.mp4 | 未確定または異常終了した動画。勝手に削除しない |
| 日時_240fps_ID.json | 撮影条件、機能スナップショット、各計測値、エラー、SHA-256 |
| 日時_240fps_ID.sensor.csv | frame number、SENSOR_TIMESTAMP、露光時間、frame duration、ISO、AE/AF/AWB状態 |
| 日時_240fps_ID.started.csv | onCaptureStartedのframe numberと時刻 |
| 日時_240fps_ID.pts.csv | エンコーダ出力PTS。単位ns |

アプリのアンインストールや「ストレージを消去」で内部ファイルは失われます。検証中は取り出し前に行わないでください。プロセス強制終了や電源断ではMP4確定を保証できません。起動途中の状態PREPARINGと.partial.mp4が残る場合があります。

## PCでフレームを検証する

[FFmpeg公式サイト](https://ffmpeg.org/download.html)から案内されるWindows配布を導入し、ffprobeをPATHに追加してください。これはAndroidアプリのビルドには不要です。

取り出した「録画ごとのJSON」を指定します。capabilities.jsonはこのコマンドの入力ではありません。

```powershell
py -3 tools/verify_recording.py recordings/test01/20261108_120000_240fps_01234567.json
```

実際のファイル名に置き換えてください。--ffprobeで実行ファイルのパスを指定できます。SHA-256・サイズを端末側JSONと照合し、ffprobeのshow_framesでデコードしたフレームの時刻・件数を調べます。MP4ヘッダのavg_frame_rate/r_frame_rateは参考値としてのみ表示します。

追加でデコードした画素の完全一致を調べる場合:

```powershell
py -3 tools/verify_recording.py recordings/test01/20261108_120000_240fps_01234567.json --ffmpeg ffmpeg
```

静止した被写体では同じ画素のフレームが生じることがあります。画素の差だけでも真の240 fpsは証明できないため、既知の時間基準と動きの確認を組み合わせます。FFmpegを使用する実ファイル検証は実機録画取得後に必要です。Pythonテストでは外部デコーダをモックしています。

ffprobe CLIが未導入でOpenCVが利用可能な環境では、補助的に全フレームのデコード件数・サイズ・SHA-256・完全一致画素を検査できます。

```powershell
py -3 tools/decode_with_opencv.py recordings/test01/20261108_083824_240fps_fe274b23.json
```

OpenCVのFPSや再生位置プロパティは240 fpsの達成判定に使いません。この補助検査だけではフレームごとの時刻と物理的な240 fpsを確認できません。

## 期待される結果と判定

APIの申告、高速セッション作成、エンコーダ動作、実測タイミング、被写体の時間分解能を別々に評価します。

- 240 fps相当の時刻間隔は約4.167 ms。10秒撮影なら約2400フレーム。実測239.8 fpsでは約2398フレームとなる。録画時間は最初の映像PTSを起点とし、カメラ準備時間は含めない。
- 全フレームのセンサー通知がある場合、SensorとEncodedの時刻からのfpsが240の±2%以内で、長い間隔・撮影失敗・バッファ欠落がなく、期間・件数・実時間がおおむね一致する場合はTIMING_CONSISTENT_REQUIRES_VISUAL_VALIDATION。
- センサーと動画の期間差は5%、全フレーム通知時の件数差は3フレームまで。制御用実時間との差は0.5秒または10%の大きい方まで。検証用の閾値であり、ハードウェア性能の保証値ではない。
- TIMING_GAPS_DETECTEDは1.5フレーム分を超える間隔、不正な時刻、撮影失敗などを検出した状態。
- INCONCLUSIVE_BATCHED_SENSOR_METADATAはバッチ単位のセンサー通知しか得られない状態。例えば8フレームごとに通知される30回/秒を、30 fps撮影やフレーム落ちと誤認しない。frame numberの進行から推定したfpsは独立したフレームごとの実測値とは区別する。Python側でもerrorsと判定保留のwarningsを分け、保留中は終了コード1を返す。
- その他のINCONCLUSIVE_*は情報不足や時刻系列の不一致。成功と見なさず、CSVとMP4を調べる。
- TARGET_RATE_NOT_OBSERVED / WALL_CLOCK_DURATION_MISMATCHは目標fpsまたは実時間との整合が取れない状態。
- RECORDING_ERRORではerrorsと.partial.mp4を保持する。
- PC検証コマンドはタイミング検査成功時に終了コード0、検査不一致で1、入力不足やデコーダ起動失敗で2を返す。

Camera2の高速撮影結果はバッチ化され、一部タイムスタンプがフレームワークで補間されることがあります。**タイムスタンプの整合だけで物理的に独立した240フレーム/秒を達成したとは判定しません。** JSONのphysical_240fps_verifiedは常にfalseです。既知の時間基準を撮影した映像をコマ送りし、フレームの重複、対象の進行量、約4.17 msの時間分解能を確認して初めて実機検証結果を追記してください。

## 対応しない場合の切り分け

| 結果 | 調べる内容 / 次の選択肢 |
|---|---|
| 高速撮影capabilityなし | 選択カメラ、実OS/ROM、Camera HAL。標準Camera2ではこの経路を使えない |
| 720p [240,240]なし | JSONのhigh_speed_modesを確認。申告される120 fps等を代替候補として報告し、別モードとして実装する |
| サイズ/fpsをエンコーダが非対応と申告 | まず別のハードウェアエンコーダを確認。「申告がない場合も試験」で実際のconfigure可否を追加調査できるが、成功は保証しない |
| configure/セッション作成失敗 | errors、codec名、profile/level、Surface、OS情報を保存。別codec、ビットレート・形式の比較試験が必要 |
| セッション成功だがフレーム不足 | センサーCSV・PTS・デコード件数・実時間・熱状態を比較。明るさ、発熱、ストレージを変えて再試験 |
| Camera2では不可だが純正カメラでスローモーション可能 | ベンダー独自経路の可能性。第三者アプリで実現可能とは扱わない |

## テスト

```powershell
.\gradlew.bat :app:testDebugUnitTest :app:lintDebug :app:assembleDebug
py -3 -m unittest discover -s tools/tests -v
```

Kotlinテスト: 実測240 fps、30 fpsへのスロー化、フレーム欠落、時刻逆転、コールバック欠如、出力期間の不一致、実時間との不一致。
Pythonテスト: 30 fps検出、メタデータの参照先制限、ハッシュ不一致、240 fpsを物理検証済みと誤認しないこと、既存動画の保持。

端末のカメラ、コーデック、権限拒否画面、画面回転、ホーム移動、低ストレージ、異常切断、発熱を伴う繰り返し撮影は実機テストが必要です。

## 次の段階

Phase 1の実機レポートを基に採用するcamera ID、codec、fps、ビットレートを決定します。後続段階の設計と検証条件はdocs/ROADMAP.mdを参照してください。このリポジトリで未実装の機能を動作確認済みとは扱いません。

公式API:
- [CameraConstrainedHighSpeedCaptureSession](https://developer.android.com/reference/android/hardware/camera2/CameraConstrainedHighSpeedCaptureSession)
- [CameraDeviceの高速セッション制限](https://developer.android.com/reference/android/hardware/camera2/CameraDevice)
- [MediaCodecの能力と性能推定](https://developer.android.com/reference/android/media/MediaCodecInfo.VideoCapabilities)
- [ffprobe](https://ffmpeg.org/ffprobe.html)


## ズーム実験用コピー（2026-11-15）

TYPE8の約1500 mm撮影で盤面と目印を大きく記録するための隔離コピー。元のDebug_camera作業ツリーは変更していません。
Python APIで configure(zoom_ratio=2.0) を指定できます。省略は現在値を維持、サービス起動時は1倍。端末の対応範囲・request keyを検査し、未対応時は拒否します。録画ではCaptureResultのzoom ratioが連続3サンプル一致するまでRECORDINGを通知しません。反映なし・反映後の喪失はエラー。sensor.csv末尾にzoom_ratio_result、撮影JSONに要求・結果・確認状態を保存します。240 fpsの物理的独立性や画質向上をこの確認だけで保証しません。
根拠: Android公式CaptureRequest.CONTROL_ZOOM_RATIO / CameraCharacteristics.CONTROL_ZOOM_RATIO_RANGE。https://developer.android.com/reference/android/hardware/camera2/CaptureRequest#CONTROL_ZOOM_RATIO
まず機体OFFで1倍/2倍の静止撮影を比較し、新しい盤面参照と認識精度を検証してから走行へ進みます。APKは試験用で、実機適用・画質改善は未確認です。
