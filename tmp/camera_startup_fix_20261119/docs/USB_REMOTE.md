# USB遠隔録画・照合後の自動削除

AndroidアプリとPC側サーバーをADB reverseで接続し、共通のPixelCameraClient APIをGUIとPythonから使います。

## 使用範囲

PCクライアントは接続済みのカメラを再利用し、録画のたびのADB設定を省きます。明示した端末serialの初回選択と、接続が失われた場合はUSBを設定し直します。接続待ちの一時的な状態照会失敗だけを再確認し、認証エラーはその場で表示します。
GUIは録画状態を250 ms間隔で照会し、履歴は別処理で5秒ごとに読み込みます。履歴読込みによって録画状態の表示を待たせず、切断前・操作前の古い応答は画面へ反映しません。開始・停止の重複クリックを抑止し、停止要求の応答が失われても同じ録画へ停止を再送しません。起動だけでは接続や録画を始めません。

- 実機: Pixel 4a (5G) / Android 14
- 撮影: 背面、1280×720、固定240 fps、H.264、1〜30秒
- AE/AF/AWB: 自動。PCから自動露出の補正値（EV）を指定可能。手動ISO・露光時間・ピント固定は未対応
- 通信: USBのADB reverse、WebSocketコマンド、HTTP動画アップロード
- PC保存: MP4、撮影JSON、センサーCSV、撮影開始CSV、PTS CSV
- 元動画削除: PCでサイズ・SHA-256照合と保存に成功した後だけ
- 旧Phase 1の手動試験動画: 自動転送・自動削除の対象外
- プレビュー、30秒を超える録画と分割保存: 未対応
- Wi-Fi: 今回の開発対象外

fpsと時間分解能の区別はREADMEのPhase 1検証手順を参照してください。今回の遠隔操作機能を使ってもphysical_240fps_verifiedをtrueにはしません。

## 初回セットアップ

Python 3.11以降を使います。このPCではpy -3の既定が古いAnaconda Python 3.8だったため、専用環境を作成しました。

```powershell
py -3.11 -m venv .venv
.venv/Scripts/python.exe -m pip install -r requirements.txt
```

検証環境で導入した主なバージョン: aiohttp 3.14.5、PySide6 6.12.0、PyAV 16.1.0（FFmpeg同梱のWindows wheel）。Python 3.11に対応する16系をQA依存に指定しています。Androidの通信にはOkHttp 5.5.1を使用します。

Android StudioからappをRunして最新版をインストールします。

## 普段のGUI操作

1. USBデバッグを有効にしたスマホを接続し、アプリを開く。
2. PCでStart_Pixel_Camera.cmdをダブルクリックするか、次を実行する。

```powershell
.venv/Scripts/python.exe -m pixel_camera.gui
```

3. 保存先を選び、GUIの「接続 / 再接続」を押す。PCサーバーを起動し、ADB reverseを設定する。
4. 初回のみGUIの接続コードをスマホへ入力し、「PC接続を開始」を押す。以後コードはスマホに保存される。PCサーバーが既に起動していれば先にスマホで開始してもよい。
5. GUIが「スマホ接続済み」になったら秒数を選び、「録画開始」を押す。必要なら「録画停止」で早めに停止する。
6. MP4確定後に転送・照合が進み、撮影履歴に保存先と「削除済み」が表示される。履歴のダブルクリックでPCの動画を開ける。

保存先の既定値はプロジェクトのrecordings/remote/です。別の保存先に切り替えるときは、録画・転送が終わってから「PCサーバーを停止」し、保存先を変更して再接続してください。稼働中サーバーと違う保存先を指定すると、書き込み先を黙って変えずエラーを返します。

「操作クライアントを切断」やGUIのウィンドウを閉じても、共通サーバーは録画・転送を継続します。スマホの「PC接続を停止」は録画停止処理を待ってサービスを終了します。未転送動画は保持します。

接続コードは.runtime/broker_8765.jsonに保存されます。.runtime/はバージョン管理対象外です。このファイルを共有すると、そのコードを知るローカルクライアントが操作可能になるため、ソース配布には含めないでください。

## Pythonスクリプト

```python
from pixel_camera import PixelCameraClient

camera = PixelCameraClient(connection="usb")
try:
    camera.connect()
    camera.configure(width=1280, height=720, fps=240)
    camera.start_recording(duration_s=10)
    result = camera.wait_for_transfer(timeout=120)
    print(result.file_path)
    print(result.frame_count, result.frame_count_source)
    print(result.source_deleted)
finally:
    camera.disconnect()
```

実行:

```powershell
.venv/Scripts/python.exe examples/record_10s.py
.venv/Scripts/python.exe examples/manual_stop.py
.venv/Scripts/python.exe examples/continuous_recording.py
```

自動停止は最初の映像PTSを0秒として測ります。例えば10秒指定では0以上10秒未満の映像を保存し、カメラ準備時間は含めません。指定時間以上の末尾フレームは意図的に除外してrecording_window.discarded_tail_framesへ記録します。MP4の長さには最後の1フレーム分の時間も含まれるため、PTSの最初と最後の差とは約1フレーム異なります。control_elapsed_secondsは従来通りカメラへの要求開始から停止までの時間です。

start_recording()の省略時は30秒で自動停止します。元の希望APIのように開始後にtime.sleep(10)してstop_recording()を呼ぶこともできます。

stop_recording()は停止要求の受付応答です。MP4確定、転送、削除の完了を意味しません。wait_for_transfer()は既定でPC確定と端末の元動画削除の通知まで待ちます。PCへの確定保存だけを待つ場合はrequire_source_deleted=Falseを指定します。

frame_countはAndroidエンコーダが出力した映像サンプル数で、frame_count_sourceにその根拠を明記します。PCで独立してデコードした件数や物理的な撮影fpsとは区別します。

- get_status(): 接続、録画状態、経過時間、容量、未転送数、転送進捗・速度
- get_capabilities(): 実機のCamera2/MediaCodec対応情報
- get_pending_files(): 端末に残っている遠隔録画ファイル
- history(): PCに保存・照合した履歴
- configure(): 1280×720・240 fpsとexposure_compensation_stepsを受け付ける
- PixelCameraError.code: BUSY、NOT_RECORDING_OWNER、TRANSFER_TIMEOUTなど
- start_recording(command_id=...): 同じIDの再送は同じ録画IDを返し、重複撮影しない

GUIとスクリプトの各インスタンスは別のclient_idを持ちます。録画中は開始したclient_idだけが停止できます。他のクライアントは監視できますが、開始・設定変更・他人の録画停止は拒否されます。

## 露出補正（EV）

2026-11-09版のアプリでは、録画開始前に自動露出の目標をPCから補正できます。
既存のpixel_camera.guiにはEVの入力欄がないため、Python APIまたはTYPE8のStart_Pixel_Framing.cmdを使います。

```python
camera.connect()
camera.configure(exposure_compensation_steps=-6)  # Pixel 4a (5G): -1 EV
```

値は整数の段階数です。範囲とEV刻みはget_capabilities()の対象カメラのev_steps_range、ev_stepを確認します。
今回のPixelは[-24, 24]、1/6 EVで、-6が-1 EV、6が+1 EVです。範囲外・非整数・非数値を拒否します。
configureでこのキーを省略すると選択値を維持し、PC接続サービスの再起動時には0へ戻ります。
録画中・転送中には設定できません。設定受付だけで実際の撮影結果を保証しません。

撮影JSONのev_steps_requested、ev_step、ev_steps_result_valuesと、sensor.csv末尾のev_steps_result列で確認します。
従来のsensor.csvの8列は順序を維持しています。露光時間・ISO・AE状態と実画像も合わせて確認してください。
カメラ機能申告のev_effect_verifiedは機種横断で未検証のためfalseのままです。
このPixelでは-2/-1/+1 EVの要求と結果値が一致し、-1と-2では暗くなることも実画像で確認しました。
物理的な240露光/秒や、全補正範囲での画質は未検証です。

## 撮影中の補助ライト

2026-11-09のライト対応APKでは、configureにtorch=True/Falseを追加できます。
Boolean以外と、ライト非対応カメラへのONを拒否します。省略時は選択値を維持し、サービス再起動でOFFに戻ります。
実際に点灯要求を出すのは録画中だけです。設定だけでは点灯せず、カメラを閉じると消灯します。

```python
camera.configure(exposure_compensation_steps=-6, torch=True)
```

撮影JSONのtorch_requestedとflash_mode_result_values、sensor.csv末尾のflash_mode_resultで結果を確認します。
Pixel 4a (5G) / 720p240で要求ON・CaptureResultのFLASH_MODE=2（TORCH）を確認しました。
今回の盤面では左上ArUcoの反射を改善せず、ライトOFFを採用しています。全フレームでの認識や反射改善を保証する機能ではありません。

## 合焦後のAF固定（調整用・任意）

`camera.configure(focus_lock=True)`で、録画開始後に合焦状態が300 ms連続するのを待ち、AF STARTを非反復の高速撮影バーストとして1回だけ送ります。その後はAF IDLEの既存反復バーストへ戻ります。高速撮影の制約を維持し、AFモードをOFFにしたり手動距離へ切り替えたりはしません。

CaptureResultがFOCUSED_LOCKEDになった後でのみPCへRECORDINGを通知します。5秒以内に固定を確認できない場合、NOT_FOCUSED_LOCKEDの場合、固定後にAF状態が変わった場合は録画をエラーにし、未完了資料を保持します。TYPE8はRECORDING確認後にのみ走行する既存の待機処理を使用してください。設定受付だけでは固定確認になりません。

```python
camera.configure(exposure_compensation_steps=-12, torch=False, focus_lock=True)
```

省略時は現在値を維持し、サービス再起動時はFalseへ戻ります。撮影JSONのfocus_lockに要求・最終状態・トリガー直前フレーム番号を記録します。sensor.csvの既存列は保持し、af_mode_result、af_trigger_requested、focus_distance_dioptersを末尾へ追加します。高速撮影の通知はバッチ単位のため、各物理露光の独立検証ではありません。新しいAPKが必要です。機種別の実写検証前は効果を保証しません。

参照: [Camera2高速撮影の対応制御](https://developer.android.com/reference/android/hardware/camera2/CameraMetadata#REQUEST_AVAILABLE_CAPABILITIES_CONSTRAINED_HIGH_SPEED_VIDEO)、[連続動画AFと固定](https://developer.android.com/reference/android/hardware/camera2/CameraMetadata#CONTROL_AF_MODE_CONTINUOUS_VIDEO)。

## 接続方式

PCがサーバー、Androidがクライアントです。WebSocketとHTTPを1つのポート8765にまとめました。

```powershell
adb reverse tcp:8765 tcp:8765
```

PixelCameraClient.connect()がUSB機器選択と上記設定を行います。複数端末がある場合はserial=を指定してください。ADBがPATHになければ、通常のAndroid SDKの配置を探します。異なる配置ではadb=でadb.exeを指定します。

PC側サーバーの既定バインド先は127.0.0.1です。Androidでは平文HTTPを127.0.0.1とlocalhostだけに限定しています。Wi-FiのLAN向け公開・暗号化・ペアリング運用は今回の開発対象外です。SDKのconnection="wifi"はADB設定を省略する接続口ですが、この版のAndroid/GUIでWi-Fi運用を検証済みとは扱いません。

## 障害からの復旧

- USBやPCが切断されても、録画は指定時間まで進んでファイルを確定する。
- PCへ転送できなければ元動画を保持し、接続回復後に自動で再試行する。
- 転送途中の.partはPCに残る。再送は先頭からやり直し、全体を再照合する。
- ACKが失われた場合はPCの録画IDとハッシュを照合し、既存の確定動画を再検証してACKを返す。別の動画として増やさない。
- AndroidはACKのID・サイズ・SHA-256を照合し、さらに削除直前に元動画のサイズ・ハッシュを再確認する。
- Androidは検証済みACKを永続化してから動画を削除し、削除結果も永続化する。削除通知が失われても、再接続時に通知を再送する。
- アプリのプロセスが録画中に強制終了した場合、MP4確定は保証できない。途中ファイルを保持し、起動後は中断エラーとして扱う。自動削除しない。
- コマンドの重複防止履歴はAndroidとPCで直近256件を保持する。PC再起動後でもAndroid側に残るIDは再実行しない。受付直後にプロセスが終了した場合、同じIDを再実行せず、状態を確認して新しいIDで撮影する。

手動回収用のtools/pull_probe.pyも利用できますが、稼働中のPCサーバーによる自動転送とは別のデバッグ用コピーです。

## 検証コマンド

```powershell
.\gradlew.bat :app:assembleDebug :app:testDebugUnitTest :app:lintDebug
.venv/Scripts/python.exe -m unittest discover -s tests -v
.venv/Scripts/python.exe -m unittest discover -s tools/tests -v
.venv/Scripts/python.exe tools/remote_smoke.py
.venv/Scripts/python.exe tools/remote_smoke.py --manual-stop
```

remote_smoke.pyは実機を録画し、PC保存と照合後に新しい遠隔録画の元動画を削除する試験です。旧手動試験動画は対象外です。

ソフトウェアテストはハッシュ不一致、メタデータ保存失敗、ACK再送、PC再起動時の台帳、パスの検証、認証失敗、コマンド重複、クライアントの競合、削除条件を確認します。長時間、容量不足、物理USBの抜き差しは実機で追加検証が必要です。

## 連続撮影試験

GUIで自動停止を10秒、連続試験の回数を20回にして「連続試験を開始」を押します。1本ずつ録画→MP4確定→転送・照合→端末削除通知→PCで全フレーム復号を完了してから次の撮影に進みます。動画の間には転送・検証時間があります。「現在の保存後に中止」は、その1本の確定・検証が終わってから中止します。GUIを閉じた場合も現在の1本を完了してから終了します。

CLIでも同じ試験機能を利用できます。

```powershell
.venv/Scripts/python.exe -m pixel_camera.stability --count 20 --duration 10
```

レポートはrecordings/reports/日時_ID/のreport.html、report.json、runs.csvです。GUIの「試験レポートを開く」からHTMLを表示できます。各回終了時にも保存するため、試験途中の結果を確認できます。エラーを検出したら次の撮影を開始せず、録画IDと原因を記録します。接続・転送失敗時は既存の保持・再送規則が適用されます。

確認する項目:

- 実際に復号できたフレーム数とAndroidエンコーダ・MP4サンプル数の一致
- 復号フレームのPTSから計算したfpsが240の±2%以内、1.5フレームを超えるPTS間隔・逆転・復号破損なし
- 指定秒数との差が50 ms以内。ヘッダの公称fpsだけでは判定しない
- PC動画のサイズ・SHA-256再照合、端末の動画削除通知
- カメラ撮影失敗・バッファ欠落、熱状態、バッテリー温度、転送後の端末空き容量

バッテリー温度はカメラやSoCの温度ではありません。Androidの熱状態がSEVERE（3）以上になった回は失敗として停止します。この試験のPASSEDは保存動画・転送処理の整合を示します。センサー通知が8フレームごとに間引かれる端末では、物理的に異なる240コマ/秒の撮影証明は引き続き判定保留です。

## コマ送りと連続フレームの保存

PCでStart_Video_Review.cmdを開き、「MP4を開く」から転送済みの動画を選びます。録画GUIでは履歴で対象動画を選択し、「選択した動画をコマ送りで検査」を押す方法もあります。録画GUIを更新前から開いている場合は、一度閉じて開き直すと新しいボタンが表示されます。

最初に全フレームを復号して実際のPTSとSHA-256を読み取ります。公称fpsからコマの時刻を作りません。前後のボタン・左右キーで1コマずつ移動でき、フレーム番号（0から）とスライダーでも移動できます。表示は保存された画素の向きです。再生ソフトの回転補正と向きが異なることがあります。

- 「この1コマをPNG保存」: 元の解像度のframe.pngと、そのコマの時刻を保存
- 「ここから24コマの一覧を保存」: 連続する最大24コマのcontact_sheet.pngとreview.jsonを保存
- 保存先: recordings/reviews/日時_ID/。「保存フォルダを開く」から確認可能

review.jsonには元動画のパス・サイズ・SHA-256、復号件数、各コマの元PTS・time_base・相対時刻・前コマとの間隔を記録します。動画の途中で読み出す場合も手前のキーフレームから復号し、解析時と同じPTSに一致したコマを表示します。解析後に元動画が変更された場合や、未確定の.partial.mp4では検査を中止します。元動画の内容や撮影JSONは変更しません。

CLIで検査画面を開く例:

```powershell
.venv/Scripts/python.exe -m pixel_camera.review recordings/remote/20261108_210911_240fps_c4703bcd.mp4
```

画面を開かず、120コマ目から24コマの一覧を保存する例:

```powershell
.venv/Scripts/python.exe -m pixel_camera.review recordings/remote/20261108_210911_240fps_c4703bcd.mp4 --export-sheet --start 120
```

### 物理的な時間分解能の確認

今ある静止場面の動画やPTSだけでは、異なるセンサー露光を毎秒240回取得した証明にはなりません。次の実物の試験では、端末とは別のクロックで動作し、周期や時刻の進み方を別途測定した対象を画面内に写します。異なる時刻を読み取れるLEDの時刻コードや、周期を測定した回転マーカー等が候補です。単純な点滅だけでは同じ状態が何度も現れるため、各コマを区別できるパターンと測定条件を用意します。60 Hz画面のストップウォッチだけを約4.17 ms間隔の証明に使いません。

1. 明るい環境で、独立した時間基準と測定対象を10秒録画する。
2. コマ送りで開始・中間・終了付近を確認し、連続コマの一覧とJSONを保存する。
3. 外部基準の進み方と、動画のPTS・各コマの変化・センサーCSVの露光時間を比較する。約4.17 msごとの情報が得られるか、重複や不連続がないか評価する。
4. 使用した基準、校正方法と不確かさ、照明、解析した録画IDとコマの範囲、判定理由を記録する。

ローリングシャッターでは画像の行によって露光開始時刻が異なります。同じ行付近で基準を読むなど、比較する位置と露光時間も考慮します。[AndroidのSENSOR_TIMESTAMP](https://developer.android.com/reference/android/hardware/camera2/CaptureResult#SENSOR_TIMESTAMP)と[SENSOR_ROLLING_SHUTTER_SKEW](https://developer.android.com/reference/android/hardware/camera2/CaptureResult#SENSOR_ROLLING_SHUTTER_SKEW)を参照してください。

「時間基準の説明」欄は観察条件のメモです。入力だけで物理240 fpsを検証済みにはせず、physical_240fps_verifiedとreference_independently_verifiedはfalseで保存します。外部基準を用意・撮影した実験の完了は、ソフトウェアの自動テストとは別に記録してください。

## 主要ソース

- Android: app/src/main/java/com/example/myapplication/remote/
- Python共通API・サーバー・GUI: python/pixel_camera/
- 実行例: examples/
- 転送試験: tests/test_remote.py

公式資料:
- [camera型Foreground Service](https://developer.android.com/develop/background-work/services/fgs/service-types#camera)
- [aiohttp](https://docs.aiohttp.org/en/stable/web_quickstart.html)
- [Qt for Python](https://doc.qt.io/qtforpython-6/)


### TYPE8探索の長時間録画

2026-11-13版の端末APIは探索連携用に最大300秒を受け付け、statusのmax_recording_secondsへ上限を返します。手動試験と既存GUIの1～30秒入力は維持します。TYPE8の録画付き探索GUIは上限を確認してから開始し、探索終了後の余白を残して録画を停止します。確認済み削除通知の再受信では同じ受領記録を再保存しません。
