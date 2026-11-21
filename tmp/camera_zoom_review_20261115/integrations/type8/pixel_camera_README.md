# TYPE8からPixelの240 fpsカメラを使う

TYPE8のPC側ツールから、Debug_cameraで作成したAndroidアプリ・Pythonライブラリを使います。
ファームウェアや既存の走行パラメータは変更しません。この入口はロボットのコマンドを送信しません。

## 最初の確認

TYPE8のルートで実行します。

```powershell
py -3.11 tools/pixel_camera_capture.py check
```

同じgitフォルダー配下のDebug_cameraを自動検出し、その.venvを使用します。
TYPE8側のPython/Bleak/NumPy環境にカメラ用パッケージを追加する必要はありません。
別の配置では --camera-repo C:/path/to/Debug_camera、または環境変数PIXEL_CAMERA_REPOで指定します。
カメラ側の初回環境構築はDebug_cameraのrequirements.txtを参照してください。

## カメラGUIを開く

tools/Start_Pixel_Camera.cmdをダブルクリックするか、次を実行します。

```powershell
py -3.11 tools/pixel_camera_capture.py gui
```

Pixelは既存のPixel Camera Probeアプリを使います。USB接続し、端末の「PC接続を開始」とPC側の「接続 / 再接続」で接続します。
端末URLは http://127.0.0.1:8765 のままです。カメラGUIの開始・停止・履歴・コマ送り検査をそのまま使えます。

接続状態だけを照会する場合:

```powershell
py -3.11 tools/pixel_camera_capture.py status
```

## TYPE8のBLEと同時に使う

カメラと既存BLE中継は、どちらも既定でPCの8765番を使用します。
同時使用時は、BLE中継とBLE GUIの両方を8766番に指定します。

ターミナル1:

```powershell
$mousePython = Join-Path $env:LOCALAPPDATA 'Programs\Python\Python311\python.exe'
& $mousePython tools/ble/mouse_ble_gateway.py --port 8766
```

ターミナル2:

```powershell
$mousePython = Join-Path $env:LOCALAPPDATA 'Programs\Python\Python311\python.exe'
& $mousePython tools/ble/mouse_gui_ble.py --gui debug --port 8766
```

連続ターンGUIは --gui turnpattern に置き換えます。
この文書の作成・check・カメラ録画試験だけでは、上記のBLE接続・走行を実行しません。
操作・電圧ガード・前センサ待ちは、既存のBLE READMEと単体GUIの手順に従ってください。

## TYPE8の試験情報と一緒に録画する

機体の配置、接続、ターン等の設定を済ませてから、ターミナル3で録画します。

```powershell
py -3.11 tools/pixel_camera_capture.py record --seconds 30 --mouse A --label long_r180
```

1. カメラ開始要求を送る。
2. 録画ID・操作クライアント・RECORDING状態が一致するまで待つ。
3. RECORDING_READYが表示されたら、TYPE8の既存GUIで動作を開始する。
4. 指定秒数で録画を自動停止し、MP4確定・転送・照合・端末動画削除通知を待つ。
5. PCで全フレームを復号し、試験結果を tools/logs/pixel_camera/*.capture.json に保存する。

1本の録画は1〜30秒です。前センサの手かざしはRECORDING_READYの確認後に行い、動作をその録画時間内に完了させます。
--mouseと--labelはユーザーが付ける条件メモです。実機A/Bやターン設定をカメラから確認した値ではありません。
例のA/long_r180を、実際に撮る機体・動作に合わせて指定してください。省略時の機体はunknownです。

動画・撮影JSON・CSVの実保存先はカメラGUIに表示されるフォルダーです。
TYPE8側の.capture.jsonは、そのパス・録画ID・SHA-256・復号結果を参照します。動画を二重にコピーしません。
TYPE8のGet log CSVで保存したCSVと.settings.jsonも保持し、対応する.capture.jsonと一組で管理してください。
ログの対応は自動推測せず、type8_log_csvは未設定で保存します。

Ctrl+Cや開始確認の失敗では、所有している録画だけを停止し、可能なら転送完了を待って失敗状態を記録します。
開始応答が失われた場合、動作や録画開始を自動再送しません。
物理的に独立した240コマ/秒の撮影証明は、従来どおり未検証です。

## 同じPythonスクリプトから使う場合

既存のPixelCameraClientを使えます。TYPE8の動作開始は、開始要求の受付後ではなく、録画中になったことを確認してから行います。
このファイルのrecord_trialには任意のon_readyコールバックがあります。明示的に渡した場合だけ呼びます。
この入口は既存GUIを自動クリックせず、機体や走行条件を勝手に選択しません。

## 検証範囲

録画ID・所有者・実際の録画状態の照合、開始失敗時にコールバックを呼ばないこと、
開始要求を重複送信しないこと、転送回復と失敗レポートをホストテストで確認しています。
TYPE8の実機走行・BLEとの同時接続による撮影は、走行条件を確認してから別に実施します。
