# TYPE8からPixelの240 fpsカメラを使う

TYPE8のPC側ツールから、Debug_cameraで作成したAndroidアプリ・Pythonライブラリを使います。
カメラ専用のpixel_camera_capture.pyはロボットのコマンドを送信しません。
撮影と単体ターンを連動させる場合は、下記の撮影タブまたはpixel_camera_turn_capture.pyを明示的に使います。撮影専用の一回許可に対応したファームウェアが必要です。

## 最初の確認

TYPE8のルートで実行します。

```powershell
py -3.11 tools/pixel_camera_capture.py check
```

このPCでpyランチャーがPython 3.11を認識しない場合は、導入済みの実行ファイルを指定します。

```powershell
& "$env:LOCALAPPDATA/Programs/Python/Python311/python.exe" tools/pixel_camera_capture.py check
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

## 調整GUIで撮影と単体走行をまとめる

`tools/Start_Pixel_Tuning.cmd`を開きます。既存の`myshell debug parameter tuner`に「Pixel撮影・結果」「撮影条件」タブを追加しています。
起動・保存済み結果の表示だけでは、接続・設定送信・録画・走行はしません。通常のApply and executeは従来の手かざし経路です。

1. Pixelを通常のカメラGUIからPCへ接続し、録画・転送が終了した待機状態にします。他のBLEツールは切断します。
2. 撮影タブで機体A/B、今回のカメラ高さ[mm]を確認し、「撮影条件」で盤面参照・機体/色検出JSONとHY0020アドレスを確認します。
3. 「撮影用接続を準備」を押します。カメラ8765・BLE8766を使い、5 msの連続CSV保存を開始します。この操作では走行しません。
4. 上部でMotion=turn、ターン名、速度プリセットを選び、Load defaultsで**実機のプリセット**を取得します。初回の未取得欄は空欄です。500調整は実機のpreset:500とvelo:0.5を取得してから編集します。
5. Lend等の必要な項目を編集します。回数1・吸引OFFに限定しています。カメラは固定し、走行範囲・開始位置・向き・停止・手離しを確認して、撮影タブの確認チェックを毎回入れます。
6. 「撮影して1回走行」を押します。実機値と画面値が一致すれば設定再送を省き、変更があればApply相当のsetを1回送ってshowで再照合します。プリセットはここでは読み直しません。
7. 録画上限は既定20秒（必要時は30秒を選択）です。RECORDING_READY後に録画ID・所有者・録画中状態・残り時間・電圧・新しい連続CSVを確認してから、撮影専用の一回実行を送ります。手かざしは不要です。
8. 今回の動作をCSVで観測し、指令停止かつ実測速度20 mm/s以下・角速度0.15 rad/s以下を200 ms以上確認してから、PC・機体・動画の時刻でさらに各1秒以上の余白を待ち、録画だけを停止します。余白中に動けば確認をやり直します。CSVの途絶・欠落・CRC異常などで確認できなければ、Android側の録画上限まで待ちます。
9. 転送・全復号・照合後に速報の停止位置と軌跡を表示します。詳細はバックグラウンドで240 fps全フレームを検証します。接続は次の試行へ引き継ぎます。

早期停止でも、フレーム数・PTS連続性・約240 fps・EOS/MP4確定・サイズ/SHA-256再照合・端末動画削除を検証します。固定秒数の条件は、今回の停止要求と停止後余白、Androidの実際の終了理由、実動画長と最終PTSの照合へ置き換えます。判断根拠・停止要求の成否・上限への退避理由は`trial.json`と`.capture.json`の`recording_stop`へ保存します。
復号した最終PTSはAndroidのMediaExtractorで確認したMP4時刻と照合します。エンコーダのµs時刻とはMP4の実timescaleとサンプル数から丸めの上限を計算して比較し、異なる時刻単位に完全一致を要求しません。動画の時刻や再生速度は変更しません。
停止要求の応答が失われても停止要求は再送せず、転送された動画で結果を確認します。この自動停止は撮影タブに適用し、カメラ単体CLIや従来の`pixel_camera_turn_capture.py`の指定秒数録画は維持します。

実行はタイムアウト・途中送信・開始応答消失でも再送しません。二重クリックや通信待ち中の通常GUIコマンドも追加走行へつなげません。
撮影・設定照合中は操作をロックし、確認チェックは開始時に必ず消費します。自動の連続走行やパラメータ採用は行いません。
機体のログで動作後200 ms以上の実測停止を確認し、速報映像でも停止後の中点位置が安定し、必要動作区間の観測率・ログとの比較条件を満たした場合だけ「速報で確認」と表示します。
確認できない場合は機体の停止・位置を操作者が確認し、実行要求を盲目的に繰り返さないでください。次の試行には新しい設置確認が必要です。

速報は4フレームに1枚（約60 fps）、詳細は全フレームです。実PTSは引き伸ばしません。
速報の未使用フレームまで精度検証済みとは扱わず、表示とJSONの`PROVISIONAL`／`sampled_provisional`で区別します。
間引きでも各使用フレームの盤面補正を独立に行い、無効なフレームへ前の変換を流用しません。
詳細が「要目視確認」になっても、機体中心の絶対位置や高さ補正済み機体yaw、物理的な独立240露光/秒を保証する意味ではありません。

保存先は`tools/logs/pixel_camera/tuning_gui_日時_ID/`です。セッション全体のCSV・raw・shell・追記式health.jsonlと、試行ごとのrunフォルダーを保存します。
各試行のtrial.jsonがD側の動画、撮影JSON、走行CSV、実機settings.json、画面の要求値、盤面・機体プロファイルを結び付けます。
`quick/`と`full/`に別々の解析JSON・相対軌跡CSV・比較図を残し、要求値と実機値は別のJSONに保存します。
set後の照合だけは実機の整数表示1単位までの切捨てを許容し、その差を要求値と実機値の両方に残します。変更なし判定ではこの許容幅を使いません。
通信状態は追記式health.jsonlで照合し、更新中のstatus.jsonを読み取りません。CRCエラー、欠落、走行区間不足では記録を成功扱いにしません。

UIを閉じるとBLE接続を閉じます。依頼済みの保存データの詳細解析は終了まで続きます。
解析PythonはこのPCでは`D:/venv/venv3-11/Scripts/python.exe`を使います。別環境では`TYPE8_ANALYSIS_PYTHON`にNumPy/OpenCV aruco/Pandas/SciPy/Matplotlib環境を指定します。カメラ用PyAV環境は従来のDebug_camera/.venvを使います。

### 画面操作なしのCLI調整

`pixel_camera_tuning_cli.py`はTkの画面を開かず、GUIと同じ設定照合・録画待ち・単体1回・停止後余白・速報／全フレーム解析・一式保存を使います。
接続を保持する常駐処理へコマンドを送り、他のアプリのフォーカスやマウス・キーボードを操作しません。
PixelカメラのPCサーバーは従来どおり必要です。カメラGUIは接続後に最小化できます。BLEを使う既存調整GUIはDisconnectしてから切り替えてください。

リポジトリのルートからカメラ用Pythonで実行します。設定JSONには**今回の機体・カメラ高さ・盤面参照・機体プロファイル**を記入します。
次は形式例です。BLEアドレスとJSONパスを今回の値へ置き換えてください。

```json
{
  "mouse": "A",
  "address": "HY0020のBLEアドレス",
  "height_mm": 800,
  "reference": "tools/logs/pixel_camera/marker_reference.json",
  "geometry": "tools/logs/pixel_camera/body_geometry_A.json",
  "char_delay": 0.08,
  "seconds": 20,
  "exposure_compensation_steps": -15
}
```

露出は端末の整数ステップです。今回のPixelは1/6 EV刻みなので、−15は−2.5 EVです。省略時はカメラ側の設定を維持します。

走行中に盤面全体がぼける場合は、撮影JSONとsensor.csvのAF状態を確認してください。AF固定対応のカメラAPKでは、PC接続後に以下を実行すると合焦後の固定を有効にできます。機体の指令は送りません。アプリのPC接続サービスを再起動するとOFFへ戻るため、再度設定します。CLIの録画開始時のconfigureは、この指定を省略して現在値を維持します。

```powershell
@'
from pixel_camera import PixelCameraClient
camera = PixelCameraClient(connection="usb")
camera.connect()
try:
    print(camera.configure(focus_lock=True))
finally:
    camera.disconnect()
'@ | & '..\..\Debug_camera\.venv\Scripts\python.exe' -
```

設定応答のTrueは要求受付です。AF固定を有効にした録画では、CaptureResultの固定確認までPREPARINGを維持し、未固定・固定失敗・固定解除は録画エラーになります。まず静止試写で撮影JSONのfocus_lock.stateがLOCKEDになることと認識を確認します。AF固定後も全フレームの品質条件は変更しません。高速撮影のセンサー通知はバッチ単位で、独立した240露光/秒を保証しません。

`serve`はカメラ待機とBLE接続を確認し、5 ms連続CSVを開始します。起動・状態照会だけでは走行しません。

```powershell
$cameraPython = '..\..\Debug_camera\.venv\Scripts\python.exe'
$session = 'tools/logs/pixel_camera/cli_session_001'
$config = 'tools/logs/pixel_camera/cli_config.json'

# 接続を保持する処理。別端末でクライアントを使います。
& $cameraPython tools/pixel_camera_tuning_cli.py --session $session serve --config $config
```

非表示で起動する場合は、上の`serve`の代わりに次を使えます。起動ログはセッション内へ保存されます。

```powershell
New-Item -ItemType Directory -Path $session -Force | Out-Null
Start-Process -FilePath $cameraPython -ArgumentList @('tools/pixel_camera_tuning_cli.py','--session',$session,'serve','--config',$config) -WorkingDirectory (Get-Location).Path -WindowStyle Hidden -RedirectStandardOutput "$session/service.stdout.log" -RedirectStandardError "$session/service.stderr.log"
```

別端末で操作します。`preset`は指定ターンのRAM値を初期化するため、保持したい候補は先に保存してください。
機体再起動後に候補へ戻す場合は、対象の500プリセットを選択してから保存候補をApplyします。

```powershell
# 状態照会だけ。実機へコマンドを送りません。
& $cameraPython tools/pixel_camera_tuning_cli.py --session $session status

# 読取りだけならread。初回や再起動後は明示的にpresetを選択します。
& $cameraPython tools/pixel_camera_tuning_cli.py --session $session preset --motion in_l45 --preset 500 --output current.settings.json
& $cameraPython tools/pixel_camera_tuning_cli.py --session $session read --motion in_l45 --output current.settings.json

# 指定項目を変更し、setを1回送った後にshowで照合します。
& $cameraPython tools/pixel_camera_tuning_cli.py --session $session apply --settings current.settings.json --set lstart=19.5 --set lend=25 --output candidate.settings.json

# 今回の設置・停止・手離しを確認した後に、1回だけ撮影・走行します。
& $cameraPython tools/pixel_camera_tuning_cli.py --session $session capture --settings candidate.settings.json --operator-ready --request-id in_l45_001

# 応答が途切れてもcaptureを再送せず、このIDで確認します。
& $cameraPython tools/pixel_camera_tuning_cli.py --session $session status --request-id in_l45_001

& $cameraPython tools/pixel_camera_tuning_cli.py --session $session close
```

`--settings`は実機設定JSONか、`actual_settings`を持つ照合記録JSONを受け取ります。`--set pid.om_kp=...`や`--set feedforward.ff_om_velo=...`も使えます。
`capture`の`--operator-ready`はその1回分だけです。同じ`request-id`は失敗・途中送信・応答消失後も再使用できません。
`--no-wait`は受付後に戻り、処理は継続します。通常は速報・全フレーム解析・一式保存まで待ちます。
待ち時間を超えたり接続が途切れたりしても、自動再送・自動再接続はしません。
撮影記録JSONまたはCLI要求状態JSONの保存中にWindowsのアクセス拒否・共有違反（5/32/33）が起きた場合だけ、保存を最大3回、待ち時間合計0.35秒で再試行します。状態照会中の一時的なファイル競合にも同じ制限を適用します。録画開始・停止や走行指令は再送しません。解消しなければ保存エラーとして失敗し、最終記録の保存に失敗してもカメラクライアントを切断します。
終了コードは通常成功0、要求・通信・撮影失敗1、撮影後の品質確認または一式保存に問題がある場合2です。状態照会のJSONも確認してください。

`service.json`は常駐処理の接続情報、`jobs/`は要求IDごとの受付・実行・結果です。
`run_<request-id>/`に従来と同じ動画索引・CSV・実機設定・quick/full・今回の確認記録を残します。
設定変更・撮影・終了は直列化され、処理中の別コマンドは拒否します。自動のパラメータ採用・連続走行は行いません。

長時間の解析・編集や返答待ちは、機体とHY0020の電源をOFFにして行います。走行・手かざし待機の終了と設定・CSV・動画の保存を確認した時点でOFFを案内し、接続も閉じられる時点で閉じます。`--no-wait`やBLE切断だけでは本体の電源は切れません。保存済みデータの解析が続いていても、そのために通電を維持する必要はありません。電源を再投入するとRAM設定は失われるため、次の測定準備を終えてからONにし、保存済み設定の復元・実機値照合・設置確認を行ってください。

### 動画・CSV・設定をDドライブにまとめる

撮影タブは全フレーム解析の終了後、動画・走行CSV・実機設定・撮影記録・速報/詳細解析を録画IDごとのフォルダーへ自動保存します。
現在の配置では`D:\TYPE8_Camera\trials\録画ID\`です。「結果フォルダーを開く」は保存済みならこのフォルダーを開きます。
解析が不成立でも元資料をまとめ、検査の成否はそのまま残します。一式保存が失敗した場合は画面へ表示し、元資料を保持します。走行指令は追加しません。

- `video.mp4`：動画。既存MP4と同じ実体を参照するハードリンクで、動画容量を二重に消費しません。
- `motion.csv` / `motion.settings.json`：走行全区間の切出しCSVと、その実機設定。
- `trial.json`：D側の動画・CSV・設定を参照する索引。保存済み結果の表示にも使えます。
- `source_trial.json` / `capture.json` / `camera/`：元の試行記録・撮影記録・カメラのメタデータ。
- `quick/` / `full/`：元の解析結果。内容・入力ハッシュ・元の絶対パスを書き換えず保持します。
- `manifest.json`：各ファイルの元パス・サイズ・SHA-256・保存方法。

元の`tools/logs/`と動画保存先は保持します。元試行の`archive.json`がD側を参照します。
`D:\TYPE8_Camera\trials\index.csv`で、機体・動作・設定・結果と動画/CSVの対応を一覧できます。
ハードリンクには元動画と同じボリュームの保存先が必要です。別ボリューム等で作成できなければ自動の動画コピーには切り替えず、元資料を残して失敗を表示します。
保存先の既定値は動画の`recordings/remote`の隣の`trials`で、その他の配置では動画フォルダー内の`type8_trials`です。`TYPE8_TRIALS_DIR`環境変数で変更できます。

保存済みの試行を、カメラ・機体へ接続せず一式保存する例：

```powershell
python tools/pixel_camera_archive.py --trial tools/logs/pixel_camera/SESSION/run_ID/trial.json --root D:/TYPE8_Camera/trials
```

`--trial`を複数指定できます。同じ入力の再実行はハッシュを照合して既存保存を使い、入力や保存済みファイルが違う場合は上書きせず停止します。
試行の資料を後から変更・追加した場合は別の`--root`へ保存して履歴を保ってください。
実行中のGUIには次回起動から適用されます。新しい自動保存処理は解析終了後にだけ動作し、録画・一回実行の手順を変えません。

保存済み結果を接続せずに表示する例:

```powershell
python tools/pixel_camera_tuning.py --review-trial tools/logs/pixel_camera/SESSION/run_ID/trial.json
```

保存データだけを再解析する場合は、解析用Pythonから`tools/analysis/pixel_camera_tuning_report.py --trial PATH --stride 4`（速報）または`--stride 1`（詳細）を使います。
出力先を上書きしないため、再解析は試行資料を保持した別フォルダーで行ってください。

## PCから明るさ・ArUcoの読み取り位置を調整する

`tools/Start_Pixel_Framing.cmd`を開きます。カメラ専用の確認画面で、TYPE8への動作指令は送りません。
2026-11-09の露出補正対応版Androidアプリが必要です。既存の録画GUIとは別の画面です。

1. 「露出補正 EV」を変更します。＋で明るく、−で暗くします。「明るさを適用」で次回撮影用に設定します。
2. 「確認撮影 3秒」を押すと、その露出補正で録画・転送・照合し、終盤の1フレームを表示します。連続ライブ映像ではありません。
3. 「範囲を編集」をONにしてTL（左上）/TR（右上）/BL（左下）/BR（右下）を選び、画像をドラッグして読み取り範囲を囲みます。マーカー全体と周囲の余白を含めます。
4. 「範囲を保存」で次回起動時にも使えます。緑は検出ID、黄は未読です。IDを推測で補完しません。
起動直後は範囲の編集をOFFにし、画像上の意図しないドラッグで認識範囲が動くことを防ぎます。

「読めたマーカーで補正」をONにすると、基準画像に登録したIDのうち、**任意の2枚以上**の検出頂点で盤面を射影補正します。
マーカーのみで位置合わせする基礎方式は2枚なら8頂点、3枚なら12頂点を使い、特定の隅や未読IDには依存しません。
現在の調整用基準は、下記の「分散した格子点による精度改善」も有効にしています。このモードでは盤面の45 mm格子線も使います。
頂点の順番はArUcoの符号に対応するため、基準画像と現在画像の同じID・同じ頂点を照合できます。
中心2点だけで射影変換を推定する方式ではありません。読めた登録済みマーカーはすべて使い、2枚の場合は8点の最小二乗、3枚以上はRANSACと残差で採否を判断します。
2枚未満、重複ID、不正頂点、変換の退化、残差過大では補正を確定せず、前の行列を使い回しません。

画面には使用ID・採用頂点数・基準画像への対応誤差（px）を表示します。格子を併用した場合は、その画像の未使用格子点RMS（mm）も表示します。水色の十字は補正結果として求めた盤面位置で、未読IDの検出結果ではありません。
`tools/logs/pixel_camera/marker_reference.json`が登録基準で、保存時に撮影フォルダーへ同じJSONをコピーします。
`inspection.json`の`board_reference`へ使用ID・対応点数・誤差・射影行列・盤面位置を保存し、`rectified_board.png`へ補正画像を保存します。
2026-11-09の登録基準は以前に盤面位置を確認した画像を使い、実検出したID5・6・7を登録しています。各2枚の全3組で確認済みです。
未登録IDの頂点を推測して追加しません。新しいIDを使う場合は、盤面校正がある基準画像で登録を更新します。

補正は、登録したマーカーが同じ平面上で相互位置を保っていることを前提とします。
カメラや盤面全体の移動には対応できますが、マーカー単体の貼り直しや、別の盤面では基準を更新します。
絶対的なmm精度は登録基準と補正方式に依存します。[初期のマーカーのみの検査](logs/pixel_camera/20261109_accuracy_final/README.md)では、3枚使用で位置RMS 2.03 mm・最大観測5.59 mmでした。
2枚の場合は組み合わせ・測定位置による誤差増大があるため、読める登録マーカーはすべて使います。変換成立や頂点対応のpx誤差だけで絶対精度を判断しません。
評価結果は登録基準の`accuracy_evaluation`へ参照を保存し、画面では「頂点対応RMS」と「盤面位置RMS（保存動画検査）」を区別して表示します。
検査対象は盤面上の見える格子点です。機体上の目印の高さによる視差は含みません。走行パラメータへは自動反映しません。

### 分散した格子点による精度改善

`pixel_camera_calibrate.py`は45 mm格子で基準を作り、`grid_refinement.enabled`を有効にします。
各フレームで実際に読めた任意の登録マーカー2枚以上を照合し、盤面全体に散らした格子交点からmm変換を求め直します。
校正点は内部格子の列・行がともに奇数の位置に固定し、その他の点は独立した検査用に残します。左上等の特定の隅には限定しません。
元画像の白線中心を使い、マーカー頂点の小さな揺れや、撮影間の見かけの盤面移動による誤差を抑えます。

初期基準には12点以上、フレームごとの補正には8点以上の広く分布した校正点が必要です。
検査点12点以上、検査RMS 0.75 mm以下・95％点1.5 mm以下・最大2 mm以下を暫定の採用条件とします。
条件不足・未読・検査不合格ではそのフレームの補正を確定せず、前の行列は使い回しません。
格子が機体や壁で隠れる場合は成立率を確認してください。マーカーだけで変換が成立したことと、この精度条件の成立を区別します。
`inspection.json`の`board_reference.grid_quality`に、その画像の校正点数・未使用検査点数・誤差・採用条件を記録します。

```powershell
& D:/venv/venv3-11/Scripts/python.exe tools/analysis/pixel_camera_calibrate.py `
  --capture tools/logs/pixel_camera/CAPTURE.capture.json `
  --seed-reference tools/logs/pixel_camera/marker_reference.json `
  --roi-settings tools/logs/pixel_camera/framing_settings.json `
  --output tools/logs/pixel_camera/calibration_NEW
```

既存の撮影と出力を上書きしません。基準の生成だけでは運用ファイルを自動更新しません。
別撮影での検査結果と運用設定は[調整準備の記録](logs/pixel_camera/20261109_tuning_setup/README.md)へ保存しています。

### 機体上マーカーの高さ

機体Aのユーザー確認値は赤（前）10 mm、黄（後）約1 mmです。
`tools/logs/pixel_camera/body_geometry_A.json`へ記録し、`video_marker_pose.py --body-geometry`で指定します。
絶対位置・機体の向きを求めるモードでは、赤と黄を別々の高さで補正した後に向きを求めます。カメラ中心の盤面座標x/y・床からの高さzが未入力なら、このモードの解析は停止します。
車軸中心の計算には、マーカー中点から車軸中心までの前方・左方向の距離（mm）も必要です。未入力なら結果はマーカー中点であり、車軸中心とは表示しません。
機体Aは中心から赤が前へ約25 mm、黄が後ろへ約25 mmという追加申告により、高さ補正後の中点を中心とする設定を記録しました。寸法は概算であり、画像のmm換算を50 mmへ強制的に合わせる用途には使いません。
カメラ高さは機体固有値と分け、測定ごとに`--camera-height-mm 800`（今回の概算800 mmの場合）を指定します。
高さが変わっても同じ方式で補正できます。新しい撮影の解析で前回値を自動流用せず、確定した値を動画のパスとともに`body_geometry_used.json`へ保存します。
カメラの真下の盤面位置は`--camera-foot-mm X Y`または設定の`camera_foot_board_mm`で指定します。座標は盤面の左上原点・右がx正・下がy正のmmで、画像中心とは限りません。
現在の約800 mmという申告を過去動画の撮影条件へ遡って適用しません。高さや水平位置を変えたら、その測定のカメラ条件も更新してください。
これらの補正の実映像での精度と、動画・CSVの厳密な同期はまだ未検証です。

### カメラの真下が不明な場合の相対軌跡

`pixel_camera_relative_pose.py`は、同じ動画内でカメラの位置・高さが固定され、機体の各目印の高さが一定という条件で、目印中点の開始位置からの変位を求めます。
この変位では未知の真下位置が相殺されるため、真下の座標を推測せずに高さ補正できます。
赤10 mm・黄約1 mmを別々に扱います。マーカー中点は、前後対称約25 mmという申告に基づく機体中心の近似です。
盤面上の絶対位置と、高さ補正済みの機体角度はこのモードからは得られません。カメラや目印の高さが動画の途中で変わる場合も対象外です。

```powershell
& D:/venv/venv3-11/Scripts/python.exe tools/analysis/pixel_camera_relative_pose.py `
  --trial tools/logs/pixel_camera/TRIAL/trial.json `
  --marker-reference tools/logs/pixel_camera/marker_reference.json `
  --body-geometry tools/logs/pixel_camera/body_geometry_A.json `
  --camera-height-mm 800 --height-source approximate_for_capture --fixed-camera `
  --video-start 6.5 --video-end 8.6 --output tools/logs/pixel_camera/relative_NEW
```

開始・終了秒は対象動画の実際の走行区間に合わせ、前後の静止状態も含めます。過去動画の高さが不明な場合は`--height-source historical_assumption`と明記します。
相対位置CSV、動画・実CSVとの比較、設定のコピー、入力ハッシュ、品質レポートを新しい出力先へ保存します。欠測はNaNと検出フラグで残し、位置合わせから除外します。
ログの動作全区間を含む時刻合わせを要求し、動作開始前200 ms～停止後100 msの観測率95％以上と前後の観測数を検査します。時刻は軌跡に合わせた推定で、独立したハードウェア同期ではありません。
`--pose-cache apparent_floor_marker_pose.csv`を指定すると、動画のSHA-256・較正基準・色検出条件・対象時間範囲が対応するキャッシュだけを再利用できます。
`REVIEW_REQUIRED`は比較の目視確認が必要という状態で、走行パラメータの自動採用を許可するものではありません。

単体ターンの`pixel_camera_tuning_report.py`は、停止中の開始点を原点にした相対変位を使い、旋回前の直進軌跡を直線近似して前方軸を決めます。
その軸に合わせて軌跡全体を回転するため、小さな設置位置・開始向きの差は相対的な終点・軌道の比較から除けます。
`summary.json`の`trajectory_reference_frame`へ使用した軸・変換行列・入口直線の観測数・長さ・残差を保存します。
終点や出口方向を目標へ合わせる回転・拡大縮小は行わず、実際の曲がり過ぎ、曲がり不足、距離誤差は残します。
姿勢補正は穴や盤面外へ出る実際の経路、床面差、較正誤差をなくすものではありません。走行前の静止認識と実経路の余裕は別に確認します。
開始姿勢を厳密にそろえた試験と、入口方向で補正した相対比較は記録上も区別し、過去の姿勢条件不成立を後から成立へ書き換えません。

厳密な開始姿勢の一致を要求しない相対比較では、試験経路に対応した盤面の縦横・斜め軸を、配置確認の角度基準として計画に保存します。
運用上の角度範囲を盤面軸と保存したOFF姿勢の両方へ重ねて適用すると、OFF時の傾きによって許される範囲が偏るため、保存OFF姿勢との差は診断値として残します。
配置の運用範囲とは別に、確認済み静止姿勢から走行直前までの位置・向きの変化を制限し、実配置からの予測経路、機体幅、穴、盤面端、停止余裕を確認してください。
この基準を変更する場合は元の姿勢参照・不成立記録を保持し、新しい条件で静止試写を取得してから走行します。絶対車軸位置・高さ補正済みyawの精度を保証する判定ではありません。

格子の一部が機体に隠れる場合は、参照JSONの`grid_refinement.training_policy`に`visible_spread`を明示できます。
各フレームの見えている格子から、公称位置の広がりだけで補正点を8〜10点選び、別の12点以上を検査用に残します。
選択に画像上の補正残差は使わず、検査点の誤差上限や必要点数は変えません。点数・広がり・検査誤差が不足するフレームは引き続き無効です。
指定がなければ従来の固定位置分割を使います。参照JSONをコピーして明示し、変更前の解析を保持してください。

固定カメラの静止映像でも、`visible_spread`の補正点がフレームごとに切り替わると、画像上で安定した目印のmm座標が揺れる場合があります。
この場合は参照JSONのコピーで`grid_refinement.training_policy`を`fixed_indices`にし、既存の`training_grid_indices`を使う方式も比較できます。
可視の補正点8点以上・別の検査点12点以上と精度条件を保てることを、静止区間と走行全体の全フレームで確認してください。
固定の補正点群と検査点群は別のフレーム間でも混ぜません。元の不成立結果と参照JSONは残し、方式を変えた比較では基準走行も同じ参照JSONで別フォルダーへ再解析します。
静止映像の再解析だけを根拠に走行を始めず、新しい撮影でも認識・静止品質・実経路を確認します。

実例と限界は[真下不明での解析記録](logs/pixel_camera/20261109_unknown_camera_foot/README.md)を参照してください。

「撮影用ライト」はPixel内蔵ライトを撮影中だけ点灯させます。チェック状態は次の設定・確認撮影に反映されます。
実機の左上マーカーでは改善せず、左下の検出率もわずかに落ちたため、現在の配置ではOFFを使います。
録画終了時にカメラを閉じると消灯します。選択したON/OFFはサービス内で保持され、通常のTYPE8撮影にも使われます。
撮影JSONの`torch_requested`と`flash_mode_result_values`（2=TORCH、0=OFF）、sensor.csv末尾の`flash_mode_result`で確認できます。
ライトは露出補正対応後に追加したため、2026-11-09のライト対応APKが必要です。

Pixel 4a (5G)の実機申告は−4〜＋4 EV、1/6 EV刻みです。指定値は端末の刻みに丸めます。
録画JSONの`ev_steps_requested`、`ev_step`、`ev_steps_result_values`で要求と実測応答を照合できます。
露出補正は自動露出の目標値を変えます。ISO・シャッター・ピントの手動固定や光学画角の変更はこの画面にはありません。
反射で失われた模様は、明るさや読み取り範囲の変更だけでは復元できない場合があります。

露出補正はAndroidのPC接続サービスを停止するまで維持され、通常のTYPE8撮影にも使われます。
`configure(width=1280, height=720, fps=240)`だけでは露出補正をリセットしません。0 EVへ戻すときも明示的に適用します。
サービスを再起動すると0へ戻ります。画面の「接続を確認」で現在値を読み直せます。
撮影用ライトの選択もサービス再起動でOFFへ戻ります。

保存先:

- `tools/logs/pixel_camera/framing_settings.json`: 読み取り範囲と最後に確認した露出補正値。再起動時に露出補正を自動送信するファイルではありません。
- `tools/logs/pixel_camera/framing/日時_ID/`: 取得フレームと`inspection.json`。元動画・撮影JSON・フレームPTS・画面回転無効・検出ID・検出頂点の元画像座標を記録します。
- 同フォルダーの`marker_reference.json`と`rectified_board.png`: 使用した登録基準のコピーと、補正済みの盤面画像。
- `tools/logs/pixel_camera/framing/*_settings.json`: 保存した読み取り範囲の履歴。

座標はMP4の自動回転を無効にした1280×720の横画像で統一します。
小さなマーカーは余白追加・3倍補間でも再検出し、その場合は`method: padded_3x`と記録します。補間で元の撮影解像度が上がるわけではありません。
それでも未読の場合は、局所コントラスト補正（CLAHE）・2倍補間・2通りのしきい値で再検出し、IDが一致した場合だけ採用します。
この場合は`method: clahe_2x_two_thresholds_agree`です。コードの誤り許容数は増やしていません。白とび率などの統計は補正前画像から求めます。
この範囲設定は確認画面で使用します。動画解析で同じ補正を使う場合は、`video_marker_pose.py --marker-reference tools/logs/pixel_camera/marker_reference.json`を明示します。
指定しない解析では、従来の4枚同時検出による較正を維持します。異なる基準を別の盤面へ黙って適用しません。

このPCの起動環境は`D:/venv/venv3-11`（OpenCV aruco・NumPy・Pillow・Tk導入済み）です。
別環境では`PIXEL_FRAMING_PYTHON`に対応Pythonの実行ファイルを指定するか、次を実行します。
通信・転送・動画検査には既存のDebug_camera/.venvを子プロセスで使います。

```powershell
& D:/venv/venv3-11/Scripts/python.exe tools/pixel_camera_framing.py
# 保存済み動画を表示して始める（この指定だけでは再撮影しない）
& D:/venv/venv3-11/Scripts/python.exe tools/pixel_camera_framing.py --video C:/path/to/video.mp4
& D:/venv/venv3-11/Scripts/python.exe -m unittest discover -s tools/tests -p test_pixel_camera_framing.py -v
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
このPCでは2026-11-09に撮影データを`D:\TYPE8_Camera\recordings`へ移行しました。今後の遠隔動画は`D:\TYPE8_Camera\recordings\remote`へ保存します。
元の`C:\Users\sato1\Documents\git\Debug_camera\recordings`はD側を指すジャンクションです。
過去の撮影JSON・解析結果・転送履歴に記録されたC側のパスも引き続き使えます。入力資料のハッシュを保つため、過去の記録を一括書き換えしていません。
移行前から開いていたGUIにC側の表示が残っている場合も実体はD側です。再起動後は保存済み設定のD側パスを表示します。
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

## BLEから単体ターンを直接実行して撮影する

pixel_camera_turn_capture.pyは撮影専用のdebug turn ... exe_cameraと既存BLE中継を使います。
**手かざしなしで動くのはこの撮影専用経路だけです。通常のdebug turn ... exeは手かざし待ちを維持します。**
先に別のBLE GUI・シェル・中継を切断します。このコマンドが8766番の中継と5 ms周期の連続CSV保存を起動します。
機体の電源・開始位置・向き・走行範囲・カメラの固定と画角を確認してから実行してください。

```powershell
& "$env:LOCALAPPDATA/Programs/Python/Python311/python.exe" tools/pixel_camera_turn_capture.py `
  --mouse A --motion long_r180 --preset 500 --address C1:B5:60:87:91:73 --seconds 30 --execute
```

アドレスは今回確認したHY0020のものです。別の機体や中継では実際のアドレスに合わせます。
--mouseはユーザー指定の機体情報で、機体を自動識別した証拠にはしません。
この入口は単体1回・吸引OFFに限定し、指定プリセットをRAMへ読み込んで実機応答の速度・回数・吸引を確認します。
既定では既存のRAM調整値はプリセット読込みで戻ります。
GUIで調整したRAM値を撮影するときは、`Apply`→`Show device values`で確認してGUIの`Disconnect`を押し、中継の終了を待ちます。
次に上のコマンドへ`--use-device-settings --expected-settings <照合用.settings.json>`を追加してください。
この組合せでは`show`だけで読み、ターン・機体指定・速度プリセット・回数・全パラメータ・PID・FF・吸引設定が照合用JSONと一致しない場合、録画と走行の前に停止します。
照合用JSONは期待値であり、実機から取得した証拠とは分けて`expected.settings.json`へ保存します。実機値は従来どおり`telemetry.settings.json`に残します。
GUIの`Apply and execute`は撮影連動の入口ではないため、この手順では使いません。

USB reverseなどでカメラがすでに接続されている場合は、`--camera-existing-connection`で既存の接続を利用できます。
接続済み・IDLEを確認し、ADBの再設定やサーバーの自動起動を省きます。端末のUSB/Wi-Fi接続方式を変更する指定ではありません。
カメラ用.venvへ自動的に切り替え、TYPE8側の既存Bleak/pyserial環境を参照します。

1. 電圧・プリセット・速度・回数1・吸引OFF・連続CSV受信を確認する。
2. 720p・240 fpsで録画し、録画ID・所有者・RECORDING状態の一致を待つ。
3. RECORDING_READY後に電圧を再確認する。録画ID・所有者・RECORDING状態・残り10秒以上を、許可前と実行前にも照合する。
4. debug camera arm tokenで15秒間・一回限りの許可を取得し、同じトークンのdebug turn ... exe_camera tokenを一度だけ送る。
5. 実機が許可を消費して手かざしなしで開始し、CAMERA_MOTION_STARTEDを表示する。**コマンド開始前に機体から手を離しておく。** 録画は30秒で終了する。
6. 転送・復号・照合後に連続CSVを閉じ、走行区間と前後500 msをmotion.csvへ切り出す。

実行コマンドは送信失敗・応答なしでも再送しません。許可の期限切れ・不一致・再利用は実機側でも拒否します。
走行がCSVに見つからない場合は成功扱いにせず、再送せず機体を確認します。撮影専用機能の非対応ファームウェアでは録画・実行前に停止します。
録画・ログ取得の成功と、動画に走行全体が入っていることは別に確認します。正常取得時もCAPTURED_REVIEW_REQUIREDとして映像確認を残します。

保存先はtools/logs/pixel_camera/日時_機体_ターン_速度_ID/です。--outputで未使用のフォルダーも指定できます。

| ファイル | 内容 |
| --- | --- |
| trial.json | 機体、録画ID、動画・CSV・設定の対応、実行結果、未確認事項 |
| telemetry.csv / telemetry.settings.json | 全連続ログと、走行前に読み返した実機設定 |
| motion.csv / motion.settings.json | 走行と前後の区間。time_msがMCU時刻で、1行を1 msと扱わない |
| telemetry.raw.bin / telemetry.shell.bin | 中継が受信した原文 |
| events.jsonl / pc.shell.bin | 電圧確認・撮影用許可・実行要求と開始応答の時刻と原文 |
| gateway.stdout.txt / gateway.stderr.txt | 中継の起動・終了・エラー |

本体のGet log CSVは直近400件（既定約0.4秒）のリングバッファです。単体ターンの全区間には連続CSVを使います。
中継動作中のtelemetry.status.jsonは原子的な置換で更新されます。Windowsで読み取りと置換が競合した事例があるため、このコマンドは中継停止後にだけ読みます。

模擬検証（BLE・カメラ・機体を使わない）:

```powershell
& "$env:LOCALAPPDATA/Programs/Python/Python311/python.exe" -m unittest discover -s tools/tests -p test_pixel_camera_turn_capture.py -v
```

2026-11-09、撮影専用の一回許可を使い、機体Aのlong_r180・preset 500・回数1・吸引OFFで手かざしなしの撮影を確認しました。
30秒・7,194フレーム、PTS平均239.813 fps、全区間CSV取得、連番欠落・CRCエラー0、前走・旋回・後走・停止の映像内収録まで確認しています。
結果はtools/logs/pixel_camera/20261109_A_turn500_long_r180_camera01/README.mdです。Bの実機は未確認です。
2026-11-08の手動GUI試行では、走行が録画時間外となったこと、本体保存ログの上書き、中継の状態JSON更新失敗を確認しました。
それらの動画・CSV・設定・失敗状態はtools/logs/pixel_camera/20261108_A_turn500_long_r180_trial*/に保持しています。

## 検証範囲

録画ID・所有者・実際の録画状態の照合、開始失敗時にコールバックを呼ばないこと、
開始要求を重複送信しないこと、転送回復と失敗レポートをホストテストで確認しています。
TYPE8フォルダーでcheckとguiの起動を確認し、record --seconds 5 --label camera_connection_checkで5秒のカメラ実機試験も完了しました。1199フレーム、MP4長4.999622秒、PTS実測239.818 fpsで、転送・独立復号・SHA-256再照合・端末削除通知まで成功しています。
結果は tools/logs/pixel_camera/20261108_230712_6337a59c.capture.json です。
その後、機体A・turn_500の右180度をGUIから1回実行しましたが、走行が録画内に収まらず、本体CSVにも旋回区間が残っていませんでした。
その後の撮影専用経路では走行全体の動画とCSVの同時取得に成功しました。最新の採用結果はtools/logs/pixel_camera/20261109_A_turn500_long_r180_camera01/README.md、初期の経緯はtools/logs/pixel_camera/20261108_handoff.mdに保存しています。
