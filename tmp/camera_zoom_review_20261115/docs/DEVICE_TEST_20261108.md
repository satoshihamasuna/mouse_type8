# Pixel 4a (5G) 初回実機試験 — 2026-11-08

## 確認した端末

- Google Pixel 4a (5G)、device: bramble
- Android 14 / API 34
- 背面カメラID 0、Camera2 FULL、高速撮影capabilityあり
- 1280×720の[240,240]を申告
- H.264: c2.qti.avc.encoder、Surface入力、720p240の申告あり

当初想定したPixel 4a (4G)とは異なる端末。以下の結果を4Gモデルの動作証明には使わない。

## 確認した録画

対象: 20261108_083824_240fps_fe274b23.mp4

| 項目 | 観測値 |
|---|---|
| 解像度 | 1280×720 |
| 指定時間 / 制御用実測時間 | 10秒 / 10.0154秒 |
| エンコード出力数 | 2,336フレーム |
| エンコーダPTSのfps | 239.775 fps |
| MP4サンプル時刻のfps | 239.810 fps |
| MP4サンプル時刻の範囲 | 9.7369秒 |
| エンコーダPTSの最大間隔 | 4.204 ms |
| MediaExtractorの最大間隔 | 4.300 ms |
| センサー通知数 | 292件 |
| センサー通知頻度 | 29.9719回/秒 |
| センサーframe number | 7, 15, 23, …, 2335（全通知で差が8） |
| frame numberの進行と通知時刻からの推定 | 239.7749 fps |
| 撮影失敗 / バッファ欠落通知 | 0 / 0 |
| エンコーダEOS / MP4確定 | 成功 / 成功 |
| ファイルサイズ | 58,881,829 bytes |
| SHA-256 | eddf5cf6ae3238cbada9394af4c472257621f1894408fd4dbe4c291ffb82dff9 |
| 熱状態（前 / 後） | 0 / 0 |

端末からPCのrecordings/test01/へコピーした。端末の元ファイルは保持。

## PCでの独立したデコード検査

既存のOpenCV 5.0.0 / FFMPEGバックエンドで全フレームを実際に読み出した。フレーム数プロパティの申告値はカウントに使用していない。

- デコードできたフレーム数: 2,336（エンコーダ出力数と一致）
- デコード画素サイズ: 1280×720
- 連続するフレームの完全な画素一致: 0組
- PC側のファイルサイズ・SHA-256: 端末JSONの値と一致
- ffprobe CLI: このPCのPATHには未導入。ffprobeによるPTSの独立検査は未実施

実行コマンド:

```powershell
py -3 tools/decode_with_opencv.py recordings/test01/20261108_083824_240fps_fe274b23.json
```

OpenCVの再生位置やFPSプロパティは、240 fps達成の判定には使用していない。

## 初版の警告と修正

初版はセンサー通知を全フレーム分受け取る前提で評価しており、292件の通知を30 fps撮影やフレーム欠落と誤認してTIMING_GAPS_DETECTEDと表示した。

実データでは各通知のframe numberが8ずつ進み、生成された高速リクエストのバッチ数も8、エンコード出力数は通知数の正確に8倍だった。したがって、通知頻度だけをセンサー撮影fpsとして表示しないよう修正した。

新しい判定はINCONCLUSIVE_BATCHED_SENSOR_METADATA。通知を補間して独立したセンサー実測値として扱うことはせず、「8フレームごとの通知」「frame numberからの推定fps」「動画PTSのfps」を区別する。動画PTS自体に長い間隔がある場合の検出は維持する。既存の端末JSONと動画は書き換えずに保持する。

実装根拠の参考:
[AOSP CameraDeviceImpl](https://android.googlesource.com/platform/frameworks/base/+/master/core/java/android/hardware/camera2/impl/CameraDeviceImpl.java)および
[高速リクエストの生成](https://android.googlesource.com/platform/frameworks/base/+/master/core/java/android/hardware/camera2/impl/CameraConstrainedHighSpeedCaptureSessionImpl.java)。
これは公開実装の参考資料であり、この端末のvendor HALの実装を直接確認したものではない。

## 未確認の点

**物理的に独立した240フレーム/秒の撮影を達成したとは、まだ断定していない。**

保存形式・時刻・件数・デコードは240 fpsの実時間動画と整合するが、センサーの個々のフレーム時刻は通知されていない。ノイズ等で画素が異なるだけでも完全一致検査は通るため、独立した既知の時間基準を持つ動く対象で約4.17 msの時間分解能を確認する必要がある。

次の試験:
1. 明るい環境で動く対象を10秒録画し、動きとフレームの重複を確認。
2. 計測したLEDパターンや安全に固定した回転マーカー等の時間基準で、物理的な時間分解能を評価。
3. ffprobeでデコードPTSを確認。
4. 複数回の連続試験、30秒、暗所、発熱時でも同じ基準で比較。

USB/Wi-Fi遠隔制御・自動転送ACK後の削除・GUI・長時間録画は未実装。
