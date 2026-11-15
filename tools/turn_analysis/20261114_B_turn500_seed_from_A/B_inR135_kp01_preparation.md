# 機体B 右135度入口・Pゲイン0.10の比較準備

2026-11-15。直前の有効な `B_inR135_Aseed_run01` を基準に、旋回Pだけ0.30→0.10を比較します。実行指令は未送信です。機体B・HY0020は両方OFF、前回サービスは終了済みです。今回の配置・手離し・両電源ONの返答後に実機値と静止画像を確認します。

半径−30 mm、Lstart47.8/Lend38.35 mm、論理角−135度/補正+0.5度、500プリセット・0.50 m/s、前走加速度6.5 m/s²、吸引OFF、回数1。SP PID1.50/0.02/0、OM PID0.10/0.002/0。前回の形状・BのFFを維持します。効果は今回のB右135度入口ではまだ未測定です。

設置は直前と同じで、前黄が画面下向き、中心は盤面約225/40 mmです。前250・右250・後60・左60 mmの余裕、機体幅・停止余裕・手離しを確認します。カメラ約1500 mm・−6段・AF固定・ライトOFFと格子盤を固定します。前回静止位置との差は10 mm/5度以内、今回の静止試写で名目経路の盤内余裕も再検査します。

`next_session_B_inR135_kp01_v5.json` が今回の保存先と実行ゲート、`B_inR135_kp01_plan.json` が前回の実機値・FF・解析根拠です。AのFFや別ターンのFFを流用せず、新しいB `in_r135 preset 500` のPID/FFを先に照合します。再起動で形状が初期値へ戻るため、保存済み実機値を使って形状も復元します。

接続サービスを非表示で起動し、service.jsonの作成・接続・電圧を確認した後、次の操作を直列に行います。ここに記載しただけでは実機へ送信しません。

```powershell
$mouseSession = 'tools/logs/pixel_camera/tuning_cli_B_inR135_kp01_20261115_prepared'
$mouseBaseline = 'tools/logs/pixel_camera/tuning_cli_B_inR135_Aseed_20261115_prepared/run_B_inR135_Aseed_run01/motion.settings.json'
$cameraPython = 'C:/Users/sato1/Documents/git/Debug_camera/.venv/Scripts/python.exe'
& $cameraPython tools/pixel_camera_tuning_cli.py --session $mouseSession preset --motion in_r135 --preset 500 --output "$mouseSession/B_inR135_preset500.settings.json"
# 新しいBのPID/FF・形状・吸引と保存基準を照合してからApplyする。
& $cameraPython tools/pixel_camera_tuning_cli.py --session $mouseSession apply --settings $mouseBaseline --set pid.om_kp=0.10 --output "$mouseSession/B_inR135_kp01_candidate.settings.json"
& $cameraPython tools/turn_analysis/20261114_B_turn500_seed_from_A/preflight_inR135_kp01.py record --session $mouseSession
& D:/venv/venv3-11/Scripts/python.exe -X utf8 tools/turn_analysis/20261114_B_turn500_seed_from_A/preflight_inR135_kp01.py review --session $mouseSession
```

静止照合の終了コード0・ready=true、B現行FFの維持、前回からPだけ変更、今回の手離し確認が揃ってから、`B_inR135_kp01_run01` を一度だけcaptureします。録画受付を走行開始と同一視せず、既存の録画ID/所有者/RECORDING確認経路を使います。exe/captureの自動再送は行いません。

走行・録画・ログ停止と保存を確認した時点で両電源OFFを案内し、全フレームで追従・終点180/45 mm・出口方向を比較します。Params反映・ビルド・書き込みは行いません。連続走行・壁あり経路・絶対車軸位置・高さ補正済みyaw・独立同期は未検証です。
