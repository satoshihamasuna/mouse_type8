# 機体B・右45度出口の旋回P0.10比較準備

次回は半径−50/Lstart37.2/Lend9.2 mm・角度補正＋0.5°を固定し、旋回Pだけ0.30→0.10。Bの並進PID1.5/.02/0、旋回I.002/D0、全FFを維持します。未実行です。

開始配置はB_outR45_start_layout.pngと同じ、中点(90,45) mm、前黄が右下45°。格子盤・カメラ約1500 mm/2倍・EV−6/AF固定は維持し、前340/右160/後60/左60 mmを目安に空きを確認。毎回の配置・停止・手離し・両方ONのユーザー確認が必要です。

メタデータnext_session_B_outR45_kp01_v1.json、計画B_outR45_kp01_plan.json、確認用予定値B_outR45_kp01_expected.settings.json。予定値は実機読返しではありません。準備処理prepare_outR45_kp01_fast.pyは新しいBプリセットの全PID/FFと元資料を照合し、今回のPだけ変更して適用・読返し、preflight_outR45_kp01_zoom2.pyで静止照合します。P以外の制御値差や表示1LSB超の形状差は停止します。機器なし検査10件成功、ハッシュoutR45_kp01_preparation_validation.json。

実行前に新セッションのoperator_ready.jsonとメタデータへ今回の確認を保存（5分以内、ID B_outR45_kp01_run01）。準備処理の終了コード0/READYと静止画像を別途観測後に、既存CLI capture --settings <実機候補JSON> --operator-ready --request-id B_outR45_kp01_run01 --no-waitを別ツールで1回だけ送信します。再送なし。保存・停止確認後は両方OFFを案内し、保存済み動画で全フレーム・固定窓静止・出口方向を評価します。前試行の失敗・欠測は隠さず保持。ソース有効値・ファーム書込みなし。
