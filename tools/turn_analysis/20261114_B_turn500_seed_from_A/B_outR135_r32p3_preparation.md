# 機体B・右135度出口の半径−32.3 mm比較準備

未実行候補。前回のBの実機値から半径r_minだけ−30→−32.3 mmへ変更します。Lstart40.7/Lend46.07 mm・論理−135°/補正＋0.5°・速度0.5 m/s・pre_accel6.5 m/s²・回数1・吸引OFF。Bの並進PID1.5/.02/0・旋回PID.10/.002/0と全FFを維持します。Bの右135入口で使った半径を参考に、今回は出口で変化量を確認する比較です。入口/出口長はその後に調整します。

開始位置を変えず、スキルの配置記録と初回前のOFF写真へ戻します：マーカー中点が盤面左上から右180/下45 mm、前黄が右下45°。現在のカメラ約1500 mm/2倍・EV−6/AF固定と盤を固定。撤回した135/45 mmへの移動図は使いません。予測経路はoutR135_r32p3_nominal_route.png（中心線だけ）。タイヤ・前後接地点・車幅・旋回/停止余裕が穴/壁に干渉しないこと、配置一致・停止・手離し・今回の両方ONをユーザーに確認します。

メタデータnext_session_B_outR135_r32p3_v1.json、計画B_outR135_r32p3_plan.json、準備処理prepare_outR135_candidate_fast.py。operator_ready.jsonにはphysical_hole_full_contact_clearance_confirmedとplacement_matches_current_off_picture_confirmedもtrueで記録。新しいBプリセット・FFを前回実機値と照合し、半径だけ変更した候補を適用・読返し、静止照合まで連続処理。終了コード0/READY/ready=trueと静止画像の別途確認後に、capture --settings <実機候補> --operator-ready --request-id B_outR135_r32p3_run01 --no-waitを1回だけ送ります。再送なし。停止・保存確認後は接触の有無と両方OFFを確認し、全フレーム・固定窓静止・相対形状と別の盤面診断を評価します。

機器なし検査11件成功。65 mmマーカー間隔の名目経路は盤内ですが、Bの全接地点の穴非干渉を画像モデルだけでは検証していません。outR135_r32p3_preparation_validation.jsonとoutR135_r32p3_nominal_containment.jsonを参照。開始配置の過去実績は今回の実行許可ではなく、毎回の確認が必要です。ファームの有効値/コメント・ビルド・書込みは変更していません。
