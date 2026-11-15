# 機体B・右135度出口：補正＋1.5°・Lstart33.7 mm比較準備

未実行候補。半径−32.3 mm・Lend46.07 mm・Bの並進PID1.5/.02/0・旋回PID.10/.002/0と全FFを維持し、角度補正だけ＋0.5→＋1.5°、Lstart40.7→33.7 mmを組み合わせます。論理角は−135°のまま、生成角は−133.5°。2本の出口約1°の過回転と位置差に基づく増分モデルから決めた候補で、個別効果の分離や予測の精度保証はありません。速度0.5 m/s・pre_accel6.5 m/s²・1回・吸引OFFです。

保存した開始位置へ戻す：前後マーカー中点は盤面左上から右180/下45 mm、前黄が右下向き。カメラ約1500 mm/2倍・EV−6/AF固定と盤を固定。スキルのstart-placements.mdと基準OFF写真を参照し、撤回した135/45 mmの図を再利用しません。新しい予測経路outR135_corr1p5_Lstart33p7_nominal_route.pngは中心線だけです。タイヤ/前後接地点・機体幅・旋回/停止余裕と穴/壁の非干渉、停止・手離し・今回の両方ONを確認します。

メタデータnext_session_B_outR135_corr1p5_Lstart33p7_v1.json、計画B_outR135_corr1p5_Lstart33p7_plan.json、準備処理は既存prepare_outR135_candidate_fast.py。新しいoperator_ready.jsonにはphysical_hole_full_contact_clearance_confirmedとplacement_matches_current_off_picture_confirmedもtrueで記録し、5分以内の今回の確認を使います。Bプリセット/全FF読返し→候補復元・照合→静止撮影・照合を続けて処理。追加の名目穴グリッド診断は静止レビュー内へまとめ、照合後に別の追加計算で待たせません。穴の完全な形状/接地点非干渉をモデルで証明したとはしません。

終了コード0/READY/ready=trueと静止画像を別途確認後、capture --settings <実機候補> --operator-ready --request-id B_outR135_corr1p5_Lstart33p7_run01 --no-waitを1回だけ送信し、再送しません。停止・保存後は接触の有無と両方OFFを確認し、全フレーム/固定窓静止・相対形状と盤面固定診断を分けて評価します。

ランナー機能は変更していないため既存11件の検査を継続参照。今回のパラメータで65 mmマーカー経路の盤内、実際の保存静止画像で新ヘルパーの通過、表示差/FFの維持条件と元資料ハッシュ照合を確認しました。検証outR135_corr1p5_Lstart33p7_preparation_validation.json。次回は未実行、両方OFF・接続終了確認済み。ファーム変更/書込みなし。
