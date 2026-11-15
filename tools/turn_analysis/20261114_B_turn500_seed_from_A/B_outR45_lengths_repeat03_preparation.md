# 機体B・右45度出口32.8/18.0 mm・3本目の準備

同じ候補の有効記録は2本。半径−50 mm、Lstart32.8/Lend18.0 mm、補正＋0.5°、Bの旋回PID.10/.002/0・並進1.5/.02/0と全FFを維持。速度0.5 m/s・前走加速度6.5 m/s²・回数1・吸引OFF。今回の走行指令は未送信です。

開始中点は盤面左上から右90/下45 mm、前黄を右下45°。前340/右160/後60/左60 mmの空き、停止・手離し・今回の両方ONを確認。カメラ約1500 mm/2倍・EV−6/AF固定と盤は固定。同じ値なので余分な調整・ビルド・実機書込みはありません。

next_session_B_outR45_lengths_repeat03_v1.json、B_outR45_lengths_repeat03_plan.jsonを使用。新セッションへoperator_ready.jsonと今回の確認を保存してからprepare_outR45_lengths_fast.py --meta <今回のメタデータ>を実行。新しいBプリセットから候補を復元し、BのFF/設定照合と静止撮影まで連続処理。終了コード0/READY/ready=trueと静止画像を別途確認した後、CLI captureを--operator-ready --request-id B_outR45_lengths_repeat03_run01 --no-waitで1回だけ送信、再送なし。停止と資料保存後は両方OFFを案内し、全フレームと固定窓静止照合、出口方向を確認します。

メタデータ参照先だけを変更した静止ヘルパーは今回の保存画像でも通過、ランナーの10件検査は機能変更がないため継続参照。検証outR45_lengths_repeat03_preparation_validation.json。3本集計はreview_out_R45_repeats.pyを使用し、平均各軸1 mm/各記録各軸2 mm/出口1°の目安とばらつきを示し、候補・実機採用・絶対精度を区別します。ユーザーの両方OFFとサービス終了は確認済み。
