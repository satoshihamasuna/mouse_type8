# 機体B・右45度出口32.8/18.0 mm・2本目

形状・制御は変更せず同じ候補を1回撮影します。半径−50 mm、Lstart32.8/Lend18.0 mm、補正＋0.5°、Bの旋回PID.10/.002/0・並進1.5/.02/0と全FFを維持、速度0.5 m/s・前走加速度6.5 m/s²・回数1・吸引OFF。候補の有効記録は現在1本。平均・ばらつきは未確定です。

開始中点は格子盤左上から右90/下45 mm、前黄が右下45°、前340/右160/後60/左60 mmの空き。カメラと盤を固定（約1500 mm・2倍・EV−6・AF固定）。設置・停止・手離し・今回の両方ON確認後、operator_ready.jsonとメタデータに記録してからprepare_outR45_lengths_fast.py --meta next_session_B_outR45_lengths_repeat02_v1.jsonを実行。新しいBプリセットから復元してFF/全設定を照合し、専用ヘルパーで静止確認。終了コード0/READY/ready=trueと画像を別途確認した後、既存CLI captureをID B_outR45_lengths_repeat02_run01・--operator-ready・--no-waitで1回だけ送ります。指令再送なし。停止・保存確認後は両方OFFを案内し、保存動画で全フレーム・固定窓静止検査と出口方向を確認します。

計画B_outR45_lengths_repeat02_plan.json、メタデータnext_session_B_outR45_lengths_repeat02_v1.json、ヘルパーpreflight_outR45_lengths_repeat02_zoom2.py、検証outR45_lengths_repeat02_preparation_validation.json。ランナー/解析の機能は変更しておらず、10件の検査を継続参照し、保存静止画像で今回の参照先と配置判定も再現しました。今回の走行指令0回、両方OFF・接続終了確認済み。ファーム変更・書込みなし。
