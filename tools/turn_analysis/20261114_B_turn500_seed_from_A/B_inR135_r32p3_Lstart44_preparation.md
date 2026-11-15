# 右135度入口・Lstart44.0 mm中間候補の準備

半径−32.3 mm・Lstart44.0/Lend38.35 mm・補正+0.5度・SP PID1.50/0.02/0・OM PID0.10/0.002/0と全B FF、500プリセット0.50 m/s、前走加速度6.5 m/s²、吸引OFF、回数1。Lstart43.5と44.5の有効な記録の間を1回比較し、その後の反復は同設定で行います。

同じ開始位置・向き、約1500 mm・−6段・AF固定・ライトOFFと盤固定、前250/右250/後60/左60 mmの空き、停止・手離しを確認後にON。新しいB preset500・電圧・PID/FFを照合し、settings_sourceへ --set lstart=44 を適用して差がLstartだけかを確認。preflight_inR135_r32p3_Lstart44.py record/reviewで静止5枚と配置を検査し、終了コード0・ready=true後、別呼出しでID B_inR135_r32p3_Lstart44_run01を一度だけcaptureします。再送しません。停止・保存後にOFFを案内し、走行/静止の全フレームを同じ品質条件で評価します。Params変更・書き込みは行いません。
