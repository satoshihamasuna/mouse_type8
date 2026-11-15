# 右135度入口・Lstart44.5 mmの比較準備

半径−32.3 mm・Lstart44.5/Lend38.35 mm・補正+0.5度・SP PID1.50/0.02/0・OM PID0.10/0.002/0と全B FF、500プリセット0.50 m/s、前走加速度6.5 m/s²、吸引OFF、回数1。前の有効記録からLstartだけ+1 mmを比較します。効果は未測定。

同じ開始位置・向き、カメラ約1500 mm・−6段・AF固定・ライトOFFと盤固定、前250/右250/後60/左60 mmの空き、停止・手離しを確認後にON。新しいB preset500・電圧・PID/FFを照合し、settings_sourceへ --set lstart=44.5 を適用、差がLstartだけかを読み返します。preflight_inR135_r32p3_Lstart44p5.py record/review で静止5枚と配置を確認し、終了コード0・ready=true後、別呼出しでrequest-id B_inR135_r32p3_Lstart44p5_run01を一度だけcaptureします。再送しません。

停止・保存後にOFFを案内し、走行/静止全フレームの同じ条件で評価。5枚の配置確認と全フレームの精度判定は混同しません。Params変更・書き込みは行いません。メタJSON: next_session_B_inR135_r32p3_Lstart44p5_v5.json。
