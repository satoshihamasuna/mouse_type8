# 機体B 右135度入口・半径−32.0 mm / Lstart41.5 mmの組合せ候補

2026-11-15。残る右+1.17/前+1.78 mmを減らすため、これまでの単独比較を参考に半径−32.5→−32.0 mmとLstart44.5→41.5 mmを同時に試します。操作回数を抑える組合せ候補で、2項目の単独効果を分離した比較ではありません。効果は未測定です。Lend38.35 mm・論理角−135度/補正+0.5度、SP PID1.50/0.02/0、OM PID0.10/0.002/0と全B FFを維持。500プリセット0.50 m/s、前走加速度6.5 m/s²、吸引OFF、回数1です。

メタJSONは `next_session_B_inR135_r32_Lstart41p5_v5.json`、計画は `B_inR135_r32_Lstart41p5_plan.json`。前回の有効性は `inR135_Lstart44p5_retry02_review/final_validity.json`。両電源OFF・サービス終了を確認済み、今回の指令は未送信です。

同じ開始位置・向き（前黄が画面下）、カメラ約1500 mm・−6段・AF固定・ライトOFFと盤を固定します。前250・右250・後60・左60 mmの空き、機体幅/停止余裕・停止・手離しを確認後にONにします。

新しいB `in_r135 preset 500` の電圧・初期PID/FF・形状を照合後、`settings_source`の保存実機値へ `--set r_min=-32 --set lstart=41.5` を適用し、差がこの2項目だけであることを読み返します。`preflight_inR135_r32_Lstart41p5.py record/review --session <今回のsession>` を撮影/解析用Pythonで順に行い、静止5枚・姿勢・名目経路・AF/露出・B FF/PIDを確認します。終了コード0・ready=trueと今回の手離し確認後、別呼出しで `capture --operator-ready --request-id B_inR135_r32_Lstart41p5_run01 --no-wait` を1回だけ実行します。再送しません。

走行/録画/ログ停止と保存を確認してOFFを案内し、全フレーム観測率95%等の同じ品質条件で終点180/45 mm・出口方向を測ります。必要なら同じ固定窓の全フレーム静止照合を追加します。比較には `compare_inR135_shape_coupled.py` を使い、候補が見えたら同じ設定で3本の再現性を確認します。Params変更・ビルド・書き込みは行いません。
