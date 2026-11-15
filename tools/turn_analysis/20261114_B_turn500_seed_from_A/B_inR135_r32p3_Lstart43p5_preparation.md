# 機体B 右135度入口・半径−32.3 mm / Lstart43.5 mmの中間候補

2026-11-15。前の−32.5/44.5 mmで終点181.17/46.78 mm、今回−32.0/41.5 mmで178.51/41.25 mmとなったため、その間の半径−32.3 mm・Lstart43.5 mmを組み合わせて1回試します。補間は仮説で、各項目の単独効果や小数点以下の精度を検証した意味ではありません。

Lend38.35 mm・論理角−135度/補正+0.5度、SP PID1.50/0.02/0、OM PID0.10/0.002/0と全B FFを維持。500プリセット0.50 m/s、前走加速度6.5 m/s²、吸引OFF、回数1です。メタJSONは `next_session_B_inR135_r32p3_Lstart43p5_v5.json`、計画は `B_inR135_r32p3_Lstart43p5_plan.json`。

現在は両電源OFF・サービス終了確認済み、次の指令は未送信です。同じ開始位置・向き（前黄が画面下）、約1500 mm・−6段・AF固定・ライトOFFと盤を固定し、前250・右250・後60・左60 mmの空き、機体幅/停止余裕・停止・手離しを確認後にONにします。

新しいB `in_r135 preset 500` の電圧・初期PID/FF・形状を確認し、`settings_source`へ `--set r_min=-32.3 --set lstart=43.5` を適用して差が2項目だけであることを読み返します。`preflight_inR135_r32p3_Lstart43p5.py record/review --session <今回のsession>` で静止5枚・姿勢・名目経路・AF/露出・B FF/PIDを確認し、終了コード0・ready=trueと今回の手離し確認後、別呼出しで `capture --operator-ready --request-id B_inR135_r32p3_Lstart43p5_run01 --no-wait` を1回だけ実行します。再送しません。

停止・保存後にOFFを案内し、同じ固定窓と品質閾値で全フレームを評価します。静止全フレームは `check_inR135_dense_static_cached.py --prepare-only` で並行準備し、入力ハッシュを照合して後の比較に再利用できます。比較は `compare_inR135_shape_coupled.py`、候補が見えたら同値3本の再現性を確認します。Params変更・ビルド・書き込みは行いません。
