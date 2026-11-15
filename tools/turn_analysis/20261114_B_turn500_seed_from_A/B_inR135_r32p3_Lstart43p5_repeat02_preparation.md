# 機体B 右135度入口・中間候補を同じ設定で新たに1回撮影

前の試行は実行・停止・保存済みですが、追加静止RMS0.519867 mmが0.50 mmを超え、採用本数から除外しました。閾値を緩めず、新しい配置・手離し・ON確認後に別IDで1回撮影します。前のexeの再送ではありません。

半径−32.3 mm・Lstart43.5/Lend38.35 mm・論理角−135度/補正+0.5度、SP PID1.50/0.02/0、OM PID0.10/0.002/0と全B FF、500プリセット0.50 m/s・前走加速度6.5 m/s²・吸引OFF・回数1、約1500 mm・−6段・AF固定・ライトOFF・前黄/後赤・v5を維持します。

メタJSONは `next_session_B_inR135_r32p3_Lstart43p5_repeat02_v5.json`、計画は `B_inR135_r32p3_Lstart43p5_repeat02_plan.json`。現在は両電源OFF・サービス終了、次の実行指令は0回。同じ開始位置・向き、前250/右250/後60/左60 mmの空きと停止余裕・停止・手離しを確認後にONにします。

新しいBのpreset500と電圧・初期PID/FFを確認し、settings_sourceの実機値をApplyして全一致を照合します。`preflight_inR135_r32p3_Lstart43p5_repeat02.py record/review --session <今回のsession>`で静止5枚と盤内余裕を確認します。終了コード0・ready=trueと今回の手離し確認後、別呼出しで `capture --operator-ready --request-id B_inR135_r32p3_Lstart43p5_repeat02 --no-wait` を1回だけ実行。停止・保存後にOFFを案内し、走行/静止の同じ品質条件で判定します。Params変更・ビルド・書き込みは行いません。
