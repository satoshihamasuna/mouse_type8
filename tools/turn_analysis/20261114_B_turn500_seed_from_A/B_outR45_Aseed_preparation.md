# 機体B・右45度出口の初回準備（2026-11-15）

ユーザー指定「右45°出口へ進む」に対応。B/HY0020は前走行後に両方OFF確認済み。今回の配置・電源ONは未確認で、機器接続・設定変更・走行指令は0回です。

## 候補と配置

Aの保存済み3本を参考に、半径−50 mm、Lstart37.2 mm、Lend9.2 mm、degree−45°、correction＋0.5°、速度0.5 m/s、前走加速度6.5 m/s²、回数1、吸引OFF。AのPID/FFはコピーしません。接続後、Bのout_r45・500プリセットを新しく取得し、BのPID（並進1.5/.02/0、旋回.30/.002/0）とFF全項目を維持して形状値だけ編集・照合します。inR135など他ターンの旋回P0.10を無断流用しません。

格子盤左上を(0,0)、右をX、下をYとし、前後マーカー中点を(90,45) mm（右2マス・下1マス）、前の黄色を右下45°へ向けます。[配置図](B_outR45_start_layout.png)。前340/右160/後60/左60 mmを目安に空きを確保。名目軌道と65 mmマーカー間隔の検査は盤内ですが、実軌道の保証ではなく、現在の静止画像で再照合します。

ファームの前走は斜め127.2792 mm、後走は直進90 mm。全動作の相対目標は右95.459415/前286.378226 mm、出口は初期方向から右45°。この配置の目標盤面位置は(225,315) mmです。Aの平均前方向誤差＋2.2485 mmは残っており、A候補をBの合格値とは扱いません。

## 短い準備処理と走行

実行計画はB_outR45_Aseed_plan.json、メタデータはnext_session_B_outR45_Aseed_v1.json、設定はtuning_cli_config_B_outR45_zoom2_v1.json。機体Bの前黄・後赤、カメラ約1500 mm・2倍・EV−6・AF固定・ライトOFFを継続します。

今回の設置・停止・空き・手離し・両方ONをユーザーが確認後、セッションへoperator_ready.jsonを保存し、メタデータをUSER_CONFIRMED_POSITION_HANDS_OFF_BOTH_ONにしてから次を実行します。確認は5分以内、IDはB_outR45_Aseed_run01です。

```powershell
& D:/venv/venv3-11/Scripts/python.exe -X utf8 tools/turn_analysis/20261114_B_turn500_seed_from_A/prepare_outR45_fast.py --meta tools/turn_analysis/20261114_B_turn500_seed_from_A/next_session_B_outR45_Aseed_v1.json
```

この処理は接続→Bプリセット読返し→B値から候補生成→適用・読返し→3秒静止撮影→5枚の設置照合を連続実行し、走行指令は送りません。失敗時は後続へ進めず、同じセッションを再実行しません。終了コード0、READY_FOR_SEPARATE_CAPTURE、static_preflight_check.jsonのready=true、静止画像の目視確認後に、別ツール呼出しでCLI captureを--operator-ready --request-id B_outR45_Aseed_run01 --no-wait付きで1回だけ送ります。指令は再送しません。

走行・停止・動画/CSV/設定保存の確認後は、すぐ両方OFFを案内し、全フレーム解析と固定窓の静止照合は並行して進めます。解析はreview_out_R45.py、密な静止検査はcheck_inR135_dense_static_cached.py（名称は旧ターンだが処理は方向非依存）を使用できます。別撮り間の向き照合に失敗した場合は失敗を残し、値の採用や集計を保留します。旧body-alignment V3の縦向きマスクは斜め向きにそのまま使いません。

## 機器なしの確認

Aの元ファイルを保持し、full解析のコピー3本で元の終点測定値を1e−6 mm以内で再現。目標座標変換と出口方向も確認しました（A_outR45_reference_check/validation.json）。新しい準備処理の検査6件は成功：B値保持、別機体/ターン/PID混入の拒否、適用後FF改変の拒否、古い設置確認の拒否、プリセット失敗時の後続停止、斜め配置/向き/盤外の判定。outR45_preparation_validation.jsonにソースハッシュを保存しました。実機値の照合は次回接続時で、Bの有効撮影は0本です。Params変更・ビルド・書込みは行っていません。


右45出口の初回準備は、B実機Lend32.009とソース32.010の厳密比較で停止しました。候補適用・撮影・走行0回、接続終了・両方OFF確認済み。整数表示の切捨て1LSBだけを許容するv2を追加し、実機読返し再現・超過差拒否を含む9件が成功。PID/FFの比較は変更していません。再開はnext_session_B_outR45_Aseed_v2.json、prepare_outR45_fast_v2.py、preflight_outR45_zoom2_v2.pyです。元の失敗資料は保持。位置維持で再度の手離し・両方ON確認が必要です。
