"""画像を目視確認した基準一回の評価を確定し、保存一式へ追加する。機器操作なし。"""
from pathlib import Path
from datetime import datetime,timezone
import json
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_outr45_20261121_v1'
OUT=BASE/'baseline01_review_v1';RUN=BASE/'baseline01/local_execution/run_B_OUTR45_1600_baseline_green_20261121_001'
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
review=read(OUT/'review.json');review.update(visual_review_completed=True,
 selected_visual_evidence=str(OUT/'observed_frames_raw_orientation.jpg'),
 visual_review='開始・旋回中・出口・停止後の4枚で前緑/後赤の実マーカーを確認。元解析と同じraw向き。')
review['visual_frames']=read(OUT/'raw_orientation_frame_evidence.json')['frames']
(OUT/'review.json').write_text(json.dumps(review,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
text=('# 機体B・1600 右out45：基準001（一回）\n\n'
 '補正+0.3度、Lstart21.8/Lend16.5mm、半径−60mm、角加減速FF0.000750、角速度Ki0.01、吸引650。充電完了確認後に一回だけ実行。BLE E1:7F:86:21:84:73 / 8767をプロセス引数まで照合。実機値一致、再送0、1ms保存237件・欠落0。撮影PASSED、停止確認通過、操作者が両方OFF・接触なしを確認。\n\n'
 '全954フレーム中950有効。入口raw wheel時計による必要区間の実観測97.33%、ArUco登録99.33%。無登録/片側欠測は除外、推定点0。入口36点・111.58mm・RMS0.418mm、出口16点・62.76mm・RMS0.318mm。入口だけの時計照合RMS0.687mm・半窓差0.180ms。品質基準は変更していない。\n\n'
 '実測入口に対する出口43.809度（目標より1.191度浅い）。Lend中点32.585/223.617mm、目標との差+0.765/+0.878mm、ノルム1.165mm。後半停止中点差+2.243/+5.064mm、ノルム5.538mm。停止点は減速・制動後の値で、Lend点と区別する。最低電圧7.754V。\n\n'
 '次は角度補正のみ+0.3→−0.9度を一回比較する候補。角度応答の傾き−1は初回の仮定であり、結果の保証ではない。Lstart/LendとPID/FFは据え置く。現在のLend解は1msの時刻差で約1.8mm変わり、角度変更でも点が動くため、同時の長さ変更を避ける。\n\n'
 '評価は高さ補正した相対マーカー中点と出口進行方向。絶対車軸・機体yaw・盤面固定の位置精度は未検証。角速度の振れが残り、PID/FF最適化完了ではない。BのEKF同定、反復、左、連続接続は未確認。今回Params変更/実機書込みなし。\n\n'
 '[軌跡](../baseline01/local_execution/run_B_OUTR45_1600_baseline_green_20261121_001/midpoint_raw_wheel_clock_v3/midpoint_trajectory.png)・[実画像](observed_frames_raw_orientation.jpg)・[数値](review.json)。\n')
(OUT/'README.md').write_text(text,encoding='utf-8')
for source,name in ((OUT/'review.json','baseline_relative_review.json'),(OUT/'observed_frames_raw_orientation.jpg','observed_frames_raw_orientation.jpg'),
 (OUT/'raw_orientation_frame_evidence.json','visual_frame_evidence.json')):
 with (RUN/name).open('xb') as f:f.write(source.read_bytes())
state_path=BASE/'resume_state.json';state=read(state_path)
state.update(state='BASELINE_001_FULL_REVIEW_COMPLETE_OFF_CONFIRMED',latest_aggregate_review=str(OUT/'review.json'),
 latest_review=str(RUN/'midpoint_raw_wheel_clock_v3/midpoint_review.json'),latest_independent_relative_geometry_eligible=True,
 latest_standard_full_summary=read(RUN/'full/summary.json'),
 next_action='基準001の一式を保存し、角度補正−0.9度のみの比較002を準備。新しい配置/ON/手離し確認までは送信しない。')
state_path.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
with (BASE/'README.md').open('a',encoding='utf-8') as f:
 f.write('\n## 基準001の実測\n\nここまで実走一回。両方OFF・接触なし。録画/停止/相対評価の品質を通過。出口43.809度、Lend誤差ノルム1.165mm、停止点差5.538mm。角度補正だけ−0.9度の比較を準備する。詳細は[基準レビュー](baseline01_review_v1/README.md)。上の0回は初回準備時点の記録。\n')
print('Baseline relative review and real-frame QA finalized; ready to archive; both OFF')
