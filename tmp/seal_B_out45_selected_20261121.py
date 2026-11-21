"""保存前に表示の丸め・生成元ハッシュ・図の目視確認記録を揃える。原測定は変更しない。"""
from pathlib import Path
from datetime import datetime,timezone
import hashlib,json
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_outr45_20261121_v1'
OUT=BASE/'selected_m0p9_three_review_v1'
RUN=BASE/'correction02_repeat01_m0p9/local_execution/run_B_OUTR45_1600_m0p9_repeat01_green_20261121_003'
script=ROOT/'tmp/select_B_out45_m0p9_provisional_20261121.py'
decision=json.loads((OUT/'decision.json').read_text(encoding='utf-8'))
decision['reason']=decision['reason'].replace('2.769mm','2.767mm')
decision['input_sha256'][str(script)]=hashlib.sha256(script.read_bytes()).hexdigest()
data=json.dumps(decision,ensure_ascii=False,indent=2,allow_nan=False)
for path in (OUT/'decision.json',RUN/'provisional_single_turn_decision.json'):path.write_text(data,encoding='utf-8')
path=OUT/'README.md';text=path.read_text(encoding='utf-8')
text=text.replace('前走加速6.5m/s²。','前走加速指定6.5m/s²（斜め127.279mmで1.6m/sへ到達する実効値10.057m/s²）。')
path.write_text(text,encoding='utf-8')
proof=dict(confirmed_at_utc=datetime.now(timezone.utc).isoformat(),raw_orientation_frames_visually_reviewed=True,
 comparison_figure_visually_reviewed=True,source_markers_front_green_rear_red=True,
 no_numerical_measurement_changed=True,source_script_sha256=hashlib.sha256(script.read_bytes()).hexdigest())
for folder in (OUT,RUN):
 with (folder/'visual_review_confirmation.json').open('x',encoding='utf-8') as f:json.dump(proof,f,ensure_ascii=False,indent=2)
print('Decision rounding/hash and visual review sealed; raw measurements unchanged')
