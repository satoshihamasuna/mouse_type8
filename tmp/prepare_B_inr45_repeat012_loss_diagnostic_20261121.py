"""012の品質不成立を保持し、既存の実フレーム診断を今回区間へ適用する。"""
from pathlib import Path
from datetime import datetime,timezone
import hashlib
import json
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr45_20261120_v1'
RUN=BASE/'geometry05_repeat01_correction_m1p2/local_execution/run_B_INR45_1600_m1p2_repeat01_green_20261121_012'
read=lambda p:json.loads(Path(p).read_text(encoding='utf-8-sig'))
report=read(RUN/'full/report.json');quality=report['required_motion_window_quality'];assert not quality['accepted_for_relative_review'] and quality['observation_rate']<.95
assert read(RUN/'operator_after_run.json')['both_powered_off'] and not read(RUN/'operator_charging_condition.json')['charging_completed_before_trial']
record=dict(recorded_at_utc=datetime.now(timezone.utc).isoformat(),status='REJECTED_FOR_RELATIVE_GEOMETRY',
 camera_state=read(RUN/'trial.json')['camera_state'],required_motion_quality=quality,thresholds_changed=False,
 exit_angle_adopted=False,Lend_adopted=False,valid_repeat_count_incremented=False,charging_completed=False,hardware_commands_sent=0,
 input_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (RUN/'trial.json',RUN/'full/report.json',RUN/'full/apparent_floor_marker_pose.csv',
 RUN/'full/apparent_floor_marker_pose.provenance.json',RUN/'operator_charging_condition.json',Path(__file__))})
with (RUN/'quality_rejection_record.json').open('x',encoding='utf-8') as f:json.dump(record,f,ensure_ascii=False,indent=2)
state_path=BASE/'resume_state.json';state=read(state_path)
state.update(state='REPEAT012_OPTICAL_QUALITY_REJECTED_CHARGING_AND_OFFLINE_DIAGNOSIS_PENDING',
 latest_independent_relative_geometry_eligible=False,latest_optical_required_motion_quality=quality,
 current_correction_same_condition_runs=1,current_correction_repeat_attempts=2,charging_completion_confirmed=False,
 physical_power_state='BOTH_OFF_USER_CONFIRMED',execution_pending=False,next_request_sent=True,
 next_action='012は充電未完了・必要区間実観測90.28%で形状評価対象外。両方OFFで充電完了を待ち、保存フレームの色/遮蔽を診断する。012は再送しない。')
state_path.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
template=ROOT/'tmp/diagnose_B_inr45_repeat008_marker_loss_20261121.py';source=template.read_text(encoding='utf-8')
replacements={
 'geometry02_repeat_02/local_execution/run_B_INR45_1600_geometry02_repeat02_green_20261120_008':str(RUN.relative_to(BASE)).replace('\\','/'),
 "OUT=RUN/'marker_loss_diagnostic_v2'":"OUT=RUN/'marker_loss_diagnostic_v1'",
 "RUN/'independent_motion_window_v1/apparent_floor_marker_pose.csv'":"RUN/'full/apparent_floor_marker_pose.csv'",
 'times=[9.3529,9.3613,9.3821,9.4029,9.4322,9.4531]':'times=[9.4741,9.4825,9.4991,9.5116,9.5283,9.5408]',
 '(9.34,9.47)':'(9.47,9.55)',
 'とくになし：照明/マーカー/取付に特段の変更なし':'012は充電未完了・両方OFF/接触なし。照明/取付の新規変更回答は取得していない。'}
for old,new in replacements.items():
 assert source.count(old)==1,old
 source=source.replace(old,new)
destination=ROOT/'tmp/diagnose_B_inr45_repeat012_marker_loss_20261121.py';compile(source,str(destination),'exec')
with destination.open('x',encoding='utf-8') as f:f.write(source)
print('012 rejected quality preserved; diagnostic driver prepared')
