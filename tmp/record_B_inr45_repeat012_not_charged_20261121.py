"""充電条件の現在の回答を、012の取得時条件として追加保存する。"""
from pathlib import Path
from datetime import datetime,timezone
import json
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr45_20261120_v1'
RUN=BASE/'geometry05_repeat01_correction_m1p2/local_execution/run_B_INR45_1600_m1p2_repeat01_green_20261121_012'
read=lambda p:json.loads(Path(p).read_text(encoding='utf-8-sig'))
assert read(RUN/'operator_after_run.json')['both_powered_off']
record=dict(confirmed_at_utc=datetime.now(timezone.utc).isoformat(),source='Current explicit reply to charging-condition question',
 question='今回の走行前に、機体Bの充電は完了していましたか？',user_reply='充電はまだ完了していなかった',
 charging_completed_before_trial=False,classification='SAME_SETTINGS_REPEAT_BEFORE_CHARGING_COMPLETION',
 previous_after_charge_wording_corrected=True,physical_power_state='BOTH_OFF_USER_CONFIRMED',hardware_commands_sent=0)
with (RUN/'operator_charging_condition.json').open('x',encoding='utf-8') as f:json.dump(record,f,ensure_ascii=False,indent=2)
state_path=BASE/'resume_state.json';state=read(state_path)
assert state['request_id']=='B_INR45_1600_m1p2_repeat01_green_20261121_012'
state.update(charging_completion_confirmed=False,charging_requested=True,current_repeat_charging_completed=False,
 charging_condition_record=str(RUN/'operator_charging_condition.json'),
 physical_power_state='BOTH_OFF_USER_CONFIRMED',
 next_action='012は充電未完了での同設定反復として評価する。両方OFFで充電完了の明示回答を待ち、その後の準備で実走条件を揃える。')
state_path.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print('012 explicitly classified as before charging completion; both OFF')
