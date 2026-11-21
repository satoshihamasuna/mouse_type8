"""指定試行の現在の両方OFF/接触なしと、一回実行・保存設定の一致を記録する。"""
from pathlib import Path
from datetime import datetime,timezone
import json
import sys
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_outr45_20261121_v1'
read=lambda p:json.loads(Path(p).read_text(encoding='utf-8-sig'))
state_path=BASE/'resume_state.json';state=read(state_path);identifier=sys.argv[1];STAGE=Path(state['next_stage'])
assert STAGE.is_relative_to(BASE) and state['next_request_id']==identifier and not state['next_request_sent']
RUN=STAGE/'local_execution'/('run_'+identifier);trial=read(RUN/'trial.json')
record=dict(confirmed_at_utc=datetime.now(timezone.utc).isoformat(),source='Current user reply: 両方OFF・接触なし',both_powered_off=True,no_contact=True)
with (RUN/'operator_after_run.json').open('x',encoding='utf-8') as f:json.dump(record,f,ensure_ascii=False,indent=2)
events=[json.loads(line) for line in (RUN/'events.jsonl').read_text(encoding='utf-8').splitlines()]
assert len([e for e in events if e['event']=='execution_write_attempt'])==1 and trial['execution_resend_count']==0
settings=read(RUN/'saved_1ms.settings.json');candidate=read(STAGE/'candidate.settings.json')
assert read(STAGE/'transport_binding_verified_20261121.json')['verified_binding']
assert settings['mouse']=='B' and settings['motion_name']=='out_r45' and settings['preset_speed_mm_s']==1600
actual_match=all(settings[key]==candidate[key] for key in ('parameters','pid','feedforward','suction','turn_count'))
state.update(state='CAPTURED_OFF_CONFIRMED_FULL_ANALYSIS_PENDING',stage=str(STAGE),request_id=identifier,request_sent=True,
 next_request_sent=True,execution_commands_sent=state['execution_commands_sent']+1,execution_pending=False,operator_ready=False,next_operator_ready=False,
 physical_power_state='BOTH_OFF_USER_CONFIRMED',current_physical_power_state_unconfirmed=False,source_trial=str(RUN/'trial.json'),
 latest_capture_passed=trial['camera_state']=='PASSED',latest_capture_independent_contiguous_window_eligible=None,
 next_action='実機値と一回実行を照合し、両方OFFで全フレーム・1ms・入口時計を評価する。',
 ble_binding_evidence=str(STAGE/'transport_binding_verified_20261121.json'),ble_binding_readback_pending=False,
 next_operator_ready_record=str(STAGE/f'operator_ready_{identifier}.json'))
state['ble_binding'].update(request_sent=True,current_binding_evidence=state['ble_binding_evidence'],pending_request_id=None,
 hardware_connections_started=state['ble_binding']['hardware_connections_started']+1)
state_path.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
assert actual_match and trial['state']=='CAPTURED_REVIEW_REQUIRED' and trial['camera_state']=='PASSED'
print(json.dumps(dict(one_execution_verified=True,actual_settings_match=actual_match,camera_state=trial['camera_state'],power='BOTH_OFF',
 saved_frames=trial['saved_log']['frame_count'],saved_drop=trial['saved_log']['drop_count']),ensure_ascii=False))
