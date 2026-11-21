"""009取得と現在の両方OFF/接触なしを記録し、保存済み設定だけ照合する。"""
from pathlib import Path
from datetime import datetime,timezone
import json
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr45_20261120_v1'
STAGE=BASE/'geometry03_correction_m0p4';identifier='B_INR45_1600_correction_m0p4_green_20261121_009'
RUN=STAGE/'local_execution'/('run_'+identifier);read=lambda p:json.loads(Path(p).read_text(encoding='utf-8-sig'))
trial=read(RUN/'trial.json');settings=read(RUN/'saved_1ms.settings.json');candidate=read(STAGE/'candidate.settings.json')
events=[json.loads(line) for line in (RUN/'events.jsonl').read_text(encoding='utf-8').splitlines()]
assert len([e for e in events if e['event']=='execution_write_attempt'])==1 and trial['execution_resend_count']==0
assert trial['state']=='CAPTURED_REVIEW_REQUIRED' and trial['camera_state']=='PASSED'
assert read(STAGE/'transport_binding_verified_20261121.json')['verified_binding']
assert settings['mouse']=='B' and settings['motion_name']=='in_r45' and settings['preset_speed_mm_s']==1600
assert all(settings[key]==candidate[key] for key in ('parameters','pid','feedforward','suction','turn_count'))
record=dict(confirmed_at_utc=datetime.now(timezone.utc).isoformat(),source='Current user reply: 両方OFF・接触なし',both_powered_off=True,no_contact=True)
with (RUN/'operator_after_run.json').open('x',encoding='utf-8') as f:json.dump(record,f,ensure_ascii=False,indent=2)
state_path=BASE/'resume_state.json';state=read(state_path)
state.update(state='009_CAPTURED_OFF_CONFIRMED_FULL_ANALYSIS_PENDING',stage=str(STAGE),request_id=identifier,request_sent=True,
 next_request_sent=True,execution_commands_sent=7,execution_pending=False,operator_ready=False,next_operator_ready=False,
 physical_power_state='BOTH_OFF_USER_CONFIRMED',current_physical_power_state_unconfirmed=False,source_trial=str(RUN/'trial.json'),
 latest_capture_passed=True,latest_capture_independent_contiguous_window_eligible=None,
 next_action='実機値と一回実行を確認済み。両方OFFで全フレーム・1ms・入口時計を評価し、−1.5度の3本と同じ基準で比較する。',
 ble_binding_evidence=str(STAGE/'transport_binding_verified_20261121.json'),ble_binding_readback_pending=False,
 next_operator_ready_record=str(STAGE/f'operator_ready_{identifier}.json'))
state['ble_binding'].update(request_sent=True,current_binding_evidence=state['ble_binding_evidence'],pending_request_id=None,hardware_connections_started=2)
state_path.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(one_execution_verified=True,actual_settings_match=True,camera_state=trial['camera_state'],power='BOTH_OFF',saved_frames=trial['saved_log']['frame_count'],saved_drop=trial['saved_log']['drop_count']),ensure_ascii=False))
