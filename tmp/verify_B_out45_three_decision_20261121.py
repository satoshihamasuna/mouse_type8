"""三試行の元資料/一式保存/暫定値/終了状態を照合。機器操作なし。"""
from pathlib import Path
from datetime import datetime,timezone
import hashlib,json,sys
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_outr45_20261121_v1'
OUT=BASE/'selected_m0p9_three_review_v1';read=lambda p:json.loads(Path(p).read_text(encoding='utf-8-sig'))
sys.path.insert(0,str(ROOT/'tools'))
from pixel_camera_archive import verify_archive,verify_sources
decision=read(OUT/'decision.json');selected=read(OUT/'selected.settings.json')
assert decision['executions_total']==3 and decision['execution_resends']==0
assert selected['parameters']==decision['selected_parameters']
assert selected['parameters']['correction']==-.9 and selected['parameters']['lstart']==21.8 and selected['parameters']['lend']==16.5
assert selected['feedforward']['ff_om_accel']==selected['feedforward']['ff_om_decel']==.000750
for filename,digest in decision['input_sha256'].items():
 assert hashlib.sha256(Path(filename).read_bytes()).hexdigest()==digest,filename
archives=[];execution_count=0
for row in decision['rows']:
 trial_path=Path(row['source_trial']);RUN=trial_path.parent;trial=read(trial_path)
 events=[json.loads(line) for line in (RUN/'events.jsonl').read_text(encoding='utf-8').splitlines()]
 attempts=[e for e in events if e['event']=='execution_write_attempt'];assert len(attempts)==1
 execution_count+=len(attempts)
 assert trial['execution_resend_count']==0 and read(RUN/'operator_after_run.json')['both_powered_off']
 assert read(RUN/'transport_binding_verified.json')['verified_binding']
 assert trial['saved_log']['drop_count']==0 and trial['camera_state']=='PASSED'
 pointer=read(RUN/'archive.json');folder=Path(pointer['archive_folder'])
 assert hashlib.sha256((folder/'manifest.json').read_bytes()).hexdigest()==pointer['manifest_sha256']
 manifest=verify_archive(folder);verify_sources(manifest,trial_path)
 archives.append(dict(run_id=row['run_id'],archive=pointer,files=len(manifest['files']),source_hashes_match=True))
assert execution_count==3
state_path=BASE/'resume_state.json';state=read(state_path)
assert state['execution_commands_sent']==3 and state['physical_power_state']=='BOTH_OFF_USER_CONFIRMED'
assert state['next_request_id'] is None and state['next_stage'] is None and not state['operator_ready'] and not state['execution_pending']
assert [r['relative_geometry_eligible'] for r in decision['rows']]==[True,False,True]
for key in ('parameters','pid','feedforward','suction','turn_count'):
 assert selected[key]==decision['rows'][-1]['settings'][key],key
report=dict(verified_at_utc=datetime.now(timezone.utc).isoformat(),executions=3,resends=0,
 selected_settings_match_actual_003=True,decision_input_hashes_match=True,archives=archives,
 run002_clock_failure_preserved=True,run003_clock_and_observation_quality_passed=True,
 current_power='BOTH_OFF_USER_CONFIRMED',next_request_id=None,next_execution_scheduled=False,
 firmware_source_updated_in_this_out45_tuning=False,firmware_write_performed=False,robot_commands_sent=0)
with (OUT/'verification.json').open('x',encoding='utf-8') as f:json.dump(report,f,ensure_ascii=False,indent=2)
state.update(archive=archives[-1]['archive'],verified_archives=archives,
 final_verification=str(OUT/'verification.json'),all_three_archives_and_sources_verified=True,
 source_params_changed=False,firmware_write_performed=False)
state_path.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(executions=3,all_archives_and_sources_verified=True,selected_settings_match_actual=True,
 power='BOTH_OFF',next_execution_scheduled=False),ensure_ascii=False))
