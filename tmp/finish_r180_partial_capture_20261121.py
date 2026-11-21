"""取得失敗の原本を保持し、OFF確認・5ms診断・同設定再開候補を保存する。実機操作なし。"""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json
import shutil
import sys
import numpy as np
import pandas as pd

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_r180_20261120_v1'
stage=BASE/'resume_20261121_Le40p5_03'
identifier='A_R180_1800_Le40p5_resume_20261121_003'
run=stage/'local_execution'/('run_'+identifier)
nextstage=BASE/'resume_20261121_Le40p5_retry01'
nextid='A_R180_1800_Le40p5_retry_20261121_001'
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
def save(p,v):
    with p.open('x',encoding='utf-8') as f:json.dump(v,f,ensure_ascii=False,indent=2,allow_nan=False)
def update(p,v):
    old=read(p);old.update(v);p.write_text(json.dumps(old,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

trial=read(run/'trial.json')
assert trial['state']=='FAILED' and trial['saved_log']['status']=='failed'
assert trial['execution_write_attempted'] and trial['execution_resend_count']==0
operator=read(stage/'local_execution/operator_results'/f'{identifier}.json')
assert operator['both_powered_off'] and not operator['contact_detected']
assert not (nextstage/'local_execution/jobs'/f'{nextid}.json').exists()
current=read(stage/'candidate.settings.json'); candidate=read(nextstage/'candidate.settings.json')
assert all(current[k]==candidate[k] for k in ('parameters','pid','feedforward','suction','turn_count','mouse'))
now=datetime.now(timezone.utc).isoformat()
raw=pd.read_csv(run/'motion.csv')
active=raw[(raw.time_ms>=trial['log_quality']['motion_start_time_ms']) &
           (raw.time_ms<=trial['log_quality']['motion_end_time_ms'])]
turn=active[abs(active['ideal.rad_velo'])>=abs(active['ideal.rad_velo']).max()*.02]
weights=np.diff(turn.time_ms.to_numpy(float),prepend=turn.time_ms.iloc[0]);weights[0]=5
err=(turn['ego.rad_velo']-turn['ideal.rad_velo']).to_numpy()
transport=stage/'local_execution'/('transport_'+identifier)
shellbytes=(transport/'telemetry.shell.bin').read_bytes()
review=dict(state='FAILED_1MS_CAPTURE_SAVED_VIDEO_AND_5MS_DIAGNOSTIC_ONLY',
    trial_id=identifier,source_trial=str(run/'trial.json'),original_capture_state='FAILED',
    execution_count=1,execution_resend_count=0,saved_1ms_available=False,
    saved_log_error=trial['saved_log']['error'],both_powered_off_confirmed=True,
    contact_detected=False,operator_off_timing_answer='案内を見た後にOFFにした',
    premature_power_off_established=False,disconnect_cause='unresolved',
    gateway_before_run_count=0,gateway_after_trial_count=0,
    competing_session_cause_established=False,ble_disconnected=True,
    live_log=dict(period_ms=5,full_motion_retained=True,lost_frames=0,crc_errors=0,
        maximum_motion_sample_gap_ms=int(active.time_ms.diff().max()),
        battery_min_v=float(active.Battery.min()),
        omega_tracking_rms_rad_s=float(np.sqrt(np.average(err**2,weights=weights))),
        omega_peak_command_rad_s=float(abs(turn['ideal.rad_velo']).max()),
        wheel_speed_mean_m_s=float(np.average(turn['ego.velo'],weights=weights)),
        saturation_sample_fraction=float(((abs(turn.V_r)>=turn.Battery*.98)|
            (abs(turn.V_l)>=turn.Battery*.98)).mean()),
        not_equivalent_to_retained_1ms=True),
    shell_received_bytes=len(shellbytes),saved_header_received=b'HEADER,' in shellbytes,
    saved_binary_banner_received=b'BINARY' in shellbytes,
    standard_analysis=dict(status='REJECTED_FAILED_CAPTURE_STATE',shell_exit_code=1,
        error='trial, settings and body geometry must identify the same machine and a passed capture',
        original_trial_state_unchanged=True),
    Lend_position=None,relative_geometry_accepted=False,parameter_adoption_allowed=False,
    ekf_identification='unavailable: retained 1ms log missing; existing control mode only recorded',
    next_candidate=str(nextstage/'candidate.settings.json'),next_trial_id=nextid,
    next_candidate_executed=False,operator_ready=False,updated_at_utc=now)
diagnostic=stage/'partial_capture_diagnostic_v1/report.json'
if diagnostic.is_file():
    review['video_diagnostic_report']=str(diagnostic)
    review['video_diagnostic']=read(diagnostic)
save(stage/'partial_capture_review.json',review)
exports=run/'review_exports';exports.mkdir()
for name in ('partial_capture_review.json','approximate_start_policy.json','detector_candidate_setup.json',
             'geometry_proposal.json','runtime_conditions_for_review.json'):
    if (stage/name).is_file():shutil.copyfile(stage/name,exports/name)
if diagnostic.is_file():shutil.copytree(diagnostic.parent,exports/'video_diagnostic')
shutil.copytree(transport,run/'transport_evidence')
prep=stage/'local_execution'/('prepare_'+identifier)
for name in ('static_check.json','rough_start_acceptance.json','static_window_quality.json','static_plan.json'):
    if (prep/name).is_file():shutil.copyfile(prep/name,exports/name)
shutil.copyfile(stage/'local_execution/runtime_modes.json',exports/'runtime_modes.json')
shutil.copyfile(stage/'run_recorded_once.py',exports/'capture_wrapper.py')
shutil.copyfile(Path(__file__),exports/'finalize_source.py')
shutil.copyfile(ROOT/'tmp/review_r180_partial_capture_20261121.py',exports/'diagnostic_source.py')
for name in ('approximate_start_policy.json','detector_candidate_setup.json','geometry_proposal.json'):
    if (stage/name).is_file():shutil.copyfile(stage/name,nextstage/name)
geometry=Path(read(stage/'config.json')['geometry'])
shutil.copyfile(geometry,nextstage/geometry.name)
update(nextstage/'config.json',dict(geometry=str(nextstage/geometry.name)))
save(nextstage/'phase_inherited_before_cleanup.json',read(nextstage/'phase.json'))
(nextstage/'phase.json').write_text(json.dumps(dict(state='PREPARED_NOT_EXECUTED',
    request_id=nextid,next_request_id=nextid,next_unsent_request_id=nextid,
    candidate=str(nextstage/'candidate.settings.json'),config=str(nextstage/'config.json'),
    source_trial=str(run/'trial.json'),source_review=str(stage/'partial_capture_review.json'),
    settings_identical_to_previous_trial=True,operator_ready=False,execution_pending=False,
    execution_write_attempted=False,request_consumed=False,turn_count=1,
    power_state='Both_OFF_confirmed',both_powered_off_confirmed=True,
    next_candidate_executed=False,parameter_adoption_allowed=False,
    physical_readiness_required_again=True,updated_at_utc=now),ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
update(stage/'phase.json',dict(state=review['state'],request_id=identifier,request_consumed=True,
    execution_write_attempted=True,execution_resend_count=0,saved_1ms_available=False,
    operator_ready=False,execution_pending=False,both_powered_off_confirmed=True,
    power_state='Both_OFF_confirmed',ble_disconnected=True,operator_contact=False,
    source_params_changed=False,parameter_adoption_allowed=False,
    review=str(stage/'partial_capture_review.json'),next_request_id=nextid,
    next_candidate_executed=False,updated_at_utc=now))
update(BASE/'preparation.json',dict(state=review['state'],phase='WAITING_SAME_SETTING_RETRY_BOTH_OFF',
    last_trial=str(run/'trial.json'),last_trial_id=identifier,saved_1ms_available=False,
    operator_ready=False,execution_pending=False,next_execution_pending=False,
    power_state='Both_OFF_confirmed',both_powered_off_confirmed=True,
    external_ble_session_present=False,ble_disconnected=True,operator_contact=False,
    current_placement_confirmed=False,current_clearance_confirmed=False,
    current_hands_clear_confirmed=False,current_physical_clearance_verified=False,
    next_request_id=nextid,next_candidate=str(nextstage/'candidate.settings.json'),
    next_candidate_executed=False,next_stage=str(nextstage),parameter_adoption_allowed=False,
    last_review=str(stage/'partial_capture_review.json'),updated_at_utc=now))
text=f'''# 機体A・1800 R180：1 msログ取得失敗

試行 `{identifier}` は1回だけ走行。半径47.6 mm・補正＋1.5°・Lstart 7.52 mm・Lend 40.5 mm。
開始中点は右315.51 mm・下134.18 mm、見掛け向き89.74°。静止観測198/198点。
動画と5 msログは保存できたが、`disp log_bin` の取得中にBLEが切断し、1 msログは未取得。
原本 `trial.json` の FAILED を保持。自動再送なし。接触なし・停止・両方OFFをユーザーが確認。
電源OFFは案内後。切断原因は未確定で、電源操作や別接続が原因と確定していない。

通常ログは走行全体を保持し、欠落0・CRC異常0、走行最低電圧は{review['live_log']['battery_min_v']:.3f} V。
5 msの角速度追従RMSは{review['live_log']['omega_tracking_rms_rad_s']:.3f} rad/s（1 ms解析と同一視しない）。
標準解析は FAILED 状態のため拒否。入口基準の動画診断は review_exports/video_diagnostic に別保存。
1 ms時計でのLend位置・EKF同定・採用判定は未実施。盤面固定の接続や絶対車体yawも未認証。
ソースのParams・ファームウェアはこの再開作業では変更・書込みしていない。

次は同設定の未実行候補 `{nextid}`。位置を戻し、両方ON・余裕・手離しの新しい確認が必要。
初期配置は概略5 mm・5°、動画では実入口方向に相対座標を合わせる。走行指令を再送しない。
'''
(run/'README.md').write_text(text,encoding='utf-8')
print(json.dumps({k:review[k] for k in ('state','trial_id','saved_1ms_available','next_trial_id','both_powered_off_confirmed')},ensure_ascii=False))
