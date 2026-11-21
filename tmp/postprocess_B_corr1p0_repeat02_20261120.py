"""補正1.0度の2本目をOFF確認後に解析する。実機・カメラ接続なし。"""
from pathlib import Path
import json
import shutil
import sys
from datetime import datetime,timezone

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_r90_20261120_v1'
RUN=BASE/'cli_session09/run_B_R90_1600_Lstart13p6_corr1p0_green_20261120_002'
sys.path.insert(0,str(ROOT/'tools'))
from pixel_camera_tuning import analyze_trial,setting_differences


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def save(path,value):
    with path.open('x',encoding='utf-8') as stream:
        json.dump(value,stream,ensure_ascii=False,indent=2,allow_nan=False)
        stream.write('\n')


trial=read(RUN/'trial.json')
assert trial['execution_write_attempted'] and trial['camera_run_start_confirmed']
assert trial['execution_resend_count']==0 and trial['saved_log']['drop_count']==0
assert trial['log_stop_evidence']['confirmed']
settings=read(RUN/'saved_1ms.settings.json')
previous=read(BASE/'device_prepare_green_corr1p0_01/actual.settings.json')
assert not setting_differences(settings,previous,quantized=True)
assert not read(RUN/'body_geometry_used.json')['marker_detection'].get('body_model')
save(RUN/'operator_postrun_confirmation.json',dict(mouse='B',request_id='B_R90_1600_Lstart13p6_corr1p0_green_20261120_002',
    operator_stopped=True,contact=False,both_off=True,source='current asynchronous user reply: 停止・接触なし・両方OFF',
    execution_commands_sent=1,execution_resend_count=0))
shutil.copy2(BASE/'preflight_green_corr1p0_repeat02_relative.json',RUN/'pre_run_relative_start_review.json')
shutil.copy2(BASE/'operator_ready_green_corr1p0_relative02.json',RUN/'operator_readiness_source.json')
prepared=read(BASE/'device_prepare_green_corr1p0_03/result.json')
save(RUN/'runtime_conditions_for_review.json',dict(pre_run_mm=180.,post_run_mm=180.,
    requested_turn_pre_accel_m_s2=settings['parameters']['pre_accel'],post_accel_m_s2=prepared['straight_after']['acc'],
    post_straight_profile=prepared['straight_after'],wall_ekf=prepared['wall_ekf'],wall_edge=prepared['wall_edge'],
    turn_ekf=prepared['turn_ekf'],source=str(BASE/'device_prepare_green_corr1p0_03/result.json')))
state=read(BASE/'resume_state.json')
state.update(state='CORRECTION1P0_REPEAT02_CAPTURED_BOTH_OFF_ANALYZING',power_state='BOTH_OFF_USER_CONFIRMED',
    robot_connected=False,last_run=str(RUN),last_request_id='B_R90_1600_Lstart13p6_corr1p0_green_20261120_002',
    execution_commands_sent=8,robot_commands_sent=8,execution_resend_count=0,execution_pending=False,
    pending_request_id=None,pending_request_sent=False,current_placement_confirmed=False,current_hands_clear_confirmed=False,
    next_request_sent=False,next_unsent_request_id='B_R90_1600_Lstart13p6_corr1p0_green_20261120_003',
    next_action='Offline full-frame relative analysis and archive. Keep both OFF. Next trial uses rough right-upper/downward placement without exact alignment.',
    next_candidate_applied=False,current_RAM_candidate_applied=False,next_candidate=str(BASE/'device_prepare_green_corr1p0_03/actual.settings.json'),
    last_capture_execution_commands_sent=1,last_capture_contact=False,last_capture_power_off_confirmed=True)
(BASE/'resume_state.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
pending=read(RUN/'deferred_analysis.json')
pending.update(state='RUNNING',started_at=datetime.now(timezone.utc).isoformat(),
    frame_strides=[1],quick_pass_omitted='Full-frame analysis is authoritative; sampled preview omitted to avoid duplicate processing.')
(RUN/'deferred_analysis.json').write_text(json.dumps(pending,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
code=analyze_trial(RUN,1)
pending.update(state='ANALYZED_REVIEW_REQUIRED' if code in (0,2) else 'ANALYSIS_FAILED',returncode=code)
(RUN/'deferred_analysis.json').write_text(json.dumps(pending,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(full_returncode=code,run=str(RUN)),ensure_ascii=False),flush=True)
