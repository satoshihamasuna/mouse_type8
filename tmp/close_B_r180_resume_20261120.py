"""3本候補保存後の再開状態を閉じる。機器操作・ファーム反映なし。"""
from pathlib import Path
import json

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_r180_20261120_v1'
read=lambda p:json.loads(Path(p).read_text(encoding='utf-8-sig'))
final=read(BASE/'final_single_turn_candidate/candidate.json')
review=read(BASE/'r47p5_three_review_v1/video_comparison.json')
assert final['identical_parameter_run_count']==3 and final['current_power_state']=='BOTH_OFF_USER_CONFIRMED'
state=read(BASE/'resume_state.json')
stages=['baseline01','angle_c0p5_s35_02','r47p5_01','r47p5_repeat_01','r47p5_repeat_02']
counts=dict(execution=0,turn_set=0,straight_set=0,preset=0)
consumed=[]
for stage in stages:
    session=BASE/stage/'local_execution'
    for path in session.rglob('events.jsonl'):
        for line in path.read_text(encoding='utf-8').splitlines():
            event=json.loads(line);command=event.get('command','')
            if event.get('event')=='execution_write_attempt':counts['execution']+=1
            if event.get('event')=='settings_write_attempt':counts['turn_set']+=1
            if event.get('event')=='shell_write_attempt' and command.startswith('debug straight set '):counts['straight_set']+=1
            if event.get('event')=='shell_write_attempt' and command=='debug turn long_r180 preset 1600':counts['preset']+=1
    for run in session.glob('run_*'):
        trial=read(run/'trial.json')
        assert trial['execution_resend_count']==0
        consumed.append(run.name.removeprefix('run_'))
assert counts['execution']==5
last=review['comparisons'][-1]
last_run=Path(last['source_trial']).parent
tracking=read(last_run/'tracking_1ms_review_v2/verification.json')
state.update(state='RIGHT180_SINGLE_TURN_CANDIDATE_3_VALID_RUNS_SAVED_BOTH_OFF',
    power_state='BOTH_OFF_USER_CONFIRMED',ble_connected=False,execution_pending=False,
    current_placement_user_confirmed=False,current_clearance_confirmed=False,current_hands_clear_confirmed=False,
    current_position_state='最終走行後の停止位置。次の実機作業では新規ID・新しい現在配置を確認する。',
    execution_commands_sent=counts['execution'],execution_resend_count=0,
    settings_write_commands_sent=counts['turn_set']+counts['straight_set'],preset_selections_sent=counts['preset'],
    command_count_sources='各取得のevents.jsonlから実書込みイベントを再集計。',
    consumed_execution_request_ids=consumed,next_unsent_request_id=None,following_unsent_request_id=None,
    next_request_sent=False,candidate='final_single_turn_candidate/B_R180_1600_single_turn_candidate.settings.json',
    candidate_manifest='final_single_turn_candidate/candidate.json',
    final_report='final_single_turn_candidate/README.md',
    completed_identical_repeat_stages=['r47p5_01','r47p5_repeat_01','r47p5_repeat_02'],
    prepared_identical_repeat_stages=[],prepared_repeat_execution_allowed_after_first_shape_review=False,
    r47p5_identical_valid_sample_count=3,r47p5_identical_additional_samples_needed=0,
    last_executed_request_id='B_R180_1600_r47p5_c0p5_s35_repeat_20261120_002',
    last_run=str(last_run.relative_to(BASE)),last_actual_settings=str((last_run/'saved_1ms.settings.json').relative_to(BASE)),
    saved_log_frames=tracking['sample_count'],saved_log_drop_count=0,
    measured_log_period_ms=tracking['measured_log_period_ms'],gyro_through_lend_deg=tracking['gyro_through_lend_deg'],
    battery_min_v=tracking['battery_min_v'],last_tracking_rms_rad_s=tracking['tracking']['tracking_rms_rad_s'],
    last_run_archive_pending=False,last_relative_path_rotation_deg=last['entry_to_exit_path_rotation_deg'],
    last_exit_spacing_at_forward45_mm=last['exit_line_right_at_forward45_mm'],last_forward_apex_mm=last['forward_apex_mm'],
    last_video_review='r47p5_three_review_v1/video_comparison.json',
    late_stationary_video_observations=last['stationary_observations'],
    late_stationary_scatter_rms_mm=last['stationary_scatter_rms_mm'],
    archives_verified=final['archives'],archive_folder=final['archives'][-1]['folder'],
    final_statistics=final['statistics'],firmware_parameter_adoption_performed=False,
    source_params_changed=False,firmware_write_performed=False,
    future_placement_policy='ユーザー指定の概略配置確認→走行動画で入口相対補正を継続する。',
    next_action='単体候補保存まで完了。現在は両方OFF・接続なし・未送信指令なし。Params反映・連続/迷路検証・左180度は未実施。消費済みIDを再送しない。')
with (BASE/'resume_state.json').open('w',encoding='utf-8') as output:
    json.dump(state,output,ensure_ascii=False,indent=2,allow_nan=False);output.write('\n')
print(json.dumps(dict(state=state['state'],counts=counts,candidate=state['candidate'],
    remaining_execution_request=None),ensure_ascii=False),flush=True)
