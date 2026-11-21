"""B右90度・補正1.0度の2本目を根拠付きで保存する。機器操作なし。"""
from pathlib import Path
import hashlib
import json
import shutil
import sys

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_r90_20261120_v1'
RUN=BASE/'cli_session09/run_B_R90_1600_Lstart13p6_corr1p0_green_20261120_002'
REVIEW=BASE/'green_correction1p0_two_video_review_v1'
sys.path.insert(0,str(ROOT/'tools'))
from pixel_camera_tuning import setting_differences
from pixel_camera_archive import archive_trial,verify_archive,write_index


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def save(path,value):
    with path.open('x',encoding='utf-8') as output:
        json.dump(value,output,ensure_ascii=False,indent=2,allow_nan=False)
        output.write('\n')


assert not (RUN/'archive.json').exists()
assert read(RUN/'operator_postrun_confirmation.json')['both_off']
assert read(BASE/'device_prepare_green_corr1p0_03/handoff_closure.json')['ble_disconnected']
report=read(RUN/'full/report.json')
summary=read(RUN/'full/summary.json')
tracking=read(RUN/'tracking_1ms_review_v2/verification.json')
comparison=read(REVIEW/'video_comparison.json')
assert comparison['repeat_count']==2 and comparison['all_relative_reviews_eligible']
current=comparison['comparisons'][1]
assert current['label']=='corr1p0_002'
assert not setting_differences(read(RUN/'saved_1ms.settings.json'),read(BASE/'device_prepare_green_corr1p0_01/actual.settings.json'),quantized=True)
prepared=read(BASE/'preparation.json')['source_hashes']
source_names=('Params/Param_B/turn_1600/turn_1600.cpp','Params/Param_B/run_param_B.cpp','Params/mouse_config.h','System/Src/myshell.cpp')
checks={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==prepared[name.replace('/','\\')] for name in source_names}
assert all(checks.values())
quality=dict(mouse='B',motion='long_r90',preset_mm_s=1600,correction_deg=1.,Lstart_mm=13.6,Lend_mm=26.31,
    full_evaluated_frames=report['quality']['frames'],valid_frames=report['quality']['valid_frames'],
    motion_observation_fraction=current['motion_observation_rate'],motion_calibration_fraction=current['motion_calibration_rate'],
    detector=report['quality'].get('detector'),detection_profile_changed=False,body_model_enabled=False,
    fixed8_actual_front_green_rear_red_identity_manually_confirmed=True,
    manual_identity_evidence='marker_identity_fixed8.jpg',
    placement_mode='relative_to_measured_motion_entry_direction',exact_start_target_required=False,
    static_full_frame_alignment_claimed=False,preflight='pre_run_relative_start_review.json',
    original_auto_stop_confirmed=summary['auto_stop_confirmed'],original_summary_preserved=True,
    capture_stop_evidence=current['actual_capture_stop_evidence'],
    late_static_observations=current['stationary_observations'],late_static_scatter_rms_mm=current['stationary_scatter_rms_mm'],
    relative_shape_review_eligible=current['relative_review_eligible'],
    relative_exit_rotation_deg=current['entry_to_exit_path_rotation_deg'],
    relative_exit_line_offset_mm=current['exit_line_offset_from270_mm'],
    correction1p0_group_count=2,baseline_correction1p5_count=3,groups_pooled=False,
    identical_numeric_settings_with_candidate_trial1_verified=True,
    repeatability_target_count=3,repeatability_target_reached=False,
    battery_min_v=tracking['battery_min_v'],operator_stopped_no_contact_both_off=True,ble_closed=True,
    source_params_changed=False,firmware_write_performed=False,firmware_sources_unchanged=checks,
    parameter_adoption_allowed=False,execution_commands_sent=1,execution_resend_count=0,
    fixed_board_alignment_accepted=False,absolute_axle_position_verified=False,
    EKF_identification='B geometry calibration and B tool support remain insufficient; no A coefficients reused.',
    next_action='Charge both OFF, rough right-upper/downward placement, then same-setting third trial after fresh readiness.')
save(RUN/'manual_quality_review.json',quality)
save(REVIEW/'repeat02_manual_quality_review.json',quality)
shutil.copytree(REVIEW,RUN/'video_shape_review_v1')
sources=RUN/'analysis_sources'
sources.mkdir()
names=('tmp/finalize_B_corr1p0_repeat02_20261120.py','tmp/postprocess_B_corr1p0_repeat02_20261120.py',
    'tmp/review_B_relative02_marker_identity_20261120.py','tmp/resume_B_r90_relative_preflight_20261120.py',
    'tmp/review_turn1600_B_video_20261120.py','tools/analysis/video_marker_pose.py',
    'tools/analysis/pixel_camera_relative_pose.py','tools/analysis/pixel_camera_tuning_report.py',
    'tools/marker_detection.py','tools/pixel_camera_detection.py')
for name in names:
    shutil.copy2(ROOT/name,sources/Path(name).name)
save(sources/'source_hashes.json',{name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in names})
pending=read(RUN/'deferred_analysis.json')
pending.update(state='ANALYZED_REVIEWED_NOT_ADOPTED',quality_review='manual_quality_review.json',
    both_power_off_user_confirmed=True,archive_completion_receipt='archive.json')
(RUN/'deferred_analysis.json').write_text(json.dumps(pending,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
archive=archive_trial(RUN/'trial.json')
manifest=verify_archive(archive)
write_index(archive.parent)
state=read(BASE/'resume_state.json')
state.update(state='CORRECTION1P0_REPEAT02_REVIEWED_ARCHIVED_BOTH_OFF',power_state='BOTH_OFF_USER_CONFIRMED',
    last_quality_review=str(RUN/'manual_quality_review.json'),last_video_review=str(REVIEW/'video_comparison.json'),
    last_relative_exit_rotation_deg=current['entry_to_exit_path_rotation_deg'],relative_exit_rotation_deg=current['entry_to_exit_path_rotation_deg'],
    last_relative_exit_forward_offset_mm=current['exit_line_offset_from270_mm'],relative_exit_forward_offset_mm=current['exit_line_offset_from270_mm'],
    last_battery_min_v=tracking['battery_min_v'],same_setting_repeat_count=2,next_candidate_group_run_count=2,
    baseline_same_setting_repeat_count=3,exact_start_target_required=False,firmware_source_hashes_unchanged=checks,
    same_setting_repeat_count_note='補正1.0度群2本。補正1.5度基準3本とは別群。',
    archive_folder=str(archive),last_archive_folder=str(archive),archive_verified=True,last_archive_verified=True,
    next_action='Await charging/rough placement reply while both OFF. No precise initial alignment. Same candidate third trial after fresh ON/clearance/hands-clear.',
    next_static_tag='green_corr1p0_repeat03_relative01',execution_pending=False,pending_request_id=None,pending_request_sent=False)
(BASE/'resume_state.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(archived=str(archive),file_count=len(manifest['files']),verified=True,
    angle_deg=current['entry_to_exit_path_rotation_deg'],exit_offset_mm=current['exit_line_offset_from270_mm'],
    candidate_count=2,parameter_adoption_allowed=False),ensure_ascii=False))
