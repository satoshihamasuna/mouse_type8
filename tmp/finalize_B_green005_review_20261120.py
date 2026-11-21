"""B1600右90°・緑005の品質確認と緑2本比較を確定する。機器操作なし。"""
from pathlib import Path
import hashlib
import json
import shutil
import copy

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT/'tools/logs/pixel_camera/turn1600_B_r90_20261120_v1'
RUN = BASE/'cli_session07/run_B_R90_1600_Lstart13p6_green_20261120_005'
REVIEW = BASE/'green_Lstart13p6_three_review_v1'


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def write_new(path,value):
    with path.open('x',encoding='utf-8') as output:
        json.dump(value,output,ensure_ascii=False,indent=2,allow_nan=False)
        output.write('\n')


assert not (RUN/'archive.json').exists()
document = read(REVIEW/'video_comparison.json')
result = next(item for item in document['comparisons'] if item['label']=='green005')
assert document['identical_device_settings_verified'] and result['relative_review_eligible']
assert read(RUN/'operator_postrun_confirmation.json')['both_off']
report = read(RUN/'full/report.json')
summary = read(RUN/'full/summary.json')
tracking = read(RUN/'tracking_1ms_review_v2/verification.json')
geometry = read(RUN/'body_geometry_used.json')
preflight = read(RUN/'pre_run_green_static_validation.json')
assert not geometry['marker_detection'].get('body_model')
prepared = read(BASE/'preparation.json')['source_hashes']
checks = {name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==prepared[name.replace('/','\\')]
    for name in ('Params/Param_B/turn_1600/turn_1600.cpp','Params/Param_B/run_param_B.cpp',
                 'Params/mouse_config.h','System/Src/myshell.cpp')}
assert all(checks.values()), checks

# 入口窓の固定6条件。最良条件の選択はしない。
pose = pd.read_csv(RUN/'full/apparent_floor_marker_pose.csv')
relative = pd.read_csv(RUN/'full/relative_marker_midpoint.csv')
valid = relative.detected.to_numpy(bool)
t = pose.time_video_s.to_numpy()
xy = relative[['center_dx_mm','center_dy_mm']].to_numpy()
trial = read(RUN/'trial.json')
before = valid & (t>=trial['video_motion_start_estimate_s']-.6) & (t<=trial['video_motion_start_estimate_s']+.1)
origin = np.median(xy[before],axis=0)


def line(points):
    assert len(points)>=15
    direction = np.linalg.svd(points-points.mean(axis=0),full_matrices=False)[2][0]
    return direction if direction @ (points[-1]-points[0])>=0 else -direction


exit_window = result['exit_window_pts_s']
exit_points = xy[valid & (t>=exit_window[0]) & (t<=exit_window[1])]
exit_direction = line(exit_points)
sensitivity = []
for start_ms in (0,20,40):
    for end_ms in (0,10):
        a,b = result['entry_window_pts_s']
        selected = valid & (t>=a+start_ms/1000) & (t<=b-end_ms/1000)
        entry = line(xy[selected])
        basis = np.column_stack([[-entry[1],entry[0]],entry])
        centre = (exit_points.mean(axis=0)-origin) @ basis
        direction = exit_direction @ basis
        rotation = np.degrees(np.arctan2(exit_direction[1],exit_direction[0])-np.arctan2(entry[1],entry[0]))
        sensitivity.append(dict(entry_start_trim_ms=start_ms,entry_end_trim_ms=end_ms,
            observations=int(selected.sum()),rotation_deg=float((rotation+180)%360-180),
            exit_offset_mm=float(centre[1]+(90-centre[0])*direction[1]/direction[0]-270)))
sensitivity_doc = dict(variants=sensitivity,best_window_selected=False,
    rotation_range_deg=[min(row['rotation_deg'] for row in sensitivity),max(row['rotation_deg'] for row in sensitivity)],
    exit_offset_range_mm=[min(row['exit_offset_mm'] for row in sensitivity),max(row['exit_offset_mm'] for row in sensitivity)])
write_new(REVIEW/'green005_entry_window_sensitivity.json',sensitivity_doc)

# 次の候補は角度補償だけを0.5度下げる。実機反映・採用とは別。
actual_path = BASE/'device_prepare_green_Lstart13p6_05/actual.settings.json'
actual = read(actual_path)
candidate = {key:copy.deepcopy(actual[key]) for key in ('schema_version','mouse','mouse_identity_source',
    'motion_name','preset_speed_mm_s','turn_count','parameters','pid','feedforward','suction')}
candidate['parameters']['correction'] = 1.0
candidate.update(source='保存済み緑3本の旋回角を根拠とする未適用候補。B専用、角度補正1.5→1.0度のみ。',
    basis_settings_path=str(actual_path),basis_settings_sha256=hashlib.sha256(actual_path.read_bytes()).hexdigest())
candidate_path = BASE/'candidate_green_correction1p0_Lstart13p6.settings.json'
write_new(candidate_path,candidate)
rows = document['comparisons']
assert len(rows)==3 and all(row['relative_review_eligible'] for row in rows)
rotations = np.array([row['entry_to_exit_path_rotation_deg'] for row in rows])
offsets = np.array([row['exit_line_offset_from270_mm'] for row in rows])
assert np.all(rotations<90)
change_review = dict(mouse='B',motion='long_r90',preset_mm_s=1600,
    baseline_comparison=str(REVIEW/'video_comparison.json'),
    baseline_comparison_sha256=hashlib.sha256((REVIEW/'video_comparison.json').read_bytes()).hexdigest(),
    baseline_count=3,all_three_under90=True,
    rotation_median_deg=float(np.median(rotations)),rotation_mean_deg=float(rotations.mean()),
    rotation_range_deg=[float(rotations.min()),float(rotations.max())],
    exit_line_offset_median_mm=float(np.median(offsets)),exit_line_offset_mean_mm=float(offsets.mean()),
    exit_line_offset_range_mm=[float(offsets.min()),float(offsets.max())],
    parameter_changes={'parameters.correction':{'from_deg':1.5,'to_deg':1.0}},
    all_other_numeric_fields_unchanged=True,candidate_file=str(candidate_path),
    motion_degree_formula='Task/Src/init_motion.cpp: degree + degree_correction; -90+1.5=-88.5 -> -90+1.0=-89.0',
    commanded_right_angle_increase_deg=.5,
    measured_improvement_verified=False,applied=False,firmware_params_modified=False,
    validation_plan='同じLstart13.6/Lend26.31/半径-52/速度1.6/吸引650/PID/FFで角度補正だけを比較。独立した候補3本で出口方向・位置・ばらつきを再評価。',
    limitation='基準3本の角度範囲は約1.17度。1本で0.5度の効果を確定せず、反復平均・中央値と各試行を比較する。電圧も一定ではない。')
write_new(REVIEW/'correction1p0_candidate_not_applied.json',change_review)
write_new(RUN/'next_candidate.not_applied.settings.json',candidate)

qa = dict(mouse='B',motion='long_r90',preset_mm_s=1600,Lstart_mm=13.6,Lend_mm=26.31,
    correction_deg=1.5,suction=tracking['suction'],front_colour='green',rear_colour='red',
    full_evaluated_frames=report['quality']['frames'],
    valid_registered_and_detected_frames=report['quality']['valid_frames'],
    per_frame_registration_fraction=report['quality']['marker_calibration']['calibration_frame_rate'],
    body_detection_given_calibration=report['quality']['body_detection_rate_given_calibration'],
    motion_window_observation_fraction=result['motion_observation_rate'],
    fixed8_marker_identity_and_LED_on_off_manually_confirmed=True,
    green_front_red_rear_round_stickers_correct_on_reviewed_samples=True,
    original_auto_stop_confirmed=summary['auto_stop_confirmed'],
    original_video_stationary=summary['video_stationary'],original_summary_preserved=True,
    false_stop_summary_reason='集計対象motion.csvに撮影終了時の長い静止窓が含まれない。元のfalse判定は保持し、連続ログと遅い動画静止窓を別に評価。',
    capture_stop_evidence=result['actual_capture_stop_evidence'],
    late_video_stationary_observations=result['stationary_observations'],
    late_video_stationary_scatter_rms_mm=result['stationary_scatter_rms_mm'],
    operator_stopped_no_contact_both_off=True,ble_closed=True,
    relative_shape_review_eligible=True,strict_start_static_alignment_passed=preflight['strict_static_alignment_passed'],
    strict_start_static_failure_reasons=preflight['static_check']['failure_reasons'],
    static_marker_yaw_not_used_as_motion_entry_axis=True,
    fixed_board_trajectory_accepted=False,absolute_axle_position_verified=False,
    relative_exit_rotation_deg=result['entry_to_exit_path_rotation_deg'],
    relative_exit_line_offset_mm=result['exit_line_offset_from270_mm'],
    entry_line_rms_mm=result['entry_line']['rms_mm'],entry_window_sensitivity=sensitivity_doc,
    same_settings_green_runs=3,repeatability_target=3,repeatability_sampling_complete=True,
    repeatability_established=False,formal_geometric_adoption_criteria_passed=False,
    battery_min_v=tracking['battery_min_v'],previous_green_battery_min_v=7.25,
    battery_not_held_constant=True,green_marker_group_only=True,
    source_params_changed=False,firmware_write_performed=False,firmware_source_hashes_unchanged=checks,
    parameter_adoption_allowed=False,execution_commands_sent=1,execution_resend_count=0,
    body_model_enabled=False,
    analysis_source_change_note='任意のbody_model検出対応が追加された解析実装。今回の両動画の同一プロファイルにはbody_model設定がなく、従来の2色選択経路のみを使用。',
    LED_boundary_used_as_independent_time_sync=False,
    EKF_identification=dict(status='B_GEOMETRY_AND_TOOL_SUPPORT_NOT_READY',A_profile_applied=False,mode_changed=False),
    decision='長さ13.6/26.31mmは維持。3本とも90度未満のため、次は角度補償だけ1.5→1.0度の未適用候補を比較する。',
    next_candidate_not_applied=change_review)
write_new(RUN/'manual_quality_review.json',qa)
write_new(REVIEW/'green005_manual_quality_review.json',qa)
shutil.copytree(REVIEW,RUN/'video_shape_review_v1')
sources = RUN/'analysis_sources'
sources.mkdir()
source_paths = [Path(__file__),ROOT/'tmp/review_turn1600_B_video_20261120.py',
    ROOT/'tools/analysis/video_marker_pose.py',ROOT/'tools/marker_detection.py']
for source in source_paths:
    shutil.copy2(source,sources/source.name)
write_new(sources/'source_hashes.json',{str(source.relative_to(ROOT)):hashlib.sha256(source.read_bytes()).hexdigest() for source in source_paths})
deferred = read(RUN/'deferred_analysis.json')
deferred.update(state='ANALYZED_REVIEWED_NOT_ADOPTED',quality_review='manual_quality_review.json',
    archive_completion_receipt='archive.json',both_power_off_user_confirmed=True,source_params_changed=False)
(RUN/'deferred_analysis.json').write_text(json.dumps(deferred,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
state = read(BASE/'resume_state.json')
state.update(state='GREEN005_REVIEWED_BOTH_OFF_READY_FOR_ARCHIVE',power_state='BOTH_OFF_USER_CONFIRMED',
    last_relative_exit_rotation_deg=result['entry_to_exit_path_rotation_deg'],
    last_relative_exit_forward_offset_mm=result['exit_line_offset_from270_mm'],
    relative_exit_rotation_deg=result['entry_to_exit_path_rotation_deg'],
    relative_exit_forward_offset_mm=result['exit_line_offset_from270_mm'],
    last_quality_review=str(RUN/'manual_quality_review.json'),last_video_review=str(REVIEW/'video_comparison.json'),
    same_setting_repeat_count=3,last_battery_min_v=tracking['battery_min_v'],
    same_setting_repeat_count_note='補償1.5度の緑前・赤後の基準群3本。次の補償1.0度は未実走の別群。',
    next_candidate=str(candidate_path),next_candidate_applied=False,next_candidate_group_run_count=0,
    next_candidate_change={'parameters.correction':{'from_deg':1.5,'to_deg':1.0}},
    next_static_tag='green_correction1p0_placed01',
    next_unsent_request_id='B_R90_1600_Lstart13p6_corr1p0_green_20261120_001',next_request_sent=False,
    execution_pending=False,pending_request_id=None,pending_request_sent=False,
    archive_verified=False,last_archive_verified=False,firmware_source_hashes_unchanged=checks,
    next_action='一式保存・照合後、両方OFFで元の登録開始点(315,45)mm・緑前が下向きへ戻した返答を待つ。次は未適用の補償1.0度候補。ONはまだ依頼しない。')
(BASE/'resume_state.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(finalized=True,green_runs=3,rotation_deg=qa['relative_exit_rotation_deg'],
    exit_line_offset_mm=qa['relative_exit_line_offset_mm'],sensitivity=sensitivity_doc,
    both_off=True,device_parameter_change=False,next_candidate_saved=str(candidate_path)),ensure_ascii=False))
