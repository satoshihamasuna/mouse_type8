"""B1600右90°・緑前マーカー初回の保存済み資料を確定する。機器操作なし。"""
from pathlib import Path
import hashlib
import json
import shutil

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT/'tools/logs/pixel_camera/turn1600_B_r90_20261120_v1'
RUN = BASE/'cli_session04/run_B_R90_1600_Lstart13p6_green_20261120_001'
REVIEW = BASE/'green_Lstart13p6_001_video_review_v1'


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def write_new(path, value):
    with path.open('x', encoding='utf-8') as output:
        json.dump(value, output, ensure_ascii=False, indent=2, allow_nan=False)
        output.write('\n')


assert not (RUN/'archive.json').exists(), '一式保存済みの元資料は変更しない'
document = read(REVIEW/'video_comparison.json')
result = document['comparisons'][0]
report = read(RUN/'full/report.json')
summary = read(RUN/'full/summary.json')
tracking = read(RUN/'tracking_1ms_review_v2/verification.json')
postrun = read(RUN/'operator_postrun_confirmation.json')
assert postrun['operator_confirmed_after_run']['both_power_off']
assert result['relative_review_eligible']
assert tracking['sample_count'] == 332 and tracking['cnt_contiguous']

hashes = read(BASE/'preparation.json')['source_hashes']
source_checks = {}
for name in ('Params/Param_B/turn_1600/turn_1600.cpp', 'Params/Param_B/run_param_B.cpp',
             'Params/mouse_config.h', 'System/Src/myshell.cpp'):
    expected = hashes[name.replace('/', '\\')]
    source_checks[name] = hashlib.sha256((ROOT/name).read_bytes()).hexdigest() == expected
assert all(source_checks.values()), source_checks

# 同じ固定6条件を全て残す。残差の小さい窓だけを選ばない。
pose = pd.read_csv(RUN/'full/apparent_floor_marker_pose.csv')
relative = pd.read_csv(RUN/'full/relative_marker_midpoint.csv')
video_t = pose.time_video_s.to_numpy()
valid = relative.detected.to_numpy(bool)
xy = relative[['center_dx_mm', 'center_dy_mm']].to_numpy()
entry_window = result['entry_window_pts_s']
exit_window = result['exit_window_pts_s']
trial = read(RUN/'trial.json')
before = valid & (video_t >= trial['video_motion_start_estimate_s']-.6)
before &= video_t <= trial['video_motion_start_estimate_s']+.1
origin = np.median(xy[before], axis=0)


def line(points):
    assert len(points) >= 15
    direction = np.linalg.svd(points-points.mean(axis=0), full_matrices=False)[2][0]
    if direction @ (points[-1]-points[0]) < 0:
        direction = -direction
    return direction


exit_points = xy[valid & (video_t >= exit_window[0]) & (video_t <= exit_window[1])]
exit_direction = line(exit_points)
sensitivity = []
for start_trim_ms in (0, 20, 40):
    for end_trim_ms in (0, 10):
        selected = valid & (video_t >= entry_window[0]+start_trim_ms/1000)
        selected &= video_t <= entry_window[1]-end_trim_ms/1000
        entry = line(xy[selected])
        basis = np.column_stack([[-entry[1], entry[0]], entry])
        centre = (exit_points.mean(axis=0)-origin) @ basis
        exit_relative = exit_direction @ basis
        rotation = np.degrees(np.arctan2(exit_direction[1], exit_direction[0])
                              - np.arctan2(entry[1], entry[0]))
        rotation = (rotation+180) % 360-180
        offset = centre[1]+(90-centre[0])*exit_relative[1]/exit_relative[0]-270
        sensitivity.append(dict(entry_start_trim_ms=start_trim_ms, entry_end_trim_ms=end_trim_ms,
            observations=int(selected.sum()), rotation_deg=float(rotation), exit_offset_mm=float(offset)))
write_new(REVIEW/'entry_window_sensitivity.json', dict(variants=sensitivity,
    policy='固定した全6条件を保存。最良窓の選択なし。出口窓と静止原点は共通。',
    rotation_range_deg=[min(x['rotation_deg'] for x in sensitivity), max(x['rotation_deg'] for x in sensitivity)],
    exit_offset_range_mm=[min(x['exit_offset_mm'] for x in sensitivity), max(x['exit_offset_mm'] for x in sensitivity)]))

qa = dict(mouse='B', motion='long_r90', preset_mm_s=1600,
    front_colour='green', rear_colour='red', Lstart_mm=13.6, Lend_mm=26.31,
    correction_deg=1.5, suction=tracking['suction'],
    full_evaluated_frames=report['quality']['frames'],
    calibration_excluded_frames=report['quality']['frames']-report['quality']['valid_frames'],
    missing_calibration_frame_excluded_from_position_evaluation=True,
    actual_four_aruco_detection_fraction=report['quality']['marker_calibration']['four_aruco_detection_rate'],
    per_frame_registration_fraction=report['quality']['marker_calibration']['calibration_frame_rate'],
    body_detection_given_calibration=report['quality']['body_detection_rate_given_calibration'],
    motion_window_observation_fraction=result['motion_observation_rate'],
    fixed_sample_8frames_visually_reviewed=True,
    correct_front_green_rear_red_round_stickers_on_reviewed_samples=True,
    run_LED_on_and_off_samples_correct=True,
    original_auto_stop_confirmed=summary['auto_stop_confirmed'],
    original_video_stationary=summary['video_stationary'], original_summary_preserved=True,
    original_false_stop_reason='集計対象motion.csvが停止前後に切り出され、撮影終了時の長い静止区間を含まない。',
    capture_stop_evidence=result['actual_capture_stop_evidence'],
    late_video_stationary_observations=result['stationary_observations'],
    late_video_stationary_scatter_rms_mm=result['stationary_scatter_rms_mm'],
    operator_stopped_no_contact_both_off=True,
    relative_shape_review_eligible=True, fixed_board_placement_passed=False,
    repeatability_established=False, green_front_group_repeat_count=1,
    earlier_yellow_marker_runs_pooled_as_repeats=False,
    relative_exit_rotation_deg=result['entry_to_exit_path_rotation_deg'],
    relative_exit_line_offset_mm=result['exit_line_offset_from270_mm'],
    entry_fit_rms_mm=result['entry_line']['rms_mm'],
    battery_min_v=tracking['battery_min_v'],
    firmware_source_hashes_unchanged=source_checks,
    firmware_write_performed=False, parameter_adoption_allowed=False,
    LED_boundary_not_used_for_Lend=True,
    LED_timing_limitation='ソースでは全ターン区間の点灯だが、実機バイナリの同一性・撮影露光時刻との対応が未確認。軌跡モデル同期と独立な確定境界とは扱わない。',
    EKF_identification=dict(status='NOT_IDENTIFIABLE_WITH_CURRENT_B_TOOL_GEOMETRY',
        reason='既存同定ツールはAと中心IMUを前提。Bの機軸・IMU/マーカー配置・高さの実測校正とB対応が不足。',
        A_profile_applied_to_B=False, mode_changed=False),
    decision='Lstart13.6 mm、Lend26.31 mm、補償1.5度を維持し、充電・同一開始配置後に緑前マーカーで反復確認する。',
    next_candidate_sent=False, execution_commands_sent=1, execution_resend_count=0)
write_new(RUN/'manual_quality_review.json', qa)
shutil.copytree(REVIEW, RUN/'video_shape_review_v1')
shutil.copy2(__file__, RUN/'review_finalization_source.py')

green_validation = dict(read(BASE/'green_front_static_validation_v1.json'))
green_validation.update(LED_on_validation='走行中の固定8点を目視。LED点灯4点・消灯3点・終了点灯1点で実ステッカーの位置と一致。',
    moving_marker_quality='全1224評価点中、較正が有効な1223点で前後マーカー検出100%。無効な較正1点は位置評価から除外。',
    robot_commands_sent_after_marker_replacement=1,
    motion_quality_review=str(RUN/'manual_quality_review.json'))
write_new(BASE/'green_front_motion_validation_v1.json', green_validation)
write_new(RUN/'green_front_motion_validation_v1.json', green_validation)

deferred = read(RUN/'deferred_analysis.json')
deferred.update(state='ANALYZED_REVIEWED_NOT_ADOPTED',
    quality_review='manual_quality_review.json', archive_completion_receipt='archive.json',
    source_params_changed=False, both_power_off_user_confirmed=True)
(RUN/'deferred_analysis.json').write_text(json.dumps(deferred, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')

state = read(BASE/'resume_state.json')
state.update(state='GREEN_LSTART13P6_RUN01_REVIEWED_READY_FOR_ARCHIVE_BOTH_OFF',
    last_relative_exit_rotation_deg=result['entry_to_exit_path_rotation_deg'],
    last_relative_exit_forward_offset_mm=result['exit_line_offset_from270_mm'],
    relative_exit_rotation_deg=result['entry_to_exit_path_rotation_deg'],
    relative_exit_forward_offset_mm=result['exit_line_offset_from270_mm'],
    last_quality_review=str(RUN/'manual_quality_review.json'),
    last_video_review=str(RUN/'video_shape_review_v1/video_comparison.json'),
    same_setting_repeat_count=1,
    same_setting_repeat_count_note='緑前・赤後の反復群1本目。旧黄色前の試行は別群。',
    next_candidate=str(BASE/'device_prepare_green_Lstart13p6_01/actual.settings.json'),
    next_static_tag='green_Lstart13p6_repeat02_placed01',
    last_request_id='B_R90_1600_Lstart13p6_green_20261120_001',
    pending_request_id=None, pending_request_sent=False, execution_pending=False,
    next_unsent_request_id='B_R90_1600_Lstart13p6_green_20261120_002',
    next_request_sent=False, current_placement_confirmed=False, current_hands_clear_confirmed=False,
    archive_verified=False, last_archive_verified=False,
    next_action='一式保存・照合後、充電完了と両方OFFで開始位置に戻した旨の返答を待つ。ONはまだ依頼しない。',
    firmware_source_hashes_unchanged=source_checks)
(BASE/'resume_state.json').write_text(json.dumps(state, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
print(json.dumps(dict(finalized=True, relative_rotation_deg=qa['relative_exit_rotation_deg'],
    relative_exit_line_offset_mm=qa['relative_exit_line_offset_mm'], entry_sensitivity=sensitivity,
    both_off=True, candidate_changed=False), ensure_ascii=False, indent=2))
