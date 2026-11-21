"""実測根拠と単体候補を保存する。既存ファームの値は変更しない。"""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json
import re

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tools/logs/pixel_camera/turn1800_A_inr135_20261120_v1'
OUT = BASE / 'Ls2p5_Le26_c2p4_review_v1'
read = lambda p: json.loads(p.read_text(encoding='utf-8-sig'))


def save(path, value):
    with path.open('x', encoding='utf-8', newline='\n') as file:
        json.dump(value, file, ensure_ascii=False, indent=2, allow_nan=False)
        file.write('\n')


review = read(OUT / 'review.json')
assert review['physical_trials'] == 3 and review['complete_video_and_1ms_trials'] == 2
assert review['all_three_operator_no_contact'] and review['both_powered_off_confirmed']
candidate = review['requested_settings']
assert candidate['mouse'] == 'A' and candidate['motion_name'] == 'in_r135'
assert candidate['preset_speed_mm_s'] == 1800 and candidate['turn_count'] == 1
assert candidate['parameters'] == dict(velo=1.8, r_min=-40., lstart=2.5, lend=26.,
    degree=-135., correction=2.4, pre_accel=6.5)
assert candidate['suction'] == dict(enabled=True, duty=650)
source = ROOT / 'Params/Param_A/turn_1800/turn_1800.cpp'
original_declaration = read(BASE / 'source_extraction.json')['declarations']['table']
normalize = lambda text: re.sub(r'\s+', '', text)
assert normalize(original_declaration) in normalize(source.read_text(encoding='utf-8'))
save(OUT / 'candidate.settings.json', candidate)
metadata = dict(schema_version=1, saved_at=datetime.now(timezone.utc).isoformat(),
    mouse='A', motion='in_r135', preset_mm_s=1800,
    status='SINGLE_TURN_RELATIVE_CANDIDATE',
    candidate_settings=str(OUT / 'candidate.settings.json'),
    review=str(OUT / 'review.json'), comparison_plot=str(OUT / 'trajectories_with_evidence_grades.png'),
    rationale='完全記録2本で出口方向134.82/134.91度、Lend残差0.40～1.25mm。2本目は1msログ欠損のためLend統計から除外し、動画の方向135.51度のみ参照。機体A・吸引650の単体相対候補として保存する。',
    physical_trials=3, complete_video_and_1ms_trials=2, video_only_trials=1,
    angle_range_all_video_deg=review['all_video_exit_angle_range_deg'],
    Lend_residual_norm_range_complete_trials_mm=review['Lend_residual_norm_range_mm'],
    initial_placement_policy='操作者の希望により大まかな確認。位置10mm・見かけ向き5度。観測点数と各使用フレームの登録ArUco条件は維持。',
    coordinate_basis=review['coordinate_basis'], camera_height_mm=1500.,
    pre_run_mm=90., post_diagonal_run_mm=127.27922061357854,
    requested_pre_accel_m_s2=6.5, effective_pre_accel_m_s2=18., post_diagonal_accel_m_s2=20.,
    requested_pid_and_ff_changed_from_original_tuning_baseline=False,
    EKF_parameters_identified=False, continuous_turns_verified=False,
    applied_to_firmware_defaults=False, firmware_build_performed_for_this_task=False,
    this_task_modified_firmware=False, right135_baseline_source_declaration_unchanged=True,
    unrelated_existing_right90_source_edits_preserved=True,
    source_file_current_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    source_declaration=original_declaration,
    original_failed_trial002_preserved=True, trial002_lost_saved_log_not_reconstructed=True,
    source_sidecar_settings_preserved=True,
    all_three_operator_no_contact=True, both_powered_off_confirmed=True,
    execution_pending=False, no_automatic_further_execution=True,
    limitations=review['limitations'],
    next_scope_if_requested='連続接続、最短走行への反映、ファームの既定値更新は別途検証する。')
save(OUT / 'candidate.meta.json', metadata)

pointer = BASE / 'preparation.json'
old = pointer.read_bytes()
with (OUT / 'preparation_before_candidate_saved.json').open('xb') as file:
    file.write(old)
state = read(BASE / 'c2p4_Ls2p5_Le26_03/placement_plan.json')
state.update(state='SINGLE_TURN_RELATIVE_CANDIDATE_SAVED',
    updated_at=metadata['saved_at'], selected_candidate=str(OUT / 'candidate.settings.json'),
    selected_candidate_meta=str(OUT / 'candidate.meta.json'),
    latest_comparison=str(OUT / 'review.json'), last_request_id='A_INR135_1800_c2p4_Ls2p5_Le26_req_20261120_003',
    last_request_executed=True, last_request_saved_log='saved', last_operator_contact_detected=False,
    power_state='Both_OFF_confirmed_after_c2p4_Ls2p5_Le26_003', both_powered_off_confirmed=True,
    execution_pending=False, operator_ready=False, current_placement_confirmed=False,
    current_clearance_confirmed=False, current_hands_clear_confirmed=False,
    current_physical_clearance_verified=False, next_request_id=None, next_candidate_executed=False,
    source_params_changed=False, firmware_defaults_updated_by_this_task=False,
    physical_trials_for_selected_candidate=3, complete_logs_for_selected_candidate=2,
    video_only_logs_for_selected_candidate=1,
    prior_root_preparation_backup=str(OUT / 'preparation_before_candidate_saved.json'))
pointer.write_text(json.dumps(state, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')

assert read(OUT / 'candidate.settings.json')['parameters'] == candidate['parameters']
assert read(pointer)['execution_pending'] is False and read(pointer)['operator_ready'] is False
print(json.dumps(dict(saved_candidate=str(OUT / 'candidate.settings.json'), status=metadata['status'],
    both_powered_off_confirmed=True, this_task_modified_firmware=False), ensure_ascii=False))
