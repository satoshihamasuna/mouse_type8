"""保存済みの同一開始姿勢2本を比較し、電源OFF中に3本目だけを準備する。機器操作なし。"""
from pathlib import Path
from datetime import datetime, timezone, timedelta
import hashlib
import json
import math
import py_compile
import shutil

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tools/logs/pixel_camera'
FIRST = BASE / 'cli_out_l45_cminus05_pose_retry_ready_20261111_165540'
SECOND = BASE / 'cli_out_l45_cminus05_pose_repeat02_ready_20261111_170254'
CHECKPOINT = BASE / 'current_ui_tuning_session.json'

def read(path):
    return json.loads(path.read_text(encoding='utf-8'))

def save_new(path, value):
    with path.open('x', encoding='utf-8') as file:
        file.write(json.dumps(value, ensure_ascii=False, indent=2) + '\n')

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

state = read(CHECKPOINT)
assert state['operator_power_off_confirmed'] and not state['any_execution_pending']
job_path = SECOND / 'jobs/out_l45_cminus05_pose_cli_003.json'
job = read(job_path)
assert job['state'] == 'DONE' and job['result']['full_returncode'] == 0
assert Path(job['result']['archive']).is_dir()
closed_path = SECOND / 'jobs/close_after_full_review_001.json'
assert read(closed_path)['state'] == 'DONE'

metric_paths = [
    FIRST / 'reviews/run_out_l45_cminus05_pose_cli_002/metrics.json',
    SECOND / 'reviews/run_out_l45_cminus05_pose_cli_003/metrics.json',
]
metrics = [read(path) for path in metric_paths]
poses = [read(folder / 'recorded_start_pose_full_diagnostic.json') for folder in (FIRST, SECOND)]
assert all(pose['consistent_with_static_pose_condition'] for pose in poses)
assert all(item['parameters'] == metrics[0]['parameters'] for item in metrics)
assert all(item['pid'] == metrics[0]['pid'] and item['feedforward'] == metrics[0]['feedforward'] for item in metrics)
assert all(item['motion_observation_rate'] >= .95 and item['turn_saturation_rows'] == 0 for item in metrics)
assert all(item['endpoint_error_norm_mm'] <= 3 and abs(item['exit_path_direction_error_right_positive_deg']) <= .5 for item in metrics)
comparison_path = SECOND / 'controlled_pose_two_run_comparison.json'
comparison = dict(
    recorded_at=datetime.now(timezone.utc).isoformat(), mouse='A', motion='out_l45', preset=500,
    parameters=metrics[0]['parameters'], pid=metrics[0]['pid'], feedforward=metrics[0]['feedforward'],
    controlled_initial_pose_trial_count=2, frame_scope='all_frames',
    thresholds_unchanged=dict(endpoint_norm_mm=3, exit_abs_error_deg=.5, initial_centre_mm=3, initial_heading_deg=1),
    all_two_runs_passed=True, formal_adoption_allowed=False, three_trial_review_not_yet_complete=True,
    endpoint_mean_right_mm=sum(item['endpoint_right_mm'] for item in metrics)/2,
    endpoint_mean_forward_mm=sum(item['endpoint_forward_mm'] for item in metrics)/2,
    maximum_endpoint_error_norm_mm=max(item['endpoint_error_norm_mm'] for item in metrics),
    maximum_abs_exit_error_deg=max(abs(item['exit_path_direction_error_right_positive_deg']) for item in metrics),
    endpoint_pair_spread_mm=math.hypot(metrics[1]['endpoint_right_mm']-metrics[0]['endpoint_right_mm'], metrics[1]['endpoint_forward_mm']-metrics[0]['endpoint_forward_mm']),
    results=[dict(metrics_json=str(path), endpoint_error_norm_mm=item['endpoint_error_norm_mm'],
        exit_path_direction_error_right_positive_deg=item['exit_path_direction_error_right_positive_deg'],
        motion_observation_rate=item['motion_observation_rate'], actual_recorded_initial_pose=pose)
        for path,item,pose in zip(metric_paths,metrics,poses)],
    sparse_boundary_failure_and_dense_adjudication_preserved=True,
    dense_adjudication_decision=str(SECOND/'pose_dense_adjudication_decision.json'),
    previous_broad_pose_runs_retained_without_reclassification=True,
    limitations_ja='相対目印中点と軌跡方向の比較。絶対車軸位置・高さ補正yaw・独立同期・連続走行は未検証。',
    input_sha256={str(path):sha(path) for path in metric_paths + [FIRST/'recorded_start_pose_full_diagnostic.json',SECOND/'recorded_start_pose_full_diagnostic.json',job_path,closed_path,SECOND/'pose_dense_adjudication_decision.json']},
)
save_new(comparison_path, comparison)

stamp = datetime.now(timezone(timedelta(hours=9))).strftime('%Y%m%d_%H%M%S')
next_folder = BASE / ('cli_out_l45_cminus05_pose_repeat03_ready_' + stamp)
next_folder.mkdir()
helpers = ['static_probe.py','submit_once.py','verify_applied_candidate.py','verify_af_capture.py','review_single_out_l45.py','verify_recorded_start_pose.py']
for name in helpers:
    content = (SECOND/name).read_text(encoding='utf-8').replace('out_l45_cminus05_pose_cli_003','out_l45_cminus05_pose_cli_004')
    if name == 'verify_applied_candidate.py':
        assert 'candidate_degree_correction_deg=0,' in content
        content = content.replace('candidate_degree_correction_deg=0,','candidate_degree_correction_deg=actual[\'parameters\'][\'correction\'],')
    (next_folder/name).write_text(content,encoding='utf-8')
    py_compile.compile(str(next_folder/name),doraise=True)
for name in ['config.json','initial_pose_reference.json','initial_pose_reference.png','initial_pose_reference_zoom.png','out_l45_hole_avoidance_layout_plan.json']:
    shutil.copy2(SECOND/name,next_folder/name)
for name in ['expected_candidate.settings.json','baseline_device.settings.json']:
    shutil.copy2(SECOND/'candidate.settings.json',next_folder/name)
plan = read(SECOND/'resume_plan.json')
plan.update(recorded_at=datetime.now(timezone.utc).isoformat(), prepared_cli_folder=str(next_folder),
    current_power='OFF_CONFIRMED',current_ble='CLOSED',motor_request_id='out_l45_cminus05_pose_cli_004',
    controlled_reproducibility_trial_number=3,valid_same_setting_controlled_metrics=[str(path) for path in metric_paths],
    previous_two_run_comparison_json=str(comparison_path),candidate_device_applied=False,candidate_executed=False,
    static_primary_review='review-dense',static_primary_window_s=[2,2.8],static_primary_stride=1,
    static_review_plan_ja='同じ2.0–2.8秒の全192フレームを初回から使用する。閾値・基準姿勢は変更しない。静止確認不成立時は走行しない。',
    no_failed_window_reselection=True,one_run_then_power_off_before_long_review=True)
save_new(next_folder/'resume_plan.json',plan)
save_new(next_folder/'prepared_helpers_validation.json',dict(recorded_at=datetime.now(timezone.utc).isoformat(),
    syntax_compilation_passed=True,no_hardware_operations=True,prepared_while_mouse_and_hy0020_off=True,
    current_operator_readiness_required=True,one_motor_request_only='out_l45_cminus05_pose_cli_004',
    dense_report_is_guard_and_review_source=True,unchanged_initial_pose_reference_sha256=sha(next_folder/'initial_pose_reference.json'),
    input_sha256={str(next_folder/name):sha(next_folder/name) for name in helpers+['config.json','resume_plan.json','expected_candidate.settings.json','baseline_device.settings.json']}))

save_new(SECOND/'checkpoint_before_full_review_record.json',state)
review_ids=state.setdefault('full_review_request_ids_recorded',[])
if 'out_l45_cminus05_pose_cli_003' not in review_ids:
    state['cli_completed_full_frame_reviews']+=1
    state['cli_valid_position_reviews']+=1
    state['valid_position_reviews']+=1
    review_ids.append('out_l45_cminus05_pose_cli_003')
state.update(state='OUT_L45_CMINUS05_POSE_TWO_VALID_POWER_OFF_REPEAT03_PREPARED',gateway_stopped=True,
    cli_server_started=False,cli_hardware_connection_verified=False,any_execution_pending=False,
    controlled_initial_pose_trial_count=2,valid_af_locked_candidate_trials=2,
    candidate_reproducibility_confirmed=False,candidate_reproducibility_measured=True,
    candidate_single_nominal_geometry_passed=True,candidate_position_not_yet_validated=False,
    current_controlled_two_run_comparison_json=str(comparison_path),
    last_review_metrics_json=str(metric_paths[-1]),last_full_frame_metrics={key:metrics[-1][key] for key in ['endpoint_right_mm','endpoint_forward_mm','endpoint_error_norm_mm','exit_path_direction_error_right_positive_deg','motion_observation_rate','auto_stop_confirmed']},
    last_recorded_start_pose_diagnostic_json=str(SECOND/'recorded_start_pose_full_diagnostic.json'),
    last_run_stop_verified_in_video_and_log=True,prepared_next_cli_folder=str(next_folder),
    next_candidate_plan_json=str(next_folder/'resume_plan.json'),next_initial_pose_reference_json=str(next_folder/'initial_pose_reference.json'),
    conditional_one_shot_authorized=False,conditional_one_shot_consumed=True,
    operator_confirmed_next_placement=False,next_operator_placement_required=True,
    fresh_uart_power_confirmation_required=True,resume_requires_new_ready_message=True,
    static_probe_required_before_motion=True,parameter_adoption_allowed=False,
    planned_next_lstart_mm=33.9,planned_next_lend_mm=11.8,planned_next_correction_deg=-.5,
    updated_at=datetime.now(timezone.utc).isoformat())
CHECKPOINT.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(next_folder=str(next_folder),all_two_runs_passed=True,
    endpoint_pair_spread_mm=comparison['endpoint_pair_spread_mm'],formal_adoption_allowed=False),ensure_ascii=False))
