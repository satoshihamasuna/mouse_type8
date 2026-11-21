"""A1800 out135の半径・長さを固定し、角度補正だけの次試行を準備する。"""
from pathlib import Path
from datetime import datetime, timezone
import ast
import copy
import hashlib
import json
import shutil

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tools/logs/pixel_camera/turn1800_A_outr135_20261120_v1'
SOURCE = BASE / 'r40_c3p0_Ls0p5_Le35_stream10_01'
RID = 'A_OUTR135_1800_r40_c3p0_Ls0p5_Le35_s10_20261121_001'
RUN = SOURCE / 'local_execution' / ('run_'+RID)
REVIEW = BASE / 'parallel_analysis' / RID / 'midpoint_actual_motion_v1/midpoint_review.json'
OUT = BASE / 'r40_c3p0_first_trial_review_20261121_v1'
read = lambda p: json.loads(p.read_text(encoding='utf-8-sig'))

def save(path, value):
    with path.open('x', encoding='utf-8') as f:
        json.dump(value, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.write('\n')

review = read(REVIEW)
tracking = read(BASE / 'parallel_analysis' / RID / 'tracking_v1/verification.json')
operator_path = SOURCE / 'local_execution/operator_results' / f'{RID}.json'
operator = read(operator_path)
assert operator['both_powered_off'] and not operator['contact_detected']
assert review['independent_optical_geometry_eligible'] and review['clock']['passed']
prior = read(SOURCE / 'candidate.settings.json')
assert prior['parameters']['r_min'] == -40 and prior['parameters']['correction'] == 3
next_correction = 5.
stages = []
for index in (1,2,3):
    stage = BASE / f'r40_c5p0_Ls0p5_Le35_stream10_{index:02}'
    stage.mkdir(exist_ok=False)
    candidate = copy.deepcopy(prior)
    candidate['parameters']['correction'] = next_correction
    candidate['source'] = 'Provisional fixed-radius/length correction-only trial after qualified136.985deg R40/c3.0 measurement. Not adopted.'
    save(stage / 'candidate.settings.json', candidate)
    for name in ('body_geometry_A_out135_yellow_s20.json','static_pose_after_settle.py','static_settle_policy.json'):
        shutil.copyfile(SOURCE/name, stage/name)
    config = read(SOURCE / 'config.json')
    config['geometry'] = str(stage / 'body_geometry_A_out135_yellow_s20.json')
    save(stage / 'config.json', config)
    rid = f'A_OUTR135_1800_r40_c5p0_Ls0p5_Le35_s10_20261121_{index:03}'
    wrapper = (SOURCE / 'run_recorded_once.py').read_text(encoding='utf-8-sig')
    assert wrapper.count(RID) == 1
    wrapper = wrapper.replace(RID,rid)
    ast.parse(wrapper)
    with (stage / 'run_recorded_once.py').open('x', encoding='utf-8') as f:
        f.write(wrapper)
    plan = read(SOURCE / 'placement_plan.json')
    plan.update(state='R40_FIXED_LENGTH_CORRECTION_ONLY_CANDIDATE_UNEXECUTED',request_id=rid,next_request_id=rid,
        candidate=str(stage/'candidate.settings.json'),config=str(stage/'config.json'),candidate_requested_parameters=candidate['parameters'],
        operator_ready=False,execution_pending=False,next_candidate_executed=False,current_placement_confirmed=False,
        current_clearance_confirmed=False,current_hands_clear_confirmed=False,current_physical_clearance_verified=False,
        both_powered_off_confirmed=True,power_state='Both_OFF_operator_confirmed_after_R40_C3P0_001',
        source_params_changed=False,source_review=str(OUT/'review.json'),last_request_id=RID,last_request_executed=True,
        last_saved_log_status='saved',last_contact_detected=False,last_successful_capture_id=RID,
        battery_charge_status='Last retained under-load minimum7.488V; re-read before execution',
        camera_health_report=str(OUT/'phone_health_before_on.json'),updated_at=datetime.now(timezone.utc).isoformat())
    save(stage/'placement_plan.json',plan)
    stages.append(dict(stage=str(stage),trial_id=rid))

save(OUT/'review.json',dict(schema_version=1,mouse='A',motion='out_r135',preset_mm_s=1800,trial_id=RID,
    operator=operator,source_review=str(REVIEW),tracking=tracking,
    measured_relative_exit_deg=review['exit_path_direction_change_deg'],
    measured_Lend_right_forward_mm=review['midpoint_Lend_right_forward_mm'],
    measured_Lend_residual_mm=review['midpoint_Lend_residual_mm'],
    relative_quality=review['independent_clock_required_motion_quality'],clock=review['clock'],
    entry_line=review['entry_line'],exit_line=review['exit_line'],late_stationary=review['late_stationary'],
    original_summary=review['original_summary'],original_model_rms_mm=review['original_trajectory_model_rms_mm'],
    standard_initial_line_guard_preserved=True,
    standard_summary_limitation='Existing summary rejected insufficient initial observations at its fast-entry window. Independent raw-wheel clock/PCA entry has27 points/110mm and unchanged strict quality bounds. Original summary remains unmodified.',
    prior_requested_parameters=prior['parameters'],next_requested_parameters=candidate['parameters'],
    changed_parameter_names=['correction'],nominal_angle_prediction_deg=review['exit_path_direction_change_deg']-2.,
    predictions_are_not_measurements=True,
    rationale='R40/c3.0で136.985度。半径変更後はジャイロ追従偏りも変化した。半径40とLs0.5/Le35を固定し、補正だけ+2度して5.0度を測定する。角度を確認してから長さを再計算する。',
    limitations=['半径41の過去補正secantは電圧・長さと交絡するため、R40の確定ゲインとして使わない。',
        '今回の-2度予測は名目角度差分。補正のみの次試行で実応答を確認する。',
        '入口方向に対する相対マーカー中点であり、絶対車軸・機軸yawや盤面固定接続の合格ではない。'],
    hardware_executions_this_trial=1,execution_resend_count=0,source_params_changed=False,
    parameter_adoption_allowed=False,out135_firmware_adopted=False,ekf_parameters_changed=False,
    ekf_status='out135 not eligible in current firmware; body-yaw/IMU-marker geometry unavailable, no K/noise adoption',stages=stages,
    input_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in
        (REVIEW,RUN/'trial.json',RUN/'saved_1ms.csv',RUN/'saved_1ms.settings.json',RUN/'telemetry.csv',operator_path,Path(__file__))}))
shutil.copyfile(BASE/'preparation.json',OUT/'before_c5p0_preparation.json')
plan=read(BASE/'r40_c5p0_Ls0p5_Le35_stream10_01/placement_plan.json')
(BASE/'preparation.json').write_text(json.dumps(plan,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(prepared=True,parameters=candidate['parameters'],changed_only=['correction'],
    measured_exit_angle_deg=review['exit_path_direction_change_deg'],nominal_next_angle_deg=review['exit_path_direction_change_deg']-2.,hardware_operations=0)))
