from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json
import math
import os
import shutil
import subprocess
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT/'tools/logs/pixel_camera/turn1800_A_outr45_20261120_v1'
STAGE = BASE/'fixed_lengths_cplus0p1_Ls19p5_Le18p8_20261121_01'
ID = 'A_OUTR45_1800_cplus0p1_Ls19p5_Le18p8_20261121_001'
RUN = STAGE/'local_execution'/('run_'+ID)
NEXT = BASE/'joint_cplus0p8_Ls19p9_Le22p1_20261121_01'
NEXT_ID = 'A_OUTR45_1800_cplus0p8_Ls19p9_Le22p1_20261121_001'

def read(path):
    return json.loads(path.read_text(encoding='utf-8'))

def save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x', encoding='utf-8') as file:
        json.dump(value, file, ensure_ascii=False, indent=2, allow_nan=False)
        file.write('\n')

def sha(path):
    with path.open('rb') as file:
        return hashlib.file_digest(file, 'sha256').hexdigest()

review = read(STAGE/'midpoint_brake_clock_v1/midpoint_review.json')
tracking = read(STAGE/'tracking_1ms_review_v1/verification.json')
summary = read(RUN/'full/summary.json')
trial = read(RUN/'trial.json')
confirmation = read(RUN/'operator_postrun_confirmation.json')
assert confirmation['both_powered_off'] and confirmation['contact_detected'] is False
assert review['independent_optical_geometry_eligible'] is True
assert trial['execution_resend_count'] == 0 and trial['saved_log']['frame_count'] == 196
error = math.hypot(*review['midpoint_Lend_residual_mm'])
angle = review['exit_path_direction_change_deg']
qualified = abs(angle-45) <= .5 and error <= 2
assert not qualified

NEXT.mkdir(exist_ok=False)
settings = read(STAGE/'candidate.settings.json')
settings['source'] = 'Unexecuted next comparison from measured45.654deg and Lend residual(-2.392,-2.703)mm. Local translation/unit angle response are unverified.'
settings['parameters'].update(correction=.8, lstart=19.9, lend=22.1)
save(NEXT/'candidate.settings.json', settings)
for name in ('config.json', 'static_pose_after_settle.py'):
    shutil.copy2(STAGE/name, NEXT/name)
wrapper = (STAGE/'run_recorded_once.py').read_text(encoding='utf-8')
assert wrapper.count(ID) == 1
with (NEXT/'run_recorded_once.py').open('x', encoding='utf-8') as file:
    file.write(wrapper.replace(ID, NEXT_ID))
basis = dict(source_review=str(STAGE/'midpoint_brake_clock_v1/midpoint_review.json'),
    old_correction_deg=.1, measured_exit_angle_deg=angle, target_angle_deg=45.,
    unit_response_angle_proposal_deg=.1+angle-45,
    chosen_correction_deg=.8, measured_length_only_solution=review['length_only_candidate_at_current_angle'],
    chosen_lstart_mm=19.9, chosen_lend_mm=22.1, executed=False, parameter_adoption_allowed=False,
    assumptions=['Length shifts project onto the measured entry/exit directions at the old angle.',
        'Unit angle response and simultaneous angle/length changes remain unverified.',
        'Pulse speed changed from1766.9 to1846.3mm/s and voltage from7.484 to7.762V; do not claim correction-only causality.'],
    requires_fresh_operator_placement_power_and_hands_clear=True)
save(NEXT/'candidate_basis.json', basis)
env = dict(os.environ, PYTHONUTF8='1')
validation = subprocess.run([str(ROOT/'../../Debug_camera/.venv/Scripts/python.exe'), str(NEXT/'run_recorded_once.py')],
    cwd=ROOT, env=env, capture_output=True, text=True, encoding='utf-8')
save(NEXT/'input_validation.json', dict(returncode=validation.returncode,
    stdout=validation.stdout, stderr=validation.stderr, robot_commands_sent=False))
assert validation.returncode == 0, validation.stderr

save(STAGE/'firmware_readiness_review.json', dict(mouse='A', motion='out_r45', speed_mm_s=1800,
    parameters=read(STAGE/'candidate.settings.json')['parameters'],
    measured_angle_deg=angle, measured_Lend_residual_mm=review['midpoint_Lend_residual_mm'],
    measured_Lend_residual_norm_mm=error, review_angle_tolerance_deg=.5, review_position_tolerance_mm=2.,
    target_achieved=False, firmware_candidate_qualified=False, source_parameters_changed=False, flash_written=False,
    optical_measurement_quality_eligible=True, native_auto_stop_confirmed=summary['auto_stop_confirmed'],
    native_trajectory_model_rmse_mm=summary['trajectory_fit_rmse_mm'],
    evaluation_frame='height-corrected relative marker midpoint aligned to measured entry; not board-fixed axle accuracy',
    physical_run_count=1, repeatability_confirmed=False, left_verified=False, continuous_run_verified=False,
    operator_confirmation=confirmation, next_unexecuted_candidate=str(NEXT/'candidate.settings.json')))
save(STAGE/'ekf_identification_status.json', dict(mouse='A', motion='out_r45', speed_mm_s=1800,
    suction_duty=650, runtime_mode='control', status='INSUFFICIENT_INDEPENDENT_BODY_YAW_GEOMETRY',
    new_coefficients_identified=False, firmware_ekf_changed=False,
    reasons=['Relative marker midpoint is valid for path geometry, but calibrated absolute body yaw and axle/IMU geometry remain unverified.',
        'No independent validation trial for new slip/noise parameters was reserved. Existing EKF mode and coefficients stayed fixed.'],
    source_trial=str(RUN/'trial.json')))

previous = read(BASE/'joint_cminus2p0_Ls19p5_Le18p8_20261121_01/midpoint_brake_clock_v1/midpoint_review.json')
fig, axes = plt.subplots(1, 2, figsize=(9, 3.9), layout='constrained')
labels = ['c -2.0 / 19.5,18.8', 'c +0.1 / 19.5,18.8']
angles = [previous['exit_path_direction_change_deg'], angle]
errors = [math.hypot(*previous['midpoint_Lend_residual_mm']), error]
for ax, values, ylabel in zip(axes, (angles, errors), ('Video exit angle [deg]', 'Relative Lend position error [mm]')):
    ax.bar(labels, values, color=['#8895a7', '#4282bb'])
    for x, value in enumerate(values): ax.text(x, value+.04, f'{value:.2f}', ha='center')
    ax.set_ylabel(ylabel)
    ax.tick_params(axis='x', labelsize=8)
    ax.grid(axis='y', alpha=.2)
axes[0].axhspan(44.5,45.5,color='green',alpha=.12)
axes[0].axhline(45,color='green',ls='--',lw=1)
axes[0].set_ylim(42,49)
axes[1].axhline(2,color='green',ls='--',lw=1)
axes[1].set_ylim(0,4.4)
fig.suptitle('Mouse A OUTR45 1800: one run per setting / no adoption')
fig.savefig(STAGE/'fixed_length_comparison.png',dpi=160)
plt.close(fig)

text = f'''## 2026-11-21の補正+0.1度・19.5 / 18.8 mmの結果

6本目は機体Aを実機識別し、補正+0.1度・Lstart19.5 / Lend18.8 mmで1回走行した。動画と保存1 msログ196点・drop0を取得。操作者は停止・接触なし・両方OFFで待機を確認し、BLE切断済み。走行再送は0回。

[中点・入口時計の評価]({STAGE.name}/midpoint_brake_clock_v1/midpoint_review.json)では出口角{angle:.3f}度、Lend残差は右-2.392 / 前-2.703 mm、距離{error:.3f} mm。前回の47.117度から角度は改善したが、位置残差は1.740から{error:.3f} mmへ増えた。[比較図]({STAGE.name}/fixed_length_comparison.png)を参照。目安の角度45±0.5度・位置2 mm以内を同時に満たさず、右45度出口はファーム反映を保留する。これは入口に合わせた相対マーカー中点の単体評価で、盤面固定精度や絶対車軸精度ではない。

入口時計RMSE0.398 mm・半窓時刻差0.424 ms、純走行・入口・出口・Lend前後は実観測/登録100%。ブレーキ終了後の静止ばらつき0.156 mm。独立時計の幾何観測条件を通過した。元native解析はモデルRMSE11.284 mmによりauto_stop_confirmed:falseで、判定を上書きしていない。

[1 ms追従]({STAGE.name}/tracking_1ms_review_v1/verification.json)はLendまでのジャイロ積分45.819度、車輪パルス平均速度1846 mm/s、最低電圧7.762 V。前回1767 mm/s・7.484 Vとの違いがあり、角度補正だけの因果差や再現性を断定しない。新EKF係数は未同定で、モードcontrolと既存係数は固定した。

次の[未実行候補]({NEXT.name}/candidate.settings.json)は補正+0.8度・Lstart19.9 / Lend22.1 mm。現在の残差を入口・出口方向へ投影した長さ解19.865 / 22.144 mmと、角度差0.654度の単位応答を仮定した比較用で、達成を保証しない。[計算根拠]({NEXT.name}/candidate_basis.json)と入力検証を保存した。新しい配置・両方ON・手離し確認なしに実行しない。現在の両方OFF待機を維持する。

[状態v8](session_outr45_status_20261121_v8.json)を参照。右45度出口のソース、Flash、EKFは変更していない。入口45度の2.0 / 40.0 mmは前タスクで左右ソース反映済みだが、出口設定とは別である。

'''
readme = BASE/'README.md'
old = readme.read_text(encoding='utf-8')
heading, tail = old.split('\n',1)
readme.write_text(heading+'\n\n'+text+tail.lstrip('\n'),encoding='utf-8')
status = dict(recorded_at_utc=datetime.now(timezone.utc).isoformat(),
    state='SIX_RUNS_REVIEWED_CPLUS0P1_NOT_QUALIFIED_NEXT_PREPARED_OFF', mouse='A', motion='out_r45',speed_mm_s=1800,
    completed_physical_runs=6, precise_1ms_complete_runs=5, latest_trial=str(RUN/'trial.json'),
    latest_recording_id=trial['recording_id'], latest_video_exit_deg=angle,
    latest_Lend_residual_mm=review['midpoint_Lend_residual_mm'],latest_Lend_residual_norm_mm=error,
    latest_gyro_through_Lend_deg=tracking['gyro_through_lend_measured_time_deg'], latest_1ms_count=196,
    latest_battery_min_v=tracking['battery_min_v'], latest_pulse_speed_mm_s=tracking['pulse_speed_measured_time_mean_mm_s'],
    corrected_optical_geometry_eligible=True, native_auto_stop_confirmed=False,
    native_model_rmse_mm=summary['trajectory_fit_rmse_mm'], both_powered_off=True,contact_detected=False,
    ble_disconnected=True,execution_resend_count=0,next_trial_id=NEXT_ID,
    next_settings=str(NEXT/'candidate.settings.json'),next_correction_deg=.8,next_lstart_mm=19.9,next_lend_mm=22.1,
    next_executed=False,next_operator_ready=False,parameter_adoption_allowed=False,
    firmware_source_changed=False,firmware_written=False,new_ekf_parameters_adopted=False,
    limitations=['Angle45.654deg and position3.609mm miss simultaneous45±0.5deg/2mm review targets.',
        'Relative marker midpoint; calibrated absolute body yaw, axle geometry, left and continuous-run validation unavailable.',
        'Native model RMSE11.284mm exceeds8mm gate; acceptance remains false.'])
save(BASE/'session_outr45_status_20261121_v8.json',status)
print(json.dumps(dict(angle_deg=angle,Lend_residual_norm_mm=error,next_input_validation=validation.returncode,
    next_executed=False,parameter_adoption_allowed=False),ensure_ascii=False),flush=True)
