from pathlib import Path
from datetime import datetime,timezone
import hashlib
import json
import math
import os
import shutil
import subprocess

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_outr45_20261120_v1'
STAGE=BASE/'retry_lengths_cminus1p7_Ls17p0_Le17p4_20261121_02'
ID='A_OUTR45_1800_cminus1p7_Ls17p0_Le17p4_20261121_002'
RUN=STAGE/'local_execution'/('run_'+ID)
NEXT=BASE/'repeat_lengths_cminus1p7_Ls17p0_Le17p4_20261121_03'
NEXT_ID='A_OUTR45_1800_cminus1p7_Ls17p0_Le17p4_20261121_003'
def read(path):return json.loads(path.read_text(encoding='utf-8'))
def save(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('x',encoding='utf-8') as file:
        json.dump(value,file,ensure_ascii=False,indent=2,allow_nan=False);file.write('\n')
def sha(path):
    with path.open('rb') as file:return hashlib.file_digest(file,'sha256').hexdigest()

trial=read(RUN/'trial.json');confirmation=read(RUN/'operator_postrun_confirmation.json')
assert confirmation['both_powered_off'] and confirmation['contact_detected'] is False
assert trial['saved_log']['frame_count']==192 and trial['execution_resend_count']==0
review=read(STAGE/'midpoint_brake_clock_v1/midpoint_review.json')
tracking=read(STAGE/'tracking_1ms_review_v1/verification.json');summary=read(RUN/'full/summary.json')
diagnostic=read(STAGE/'post_lend_heading_diagnostic.json')
angle=review['exit_path_direction_change_deg'];error=math.hypot(*review['midpoint_Lend_residual_mm'])
assert review['independent_optical_geometry_eligible'] and abs(angle-45)>.5 and error>2
rows=[];fixed=[]
for folder in (BASE/'lengths_cminus1p7_Ls15p4_Le15p9_20261121_01',BASE/'repeat_cminus1p7_Ls15p4_Le15p9_20261121_02',STAGE):
    optical=read(folder/'midpoint_brake_clock_v1/midpoint_review.json');response=read(folder/'tracking_1ms_review_v1/verification.json')
    actual_path=Path(response['source_settings']);actual=read(actual_path)
    context={key:actual[key] for key in ('mouse','motion_name','preset_speed_mm_s','turn_count','pid','feedforward','suction')}
    context['parameters']={key:value for key,value in actual['parameters'].items() if key not in ('lstart','lend')}
    fixed.append(context)
    rows.append(dict(stage=folder.name,parameters=actual['parameters'],angle_deg=optical['exit_path_direction_change_deg'],
        residual_mm=optical['midpoint_Lend_residual_mm'],position_error_mm=math.hypot(*optical['midpoint_Lend_residual_mm']),
        gyro_through_Lend_deg=response['gyro_through_lend_measured_time_deg'],pulse_speed_mm_s=response['pulse_speed_measured_time_mean_mm_s'],
        battery_min_v=response['battery_min_v'],source_settings=str(actual_path),source_settings_sha256=sha(actual_path)))
assert all(value==fixed[0] for value in fixed)
save(STAGE/'length_comparison.json',dict(rows=rows,readbacks_equal_except_lengths=True,new_lengths_measured_runs=1,
    optical_measurement_eligible=True,unexplained_path_body_heading_difference_deg=diagnostic['optical_path_minus_mixed_period_body_change_deg'],
    angle_only_unit_response_prediction_deg=-1.7+angle-45,angle_prediction_selected=False,
    length_only_projection=review['length_only_candidate_at_current_angle'],length_projection_selected=False,
    next_step='Independent repeat of same17.0/17.4mm before any further numerical retune.',parameter_adoption_allowed=False))
save(STAGE/'firmware_readiness_review.json',dict(mouse='A',motion='out_r45',speed_mm_s=1800,
    parameters=read(RUN/'saved_1ms.settings.json')['parameters'],measured_angle_deg=angle,
    measured_Lend_residual_mm=review['midpoint_Lend_residual_mm'],measured_Lend_residual_norm_mm=error,
    review_angle_tolerance_deg=.5,review_position_tolerance_mm=2.,angle_target_passed=False,position_target_passed=False,
    firmware_candidate_qualified=False,optical_measurement_quality_eligible=True,
    native_auto_stop_confirmed=summary['auto_stop_confirmed'],native_model_rmse_mm=summary['trajectory_fit_rmse_mm'],
    observed_body_path_difference_unexplained=True,new_lengths_measured_runs=1,position_repeatability_confirmed=False,
    source_parameters_changed=False,flash_written=False,operator_confirmation=confirmation,
    evaluation_frame='Measured entry aligned relative marker midpoint; not calibrated board-fixed axle/body yaw.',
    left_verified=False,continuous_run_verified=False))
ekf=read(BASE/'repeat_cminus1p7_Ls15p4_Le15p9_20261121_02/ekf_identification_status.json')
ekf.update(source_trial=str(RUN/'trial.json'),source_eligibility_sha256=sha(ROOT/'Task/Inc/turn_pose_ekf.h'))
save(STAGE/'ekf_identification_status.json',ekf)

NEXT.mkdir(exist_ok=False)
settings=read(STAGE/'candidate.settings.json')
settings['source']='Unexecuted independent repeat after current17.0/17.4mm video46.713deg, position2.968mm. Hold correction-1.7deg; path/body heading diagnostic difference1.55deg remains unexplained.'
save(NEXT/'candidate.settings.json',settings)
save(NEXT/'candidate_basis.json',dict(type='independent_repeat_same_settings',source_trial=str(RUN/'trial.json'),
    source_review=str(STAGE/'midpoint_brake_clock_v1/midpoint_review.json'),
    source_diagnostic=str(STAGE/'post_lend_heading_diagnostic.json'),settings_changed=False,executed=False,
    parameter_adoption_allowed=False,B_capture_stopped_by_explicit_user_instruction=True,
    automatic_execution_replay_allowed=False,requires_fresh_operator_placement_power_and_hands_clear=True))
for name in ('config.json','static_pose_after_settle.py'):shutil.copy2(STAGE/name,NEXT/name)
wrapper=(STAGE/'run_recorded_once.py').read_text(encoding='utf-8');assert wrapper.count(ID)==1
with (NEXT/'run_recorded_once.py').open('x',encoding='utf-8') as file:file.write(wrapper.replace(ID,NEXT_ID))
validation=subprocess.run([str(ROOT/'../../Debug_camera/.venv/Scripts/python.exe'),str(NEXT/'run_recorded_once.py')],
    cwd=ROOT,env=dict(os.environ,PYTHONUTF8='1'),capture_output=True,text=True,encoding='utf-8')
save(NEXT/'input_validation.json',dict(returncode=validation.returncode,stdout=validation.stdout,stderr=validation.stderr,robot_commands_sent=False))
assert validation.returncode==0,validation.stderr

fig,axes=plt.subplots(1,2,figsize=(10,4),layout='constrained')
labels=['15.4/15.9 run1','15.4/15.9 run2','17.0/17.4 run1']
for ax,key,title in zip(axes,('angle_deg','position_error_mm'),('Video exit angle [deg]','Relative Lend position error [mm]')):
    values=[row[key] for row in rows];ax.bar(labels,values,color=['#8895a7','#4282bb','#c77c34'])
    for i,value in enumerate(values):ax.text(i,value+.035,f'{value:.2f}',ha='center')
    ax.set_ylabel(title);ax.tick_params(axis='x',labelsize=8);ax.grid(axis='y',alpha=.2)
axes[0].set_ylim(43,48);axes[0].axhspan(44.5,45.5,color='green',alpha=.12);axes[0].axhline(45,color='green',ls='--',lw=1)
axes[1].set_ylim(0,4.5);axes[1].axhline(2,color='green',ls='--',lw=1)
fig.suptitle('Mouse A OUTR45 1800: fixed correction-1.7deg / no adoption')
fig.savefig(STAGE/'length_comparison.png',dpi=160);plt.close(fig)

entry=f'''## 2026-11-21の17.0 / 17.4 mm・新要求002の結果

B側停止・A続行の明示指示と、Bの完了・切断・走行処理終了を確認した。新しい配置・両方ON・手離し確認後、補正-1.7度・17.0 / 17.4 mmを新要求002で1回実行した。実走は通算12本、完全な保存1 msは11本。先のカメラBUSY要求001は走行0回のまま保持し、再送していない。今回も機体A・idle・実機値を照合し、長さ以外の設定は直前2本と一致した。

保存1 msログ192点・drop0、連続CSVの走行区間は欠落0・CRC0。操作者は停止・接触なし・両方OFFを確認し、BLE切断済み。[相対中点評価]({STAGE.name}/midpoint_brake_clock_v1/midpoint_review.json)は出口角46.713度、Lend残差は右+2.965 / 前-0.136 mm、距離{error:.3f} mmで、角度45±0.5度・位置2 mm以内の目安を満たさず未採用。[比較図]({STAGE.name}/length_comparison.png)と[実機設定比較]({STAGE.name}/length_comparison.json)を参照。入口に合わせた相対評価で、盤面固定・絶対車軸の合格ではない。

Lendまでの1 msジャイロ積分45.668度、車輪パルス平均速度1848 mm/s、最低電圧7.344 V。[Lend後の診断]({STAGE.name}/post_lend_heading_diagnostic.json)では、5 msログのLend後から直進指令終了まで-0.494度、入口・出口窓の混合周期body heading差45.164度に対して動画経路角46.713度で約1.549度の差が残る。追加の回頭で説明できたとは扱わず、未校正のbody yawと経路接線・滑り/幾何を区別する。5 ms以降を精密1 msデータとして再構成していない。

純走行・入口・出口・Lend前後の実観測/登録100%。入口28点・出口31点、入口時計RMSE0.383 mm・半窓時刻差0.706 ms、ブレーキ終了後静止ばらつき0.099 mmで独立観測品質を通過した。元nativeモデルRMSE20.018 mmとauto_stop_confirmed:falseは保持する。

今回1本から補正0.0度や長さ19.928 / 13.327 mmを導く診断は次値に採らず、[同じ補正-1.7度・17.0 / 17.4 mmの新要求003]({NEXT.name}/candidate.settings.json)を準備し、再現性を先に確認する。入力検証済み・未実行で、新しい配置・両方ON・手離し確認が必要。B側録画走行を止めてAを優先する指示を引き継ぐ。現在は両方OFFで待機する。

[状態v14](session_outr45_status_20261121_v14.json)を参照。ファームソース・Flash・EKF係数は変更していない。

'''
path=BASE/'README.md';old=path.read_text(encoding='utf-8');heading,tail=old.split('\n',1)
path.write_text(heading+'\n\n'+entry+tail.lstrip('\n'),encoding='utf-8')
save(BASE/'session_outr45_status_20261121_v14.json',dict(recorded_at_utc=datetime.now(timezone.utc).isoformat(),
    state='TWELVE_RUNS_REVIEWED_LENGTHS17_TARGETS_FAILED_SAME_SETTING_REPEAT_READY_OFF',
    mouse='A',motion='out_r45',speed_mm_s=1800,completed_physical_runs=12,precise_1ms_complete_runs=11,
    latest_trial=str(RUN/'trial.json'),latest_recording_id=trial['recording_id'],latest_video_exit_deg=angle,
    latest_Lend_residual_mm=review['midpoint_Lend_residual_mm'],latest_Lend_residual_norm_mm=error,
    latest_gyro_through_Lend_deg=tracking['gyro_through_lend_measured_time_deg'],latest_1ms_count=192,
    latest_battery_min_v=tracking['battery_min_v'],latest_pulse_speed_mm_s=tracking['pulse_speed_measured_time_mean_mm_s'],
    body_path_diagnostic_difference_deg=diagnostic['optical_path_minus_mixed_period_body_change_deg'],
    angle_target_passed=False,position_target_passed=False,corrected_optical_geometry_eligible=True,
    native_auto_stop_confirmed=False,native_model_rmse_mm=summary['trajectory_fit_rmse_mm'],
    both_powered_off=True,contact_detected=False,ble_disconnected=True,execution_resend_count=0,
    B_capture_stopped_by_user=True,camera_BUSY_failed_request_preserved=True,
    next_trial_id=NEXT_ID,next_settings=str(NEXT/'candidate.settings.json'),next_type='independent_repeat_same_settings',
    next_correction_deg=-1.7,next_lstart_mm=17.0,next_lend_mm=17.4,next_executed=False,next_operator_ready=False,
    parameter_adoption_allowed=False,firmware_source_changed=False,firmware_written=False,new_ekf_parameters_adopted=False))
print(json.dumps(dict(angle_deg=angle,position_error_mm=error,next_type='independent_repeat_same_settings',next_validation=validation.returncode,next_executed=False)),flush=True)
