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
STAGE=BASE/'lengths_cminus1p7_Ls15p4_Le15p9_20261121_01'
PREVIOUS=BASE/'fixed_lengths_cminus1p7_Ls19p9_Le22p1_20261121_01'
ID='A_OUTR45_1800_cminus1p7_Ls15p4_Le15p9_20261121_001'
RUN=STAGE/'local_execution'/('run_'+ID)
NEXT=BASE/'repeat_cminus1p7_Ls15p4_Le15p9_20261121_02'
NEXT_ID='A_OUTR45_1800_cminus1p7_Ls15p4_Le15p9_20261121_002'
def read(path):return json.loads(path.read_text(encoding='utf-8'))
def save(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('x',encoding='utf-8') as file:
        json.dump(value,file,ensure_ascii=False,indent=2,allow_nan=False);file.write('\n')
def sha(path):
    with path.open('rb') as file:return hashlib.file_digest(file,'sha256').hexdigest()

trial=read(RUN/'trial.json');confirmation=read(RUN/'operator_postrun_confirmation.json')
assert confirmation['both_powered_off'] and confirmation['contact_detected'] is False
assert trial['saved_log']['frame_count']==191 and trial['execution_resend_count']==0
review=read(STAGE/'midpoint_brake_clock_v1/midpoint_review.json')
summary=read(RUN/'full/summary.json');tracking=read(STAGE/'tracking_1ms_review_v1/verification.json')
angle=review['exit_path_direction_change_deg'];error=math.hypot(*review['midpoint_Lend_residual_mm'])
assert review['independent_optical_geometry_eligible'] and abs(angle-45)<=.5 and error>2
rows=[];fixed=[]
for folder in (PREVIOUS,STAGE):
    optical=read(folder/'midpoint_brake_clock_v1/midpoint_review.json')
    response=read(folder/'tracking_1ms_review_v1/verification.json');path=Path(response['source_settings']);actual=read(path)
    context={key:actual[key] for key in ('mouse','motion_name','preset_speed_mm_s','turn_count','pid','feedforward','suction')}
    context['parameters']={key:value for key,value in actual['parameters'].items() if key not in ('lstart','lend')}
    fixed.append(context)
    rows.append(dict(stage=folder.name,parameters=actual['parameters'],angle_deg=optical['exit_path_direction_change_deg'],
        Lend_residual_mm=optical['midpoint_Lend_residual_mm'],position_error_mm=math.hypot(*optical['midpoint_Lend_residual_mm']),
        gyro_through_Lend_deg=response['gyro_through_lend_measured_time_deg'],pulse_speed_mm_s=response['pulse_speed_measured_time_mean_mm_s'],
        battery_min_v=response['battery_min_v'],source_settings=str(path),source_settings_sha256=sha(path)))
assert fixed[0]==fixed[1]
save(STAGE/'length_only_comparison.json',dict(rows=rows,readbacks_equal_except_lstart_lend=True,
    new_lengths_measured_runs=1,new_length_only_projection=review['length_only_candidate_at_current_angle'],
    new_projection_is_next_candidate=False,position_repeatability_confirmed=False,
    reason='Error improved from9.916 to3.692mm but still exceeds2mm. Prior same-setting forward position varied5.11mm; repeat new lengths before another adjustment.'))

NEXT.mkdir(exist_ok=False)
settings=read(STAGE/'candidate.settings.json')
settings['source']='Unexecuted independent repeat of correction-1.7deg Lstart15.4/Lend15.9mm after one video45.430deg, position3.692mm. No parameter changes or automatic replay.'
save(NEXT/'candidate.settings.json',settings)
for name in ('config.json','static_pose_after_settle.py'):shutil.copy2(STAGE/name,NEXT/name)
wrapper=(STAGE/'run_recorded_once.py').read_text(encoding='utf-8');assert wrapper.count(ID)==1
with (NEXT/'run_recorded_once.py').open('x',encoding='utf-8') as file:file.write(wrapper.replace(ID,NEXT_ID))
save(NEXT/'candidate_basis.json',dict(source_optical_review=str(STAGE/'midpoint_brake_clock_v1/midpoint_review.json'),
    type='independent_repeat_same_settings',settings_changed=False,measured_angle_deg=angle,
    measured_Lend_residual_mm=review['midpoint_Lend_residual_mm'],executed=False,parameter_adoption_allowed=False,
    automatic_execution_replay_allowed=False,requires_fresh_operator_placement_power_and_hands_clear=True,
    reason='Verify position repeatability before using the one-run18.010/17.257mm projection; earlier identical settings showed5.11mm forward variation.'))
validation=subprocess.run([str(ROOT/'../../Debug_camera/.venv/Scripts/python.exe'),str(NEXT/'run_recorded_once.py')],
    cwd=ROOT,env=dict(os.environ,PYTHONUTF8='1'),capture_output=True,text=True,encoding='utf-8')
save(NEXT/'input_validation.json',dict(returncode=validation.returncode,stdout=validation.stdout,stderr=validation.stderr,
    robot_commands_sent=False));assert validation.returncode==0,validation.stderr
save(STAGE/'firmware_readiness_review.json',dict(mouse='A',motion='out_r45',speed_mm_s=1800,
    parameters=read(RUN/'saved_1ms.settings.json')['parameters'],measured_angle_deg=angle,
    measured_Lend_residual_mm=review['midpoint_Lend_residual_mm'],measured_Lend_residual_norm_mm=error,
    review_angle_tolerance_deg=.5,review_position_tolerance_mm=2.,angle_target_passed=True,position_target_passed=False,
    firmware_candidate_qualified=False,optical_measurement_quality_eligible=True,
    native_auto_stop_confirmed=summary['auto_stop_confirmed'],native_trajectory_model_rmse_mm=summary['trajectory_fit_rmse_mm'],
    new_lengths_measured_runs=1,position_repeatability_confirmed=False,source_parameters_changed=False,flash_written=False,
    left_verified=False,continuous_run_verified=False,operator_confirmation=confirmation,
    evaluation_frame='Measured entry aligned relative marker midpoint, not calibrated board-fixed axle/body yaw.',
    next_unexecuted_repeat=str(NEXT/'candidate.settings.json')))
ekf=read(PREVIOUS/'ekf_identification_status.json');ekf.update(source_trial=str(RUN/'trial.json'),
    source_eligibility_sha256=sha(ROOT/'Task/Inc/turn_pose_ekf.h'))
save(STAGE/'ekf_identification_status.json',ekf)

fig,axes=plt.subplots(1,2,figsize=(9,4),layout='constrained')
for ax,key,title in zip(axes,('angle_deg','position_error_mm'),('Video exit angle [deg]','Relative Lend position error [mm]')):
    values=[row[key] for row in rows];ax.bar(['19.9/22.1mm','15.4/15.9mm'],values,color=['#8895a7','#4282bb'])
    for x,value in enumerate(values):ax.text(x,value+.04,f'{value:.2f}',ha='center')
    ax.set_ylabel(title);ax.grid(axis='y',alpha=.2)
axes[0].set_ylim(43,47);axes[0].axhspan(44.5,45.5,color='green',alpha=.12);axes[0].axhline(45,color='green',ls='--',lw=1)
axes[1].set_ylim(0,11.2);axes[1].axhline(2,color='green',ls='--',lw=1)
fig.suptitle('Mouse A OUTR45 1800: correction-1.7deg, one run each, no adoption')
fig.savefig(STAGE/'length_only_comparison.png',dpi=160);plt.close(fig)

entry=f'''## 2026-11-21の補正-1.7度・15.4 / 15.9 mmの結果

10本目は補正-1.7度を維持してLstart15.4 / Lend15.9 mmへ変更し、機体A・idleと実機値を照合して1回実行した。読み戻しは長さ以外の全パラメータ・PID/FF・吸引・速度・回数が前回と一致。保存1 msログ191点・drop0、連続CSVの走行区間は欠落0・CRC0。操作者は停止・接触なし・両方OFFを確認し、BLE切断済み。走行再送は0回。

[相対中点の評価]({STAGE.name}/midpoint_brake_clock_v1/midpoint_review.json)で出口角45.430度、Lend残差は右-0.967 / 前-3.563 mm、距離{error:.3f} mm。角度45±0.5度の目安は満たし、位置ずれは前回9.916 mmから縮まったが、位置2 mm以内には届かずファーム反映は保留する。[比較図]({STAGE.name}/length_only_comparison.png)と[実機設定比較]({STAGE.name}/length_only_comparison.json)を参照。入口に合わせた相対マーカー中点の評価で、盤面固定・絶対車軸の合格ではない。

Lendまでのジャイロ積分45.045度で前回45.081度に近い。車輪パルス平均速度1835 mm/s、最低電圧7.516 V。純走行・入口・出口・Lend前後の実観測/登録100%。入口30点・出口31点、入口時計RMSE0.395 mm・半窓時刻差0.164 ms、ブレーキ終了後静止ばらつき0.054 mmで独立観測品質を通過。元nativeモデルRMSE15.938 mmとauto_stop_confirmed:falseは変更していない。

今回1本からの長さ投影は18.010 / 17.257 mmとなるが、先に別の同設定2本で前方向5.112 mmの変動があったため、これを次の調整値には採らない。次は[同じ補正-1.7度・15.4 / 15.9 mmの新しい試行]({NEXT.name}/candidate.settings.json)を1回取得し、位置の再現性を先に確認する。入力検証済みで未実行。次の実行には新しい配置・両方ON・手離し確認が必要。現在は両方OFFで待機する。

[状態v12](session_outr45_status_20261121_v12.json)を参照。右45度出口のファームソース・Flash・EKF係数は未変更。

'''
path=BASE/'README.md';old=path.read_text(encoding='utf-8');heading,tail=old.split('\n',1)
path.write_text(heading+'\n\n'+entry+tail.lstrip('\n'),encoding='utf-8')
save(BASE/'session_outr45_status_20261121_v12.json',dict(recorded_at_utc=datetime.now(timezone.utc).isoformat(),
    state='TEN_RUNS_REVIEWED_LENGTHS15_ANGLE_PASSED_POSITION_IMPROVED_REPEAT_READY_OFF',
    mouse='A',motion='out_r45',speed_mm_s=1800,completed_physical_runs=10,precise_1ms_complete_runs=9,
    latest_trial=str(RUN/'trial.json'),latest_recording_id=trial['recording_id'],latest_video_exit_deg=angle,
    latest_Lend_residual_mm=review['midpoint_Lend_residual_mm'],latest_Lend_residual_norm_mm=error,
    latest_gyro_through_Lend_deg=tracking['gyro_through_lend_measured_time_deg'],latest_1ms_count=191,
    latest_battery_min_v=tracking['battery_min_v'],latest_pulse_speed_mm_s=tracking['pulse_speed_measured_time_mean_mm_s'],
    angle_target_passed=True,position_target_passed=False,new_lengths_measured_runs=1,position_repeatability_confirmed=False,
    corrected_optical_geometry_eligible=True,native_auto_stop_confirmed=False,native_model_rmse_mm=summary['trajectory_fit_rmse_mm'],
    both_powered_off=True,contact_detected=False,ble_disconnected=True,execution_resend_count=0,
    next_trial_id=NEXT_ID,next_settings=str(NEXT/'candidate.settings.json'),next_type='independent_repeat_same_settings',
    next_correction_deg=-1.7,next_lstart_mm=15.4,next_lend_mm=15.9,next_executed=False,next_operator_ready=False,
    parameter_adoption_allowed=False,firmware_source_changed=False,firmware_written=False,new_ekf_parameters_adopted=False))
print(json.dumps(dict(angle_deg=angle,position_error_mm=error,next_type='independent_repeat_same_settings',
    next_validation=validation.returncode,next_executed=False)),flush=True)
