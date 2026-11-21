from pathlib import Path
from datetime import datetime, timezone
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
STAGE=BASE/'joint_cplus0p8_Ls19p9_Le22p1_20261121_01'
ID='A_OUTR45_1800_cplus0p8_Ls19p9_Le22p1_20261121_001'
RUN=STAGE/'local_execution'/('run_'+ID)
NEXT=BASE/'repeat_cplus0p8_Ls19p9_Le22p1_20261121_02'
NEXT_ID='A_OUTR45_1800_cplus0p8_Ls19p9_Le22p1_20261121_002'

def read(path): return json.loads(path.read_text(encoding='utf-8'))
def save(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('x',encoding='utf-8') as file:
        json.dump(value,file,ensure_ascii=False,indent=2,allow_nan=False);file.write('\n')
def sha(path):
    with path.open('rb') as file:return hashlib.file_digest(file,'sha256').hexdigest()

review=read(STAGE/'midpoint_brake_clock_v1/midpoint_review.json')
tracking=read(STAGE/'tracking_1ms_review_v1/verification.json')
trial=read(RUN/'trial.json')
summary=read(RUN/'full/summary.json')
confirmation=read(RUN/'operator_postrun_confirmation.json')
assert confirmation['both_powered_off'] and confirmation['contact_detected'] is False
assert review['independent_optical_geometry_eligible'] and trial['saved_log']['frame_count']==199
assert trial['execution_resend_count']==0
angle=review['exit_path_direction_change_deg']
error=math.hypot(*review['midpoint_Lend_residual_mm'])
qualified=abs(angle-45)<=.5 and error<=2
assert not qualified

NEXT.mkdir(exist_ok=False)
settings=read(STAGE/'candidate.settings.json')
settings['source']='Unexecuted independent repeat of c+0.8deg Lstart19.9/Lend22.1mm. Check repeatability after video42.419deg and gyro42.016deg; no numerical retune or automatic replay.'
save(NEXT/'candidate.settings.json',settings)
for name in ('config.json','static_pose_after_settle.py'):shutil.copy2(STAGE/name,NEXT/name)
wrapper=(STAGE/'run_recorded_once.py').read_text(encoding='utf-8')
assert wrapper.count(ID)==1
with (NEXT/'run_recorded_once.py').open('x',encoding='utf-8') as file:file.write(wrapper.replace(ID,NEXT_ID))
save(NEXT/'candidate_basis.json',dict(source_trial=str(RUN/'trial.json'),
    source_optical_review=str(STAGE/'midpoint_brake_clock_v1/midpoint_review.json'),
    type='independent_repeat_same_settings',settings_changed=False,executed=False,
    automatic_execution_replay_allowed=False,parameter_adoption_allowed=False,
    reason='Current42.419deg fails target; angle response across distinct settings is too different to justify another single-run linear extrapolation.',
    requires_fresh_operator_placement_power_and_hands_clear=True))
validation=subprocess.run([str(ROOT/'../../Debug_camera/.venv/Scripts/python.exe'),str(NEXT/'run_recorded_once.py')],
    cwd=ROOT,env=dict(os.environ,PYTHONUTF8='1'),capture_output=True,text=True,encoding='utf-8')
save(NEXT/'input_validation.json',dict(returncode=validation.returncode,stdout=validation.stdout,
    stderr=validation.stderr,robot_commands_sent=False))
assert validation.returncode==0,validation.stderr

stages=['joint_cminus2p0_Ls19p5_Le18p8_20261121_01',
    'fixed_lengths_cplus0p1_Ls19p5_Le18p8_20261121_01',STAGE.name]
rows=[]
for stage in stages:
    folder=BASE/stage
    optical=read(folder/'midpoint_brake_clock_v1/midpoint_review.json')
    response=read(folder/'tracking_1ms_review_v1/verification.json')
    actual=read(Path(response['source_settings']))
    fixed=dict(pid=actual['pid'],feedforward=actual['feedforward'],suction=actual['suction'],
        velo=actual['parameters']['velo'],r_min=actual['parameters']['r_min'],
        pre_accel=actual['parameters']['pre_accel'],degree=actual['parameters']['degree'])
    rows.append(dict(stage=stage,parameters=actual['parameters'],exit_angle_deg=optical['exit_path_direction_change_deg'],
        midpoint_Lend_residual_mm=optical['midpoint_Lend_residual_mm'],
        midpoint_Lend_residual_norm_mm=math.hypot(*optical['midpoint_Lend_residual_mm']),
        gyro_through_Lend_deg=response['gyro_through_lend_measured_time_deg'],
        pulse_speed_mm_s=response['pulse_speed_measured_time_mean_mm_s'],battery_min_v=response['battery_min_v'],
        fixed_settings=fixed,source_actual_settings=response['source_settings'],
        source_actual_settings_sha256=sha(Path(response['source_settings']))))
fixed_equal=all(row['fixed_settings']==rows[0]['fixed_settings'] for row in rows)
assert fixed_equal
save(STAGE/'three_run_response_comparison.json',dict(rows=rows,fixed_readback_settings_equal=fixed_equal,
    same_setting_repeats=0,new_coefficients_identified=False,
    interpretation='Different angle/length settings and varying speed/voltage. Video and gyro both show current underturn. Repeat current settings before further extrapolation.'))
save(STAGE/'firmware_readiness_review.json',dict(mouse='A',motion='out_r45',speed_mm_s=1800,
    parameters=read(RUN/'saved_1ms.settings.json')['parameters'],measured_angle_deg=angle,
    measured_Lend_residual_mm=review['midpoint_Lend_residual_mm'],measured_Lend_residual_norm_mm=error,
    review_angle_tolerance_deg=.5,review_position_tolerance_mm=2.,target_achieved=False,
    firmware_candidate_qualified=False,optical_measurement_quality_eligible=True,
    native_auto_stop_confirmed=summary['auto_stop_confirmed'],native_trajectory_model_rmse_mm=summary['trajectory_fit_rmse_mm'],
    evaluation_frame='Relative marker midpoint aligned to measured entry, not calibrated board-fixed axle/body yaw.',
    source_parameters_changed=False,flash_written=False,repeatability_confirmed=False,
    left_verified=False,continuous_run_verified=False,operator_confirmation=confirmation,
    next_unexecuted_repeat=str(NEXT/'candidate.settings.json')))
save(STAGE/'ekf_identification_status.json',dict(mouse='A',motion='out_r45',speed_mm_s=1800,suction_duty=650,
    runtime_mode='control',new_coefficients_identified=False,firmware_ekf_changed=False,
    current_source_eligibility='Task/Inc/turn_pose_ekf.h: Long_turnR90, or suction Long_turnR180 only; OUTR45 excluded.',
    source_eligibility_sha256=sha(ROOT/'Task/Inc/turn_pose_ekf.h'),
    source_is_not_a_binary_firmware_identity=True,
    status='NO_NEW_EKF_PROFILE_ADOPTED',
    limitations=['Calibrated body yaw and axle/IMU geometry remain unverified. Path midpoint alone does not identify slip/noise coefficients.',
        'Current source eligibility excludes OUTR45; modecontrol readback alone is not proof of active EKF application.']))

fig,axes=plt.subplots(1,2,figsize=(10,4),layout='constrained')
labels=['c-2.0\n19.5/18.8','c+0.1\n19.5/18.8','c+0.8\n19.9/22.1']
for ax,key,title in zip(axes,('exit_angle_deg','midpoint_Lend_residual_norm_mm'),
        ('Video exit angle [deg]','Relative Lend position error [mm]')):
    values=[row[key] for row in rows]
    ax.bar(labels,values,color=['#8895a7','#4282bb','#c77c34'])
    for x,value in enumerate(values):ax.text(x,value+.04,f'{value:.2f}',ha='center')
    ax.set_ylabel(title);ax.grid(axis='y',alpha=.2)
axes[0].set_ylim(40,49);axes[0].axhspan(44.5,45.5,color='green',alpha=.12)
axes[0].axhline(45,color='green',ls='--',lw=1)
axes[1].set_ylim(0,4.4);axes[1].axhline(2,color='green',ls='--',lw=1)
fig.suptitle('Mouse A OUTR45 1800: distinct settings, one run each / no adoption')
fig.savefig(STAGE/'three_run_comparison.png',dpi=160);plt.close(fig)

entry=f'''## 2026-11-21の補正+0.8度・19.9 / 22.1 mmの結果

7本目は機体A・右45度出口1800を補正+0.8度・Lstart19.9 / Lend22.1 mmで1回走行した。機体A・idleと実機設定を照合し、保存1 msログ199点・drop0、連続CSVの走行区間は欠落0・CRC0。操作者は停止・接触なし・両方OFFで待機を確認し、BLE切断済み。走行の再送は0回。

[入口時計・中点の評価]({STAGE.name}/midpoint_brake_clock_v1/midpoint_review.json)で出口角{angle:.3f}度、目標より2.581度不足。Lend残差は右+1.066 / 前+1.872 mm、距離{error:.3f} mm。位置は前回3.609 mmから縮まったが、角度45±0.5度・位置2 mm以内の両条件を満たさず、ファーム反映を保留する。入口に合わせた相対マーカー中点の単体評価で、盤面固定・絶対車軸の合格ではない。

純走行・入口・出口・Lend前後の実観測/登録100%、入口29点・出口31点。入口時計RMSE0.438 mm・半窓時刻差0.820 ms、ブレーキ終了後の静止ばらつき0.103 mmで、独立した観測品質を通過。元nativeは軌跡モデルRMSE8.700 mmのためauto_stop_confirmed:falseを保持した。

[1 ms追従]({STAGE.name}/tracking_1ms_review_v1/verification.json)はLendまでのジャイロ積分42.016度、車輪パルス平均速度1794 mm/s、最低電圧7.711 V。動画も不足方向で一致する。[3本の比較]({STAGE.name}/three_run_comparison.png)と[実機設定比較]({STAGE.name}/three_run_response_comparison.json)を保存した。PID/FF・半径・吸引等の読み戻しは一致するが、補正・長さ・速度・電圧は同条件ではなく、補正だけの効果や同条件のばらつきを断定しない。

次は同じ補正+0.8度・19.9 / 22.1 mmを[独立した新しい試行]({NEXT.name}/candidate.settings.json)としてもう1回取得し、再現性を先に確認する。入力検証済みで未実行。今回の送信済み指令は再送せず、新しい配置・両方ON・手離し確認を必要とする。現在は両方OFFで待機を維持する。

[状態v9](session_outr45_status_20261121_v9.json)を参照。出口45度のファームソース・Flash・EKF係数は変更していない。既存ソースのEKF適用対象は右90度/吸引時右180度で、右45度出口を含まないため、modecontrol表示だけで適用ありとは扱わない。新EKF係数は未同定。

'''
path=BASE/'README.md'
old=path.read_text(encoding='utf-8');heading,tail=old.split('\n',1)
path.write_text(heading+'\n\n'+entry+tail.lstrip('\n'),encoding='utf-8')
save(BASE/'session_outr45_status_20261121_v9.json',dict(recorded_at_utc=datetime.now(timezone.utc).isoformat(),
    state='SEVEN_RUNS_REVIEWED_CPLUS0P8_UNDERTURN_SAME_SETTING_REPEAT_READY_OFF',
    mouse='A',motion='out_r45',speed_mm_s=1800,completed_physical_runs=7,precise_1ms_complete_runs=6,
    latest_trial=str(RUN/'trial.json'),latest_recording_id=trial['recording_id'],latest_video_exit_deg=angle,
    latest_Lend_residual_mm=review['midpoint_Lend_residual_mm'],latest_Lend_residual_norm_mm=error,
    latest_gyro_through_Lend_deg=tracking['gyro_through_lend_measured_time_deg'],latest_1ms_count=199,
    latest_battery_min_v=tracking['battery_min_v'],latest_pulse_speed_mm_s=tracking['pulse_speed_measured_time_mean_mm_s'],
    corrected_optical_geometry_eligible=True,native_auto_stop_confirmed=False,native_model_rmse_mm=summary['trajectory_fit_rmse_mm'],
    both_powered_off=True,contact_detected=False,ble_disconnected=True,execution_resend_count=0,
    next_trial_id=NEXT_ID,next_settings=str(NEXT/'candidate.settings.json'),next_type='independent_repeat_same_settings',
    next_executed=False,next_operator_ready=False,parameter_adoption_allowed=False,
    firmware_source_changed=False,firmware_written=False,new_ekf_parameters_adopted=False,
    limitations=['Current angle42.419deg and position2.154mm miss simultaneous45±0.5deg/2mm review targets.',
        'No same-setting repeat yet for this candidate; distinct settings/speed/voltage do not isolate parameter effects.',
        'Relative marker midpoint, absolute body yaw/axle geometry, left and continuous-run validation unavailable.']))
print(json.dumps(dict(angle_deg=angle,Lend_residual_norm_mm=error,next_validation=validation.returncode,
    next_type='independent_repeat_same_settings',next_executed=False,parameter_adoption_allowed=False)),flush=True)
