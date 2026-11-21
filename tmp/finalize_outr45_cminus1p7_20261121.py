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
STAGE=BASE/'fixed_lengths_cminus1p7_Ls19p9_Le22p1_20261121_01'
ID='A_OUTR45_1800_cminus1p7_Ls19p9_Le22p1_20261121_001'
RUN=STAGE/'local_execution'/('run_'+ID)
NEXT=BASE/'lengths_cminus1p7_Ls15p4_Le15p9_20261121_01'
NEXT_ID='A_OUTR45_1800_cminus1p7_Ls15p4_Le15p9_20261121_001'

def read(path):return json.loads(path.read_text(encoding='utf-8'))
def save(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('x',encoding='utf-8') as file:
        json.dump(value,file,ensure_ascii=False,indent=2,allow_nan=False);file.write('\n')
def sha(path):
    with path.open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()

review=read(STAGE/'midpoint_brake_clock_v1/midpoint_review.json')
tracking=read(STAGE/'tracking_1ms_review_v1/verification.json')
summary=read(RUN/'full/summary.json');trial=read(RUN/'trial.json')
confirmation=read(RUN/'operator_postrun_confirmation.json')
assert confirmation['both_powered_off'] and confirmation['contact_detected'] is False
assert trial['saved_log']['frame_count']==197 and trial['execution_resend_count']==0
assert review['independent_optical_geometry_eligible']
angle=review['exit_path_direction_change_deg'];error=math.hypot(*review['midpoint_Lend_residual_mm'])
angle_passed=abs(angle-45)<=.5;position_passed=error<=2
assert angle_passed and not position_passed

rows=[];fixed=[]
for stage in ('joint_cplus0p8_Ls19p9_Le22p1_20261121_01','repeat_cplus0p8_Ls19p9_Le22p1_20261121_02',STAGE.name):
    folder=BASE/stage;optical=read(folder/'midpoint_brake_clock_v1/midpoint_review.json')
    response=read(folder/'tracking_1ms_review_v1/verification.json');actual_path=Path(response['source_settings']);actual=read(actual_path)
    same={key:actual[key] for key in ('mouse','motion_name','preset_speed_mm_s','turn_count','pid','feedforward','suction')}
    same['parameters']={key:value for key,value in actual['parameters'].items() if key!='correction'}
    fixed.append(same)
    rows.append(dict(stage=stage,parameters=actual['parameters'],angle_deg=optical['exit_path_direction_change_deg'],
        position_residual_mm=optical['midpoint_Lend_residual_mm'],position_error_mm=math.hypot(*optical['midpoint_Lend_residual_mm']),
        gyro_through_Lend_deg=response['gyro_through_lend_measured_time_deg'],pulse_speed_mm_s=response['pulse_speed_measured_time_mean_mm_s'],
        battery_min_v=response['battery_min_v'],source_settings=str(actual_path),source_settings_sha256=sha(actual_path)))
assert all(item==fixed[0] for item in fixed)
old_mean=(rows[0]['angle_deg']+rows[1]['angle_deg'])/2
save(STAGE/'correction_only_comparison.json',dict(rows=rows,all_settings_except_correction_equal=True,
    old_angle_mean_deg=old_mean,new_angle_deg=angle,new_correction_deg=-1.7,
    observed_slope_deg_per_correction_deg=(angle-old_mean)/(-1.7-.8),
    calibrated_causal_transfer_function=False,
    limitations=['Speed and voltage differ between runs. New correction is measured only once.',
        'Prior same-setting forward position varied5.11mm; do not claim position repeatability.']))

NEXT.mkdir(exist_ok=False)
settings=read(STAGE/'candidate.settings.json')
settings['source']='Unexecuted length-only comparison from angle44.958deg and current relative Lend residual(4.362,8.905)mm. Hold correction-1.7deg and all control settings.'
settings['parameters'].update(lstart=15.4,lend=15.9)
save(NEXT/'candidate.settings.json',settings)
for name in ('config.json','static_pose_after_settle.py'):shutil.copy2(STAGE/name,NEXT/name)
wrapper=(STAGE/'run_recorded_once.py').read_text(encoding='utf-8');assert wrapper.count(ID)==1
with (NEXT/'run_recorded_once.py').open('x',encoding='utf-8') as file:file.write(wrapper.replace(ID,NEXT_ID))
save(NEXT/'candidate_basis.json',dict(source_optical_review=str(STAGE/'midpoint_brake_clock_v1/midpoint_review.json'),
    measured_angle_deg=angle,measured_Lend_residual_mm=review['midpoint_Lend_residual_mm'],
    length_projection_solution=review['length_only_candidate_at_current_angle'],chosen_lstart_mm=15.4,chosen_lend_mm=15.9,
    fixed_correction_deg=-1.7,changed_settings=['lstart','lend'],executed=False,parameter_adoption_allowed=False,
    assumptions=['Translate along measured entry/exit directions while retaining the currently measured turn shape.',
        'Angle may change when lengths change; remeasure angle and position together.',
        'Same-setting forward position previously varied5.11mm. This one-run length projection is a candidate, not an adoption.'],
    requires_fresh_operator_placement_power_and_hands_clear=True))
validation=subprocess.run([str(ROOT/'../../Debug_camera/.venv/Scripts/python.exe'),str(NEXT/'run_recorded_once.py')],
    cwd=ROOT,env=dict(os.environ,PYTHONUTF8='1'),capture_output=True,text=True,encoding='utf-8')
save(NEXT/'input_validation.json',dict(returncode=validation.returncode,stdout=validation.stdout,stderr=validation.stderr,
    robot_commands_sent=False));assert validation.returncode==0,validation.stderr

save(STAGE/'firmware_readiness_review.json',dict(mouse='A',motion='out_r45',speed_mm_s=1800,
    parameters=read(RUN/'saved_1ms.settings.json')['parameters'],measured_angle_deg=angle,
    measured_Lend_residual_mm=review['midpoint_Lend_residual_mm'],measured_Lend_residual_norm_mm=error,
    review_angle_tolerance_deg=.5,review_position_tolerance_mm=2.,angle_target_passed=True,position_target_passed=False,
    target_achieved=False,firmware_candidate_qualified=False,optical_measurement_quality_eligible=True,
    native_auto_stop_confirmed=summary['auto_stop_confirmed'],native_trajectory_model_rmse_mm=summary['trajectory_fit_rmse_mm'],
    evaluation_frame='Measured entry aligned relative marker midpoint, not board-fixed calibrated axle/body yaw.',
    new_correction_measured_runs=1,position_repeatability_confirmed=False,left_verified=False,continuous_run_verified=False,
    source_parameters_changed=False,flash_written=False,operator_confirmation=confirmation,
    next_unexecuted_comparison=str(NEXT/'candidate.settings.json')))
ekf=read(BASE/'repeat_cplus0p8_Ls19p9_Le22p1_20261121_02/ekf_identification_status.json')
ekf.update(source_trial=str(RUN/'trial.json'),source_eligibility_sha256=sha(ROOT/'Task/Inc/turn_pose_ekf.h'))
save(STAGE/'ekf_identification_status.json',ekf)

fig,axes=plt.subplots(1,2,figsize=(10,4),layout='constrained')
labels=['c+0.8 run1','c+0.8 run2','c-1.7 run1']
for ax,key,title in zip(axes,('angle_deg','position_error_mm'),('Video exit angle [deg]','Relative Lend position error [mm]')):
    values=[row[key] for row in rows];ax.bar(labels,values,color=['#8895a7','#4282bb','#c77c34'])
    for x,value in enumerate(values):ax.text(x,value+.04,f'{value:.2f}',ha='center')
    ax.set_ylabel(title);ax.tick_params(axis='x',labelsize=9);ax.grid(axis='y',alpha=.2)
axes[0].set_ylim(40,47);axes[0].axhspan(44.5,45.5,color='green',alpha=.12);axes[0].axhline(45,color='green',ls='--',lw=1)
axes[1].set_ylim(0,11.2);axes[1].axhline(2,color='green',ls='--',lw=1)
fig.suptitle('Mouse A OUTR45 1800: fixed Lstart19.9/Lend22.1, no adoption')
fig.savefig(STAGE/'correction_only_comparison.png',dpi=160);plt.close(fig)

entry=f'''## 2026-11-21の補正-1.7度・19.9 / 22.1 mmの結果

9本目は補正だけ+0.8度から-1.7度へ変更し、長さ19.9 / 22.1 mm・半径・PID/FF・吸引・速度・回数の読み戻し一致を確認して1回実行した。機体A・idleを照合。保存1 msログ197点・drop0、連続CSVの走行区間は欠落0・CRC0。操作者は停止・接触なし・両方OFFを確認し、BLE切断済み。走行再送は0回。

[相対中点の評価]({STAGE.name}/midpoint_brake_clock_v1/midpoint_review.json)で出口角44.958度となり、角度45±0.5度の目安に入った。Lend残差は右+4.362 / 前+8.905 mm、距離{error:.3f} mmで位置2 mm以内を満たさず、パラメータ一式のファーム反映は保留する。角度補正-1.7度も現長さで単発確認した候補で、別の長さへ無条件に適用できる採用値ではない。

比較元の+0.8度2本は42.419 / 42.556度。今回のLendまでのジャイロ積分45.081度、車輪パルス平均速度1849 mm/s、最低電圧7.543 V。[比較図]({STAGE.name}/correction_only_comparison.png)と[設定比較JSON]({STAGE.name}/correction_only_comparison.json)を保存した。速度・電圧は変わり、新しい補正は1本だけである。先の同設定2本には前方向5.112 mmの位置変動もあり、位置再現性を確認済みとはしない。

純走行・入口・出口・Lend前後の実観測/登録100%。入口28点・出口24点、入口時計RMSE0.405 mm・半窓時刻差0.764 ms、ブレーキ終了後の静止ばらつき0.103 mmで独立観測品質を通過した。元nativeのモデルRMSE19.765 mmとauto_stop_confirmed:falseはそのまま保持した。観測品質の通過とモデル・目標の達成を分ける。評価基準は入口に合わせた相対マーカー中点で、盤面固定や絶対車軸精度ではない。

今回の実測出口方向へ投影するとLstart15.363 / Lend15.927 mmとなるため、次は[補正-1.7度を維持した15.4 / 15.9 mm]({NEXT.name}/candidate.settings.json)を比較する。[計算根拠]({NEXT.name}/candidate_basis.json)と入力検証を保存したが未実行。形状が長さ変更で変わらない仮定を含むため、出口角と位置を再測定する。次の実行には新しい配置・両方ON・手離し確認が必要。現在は両方OFFで待機する。

[状態v11](session_outr45_status_20261121_v11.json)を参照。右45度出口のファームソース・Flash・EKF係数は未変更。

'''
path=BASE/'README.md';old=path.read_text(encoding='utf-8');heading,tail=old.split('\n',1)
path.write_text(heading+'\n\n'+entry+tail.lstrip('\n'),encoding='utf-8')
save(BASE/'session_outr45_status_20261121_v11.json',dict(recorded_at_utc=datetime.now(timezone.utc).isoformat(),
    state='NINE_RUNS_REVIEWED_CMINUS1P7_ANGLE_PASSED_POSITION_FAILED_LENGTH_CANDIDATE_READY_OFF',
    mouse='A',motion='out_r45',speed_mm_s=1800,completed_physical_runs=9,precise_1ms_complete_runs=8,
    latest_trial=str(RUN/'trial.json'),latest_recording_id=trial['recording_id'],latest_video_exit_deg=angle,
    latest_Lend_residual_mm=review['midpoint_Lend_residual_mm'],latest_Lend_residual_norm_mm=error,
    latest_gyro_through_Lend_deg=tracking['gyro_through_lend_measured_time_deg'],latest_1ms_count=197,
    latest_battery_min_v=tracking['battery_min_v'],latest_pulse_speed_mm_s=tracking['pulse_speed_measured_time_mean_mm_s'],
    angle_target_passed=True,position_target_passed=False,new_correction_measured_runs=1,position_repeatability_confirmed=False,
    corrected_optical_geometry_eligible=True,native_auto_stop_confirmed=False,native_model_rmse_mm=summary['trajectory_fit_rmse_mm'],
    both_powered_off=True,contact_detected=False,ble_disconnected=True,execution_resend_count=0,
    next_trial_id=NEXT_ID,next_settings=str(NEXT/'candidate.settings.json'),next_correction_deg=-1.7,
    next_lstart_mm=15.4,next_lend_mm=15.9,next_type='length_only_comparison',next_executed=False,next_operator_ready=False,
    parameter_adoption_allowed=False,firmware_source_changed=False,firmware_written=False,new_ekf_parameters_adopted=False))
print(json.dumps(dict(angle_deg=angle,position_error_mm=error,angle_target_passed=True,
    next_correction_deg=-1.7,next_lstart_mm=15.4,next_lend_mm=15.9,next_validation=validation.returncode,next_executed=False)),flush=True)
