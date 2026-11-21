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
STAGE=BASE/'repeat_cplus0p8_Ls19p9_Le22p1_20261121_02'
PREVIOUS=BASE/'joint_cplus0p8_Ls19p9_Le22p1_20261121_01'
ID='A_OUTR45_1800_cplus0p8_Ls19p9_Le22p1_20261121_002'
RUN=STAGE/'local_execution'/('run_'+ID)
NEXT=BASE/'fixed_lengths_cminus1p7_Ls19p9_Le22p1_20261121_01'
NEXT_ID='A_OUTR45_1800_cminus1p7_Ls19p9_Le22p1_20261121_001'

def read(path):return json.loads(path.read_text(encoding='utf-8'))
def save(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('x',encoding='utf-8') as file:
        json.dump(value,file,ensure_ascii=False,indent=2,allow_nan=False);file.write('\n')
def sha(path):
    with path.open('rb') as file:return hashlib.file_digest(file,'sha256').hexdigest()

trial=read(RUN/'trial.json')
confirmation=read(RUN/'operator_postrun_confirmation.json')
assert confirmation['both_powered_off'] and confirmation['contact_detected'] is False
assert trial['saved_log']['frame_count']==196 and trial['execution_resend_count']==0
fields=['mouse','motion_name','preset_speed_mm_s','turn_count','parameters','pid','feedforward','suction']
rows=[];actual=[]
for folder in (PREVIOUS,STAGE):
    optical=read(folder/'midpoint_brake_clock_v1/midpoint_review.json')
    tracking=read(folder/'tracking_1ms_review_v1/verification.json')
    settings_path=Path(tracking['source_settings']);settings=read(settings_path)
    actual.append({key:settings[key] for key in fields})
    assert optical['independent_optical_geometry_eligible']
    rows.append(dict(stage=folder.name,angle_deg=optical['exit_path_direction_change_deg'],
        Lend_right_forward_mm=optical['midpoint_Lend_right_forward_mm'],
        Lend_residual_mm=optical['midpoint_Lend_residual_mm'],Lend_error_norm_mm=math.hypot(*optical['midpoint_Lend_residual_mm']),
        gyro_through_Lend_deg=tracking['gyro_through_lend_measured_time_deg'],
        pulse_speed_mm_s=tracking['pulse_speed_measured_time_mean_mm_s'],battery_min_v=tracking['battery_min_v'],
        saved_count=tracking['sample_count'],entry_clock_rms_mm=optical['clock']['incoming_raw_wheel_fit_rms_mm'],
        entry_clock_half_spread_ms=optical['clock']['half_window_zero_spread_ms'],
        Lstart_delta_from_exit_line_mm=optical['Lstart_delta_from_exit_line_mm'],
        segment_observation_quality=optical['segment_observation_quality'],
        source_settings=str(settings_path),source_settings_sha256=sha(settings_path)))
assert actual[0]==actual[1]
mean_angle=sum(row['angle_deg'] for row in rows)/2
angle_span=abs(rows[1]['angle_deg']-rows[0]['angle_deg'])
delta=[rows[1]['Lend_residual_mm'][i]-rows[0]['Lend_residual_mm'][i] for i in range(2)]
position_change=math.hypot(*delta)
chosen_correction=round(.8+mean_angle-45,1)
assert chosen_correction==-1.7
comparison=dict(mouse='A',motion='out_r45',speed_mm_s=1800,same_settings_readback=True,rows=rows,
    repeated_runs=2,mean_angle_deg=mean_angle,angle_range_deg=angle_span,
    Lend_position_change_right_forward_mm=delta,Lend_position_change_norm_mm=position_change,
    angular_underturn_repeated=True,position_repeatability_confirmed=False,
    calibrated_absolute_axle_body_yaw=False,formal_statistical_repeatability_claim=False,
    interpretation='Two runs both underturn near42.5deg; forward position changes5.11mm. Do not retune lengths from either run alone.',
    source_files_unchanged=True)
save(STAGE/'same_setting_repeat_comparison.json',comparison)

NEXT.mkdir(exist_ok=False)
settings=read(STAGE/'candidate.settings.json')
settings['source']='Unexecuted angle-only comparison: two c+0.8deg runs measured42.419/42.556deg. Unit response proposes-1.7deg; keep Lstart19.9/Lend22.1mm because position varies.'
settings['parameters']['correction']=chosen_correction
save(NEXT/'candidate.settings.json',settings)
for name in ('config.json','static_pose_after_settle.py'):shutil.copy2(STAGE/name,NEXT/name)
wrapper=(STAGE/'run_recorded_once.py').read_text(encoding='utf-8');assert wrapper.count(ID)==1
with (NEXT/'run_recorded_once.py').open('x',encoding='utf-8') as file:file.write(wrapper.replace(ID,NEXT_ID))
save(NEXT/'candidate_basis.json',dict(source_comparison=str(STAGE/'same_setting_repeat_comparison.json'),
    old_correction_deg=.8,measured_mean_angle_deg=mean_angle,target_angle_deg=45,
    unit_response_proposal_deg=.8+mean_angle-45,chosen_correction_deg=chosen_correction,
    fixed_lstart_mm=19.9,fixed_lend_mm=22.1,changed_setting='correction_only',executed=False,
    parameter_adoption_allowed=False,
    assumptions=['Unit local response is a comparison prediction, not a calibrated transfer function.',
        'Two near-identical angles do not establish position repeatability or continuous-run stability.',
        'Hold lengths/PID/FF/radius/suction unchanged to isolate the next angle comparison.'],
    requires_fresh_operator_placement_power_and_hands_clear=True))
validation=subprocess.run([str(ROOT/'../../Debug_camera/.venv/Scripts/python.exe'),str(NEXT/'run_recorded_once.py')],
    cwd=ROOT,env=dict(os.environ,PYTHONUTF8='1'),capture_output=True,text=True,encoding='utf-8')
save(NEXT/'input_validation.json',dict(returncode=validation.returncode,stdout=validation.stdout,
    stderr=validation.stderr,robot_commands_sent=False));assert validation.returncode==0,validation.stderr

current=rows[-1];review=read(STAGE/'midpoint_brake_clock_v1/midpoint_review.json')
summary=read(RUN/'full/summary.json');tracking=read(STAGE/'tracking_1ms_review_v1/verification.json')
save(STAGE/'firmware_readiness_review.json',dict(mouse='A',motion='out_r45',speed_mm_s=1800,
    parameters=actual[-1]['parameters'],measured_angle_deg=current['angle_deg'],
    measured_Lend_residual_mm=current['Lend_residual_mm'],measured_Lend_residual_norm_mm=current['Lend_error_norm_mm'],
    review_angle_tolerance_deg=.5,review_position_tolerance_mm=2.,target_achieved=False,
    firmware_candidate_qualified=False,optical_measurement_quality_eligible=True,
    native_auto_stop_confirmed=summary['auto_stop_confirmed'],native_trajectory_model_rmse_mm=summary['trajectory_fit_rmse_mm'],
    angular_underturn_repeated=True,angle_span_two_runs_deg=angle_span,position_repeatability_confirmed=False,
    source_parameters_changed=False,flash_written=False,left_verified=False,continuous_run_verified=False,
    operator_confirmation=confirmation,next_unexecuted_comparison=str(NEXT/'candidate.settings.json'),
    evaluation_frame='Measured entry aligned relative marker midpoint, not calibrated board-fixed axle/body yaw.'))
ekf=read(PREVIOUS/'ekf_identification_status.json')
ekf.update(source_trial=str(RUN/'trial.json'),source_eligibility_sha256=sha(ROOT/'Task/Inc/turn_pose_ekf.h'))
save(STAGE/'ekf_identification_status.json',ekf)

fig,axes=plt.subplots(1,2,figsize=(9,4),layout='constrained')
for ax,key,title in zip(axes,('angle_deg','Lend_error_norm_mm'),('Video exit angle [deg]','Relative Lend position error [mm]')):
    values=[row[key] for row in rows]
    ax.bar(['Run1','Run2'],values,color=['#4282bb','#c77c34'])
    for x,value in enumerate(values):ax.text(x,value+.04,f'{value:.2f}',ha='center')
    ax.set_ylabel(title);ax.grid(axis='y',alpha=.2)
axes[0].set_ylim(40,47);axes[0].axhspan(44.5,45.5,color='green',alpha=.12);axes[0].axhline(45,color='green',ls='--',lw=1)
axes[1].set_ylim(0,8.2);axes[1].axhline(2,color='green',ls='--',lw=1)
fig.suptitle('Mouse A OUTR45 1800: same c+0.8 / Lstart19.9 / Lend22.1, no adoption')
fig.savefig(STAGE/'same_setting_repeat_comparison.png',dpi=160);plt.close(fig)

entry=f'''## 2026-11-21の補正+0.8度・19.9 / 22.1 mmの再測定

8本目は直前と同じ補正+0.8度・Lstart19.9 / Lend22.1 mmで、独立した新しい試行として1回実行した。機体A・idleと設定読み戻しを確認し、保存1 msログ196点・drop0、連続CSVの走行区間は欠落0・CRC0。操作者は停止・接触なし・両方OFFを確認し、BLE切断済み。走行再送は0回。

[相対中点の評価]({STAGE.name}/midpoint_brake_clock_v1/midpoint_review.json)は出口角42.556度、Lend残差は右+1.002 / 前+6.984 mm、距離{current['Lend_error_norm_mm']:.3f} mm。[同じ設定の比較]({STAGE.name}/same_setting_repeat_comparison.png)では出口角42.419 / 42.556度、角度差0.137度で、2本とも不足した。位置残差は2.154 / {current['Lend_error_norm_mm']:.3f} mm、位置差は右-0.064 / 前+5.112 mmで、位置の再現性は確認できない。入口に合わせた相対評価であり、盤面固定・絶対車軸の合格ではない。

Lendまでのジャイロ積分は42.016 / 42.149度。実機取得の全パラメータ・PID・FF・吸引・回数は一致する。車輪パルス平均速度は1794 / 1826 mm/s、最低電圧7.711 / 7.625 Vで同一ではない。出口直線からのLstart補正診断も-0.203 / -6.361 mmと変化し、位置差をLend時計だけの誤差とは断定しない。[比較JSON]({STAGE.name}/same_setting_repeat_comparison.json)に入力ハッシュを保存した。

今回の純走行区間は実観測97.56%・登録98.78%。入口27点・出口24点で、欠測フレーム2257/2280は観測に使わず、Lendの補間前後は実検出・登録を確認した。入口時計RMSE0.413 mm・半窓時刻差0.379 ms、ブレーキ終了後静止ばらつき0.095 mm。従来の独立時計・観測品質条件を通過した。元nativeモデルRMSE17.820 mmとauto_stop_confirmed:falseは変更していない。

2本の平均角{mean_angle:.3f}度から単位応答を仮定した計算では、次は[補正-1.7度だけの比較]({NEXT.name}/candidate.settings.json)となる。長さ19.9 / 22.1 mm・半径・PID/FF・吸引は固定し、位置の長さ補正は保留する。[根拠]({NEXT.name}/candidate_basis.json)・入力検証を保存したが未実行で、線形応答や目標達成を保証しない。次の実行には新しい配置・両方ON・手離し確認が必要。現在は両方OFFで待機する。

[状態v10](session_outr45_status_20261121_v10.json)を参照。右45度出口のファームソース・Flash・EKF係数は未変更。2本の角度不足の反復確認であり、統計的な再現性や連続走行の検証ではない。

'''
path=BASE/'README.md';old=path.read_text(encoding='utf-8');heading,tail=old.split('\n',1)
path.write_text(heading+'\n\n'+entry+tail.lstrip('\n'),encoding='utf-8')
save(BASE/'session_outr45_status_20261121_v10.json',dict(recorded_at_utc=datetime.now(timezone.utc).isoformat(),
    state='EIGHT_RUNS_REVIEWED_UNDERTURN_REPEATED_POSITION_VARIES_CMINUS1P7_READY_OFF',
    mouse='A',motion='out_r45',speed_mm_s=1800,completed_physical_runs=8,precise_1ms_complete_runs=7,
    latest_trial=str(RUN/'trial.json'),latest_recording_id=trial['recording_id'],latest_video_exit_deg=current['angle_deg'],
    latest_Lend_residual_mm=current['Lend_residual_mm'],latest_Lend_residual_norm_mm=current['Lend_error_norm_mm'],
    latest_gyro_through_Lend_deg=tracking['gyro_through_lend_measured_time_deg'],latest_1ms_count=196,
    latest_battery_min_v=tracking['battery_min_v'],latest_pulse_speed_mm_s=tracking['pulse_speed_measured_time_mean_mm_s'],
    angular_underturn_repeated=True,angle_range_two_runs_deg=angle_span,position_change_two_runs_mm=position_change,
    position_repeatability_confirmed=False,corrected_optical_geometry_eligible=True,native_auto_stop_confirmed=False,
    native_model_rmse_mm=summary['trajectory_fit_rmse_mm'],both_powered_off=True,contact_detected=False,
    ble_disconnected=True,execution_resend_count=0,next_trial_id=NEXT_ID,next_settings=str(NEXT/'candidate.settings.json'),
    next_correction_deg=chosen_correction,next_lstart_mm=19.9,next_lend_mm=22.1,next_type='correction_only_comparison',
    next_executed=False,next_operator_ready=False,parameter_adoption_allowed=False,
    firmware_source_changed=False,firmware_written=False,new_ekf_parameters_adopted=False))
print(json.dumps(dict(angle_deg=current['angle_deg'],position_error_mm=current['Lend_error_norm_mm'],
    mean_angle_deg=mean_angle,angle_range_deg=angle_span,position_change_mm=position_change,
    next_correction_deg=chosen_correction,next_validation=validation.returncode,next_executed=False)),flush=True)
