from pathlib import Path
from datetime import datetime,timezone
import hashlib
import json
import math
import shutil

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_outr45_20261120_v1'
STAGE=BASE/'repeat_cminus1p7_Ls15p4_Le15p9_20261121_02'
ID='A_OUTR45_1800_cminus1p7_Ls15p4_Le15p9_20261121_002'
RUN=STAGE/'local_execution'/('run_'+ID)
FAILED=BASE/'lengths_cminus1p7_Ls17p0_Le17p4_20261121_01'
FAILED_ID='A_OUTR45_1800_cminus1p7_Ls17p0_Le17p4_20261121_001'
CURRENT=BASE/'retry_lengths_cminus1p7_Ls17p0_Le17p4_20261121_02'
CURRENT_ID='A_OUTR45_1800_cminus1p7_Ls17p0_Le17p4_20261121_002'
def read(path):return json.loads(path.read_text(encoding='utf-8'))
def save(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('x',encoding='utf-8') as file:
        json.dump(value,file,ensure_ascii=False,indent=2,allow_nan=False);file.write('\n')
def sha(path):
    with path.open('rb') as file:return hashlib.file_digest(file,'sha256').hexdigest()

rows=[];settings=[]
for folder in (BASE/'lengths_cminus1p7_Ls15p4_Le15p9_20261121_01',STAGE):
    optical=read(folder/'midpoint_brake_clock_v1/midpoint_review.json')
    tracking=read(folder/'tracking_1ms_review_v1/verification.json');actual_path=Path(tracking['source_settings']);actual=read(actual_path)
    assert optical['independent_optical_geometry_eligible']
    settings.append({key:actual[key] for key in ('mouse','motion_name','preset_speed_mm_s','turn_count','parameters','pid','feedforward','suction')})
    rows.append(dict(stage=folder.name,angle_deg=optical['exit_path_direction_change_deg'],
        Lend_residual_mm=optical['midpoint_Lend_residual_mm'],error_norm_mm=math.hypot(*optical['midpoint_Lend_residual_mm']),
        gyro_through_Lend_deg=tracking['gyro_through_lend_measured_time_deg'],pulse_speed_mm_s=tracking['pulse_speed_measured_time_mean_mm_s'],
        battery_min_v=tracking['battery_min_v'],projection=optical['length_only_candidate_at_current_angle'],
        source_settings=str(actual_path),source_settings_sha256=sha(actual_path)))
assert settings[0]==settings[1]
delta=[rows[1]['Lend_residual_mm'][i]-rows[0]['Lend_residual_mm'][i] for i in range(2)]
mean_angle=sum(row['angle_deg'] for row in rows)/2
mean_ls=sum(row['projection']['request_Lstart_mm'] for row in rows)/2
mean_le=sum(row['projection']['request_Lend_mm'] for row in rows)/2
predicted=[]
for row in rows:
    angle=math.radians(row['angle_deg'])
    residual=[row['Lend_residual_mm'][0]+1.5*math.sin(angle),row['Lend_residual_mm'][1]+1.6+1.5*math.cos(angle)]
    predicted.append(dict(source_stage=row['stage'],unverified_residual_mm=residual,unverified_error_norm_mm=math.hypot(*residual)))
save(STAGE/'same_setting_repeat_comparison_v2.json',dict(rows=rows,same_actual_settings=True,mean_angle_deg=mean_angle,
    angle_span_deg=abs(rows[1]['angle_deg']-rows[0]['angle_deg']),position_change_mm=delta,
    position_change_norm_mm=math.hypot(*delta),mean_projected_lstart_mm=mean_ls,mean_projected_lend_mm=mean_le,
    angle_review_pass_both=True,position_review_pass_both=False,position_tolerance_mm=2.,
    next_length_proposal=dict(lstart_mm=17.0,lend_mm=17.4,correction_deg=-1.7,predictions=predicted),
    prediction_verified=False,parameter_adoption_allowed=False,formal_statistical_repeatability_claim=False))
review=read(STAGE/'midpoint_brake_clock_v1/midpoint_review.json');summary=read(RUN/'full/summary.json')
confirmation=read(RUN/'operator_postrun_confirmation.json');assert confirmation['both_powered_off']
save(STAGE/'firmware_readiness_review.json',dict(mouse='A',motion='out_r45',speed_mm_s=1800,
    actual_settings=settings[-1],measured_angle_deg=rows[-1]['angle_deg'],position_error_mm=rows[-1]['error_norm_mm'],
    review_angle_tolerance_deg=.5,review_position_tolerance_mm=2.,angle_target_passed=True,position_target_passed=False,
    firmware_candidate_qualified=False,measured_same_setting_runs=2,optical_measurement_quality_eligible=True,
    native_auto_stop_confirmed=summary['auto_stop_confirmed'],native_model_rmse_mm=summary['trajectory_fit_rmse_mm'],
    operator_confirmation=confirmation,source_parameters_changed=False,flash_written=False,
    left_verified=False,continuous_run_verified=False,
    evaluation_frame='Measured entry aligned relative marker midpoint; not calibrated board-fixed axle/body yaw.'))
ekf=read(BASE/'lengths_cminus1p7_Ls15p4_Le15p9_20261121_01/ekf_identification_status.json')
ekf['source_trial']=str(RUN/'trial.json');save(STAGE/'ekf_identification_status.json',ekf)

fig,axes=plt.subplots(1,2,figsize=(9,4),layout='constrained')
for ax,key,label in zip(axes,('angle_deg','error_norm_mm'),('Video exit angle [deg]','Relative Lend position error [mm]')):
    values=[row[key] for row in rows];ax.bar(['Run1','Run2'],values,color=['#4282bb','#c77c34'])
    for i,value in enumerate(values):ax.text(i,value+.025,f'{value:.2f}',ha='center')
    ax.set_ylabel(label);ax.grid(axis='y',alpha=.2)
axes[0].set_ylim(43,47);axes[0].axhspan(44.5,45.5,color='green',alpha=.12);axes[0].axhline(45,color='green',ls='--',lw=1)
axes[1].set_ylim(0,4.4);axes[1].axhline(2,color='green',ls='--',lw=1)
fig.suptitle('Mouse A OUTR45 1800: same c-1.7 / 15.4,15.9mm / no adoption')
fig.savefig(STAGE/'same_setting_repeat_comparison.png',dpi=160);plt.close(fig)

job=read(FAILED/'local_execution/jobs'/(FAILED_ID+'.json'))
assert job['result']['execution_write_attempted'] is False and job['event']=='static_check'
manifest_path=ROOT/'tools/logs/pixel_camera/20261121_044157_eadbbf32.capture.json'
shutil.copy2(manifest_path,FAILED/'failed_static_capture_manifest.snapshot.json')
bjob_path=ROOT/'tools/logs/pixel_camera/search370_B_20261121_v1/right_kp5_01/local_execution/jobs/B_SEARCHR370_kp5_20261121_004.json'
bjob=read(bjob_path)
save(FAILED/'capture_overlap_evidence.json',dict(source_job=str(bjob_path),source_job_sha256=sha(bjob_path),
    B_started_at=bjob['started_at'],B_finished_at=bjob.get('finished_at'),B_final_state=bjob['state'],
    A_static_started_at=read(manifest_path)['started_at'],A_execution_write_attempted=False,
    A_static_error=read(manifest_path)['error'],initial_approval_review_rejected=True,
    initial_approval_review_reason='B capture process could compete for the same camera.',
    additional_read_only_checks_completed=True,second_approval_review_passed=True,
    capture_then_failed_from_BUSY=True,user_instruction='B側の録画走行を停止し、Aを続ける',
    resumed_with_new_request_id=CURRENT_ID,old_request_not_replayed=True))
current_run=CURRENT/'local_execution'/('run_'+CURRENT_ID)
assert read(current_run/'operator_postrun_confirmation.json')['both_powered_off']
save(FAILED/'operator_resolution.json',dict(off_between_failed_attempt_and_retry_confirmed=False,
    new_ON_placement_hands_clear_confirmed=True,B_stop_and_A_priority_explicit=True,
    final_followup_run_off_confirmed=True,followup_confirmation=str(current_run/'operator_postrun_confirmation.json'),
    failed_request_execution_write_attempted=False))

entry=f'''## 2026-11-21の15.4 / 15.9 mm再測定と17.0 / 17.4 mmの準備

11本目は同じ補正-1.7度・15.4 / 15.9 mmで1回実行した。保存1 msログ191点・drop0、連続CSVの走行区間は欠落0・CRC0。機体A・idle・実機設定を照合し、操作者は停止・接触なし・両方OFFを確認した。BLE切断済みで走行再送は0回。

[中点評価]({STAGE.name}/midpoint_brake_clock_v1/midpoint_review.json)は出口角44.708度、Lend残差は右-1.105 / 前-1.689 mm、距離{rows[-1]['error_norm_mm']:.3f} mm。前回45.430度・3.692 mmと合わせ、角度は2本とも45±0.5度の目安内だが、位置2 mm以内は満たしていない。[比較図]({STAGE.name}/same_setting_repeat_comparison.png)と[設定・投影比較]({STAGE.name}/same_setting_repeat_comparison_v2.json)を保存した。2本の角度平均{mean_angle:.3f}度、位置差{math.hypot(*delta):.3f} mm。入口基準の相対マーカー中点評価であり、盤面固定・絶対車軸の合格ではない。

今回のジャイロ積分45.120度、車輪パルス平均速度1837 mm/s、最低電圧7.441 V。純走行・入口・出口・Lend前後の実観測/登録100%、入口29点・出口34点、入口時計RMSE0.342 mm・半窓時刻差0.402 ms、静止ばらつき0.069 mm。元nativeモデルRMSE13.020 mmとauto_stop_confirmed:falseは変更していない。

2本の長さ投影平均17.0 / 17.4 mmを次候補として準備した。最初の要求{FAILED_ID}は静止録画開始時にカメラBUSYで停止し、Aの接続・走行は0回。事前の自動審査はBとの撮影競合を理由に一度拒否され、追加確認後の再審査は通ったが、確認直後にBの新規撮影が始まっていた。[競合時刻の証拠]({FAILED.name}/capture_overlap_evidence.json)と失敗manifestを保持する。

ユーザーのB側停止・A続行指示後、Bの完了・切断・走行処理終了を確認し、新しい配置・両方ON・手離し確認で要求{CURRENT_ID}を別試行として実行した。この後続試行は録画・保存1 msログ192点を取得済みで全フレーム解析中。停止・接触なし・両方OFFの操作者確認済み。

[状態v13](session_outr45_status_20261121_v13.json)を参照。ファームソース・Flash・EKF係数は変更していない。

'''
path=BASE/'README.md';old=path.read_text(encoding='utf-8');heading,tail=old.split('\n',1)
path.write_text(heading+'\n\n'+entry+tail.lstrip('\n'),encoding='utf-8')
save(BASE/'session_outr45_status_20261121_v13.json',dict(recorded_at_utc=datetime.now(timezone.utc).isoformat(),
    state='ELEVENTH_REVIEW_COMPLETE_BUSY_FAILURE_PRESERVED_TWELFTH_ANALYSIS_RUNNING_OFF',
    mouse='A',motion='out_r45',speed_mm_s=1800,completed_physical_runs=12,precise_1ms_complete_runs=11,
    latest_completed_review_trial=str(RUN/'trial.json'),latest_review_angle_deg=rows[-1]['angle_deg'],
    latest_review_position_error_mm=rows[-1]['error_norm_mm'],current_trial=str(current_run/'trial.json'),
    current_analysis_pending=True,current_both_powered_off=True,current_contact_detected=False,
    failed_camera_request_id=FAILED_ID,failed_request_executed=False,failed_error='CAMERA_BUSY',
    current_request_id=CURRENT_ID,execution_resend_count=0,B_capture_stopped_by_user=True,
    parameter_adoption_allowed=False,firmware_source_changed=False,firmware_written=False,new_ekf_parameters_adopted=False))
print(json.dumps(dict(previous_review_finalized=True,BUSY_failure_preserved=True,current_analysis_pending=True)),flush=True)
