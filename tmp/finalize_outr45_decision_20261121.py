from pathlib import Path
from datetime import datetime,timezone
import hashlib
import json
import math

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_outr45_20261120_v1'
STAGE=BASE/'repeat_lengths_cminus1p7_Ls17p0_Le17p4_20261121_03'
ID='A_OUTR45_1800_cminus1p7_Ls17p0_Le17p4_20261121_003'
RUN=STAGE/'local_execution'/('run_'+ID)
OUT=ROOT/'tools/turn_analysis/20261121_A_turn1800_OUTR45_adoption'
AUDIT=BASE/'firmware_outR45_adoption_20261121_v1'
def read(path):return json.loads(path.read_text(encoding='utf-8'))
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def save(path,value):
    with path.open('x',encoding='utf-8') as file:
        json.dump(value,file,ensure_ascii=False,indent=2,allow_nan=False);file.write('\n')
review=read(STAGE/'midpoint_brake_clock_v1/midpoint_review.json')
tracking=read(STAGE/'tracking_1ms_review_v1/verification.json');trial=read(RUN/'trial.json')
summary=read(RUN/'full/summary.json');confirmation=read(RUN/'operator_postrun_confirmation.json')
assert trial['saved_log']['frame_count']==193 and trial['execution_resend_count']==0
assert confirmation['both_powered_off'] and confirmation['contact_detected'] is False
assert review['independent_optical_geometry_eligible'] and read(OUT/'firmware_verification.json')['passed']
rows=[];actuals=[]
for folder in (BASE/'retry_lengths_cminus1p7_Ls17p0_Le17p4_20261121_02',STAGE):
    item=read(folder/'midpoint_brake_clock_v1/midpoint_review.json');run=Path(item['source_trial']).parent
    actual=read(run/'saved_1ms.settings.json')
    actuals.append({key:actual[key] for key in ('mouse','motion_name','preset_speed_mm_s','turn_count','parameters','pid','feedforward','suction')})
    response=read(folder/'tracking_1ms_review_v1/verification.json')
    rows.append(dict(stage=folder.name,angle_deg=item['exit_path_direction_change_deg'],residual_mm=item['midpoint_Lend_residual_mm'],
        position_error_mm=math.hypot(*item['midpoint_Lend_residual_mm']),gyro_through_Lend_deg=response['gyro_through_lend_measured_time_deg'],
        pulse_speed_mm_s=response['pulse_speed_measured_time_mean_mm_s'],battery_min_v=response['battery_min_v']))
assert actuals[0]==actuals[1]
save(STAGE/'same_setting_repeat_comparison.json',dict(rows=rows,same_actual_settings=True,
    mean_angle_deg=sum(row['angle_deg'] for row in rows)/2,angle_range_deg=abs(rows[1]['angle_deg']-rows[0]['angle_deg']),
    selection='Not adopted: both optical exit directions exceed45.5deg.',parameter_adoption_allowed=False,
    future_unit_response_prediction_deg=-1.7+sum(row['angle_deg'] for row in rows)/2-45,
    future_prediction_selected=False,additional_run_requested=False))
save(STAGE/'firmware_readiness_review.json',dict(mouse='A',motion='out_r45',speed_mm_s=1800,
    parameters=actuals[-1]['parameters'],measured_angle_deg=rows[-1]['angle_deg'],position_error_mm=rows[-1]['position_error_mm'],
    angle_target_passed=False,position_target_passed=False,firmware_candidate_qualified=False,
    independent_optical_geometry_eligible=True,native_auto_stop_confirmed=summary['auto_stop_confirmed'],
    native_model_rmse_mm=summary['trajectory_fit_rmse_mm'],source_parameters_changed_to_this_trial=False,
    alternate_selected_settings=dict(lstart_mm=15.4,lend_mm=15.9,correction_deg=-1.7),
    alternate_selection_status='User-requested provisional decision, not a change to original measurement gates.',
    operator_confirmation=confirmation))
ekf=read(BASE/'retry_lengths_cminus1p7_Ls17p0_Le17p4_20261121_02/ekf_identification_status.json')
ekf['source_trial']=str(RUN/'trial.json');save(STAGE/'ekf_identification_status.json',ekf)
save(STAGE/'visual_review_findings.json',dict(method='Reviewed raw pre/entry/turn/Lend/exit/stopped contact sheets and actual midpoint overlays for both17.0/17.4mm runs.',
    raw_frame_visual_review_performed=True,full_real_time_video_playthrough_claimed=False,
    marker_colours=dict(front='red',rear='yellow'),trace_follows_observed_marker_midpoint=True,
    slow_overlay_video_available=True,visual_manifest=str(STAGE/'visual_trajectory_review_v1/manifest.json'),
    board_fixed_review=read(STAGE/'visual_trajectory_review_v1/board_fixed_review.json'),
    interpretation='Board-fixed exit inclination includes initial placement heading plus relative turn overrun. Relative residual is not board-fixed axle error.'))

selected={'lengths_cminus1p7_Ls15p4_Le15p9_20261121_01','repeat_cminus1p7_Ls15p4_Le15p9_20261121_02'}
cases=[('resume20261121_baseline01','midpoint_raw_wheel_clock_v1'),
 ('repeat_corr_minus1p5_20261121_02','midpoint_raw_wheel_clock_v2'),
 ('next_corr_minus2p5_20261121_01','midpoint_raw_wheel_clock_v1'),
 ('joint_cminus2p0_Ls19p5_Le18p8_20261121_01','midpoint_brake_clock_v1'),
 ('fixed_lengths_cplus0p1_Ls19p5_Le18p8_20261121_01','midpoint_brake_clock_v1'),
 ('joint_cplus0p8_Ls19p9_Le22p1_20261121_01','midpoint_brake_clock_v1'),
 ('repeat_cplus0p8_Ls19p9_Le22p1_20261121_02','midpoint_brake_clock_v1'),
 ('fixed_lengths_cminus1p7_Ls19p9_Le22p1_20261121_01','midpoint_brake_clock_v1'),
 ('lengths_cminus1p7_Ls15p4_Le15p9_20261121_01','midpoint_brake_clock_v1'),
 ('repeat_cminus1p7_Ls15p4_Le15p9_20261121_02','midpoint_brake_clock_v1'),
 ('retry_lengths_cminus1p7_Ls17p0_Le17p4_20261121_02','midpoint_brake_clock_v1'),
 (STAGE.name,'midpoint_brake_clock_v1')]
comparison=[]
for folder,output in cases:
    path=BASE/folder/output/'midpoint_review.json';item=read(path);run=Path(item['source_trial']).parent
    actual=read(run/'saved_1ms.settings.json')
    comparison.append(dict(stage=folder,parameters=actual['parameters'],angle_deg=item['exit_path_direction_change_deg'],
        residual_mm=item['midpoint_Lend_residual_mm'],error_norm_mm=math.hypot(*item['midpoint_Lend_residual_mm']),
        selected_for_provisional_adoption=folder in selected,source_review=str(path),source_review_sha256=sha(path)))
save(OUT/'decision_comparison.json',dict(precise_trials=comparison,total_physical_runs=13,
    complete_retained_1ms_runs=12,missing_1ms_trial='next_corr_minus1p5_20261121_01 excluded from precise geometry',
    camera_BUSY_attempt_executed=False,decision='Select measured15.4/15.9mm c-1.7deg pair. No unexecuted c0 prediction or17.0/17.4mm adoption.'))
selected_manifest=AUDIT/'selected_visual_review_v1/manifest.json';assert selected_manifest.exists()
adoption=read(OUT/'adoption.json');adoption.update(selected_visual_review=dict(manifest=str(selected_manifest),sha256=sha(selected_manifest),
    raw_selected_frames_and_observed_trace_visually_checked=True),decision_comparison='decision_comparison.json',
    additional_live_measurement_planned=False)
(OUT/'adoption.json').write_text(json.dumps(adoption,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
with (OUT/'README.md').open('a',encoding='utf-8') as file:file.write('''

## 動画の実画像確認と全候補比較

採用2本についても赤前・黄後の実フレーム、入口・旋回・出口・停止のコマ画像、動画上に重ねた実測中点軌跡を確認した。推定点や予定軌跡は実観測として描いていない。[採用2本の画像・8倍スロー資料](../../logs/pixel_camera/turn1800_A_outr45_20261120_v1/firmware_outR45_adoption_20261121_v1/selected_visual_review_v1/)へ保存している。

直近の不採用17.0/17.4mmは2本とも約46.7度だった。盤面固定の見かけ中点も別出力で確認し、最新動画では予定入口から約3.6度、予定出口から約5.3度の傾きがあった。入口に合わせた相対残差と盤面固定の位置/方向誤差は別であり、後者を合格とはしていない。[全12本の精密比較と採用区分](decision_comparison.json)を参照。別の1本は保持1ms欠損で精密比較から除外した。
''')

entry=f'''## 2026-11-21のパラメータ決定・ソース反映

「そろそろパラメータけっていできない?」により、追加の単発追込みを止め、実測済みの補正-1.7度・Lstart15.4 / Lend15.9 mm・半径-55 mmを機体Aの右45度出口1800へ暫定採用した。採用2本の出口角45.430 / 44.708度、位置残差3.692 / 2.018 mm。位置2mm目標は未達のまま、既知の残差を明示した実用上の暫定決定である。元の測定品質・native失敗判定は変更していない。

[採用根拠・設定・ログ・検証](../../../turn_analysis/20261121_A_turn1800_OUTR45_adoption/README.md)へ保存。右だけ旧撮影方式フラグから独立させ、論理角-45度・物理指令角-46.7度とした。左・B・他ターン・PID/FF参照は変更前と同じことを照合。Debug-MouseAのConfigure/Build成功、参照共有ホストテスト1件成功・スキップ0、ARM ELF内の値・補正・2つの1800モード参照も一致した。本体Flash書き込みは未実施。追加走行の予定はない。

13本目の同じ17.0 / 17.4 mm再測定は出口角46.636度、Lend残差右+2.808 / 前+0.791 mm、距離{rows[-1]['position_error_mm']:.3f} mmで未採用。前回46.713度と同様の回り過ぎが再現し、実機設定は一致した。保存1ms193点・drop0、連続CSV欠落0・CRC0、操作者は停止・接触なし・両方OFFを確認。B側停止・A優先の指示を引き継ぎ、今回は撮影競合なしで1回実行・BLE切断済み。

今回、[実画像の軌跡](repeat_lengths_cminus1p7_Ls17p0_Le17p4_20261121_03/visual_trajectory_review_v1/run2_observed_trace.png)と[盤面固定比較](repeat_lengths_cminus1p7_Ls17p0_Le17p4_20261121_03/visual_trajectory_review_v1/board_fixed_apparent_midpoint.png)を確認した。最新動画の入口は予定より3.630度、出口は5.266度傾いている。盤面上の見かけの中点Lendは約123.34 / 213.34 mm、予定135 / 225 mmと異なる。絶対車軸・高さ補正の絶対値は未校正であり、入口に合わせた相対2.92mmを盤面固定誤差とは扱わない。

全フレーム追跡と軌跡図の確認に加え、実画像のコマ・中点重ね画像・スロー資料も保存した。動画全体のリアルタイム目視再生を実施したとは主張しない。採用は入口基準の単体形状で、盤面固定設置・左・連続/最短接続・新EKF係数の採用は未確認。[状態v15](session_outr45_status_20261121_v15.json)を参照。

'''
path=BASE/'README.md';old=path.read_text(encoding='utf-8');heading,tail=old.split('\n',1)
path.write_text(heading+'\n\n'+entry+tail.lstrip('\n'),encoding='utf-8')
save(BASE/'session_outr45_status_20261121_v15.json',dict(recorded_at_utc=datetime.now(timezone.utc).isoformat(),
    state='PROVISIONAL_PARAMETERS_DECIDED_SOURCE_BUILD_ELF_VERIFIED_NO_PENDING_RUN_OFF',
    mouse='A',motion='out_r45',speed_mm_s=1800,completed_physical_runs=13,precise_1ms_complete_runs=12,
    latest_trial=str(RUN/'trial.json'),latest_recording_id=trial['recording_id'],latest_video_exit_deg=rows[-1]['angle_deg'],
    latest_position_error_mm=rows[-1]['position_error_mm'],latest_native_auto_stop_confirmed=summary['auto_stop_confirmed'],
    selected_parameters=adoption['parameters'],selected_adoption_record=str(OUT/'adoption.json'),
    selected_position_2mm_goal_met=False,selected_status='PROVISIONAL_WITH_KNOWN_RESIDUAL',
    firmware_source_changed=True,Debug_MouseA_build_passed=True,host_test_passed=1,host_test_skipped=0,
    ELF_values_and_references_verified=True,firmware_written=False,new_ekf_parameters_adopted=False,
    both_powered_off=True,contact_detected=False,ble_disconnected=True,B_capture_stop_instruction_persists=True,
    execution_resend_count=0,next_trial_id=None,additional_live_measurement_planned=False))
print(json.dumps(dict(final_measurement_angle_deg=rows[-1]['angle_deg'],final_measurement_position_error_mm=rows[-1]['position_error_mm'],
    adopted=adoption['parameters'],source_and_ELF_verified=True,no_more_runs_planned=True)),flush=True)
