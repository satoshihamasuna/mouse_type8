"""元の不合格を残し、検証済み色相の相対形状評価と同設定反復を記録する。"""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json
import math
import shutil
import sys
root=Path(__file__).resolve().parents[1]
base=root/'tools/logs/pixel_camera/turn1600_B_outr135_20261121_resume_v1'
stage=base/'geometry_c1p0_Ls5p3_Le31_01'
identifier='B_OUTR135_1600_c1p0_Ls5p3_Le31_green_20261121_001'
run=stage/'local_execution'/('run_'+identifier)
analysis=run.parent/(run.name+'_reanalysis_hue165_v1')
read=lambda path:json.loads(path.read_text(encoding='utf-8-sig'))
sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
def save(path,value):
    with path.open('x',encoding='utf-8') as stream:
        json.dump(value,stream,ensure_ascii=False,indent=2,allow_nan=False)
        stream.write('\n')
validation=read(base/'rear_hue165_validation_v2/sample_validation.json')
assert validation['state']=='SAMPLED_VALIDATION_PASSED_FULL_REPROCESS_REQUIRED'
profile=Path(validation['candidate'])
assert sha(profile)==sha(analysis/'body_geometry_used.json')
geometry=read(analysis/'midpoint_raw_wheel_clock_v1/midpoint_review.json')
report=read(analysis/'full/report.json');original_report=read(run/'full/report.json')
tracking=read(run/'tracking_1ms_review_v3/verification.json');trial=read(run/'trial.json')
operator=read(stage/'local_execution/operator_results'/(identifier+'.json'))
job=read(stage/'local_execution/jobs'/(identifier+'.json'))
assert job['request']['config']['address']=='E1:7F:86:21:84:73'
assert operator['both_powered_off'] and operator['contact_detected'] is False
assert geometry['independent_optical_geometry_eligible'] and geometry['clock']['passed']
assert report['required_motion_window_quality']['observation_rate']>=.95
assert report['required_motion_window_quality']['calibration_rate']==1
assert not original_report['required_motion_window_quality']['accepted_for_relative_review']
assert trial['log_quality']['lost_frames']==trial['log_quality']['crc_errors_during_trial']==0
assert trial['recording_stop']['log_stop_evidence']['confirmed'] and tracking['execution_resend_count']==0
samples=read(analysis/'marker_visual_check_v1/samples.json');assert len(samples['samples'])==16
save(analysis/'marker_visual_check_v1/visual_review.json',dict(state='SAMPLED_ACTUAL_FRONT_REAR_PAIRS_CONFIRMED',
    inspected_image='actual_marker_pairs.jpg',samples=16,observation='入口・旋回・出口の抽出16画像で実前緑/後赤・桃色のマーカーへの重なりを確認。',
    recovered_samples_visually_reviewed=11,recovered_image=str(run/'rear_hue_diagnostic_v1/rear_hue165_pairs.jpg'),
    whole_video_human_review_performed=False,acceptance_thresholds_unchanged=True))
full_validation=dict(state='CURRENT_VIDEO_QUALIFIED_FOR_RELATIVE_REVIEW',profile=str(profile),profile_sha256=sha(profile),
    original_failed_report=str(run/'full/report.json'),reanalysis_report=str(analysis/'full/report.json'),
    sample_validation=str(base/'rear_hue165_validation_v2/sample_validation.json'),
    sample_validation_v1='色相変更単独の比較。v2では独立画像でも候補の検出設定全体（赤S30/高色相165..179）を比較。',
    full_frames=report['quality']['frames'],valid_frames=report['quality']['valid_frames'],
    required_motion_observation_fraction=report['required_motion_window_quality']['observation_rate'],
    registration_fraction=1.,quality_threshold_unchanged=.95,original_profile_and_failure_preserved=True,
    scope='今回のB1600右135度出口・同じ撮影条件。次の独立走行でも実ArUco/95%/誤認/直線品質を検証する。',
    hardware_operations=0)
save(base/'rear_hue165_validation_v2/full_validation.json',full_validation)
save(analysis/'supplemental_profile_validation.json',full_validation)
sys.path.insert(0,str(root/'tools'))
from pixel_camera_tuning_mcp import McpTuningService
prepared=[]
for number in (2,3):
    pending=base/f'geometry_c1p0_Ls5p3_Le31_{number:02d}'
    assert not list((pending/'local_execution/jobs').glob('*.json'))
    config_path=pending/'config.json';config=read(config_path)
    assert config['mouse']=='B' and config['address']=='E1:7F:86:21:84:73'
    shutil.copyfile(config_path,pending/'config_before_rear_hue165.json')
    config['geometry']=str(profile)
    config_path.write_text(json.dumps(config,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    plan=read(pending/'placement_plan.json')
    payload=McpTuningService(pending/'hue165_offline_validation',{}).prepare(str(config_path),
        str(pending/'candidate.settings.json'),plan['placement'],None,20.)
    assert payload['settings']['parameters']==tracking['parameters']
    save(pending/'offline_validation_hue165.json',dict(state='VALIDATED_NOT_EXECUTED',payload=payload,
        supersedes_geometry_in='offline_validation.json',full_profile_validation=str(base/'rear_hue165_validation_v2/full_validation.json'),
        previous_geometry_profile_preserved=True,hardware_operations=0))
    prepared.append(str(pending))
target=geometry['target_midpoint_Lend_right_forward_mm']
error=math.dist(geometry['midpoint_Lend_right_forward_mm'],target)
stop_target=[112.5*math.sqrt(2),67.5*math.sqrt(2)]
stop_actual=geometry['late_stationary']['midpoint_right_forward_mm']
result=dict(recorded_at=datetime.now(timezone.utc).isoformat(),mouse='B',motion='out_r135',preset_mm_s=1600,
    request_id=identifier,physical_run_count=1,successful_same_candidate_runs=1,ble_address=job['request']['config']['address'],
    original_trial=str(run/'trial.json'),reanalysis_trial=str(analysis/'trial.json'),parameters=tracking['parameters'],
    geometry=geometry,tracking=tracking,operator_result=operator,original_report=original_report,reanalysis_report=report,
    full_profile_validation=str(base/'rear_hue165_validation_v2/full_validation.json'),
    original_failure_preserved=True,position_error_mm=error,
    relative_stop=dict(actual_right_forward_mm=stop_actual,target_right_forward_mm=stop_target,
        error_mm=math.dist(stop_actual,stop_target),measured_Lend_to_stop_mm=math.dist(geometry['midpoint_Lend_right_forward_mm'],stop_actual),
        note='入口相対マーカー中点の停止点。出口形状の評価と別に保持し、停止精度の合格とはしない。'),
    original_summary_preserved=read(analysis/'full/summary.json'),capture_stop_evidence=trial['recording_stop']['log_stop_evidence'],
    next_stage=prepared[0],next_request_id='B_OUTR135_1600_c1p0_Ls5p3_Le31_green_20261121_002',next_request_sent=False,
    next_policy='5.3/31.0mm・補正1.0度を変更せず、同一条件の反復でばらつきを確認する。赤の色相候補は検証済み165..179を次の取得で再検証。',
    power_state='BOTH_OFF_USER_CONFIRMED',contact_detected=False,adopted=False,source_params_changed=False,firmware_write_performed=False,
    limitations=['単体1本の入口相対形状。固定盤面の出口/停止、絶対車軸・機軸、BのEKF係数、連続接続は未確認。',
        '元の公式summary判定は維持。取得側連続ログ・動画静止・操作者報告を停止の別証拠として記録。',
        '色相候補の独立検証は80画像で、独立4動画すべての再解析ではない。今回829フレームは別に全解析。'])
save(base/'geometry001_hue165_review.json',result)
phase=read(stage/'phase.json')
phase.update(state='CAPTURED_REANALYSIS_RELATIVE_REVIEWED_BOTH_OFF',next_candidate_executed=True,
    execution_pending=False,successful_run_count=1,review=str(base/'geometry001_hue165_review.json'),
    original_quality_rejected=True,reanalysis_quality_passed=True)
(stage/'phase.json').write_text(json.dumps(phase,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
state=read(base/'resume_state.json')
next_stage=Path(prepared[0])
state.update(state='GEOMETRY5P3_31_SUCCESS01_HUE165_REVIEWED_REPEAT_PREPARED_BOTH_OFF',
    power_state='BOTH_OFF_USER_CONFIRMED',operator_ready=False,execution_pending=False,ble_connected=False,
    current_placement_confirmed=False,current_hands_clear_confirmed=False,contact_detected=False,
    last_run=str(run),last_request_id=identifier,reanalysis=str(analysis),original_quality_rejected=True,
    reanalysis_quality_passed=True,successful_same_candidate_runs=1,
    relative_exit_rotation_deg=geometry['exit_path_direction_change_deg'],position_error_mm=error,
    video_required_observation_fraction=report['required_motion_window_quality']['observation_rate'],video_registration_fraction=1.,
    next_stage=str(next_stage),next_request_id=result['next_request_id'],next_request_sent=False,
    candidate=str(next_stage/'candidate.settings.json'),config=str(next_stage/'config.json'),
    next_action='出口180/45mm・緑前右下45度へ戻し、今回の干渉なし/ON/手離しを確認。競合を再確認後、B用BLE E1:7F:86:21:84:73で同一候補の2本目を新IDで1回測る。')
(base/'resume_state.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
with (base/'README.md').open('a',encoding='utf-8') as stream:
    stream.write(f'\n初回20261121_001をB用BLEで1回取得。両方OFF・接触なし。1msログ280点/drop0、走行中欠落/CRC0、飽和0%、最低7.719V。元検出の必要区間91.875%不合格を保持。彩度変更は回復0件。実後マーカーの桃色化を画像で診断し、赤の高色相域下限170→165だけ変えた候補で13欠測中11画像の実前後マーカーを確認。今回56画像と独立4動画80画像で既存検出消失0、検出点変化最大1.406mm（上限1.5mm）、前緑/面積/彩度/明度/間隔/品質95%は維持。原本と別の兄弟フォルダーで全829フレーム再解析し827実ペア、必要区間98.742%、ArUco100%。\n\n入口相対出口角{geometry["exit_path_direction_change_deg"]:.3f}度、Lend右{geometry["midpoint_Lend_right_forward_mm"][0]:.3f}/前{geometry["midpoint_Lend_right_forward_mm"][1]:.3f}mm、目標との差{error:.3f}mm。単体1本なので5.3/31.0mm・補正+1.0度のまま反復する。停止点の入口相対目標差{result["relative_stop"]["error_mm"]:.3f}mmは別に記録し、停止精度の合格とはしない。未実行stage02/03だけ検証済み色相候補へ切替えオフライン再検証。現在OFF、次の操作者確認前は未送信。詳細geometry001_hue165_review.json。\n')
print(json.dumps(dict(angle_deg=geometry['exit_path_direction_change_deg'],position_error_mm=error,
    next_request_id=result['next_request_id'],next_request_sent=False),ensure_ascii=False))
