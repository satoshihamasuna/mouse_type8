"""出口補正+1.0度の初回結果を保存し、長さ5.3/31.0mm候補へつなぐ。機器操作なし。"""
from pathlib import Path
from datetime import datetime, timezone
import json
import math

root=Path(__file__).resolve().parents[1]
base=root/'tools/logs/pixel_camera/turn1600_B_outr135_20261120_v1'
stage=base/'angle_c1p0_Ls6_Le27p9_01'
identifier='B_OUTR135_1600_c1p0_Ls6_Le27p9_green_20261120_001'
run=stage/'local_execution'/('run_'+identifier)
next_stage=base/'geometry_c1p0_Ls5p3_Le31_01'
next_id='B_OUTR135_1600_c1p0_Ls5p3_Le31_green_20261120_001'
read=lambda path:json.loads(path.read_text(encoding='utf-8-sig'))
def save(path,value):
    with path.open('x',encoding='utf-8') as stream:
        json.dump(value,stream,ensure_ascii=False,indent=2,allow_nan=False)
        stream.write('\n')
geometry=read(run/'midpoint_raw_wheel_clock_v1/midpoint_review.json')
report=read(run/'full/report.json')
tracking=read(run/'tracking_1ms_review_v3/verification.json')
trial=read(run/'trial.json')
operator=read(stage/'local_execution/operator_results'/(identifier+'.json'))
candidate=read(next_stage/'candidate.settings.json')
actual=read(run/'saved_1ms.settings.json')
assert geometry['independent_optical_geometry_eligible']
assert operator['both_powered_off'] and operator['contact_detected'] is False
assert trial['recording_stop']['log_stop_evidence']['confirmed']
assert trial['log_quality']['lost_frames']==trial['log_quality']['crc_errors_during_trial']==0
for key in ('mouse','motion_name','preset_speed_mm_s','pid','feedforward','suction','turn_count'):
    assert candidate[key]==actual[key]
assert {key:value for key,value in candidate['parameters'].items() if key not in ('lstart','lend')}=={key:value for key,value in actual['parameters'].items() if key not in ('lstart','lend')}
assert candidate['parameters']['lstart']==5.3 and candidate['parameters']['lend']==31.
assert not (next_stage/'local_execution/jobs'/(next_id+'.json')).exists()
samples=read(run/'marker_visual_check_v1/samples.json');assert len(samples['samples'])==16
save(run/'marker_visual_check_v1/visual_review.json',dict(state='SAMPLED_ACTUAL_FRONT_REAR_PAIRS_CONFIRMED',
    inspected_image='actual_marker_pairs.jpg',samples=16,front_colour='green',rear_colour='red',
    observation='入口・旋回・出口の16画像で前端緑と後端赤の実マーカーへの重なりを確認。背景格子/基板の誤認は見られない。',
    full_frame_quality_report=str(run/'full/report.json'),acceptance_thresholds_unchanged=True,
    profile_changed_after_capture=False,scope='抽出16画像の視認と全838フレームの観測品質。全画像の人手確認ではない。'))
error=math.dist(geometry['midpoint_Lend_right_forward_mm'],geometry['target_midpoint_Lend_right_forward_mm'])
result=dict(recorded_at=datetime.now(timezone.utc).isoformat(),mouse='B',motion='out_r135',preset_mm_s=1600,
    source_trial=str(run/'trial.json'),operator_result=operator,geometry=geometry,tracking=tracking,
    position_error_mm=error,previous_two_run_review=str(base/'lengths_same_candidate_two_review_v1/review.json'),
    original_summary_preserved=read(run/'full/summary.json'),capture_stop_evidence=trial['recording_stop']['log_stop_evidence'],
    next_parameters=candidate['parameters'],next_stage=str(next_stage),next_request_id=next_id,next_request_sent=False,
    power_state='BOTH_OFF_USER_CONFIRMED',contact_detected=False,adopted=False,source_params_changed=False,firmware_write_performed=False,
    rationale='補正+1.0度の出口角134.749度で不足0.251度。Lend右93.291/前161.927mmに対し目標95.459/159.099mm。測定出口方向を保つ解Lstart5.321/Lend30.953mmを5.3/31.0mmへ丸め、角度補正と半径を固定して比較。',
    limitations=['同じ角度候補は成功1本。次の長さ変更後に角度・位置・反復を確認する。',
        '入口相対の高さ補正マーカー中点。固定盤面の出口/停止、絶対車軸/機軸、BのEKF係数、連続接続は未確認。'])
save(base/'c1p0_001_review_next_lengths5p3_31.json',result)
phase=read(stage/'phase.json')
phase.update(state='CAPTURED_RELATIVE_REVIEWED_BOTH_OFF',request_id=identifier,next_candidate_executed=True,
    execution_pending=False,successful_run_count=1,review=str(base/'c1p0_001_review_next_lengths5p3_31.json'))
(stage/'phase.json').write_text(json.dumps(phase,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
for number in (2,3):
    unused=base/f'angle_c1p0_Ls6_Le27p9_{number:02d}'
    old=read(unused/'phase.json')
    assert not list((unused/'local_execution/jobs').glob('*.json'))
    save(unused/'superseded_by_lengths5p3_31.json',dict(state='SUPERSEDED_NOT_EXECUTED',
        original_phase=old,new_stage=str(next_stage),basis=str(base/'c1p0_001_review_next_lengths5p3_31.json'),
        original_files_preserved=True))
state=read(base/'resume_state.json')
state.update(state='C1P0_SUCCESS01_REVIEWED_LENGTHS5P3_31_PREPARED_BOTH_OFF',power_state='BOTH_OFF_USER_CONFIRMED',
    operator_ready=False,ble_connected=False,execution_pending=False,current_placement_confirmed=False,
    current_hands_clear_confirmed=False,contact_detected=False,last_request_id=identifier,last_run=str(run),last_error=None,
    last_execution_write_attempted=True,execution_commands_sent=1,relative_exit_rotation_deg=geometry['exit_path_direction_change_deg'],
    video_required_observation_fraction=report['required_motion_window_quality']['observation_rate'],
    video_registration_fraction=report['required_motion_window_quality']['calibration_rate'],
    candidate=str(next_stage/'candidate.settings.json'),config=str(next_stage/'config.json'),
    next_stage=str(next_stage),next_request_id=next_id,next_request_sent=False,
    next_action='同じ出口180/45mm・緑前右下45度へ戻し、今回の干渉なし/両方ON/手離し確認後、補正+1.0度・長さ5.3/31.0mmを新IDで1回比較。')
(base/'resume_state.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
with (base/'README.md').open('a',encoding='utf-8') as stream:
    stream.write(f'\n補正+1.0度・長さ6.0/27.9mmの初回は134.749度、Lend右93.291/前161.927mm、目標との差{error:.3f}mm。必要区間観測98.750%、各フレームArUco100%、全838中836実ペア。抽出16画像で実マーカーを確認。1msログ277点/drop0、走行中欠落/CRC0、追従RMS3.150rad/s・飽和0%、最低7.328V。両方OFF・接触なし。\n\n測定した出口方向を保つ長さ解5.321/30.953mmから次はLstart5.3/Lend31.0mm、補正+1.0度・半径−41mm維持を比較。3stageをオフライン検証、今回の操作者確認前は未送信。旧6.0/27.9mmの補正1.0度反復stage02/03は未実行のまま保管。次の長さ変更後に角度と再現性を確認する。出口値のソース反映/書込みは未実施。詳細c1p0_001_review_next_lengths5p3_31.json。\n')
print(json.dumps(dict(angle_deg=geometry['exit_path_direction_change_deg'],position_error_mm=error,
    next_parameters=candidate['parameters'],next_request_sent=False),ensure_ascii=False))
