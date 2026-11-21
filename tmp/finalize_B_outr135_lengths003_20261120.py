"""出口同一候補2本目の保存結果と次の角度候補を対応付ける。機器操作なし。"""
from pathlib import Path
from datetime import datetime, timezone
import json

root=Path(__file__).resolve().parents[1]
base=root/'tools/logs/pixel_camera/turn1600_B_outr135_20261120_v1'
stage=base/'lengths_Ls6_Le27p9_c1p9_03'
identifier='B_OUTR135_1600_Ls6_Le27p9_c1p9_green_20261120_003'
run=stage/'local_execution'/('run_'+identifier)
next_stage=base/'angle_c1p0_Ls6_Le27p9_01'
next_id='B_OUTR135_1600_c1p0_Ls6_Le27p9_green_20261120_001'
read=lambda path:json.loads(path.read_text(encoding='utf-8-sig'))
def save(path,value):
    with path.open('x',encoding='utf-8') as stream:
        json.dump(value,stream,ensure_ascii=False,indent=2,allow_nan=False)
        stream.write('\n')

comparison=read(base/'lengths_same_candidate_two_review_v1/review.json')
geometry=read(run/'midpoint_raw_wheel_clock_v1/midpoint_review.json')
report=read(run/'full/report.json')
tracking=read(run/'tracking_1ms_review_v3/verification.json')
trial=read(run/'trial.json')
operator=read(stage/'local_execution/operator_results'/(identifier+'.json'))
actual=read(run/'saved_1ms.settings.json')
candidate=read(next_stage/'candidate.settings.json')
assert geometry['independent_optical_geometry_eligible']
assert trial['recording_stop']['log_stop_evidence']['confirmed']
assert trial['log_quality']['lost_frames']==trial['log_quality']['crc_errors_during_trial']==0
assert candidate['parameters']['correction']==1.0
for key in ('mouse','motion_name','preset_speed_mm_s','pid','feedforward','suction','turn_count'):
    assert candidate[key]==actual[key]
assert {key:value for key,value in candidate['parameters'].items() if key!='correction'}=={key:value for key,value in actual['parameters'].items() if key!='correction'}
assert not (next_stage/'local_execution/jobs'/(next_id+'.json')).exists()
samples=read(run/'marker_visual_check_v1/samples.json')
assert len(samples['samples'])==16
save(run/'marker_visual_check_v1/visual_review.json',dict(state='SAMPLED_ACTUAL_FRONT_REAR_PAIRS_CONFIRMED',
    inspected_image='actual_marker_pairs.jpg',samples=16,front_colour='green',rear_colour='red',
    observation='入口・旋回・出口の16画像で前端緑と後端赤の実マーカーへの重なりを確認。背景格子/基板の誤認は見られない。',
    full_frame_quality_report=str(run/'full/report.json'),acceptance_thresholds_unchanged=True,
    profile_changed_after_capture=False,scope='抽出16画像の視認と全899フレームの観測品質。全画像の人手確認ではない。'))
decision=dict(recorded_at=datetime.now(timezone.utc).isoformat(),mouse='B',motion='out_r135',preset_mm_s=1600,
    source_trial=str(run/'trial.json'),comparison_review=str(base/'lengths_same_candidate_two_review_v1/review.json'),
    operator_result=operator,power_state='BOTH_OFF_USER_CONFIRMED',contact_detected=False,
    geometry=geometry,tracking=tracking,original_summary_preserved=read(run/'full/summary.json'),
    capture_stop_evidence=trial['recording_stop']['log_stop_evidence'],
    mean_angle_deg=comparison['mean_exit_angle_deg'],between_run_angle_range_deg=comparison['exit_angle_range_deg'],
    next_parameters=candidate['parameters'],next_stage=str(next_stage),next_request_id=next_id,
    next_request_sent=False,adopted=False,source_params_changed=False,firmware_write_performed=False,
    rationale='同じ6.0/27.9mm・補正1.9度の2本は133.667/134.544度、平均134.106度。両方不足のため、まず補正だけ0.9度減らした+1.0度を比較。長さは角度変更後に再評価する。',
    correction_sign_evidence=dict(source='Task/Src/init_motion.cpp',expression='motion_degree = degree + degree_correction',
        measured_target_integral_deg=tracking['target_integral_deg'],current_logical_degree=-135.,
        current_motion_degree=-133.1,next_motion_degree=-134.,
        prediction_only='実角の変化が補正差に等しいとは保証せず、独立走行で検証する。'),
    limitations=['入口相対の高さ補正マーカー中点。固定盤面の出口/停止、絶対車軸・機軸、BのEKF係数、連続接続は未確認。',
        '今回の2本間に0.877度の差がある。+1.0度は未実行候補で、採用値ではない。'])
save(base/'lengths003_review_next_c1p0.json',decision)
phase=read(stage/'phase.json')
phase.update(state='CAPTURED_RELATIVE_REVIEWED_BOTH_OFF',request_id=identifier,
    next_candidate_executed=True,execution_pending=False,successful_run_count=1,
    review=str(base/'lengths003_review_next_c1p0.json'),operator_power_state='BOTH_OFF_USER_CONFIRMED')
(stage/'phase.json').write_text(json.dumps(phase,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
state=read(base/'resume_state.json')
state.update(state='LENGTHS6_27P9_SUCCESS02_REVIEWED_C1P0_PREPARED_BOTH_OFF',
    power_state='BOTH_OFF_USER_CONFIRMED',operator_ready=False,ble_connected=False,execution_pending=False,
    current_placement_confirmed=False,current_hands_clear_confirmed=False,contact_detected=False,
    last_request_id=identifier,last_run=str(run),last_error=None,
    last_execution_write_attempted=True,execution_commands_sent=1,
    relative_exit_rotation_deg=geometry['exit_path_direction_change_deg'],
    video_required_observation_fraction=report['required_motion_window_quality']['observation_rate'],
    video_registration_fraction=report['required_motion_window_quality']['calibration_rate'],
    candidate=str(next_stage/'candidate.settings.json'),config=str(next_stage/'config.json'),
    next_stage=str(next_stage),next_request_id=next_id,next_request_sent=False,
    next_action='同じ出口180/45mm・緑前右下45度へ戻し、今回の干渉なし/両方ON/手離し確認後、補正+1.0度だけ変えた候補を新IDで1回比較。')
(base/'resume_state.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
note='\n長さ6.0/27.9mm・補正1.9度の2本目（要求003）は134.544度、Lend右94.098/前160.667mm、目標との差2.077mm。必要区間観測99.375%、各フレームArUco100%、全899中898実ペア。抽出16画像で実前後マーカーを確認。1msログ277点/drop0、走行中欠落/CRC0、追従RMS3.240rad/s・飽和0%、最低7.402V、両方OFF・接触なし。\n\n同一候補2本の平均角134.106度、間の差0.877度。2本とも不足のため、長さ6.0/27.9mmを維持して角度補正+1.9→+1.0度だけ変えた未実行候補を3stage準備。符号はdegree+degree_correctionと実機目標積分133.116度を照合。次回の実角・長さを測って判断し、出口値のソース反映/書込みは未実施。詳細lengths_same_candidate_two_review_v1/README.mdとlengths003_review_next_c1p0.json。\n'
with (base/'README.md').open('a',encoding='utf-8') as stream:stream.write(note)
with (base/'lengths_same_candidate_two_review_v1/README.md').open('a',encoding='utf-8') as stream:
    stream.write('\n次は角度補正のみ+1.9→+1.0度の比較。符号はdegree+degree_correction、実角変化は未検証。長さは6.0/27.9mmを維持し角度変更後に評価する。angle_c1p0_Ls6_Le27p9_01〜03はオフライン検証済み、操作者の今回の配置/ON/手離し確認前は未送信。\n')
print(json.dumps(dict(current_angle_deg=geometry['exit_path_direction_change_deg'],next_parameters=candidate['parameters'],next_request_sent=False),ensure_ascii=False))
