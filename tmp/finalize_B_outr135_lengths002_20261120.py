"""B1600右135度出口の長さ候補1本目を記録し、同一候補の反復を準備する。"""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json
import math

root = Path(__file__).resolve().parents[1]
base = root/'tools/logs/pixel_camera/turn1600_B_outr135_20261120_v1'
stage = base/'lengths_Ls6_Le27p9_c1p9_02'
identifier = 'B_OUTR135_1600_Ls6_Le27p9_c1p9_green_20261120_002'
run = stage/'local_execution'/('run_'+identifier)
read = lambda path: json.loads(path.read_text(encoding='utf-8-sig'))
def save(path, value):
    with path.open('x', encoding='utf-8') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')

geometry = read(run/'midpoint_raw_wheel_clock_v1/midpoint_review.json')
tracking = read(run/'tracking_1ms_review_v3/verification.json')
report = read(run/'full/report.json')
trial = read(run/'trial.json')
operator = read(stage/'local_execution/operator_results'/(identifier+'.json'))
baseline_run = base/'baseline01/local_execution/run_B_OUTR135_1600_baseline_green_20261120_003'
baseline = read(baseline_run/'rear_red_s30_reanalysis_v1/midpoint_raw_wheel_clock_v1/midpoint_review.json')
assert geometry['independent_optical_geometry_eligible']
assert geometry['clock']['passed']
assert tracking['execution_resend_count'] == 0
assert tracking['parameters'] == read(stage/'candidate.settings.json')['parameters']
assert trial['log_quality']['lost_frames'] == trial['log_quality']['crc_errors_during_trial'] == 0
assert trial['recording_stop']['log_stop_evidence']['confirmed']
assert report['required_motion_window_quality']['observation_rate'] >= .95
assert report['required_motion_window_quality']['calibration_rate'] == 1
samples = read(run/'marker_visual_check_v1/samples.json')
assert len(samples['samples']) == 16
visual_path = run/'marker_visual_check_v1/visual_review.json'
visual = dict(
    state='SAMPLED_ACTUAL_FRONT_REAR_PAIRS_CONFIRMED', inspected_image='actual_marker_pairs.jpg',
    samples=16, front_colour='green', rear_colour='red',
    observation='入口・旋回・出口の16画像で前端緑と後端赤の実マーカーへの重なりを確認。背景格子/基板の誤認は見られない。',
    full_frame_quality_report=str(run/'full/report.json'),
    acceptance_thresholds_unchanged=True, profile_changed_after_capture=False,
    scope='抽出16画像の視認と全827フレームの観測品質。全画像の人手確認ではない。')
if visual_path.exists():
    assert read(visual_path) == visual
else:
    save(visual_path, visual)

target = geometry['target_midpoint_Lend_right_forward_mm']
def position_error(result):
    return math.dist(result['midpoint_Lend_right_forward_mm'], target)

next_stage = base/'lengths_Ls6_Le27p9_c1p9_03'
next_id = 'B_OUTR135_1600_Ls6_Le27p9_c1p9_green_20261120_003'
assert read(next_stage/'candidate.settings.json')['parameters'] == tracking['parameters']
assert not (next_stage/'local_execution/jobs'/(next_id+'.json')).exists()
result = dict(recorded_at=datetime.now(timezone.utc).isoformat(),
    mouse='B', motion='out_r135', preset_mm_s=1600, request_id=identifier,
    source_trial=str(run/'trial.json'), parameters=tracking['parameters'], operator_result=operator,
    geometry=geometry, tracking=tracking,
    comparison=dict(baseline_trial=str(baseline_run/'trial.json'),
        baseline_reanalysis=str(baseline_run/'rear_red_s30_reanalysis_v1/trial.json'),
        baseline_Lend_error_mm=position_error(baseline), current_Lend_error_mm=position_error(geometry),
        baseline_exit_angle_deg=baseline['exit_path_direction_change_deg'],
        current_exit_angle_deg=geometry['exit_path_direction_change_deg'],
        reference='各試行の実測入口を基準にした高さ補正前後マーカー中点'),
    full_frames=report['quality']['frames'], valid_frames=report['quality']['valid_frames'],
    capture_stop_evidence=trial['recording_stop']['log_stop_evidence'],
    original_summary_preserved=read(run/'full/summary.json'),
    power_state='BOTH_OFF_USER_CONFIRMED', contact_detected=False,
    proposed_next_action='長さ6.0/27.9mm・補正1.9度の同一候補をもう1本測り、角度差の再現性を確認する。',
    proposed_next_stage=str(next_stage), proposed_next_request_id=next_id,
    next_execution_sent=False, adopted=False, source_params_changed=False, firmware_write_performed=False,
    limitations=['入口相対の単体形状。固定盤面の出口/停止位置、絶対車軸/機軸、BのEKF係数、連続接続は未確定。',
        '同じ候補の成功走行は今回1本。角度133.67度に対する補正変更と長さの再計算は反復後に判断。',
        '元summaryのauto_stop_confirmed=falseは維持。取得側の連続ログ静止確認・動画の静止・操作者OFF/接触なしを別証拠として記録。'])
save(base/'lengths002_review.json', result)
phase = read(stage/'phase.json')
phase.update(state='CAPTURED_RELATIVE_REVIEWED_BOTH_OFF', request_id=identifier,
    next_candidate_executed=True, execution_pending=False, review=str(base/'lengths002_review.json'),
    successful_run_count=1, operator_power_state='BOTH_OFF_USER_CONFIRMED')
(stage/'phase.json').write_text(json.dumps(phase, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
state = read(base/'resume_state.json')
state.update(state='LENGTHS6_27P9_SUCCESS01_REVIEWED_REPEAT_PREPARED_BOTH_OFF',
    power_state='BOTH_OFF_USER_CONFIRMED', operator_ready=False, ble_connected=False,
    current_placement_confirmed=False, current_hands_clear_confirmed=False,
    operator_power_off_confirmation='両方OFF・接触なし', contact_detected=False,
    last_error=None, execution_pending=False, last_request_id=identifier, last_run=str(run),
    last_execution_write_attempted=True, execution_commands_sent=1,
    candidate=str(next_stage/'candidate.settings.json'), config=str(next_stage/'config.json'),
    relative_exit_rotation_deg=geometry['exit_path_direction_change_deg'],
    video_required_observation_fraction=report['required_motion_window_quality']['observation_rate'],
    video_registration_fraction=report['required_motion_window_quality']['calibration_rate'],
    next_stage=str(next_stage), next_request_id=next_id, next_request_sent=False,
    next_action='同じ出口180/45mm・緑前右下45度へ戻し、今回の干渉なし/両方ON/手離し確認後、未送信の新ID003で同一候補を1回反復。')
(base/'resume_state.json').write_text(json.dumps(state, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
with (base/'README.md').open('a', encoding='utf-8') as stream:
    stream.write('\n長さ候補要求001は再び実機照会中にBLE切断、走行指令0件。OFF→ON・配置維持の操作者確認を受けた新ID002で1回だけ実行し、動画・1msログ277点/drop0を取得。両方OFF・接触なし。'
        f'必要区間観測{report["required_motion_window_quality"]["observation_rate"]*100:.3f}%、各フレームArUco100%、全{report["quality"]["frames"]}フレーム中{report["quality"]["valid_frames"]}実ペア、抽出16画像で実マーカーを確認。'
        f'入口相対Lend右{geometry["midpoint_Lend_right_forward_mm"][0]:.3f}/前{geometry["midpoint_Lend_right_forward_mm"][1]:.3f}mm、目標からの距離は{position_error(baseline):.3f}→{position_error(geometry):.3f}mmへ縮小。'
        f'出口角{geometry["exit_path_direction_change_deg"]:.3f}度（不足約1.33度）。追従RMS{tracking["tracking"]["tracking_rms_rad_s"]:.3f}rad/s・飽和0%、最低電圧{tracking["battery_min_v"]:.3f}V。'
        '角度と長さの追加変更は保留し、同一候補6.0/27.9mm・補正1.9度を新ID003で反復する準備を保持。今回の開始位置への戻し/干渉なし/ON/手離しを確認するまで未送信。詳細lengths002_review.json。\n')
print(json.dumps(dict(comparison=result['comparison'], next_request_id=next_id, next_execution_sent=False), ensure_ascii=False))
