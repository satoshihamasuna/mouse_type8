"""完全取得したR180再開試行と、次の同一電池反復の条件を整理する。実機操作なし。"""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json
import shutil

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tools/logs/pixel_camera/turn1800_A_r180_20261120_v1'
STAGE = BASE / 'continuation_c4p25_r48_02'
IDENTIFIER = 'A_R180_1800_continuation_c4p25_r48_20261120_002'
RUN = STAGE / 'local_execution' / ('run_' + IDENTIFIER)
NEXT = BASE / 'repeat_verified_c4p25_r48_01'
read = lambda path: json.loads(path.read_text(encoding='utf-8-sig'))
trial, settings = read(RUN / 'trial.json'), read(RUN / 'saved_1ms.settings.json')
review = read(STAGE / 'entry_clock_midpoint_review_v1/report.json')
tracking = read(RUN / 'tracking_1ms_review_v1/verification.json')
operator_path = STAGE / 'local_execution/operator_results' / (IDENTIFIER + '.json')
operator = read(operator_path)
assert trial['state'] == 'CAPTURED_REVIEW_REQUIRED' and trial['saved_log']['status'] == 'saved'
assert trial['saved_log']['drop_count'] == 0 and trial['execution_resend_count'] == 0
assert review['relative_geometry_eligible'] and all(review['checks'].values())
assert operator['both_powered_off'] and operator['contact_detected'] is False
next_settings = read(NEXT / 'candidate.settings.json')
keys = ['mouse', 'motion_name', 'preset_speed_mm_s', 'parameters', 'pid', 'feedforward', 'suction', 'turn_count']
assert all(settings[key] == next_settings[key] for key in keys)
exports = RUN / 'review_exports'
exports.mkdir(exist_ok=False)
for source, name in ((STAGE / 'entry_clock_midpoint_review_v1', 'midpoint'),
                     (STAGE / 'front_phase_review_v1', 'front')):
    shutil.copytree(source, exports / name)
shutil.copyfile(operator_path, exports / 'operator_result.json')
source_settings = BASE / 'angle_c4p25_03/local_execution/run_A_R180_1800_angle_c4p25_r48_Le31p5_20261120_003/saved_1ms.settings.json'
assert all(settings[key] == read(source_settings)[key] for key in keys)
result = dict(recorded_at_utc=datetime.now(timezone.utc).isoformat(), mouse='A', motion='long_r180',
    actual_settings_unchanged=True, parameters=settings['parameters'],
    physical_executions_this_attempt=1, execution_resend_count=0,
    saved_1ms_frames=trial['saved_log']['frame_count'], saved_1ms_drops=0,
    both_powered_off=True, contact_detected=False,
    relative_review=review, tracking=tracking,
    standard_summary_state=read(RUN / 'full/summary.json'), standard_original_failures_preserved=True,
    previous_ble_disconnect_operator_report='Operator stated power was switched OFF before save-complete notice.',
    previous_ble_disconnect_consistent_with_operator_power_off=True,
    firmware_changes=False, parameter_adoption_allowed=False,
    next_test='Same battery, same settings, one independent repeat. Fresh readiness required.',
    next_stage=str(NEXT), next_operator_ready=False, next_execution_pending=False,
    ekf_identification='Not adopted: repeatability unresolved; absolute marker/body yaw and camera foot remain uncalibrated.',
    source_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in
        [RUN / 'trial.json', RUN / 'saved_1ms.settings.json', RUN / 'saved_1ms.csv',
         STAGE / 'entry_clock_midpoint_review_v1/report.json', operator_path]})
with (STAGE / 'review_summary.json').open('x', encoding='utf-8') as output:
    json.dump(result, output, ensure_ascii=False, indent=2, allow_nan=False)
    output.write('\n')
text = f'''# 機体A・1800 R180：同一設定の再開取得

半径48mm、補正＋4.25°、Lstart7.52mm、Lend31.5mm。PID/FF、吸引650、前後走行と撮影条件を維持。
設定は12:13の完全取得試行と全項目一致。今回も実行指令は1回だけで再送なし。
1msログ{trial['saved_log']['frame_count']}件を欠落0で保存。動画の撮影検査もPASSED、BLE切断後に操作者が接触なし・停止・両方OFFを確認。

入口直線の生エンコーダ距離だけで時刻を照合した相対マーカー中点は、
返り幅（前45mm断面）{review['return_spacing_at_forward45_mm']:.2f}mm、返り角{review['path_angle_deg']:.2f}deg、
Lendは(右{review['lend_right_forward_mm'][0]:.2f},前{review['lend_right_forward_mm'][1]:.2f})mm。
名目Lendは(90,90)mm。必要窓の観測率{100 * review['required_observation_rate']:.2f}%、純走行窓{100 * review['motion_observation_rate']:.2f}%。
各使用frameの実ArUco登録と95%条件を維持し、欠測は除外する。
入口時計RMSは{review['timing']['incoming_rms_mm']:.3f}mm、半窓の差{review['timing']['half_window_spread_ms']:.3f}ms。
静止窓の散らばりは{review['stationary_scatter_rms_mm']:.3f}mm RMS。相対観測・時計・静止の個別検査を通過。

標準ツールの全軌跡モデル合わせはRMSE{review['original_trajectory_fit_rmse_mm']:.2f}mmで、初期直進不足のsummary不合格を保持する。
独立入口時計の診断と区別し、元判定を書き換えない。絶対車軸位置・body yaw・盤面固定接続・最短走行の合格ではない。

1msログの旋回追従RMSは{tracking['nominal_1ms_tracking_metrics']['tracking_rms_rad_s']:.3f}rad/s、出力飽和0%。
実測周期は{tracking['measured_period_ms']:.6f}ms、最低電圧は{tracking['battery_min_v']:.2f}V。
以前の同一設定の完全取得79.23mm/182.44degとは差があり、再現性は未確定。
直前の不完全取得95.81mm/178.61degは参考値のまま保持。

次は今回と同じ電池・同じ設定でもう1本確認する準備のみを実施。現在の配置・ON・手離し確認は未取得で、未実行。
半径54mm案と補正3.5deg戻し案は保留し、ファームウェアおよび採用値は変更していない。
EKF係数も、再現性と絶対機軸yaw/カメラ真下が未確定のため今回採用していない。

- [中点の相対評価](review_exports/midpoint/report.json)
- [軌跡図](review_exports/midpoint/relative_trajectory.png)
- [赤前点の照合](review_exports/front/front_reference.json)
- [1msログ追従](tracking_1ms_review_v1/verification.json)
- [元の取得記録](trial.json)
'''
with (RUN / 'continuation_review_README.md').open('x', encoding='utf-8') as output:
    output.write(text)
previous = BASE / 'continuation_c4p25_r48_01/phase.json'
phase = read(previous)
phase.update(state='FAILED_STATIC_HEADING_NO_MOTION', request_consumed=True, operator_ready=False,
    execution_pending=False, execution_write_attempted=False, power_state='Both_OFF_confirmed')
previous.write_text(json.dumps(phase, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
for path in (STAGE / 'phase.json', BASE / 'preparation.json'):
    phase = read(path)
    phase.update(phase='FULL_CAPTURE_SAVED_SAME_BATTERY_REPEAT_PREPARED',
        state='FULL_CAPTURE_RELATIVE_REVIEW_NOT_ADOPTED', operator_ready=False, execution_pending=False,
        power_state='Both_OFF_confirmed', ble_disconnected=True, operator_contact=False,
        current_placement_confirmed=False, current_clearance_confirmed=False,
        current_hands_clear_confirmed=False, current_physical_clearance_verified=False,
        source_params_changed=False, parameter_adoption_allowed=False, next_candidate_executed=False,
        last_completed_trial=str(RUN / 'trial.json'), review_summary=str(STAGE / 'review_summary.json'),
        next_candidate=str(NEXT / 'candidate.settings.json'), candidate=str(NEXT / 'candidate.settings.json'),
        config=str(NEXT / 'config.json'), next_request_id='A_R180_1800_repeat_verified_c4p25_r48_20261120_001')
    path.write_text(json.dumps(phase, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
print(json.dumps(dict(width_mm=review['return_spacing_at_forward45_mm'], angle_deg=review['path_angle_deg'],
    frame_count=trial['saved_log']['frame_count'], next_stage=str(NEXT), hardware_commands_sent=0), ensure_ascii=False))
