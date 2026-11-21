"""変更なし反復の実行事実と、不完全取得の限界を保存する。追加走行なし。"""
from pathlib import Path
from datetime import datetime, timezone
import json
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tools/logs/pixel_camera/turn1800_A_r180_20261120_v1'
STAGE = BASE / 'unchanged_c4p25_repeat_01'
IDENTIFIER = 'A_R180_1800_unchanged_c4p25_repeat_20261120_001'
RUN = STAGE / 'local_execution' / ('run_' + IDENTIFIER)
DIAG = RUN / 'partial_capture_geometry_diagnostic_v1'
read = lambda path: json.loads(path.read_text(encoding='utf-8-sig'))
trial, diagnostic = read(RUN / 'trial.json'), read(DIAG / 'report.json')
operator = read(STAGE / 'local_execution/operator_results' / (IDENTIFIER + '.json'))
assert operator['both_powered_off'] and operator['contact_detected'] is False
assert trial['execution_write_attempted'] and trial['execution_resend_count'] == 0
assert diagnostic['actual_settings_identical'] and not diagnostic['saved_1ms_available']
assert not diagnostic['diagnostic_geometry_quality_passed']
curve = pd.read_csv(DIAG / 'relative_trajectory.csv')
t = curve.time_video_s.to_numpy()
xy = curve[['right_mm', 'forward_mm']].to_numpy()
valid = curve.detected.to_numpy(bool)
sensitivity = []
for shift_ms in (-5, 0, 5):
    window = np.asarray(diagnostic['exit_window_s']) + shift_ms / 1000
    points = xy[valid & (t >= window[0]) & (t <= window[1])]
    direction = np.linalg.svd(points - points.mean(0), full_matrices=False)[2][0]
    if direction @ (points[-1] - points[0]) < 0:
        direction = -direction
    centre = points.mean(0)
    width = float(centre[0] + (45 - centre[1]) * direction[0] / direction[1])
    angle = float(np.degrees(np.arctan2(direction[0], direction[1])) % 360)
    sensitivity.append(dict(exit_window_shift_ms=shift_ms, points=len(points),
        width_at_forward45_mm=width, angle_deg=angle))
with (DIAG / 'exit_phase_sensitivity.json').open('x', encoding='utf-8') as output:
    json.dump(dict(scope='5ms位相の出口窓感度。静止判定不合格・1ms欠落は再分類しない。',
        results=sensitivity, parameter_adoption_allowed=False), output, ensure_ascii=False, indent=2)
result = dict(recorded_at_utc=datetime.now(timezone.utc).isoformat(), mouse='A', motion='long_r180',
    preset_mm_s=1800, user_request='直前の実走と設定を変更せずにもう1回',
    physical_execution_count=1, execution_resend_count=0, settings_identical=True,
    parameters=diagnostic['parameters'], both_powered_off_confirmed=True, contact_detected=False,
    trial_capture_state=trial['state'], camera_capture_state=trial['camera_state'],
    saved_1ms_available=False, saved_5ms_full_motion=True, saved_5ms_lost_frames=0,
    reference_only_geometry=dict(width45_mm=diagnostic['return_spacing_at_forward45_mm'],
        angle_deg=diagnostic['path_angle_deg'], previous_width45_mm=diagnostic['previous_width45_mm'],
        previous_angle_deg=diagnostic['previous_path_angle_deg']),
    minimum_motion_battery_v=diagnostic['minimum_motion_battery_v'],
    previous_minimum_battery_v=diagnostic['prior_minimum_battery_v'],
    video_stationary_quality_passed=False, stationary_scatter_rms_mm=diagnostic['stationary_scatter_rms_mm'],
    diagnostic_quality_passed=False, repeated_shape_accepted=False, cause_identified=False,
    parameter_adoption_allowed=False, source_params_changed=False, next_execution_pending=False,
    limitations=diagnostic['limitations'], diagnostic_report=str(DIAG / 'report.json'))
with (RUN / 'unchanged_repeat_result.json').open('x', encoding='utf-8') as output:
    json.dump(result, output, ensure_ascii=False, indent=2, allow_nan=False)
    output.write('\n')
text = f'''# 機体A・1800 R180の変更なし反復

直前の実走と同じ設定で物理走行を1回だけ実施。実機の読返しで半径48mm、補正＋4.25°、Lstart7.52mm、Lend31.5mm、PID/FF・吸引650・回数1の一致を確認。
停止・接触なし・機体/HY0020両方OFFを操作者が確認。ファームウェアおよび採用パラメータは変更していない。

走行後のBLE切断により1msログ取得に失敗した。元のtrial.jsonのFAILEDを保持し、指令を再送していない。
動画の撮影検査はPASSED、5ms配信は走行区間を保持しsequence連続・最大間隔6ms・欠落0。1ms点や生パルスを補完していない。

| マーカー中点の相対診断 | 前の走行 | 今回・参考 |
| --- | ---: | ---: |
| 前45mm断面の返り幅 [mm] | {diagnostic['previous_width45_mm']:.2f} | {diagnostic['return_spacing_at_forward45_mm']:.2f} |
| 返り角 [deg] | {diagnostic['previous_path_angle_deg']:.2f} | {diagnostic['path_angle_deg']:.2f} |
| 走行ログの最低電圧 [V] | {diagnostic['prior_minimum_battery_v']:.2f} | {diagnostic['minimum_motion_battery_v']:.2f} |

今回の時計は5ms配信の20点平均速度と入口直線だけで照合し、旋回・退出や尺度を合わせていない。
前回の1ms生パルス時計と条件が異なる。位相境界の標本化差は約±2.5msで、精密なLend同定は行わない。
実ArUco2/3を参照した必要窓のマーカー観測率は{100 * diagnostic['required_observation_rate']:.2f}%、純走行窓は{100 * diagnostic['motion_observation_rate']:.2f}%。入口/退出の直線・時計は既存の診断基準内。
ただし所定の停止窓の中点散らばりは{diagnostic['stationary_scatter_rms_mm']:.2f}mm RMSで、1mmの基準に不合格。
この不合格を保持し、今回の数値は参考診断に限定する。再現性の合格・採用・盤面固定接続の合格ではない。
電圧差を記録したが、今回の応答差やBLE切断の物理原因は確定していない。

- [比較図](partial_capture_geometry_diagnostic_v1/comparison.png)
- [診断と入力ハッシュ](partial_capture_geometry_diagnostic_v1/report.json)
- [出口窓±5msの感度](partial_capture_geometry_diagnostic_v1/exit_phase_sensitivity.json)
- [元の取得記録](trial.json)
- [5ms走行ログ](motion.csv)
- [実機設定](motion.settings.json)
'''
with (RUN / 'unchanged_repeat_README.md').open('x', encoding='utf-8') as output:
    output.write(text)
for path in (STAGE / 'phase.json', BASE / 'preparation.json'):
    phase = read(path)
    phase.update(state='UNCHANGED_REPEAT_COMPLETED_REFERENCE_ONLY',
        phase='UNCHANGED_REPEAT_COMPLETED_REFERENCE_ONLY', operator_ready=False,
        execution_pending=False, next_execution_pending=False, power_state='Both_OFF_confirmed',
        operator_contact=False, source_params_changed=False, parameter_adoption_allowed=False,
        repeat_result=str(RUN / 'unchanged_repeat_result.json'), next_request_id=None)
    path.write_text(json.dumps(phase, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
print(json.dumps(dict(result=str(RUN / 'unchanged_repeat_result.json'),
    physical_execution_count=1, settings_identical=True, diagnostic_quality=False,
    width45_mm=diagnostic['return_spacing_at_forward45_mm'], angle_deg=diagnostic['path_angle_deg'],
    exit_window_sensitivity=sensitivity, both_powered_off=True), ensure_ascii=False))
