"""同一電池2本の相対再現性と、半径だけを変える次候補を保存する。実機操作なし。"""
from pathlib import Path
from datetime import datetime, timezone
import json
import shutil
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tools/logs/pixel_camera/turn1800_A_r180_20261120_v1'
STAGES = [BASE / 'continuation_c4p25_r48_02', BASE / 'repeat_verified_c4p25_r48_01']
NEXT = BASE / 'radius46p2_c4p25_01'
read = lambda p: json.loads(p.read_text(encoding='utf-8-sig'))
reviews = [read(s / 'entry_clock_midpoint_review_v1/report.json') for s in STAGES]
runs = [Path(item['source_trial']).parent for item in reviews]
settings = [read(run / 'saved_1ms.settings.json') for run in runs]
keys = ['mouse', 'motion_name', 'preset_speed_mm_s', 'parameters', 'pid', 'feedforward', 'suction', 'turn_count']
assert all(settings[0][key] == settings[1][key] for key in keys)
assert all(item['relative_geometry_eligible'] for item in reviews)
candidate = read(NEXT / 'candidate.settings.json')
assert candidate['parameters'] == dict(settings[1]['parameters'], r_min=-46.2)
assert all(candidate[key] == settings[1][key] for key in keys if key != 'parameters')
widths = [item['return_spacing_at_forward45_mm'] for item in reviews]
angles = [item['path_angle_deg'] for item in reviews]
mean_width = float(np.mean(widths))
result = dict(recorded_at_utc=datetime.now(timezone.utc).isoformat(),
    state='TWO_TRIAL_CURRENT_BATTERY_RELATIVE_COMPARISON', mouse='A', motion='long_r180',
    identical_settings=True, same_physical_battery_for_pair_operator_confirmed=True,
    battery_type_capacity_mount_same_as_1213_operator_confirmed=True,
    other_than_battery_no_tire_body_or_firmware_changes_operator_confirmed=True,
    widths_at_forward45_mm=widths, path_angles_deg=angles,
    width_span_mm=float(np.ptp(widths)), angle_span_deg=float(np.ptp(angles)),
    mean_width_mm=mean_width, mean_angle_deg=float(np.mean(angles)),
    lend_right_forward_mm=[item['lend_right_forward_mm'] for item in reviews],
    required_observation_rates=[item['required_observation_rate'] for item in reviews],
    motion_observation_rates=[item['motion_observation_rate'] for item in reviews],
    before_1213_valid_trial_difference_unexplained=True,
    old_1213_trial_not_deleted_or_reclassified=True, battery_causality_identified=False,
    sample_count=2, statistical_repeatability_guarantee=False, fixed_board_accepted=False,
    parameter_adoption_allowed=False, firmware_changed=False,
    next_candidate=str(NEXT / 'candidate.settings.json'),
    next_change=dict(r_min_before_mm=-48., r_min_after_mm=-46.2),
    predicted_width_if_proportional_mm=mean_width * 46.2 / 48,
    prediction_is_measured_causal_response=False,
    working_candidate_targets=dict(width_at_forward45_mm=90., width_tolerance_mm=2.,
        path_angle_deg=180., path_tolerance_deg=1., nominal_lend_right_forward_mm=[90., 90.],
        lend_right_tolerance_mm=2., lend_forward_tolerance_mm=3.),
    next_operator_ready=False, next_execution_pending=False,
    source_reports=[str(s / 'entry_clock_midpoint_review_v1/report.json') for s in STAGES])
with (STAGES[1] / 'paired_geometry_review.json').open('x', encoding='utf-8') as file:
    json.dump(result, file, ensure_ascii=False, indent=2, allow_nan=False)
    file.write('\n')
plt.rcParams['font.family'] = 'Yu Gothic'
fig, ax = plt.subplots(figsize=(6.5, 7.5), layout='constrained')
old = BASE / 'angle_c4p25_03/entry_clock_midpoint_review_v1'
for label, source, colour in [('12:13の試行・条件差の原因未確定', old, '.65'),
    ('今回の同電池1本目', STAGES[0] / 'entry_clock_midpoint_review_v1', 'tab:blue'),
    ('今回の同電池2本目', STAGES[1] / 'entry_clock_midpoint_review_v1', 'tab:orange')]:
    data = pd.read_csv(source / 'relative_trajectory.csv')
    xy = data[['right_mm', 'forward_mm']].to_numpy()
    xy[~data.detected.to_numpy(bool)] = np.nan
    ax.plot(xy[:, 0], xy[:, 1], label=label, color=colour)
ax.axvline(90, ls='--', color='black', lw=.8, label='名目の返り幅90mm')
ax.set(xlabel='実測入口に対する右 [mm]', ylabel='実測入口に対する前 [mm]',
    title='A / 1800 R180 / 半径48mm・補正＋4.25°', aspect='equal')
ax.grid(); ax.legend(fontsize=8)
fig.savefig(STAGES[1] / 'paired_geometry_comparison.png', dpi=140); plt.close(fig)
run = runs[1]
operator_path = STAGES[1] / 'local_execution/operator_results/A_R180_1800_repeat_verified_c4p25_r48_20261120_001.json'
operator = read(operator_path)
assert operator['both_powered_off'] and operator['contact_detected'] is False
exports = run / 'review_exports'
exports.mkdir(exist_ok=False)
for source, name in [(STAGES[1] / 'entry_clock_midpoint_review_v1', 'midpoint'),
                     (STAGES[1] / 'front_phase_review_v1', 'front')]:
    shutil.copytree(source, exports / name)
for name in ('paired_geometry_review.json', 'paired_geometry_comparison.png', 'paired_log_repeatability.json'):
    shutil.copyfile(STAGES[1] / name, exports / name)
shutil.copyfile(operator_path, exports / 'operator_result.json')
text = f'''# 機体A・1800 R180：同一電池・同一設定2本

半径48mm・補正＋4.25°・Lstart7.52mm・Lend31.5mm、PID/FF・吸引650を維持し、同じ電池を使って独立2本を取得。
両方で動画・1msログを保存。操作者が接触なし・停止・機体/HY0020両方OFFを確認。

| 入口基準の相対マーカー中点 | 1本目 | 2本目 |
| --- | ---: | ---: |
| 前45mm断面の返り幅 [mm] | {widths[0]:.3f} | {widths[1]:.3f} |
| 返り角 [deg] | {angles[0]:.3f} | {angles[1]:.3f} |
| Lendの右 [mm] | {reviews[0]['lend_right_forward_mm'][0]:.3f} | {reviews[1]['lend_right_forward_mm'][0]:.3f} |
| Lendの前 [mm] | {reviews[0]['lend_right_forward_mm'][1]:.3f} | {reviews[1]['lend_right_forward_mm'][1]:.3f} |

返り幅の2本差は{np.ptp(widths):.3f}mm、角度差は{np.ptp(angles):.3f}deg。
各フレームの実ArUco2/3登録と欠測除外を維持し、入口直線の1ms生エンコーダ距離だけで時刻を照合した。
動画の観測率・入口時計・直線・静止の個別基準を両方で通過。
標準の全軌跡モデル合わせのsummary不合格は保持し、独立入口時計による相対診断と区別する。
絶対車軸位置・機軸yaw・盤面固定接続の合格や統計的な再現性保証ではない。

12:13の79.23mm/182.44degとの差の原因は未確定で、旧試行を除去・不合格への再分類はしていない。
操作者は電池交換以外の車体・タイヤ・ファーム変更なし、および電池の種類・容量・取付位置も同じと回答。
電池交換だけが差の原因と確定したわけではない。今回の電池条件における2本の比較として扱う。

次候補は半径だけ48→46.2mm。平均幅{mean_width:.3f}mmを比例換算すると約{mean_width * 46.2 / 48:.3f}mmとなるが、
実測した因果応答ではなく試験候補。補正・前後長・PID/FF・吸引は変更しない。
作業目標は幅90±2mm・角180±1deg・Lend(90,90)mmに対する右±2mm/前±3mm。
現在は準備のみで未実行、採用値・ファームは未変更。次の配置・余裕・ON・手離し確認が必要。
'''
with (run / 'paired_repeat_README.md').open('x', encoding='utf-8') as file:
    file.write(text)
with (NEXT / 'geometry_proposal.json').open('x', encoding='utf-8') as file:
    json.dump(result, file, ensure_ascii=False, indent=2, allow_nan=False)
for path in (STAGES[1] / 'phase.json', BASE / 'preparation.json'):
    phase = read(path)
    phase.update(state='SAME_BATTERY_PAIR_REVIEWED_RADIUS46P2_PREPARED',
        phase='SAME_BATTERY_PAIR_REVIEWED_RADIUS46P2_PREPARED', operator_ready=False,
        execution_pending=False, power_state='Both_OFF_confirmed', ble_disconnected=True,
        operator_contact=False, current_placement_confirmed=False, current_clearance_confirmed=False,
        current_hands_clear_confirmed=False, current_physical_clearance_verified=False,
        next_candidate_executed=False, source_params_changed=False, parameter_adoption_allowed=False,
        candidate=str(NEXT / 'candidate.settings.json'), next_candidate=str(NEXT / 'candidate.settings.json'),
        config=str(NEXT / 'config.json'), next_request_id='A_R180_1800_radius46p2_c4p25_20261120_001',
        last_completed_trial=str(run / 'trial.json'), paired_review=str(STAGES[1] / 'paired_geometry_review.json'))
    path.write_text(json.dumps(phase, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
print(json.dumps(dict(widths_mm=widths, angles_deg=angles, width_span_mm=float(np.ptp(widths)),
    angle_span_deg=float(np.ptp(angles)), predicted_candidate_width_mm=mean_width * 46.2 / 48,
    physical_execution=False), ensure_ascii=False))
