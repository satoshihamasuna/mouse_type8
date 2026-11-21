"""完全記録の1・3本目と、1msログを失った2本目の動画診断を区別して比較する。"""
from pathlib import Path
import hashlib
import json
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tools/logs/pixel_camera/turn1800_A_inr135_20261120_v1'
OUT = BASE / 'Ls2p5_Le26_c2p4_review_v1'
read = lambda p: json.loads(p.read_text(encoding='utf-8-sig'))
contact_path = BASE / 'lengths_Ls2p5_Le26_c2p4_preparation_v1/all_three_contact_and_off_confirmation.json'
contact = read(contact_path)
assert contact['all_three_no_contact'] and contact['both_powered_off']
rows, curves, evidence = [], [], [contact_path]
expected = read(BASE / 'c2p4_Ls2p5_Le26_01/candidate.settings.json')
reference_actual = None
for index in (1, 3):
    identifier = f'A_INR135_1800_c2p4_Ls2p5_Le26_req_20261120_{index:03}'
    stage = BASE / f'c2p4_Ls2p5_Le26_{index:02}'
    run = stage / 'local_execution' / ('run_' + identifier)
    folder = BASE / 'parallel_analysis' / identifier
    review_path = folder / 'midpoint_raw_wheel_clock_v1/midpoint_review.json'
    tracking_path = folder / 'tracking_1ms_v1/verification.json'
    review, tracking = read(review_path), read(tracking_path)
    settings = read(run / 'saved_1ms.settings.json')
    actual = {key: settings[key] for key in ('parameters', 'pid', 'feedforward', 'suction')}
    if reference_actual is None:
        reference_actual = actual
    assert actual == reference_actual
    assert read(stage / 'candidate.settings.json') == expected
    assert review['independent_optical_geometry_eligible'] and review['clock']['passed']
    assert tracking['cnt_contiguous'] and tracking['saved_log_status']['drop_count'] == 0
    residual = np.array(review['midpoint_Lend_residual_mm'])
    stop = np.array(review['late_stationary']['midpoint_right_forward_mm']) - [180., 45.]
    rows.append(dict(trial_index=index, trial_id=identifier, evidence_grade='COMPLETE_VIDEO_AND_SAVED_1MS',
        source_review=str(review_path), exit_angle_deg=review['exit_path_direction_change_deg'],
        exit_angle_error_deg=review['exit_path_error_from135_deg'],
        Lend_right_forward_mm=review['midpoint_Lend_right_forward_mm'],
        Lend_residual_right_forward_mm=residual.tolist(), Lend_residual_norm_mm=float(np.linalg.norm(residual)),
        stop_residual_right_forward_mm=stop.tolist(),
        observation_rate=review['independent_clock_required_motion_quality']['observation_rate'],
        registration_rate=review['independent_clock_required_motion_quality']['calibration_rate'],
        clock_rms_mm=review['clock']['incoming_raw_wheel_fit_rms_mm'],
        tracking_rms_rad_s=tracking['nominal_1ms_tracking_metrics']['tracking_rms_rad_s'],
        saturation_pct=tracking['nominal_1ms_tracking_metrics']['saturation_pct'],
        battery_min_v=tracking['battery_min_v'], original_summary=review['original_summary']))
    frame = pd.read_csv(folder / 'midpoint_raw_wheel_clock_v1/midpoint_relative_trajectory.csv')
    quality = review['independent_clock_required_motion_quality']
    interval = (frame.time_video_s >= quality['video_motion_start_s'] - .02) & (frame.time_video_s <= quality['video_motion_end_s'] + .05)
    curves.append(np.where(frame.registered_midpoint_valid.to_numpy(bool)[:, None],
        frame[['right_mm', 'forward_mm']].to_numpy(), np.nan)[interval])
    evidence.extend([review_path, tracking_path, run / 'saved_1ms.settings.json', stage / 'candidate.settings.json'])

diagnostic_folder = BASE / 'c2p4_Ls2p5_Le26_02/optical_diagnostic_v2'
diagnostic = read(diagnostic_folder / 'report.json')
assert diagnostic['trial_state_unchanged'] == 'FAILED'
assert diagnostic['observation_fraction'] >= .95 and diagnostic['registration_fraction'] >= .95
assert not diagnostic['precise_Lend_measured'] and not diagnostic['parameter_adoption_allowed']
video_only = dict(trial_index=2, evidence_grade='VIDEO_ONLY_MISSING_SAVED_1MS',
    source_review=str(diagnostic_folder / 'report.json'),
    exit_angle_deg=diagnostic['exit_path_direction_change_deg'],
    exit_angle_error_deg=diagnostic['exit_path_error_from135_deg'],
    Lend_measured=False, original_trial_state=diagnostic['trial_state_unchanged'],
    observation_rate=diagnostic['observation_fraction'], registration_rate=diagnostic['registration_fraction'],
    excluded_from_Lend_and_tracking_statistics=True)
evidence.extend([diagnostic_folder / 'report.json', Path(__file__)])
points = np.array([row['Lend_right_forward_mm'] for row in rows])
full_angles = np.array([row['exit_angle_deg'] for row in rows])
angles = np.r_[full_angles, video_only['exit_angle_deg']]
report = dict(mouse='A', motion='in_r135', preset_mm_s=1800,
    requested_settings=expected, verified_actual_settings=reference_actual,
    physical_trials=3, complete_video_and_1ms_trials=2, video_only_trials=1,
    complete_rows=rows, video_only_row=video_only,
    all_video_exit_angle_range_deg=[float(angles.min()), float(angles.max())],
    all_video_mean_exit_angle_deg=float(angles.mean()),
    complete_records_exit_angle_range_deg=[float(full_angles.min()), float(full_angles.max())],
    mean_Lend_right_forward_mm=points.mean(0).tolist(),
    mean_Lend_residual_right_forward_mm=(points.mean(0)-[90., 135.]).tolist(),
    Lend_peak_to_peak_right_forward_mm=np.ptp(points, axis=0).tolist(),
    Lend_residual_norm_range_mm=[min(row['Lend_residual_norm_mm'] for row in rows), max(row['Lend_residual_norm_mm'] for row in rows)],
    all_three_operator_no_contact=True, both_powered_off_confirmed=True,
    coordinate_basis='height-corrected relative marker midpoint and measured entry direction; no calibrated absolute axle/body yaw',
    source_firmware_changed=False, parameter_adoption_allowed=False, candidate_selection_state='AWAITING_REVIEW',
    continuous_turns_verified=False, EKF_identified=False, hardware_operations=0,
    limitations=['2本目は通信切断・電源再投入で保存1msログを失った。元のFAILED記録を保持し、Lend・追従統計へ混ぜない。',
        '完全記録は1本目と3本目。通常自動解析の初期直進観測不足は保持し、独立入口時計と全動作区間の観測品質で別評価した。',
        '単体の相対評価であり、盤面固定の絶対位置や連続・最短接続、EKF係数の採用は未確認。'],
    source_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in evidence})
OUT.mkdir(exist_ok=False)
with (OUT / 'review.json').open('x', encoding='utf-8') as file:
    json.dump(report, file, ensure_ascii=False, indent=2, allow_nan=False)
    file.write('\n')
pd.DataFrame(rows + [video_only]).to_csv(OUT / 'trials.csv', index=False)
plt.rcParams['font.family'] = 'Yu Gothic'
fig, ax = plt.subplots(figsize=(7, 7), layout='constrained')
for curve, row in zip(curves, rows):
    item, = ax.plot(*curve.T, label=f"{row['trial_index']}本目：完全記録・出口 {row['exit_angle_deg']:.2f}°")
    ax.scatter(*row['Lend_right_forward_mm'], color=item.get_color(), s=40)
frame = pd.read_csv(diagnostic_folder / 'relative_midpoint.csv')
valid = frame.measured_pair_and_registration_valid.to_numpy(bool)
interval = (frame.time_video_s >= diagnostic['entry_window_s'][0] - .15) & (frame.time_video_s <= diagnostic['exit_window_s'][1] + .1)
xy = np.where(valid[:, None], frame[['relative_right_mm', 'relative_forward_mm']].to_numpy(), np.nan)
ax.plot(*xy[interval].T, '--', color='gray', label=f"2本目：動画のみ・出口 {video_only['exit_angle_deg']:.2f}°")
ax.scatter(90, 135, color='black', marker='x', s=65, label='Lend目標 (90,135) mm')
ax.plot([90, 180], [135, 45], ':', color='black', label='目標出口線')
ax.scatter(180, 45, color='black', marker='+', s=65, label='停止目標 (180,45) mm')
ax.set(xlabel='入口に対する右 [mm]', ylabel='入口に対する前 [mm]', aspect='equal',
    title='機体A / 右135°入口1800 / 相対マーカー中点\nLstart 2.5・Lend 26 mm・補正＋2.4°')
ax.legend(fontsize=8)
ax.grid(alpha=.3)
fig.savefig(OUT / 'trajectories_with_evidence_grades.png', dpi=150)
plt.close(fig)
print(json.dumps({key: report[key] for key in ('physical_trials', 'complete_video_and_1ms_trials',
    'all_video_exit_angle_range_deg', 'complete_records_exit_angle_range_deg',
    'mean_Lend_residual_right_forward_mm', 'Lend_residual_norm_range_mm')}, ensure_ascii=False, indent=2))
