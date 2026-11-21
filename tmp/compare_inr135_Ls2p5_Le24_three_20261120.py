"""機体A・右135入口1800の同一候補3本を、保存データだけで比較する。"""
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
OUT = BASE / 'Ls2p5_Le24_c3p5_three_review_v1'
read = lambda p: json.loads(p.read_text(encoding='utf-8-sig'))
rows, evidence = [], []
reference_settings = None
curves = []
for index in (1, 2, 3):
    identifier = f'A_INR135_1800_c3p5_Ls2p5_Le24_req_20261120_{index:03}'
    stage = BASE / f'c3p5_Ls2p5_Le24_{index:02}'
    run = stage / 'local_execution' / ('run_' + identifier)
    analysis = BASE / 'parallel_analysis' / identifier
    folder = analysis / 'midpoint_raw_wheel_clock_v1'
    review = read(folder / 'midpoint_review.json')
    tracking = read(analysis / 'tracking_1ms_v1/verification.json')
    settings = read(stage / 'candidate.settings.json')
    actual = read(run / 'saved_1ms.settings.json')
    operator_path = stage / 'local_execution/operator_results' / (identifier + '.json')
    operator = read(operator_path)
    assert review['independent_optical_geometry_eligible'] is True
    assert review['clock']['passed'] is True
    assert operator['contact_detected'] is False and operator['stopped'] is True
    assert tracking['cnt_contiguous'] is True and tracking['saved_log_status']['drop_count'] == 0
    if reference_settings is None:
        reference_settings = settings
        reference_actual = {key: actual[key] for key in ('parameters', 'pid', 'feedforward', 'suction')}
    assert settings == reference_settings
    assert {key: actual[key] for key in reference_actual} == reference_actual
    residual = np.asarray(review['midpoint_Lend_residual_mm'])
    stop = np.asarray(review['late_stationary']['midpoint_right_forward_mm']) - [180., 45.]
    row = dict(trial_id=identifier, source_review=str(folder / 'midpoint_review.json'),
        exit_angle_deg=review['exit_path_direction_change_deg'],
        exit_angle_error_deg=review['exit_path_error_from135_deg'],
        Lend_right_forward_mm=review['midpoint_Lend_right_forward_mm'],
        Lend_residual_right_forward_mm=residual.tolist(), Lend_residual_norm_mm=float(np.linalg.norm(residual)),
        stop_residual_right_forward_mm=stop.tolist(),
        observation_rate=review['independent_clock_required_motion_quality']['observation_rate'],
        registration_rate=review['independent_clock_required_motion_quality']['calibration_rate'],
        clock_rms_mm=review['clock']['incoming_raw_wheel_fit_rms_mm'],
        tracking_rms_rad_s=tracking['nominal_1ms_tracking_metrics']['tracking_rms_rad_s'],
        saturation_pct=tracking['nominal_1ms_tracking_metrics']['saturation_pct'],
        battery_min_v=tracking['battery_min_v'], sample_count=tracking['sample_count'],
        original_summary=review['original_summary'], contact_detected=False)
    rows.append(row)
    frame = pd.read_csv(folder / 'midpoint_relative_trajectory.csv')
    quality = review['independent_clock_required_motion_quality']
    interval = (frame.time_video_s >= quality['video_motion_start_s'] - .02) & (frame.time_video_s <= quality['video_motion_end_s'] + .05)
    valid = frame.registered_midpoint_valid.astype(bool).to_numpy()
    curves.append(np.where(valid[:, None], frame[['right_mm', 'forward_mm']].to_numpy(), np.nan)[interval])
    evidence.extend([folder / 'midpoint_review.json', analysis / 'tracking_1ms_v1/verification.json',
        stage / 'candidate.settings.json', run / 'saved_1ms.settings.json', operator_path])

angles = np.array([row['exit_angle_deg'] for row in rows])
points = np.array([row['Lend_right_forward_mm'] for row in rows])
report = dict(mouse='A', motion='in_r135', preset_mm_s=1800,
    requested_settings=reference_settings, verified_actual_settings=reference_actual,
    trial_count=3, rows=rows,
    mean_exit_angle_deg=float(angles.mean()), median_exit_angle_deg=float(np.median(angles)),
    exit_angle_range_deg=[float(angles.min()), float(angles.max())],
    exit_angle_peak_to_peak_deg=float(np.ptp(angles)),
    mean_Lend_right_forward_mm=points.mean(0).tolist(),
    mean_Lend_residual_right_forward_mm=(points.mean(0)-[90., 135.]).tolist(),
    Lend_peak_to_peak_right_forward_mm=np.ptp(points, axis=0).tolist(),
    all_relative_motion_quality_passed=True, all_operator_no_contact=True,
    all_requested_and_actual_settings_identical=True,
    both_powered_off_confirmed_after_third=operator['both_powered_off'],
    coordinate_basis='height-corrected relative marker midpoint and independently measured entry axes; no absolute axle calibration',
    source_firmware_changed=False, parameter_adoption_allowed=False,
    EKF_identified=False, hardware_operations=0,
    limitations=['通常解析の初期直進観測不足は保持。各試行の独立入口時計と全動作区間の品質を別に検証した。',
        '盤面固定の開始・出口線の合格や、絶対車軸位置精度を保証しない。',
        '1800・右135度は現行ターンEKFの校正範囲外。K・雑音の採用値は変更しない。'],
    source_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in evidence + [Path(__file__)]})
OUT.mkdir(exist_ok=False)
with (OUT / 'review.json').open('x', encoding='utf-8') as file:
    json.dump(report, file, ensure_ascii=False, indent=2, allow_nan=False)
    file.write('\n')
pd.DataFrame(rows).to_csv(OUT / 'trials.csv', index=False)
plt.rcParams['font.family'] = 'Yu Gothic'
fig, ax = plt.subplots(figsize=(7, 7), layout='constrained')
for index, (curve, row) in enumerate(zip(curves, rows), 1):
    item, = ax.plot(*curve.T, label=f"{index}本目：出口 {row['exit_angle_deg']:.2f}°")
    ax.scatter(*row['Lend_right_forward_mm'], color=item.get_color(), s=40)
ax.scatter(90, 135, color='black', marker='x', s=65, label='Lend目標 (90,135) mm')
ax.plot([90, 180], [135, 45], '--', color='gray', label='目標出口線')
ax.scatter(180, 45, color='black', marker='+', s=65, label='停止目標 (180,45) mm')
ax.set(xlabel='入口に対する右 [mm]', ylabel='入口に対する前 [mm]', aspect='equal',
    title='機体A / 右135°入口1800 / 同一設定3本\nLstart 2.5・Lend 24 mm・補正＋3.5° / 相対中点')
ax.legend(fontsize=8)
ax.grid(alpha=.3)
fig.savefig(OUT / 'three_trajectories.png', dpi=150)
plt.close(fig)
print(json.dumps({key: report[key] for key in ('trial_count', 'mean_exit_angle_deg', 'exit_angle_range_deg',
    'exit_angle_peak_to_peak_deg', 'mean_Lend_residual_right_forward_mm', 'Lend_peak_to_peak_right_forward_mm')}, ensure_ascii=False, indent=2))
