"""A1800右45入口の同一候補3本を相対基準で集計する。実機操作・採用値反映なし。"""
from pathlib import Path
from datetime import datetime, timezone, timedelta
from hashlib import sha256
import csv
import json
import shutil
import sys

import cv2
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tools/logs/pixel_camera/turn1800_A_inr45_20261120_v1'
VALIDATION = BASE / 'rear_yellow_s20_validation_v1'
OUT = BASE / 'single_turn_candidate_3run_review_v1'
sys.path[:0] = [str(ROOT / 'tools'), str(ROOT / 'tools/analysis')]
from pixel_camera_tuning import setting_differences
from video_marker_pose import board_to_pose_transform, transform_points

TRIALS = [
    ('first', 'front_Ls2p0_Le40_01', 'A_INR45_1800_front_Ls2p0_Le40_20261120_001'),
    ('second', 'resume1620_01', 'A_INR45_1800_resume1620_20261120_001'),
    ('third', 'repeat03_resume01', 'A_INR45_1800_repeat03_resume_20261120_001'),
]

def read(path):
    return json.loads(path.read_text(encoding='utf-8'))

def save(path, value):
    with path.open('x', encoding='utf-8') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')

def digest(path):
    with path.open('rb') as stream:
        return __import__('hashlib').file_digest(stream, 'sha256').hexdigest()

OUT.mkdir(exist_ok=False)
now = datetime.now(timezone(timedelta(hours=9))).isoformat()
inputs, results, previous_settings = set(), [], None
figure, axis = plt.subplots(figsize=(7, 8), layout='constrained')
for index, (label, stage_name, identifier) in enumerate(TRIALS, 1):
    stage = BASE / stage_name
    run = stage / 'local_execution' / ('run_' + identifier)
    trial = read(run / 'trial.json')
    settings = read(run / 'saved_1ms.settings.json')
    requested = read(run / 'requested.settings.json')
    report_folder = VALIDATION / (label + '_midpoint_review')
    review = read(report_folder / 'midpoint_review.json')
    full = VALIDATION / (label + '_full_core')
    stop_folder = VALIDATION / (label + '_stop_and_recovered_images')
    stopped = read(stop_folder / 'confirmed_stop_review.json')
    operator_path = stage / 'local_execution/operator_results' / (identifier + '.json')
    operator = read(operator_path)
    tracking_path = BASE / 'parallel_analysis' / identifier / ('tracking_1ms_v1' if index == 1 else 'tracking_1ms_review') / 'verification.json'
    tracking = read(tracking_path)
    assert settings['mouse'] == 'A' and settings['motion_name'] == 'in_r45'
    assert settings['preset_speed_mm_s'] == 1800 and settings['turn_count'] == 1
    assert settings['suction'] == dict(enabled=True, duty=650)
    assert not setting_differences(settings, requested, quantized=True)
    if previous_settings is not None:
        assert not setting_differences(settings, previous_settings, quantized=True)
    previous_settings = settings
    assert trial['execution_write_attempted'] is True and trial['execution_resend_count'] == 0
    assert trial['log_quality']['full_motion_retained'] and trial['saved_log']['drop_count'] == 0
    assert operator['both_powered_off'] and operator['contact_detected'] is False
    assert stopped['passed']
    quality = review['independent_clock_required_motion_quality']
    segments = review['segment_observation_quality']
    motion_gates = bool(quality['accepted_for_relative_review'] and review['clock']['passed']
        and all(segments[key]['observation_rate'] >= .95 for key in ('pure_motion', 'entry', 'exit'))
        and review['entry_line']['rms_mm'] <= .75 and review['exit_line']['rms_mm'] <= .75
        and trial['log_quality']['lost_frames'] == trial['log_quality']['crc_errors_during_trial'] == 0)
    assert motion_gates

    # 保存済み入口座標系を再現し、停止点を同じ座標系へ投影する。終点には合わせない。
    pose = pd.read_csv(full / 'apparent_floor_marker_pose.csv')
    reference = read(full / 'marker_reference_used.json')
    inverse = np.linalg.inv(board_to_pose_transform(reference))
    front = transform_points(pose[['blue_x_mm', 'blue_y_mm']].to_numpy(), inverse).astype(float)
    rear = transform_points(pose[['yellow_x_mm', 'yellow_y_mm']].to_numpy(), inverse).astype(float)
    weighted = ((1 - 10 / 1500) * front + (1 - 1 / 1500) * rear) / 2
    valid = pose.marker_detected_raw.to_numpy(bool) & pose.calibration_valid.to_numpy(bool)
    stamps = pose.time_video_s.to_numpy()
    start_used = review['entry_window_pts_s'][0] - .010
    baseline = valid & (stamps >= start_used - .18) & (stamps <= start_used - .04)
    origin = np.median(weighted[baseline], axis=0)
    lo, hi = review['entry_window_pts_s']
    entry = weighted[valid & (stamps >= lo) & (stamps <= hi)]
    direction = np.linalg.svd(entry - entry.mean(0), full_matrices=False)[2][0]
    if direction @ (entry[-1] - entry[0]) < 0:
        direction = -direction
    basis = np.column_stack(([-direction[1], direction[0]], direction))
    local = (weighted - origin) @ basis
    existing = pd.read_csv(report_folder / 'midpoint_relative_trajectory.csv')
    reconstruction_difference = float(np.max(np.abs(local[valid] - existing[['right_mm', 'forward_mm']].to_numpy()[valid])))
    assert reconstruction_difference < 1e-5
    stop_pose = pd.read_csv(stop_folder / 'confirmed_stop_pose.csv')
    stop_valid = stop_pose.marker_detected_raw.to_numpy(bool) & stop_pose.calibration_valid.to_numpy(bool)
    stop_front = transform_points(stop_pose[['blue_x_mm', 'blue_y_mm']].to_numpy(), inverse).astype(float)
    stop_rear = transform_points(stop_pose[['yellow_x_mm', 'yellow_y_mm']].to_numpy(), inverse).astype(float)
    stop_weighted = ((1 - 10 / 1500) * stop_front + (1 - 1 / 1500) * stop_rear) / 2
    endpoint = np.median(((stop_weighted - origin) @ basis)[stop_valid], axis=0)

    residual = np.array(review['midpoint_Lend_residual_mm'])
    archive = read(run / 'archive.json')['archive_folder']
    result = dict(trial=index, label=label, source_trial=str(run / 'trial.json'), archive=archive,
        exit_angle_deg=review['exit_path_direction_change_deg'],
        angle_error_deg=review['exit_path_error_from45_deg'],
        lend_right_mm=review['midpoint_Lend_right_forward_mm'][0],
        lend_forward_mm=review['midpoint_Lend_right_forward_mm'][1],
        lend_error_right_mm=float(residual[0]), lend_error_forward_mm=float(residual[1]),
        lend_error_norm_mm=float(np.linalg.norm(residual)),
        required_motion_observation_fraction=quality['observation_rate'],
        pure_motion_observation_fraction=segments['pure_motion']['observation_rate'],
        calibration_fraction=quality['calibration_rate'],
        measured_period_ms=tracking['measured_period_ms'],
        gyro_through_lend_deg=tracking['gyro_through_lend_measured_time_deg'],
        pulse_speed_mm_s=tracking['pulse_speed_measured_time_mean_mm_s'],
        saturation_pct=tracking['nominal_1ms_tracking_metrics']['saturation_pct'],
        battery_min_v=tracking['battery_min_v'],
        stop_right_mm=float(endpoint[0]), stop_forward_mm=float(endpoint[1]),
        stop_error_right_mm=float(endpoint[0] - 135), stop_error_forward_mm=float(endpoint[1] - 270),
        confirmed_stop_scatter_rms_mm=stopped['scatter_rms_mm'],
        early_post_stop_scatter_rms_mm=review['late_stationary']['scatter_rms_mm'],
        recovered_actual_pairs=stopped['recovered_actual_pairs'],
        relative_motion_geometry_gates_passed=True, confirmed_stop_gates_passed=True,
        original_native_summary=review['original_summary'],
        original_core_eligibility_with_early_stop_window=review['independent_optical_geometry_eligible'],
        basis_reconstruction_max_difference_mm=reconstruction_difference,
        source_params_changed=False, contact_detected=False, both_powered_off=True)
    results.append(result)
    motion_start = quality['video_motion_start_s']
    motion_end = quality['video_motion_end_s']
    display = (existing.time_video_s >= motion_start - .04) & (existing.time_video_s <= motion_end + .1)
    xy = existing[['right_mm', 'forward_mm']].to_numpy()
    xy[~existing.registered_midpoint_valid.to_numpy(bool)] = np.nan
    line, = axis.plot(*xy[display].T, label=f'run {index}: {result["exit_angle_deg"]:.2f} deg')
    axis.scatter(result['lend_right_mm'], result['lend_forward_mm'], color=line.get_color(), s=35)
    axis.scatter(*endpoint, color=line.get_color(), marker='s', s=35)
    inputs.update([run / filename for filename in ('trial.json', 'saved_1ms.csv', 'saved_1ms.settings.json',
        'requested.settings.json', 'telemetry.csv', 'marker_reference_used.json', 'body_geometry_used.json', 'archive.json')])
    inputs.update([report_folder / 'midpoint_review.json', report_folder / 'midpoint_relative_trajectory.csv',
        full / 'apparent_floor_marker_pose.csv', full / 'apparent_floor_marker_pose.provenance.json',
        stop_folder / 'confirmed_stop_pose.csv', stop_folder / 'confirmed_stop_review.json',
        stop_folder / 'recovered_pairs_contact_sheet.png', tracking_path, operator_path])

axis.plot([45, 135], [180, 270], '--', color='.4', label='45 deg exit reference')
axis.scatter(45, 180, marker='x', color='black', s=90, label='Lend target (45,180) mm')
axis.scatter(135, 270, marker='x', color='.3', s=90, label='Stop reference (135,270) mm')
axis.set(xlabel='Right from measured entry [mm]', ylabel='Forward from measured entry [mm]',
    title='Mouse A / in_r45 / preset 1800 / suction 650\nRelative height-corrected marker midpoint',
    aspect='equal', xlim=(-8, 155), ylim=(-5, 295))
axis.grid(alpha=.25)
axis.legend(fontsize=8)
figure.savefig(OUT / 'trajectory_three.png', dpi=160)
plt.close(figure)

angles = np.array([r['exit_angle_deg'] for r in results])
errors = np.array([[r['lend_error_right_mm'], r['lend_error_forward_mm']] for r in results])
candidate = read(BASE / 'repeat03_resume01/candidate.settings.json')
save(OUT / 'candidate.settings.json', candidate)
summary = dict(created_at_jst=now, mouse='A', motion='in_r45', preset_mm_s=1800,
    status='SINGLE_TURN_RELATIVE_CANDIDATE_CONFIRMED', physical_runs=3, newly_captured_in_this_conversation=2,
    candidate_parameters=candidate['parameters'], suction=candidate['suction'],
    evaluation_frame='relative marker midpoint; origin and axes from pre-motion and measured entry only',
    absolute_axle_or_body_yaw_verified=False, board_fixed_connection_verified=False,
    exit_angle_range_deg=[float(angles.min()), float(angles.max())],
    exit_angle_mean_deg=float(angles.mean()), maximum_absolute_angle_error_deg=float(np.max(np.abs(angles - 45))),
    maximum_lend_error_norm_mm=float(max(r['lend_error_norm_mm'] for r in results)),
    mean_lend_error_right_forward_mm=errors.mean(0).tolist(),
    analytical_length_only_estimate_unexecuted=dict(lstart_mm=float(np.mean([
        read(VALIDATION / (label + '_midpoint_review/midpoint_review.json'))['length_only_candidate_at_current_angle']['request_Lstart_mm']
        for label, _, _ in TRIALS])), lend_mm=float(np.mean([
        read(VALIDATION / (label + '_midpoint_review/midpoint_review.json'))['length_only_candidate_at_current_angle']['request_Lend_mm']
        for label, _, _ in TRIALS])), adopted=False,
        reason='About 1 mm adjustment is comparable to timing and measurement sensitivity; preserve the physically repeated candidate.'),
    detector_candidate=str(VALIDATION / 'body_geometry_A_yellow_s20_candidate.json'),
    detector_review='All 27 recovered pairs visually inspected on registered-frame contact sheets; actual rear disk confirmed.',
    original_rejections_preserved=True, original_native_auto_stop_acceptance_replaced=False,
    confirmed_stop_window='Per-trial recording_stop confirmation + [0.10,0.54] s, fixed before inspection',
    firmware_table_modified=False, firmware_written=False, continuous_or_maze_run_verified=False,
    turn_ekf=dict(mode_recorded='control', current_source_eligible=False, identified_k=None,
        reason='Current source permits suction right90/right180 near1600;1800 in_r45 is outside pattern/speed scope. Height-corrected body yaw is uncalibrated; do not identify slip K from marker-path angle.'),
    power_state='Both_OFF_confirmed_after_last_run', trials=results)
save(OUT / 'review.json', summary)
with (OUT / 'measurements.csv').open('x', encoding='utf-8-sig', newline='') as stream:
    keys = [k for k in results[0] if not isinstance(results[0][k], dict)]
    writer = csv.DictWriter(stream, fieldnames=keys)
    writer.writeheader()
    writer.writerows({k: row[k] for k in keys} for row in results)
save(OUT / 'visual_recovered_pair_review.json', dict(reviewed_at_jst=now, recovered_pairs=27,
    all_recovered_pairs_on_actual_front_and_rear_markers=True, required_decoded_aruco_ids=[2, 3],
    new_parameter_adoption_allowed=False, detector_scope='Three current A1800 right45 saved trials only'))

rows = '\n'.join(f'| {r["trial"]} | {r["exit_angle_deg"]:.3f} | {r["lend_error_right_mm"]:+.2f} | {r["lend_error_forward_mm"]:+.2f} | {r["lend_error_norm_mm"]:.2f} | {100*r["required_motion_observation_fraction"]:.2f}% | {r["confirmed_stop_scatter_rms_mm"]:.3f} |' for r in results)
stop_rows = '\n'.join(f'| {r["trial"]} | {r["stop_error_right_mm"]:+.2f} | {r["stop_error_forward_mm"]:+.2f} | {r["battery_min_v"]:.3f} | {r["saturation_pct"]:.2f}% |' for r in results)
readme = f'''# 機体A・右45°入口・1800の単体候補

同一の **Rmin −53 mm / Lstart 2.0 mm / Lend 40.0 mm / 角度補正 0°** を、吸引650・単体回数1で3本比較した。既存1本と今回取得2本を使用し、全3本で操作者が接触なしと両方OFFを確認している。作成日時は {now}。

これは実測入口へ合わせた相対マーカー中点の候補である。盤面固定の出口・停止点、車軸の絶対位置、機体yaw、連続・最短走行の採用は別評価となる。機体別ソースへの反映と書込みは行っていない。

## 結果

Lend目標は入口基準の右45 / 前180 mm。各フレームで実復号した登録ArUcoを使用し、黄マーカーの最低彩度だけを30から20へ変えた解析用候補で再評価した。

| 試行 | 出口方向 ° | Lend右差 mm | Lend前差 mm | 差の大きさ mm | 必要区間観測率 | 停止後散乱RMS mm |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
{rows}

出口方向は **{angles.min():.3f}～{angles.max():.3f}°**、Lend位置差の最大は **{summary['maximum_lend_error_norm_mm']:.2f} mm**。保存ログは全3本で欠落・CRC異常0、設定読戻しは同じ。保存ログとMCU時刻の一致から周期を求め、固定1 msと混同せずに積算した。Lendまでのジャイロ積算は45.39～45.49°だった。

![3本の相対軌跡](trajectory_three.png)

ターン後の停止点は別に記録した。基準は右135 / 前270 mmで、以下の停止差をLendへ埋め込んでいない。最低電圧と旋回中の飽和率にも試行差がある。

| 試行 | 停止点右差 mm | 停止点前差 mm | ログ最低電圧 V | 旋回中飽和率 |
| --- | ---: | ---: | ---: | ---: |
{stop_rows}

## 検出と停止判定の根拠

彩度20の候補を試行2の欠測像から作り、試行1・3を含む全動作フレームで検証した。回復した27組すべての画像を目視し、実在する前赤・後黄の点を確認した。色以外の物理幾何、ArUco登録、マーカー間隔40～65 mm、観測率95%などの条件は同じ。元の彩度30の失敗結果を保持している。

試行2は指令終了直後0.25～0.65秒で散乱1.93 mmだったため、この早い窓の不合格を残した。全3本とも、既存のログ停止確認時刻から固定で+0.10～+0.54秒を別途検証し、観測率100%、散乱RMS1 mm以下、中央値から3 mm以内を確認した。静かな区間を探して窓を変更していない。

既存GUIの自動summaryは初期直進の観測数判定で未合格のまま保持した。本結果は入口の生車輪距離だけで時計を合わせ、入口軸・出口直線・Lendを別途検証した補助評価であり、自動判定を書き換えていない。

## 保存候補と未確認事項

- [実行した候補設定](candidate.settings.json)、[数値集計](measurements.csv)、[詳細・入力対応](review.json)、[入力ハッシュ](inputs_manifest.json)。
- [解析用の黄検出候補](../rear_yellow_s20_validation_v1/body_geometry_A_yellow_s20_candidate.json)と、その配下の全フレームCSV・回復点画像・停止確認CSVを保存した。撮影用の共有設定と機体ファームは変更していない。
- 長さだけの平均的な逆算はLstart約0.9 mm / Lend約39.3 mmだが、約1 mmの変更は時計・画像の感度と同程度のため未実行の参考値とした。3回実走した2.0 / 40.0 mmを保存候補とする。
- EKFは記録上controlだが、現行ソースの1800・斜め入口は適用対象外。今回のK・雑音を同定済みとは扱わない。機体yawとマーカー高さ・カメラの独立校正も未完了。
- 相対入口に合わせた評価であり、盤面固定接続、停止点の基準達成、連続・最短走行と正式採用値への反映は未確認。

3本の動画・CSV・設定はDドライブへ一式保存してハッシュ照合した。保存先はreview.jsonの各archiveに記載した。全入力と今回の解析コードのハッシュをinputs_manifest.jsonへ記録した。
'''
(OUT / 'README.md').write_text(readme, encoding='utf-8')
sources = OUT / 'analysis_source'
sources.mkdir()
for filename in ('review_saved_inr45_s20_20261120.py', 'review_inr45_midpoint_s20_saved_20261120.py',
    'validate_inr45_s20_stop_and_images_20261120.py', 'finalize_inr45_1800_three_20261120.py',
    'parallel_front_run_review_20261120.py', 'parallel_inr45_candidate_tracking_20261120.py'):
    source = ROOT / 'tmp' / filename
    shutil.copy2(source, sources / filename)
    inputs.add(source)
inputs.add(VALIDATION / 'body_geometry_A_yellow_s20_candidate.json')
inputs.update(ROOT / filename for filename in ('tools/analysis/video_marker_pose.py', 'tools/marker_detection.py',
    'tools/analysis/pixel_camera_relative_pose.py', 'tools/pixel_camera_geometry.py',
    'Params/Param_A/turn_1800/turn_1800.cpp', 'Params/turn_pose_param.h', 'Task/Inc/turn_pose_ekf.h'))
save(OUT / 'inputs_manifest.json', dict(created_at_jst=now,
    files={str(path): dict(sha256=digest(path), size_bytes=path.stat().st_size) for path in sorted(inputs)},
    source_inputs_modified=False, device_connections=0))
print(json.dumps(dict(output=str(OUT), angle_range=summary['exit_angle_range_deg'],
    max_lend_error_mm=summary['maximum_lend_error_norm_mm'], stop_offsets=[
        [r['stop_error_right_mm'], r['stop_error_forward_mm']] for r in results],
    all_three_motion_and_confirmed_stop_gates_passed=True), ensure_ascii=False), flush=True)
