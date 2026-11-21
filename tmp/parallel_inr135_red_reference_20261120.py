"""A1800 in_r135の赤前点を各frameの登録ArUcoから別参照として評価する。"""
from pathlib import Path
import argparse
import hashlib
import json
import sys

import cv2
import numpy as np
import pandas as pd
from scipy.optimize import minimize_scalar
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tools/logs/pixel_camera/turn1800_A_inr135_20261120_v1'
sys.path[:0] = [str(ROOT / 'tools/analysis')]
from video_marker_pose import (colour_components, marker_registration, registration_quad,
    pose_world_corners, transform_points, board_to_pose_transform)


def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def save(path, value):
    with path.open('x', encoding='utf-8') as file:
        json.dump(value, file, ensure_ascii=False, indent=2, allow_nan=False)
        file.write('\n')


def line(points):
    if len(points) < 15:
        raise ValueError('登録済み直線観測が15点未満')
    direction = np.linalg.svd(points-points.mean(0), full_matrices=False)[2][0]
    if direction @ (points[-1]-points[0]) < 0:
        direction = -direction
    span = float(np.ptp(points @ direction))
    if span < 30:
        raise ValueError('直線の観測範囲が30mm未満')
    residual = points-points.mean(0)
    residual -= np.outer(residual @ direction, direction)
    return direction, dict(points=len(points), span_mm=span,
        rms_mm=float(np.sqrt(np.mean(np.sum(residual**2, 1)))))


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--run', type=Path, required=True)
parser.add_argument('--out', type=Path, required=True)
parser.add_argument('--report', type=Path, required=True)
args = parser.parse_args()
run, out = args.run.resolve(), args.out.resolve()
assert run.is_relative_to(BASE.resolve()) and out.is_relative_to((BASE / 'parallel_analysis').resolve())
assert not out.exists(), '既存出力は保持する'
trial, settings, reference, geometry, coarse = map(read, (run / 'trial.json', run / 'saved_1ms.settings.json',
    run / 'marker_reference_used.json', run / 'body_geometry_used.json', args.report))
assert trial['motion'] == settings['motion_name'] == 'in_r135' and settings['mouse'] == 'A'
assert settings['preset_speed_mm_s'] == 1800 and settings['log']['period_ms'] == 1
assert reference['reference_id'] == geometry['reference_id']
assert set(map(int, reference['markers_px'])) == {2, 3}
assert geometry['front_colour'] == 'red' and geometry['front_height_mm'] == 10 and trial['camera_height_mm'] == 1500
raw, stream = pd.read_csv(run / 'saved_1ms.csv'), pd.read_csv(run / 'telemetry.csv')
assert np.array_equal(raw.cnt, np.arange(len(raw)))
signature = ['ideal.velo', 'ego.velo', 'ideal.rad_velo', 'ego.rad_velo', 'V_r', 'V_l']
matches = raw.merge(stream[['time_ms'] + signature], on=signature)
assert len(matches) >= 18 and matches.cnt.is_unique
tick, origin_mcu = np.polyfit(matches.cnt, matches.time_ms, 1)
assert .95 < tick < 1.05
mcu_start, mcu_stop = [trial['log_quality'][key] for key in ('motion_start_time_ms', 'motion_end_time_ms')]
quality = coarse['required_motion_window_quality']
oldzero = quality['video_motion_start_s'] + (origin_mcu-mcu_start) / 1000
rate = raw['ideal.rad_velo'].to_numpy(float)
turn_index = np.flatnonzero(np.abs(rate) > .02*np.max(abs(rate)))[0]
turn_mcu = origin_mcu + tick*turn_index
saved_time = raw.cnt.to_numpy(float)*tick/1000
wheel_distance = np.cumsum((raw['Encoder_GetProperty_Right().sp_pulse']-
    raw['Encoder_GetProperty_Left().sp_pulse'])*15.02*np.pi/172/2).to_numpy()


def oldpts(mcu):
    return float(oldzero + (mcu-origin_mcu)/1000)


cv2.setNumThreads(2)
cap = cv2.VideoCapture(trial['video_file'])
cap.set(cv2.CAP_PROP_ORIENTATION_AUTO, 0)
rows, index, previous, previous_stamp = [], 0, None, None
while True:
    ok, frame = cap.read()
    if not ok:
        break
    stamp = cap.get(cv2.CAP_PROP_POS_MSEC)/1000
    if stamp < oldpts(mcu_start)-.25:
        index += 1
        continue
    if stamp > oldpts(mcu_stop)+.70:
        break
    registration = marker_registration(frame, reference)
    ids = set(map(int, registration['detected_marker_ids']))
    quad = registration_quad(registration)
    registered = quad is not None and {2, 3} <= ids
    row = dict(frame=index, time_video_s=stamp, calibration_valid=registered,
        decoded_ids=','.join(map(str, sorted(ids))), red_detected=False,
        red_x_mm=np.nan, red_y_mm=np.nan, candidates_on_board=0)
    if registered:
        matrix = np.linalg.inv(board_to_pose_transform(reference)) @ cv2.getPerspectiveTransform(quad, pose_world_corners(360.))
        components = colour_components(frame, 'red', geometry['marker_detection'])
        if components:
            coordinates = transform_points([value[0] for value in components], matrix)
            eligible = [(p, area) for p, (_, area) in zip(coordinates, components) if 0 <= p[0] <= 360 and 0 <= p[1] <= 360]
            row['candidates_on_board'] = len(eligible)
            if eligible:
                anchor = np.array([315., 160.]) if previous is None else previous
                point, area = min(eligible, key=lambda value:np.linalg.norm(value[0]-anchor))
                # 240fps・最高2.5m/sを想定する連続性ゲート。20ms超の欠測は追跡連続と見なさない。
                maxstep = 30. if previous is None else 4.+2500*min(.020, stamp-previous_stamp)
                if np.linalg.norm(point-anchor) <= maxstep:
                    row.update(red_detected=True, red_x_mm=float(point[0]), red_y_mm=float(point[1]), component_area_px=int(area))
                    previous, previous_stamp = point, stamp
    rows.append(row)
    index += 1
cap.release()
data = pd.DataFrame(rows)
out.mkdir(parents=True, exist_ok=False)
data.to_csv(out / 'red_observations.csv', index=False)
time = data.time_video_s.to_numpy()
valid = (data.calibration_valid & data.red_detected).to_numpy(bool)
floor = data[['red_x_mm', 'red_y_mm']].to_numpy()
before = valid & (time >= oldpts(mcu_start)-.22) & (time <= oldpts(mcu_start)-.06)
assert before.sum() >= 12
video_origin = np.median(floor[before], axis=0)
factor = (1500-10)/1500
entry_window = [oldpts(mcu_start)+.010, oldpts(turn_mcu)-.008]
entry_mask = valid & (time >= entry_window[0]) & (time <= entry_window[1])
entry, eq = line(floor[entry_mask])
basis = np.column_stack(([-entry[1], entry[0]], entry))
xy = factor*(floor-video_origin)@basis


def clock_fit(mask):
    def error(zero):
        return np.interp(time[mask]-zero, saved_time, wheel_distance)-xy[mask, 1]
    fit = minimize_scalar(lambda zero:np.mean(error(zero)**2), bounds=(oldzero-.025, oldzero+.025),
        method='bounded', options={'xatol':1e-10})
    return float(fit.x), float(np.sqrt(fit.fun))


zero, clockrms = clock_fit(entry_mask)
middle = np.median(time[entry_mask])
halves = [clock_fit(entry_mask & (time <= middle))[0], clock_fit(entry_mask & (time > middle))[0]]


def pts(mcu):
    return float(zero+(mcu-origin_mcu)/1000)


entry_window = [pts(mcu_start)+.010, pts(turn_mcu)-.008]
entry_mask = valid & (time >= entry_window[0]) & (time <= entry_window[1])
entry, eq = line(floor[entry_mask])
basis = np.column_stack(([-entry[1], entry[0]], entry))
xy = factor*(floor-video_origin)@basis
lend = zero+saved_time[-1]
exit_window = [lend+.015, pts(mcu_stop)-.025]
exit_mask = valid & (time >= exit_window[0]) & (time <= exit_window[1])
leaving, xq = line(xy[exit_mask])
angle = float(np.degrees(np.arctan2(leaving[0], leaving[1]))) % 360


def point(stamp):
    t = time[valid]
    idx = np.searchsorted(t, stamp)
    assert 0 < idx < len(t) and t[idx]-t[idx-1] <= .020
    return np.array([np.interp(stamp, t, axis[valid]) for axis in xy.T])


motion = (time >= pts(mcu_start)-.2) & (time <= pts(mcu_stop)+.1)
observation_rate = float(valid[motion].mean())
clock_pass = bool(clockrms <= 1.5 and abs(halves[0]-halves[1]) <= .002 and abs(zero-oldzero) < .024)
front_quality = bool(observation_rate >= .95 and eq['rms_mm'] <= .75 and xq['rms_mm'] <= .75 and clock_pass)
measured = point(lend)
theta = np.deg2rad(135.)
target_midpoint = np.array([90., 135.])
target_front = target_midpoint + 25*np.array([np.sin(theta), np.cos(theta)-1])
result = dict(mouse='A', motion='in_r135', state='FRONT_RED_RELATIVE_REFERENCE_ONLY', source_trial=str(run / 'trial.json'),
    coarse_report=str(args.report), body_pair_required_motion_quality_preserved=quality,
    body_pair_summary_present=(run / 'full/summary.json').is_file(),
    body_pair_full_quality_at_analysis=read(run / 'full/report.json')['required_motion_window_quality'] if (run / 'full/report.json').is_file() else None,
    observed_point='赤前点。欠測後点・中点・車軸を生成していない。', reference='自己静止原点・実測赤入口方向・尺度固定。右正/前正。',
    formula='delta_front_true=(1500-10)/1500*delta_front_floor; fixed camera foot cancels',
    registered_ids_required=[2, 3], per_frame_registration_required=True,
    evaluated_frames=int(motion.sum()), observed_frames=int(valid[motion].sum()), observation_rate=observation_rate,
    registration_rate=float(data.calibration_valid[motion].mean()),
    path_direction_change_deg=angle, path_direction_error_from135_deg=angle-135.,
    entry_line=eq, exit_line=xq, entry_window_pts_s=entry_window, exit_window_pts_s=exit_window,
    front_point_observation_quality_passed=front_quality,
    measured_front_Lend_right_forward_mm=measured.tolist(), approximate_front_target_right_forward_mm=target_front.tolist(),
    measured_minus_approximate_target_mm=(measured-target_front).tolist(),
    half_span_scenarios=[dict(assumed_halfspan_mm=radius, target_front_right_forward_mm=(target_midpoint+radius*np.array([np.sin(theta),np.cos(theta)-1])).tolist()) for radius in (20.,25.,30.)],
    clock=dict(samples=int(entry_mask.sum()), measured_tick_ms=float(tick), saved_zero_video_s=zero,
        change_from_model_clock_ms=(zero-oldzero)*1000, incoming_fit_rms_mm=clockrms,
        half_window_zero_spread_ms=abs(halves[0]-halves[1])*1000, timing_quality_passed=clock_pass,
        turn_or_exit_used_for_clock=False, encoder_scale_fitted=False, independent_hardware_synchronization=False),
    Lend_timing_sensitivity=[dict(delta_ms=delta, right_forward_mm=point(lend+delta/1000).tolist()) for delta in (-5,-1,0,1,5)],
    midpoint_reconstructed=False, midpoint_observation_quality_replaced=False,
    absolute_body_yaw_available=False, absolute_axle_position_available=False,
    precise_Lstart_Lend_adoption_allowed=False, parameter_adoption_allowed=False,
    limitations=['赤の独立参照を身体pair95%判定の代用にしない。', 'r25mmは操作者概算。135度での実測lever校正ではない。',
        '入口タイヤ距離と赤移動の一致を仮定。尺度・ピッチ/ロール・露光時刻・機軸と進行方向の差は残る。',
        '赤成分の位置連続性条件は診断用。誤認率は別途画像確認する。'],
    sources_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in
        (run / 'trial.json', run / 'saved_1ms.csv', run / 'saved_1ms.settings.json', run / 'telemetry.csv', run / 'marker_reference_used.json', run / 'body_geometry_used.json', args.report, Path(__file__))})
data.assign(front_relative_valid=valid, right_mm=xy[:,0], forward_mm=xy[:,1]).to_csv(out / 'front_relative_trajectory.csv', index=False)
save(out / 'front_reference.json', result)
plt.rcParams['font.family'] = 'Yu Gothic'
fig, ax = plt.subplots(figsize=(7, 7), layout='constrained')
plot = np.where(valid[:, None], xy, np.nan)
ax.plot(*plot.T, label='赤前点の高さ補正相対変位')
ax.scatter(*target_front, marker='x', c='black', label='概算r25mmの赤前Lend目標')
ax.scatter(*measured, c='red', label='入口距離だけで照合したLend')
ax.set(xlabel='入口に対する右 [mm]', ylabel='入口に対する前 [mm]', aspect='equal',
    title=f'機体A1800 右135°入口：赤前点参照 / 方向差 {angle:.3f}°')
ax.legend(fontsize=8); ax.grid(alpha=.3)
fig.savefig(out / 'front_reference.png', dpi=150)
plt.close(fig)
print(json.dumps({key:result[key] for key in ('path_direction_change_deg','path_direction_error_from135_deg',
    'observation_rate','front_point_observation_quality_passed','measured_front_Lend_right_forward_mm',
    'measured_minus_approximate_target_mm','clock')}, ensure_ascii=False), flush=True)
