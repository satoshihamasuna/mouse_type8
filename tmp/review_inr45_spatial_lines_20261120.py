"""保存済み動画の固定した空間窓から直線方向だけを参考評価する。Lend時刻は求めない。"""
from pathlib import Path
import hashlib
import json
import sys

import cv2
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

cv2.setNumThreads(4)
ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT/'tools/logs/pixel_camera/turn1800_A_inr45_20261120_v1'
OUT = BASE/'spatial_lines_diagnostic_v3'
sys.path[:0] = [str(ROOT/'tools'), str(ROOT/'tools/analysis')]
from video_marker_pose import read_video_pose, board_to_pose_transform, transform_points
from pixel_camera_relative_pose import load_pose_cache

TRIALS = [
    ('first', 'mid_status_retry_02', 'A_INR45_1800_mid_status_retry_20261120_002'),
    ('second', 'mid_exact_repeat_02', 'A_INR45_1800_mid_exact_repeat_20261120_002'),
    ('third_missing_1ms', 'mid_final_repeat_01', 'A_INR45_1800_mid_final_repeat_20261120_001'),
]

def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))

def save(path, value):
    with Path(path).open('x', encoding='utf-8') as f:
        json.dump(value, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.write('\n')

def digest(path):
    with Path(path).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def line(points):
    if len(points) < 10:
        raise ValueError('Fewer than 10 observations in fixed spatial window')
    d = np.linalg.svd(points-points.mean(0), full_matrices=False)[2][0]
    if d@(points[-1]-points[0]) < 0:
        d = -d
    r = points-points.mean(0)
    r -= np.outer(r@d, d)
    span = float(np.ptp(points@d))
    rms = float(np.sqrt(np.mean(np.sum(r*r, axis=1))))
    return d, dict(points=len(points), span_mm=span, rms_mm=rms,
                   passed=span >= 30 and rms <= .75)

OUT.mkdir(exist_ok=False)
plan = dict(video_window_s=[8.8, 10.6], frame_stride=1,
    third_video_window_s=[7.8, 10.6],
    static_origin_duration_s=.2, maximum_origin_scatter_rms_mm=1., entry_distance_from_origin_mm=[5., 80.],
    exit_forward_from_entry_mm=[185., 255.], entry_and_exit_min_points=10,
    minimum_line_span_mm=30., maximum_line_rms_mm=.75,
    purpose='Supplementary path direction only. No 1 ms timing, precise Lend, or parameter adoption.',
    original_trial_states_preserved=True,
    superseded_plan='v1 retained: insufficient exit observations; v2 retained: third recording starts motion earlier, so its 8.8..9.0s origin window was moving. Extend third source earlier and require a stationary origin. Spatial windows and line gates remain identical for all trials.')
save(OUT/'plan.json', plan)
results = []
fig, axes = plt.subplots(1, 2, figsize=(11, 7), layout='constrained')
for label, stage, identifier in TRIALS:
    run = BASE/stage/'local_execution'/('run_'+identifier)
    trial = read(run/'trial.json')
    ref = read(run/'marker_reference_used.json')
    geo = read(run/'body_geometry_used.json')
    assert trial['mouse']=='A' and trial['motion']=='in_r45' and trial['preset']==1800
    assert trial['camera_state']=='PASSED' and trial['camera_run_start_confirmed']
    assert trial['log_quality']['full_motion_retained']
    model = geo.get('marker_detection', {}).get('body_model', {})
    assert not model.get('enabled', bool(model))
    folder = OUT/label
    folder.mkdir()
    requested_window = plan['third_video_window_s'] if label=='third_missing_1ms' else plan['video_window_s']
    cache = run/'full_core_v1/apparent_floor_marker_pose.csv'
    if cache.exists():
        cached_window = read(cache.with_suffix('.provenance.json'))['video_window']
        window = [max(requested_window[0], cached_window[0]),
                  min(requested_window[1], cached_window[1])]
        pose = load_pose_cache(cache, trial, ref, geo, window, 1)
        source = 'existing verified full-core cache; spatial windows identical'
    else:
        pose, _, _, _ = read_video_pose(Path(trial['video_file']), geo['front_colour'],
            marker_reference=ref, video_window=requested_window,
            marker_detection=geo.get('marker_detection'), rear_colour=geo.get('rear_colour', 'yellow'),
            nominal_marker_span_mm=geo.get('nominal_marker_span_mm'),
            marker_span_limits_mm=geo.get('marker_span_limits_mm'), frame_stride=1,
            progress=lambda stage, count, pts: print(f'{stage}: {count} frames, PTS {pts:.3f}', flush=True))
        source = 'direct observed colour pairs with per-frame board registration'
    pose.to_csv(folder/'pose.csv', index=False)
    valid = pose.marker_detected_raw.to_numpy(bool) & pose.calibration_valid.to_numpy(bool)
    assert valid.mean() >= .95
    t = pose.time_video_s.to_numpy()
    inverse = np.linalg.inv(board_to_pose_transform(ref))
    f = transform_points(pose[['blue_x_mm','blue_y_mm']].to_numpy(), inverse).astype(float)
    r = transform_points(pose[['yellow_x_mm','yellow_y_mm']].to_numpy(), inverse).astype(float)
    mid = ((1-geo['front_height_mm']/1500)*f+(1-geo['rear_height_mm']/1500)*r)/2
    stationary = valid & (t>=requested_window[0]) & (t<=requested_window[0]+.2)
    assert stationary.sum() >= 12
    origin = np.median(mid[stationary], axis=0)
    origin_scatter = float(np.sqrt(np.mean(np.sum((mid[stationary]-origin)**2,axis=1))))
    assert origin_scatter <= plan['maximum_origin_scatter_rms_mm']
    centered = mid-origin
    radius = np.linalg.norm(centered, axis=1)
    incoming = valid & (radius>=5) & (radius<=80)
    d, entry = line(centered[incoming])
    basis = np.column_stack(([-d[1], d[0]], d))
    local = centered@basis
    outgoing = valid & (local[:,1]>=185) & (local[:,1]<=255)
    e, exit_ = line(local[outgoing])
    angle = float(np.degrees(np.arctan2(e[0],e[1])))
    xy = local.copy()
    xy[~valid] = np.nan
    colour = axes[0].plot(*xy.T, label=f'{label}: {angle:.3f} deg')[0].get_color()
    axes[1].plot(*xy.T, color=colour, label=label)
    axes[1].scatter(*local[outgoing].T, color=colour, s=8)
    result = dict(trial_id=identifier, original_state=trial['state'], source=source,
        observation_rate=float(valid.mean()), origin_scatter_rms_mm=origin_scatter,
        source_window_s=requested_window, entry=entry, exit=exit_,
        path_direction_change_deg=angle, spatial_line_gates_passed=entry['passed'] and exit_['passed'],
        precise_Lend_available=False, parameter_adoption_allowed=False,
        source_sha256={str(p):digest(p) for p in [run/'trial.json',run/'marker_reference_used.json',
            run/'body_geometry_used.json',Path(trial['video_file']),Path(__file__)]})
    pd.DataFrame(dict(time_video_s=t,right_mm=local[:,0],forward_mm=local[:,1],
        valid=valid,entry_used=incoming,exit_used=outgoing)).to_csv(folder/'relative.csv',index=False)
    save(folder/'review.json',result)
    results.append(result)
for ax in axes:
    ax.grid(alpha=.25)
    ax.set_aspect('equal')
    ax.set(xlabel='Right from spatial entry [mm]',ylabel='Forward from spatial entry [mm]')
axes[0].set_title('Fixed spatial windows / supplementary direction')
axes[0].legend(fontsize=8)
axes[1].set(xlim=(30,125),ylim=(175,260),title='Exit window: forward 185..255 mm')
fig.savefig(OUT/'spatial_directions.png',dpi=180)
save(OUT/'review.json',dict(plan=plan,results=results,robot_commands_sent=0,
    original_trials_modified=False,precise_Lend_available=False,parameter_adoption_allowed=False))
print(json.dumps([{'trial':r['trial_id'],'direction_deg':r['path_direction_change_deg'],
    'line_gates':r['spatial_line_gates_passed']} for r in results],indent=2),flush=True)
