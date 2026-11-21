"""光学速度による直線窓の診断。Lend同期・パラメータ採用には使わない。"""
from pathlib import Path
import json
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

folder = Path(sys.argv[1])
p = pd.read_csv(folder / 'relative_midpoint_diagnostic.csv')
t = p.time_video_s.to_numpy()
xy = p[['x_mm', 'y_mm']].to_numpy()
valid = p.raw_registered.to_numpy(bool)
velocity = np.full(len(p), np.nan)
for i in range(3, len(p)-3):
    if valid[i-3:i+4].all() and t[i+3]-t[i-3] < .03:
        velocity[i] = np.linalg.norm(xy[i+3]-xy[i-3]) / (t[i+3]-t[i-3])
active = np.flatnonzero(velocity > 300)
assert len(active) > 30
groups = np.split(active, np.flatnonzero(np.diff(t[active]) > .06)+1)
groups = [g for g in groups if len(g) >= 10]
assert groups
# Missing marker pairs can split one short motion; keep both straight phases.
start, end = float(t[groups[0][0]]), float(t[groups[-1][-1]])
assert end-start <= .7, 'Multiple motion episodes require manual review'

def line(mask):
    q = xy[mask]
    assert len(q) >= 15, len(q)
    d = np.linalg.svd(q-q.mean(0), full_matrices=False)[2][0]
    if d@(q[-1]-q[0]) < 0:
        d = -d
    residual = q-q.mean(0)
    residual -= np.outer(residual@d, d)
    span = float(np.ptp(q@d))
    rms = float(np.sqrt(np.mean(np.sum(residual**2, axis=1))))
    return d, dict(points=len(q), span_mm=span, rms_mm=rms,
        passed=bool(span >= 30 and rms <= .75))

entries = []
for entry_width in (.08, .09, .10):
    entry_window = [start+.015, start+.015+entry_width]
    em = valid & (t >= entry_window[0]) & (t <= entry_window[1])
    for exit_width in (.06, .07, .08, .09):
        exit_window = [end-.005-exit_width, end-.005]
        xm = valid & (t >= exit_window[0]) & (t <= exit_window[1])
        try:
            a, aq = line(em)
            b, bq = line(xm)
            angle = float(np.degrees(np.arctan2(a[0]*b[1]-a[1]*b[0], a@b)) % 360)
            entries.append(dict(entry_window_s=entry_window, exit_window_s=exit_window,
                entry=aq, exit=bq, angle_deg=angle))
        except AssertionError as error:
            entries.append(dict(entry_window_s=entry_window, exit_window_s=exit_window, error=str(error)))
required = (t >= start-.2) & (t <= end+.1)
out = dict(schema_version=2, state='OPTICAL_DIRECTION_DIAGNOSTIC_ONLY', optical_speed_threshold_mm_s=300,
    optical_speed_difference_frames=6, optical_moving_window_s=[start, end],
    required_video_interval_s=[start-.2, end+.1],
    required_raw_registered_observation_rate=float(valid[required].mean()),
    direction_window_sensitivity=entries, Lend_position_available=False,
    full_motion_log_available=False, parameter_adoption_allowed=False,
    limitations=['Time windows derive from optical motion, not MCU Lend synchronization.',
        'Relative marker midpoint path direction; no absolute body yaw or axle calibration.',
        'All original trial quality flags remain unchanged.'])
with (folder / 'direction_diagnostic_v2.json').open('x', encoding='utf-8') as f:
    json.dump(out, f, indent=2, allow_nan=False)
fig, axs = plt.subplots(1, 2, figsize=(10, 5), layout='constrained')
axs[0].plot(*np.where(valid[:, None], xy, np.nan).T, '.', markersize=2)
axs[0].set(xlabel='Registered X [mm]', ylabel='Registered Y [mm]', aspect='equal')
axs[1].plot(t, velocity)
axs[1].axhline(300, color='k', linestyle='--')
axs[1].set(xlim=(start-.2, end+.2), xlabel='Video PTS [s]', ylabel='Optical speed [mm/s]')
for ax in axs:
    ax.grid(alpha=.3)
fig.savefig(folder / 'optical_motion_v2.png', dpi=140)
print(json.dumps(dict(moving_window_s=[start, end], raw_rate=out['required_raw_registered_observation_rate'],
    accepted_line_angles=[round(e['angle_deg'], 6) for e in entries if e.get('entry', {}).get('passed')
        and e.get('exit', {}).get('passed')], adoption_allowed=False)), flush=True)
