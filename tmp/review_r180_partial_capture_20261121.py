"""1 msログ欠落試行を、原本のFAILEDを保持して動画と5 msログで診断する。"""
from pathlib import Path
import argparse
import hashlib
import json
import sys
import numpy as np
import pandas as pd
from scipy.optimize import minimize_scalar
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'tools'), str(ROOT/'tools/analysis')]
from video_marker_pose import read_video_pose, transform_points, board_to_pose_transform
from pixel_camera_geometry import relative_marker_midpoint

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--run', type=Path, required=True)
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
run, out = a.run.resolve(), a.output.resolve()
assert run.is_relative_to(ROOT) and out.is_relative_to(ROOT) and not out.exists()
read = lambda f: json.loads(f.read_text(encoding='utf-8-sig'))
trial, settings = read(run/'trial.json'), read(run/'motion.settings.json')
g, ref = read(run/'body_geometry_used.json'), read(run/'marker_reference_used.json')
assert trial['state'] == 'FAILED' and trial['saved_log']['status'] == 'failed'
assert trial['camera_state'] == 'PASSED' and trial['camera_run_start_confirmed']
assert trial['log_quality']['full_motion_retained']
assert trial['mouse'] == settings['mouse'] == g['mouse'] == 'A'
assert settings['motion_name'] == 'long_r180' and settings['turn_count'] == 1
out.mkdir(parents=True)
start = max(0, trial['video_motion_start_estimate_s']-1)
end = trial['recording_stop']['request_camera_seconds']
pose, _, _, _ = read_video_pose(Path(trial['video_file']), g['front_colour'],
    marker_reference=ref, video_window=(start, end), marker_detection=g.get('marker_detection'),
    rear_colour=g['rear_colour'], nominal_marker_span_mm=g.get('nominal_marker_span_mm'),
    marker_span_limits_mm=g.get('marker_span_limits_mm'), frame_stride=1,
    progress=lambda stage, n, pts: print(f'{stage}: {n} frames PTS {pts:.3f}', flush=True))
pose.to_csv(out/'actual_marker_pose.csv', index=False)
board = np.linalg.inv(board_to_pose_transform(ref))
front = transform_points(pose[['blue_x_mm','blue_y_mm']].to_numpy(), board)
rear = transform_points(pose[['yellow_x_mm','yellow_y_mm']].to_numpy(), board)
v = pose.marker_detected_raw.to_numpy(bool) & pose.calibration_valid.to_numpy(bool)
relative = relative_marker_midpoint(front, rear, trial['camera_height_mm'],
    g['front_height_mm'], g['rear_height_mm'], v)
xy, v = relative['displacement_xy_mm'], relative['valid']
t = pose.time_video_s.to_numpy()
baseline = v & (t <= start+.2)
assert baseline.sum() >= 12
xy -= np.median(xy[baseline], axis=0)
pd.DataFrame(dict(frame=pose.frame, time_video_s=t, observed=v,
    x_mm=xy[:,0], y_mm=xy[:,1])).to_csv(out/'actual_midpoint.csv', index=False)
raw = pd.read_csv(run/'motion.csv')
assert raw.period_ms.eq(5).all() and raw.lost_frames.eq(0).all()
mt = (raw.time_ms.to_numpy(float)-trial['log_quality']['motion_start_time_ms'])/1000
rad = abs(raw['ideal.rad_velo'].to_numpy(float))
turn = rad > max(rad)*.02
turn_start, turn_end = mt[turn][[0,-1]]
moving = np.flatnonzero(v & (np.linalg.norm(xy,axis=1)>2))
onset = next(t[i] for i in moving if v[i:i+3].all()
    and (np.linalg.norm(xy[i:i+3],axis=1)>2).all())

def line(mask):
    q=xy[mask]
    assert len(q)>=15, ('line observations',len(q))
    d=np.linalg.svd(q-q.mean(0),full_matrices=False)[2][0]
    if d@(q[-1]-q[0])<0:d=-d
    residual=q-q.mean(0); residual-=np.outer(residual@d,d)
    quality=dict(points=len(q),span_mm=float(np.ptp(q@d)),
        rms_mm=float(np.sqrt(np.mean(np.sum(residual**2,axis=1)))))
    assert quality['span_mm']>=30 and quality['rms_mm']<=.75,quality
    return d,quality

# 動画の動き出しと、MCU入口直進の時間幅だけで窓を選ぶ。
# 旋回や出口を使って時計・尺度・入口方向を合わせない。
entry_mask=v & (t>=onset+.005) & (t<=onset+turn_start-.035)
entry, entry_q=line(entry_mask)
basis=np.column_stack(([-entry[1],entry[0]],entry))
xy=xy@basis
distance=np.zeros(len(mt))
velocity=raw['ego.velo'].to_numpy(float)*1000
distance[1:]=np.cumsum((velocity[:-1]+velocity[1:])*.5*np.diff(mt))
distance-=np.interp(0,mt,distance)

def fit(mask):
    result=minimize_scalar(lambda z:np.mean((np.interp(t[mask]-z,mt,distance)-xy[mask,1])**2),
        bounds=(onset-.10,onset+.03),method='bounded')
    return float(result.x),float(np.sqrt(result.fun))

zero,rms=fit(entry_mask)
selected=np.flatnonzero(entry_mask); half=np.median(t[selected])
z1,_=fit(entry_mask&(t<=half));z2,_=fit(entry_mask&(t>half))
exit_mask=v & (t>=zero+turn_end+.025) & (t<=zero+turn_end+.095)
exit_direction,exit_q=line(exit_mask)
angle=float(np.degrees(np.arctan2(-exit_direction[0],exit_direction[1])))
if angle<0:angle+=360
exit_points=xy[exit_mask]
width=float(np.interp(45,*[exit_points[:,k][np.argsort(exit_points[:,1])] for k in (1,0)]))
crossing_contained=bool(exit_points[:,1].min()<=45<=exit_points[:,1].max())
pure=(t>=zero)&(t<=zero+mt[raw.time_ms.to_numpy()==trial['log_quality']['motion_end_time_ms']][0])
required=(t>=zero-.2)&(t<=zero+(trial['log_quality']['motion_end_time_ms']-trial['log_quality']['motion_start_time_ms'])/1000+.1)
quality=dict(actual_pair_rate=float(v[pure].mean()),required_pair_rate=float(v[required].mean()),
    registration=pose.attrs['marker_calibration'], detector=pose.attrs.get('body_detection',{}),
    entry_line=entry_q, exit_line=exit_q, entry_clock_rms_mm=rms,
    entry_clock_half_spread_ms=abs(z1-z2)*1000,exit_crossing_forward45_contained=crossing_contained)
result=dict(status='DIAGNOSTIC_ONLY_MISSING_1MS_LOG',source_trial=str(run/'trial.json'),
    original_capture_state=trial['state'],source_log_period_ms=5,
    clock_method='incoming_only_trapezoidal_live_wheel_speed_5ms_not_retained_1ms_encoder_clock',
    clock_zero_video_s=zero, video_onset_s=float(onset), turn_log_window_s=[turn_start,turn_end],
    entry_window_video_s=[float(t[entry_mask].min()),float(t[entry_mask].max())],
    exit_window_video_s=[float(t[exit_mask].min()),float(t[exit_mask].max())],
    diagnostic_width_at_forward45_mm=width if crossing_contained else None,
    diagnostic_path_angle_deg=angle, Lend_position=None,
    Lend_unavailable_reason='Retained 1ms log missing; 5ms phase boundary cannot establish +/-3mm Lend criterion',
    quality=quality, observations_imputed=False, scale_fitted=False,
    fixed_board_accepted=False, relative_geometry_accepted=False, parameter_adoption_allowed=False,
    battery_loaded_min_v=float(raw.loc[(mt>=0)&(mt<=.412),'Battery'].min()),
    input_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in
        [run/'trial.json',run/'motion.csv',run/'motion.settings.json',run/'body_geometry_used.json',
         run/'marker_reference_used.json',Path(trial['video_file']),Path(__file__)]})
with (out/'report.json').open('x',encoding='utf-8') as f:json.dump(result,f,ensure_ascii=False,indent=2)
fig,ax=plt.subplots(figsize=(6,6),layout='constrained')
ax.plot(xy[v,0],xy[v,1],'.',ms=2,label='Actual registered midpoint')
ax.plot(xy[entry_mask,0],xy[entry_mask,1],'.',label='Incoming reference')
ax.plot(xy[exit_mask,0],xy[exit_mask,1],'.',label='Exit diagnostic window')
ax.set(xlabel='Right [mm]',ylabel='Forward [mm]',title='Diagnostic only: retained 1 ms log unavailable')
ax.axis('equal');ax.grid();ax.legend();fig.savefig(out/'trajectory.png',dpi=150);plt.close(fig)
print(json.dumps(result,ensure_ascii=False),flush=True)
