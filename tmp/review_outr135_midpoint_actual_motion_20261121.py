"""IN135005の検証済みcacheから高さ補正中点と入口raw wheel時計を単発評価する。"""
from pathlib import Path
import hashlib
import json
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.optimize import minimize_scalar

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_outr135_20261120_v1'
RUN=Path(sys.argv[1]).resolve()
FULL=RUN/'full'
OUT=Path(sys.argv[2]).resolve()
assert OUT.is_relative_to(BASE.resolve())
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'tools/analysis')]
from pixel_camera_relative_pose import load_pose_cache, required_motion_quality
from pixel_camera_geometry import relative_marker_midpoint
from video_marker_pose import board_to_pose_transform, transform_points


def read(path):return json.loads(path.read_text(encoding='utf-8-sig'))


def sha(path):
    with path.open('rb') as file:return hashlib.file_digest(file,'sha256').hexdigest()


def line(points):
    if len(points)<15:raise ValueError(f'直線観測不足: {len(points)}')
    direction=np.linalg.svd(points-points.mean(0),full_matrices=False)[2][0]
    if direction@(points[-1]-points[0])<0:direction=-direction
    span=float(np.ptp(points@direction))
    if span<30:raise ValueError(f'直線span不足: {span}')
    residual=points-points.mean(0);residual-=np.outer(residual@direction,direction)
    return direction,dict(points=len(points),span_mm=span,rms_mm=float(np.sqrt(np.mean(np.sum(residual**2,axis=1)))))


assert not OUT.exists()
trial,settings,reference,geometry,report,summary=map(read,(RUN/'trial.json',RUN/'saved_1ms.settings.json',
    RUN/'marker_reference_used.json',RUN/'body_geometry_used.json',FULL/'report.json',FULL/'summary.json'))
assert trial['motion']==settings['motion_name']=='out_r135'
assert report['frame_stride']==1 and report['required_motion_window_quality']['accepted_for_relative_review']
assert report['required_motion_window_quality']['observation_rate']>=.95
assert report['required_motion_window_quality']['calibration_rate']>=.95
provenance=read(FULL/'apparent_floor_marker_pose.provenance.json')
assert set(provenance['marker_calibration']['used_marker_ids'])=={2,3}
pose=load_pose_cache(FULL/'apparent_floor_marker_pose.csv',trial,reference,geometry,provenance['video_window'],1)
raw,stream=pd.read_csv(RUN/'saved_1ms.csv'),pd.read_csv(RUN/'telemetry.csv')
assert np.array_equal(raw.cnt,np.arange(len(raw)))
signature=['ideal.velo','ego.velo','ideal.rad_velo','ego.rad_velo','V_r','V_l']
matches=raw.merge(stream[['time_ms']+signature],on=signature)
assert len(matches)>=18 and matches.cnt.is_unique
tick,origin=np.polyfit(matches.cnt,matches.time_ms,1)
transform=np.linalg.inv(board_to_pose_transform(reference))
front=transform_points(pose[['blue_x_mm','blue_y_mm']].to_numpy(),transform)
rear=transform_points(pose[['yellow_x_mm','yellow_y_mm']].to_numpy(),transform)
observed=pose.marker_detected_raw.to_numpy(bool)&pose.calibration_valid.to_numpy(bool)
relative=relative_marker_midpoint(front,rear,trial['camera_height_mm'],geometry['front_height_mm'],geometry['rear_height_mm'],observed)
xy_board,valid=relative['displacement_xy_mm'],relative['valid']
time=pose.time_video_s.to_numpy()
old=read(FULL/'relative_marker_midpoint.csv') if False else pd.read_csv(FULL/'relative_marker_midpoint.csv')
baseline=valid&(time<=provenance['video_window'][0]+.2)
recomputed=xy_board-np.nanmedian(xy_board[baseline],axis=0)
cache_difference=float(np.max(np.abs(recomputed[valid]-old[['center_dx_mm','center_dy_mm']].to_numpy()[valid])))
# Apparent coordinates pass through OpenCV float32 homographies and CSV.
# Bound only cache arithmetic round-trip by two float32 coordinate ULPs.
# Motion observation, line/clock/stationary and provenance guards are unchanged.
coordinate_magnitude=max(1.,float(np.nanmax(np.abs(np.r_[front[observed].ravel(),rear[observed].ravel()]))))
cache_float32_ulp_mm=float(np.spacing(np.float32(coordinate_magnitude)))
cache_roundtrip_bound_mm=max(1e-5,2*cache_float32_ulp_mm)
assert np.isfinite(cache_difference) and cache_difference<=cache_roundtrip_bound_mm,(cache_difference,cache_roundtrip_bound_mm)

start,stop=[trial['log_quality'][key] for key in ('motion_start_time_ms','motion_end_time_ms')]
oldzero=report['required_motion_window_quality']['video_motion_start_s']+(origin-start)/1000
turn_index=np.flatnonzero(raw['motion.pattern'].to_numpy()==13)[0]
turn_start_mcu=origin+tick*turn_index
saved_time=raw.cnt.to_numpy()*tick/1000
wheel_distance=np.cumsum((raw['Encoder_GetProperty_Right().sp_pulse']-raw['Encoder_GetProperty_Left().sp_pulse'])*15.02*np.pi/172/2).to_numpy()
zero=float(oldzero)
iterations=[]
for iteration in range(2):
    start_pts=zero+(start-origin)/1000
    before=valid&(time>=start_pts-.18)&(time<=start_pts-.04)
    assert before.sum()>=12
    video_origin=np.median(xy_board[before],axis=0)
    entry_window=[start_pts+.010,zero+(turn_start_mcu-origin)/1000-.008]
    entry_mask=valid&(time>=entry_window[0])&(time<=entry_window[1])
    entry,entry_q=line(xy_board[entry_mask])
    basis=np.column_stack(([-entry[1],entry[0]],entry))
    local=(xy_board-video_origin)@basis
    def fit(mask):
        def residual(value):return np.interp(time[mask]-value,saved_time,wheel_distance)-local[mask,1]
        optimum=minimize_scalar(lambda value:np.mean(residual(value)**2),bounds=(oldzero-.025,oldzero+.025),
            method='bounded',options={'xatol':1e-10})
        return float(optimum.x),residual(optimum.x)
    zero,errors=fit(entry_mask)
    median_stamp=np.median(time[entry_mask])
    halves=[fit(entry_mask&(time<=median_stamp))[0],fit(entry_mask&(time>median_stamp))[0]]
    clock_rms=float(np.sqrt(np.mean(errors**2)))
    half_spread=float(abs(halves[0]-halves[1])*1000)
    iterations.append(dict(iteration=iteration+1,saved_zero_video_s=zero,clock_rms_mm=clock_rms,
        half_window_zero_spread_ms=half_spread,entry_window_pts_s=entry_window,entry_line=entry_q))


def pts(mcu):return float(zero+(mcu-origin)/1000)


lend_time=zero+saved_time[-1]
original_exit_window=[lend_time+.015,pts(stop)-.025]
original_exit_mask=valid&(time>=original_exit_window[0])&(time<=original_exit_window[1])
actual_stationary_start=trial['recording_stop']['log_stop_evidence']['stationary_start_time_ms']
assert actual_stationary_start is not None
mcu_at_video=origin+(time-zero)*1000
actual_speed_at_video=np.interp(mcu_at_video,stream.time_ms.to_numpy(),stream['ego.velo'].to_numpy())
exit_motion_limit=pts(actual_stationary_start)-.025
speed_limit_m_s=.3
first_slow=np.flatnonzero((time>=lend_time+.015)&(time<=exit_motion_limit)&(actual_speed_at_video<=speed_limit_m_s))
if not len(first_slow):raise ValueError('?????????????????')
exit_window=[lend_time+.015,float(time[first_slow[0]])]
exit_mask=valid&(time>=exit_window[0])&(time<exit_window[1])&(actual_speed_at_video>speed_limit_m_s)

exit_direction,exit_q=line(local[exit_mask])
angle=float(np.degrees(np.arctan2(exit_direction[0],exit_direction[1]))%360)


def point(stamp):
    available=time[valid];i=np.searchsorted(available,stamp)
    if not 0<i<len(available) or available[i]-available[i-1]>.020:raise ValueError('20msを超える欠測または範囲外補間')
    return np.array([np.interp(stamp,available,axis[valid]) for axis in local.T])


target=np.array([135./np.sqrt(2),225./np.sqrt(2)])
measured=point(lend_time)


def lengths(stamp):
    measured=point(stamp)
    delta_le=(target[0]-measured[0])/exit_direction[0]
    delta_ls=target[1]-measured[1]-delta_le*exit_direction[1]
    return dict(midpoint_Lend_right_forward_mm=measured.tolist(),delta_Lstart_mm=float(delta_ls),delta_Lend_mm=float(delta_le),
        request_Lstart_mm=float(settings['parameters']['lstart']+delta_ls),request_Lend_mm=float(settings['parameters']['lend']+delta_le))


normal=np.array([-exit_direction[1],exit_direction[0]])
line_delta_ls=float((target@normal-np.median(local[exit_mask]@normal))/normal[1])
window_quality=required_motion_quality(time,valid,pose.calibration_valid.to_numpy(bool),stream.time_ms.to_numpy(),trial,
    zero+(float(stream.time_ms.iloc[0])-origin)/1000,1)
window_quality['timing_basis']='Incoming raw wheel distance only; no turn, exit or scale fit'
steady_window=[trial['recording_stop']['confirmation_camera_seconds']+.10,trial['recording_stop']['confirmation_camera_seconds']+.54]
steady=valid&(time>=steady_window[0])&(time<=steady_window[1])
assert steady.sum()>=12
scatter=float(np.sqrt(np.sum(np.var(local[steady],axis=0))))
timing_ok=clock_rms<=1.5 and half_spread<=2 and abs(zero-oldzero)<.024
eligible=bool(window_quality['accepted_for_relative_review'] and timing_ok and entry_q['rms_mm']<=.75 and exit_q['rms_mm']<=.75
    and scatter<=1 and trial['log_quality']['lost_frames']==0 and trial['log_quality']['crc_errors_during_trial']==0)
result=dict(schema_version=1,mouse='A',motion='out_r135',source_trial=str(RUN/'trial.json'),
    observed_point='height-corrected relative marker midpoint; not an absolute axle or body heading',
    method='validated cached apparent front/rear -> existing relative_marker_midpoint API -> measured entry axes -> incoming raw wheel clock only',
    heights=dict(camera_height_mm=1500.,front_height_mm=10.,rear_height_mm=1.,camera_foot_unknown=True,camera_foot_constant_cancels=True),
    nominal_half_span_used=False,absolute_axle_or_body_yaw_available=False,
    original_full_required_motion_quality=report['required_motion_window_quality'],original_summary=summary,
    original_trajectory_model_rms_mm=report['trajectory_fit_rmse_mm'],original_auto_stop_acceptance_replaced=False,
    recomputed_relative_cache_max_difference_mm=cache_difference,cache_provenance_verified=True,
    numerical_cache_equivalence=dict(coordinate_magnitude_mm=coordinate_magnitude,
        float32_coordinate_ulp_mm=cache_float32_ulp_mm,two_ulp_roundtrip_bound_mm=cache_roundtrip_bound_mm,
        actual_max_difference_mm=cache_difference,original_guard_mm=1e-5,
        note='Numerical cache comparison only; not physical precision and no change to motion quality thresholds.',
        original_helper_sha256='4a838af734b01bfa9ccdf784b9ca08054c7e8265497269f8a7b03f9fc9883735'),
    independent_clock_required_motion_quality=window_quality,independent_optical_geometry_eligible=eligible,
    entry_line=entry_q,exit_line=exit_q,entry_window_pts_s=entry_window,exit_window_pts_s=exit_window,
    exit_selection=dict(method='registered raw pose while actual ego.velo exceeds0.3m/s, from Lend+15ms to first speed crossing',
        speed_threshold_m_s=speed_limit_m_s,ideal_end_window_pts_s=original_exit_window,
        ideal_end_window_observations=int(original_exit_mask.sum()),actual_motion_observations=int(exit_mask.sum()),
        minimum_points_unchanged=15,minimum_span_mm_unchanged=30.,maximum_line_rms_mm_unchanged=.75,
        pose_inferences_counted_as_observations=0,actual_stationary_start_mcu_ms=actual_stationary_start),
    exit_path_direction_change_deg=angle,exit_path_error_from135_deg=angle-135.,
    Lend_video_pts_s=lend_time,midpoint_Lend_right_forward_mm=measured.tolist(),target_midpoint_Lend_right_forward_mm=target.tolist(),
    midpoint_Lend_residual_mm=(measured-target).tolist(),length_only_candidate_at_current_angle=lengths(lend_time),
    Lstart_delta_from_exit_line_mm=line_delta_ls,
    timing_sensitivity=[dict(delta_ms=delta,**lengths(lend_time+delta/1000)) for delta in (-5,-1,0,1,5)],
    clock=dict(saved_zero_video_s=zero,measured_tick_ms=float(tick),change_from_model_clock_ms=(zero-oldzero)*1000,
        incoming_raw_wheel_fit_rms_mm=clock_rms,half_window_zero_spread_ms=half_spread,passed=bool(timing_ok),
        exit_or_turn_used_for_fit=False,scale_fitted=False,independent_hardware_synchronization=False,iterations=iterations),
    late_stationary=dict(window_pts_s=steady_window,observations=int(steady.sum()),scatter_rms_mm=scatter,
        midpoint_right_forward_mm=np.median(local[steady],axis=0).tolist()),
    current_angle_correction_deg=settings['parameters']['correction'],precise_parameter_adoption_allowed=False,hardware_operations=0,source_params_changed=False,
    limitations=['全raw観測率とclock品質を保持。独立hardware同期、絶対尺度の実測校正ではない。',
        'camera/各marker高さが一定と仮定。前後マーカーの物理中点を車軸へ同一視しない。',
        '長さ解は現在138度出口方向を保つ仮定。補正+3.5度は終端を変えるため再計測が必要。',
        '元summaryの初期直進窓不足と軌道モデルRMS17.77mmを別評価として保持。'],
    input_sha256={str(path):sha(path) for path in (RUN/'trial.json',RUN/'saved_1ms.csv',RUN/'saved_1ms.settings.json',RUN/'telemetry.csv',
        FULL/'report.json',FULL/'summary.json',FULL/'apparent_floor_marker_pose.csv',FULL/'apparent_floor_marker_pose.provenance.json',
        RUN/'marker_reference_used.json',RUN/'body_geometry_used.json',Path(__file__))})
result['current_parameters']=settings['parameters']
result['source_analysis_template']=str(ROOT/'tmp/parallel_inr135_midpoint005_20261120.py')
result['limitations']=[
    '全raw観測率と時計品質を保持。独立hardware同期・絶対尺度の校正ではない。',
    '高さ一定を仮定した相対マーカー中点。車軸・機軸の絶対校正ではない。',
    '長さ解は今回の出口方向・半径・角度補正を維持する局所候補。変更後は再計測する。',
    '元のsummaryと軌跡モデルRMSは別評価として保持。'
]
OUT.mkdir(exist_ok=False)
with (OUT/'midpoint_review.json').open('x',encoding='utf-8') as file:json.dump(result,file,ensure_ascii=False,indent=2,allow_nan=False);file.write('\n')
pd.DataFrame(dict(frame=pose.frame,time_video_s=time,registered_midpoint_valid=valid,right_mm=local[:,0],forward_mm=local[:,1])).to_csv(OUT/'midpoint_relative_trajectory.csv',index=False)
pd.DataFrame(dict(time_video_s=time[entry_mask],wheel_forward_mm=local[entry_mask,1]+errors,midpoint_forward_mm=local[entry_mask,1],residual_mm=errors)).to_csv(OUT/'entry_clock_fit.csv',index=False)
plt.rcParams['font.family']='Yu Gothic'
fig,ax=plt.subplots(figsize=(6,7),layout='constrained')
mask=(time>=pts(start)-.04)&(time<=pts(stop)+.1)
ax.plot(*np.where(valid[:,None],local,np.nan)[mask].T,label='高さ補正相対マーカー中点')
ax.scatter(*measured,c='red',label='入口raw wheelで時計照合したLend')
ax.scatter(*target,marker='x',c='black',label='目標中点95.46/159.10mm')
ax.set(xlabel='入口に対する右 [mm]',ylabel='入口に対する前 [mm]',aspect='equal',title=f'OUT135 midpoint / {angle:.3f}°')
ax.legend(fontsize=8);ax.grid(alpha=.3);fig.savefig(OUT/'midpoint_trajectory.png',dpi=140);plt.close(fig)
print(json.dumps({key:result[key] for key in ('independent_optical_geometry_eligible','exit_path_direction_change_deg',
    'midpoint_Lend_right_forward_mm','midpoint_Lend_residual_mm','length_only_candidate_at_current_angle',
    'Lstart_delta_from_exit_line_mm','clock','late_stationary')},ensure_ascii=False,indent=2),flush=True)
