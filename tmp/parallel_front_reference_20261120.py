"""赤前マーカーの独立相対参照と剛体自己校正を保存資料で検証する。実機操作なし。"""
from pathlib import Path
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
BASE = ROOT/'tools/logs/pixel_camera/turn1800_A_inr45_20261120_v1'
OUT = BASE/'parallel_front_reference'
RUN = BASE/'baseline01/local_execution/run_A_INR45_1800_baseline_20261120_002'
RED = BASE/'parallel_analysis/red_direction_diagnostic_v2'
COMPARISON = ROOT/'tools/logs/pixel_camera/turn1800_A_r90_20261119_v1/comparison_c1p25_three_v2/video_comparison.json'
sys.path[:0] = [str(ROOT/'tools/analysis'), str(ROOT/'tools')]
from video_marker_pose import board_to_pose_transform, transform_points


def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def sha(path):
    result = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(8*1024*1024), b''):
            result.update(chunk)
    return result.hexdigest()


def save(path, value):
    with path.open('x', encoding='utf-8') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')


def line(points):
    assert len(points) >= 15
    direction = np.linalg.svd(points-points.mean(0), full_matrices=False)[2][0]
    if direction@(points[-1]-points[0]) < 0:
        direction = -direction
    span = float(np.ptp(points@direction))
    assert span >= 30
    residual = points-points.mean(0)
    residual -= np.outer(residual@direction, direction)
    return direction, dict(points=len(points), span_mm=span,
                           rms_mm=float(np.sqrt(np.mean(np.sum(residual**2, 1)))))


def point(time, xy, valid, stamp):
    t, values = time[valid], xy[valid]
    idx = np.searchsorted(t, stamp)
    assert 0 < idx < len(t) and t[idx]-t[idx-1] <= .020
    return np.array([np.interp(stamp, t, values[:,i]) for i in range(2)])


def target(theta, radius, midpoint):
    return np.asarray(midpoint)+radius*np.array([np.sin(theta), np.cos(theta)-1])


if OUT.exists():
    raise FileExistsError(OUT)
OUT.mkdir()
af, ar, beta = 1490/1500, 1499/1500, 9/1500
inputs = [COMPARISON, RED/'diagnostic.json', RED/'red_observations.csv', RUN/'saved_1ms.csv',
          RUN/'telemetry.csv', RUN/'trial.json', RUN/'full/report.json', Path(__file__)]

# 数学の検証: 任意の移動と旋回、既知のCで生成した観測からC不要の変位/初期barを回復。
angle = np.linspace(0, np.pi/2, 101)
mid = np.column_stack((np.sin(angle)*70, np.linspace(0,180,101)))
d = 50*np.column_stack((np.sin(angle),np.cos(angle)))
camera = np.array([180.,210.])
ftrue, rtrue = mid+d/2, mid-d/2
ffloor = (ftrue-(1-af)*camera)/af
rfloor = (rtrue-(1-ar)*camera)/ar
delta = af*(ffloor-ffloor[0])-ar*(rfloor-rfloor[0])
d0 = np.linalg.lstsq(2*delta, -np.sum(delta**2,1), rcond=None)[0]
assert np.allclose(af*(ffloor-ffloor[0]), ftrue-ftrue[0], atol=1e-10)
assert np.allclose(d0,d[0],atol=1e-10)
assert np.allclose((d0-(af*ffloor[0]-ar*rfloor[0]))/beta,camera,atol=1e-8)

# R90同条件3本。元動画/CSV/基準/検出設定と毎frame登録maskのprovenanceを照合。
cases = [v for v in read(COMPARISON)['comparisons'] if v['camera_settings']['ev']==-1.5
         and v['correction_deg']==1.25]
assert len(cases)==3
observations, rigid, empirical = [], [], []
for index, case in enumerate(cases):
    run = Path(case['source_trial']).parent
    cache = run/'full/apparent_floor_marker_pose.csv'
    provenance_path = cache.with_suffix('.provenance.json')
    provenance = read(provenance_path)
    trial, geometry, reference = read(run/'trial.json'), read(run/'full/body_geometry_used.json'), read(run/'marker_reference_used.json')
    inputs.extend([run/'trial.json',cache,provenance_path,run/'full/body_geometry_used.json',run/'marker_reference_used.json'])
    assert provenance['video_sha256']==sha(Path(trial['video_file']))
    assert provenance['cached_pose_sha256']==sha(cache)
    assert provenance['reference_digest']==hashlib.sha256(json.dumps(reference,sort_keys=True).encode()).hexdigest()
    assert provenance['marker_detection']==geometry['marker_detection']
    assert provenance['marker_calibration']['per_frame_registration_required'] is True
    assert set(provenance['marker_calibration']['used_marker_ids'])=={2,3}
    assert provenance['frame_stride']==1 and case['relative_observation_quality_passed']
    data = pd.read_csv(cache)
    valid = (data.marker_detected_raw & data.calibration_valid).to_numpy(bool)
    time = data.time_video_s.to_numpy()
    # floor列は実測pairのフレームのみ利用。欠測への既存補間値はvalidで全除外。
    f = data[['front_floor_x_mm','front_floor_y_mm']].to_numpy()
    r = 2*data[['midpoint_floor_x_mm','midpoint_floor_y_mm']].to_numpy()-f
    matrix = np.linalg.inv(board_to_pose_transform(reference))
    f, r = transform_points(f,matrix).astype(float), transform_points(r,matrix).astype(float)
    before = valid & (time<case['entry_window_pts_s'][0]-.1)
    assert before.sum()>=12
    f0, r0 = np.median(f[before],0), np.median(r[before],0)
    D = af*f-ar*r; D0 = af*f0-ar*r0
    delta = af*(f-f0)-ar*(r-r0)
    selected = valid & (time>=case['entry_window_pts_s'][0]) & (time<=case['exit_window_pts_s'][1])
    d0 = np.linalg.lstsq(2*delta[selected],-np.sum(delta[selected]**2,1),rcond=None)[0]
    lengths = np.linalg.norm(delta[selected]+d0,axis=1)
    rigid.append(dict(label=f'c1p25_{index+1:02}',source_trial=str(run/'trial.json'),
        samples=int(selected.sum()),initial_bar_vector_mm=d0.tolist(),bar_length_mm=float(np.linalg.norm(d0)),
        rigid_length_std_mm=float(np.std(lengths)),rigid_length_range_mm=float(np.ptp(lengths)),
        design_condition=float(np.linalg.cond(2*delta[selected])),
        inferred_camera_foot_board_mm=((d0-D0)/beta).tolist(),
        inferred_camera_foot_is_physical_measurement=False))
    entry_mask = valid & (time>=case['entry_window_pts_s'][0]) & (time<=case['entry_window_pts_s'][1])
    entry, entryq = line(f[entry_mask])
    basis = np.column_stack(([-entry[1],entry[0]],entry))
    shift = point(time,delta,valid,case['estimated_Lend_video_pts_s'])@basis/2
    theta=np.radians(case['entry_to_exit_path_rotation_deg'])
    direction=np.array([np.sin(theta),np.cos(theta)-1])
    effective_radius=float(shift@direction/(direction@direction))
    empirical.append(dict(label=f'c1p25_{index+1:02}',measured_front_minus_midpoint_displacement_mm=shift.tolist(),
        exit_path_angle_deg=float(np.degrees(theta)),effective_terminal_lever_mm=effective_radius,
        nominal25_conversion_error_mm=float(np.linalg.norm(shift-25*direction)),
        limitations='Includes initial/final body-to-path angle, detection/registration and timing errors; not a physical half-span measurement.'))
    observations.append(dict(D=D[selected],shift=shift,direction=direction))

# 初めの2本だけでC/長さをfitし、3本目には再fitせず評価。
train=np.concatenate([v['D'] for v in observations[:2]])
design=np.column_stack((2*train,-np.ones(len(train))))
solution=np.linalg.lstsq(design,-np.sum(train**2,1),rcond=None)[0]
g=solution[:2]; length=float(np.sqrt(g@g+solution[2]))
held_length=np.linalg.norm(observations[2]['D']+g,axis=1)-length
radius=sum(v['shift']@v['direction'] for v in observations[:2])/sum(v['direction']@v['direction'] for v in observations[:2])
held_error=observations[2]['shift']-radius*observations[2]['direction']
holdout=dict(training_labels=['c1p25_01','c1p25_02'],holdout_label='c1p25_03',refit_on_holdout=False,
    inferred_camera_foot_board_mm=(g/beta).tolist(),bar_length_mm=length,design_condition=float(np.linalg.cond(design)),
    holdout_length_rms_mm=float(np.sqrt(np.mean(held_length**2))),holdout_length_abs95_mm=float(np.percentile(abs(held_length),95)),
    effective_terminal_lever_from_training_mm=float(radius),
    holdout_front_target_conversion_error_vector_mm=held_error.tolist(),holdout_front_target_conversion_error_norm_mm=float(np.linalg.norm(held_error)),
    physical_camera_foot_or_half_span_adoption_allowed=False,
    reason='Individual clip solutions differ by hundreds ofmm and the pooled linear system is ill-conditioned. Effective terminal lever validation is R90-only and not a proof for45/135 or axle geometry.')

# in_r45の赤単独相対変位。欠測rearは生成せず、登録ArUco2/3の実復号CSVを使用。
diagnostic=read(RED/'diagnostic.json'); data=pd.read_csv(RED/'red_observations.csv')
decoded=np.array([{2,3}<=set(map(int,str(s).split(','))) for s in data.decoded_ids])
valid=(data.calibration_valid&data.red_detected).to_numpy(bool)&decoded
time=data.time_video_s.to_numpy(); front=data[['red_x_mm','red_y_mm']].to_numpy()
assert valid.mean()>=.95
entry_mask=valid&(time>=diagnostic['entry_window_pts_s'][0])&(time<=diagnostic['entry_window_pts_s'][1])
exit_mask=valid&(time>=diagnostic['exit_window_pts_s'][0])&(time<=diagnostic['exit_window_pts_s'][1])
entry,eq=line(front[entry_mask]); leaving,xq=line(front[exit_mask])
basis=np.column_stack(([-entry[1],entry[0]],entry))
before=valid&(time>=time[0]+.02)&(time<=diagnostic['entry_window_pts_s'][0]-.04)
assert before.sum()>=12
origin=np.median(front[before],axis=0)
xy=af*(front-origin)@basis; xy[~valid]=np.nan
raw,stream=pd.read_csv(RUN/'saved_1ms.csv'),pd.read_csv(RUN/'telemetry.csv')
signature=['ideal.velo','ego.velo','ideal.rad_velo','ego.rad_velo','V_r','V_l']
matches=raw.merge(stream[['time_ms']+signature],on=signature)
tick, mcu_origin=np.polyfit(matches.cnt,matches.time_ms,1)
trial, oldreport=read(RUN/'trial.json'),read(RUN/'full/report.json')
oldzero=oldreport['required_motion_window_quality']['video_motion_start_s']+(mcu_origin-trial['log_quality']['motion_start_time_ms'])/1000
saved_time=raw.cnt.to_numpy()*tick/1000
wheel_distance=np.cumsum((raw['Encoder_GetProperty_Right().sp_pulse']-raw['Encoder_GetProperty_Left().sp_pulse'])*15.02*np.pi/172/2).to_numpy()


def fit_clock(mask):
    def residual(zero):
        return np.interp(time[mask]-zero,saved_time,wheel_distance)-xy[mask,1]
    fit=minimize_scalar(lambda z:np.mean(residual(z)**2),bounds=(oldzero-.025,oldzero+.025),method='bounded',options={'xatol':1e-10})
    return float(fit.x),float(np.sqrt(fit.fun))


zero,clock_rms=fit_clock(entry_mask)
middle=np.median(time[entry_mask]); halves=[fit_clock(entry_mask&(time<=middle))[0],fit_clock(entry_mask&(time>middle))[0]]
lend_time=zero+saved_time[-1]
measured=point(time,xy,valid,lend_time)
nominal=target(np.pi/4,25,[45,180])
sensitivities=[]
for r in (20.,25.,float(radius),30.):
    nominal_r=target(np.pi/4,r,[45,180])
    sensitivities.append(dict(half_span_scenario_mm=r,target_front_Lend_mm=nominal_r.tolist(),measured_minus_target_mm=(measured-nominal_r).tolist(),
                             scenario_is_measured_physical_radius=False))
clock=dict(method='Incoming raw wheel pulse distance versus front-red displacement only; no turn/exit used, no scale fit',
    samples=int(entry_mask.sum()),measured_tick_ms=float(tick),saved_zero_video_s=zero,
    change_from_existing_path_fit_ms=float((zero-oldzero)*1000),incoming_fit_rms_mm=clock_rms,
    half_window_zero_spread_ms=float(abs(halves[0]-halves[1])*1000),lend_video_pts_s=float(lend_time),
    timing_quality_passed=bool(clock_rms<=1.5 and abs(halves[0]-halves[1])<=.002),
    independent_hardware_synchronization=False,encoder_scale_fitted=False,
    assumptions=['A tire diameter15.02mm,172counts/rev; straight wheel displacement equals red displacement, fixed camera/height.',
                 'Front marker pitch, wheel slip, exposure timing and absolute scale remain unverified.'])
result=dict(mouse='A',motion='in_r45',state='FRONT_RED_RELATIVE_REFERENCE_ONLY',
    observed_point='front-red at height10mm; not a measured/reconstructed marker midpoint or axle',
    reference='Own stationary red origin and measured incoming red-path direction; local right positive, forward positive',
    displacement_formula='delta_front_true=(1500-10)/1500 * delta_front_floor; camera foot cancels',
    nominal_target_formula='front_target=midpoint_target+r*(sin(theta),cos(theta)-1), nominal initial body direction aligned with incoming path',
    required_registered_aruco_ids=[2,3],observed_frames=int(valid.sum()),evaluated_frames=len(data),observation_rate=float(valid.mean()),
    thresholds=dict(observation_rate=.95,minimum_line_points=15,minimum_line_span_mm=30),
    entry_line=eq,exit_line=xq,path_direction_change_deg=float(np.degrees(np.arctan2((leaving@basis)[0],(leaving@basis)[1]))),
    camera_height_mm=1500.,front_height_mm=10.,nominal_half_span_mm=25.,nominal_half_span_is_approximate=True,
    measured_front_Lend_right_forward_mm=measured.tolist(),nominal_front_target_right_forward_mm=nominal.tolist(),
    nominal_front_target_residual_mm=(measured-nominal).tolist(),half_span_scenarios=sensitivities,clock=clock,
    Lend_timing_sensitivity=[dict(delta_ms=dt,right_forward_mm=point(time,xy,valid,lend_time+dt/1000).tolist()) for dt in (-5,-1,0,1,5)],
    front_point_observation_quality_passed=True,missing_rear_generated=False,midpoint_observation_quality_passed=False,
    absolute_body_yaw_available=False,absolute_axle_position_available=False,
    precise_Lstart_Lend_adoption_allowed=False,firmware_parameters_changed=False,
    use='Available for comparing same-reference front-point trajectories/directions and explicitly approximate candidate guidance. Do not reuse the midpoint report pass/fail or treat an approximate r target as precise axle geometry.',
    limitations=['既存の中点観測71.6%不合格は保持。赤単独98.765%は別参照点の観測率。',
                 'r25mmは操作者概算。R90の経験lever約27mmは45度への採用値ではない。',
                 '初期/出口の機軸と進行方向の差、ピッチ/ロール、歪み、H/hの誤差を含む。',
                 '自己校正のcamerafootは不安定で、実測camerafootやbody yaw校正へ採用しない。'])
data.assign(front_relative_valid=valid,right_mm=xy[:,0],forward_mm=xy[:,1]).to_csv(OUT/'inr45_front_relative_trajectory.csv',index=False)
hashes={str(p):sha(p) for p in inputs}
save(OUT/'rigid_selfcalibration.json',dict(synthetic_recovery_passed=True,per_clip=rigid,holdout=holdout,empirical_terminal_conversion=empirical,input_sha256=hashes))
save(OUT/'front_reference.json',result)
plt.rcParams['font.family']='Yu Gothic'
fig,axes=plt.subplots(1,2,figsize=(11,5),constrained_layout=True)
axes[0].plot(xy[:,0],xy[:,1],label='実測 赤前点（高さ補正相対変位）')
axes[0].scatter(*nominal,c='black',marker='x',label='名目赤前Lend r25mm')
axes[0].scatter(*measured,c='red',label='前走だけで時刻照合したLend')
axes[0].set(xlabel='入口に対する右 [mm]',ylabel='入口に対する前 [mm]',aspect='equal',title='in_r45 / 赤前点の独立相対参照')
axes[0].grid();axes[0].legend(fontsize=8)
axes[1].plot([1,2,3],[v['bar_length_mm'] for v in rigid],'o-',label='各動画の剛体LS推定bar長')
axes[1].axhline(50,c='.5',ls='--',label='ユーザー概算50mm（真値ではない）')
axes[1].set(xlabel='R90同条件動画',ylabel='bar長 [mm]',title='自己校正は動画間で不安定')
axes[1].set_xticks([1,2,3]);axes[1].grid();axes[1].legend(fontsize=8)
fig.savefig(OUT/'front_reference_review.png',dpi=150);plt.close(fig)
assert all(sha(Path(p))==value for p,value in hashes.items())
print(json.dumps(dict(state=result['state'],observed_frames=result['observed_frames'],front_target=nominal.tolist(),front_Lend=measured.tolist(),clock=clock,
    holdout=holdout,source_unchanged=True),ensure_ascii=False,indent=2))
