"""機体A・右45°入口の動画と1ms位置ログを入口直線基準で比較する。"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt
import numpy as np
import pandas as pd

from review_turn1600_A_r180_onboard_slip_html_20261119 import image, table

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools/analysis'))
from video_marker_pose import board_to_pose_transform, fit_time_and_pose, rigid_fit
from turn_slip_conditions import observe


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def line_fit(points):
    if len(points)<15:
        raise ValueError('直線を評価する登録済み観測が不足しています')
    centered=points-points.mean(axis=0)
    direction=np.linalg.svd(centered,full_matrices=False)[2][0]
    if direction@(points[-1]-points[0])<0:
        direction=-direction
    along=centered@direction
    span=float(np.ptp(along))
    if span<30:
        raise ValueError('直線の観測範囲が30mm未満です')
    rms=float(np.sqrt(np.mean(np.sum((centered-along[:,None]*direction)**2,axis=1))))
    return direction,dict(points=len(points),span_mm=span,rms_mm=rms)


def align(trial, saved, stream, pose, xy, valid, reference):
    signature=['ideal.velo','ego.velo','ideal.rad_velo','ego.rad_velo','V_r','V_l']
    match=saved.merge(stream[['time_ms']+signature],on=signature)
    assert len(match)>=18 and match.cnt.is_unique
    period,origin=np.polyfit(match.cnt,match.time_ms,1)
    assert .95<period<1.05
    start,stop=(trial['log_quality'][k] for k in ('motion_start_time_ms','motion_end_time_ms'))
    still=trial['recording_stop']['log_stop_evidence']['stationary_start_time_ms']
    live=stream.loc[stream.time_ms.between(start-500,still+1500)]
    zero=live.time_ms.iloc[0]
    t=(live.time_ms.to_numpy()-zero)/1000
    omega=live['ego.rad_velo'].to_numpy()
    average=(live['ego.velo']-.010*live['ego.accel']).to_numpy()*1000/period
    velo=np.interp(t+9.5*period/1000,t,average)
    theta=np.r_[0,np.cumsum(.5*(omega[:-1]+omega[1:])*np.diff(t))]
    yaw=.5*(theta[:-1]+theta[1:])
    travel=.5*(velo[:-1]+velo[1:])*np.diff(t)
    position=np.column_stack([np.r_[0,np.cumsum(-travel*np.sin(yaw))],
                              np.r_[0,np.cumsum(travel*np.cos(yaw))]])
    log=pd.DataFrame(dict(time_log_s=t,x_local_mm=position[:,0],y_local_mm=position[:,1],
                          heading_local_rad=theta,speed_log_mm_s=velo,omega_log_rad_s=omega))
    transform=board_to_pose_transform(reference)[:2,:2]
    points=xy@transform.T
    filled=pd.DataFrame(np.where(valid[:,None],points,np.nan)).interpolate(limit_direction='both').to_numpy()
    image_pose=pose.assign(center_x_mm=filled[:,0],center_y_mm=filled[:,1],
        speed_playback_mm_s=np.linalg.norm(np.gradient(filled,pose.time_video_s,axis=0),axis=1),
        marker_detected_raw=valid)
    need=((start-zero)/1000-.2,(stop-zero)/1000+.1)
    _,_,coarse,_,_,_=fit_time_and_pose(image_pose,log,1.,need)
    time=pose.time_video_s.to_numpy()
    best=None
    for offset in np.linspace(coarse-.012,coarse+.012,161):
        elapsed=time-offset
        ok=valid&(elapsed>=t[0])&(elapsed<=t[-1])
        model=np.column_stack([np.interp(elapsed[ok],t,x) for x in position.T])
        rotation,translation,rms=rigid_fit(model,points[ok])
        if best is None or rms<best[0]:
            diff=np.linalg.norm(model@rotation+translation-points[ok],axis=1)
            motion=(elapsed[ok]>=(start-zero)/1000)&(elapsed[ok]<=(stop-zero)/1000)
            best=(rms,offset,float(np.sqrt(np.mean(diff[motion]**2))))
    rms,offset,motion_rms=best
    return (lambda mcu:float(offset+(mcu-zero)/1000)),period,origin,rms,motion_rms


def onboard_positions(saved, lend):
    position=saved[['ego.turn_x','ego.turn_y']].to_numpy()*[-1,1]
    rate=saved['ideal.rad_velo'].to_numpy()
    yaw=saved['ego.radian'].to_numpy()
    travel=saved['ego.length'].to_numpy()
    pattern=saved['motion.pattern'].to_numpy()
    pre=np.flatnonzero(pattern==1)
    turn=np.flatnonzero(pattern==7)
    resets=np.flatnonzero((np.arange(len(saved))>np.argmax(abs(rate)))&(rate==0)&(pattern==7)&(travel==0))
    assert len(resets)==1 and pre[-1]+2==turn[0] and turn[-1]+1==len(saved)-1
    reset=resets[0]
    def rotation(theta):
        c,s=np.cos(theta),np.sin(theta)
        return np.array([[c,-s],[s,c]])
    in_yaw=yaw[pre[-1]]
    start=position[pre[-1]]+rotation(in_yaw)@[0,travel[pre[-1]+1]]
    turn_pos=position[pre[-1]+1:reset]@rotation(in_yaw).T+start
    out_yaw=in_yaw+yaw[reset-1]
    exit_pos=position[reset:turn[-1]+1]@rotation(out_yaw).T+2*turn_pos[-1]-turn_pos[-2]
    end_step=lend+travel[-1]-travel[-2]
    assert 0<=end_step<3
    end=exit_pos[-1]+rotation(out_yaw+yaw[-2])@[0,end_step]
    return np.vstack([position[pre],turn_pos,exit_pos,end]),reset


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    parser.add_argument('out',type=Path)
    args=parser.parse_args()
    run,out=args.run.resolve(),args.out.resolve()
    full=run/'full'
    trial=read(run/'trial.json')
    settings=read(run/'saved_1ms.settings.json')
    params=settings['parameters']
    assert trial['motion']=='in_r45' and settings['mouse']=='A' and settings['preset_speed_mm_s']==1600
    saved,stream,pose,relative=map(pd.read_csv,(run/'saved_1ms.csv',run/'telemetry.csv',
                       full/'apparent_floor_marker_pose.csv',full/'relative_marker_midpoint.csv'))
    quality=read(full/'report.json')
    xy=relative[['center_dx_mm','center_dy_mm']].to_numpy()
    valid=relative.detected.to_numpy(bool)
    time=relative.time_video_s.to_numpy()
    pts,period,origin,rms,motion_rms=align(trial,saved,stream,pose,xy,valid,read(run/'marker_reference_used.json'))
    p=saved['motion.pattern'].to_numpy()
    start,stop=(trial['log_quality'][k] for k in ('motion_start_time_ms','motion_end_time_ms'))
    # Use the full approach apart from its first 8 ms and the transition.
    # Registration-detection gaps may otherwise remove too many observations.
    before=valid&(time>=pts(start+8))&(time<=pts(origin+period*np.flatnonzero(p==7)[0]-3))
    end_pts=pts(origin+period*saved.cnt.iloc[-1])
    after=valid&(time>=end_pts+.008)&(time<=pts(stop-8))
    entry,entry_stats=line_fit(xy[before])
    exit_axis,exit_stats=line_fit(xy[after])
    basis=np.column_stack([[-entry[1],entry[0]],entry])
    pre_motion=valid&(time<pts(start)-.025)&(time>=pts(start)-.25)
    assert pre_motion.sum()>=12
    local=(xy-np.median(xy[pre_motion],axis=0))@basis
    exit_direction=exit_axis@basis
    angle=float(np.degrees(np.arctan2(exit_direction[0],exit_direction[1])))
    stop_info=trial['recording_stop']
    steady=valid&(time>=stop_info['confirmation_camera_seconds']+.1)&(time<=stop_info['confirmation_camera_seconds']+.54)
    assert steady.sum()>=12
    final=np.median(local[steady],axis=0)
    scatter=float(np.sqrt(np.sum(np.var(local[steady],axis=0))))
    target=np.array([45.,180.])
    def length_at(stamp):
        xy_end=np.array([np.interp(stamp,time[valid],axis[valid]) for axis in local.T])
        extra=(target[0]-xy_end[0])/exit_direction[0]
        return dict(endpoint_right_forward_mm=xy_end.tolist(),
                    Lstart_mm=float(params['lstart']+target[1]-xy_end[1]-extra*exit_direction[1]),
                    Lend_mm=float(params['lend']+extra))
    end=length_at(end_pts)
    lengths={key:length_at(end_pts+ms*.001) for key,ms in (('minus5ms',-5),('plus5ms',5))}
    position,reset=onboard_positions(saved,params['lend'])
    raw,raw_settings=pd.read_csv(run/'motion.csv'),read(run/'motion.settings.json')
    try:
        slip=dict(state='DIAGNOSTIC_ONLY',
                  diagnostics=observe(raw,raw_settings,pose,read(full/'body_geometry_used.json'),quality,.020).diagnostics)
    except ValueError as error:
        slip=dict(state='UNAVAILABLE',reason=str(error))
    z=quality['required_motion_window_quality']
    accepted=z['accepted_for_relative_review'] and rms<=8 and motion_rms<=8 and scatter<=1
    velocity_error=saved['ego.rad_velo']-saved['ideal.rad_velo']
    core=(p==7)&(abs(saved['ideal.rad_velo'])>.02*abs(saved['ideal.rad_velo']).max())
    result=dict(mouse='A',motion='in_r45',settings=settings,trial=str(run/'trial.json'),
        reference='Video marker midpoint at motion start with observed entry direction; fixed scale',
        exit_direction_deg=angle,exit_error_deg=angle-45,entry_line=entry_stats,exit_line=exit_stats,
        target_end_right_forward_mm=target.tolist(),target_stop_right_forward_mm=[135,270],
        stopped_right_forward_mm=final.tolist(),stationary_scatter_mm=scatter,
        optical_rate=z['observation_rate'],registration_rate=z['calibration_rate'],motion_model_rmse_mm=motion_rms,
        model_rmse_mm=rms,relative_review_eligible=accepted,one_ms_to_mcu_period_ms=float(period),
        estimated_Lend_pts_s=end_pts,length_only_estimates=end,length_sensitivity=lengths,
        onboard_Lend_end_right_forward_mm=position[-1].tolist(),
        omega_tracking_rmse_rad_s=float(np.sqrt(np.mean(velocity_error[core]**2))),slip=slip,
        parameter_adoption_allowed=False,
        sources_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest()
                       for f in [run/'trial.json',run/'saved_1ms.csv',run/'telemetry.csv',full/'report.json',Path(__file__)]})
    out.mkdir(exist_ok=False)
    (out/'review.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    mask=(time>=pts(start)-.035)&(time<=pts(stop)+.33)
    fig,ax=plt.subplots(figsize=(6,6.7),layout='constrained')
    plot=np.where(valid[:,None],local,np.nan)
    ax.plot(*plot[mask].T,lw=2,label='Video marker midpoint')
    ax.plot(*position[:reset].T,color='#ad4823',lw=1.7,label='Recorded onboard position')
    ax.plot(*position[reset-1:].T,color='#ad4823',ls='--',label='Onboard Lend, joined by gyro')
    ax.scatter(*end['endpoint_right_forward_mm'],s=38,label='Video estimated Lend end')
    ax.scatter(*target,marker='x',s=42,label='Target Lend end')
    ax.scatter(*final,s=42,marker='o',facecolors='none',edgecolors='#16767b',label='Observed stop')
    ax.scatter(135,270,s=42,marker='x',label='Target stop')
    ax.set(xlabel='Right [mm]',ylabel='Forward [mm]',aspect='equal')
    ax.legend(fontsize=8);ax.grid(alpha=.2)
    graph=image(fig,'動画と機体内部の右45度入口軌道')
    f=lambda xy:' / '.join(f'{x:.2f}' for x in xy)
    metrics=table(['指標','実測・推定','目標'],[
        ['出口方向',f'{angle:.2f}°','45°'],['Lend終端 右 / 前 mm',f(end['endpoint_right_forward_mm']),'45 / 180'],
        ['停止点 右 / 前 mm',f(final),'135 / 270'],['動画・ログの走行中照合RMSE',f'{motion_rms:.2f} mm','8.00 mm']])
    page=f"""<!doctype html><html lang="ja"><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>機体A turn1600 右45°入口</title><style>body{{max-width:840px;margin:20px auto;padding:0 16px;color:#183442;font:16px/1.6 system-ui}}
img{{max-width:100%}} td,th{{padding:6px 16px;border-bottom:1px solid #cbdade;text-align:left}}table{{border-collapse:collapse}}</style>
<h1>機体A turn1600 右45°入口</h1><p>1.6 m/s、吸引duty 650。半径−55 mm、Lstart 6.46 mm、Lend 39.86 mm、補正0°。</p>
{metrics}<figure>{graph}</figure><p>進入直線を基準に比較。破線はログ原点のリセット後の区間をジャイロ角で接続したものです。動画のLend終端は時刻合わせによる推定です。</p>
<p><a href="review.json">評価値と入力ログ</a></p></html>"""
    (out/'compare.html').write_text(page,encoding='utf-8')
    print(json.dumps({k:result[k] for k in ('exit_direction_deg','stopped_right_forward_mm','optical_rate',
          'motion_model_rmse_mm','relative_review_eligible','length_only_estimates','length_sensitivity','slip')},ensure_ascii=False))


if __name__=='__main__':
    main()
