"""保存runの赤前点だけを相対評価するCLI。実機・中点補間・camerafoot推定なし。

python tmp/parallel_front_run_review_20261120.py --run <saved-run> --output <new-parallel-dir>
--motion in_r45|in_r135|out_r45|out_r135|long_r180。full解析前も赤の静止→移動からseedを検出。
--midpoint-target RIGHT FORWARD で同じ入口基準の名目Lend目標を明示できる。
"""
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

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'tools/analysis')]
from video_marker_pose import colour_components,marker_registration,registration_quad,pose_world_corners,transform_points,board_to_pose_transform
SHAPES={'in_r45':(7,45.,[45.,180.]),'in_r135':(11,135.,[90.,135.]),
        'out_r45':(9,45.,[45/np.sqrt(2),315/np.sqrt(2)]),'out_r135':(13,135.,[135/np.sqrt(2),225/np.sqrt(2)]),
        'long_r180':(5,180.,[90.,90.])}


def read(path): return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def decoded_set(value):
    if pd.isna(value) or not str(value).strip():return set()
    return {int(v.strip()) for v in str(value).split(',') if v.strip()}


def select_red_component(candidates,history,stamp):
    """予測は実在成分の選択だけに使用。競合・逆走/大残差は欠測にする。"""
    if not candidates:return None,'no_component',None,None
    points=[(np.array([v['x_mm'],v['y_mm']]),v['area_px']) for v in candidates]
    if not history:
        ranked=sorted(points,key=lambda v:v[1],reverse=True)
        if len(ranked)>1 and ranked[1][1]>=ranked[0][1]*.8:return None,'ambiguous_initial_area',None,None
        return ranked[0],'accepted_initial',None,None
    previous_time,previous=history[-1];dt=stamp-previous_time;prediction=previous
    velocity=None
    if len(history)>=2:
        velocity=(history[-1][1]-history[-2][1])/(history[-1][0]-history[-2][0])
        prediction=previous+velocity*dt
    ranked=sorted([(float(np.linalg.norm(q-prediction)),q,a) for q,a in points],key=lambda v:v[0])
    gap=float(ranked[1][0]-ranked[0][0]) if len(ranked)>1 else None
    if gap is not None and gap<5.:return None,'ambiguous_prediction_margin',ranked[0][0],gap
    error,q,area=ranked[0]
    if error>15.:return None,'ambiguous_prediction_residual',error,gap
    if np.linalg.norm(q-previous)>15*max(1.,dt*240):return None,'continuity_rejected',error,gap
    if velocity is not None and np.linalg.norm(q-previous)>2 and np.linalg.norm(velocity)>200 and (q-previous)@velocity<0:
        return None,'ambiguous_reversal',error,gap
    return (q,area),'accepted',error,gap


def sha(path):
    result=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda:stream.read(8*1024*1024),b''):result.update(chunk)
    return result.hexdigest()


def save(path,value):
    with path.open('x',encoding='utf-8') as stream:json.dump(value,stream,ensure_ascii=False,indent=2,allow_nan=False)


def line(points):
    if len(points)<15:raise ValueError(f'直線観測15点未満: {len(points)}')
    d=np.linalg.svd(points-points.mean(0),full_matrices=False)[2][0]
    if d@(points[-1]-points[0])<0:d=-d
    span=float(np.ptp(points@d))
    if span<30:raise ValueError(f'直線span30mm未満: {span}')
    residual=points-points.mean(0);residual-=np.outer(residual@d,d)
    return d,dict(points=len(points),span_mm=span,rms_mm=float(np.sqrt(np.mean(np.sum(residual**2,1)))))


def point(time,xy,valid,stamp):
    t=time[valid];values=xy[valid];i=np.searchsorted(t,stamp)
    if not 0<i<len(t) or t[i]-t[i-1]>.020:raise ValueError('Lend位置に20ms超の欠測/範囲外があります')
    return np.array([np.interp(stamp,t,values[:,k]) for k in (0,1)])


def main():
    p=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--motion',choices=SHAPES);p.add_argument('--plan',type=Path)
    p.add_argument('--body-geometry',type=Path,help='保存動画の検出候補。元の設定は上書きせず入力hashへ記録する')
    p.add_argument('--midpoint-target',nargs=2,type=float,metavar=('RIGHT','FORWARD'))
    p.add_argument('--observations',type=Path,help='同runの実ArUco2/3+赤CSVを再利用。隣接diagnostic/provenance照合必須')
    p.add_argument('--observations-meta',type=Path)
    p.add_argument('--motion-start-video-s',type=float,help='明示した動き始めseed。省略時はfull reportまたは赤移動検出')
    p.add_argument('--front-onset',action='store_true',help='full reportを使わず赤の静止→移動検出をseedにする')
    p.add_argument('--entry-window',nargs=2,type=float);p.add_argument('--exit-window',nargs=2,type=float)
    p.add_argument('--clock-search-ms',type=float,default=100.)
    args=p.parse_args();run=args.run.resolve();out=args.output.resolve()
    try:out.relative_to(ROOT)
    except ValueError:raise ValueError('出力先はworkspace内にしてください')
    if out.exists():raise FileExistsError(out)
    trial=read(run/'trial.json');settings=read(run/'saved_1ms.settings.json');settings=settings.get('actual_settings',settings)
    motion=args.motion or settings['motion_name']
    if motion not in SHAPES or settings['motion_name']!=motion:raise ValueError('対象motionが設定と一致しません')
    if settings['mouse']!='A' or settings['turn_count']!=1:raise ValueError('この版は機体A・単体1回用です')
    geometry_path=args.body_geometry.resolve() if args.body_geometry else run/'body_geometry_used.json'
    geometry_path.relative_to(ROOT)
    reference,geometry=read(run/'marker_reference_used.json'),read(geometry_path)
    if trial['camera_height_mm']!=1500 or geometry['front_height_mm']!=10 or geometry['front_colour']!='red':
        raise ValueError('この版はH1500mm・赤前height10mmを明示確認したrun用です')
    pattern,angle,fallback_target=SHAPES[motion]
    planpath=args.plan
    if planpath is None:
        planpath=next((v/'placement_plan.json' for v in run.parents if (v/'placement_plan.json').is_file()),None)
    plan=read(planpath) if planpath else {}
    nominal=args.midpoint_target
    target_source='--midpoint-target' if nominal is not None else None
    if nominal is None:
        for key in ('nominal_Lend_right_forward_mm','nominal_Lend_from_start_right_forward_mm'):
            if key in plan:nominal=plan[key];target_source=f'{planpath}:{key}';break
    if nominal is None:
        board=plan.get('nominal_Lend_board_mm',plan.get('reference_nominal_Lend_board_mm'))
        if board is not None and 'planned_start_board_mm' in plan:
            h=np.radians(plan['planned_heading_image_deg']);e=np.array([np.cos(h),np.sin(h)])
            nominal=(np.asarray(board)-plan['planned_start_board_mm'])@np.column_stack(([-e[1],e[0]],e))
            target_source=f'{planpath}:board_delta_rotated_by_nominal_heading'
    if nominal is None:nominal=fallback_target;target_source='Explicit current A1800 shape fallback; see SHAPES and README'
    nominal=np.asarray(nominal,float)
    raw,stream=pd.read_csv(run/'saved_1ms.csv'),pd.read_csv(run/'telemetry.csv')
    if not np.array_equal(raw.cnt.to_numpy(),np.arange(len(raw))):raise ValueError('saved cntが連続ではありません')
    signature=['ideal.velo','ego.velo','ideal.rad_velo','ego.rad_velo','V_r','V_l']
    matches=raw.merge(stream[['time_ms']+signature],on=signature)
    if len(matches)<5:raise ValueError('1ms保存ログとstreamの一致点が不足')
    tick,mcu_origin=np.polyfit(matches.cnt,matches.time_ms,1)
    saved_time=raw.cnt.to_numpy()*tick/1000
    turn_indices=np.flatnonzero(raw['motion.pattern'].to_numpy()==pattern)
    if not len(turn_indices):raise ValueError('対象旋回patternが保存ログにありません')
    before_turn=float(saved_time[turn_indices[0]])
    delta_start=(mcu_origin-trial['log_quality']['motion_start_time_ms'])/1000
    duration=(trial['log_quality']['motion_end_time_ms']-trial['log_quality']['motion_start_time_ms'])/1000
    seed=args.motion_start_video_s;seed_source='explicit argument'
    oldreport=run/'full/report.json'
    if seed is None and oldreport.is_file() and not args.front_onset:
        seed=read(oldreport).get('required_motion_window_quality',{}).get('video_motion_start_s')
        seed_source='existing full report seed only; final clock uses incoming wheel distance'
    evidence=[run/'trial.json',run/'saved_1ms.settings.json',run/'saved_1ms.csv',run/'telemetry.csv',run/'marker_reference_used.json',geometry_path,Path(__file__)]
    if oldreport.is_file():evidence.append(oldreport)
    if planpath:evidence.append(planpath)
    if args.observations:
        meta=args.observations_meta or args.observations.parent/'observations.provenance.json'
        if not meta.is_file():meta=args.observations.parent/'diagnostic.json'
        metadata=read(meta)
        if Path(metadata['source_trial']).resolve()!=(run/'trial.json').resolve():raise ValueError('赤観測CSVのrunが一致しません')
        if metadata.get('actual_registered_aruco_per_frame') is not True:raise ValueError('毎frame実登録の証拠がありません')
        data=pd.read_csv(args.observations);evidence.extend([args.observations,meta])
    else:
        estimate=float(trial.get('video_motion_start_estimate_s',7.))
        start=max(0.,(seed-.3 if seed is not None else estimate-.3))
        end=(seed+duration+.3 if seed is not None else estimate+duration+2.3)
        print(f'赤/ArUco毎frame解析 {start:.3f}..{end:.3f}s',flush=True)
        cap=cv2.VideoCapture(trial['video_file']);cap.set(cv2.CAP_PROP_ORIENTATION_AUTO,0);cv2.setNumThreads(2)
        if not cap.isOpened():raise ValueError('元動画を開けません')
        rows=[];index=0;previous=None;previous_time=None;history=[]
        while True:
            ok,frame=cap.read()
            if not ok:break
            stamp=cap.get(cv2.CAP_PROP_POS_MSEC)/1000
            if stamp<start:index+=1;continue
            if stamp>end:break
            registration=marker_registration(frame,reference);quad=registration_quad(registration)
            row=dict(frame=index,time_video_s=stamp,calibration_valid=quad is not None,
                decoded_ids=','.join(map(str,registration['detected_marker_ids'])),red_detected=False,
                red_x_mm=np.nan,red_y_mm=np.nan,candidates_on_board=0,red_candidates_json='[]')
            if quad is not None and {2,3}<=set(registration['detected_marker_ids']):
                matrix=np.linalg.inv(board_to_pose_transform(reference))@cv2.getPerspectiveTransform(quad,pose_world_corners(reference.get('board_side_mm',360)))
                components=colour_components(frame,'red',geometry['marker_detection'])
                if components:
                    coords=transform_points([v[0] for v in components],matrix)
                    eligible=[(q,a) for q,(_,a) in zip(coords,components) if 0<=q[0]<=360 and 0<=q[1]<=360]
                    row['candidates_on_board']=len(eligible)
                    row['red_candidates_json']=json.dumps([dict(x_mm=float(q[0]),y_mm=float(q[1]),area_px=int(a)) for q,a in eligible])
                    if eligible:
                        prediction=previous
                        if len(history)>=2:
                            velocity=(history[-1][1]-history[-2][1])/(history[-1][0]-history[-2][0])
                            prediction=previous+velocity*(stamp-previous_time)
                        q,area=(max(eligible,key=lambda v:v[1]) if previous is None else min(eligible,key=lambda v:np.linalg.norm(v[0]-prediction)))
                        maxstep=15*max(1.,(stamp-previous_time)*240) if previous is not None else 15
                        if previous is None or np.linalg.norm(q-previous)<=maxstep:
                            row.update(red_detected=True,red_x_mm=float(q[0]),red_y_mm=float(q[1]),component_area_px=int(area))
                            previous,previous_time=q,stamp
                            history.append((stamp,q.copy()));history=history[-2:]
            rows.append(row);index+=1
            if len(rows)%240==0:print(f'{len(rows)}frames / {stamp:.3f}s',flush=True)
        cap.release();data=pd.DataFrame(rows)
    if 'red_candidates_json' in data:
        history=[]
        data['red_ambiguous']=False;data['red_selector_status']='unregistered'
        data['prediction_error_mm']=np.nan;data['candidate_margin_mm']=np.nan
        for index,row in data.iterrows():
            data.at[index,'red_detected']=False;data.at[index,'red_x_mm']=np.nan;data.at[index,'red_y_mm']=np.nan
            if not row.calibration_valid or not {2,3}<=decoded_set(row.decoded_ids):continue
            candidates=json.loads(row.red_candidates_json) if pd.notna(row.red_candidates_json) else []
            selected,status,error,margin=select_red_component(candidates,history,float(row.time_video_s))
            data.at[index,'red_selector_status']=status;data.at[index,'red_ambiguous']=status.startswith('ambiguous')
            if error is not None:data.at[index,'prediction_error_mm']=error
            if margin is not None:data.at[index,'candidate_margin_mm']=margin
            if selected is not None:
                q,area=selected
                data.at[index,'red_detected']=True;data.at[index,'red_x_mm']=q[0];data.at[index,'red_y_mm']=q[1];data.at[index,'component_area_px']=area
                history.append((float(row.time_video_s),q.copy()));history=history[-2:]
    evidence_hashes={str(v):sha(v) for v in evidence}
    out.mkdir(parents=True)
    data.to_csv(out/'red_observations.csv',index=False)
    save(out/'observations.provenance.json',dict(source_trial=str(run/'trial.json'),actual_registered_aruco_per_frame=True,
        video_file=trial['video_file'],video_sha256=sha(Path(trial['video_file'])),source_inputs=evidence_hashes,
        colour_thresholds_changed=bool(args.body_geometry),body_geometry_override=str(geometry_path) if args.body_geometry else None,frame_stride=1,missing_registration_policy='exclude_frame',
        red_selector='last2-point constant velocity; >=5mm candidate-distance margin; prediction error<=15mm; reversal/ambiguous invalid; no imputed observation'))
    time=data.time_video_s.to_numpy();floor=data[['red_x_mm','red_y_mm']].to_numpy()
    decoded=np.array([{2,3}<=decoded_set(s) for s in data.decoded_ids])
    valid=(data.calibration_valid&data.red_detected).to_numpy(bool)&decoded
    if seed is None:
        baseline=valid&(time<=time[0]+.2)
        if baseline.sum()<12:raise ValueError('onset検出の静止赤観測が12点未満')
        first=np.median(floor[baseline],axis=0);distance=np.linalg.norm((floor-first)*(1490/1500),axis=1)
        active=valid&(np.nan_to_num(distance,nan=0.)>=2.)
        onset=next((i for i in range(len(time)-2) if time[i]>time[0]+.2 and active[i:i+3].all() and time[i+2]-time[i]<.020),None)
        if onset is None:raise ValueError('静止→2mm以上移動3frame連続を検出できません')
        acceleration=18/np.sqrt(2) if motion.startswith('out_') else 18.
        seed=float(time[onset]-np.sqrt(2*.002/acceleration))
        seed_source='red moves>=2mm for3consecutive frames; subtract sqrt(2*2mm/preacceleration); seed only'
    saved_zero_seed=seed+delta_start
    entry_window=args.entry_window or [seed+.010,saved_zero_seed+before_turn-.008]
    exit_window=args.exit_window or ([saved_zero_seed+saved_time[-1]+.005,seed+duration+.010] if motion=='long_r180' else [saved_zero_seed+saved_time[-1]+.025,seed+duration-.035])
    entry_mask=valid&(time>=entry_window[0])&(time<=entry_window[1])
    exit_mask=valid&(time>=exit_window[0])&(time<=exit_window[1])
    entry,eq=line(floor[entry_mask]);leaving,xq=line(floor[exit_mask]);basis=np.column_stack(([-entry[1],entry[0]],entry))
    baseline=valid&(time>=seed-.18)&(time<=seed-.04)
    if baseline.sum()<12:raise ValueError('開始点の静止赤観測12点未満')
    origin=np.median(floor[baseline],axis=0);xy=(1490/1500)*(floor-origin)@basis;xy[~valid]=np.nan
    distance=np.cumsum((raw['Encoder_GetProperty_Right().sp_pulse']-raw['Encoder_GetProperty_Left().sp_pulse'])*15.02*np.pi/172/2).to_numpy()
    def fit(mask):
        value=minimize_scalar(lambda z:np.mean((np.interp(time[mask]-z,saved_time,distance)-xy[mask,1])**2),
            bounds=(saved_zero_seed-args.clock_search_ms/1000,saved_zero_seed+args.clock_search_ms/1000),method='bounded',options={'xatol':1e-10})
        return float(value.x),float(np.sqrt(value.fun))
    zero,rms=fit(entry_mask);middle=np.median(time[entry_mask]);halves=[fit(entry_mask&(time<=middle))[0],fit(entry_mask&(time>middle))[0]]
    lend_time=zero+saved_time[-1];measured=point(time,xy,valid,lend_time)
    phi=float(np.degrees(np.arctan2((leaving@basis)[0],(leaving@basis)[1]))%360)
    actual_start=zero-delta_start;actual_stop=actual_start+duration
    quality_window=(time>=actual_start-.2)&(time<=actual_stop+.1)
    rate=float(valid[quality_window].mean()) if quality_window.any() else 0.
    registered=data.calibration_valid.to_numpy(bool)&decoded
    registered_count=int(registered[quality_window].sum())
    registration_rate=float(registered[quality_window].mean()) if quality_window.any() else 0.
    conditional_red_rate=float(valid[quality_window].sum()/registered_count) if registered_count else 0.
    motion_window=(time>=actual_start)&(time<=actual_stop)
    motion_rate=float(valid[motion_window].mean()) if motion_window.any() else 0.
    coverage=bool(time[0]<=actual_start-.2+.010 and time[-1]>=actual_stop+.1-.010)
    timing_ok=bool(rms<=1.5 and abs(halves[0]-halves[1])<=.002 and abs(zero-saved_zero_seed)<args.clock_search_ms/1000*.98)
    line_ok=bool(eq['rms_mm']<=.75 and xq['rms_mm']<=.75)
    scenarios=[];th=np.radians(angle);path=np.array([np.sin(np.radians(phi)),np.cos(np.radians(phi))]);normal=np.array([-path[1],path[0]])
    for radius in (20.,25.,30.):
        goal=nominal+radius*np.array([np.sin(th),np.cos(th)-1]);error=measured-goal
        scenarios.append(dict(assumed_half_span_mm=radius,front_target_right_forward_mm=goal.tolist(),front_Lend_error_right_forward_mm=error.tolist(),along_exit_error_mm=float(error@path),cross_exit_error_mm=float(error@normal),half_span_is_measured=False))
    return_line=None
    if motion=='long_r180':
        exit_direction=leaving@basis;exit_mean=xy[exit_mask].mean(0)
        return_line=dict(observed_point='front-red relative line only',nominal_spacing_mm=float(nominal[0]),
            observed_forward_range_mm=[float(xy[exit_mask,1].min()),float(xy[exit_mask,1].max())],sections=[])
        if abs(exit_direction[1])<.9:raise ValueError('右180°の返り直線方向を評価できません')
        for forward in (0.,45.):
            width=float(exit_mean[0]+(forward-exit_mean[1])*exit_direction[0]/exit_direction[1])
            low,high=return_line['observed_forward_range_mm']
            return_line['sections'].append(dict(forward_mm=forward,return_spacing_mm=width,error_from_nominal_mm=width-float(nominal[0]),
                extrapolated=bool(forward<low or forward>high),extrapolation_distance_mm=float(max(low-forward,forward-high,0.))))
    state='FRONT_RED_QUALITY_REJECTED'
    if rate>=.95 and coverage and timing_ok and line_ok:
        state='FRONT_RED_RELATIVE_REVIEW' if motion_rate>=.95 else 'FRONT_RED_LINE_DIAGNOSTIC_ONLY'
    result=dict(state=state,
        source_trial=str(run/'trial.json'),mouse='A',motion=motion,observed_point='front-red relative displacement only; no reconstructed midpoint/axle',
        actual_registered_aruco_per_frame=True,required_ids=[2,3],height_correction=dict(camera_height_mm=1500,front_height_mm=10,factor=1490/1500,camera_foot_required=False),
        observed_frames=int(valid[quality_window].sum()),evaluated_frames=int(quality_window.sum()),observation_rate=rate,required_window_contained=coverage,
        registered_aruco_frames=registered_count,registration_rate=registration_rate,red_detection_given_registration_rate=conditional_red_rate,
        rear_pair_observation_rate=None,rear_pair_quality_source='Original root-managed full/report.json only; front-red does not replace it.',
        motion_only_observed_frames=int(valid[motion_window].sum()),motion_only_evaluated_frames=int(motion_window.sum()),motion_only_observation_rate=motion_rate,
        all_motion_front_quality_passed=bool(motion_rate>=.95),
        entry_window_observation_rate=float(valid[(time>=entry_window[0])&(time<=entry_window[1])].mean()),
        exit_window_observation_rate=float(valid[(time>=exit_window[0])&(time<=exit_window[1])].mean()),
        required_window_pts_s=[float(actual_start-.2),float(actual_stop+.1)],entry_window_pts_s=list(map(float,entry_window)),exit_window_pts_s=list(map(float,exit_window)),
        entry_line=eq,exit_line=xq,line_quality_passed=line_ok,path_direction_change_deg=phi,nominal_turn_angle_deg=angle,path_angle_error_deg=float((phi-angle+180)%360-180),
        midpoint_nominal_target_right_forward_mm=nominal.tolist(),target_source=target_source,
        measured_front_Lend_right_forward_mm=measured.tolist(),half_span_scenarios=scenarios,return_line=return_line,
        clock=dict(seed_source=seed_source,motion_seed_video_s=float(seed),search_half_width_ms=args.clock_search_ms,saved_zero_video_s=zero,
            measured_tick_ms=float(tick),incoming_fit_rms_mm=rms,half_window_zero_spread_ms=float(abs(halves[0]-halves[1])*1000),lend_video_pts_s=float(lend_time),
            passed=timing_ok,exit_or_turn_used_for_fit=False,scale_fitted=False,independent_hardware_sync=False),
        thresholds=dict(observation_rate=.95,line_points=15,line_span_mm=30,line_rms_mm=.75,clock_rms_mm=1.5,clock_half_window_spread_ms=2),
        missing_rear_generated=False,absolute_body_yaw_available=False,midpoint_observation_claimed=False,precise_parameter_adoption_allowed=False,
        limitations=['r20/25/30 are sensitivity scenarios, not measured halfspan or confidence intervals.',
            'Initial body direction is nominally aligned with incoming path; slip/pitch/roll and scale remain unverified.',
            'Front height and camera fixed assumption; decoded registration and nearest red continuity do not independently prove false-positive rate.',
            'Right-front target is a nominal transform only, not an observed midpoint or axle.'],
        colour_thresholds_changed=bool(args.body_geometry),body_geometry_override=str(geometry_path) if args.body_geometry else None,physical_execution=False)
    result['red_selector']='nearest last2-observation constant-velocity prediction; real component only; 15mm/nominal-frame continuity limit unchanged'
    result['ambiguous_frames_required_window']=int(data.loc[quality_window,'red_ambiguous'].sum()) if 'red_ambiguous' in data else None
    result['selector_confidence']=dict(candidate_distance_margin_mm=5.,maximum_prediction_residual_mm=15.,ambiguous_or_reversal_policy='invalid and included in denominator',new_selector_applied='red_candidates_json' in data)
    data.assign(front_relative_valid=valid,right_mm=xy[:,0],forward_mm=xy[:,1]).to_csv(out/'front_relative_trajectory.csv',index=False)
    result['input_sha256']=evidence_hashes
    save(out/'front_reference.json',result)
    plt.rcParams['font.family']='Yu Gothic';fig,ax=plt.subplots(figsize=(6,6),constrained_layout=True)
    ax.plot(xy[:,0],xy[:,1],label='赤前点の高さ補正済み相対変位')
    ax.scatter(*measured,c='red',label='前走だけで時刻照合したLend')
    ax.scatter(*scenarios[1]['front_target_right_forward_mm'],marker='x',c='black',label='r25概算の名目赤前Lend')
    ax.set(xlabel='入口に対する右 [mm]',ylabel='入口に対する前 [mm]',title=f'{motion} / {result["state"]}',aspect='equal');ax.grid();ax.legend(fontsize=8)
    fig.savefig(out/'front_reference.png',dpi=130);plt.close(fig)
    print(json.dumps(dict(output=str(out),state=result['state'],motion=motion,observation_rate=rate,angle_deg=phi,front_Lend=measured.tolist(),nominal25=scenarios[1],clock=result['clock']),ensure_ascii=False,indent=2),flush=True)


if __name__=='__main__':main()
