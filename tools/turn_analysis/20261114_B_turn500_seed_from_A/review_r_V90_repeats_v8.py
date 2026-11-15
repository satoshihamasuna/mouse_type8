"""同一B右45出口候補・同一2倍撮影の有効記録を比較する。機器操作なし。"""
import argparse, hashlib, json
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def load(path): return json.loads(Path(path).read_text(encoding='utf-8'))
def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def verify(hashes):
    for path,expected in hashes.items(): assert sha(path)==expected,path

def review(paths,output):
    rows=[];sources={};curves=[];baseline=None;acquisition=None;recording_ids=set()
    for index,path in enumerate(paths,1):
        r=load(path);final=path.parent/'final_validity.json';v=load(final)
        assert v['full_trajectory_review_passed'] and v['dense_static_confirmation_passed']
        assert r['mouse']=='B' and r['motion']=='r_v90' and r['status']=='VALID_SINGLE_TRIAL_REVIEW'
        verify(v['input_sha256']);verify(r['input_sha256'])
        settings={key:r['settings'][key] for key in ('mouse','motion_name','parameters','pid','feedforward','suction','preset_speed_mm_s','turn_count')}
        if baseline is None: baseline=settings
        else: assert settings==baseline,'Different settings must not be aggregated'
        summary=r['summary'];assert summary['frame_stride']==1 and summary['auto_stop_confirmed']
        assert summary['motion_observation_rate']>=.95
        trial_path=Path(summary['trial_json']);trial=load(trial_path);run=trial_path.parent
        assert trial['recording_id'] not in recording_ids, 'Duplicate physical trial'
        recording_ids.add(trial['recording_id'])
        assert v['hardware_power']=='BOTH_OFF_USER_CONFIRMED' and v['operator_reported_no_hole_or_wall_contact']
        reference=load(run/'marker_reference_used.json');geometry=load(run/'body_geometry_used.json')
        capture=load(trial['capture_json']);camera=load(capture['metadata_file'])
        assert camera['zoom_ratio_requested']==2 and camera['zoom_verification']['confirmed']
        assert camera['focus_lock']['state']=='LOCKED' and camera['ev_steps_result_values']==[-6]
        condition=dict(reference=reference,geometry=geometry,height_mm=trial['camera_height_mm'],zoom=2,EV_steps=-6,AF='LOCKED')
        if acquisition is None: acquisition=condition
        else: assert condition==acquisition,'Camera/geometry conditions differ'
        report=Path(summary['report_json']);assert sha(report)==summary['report_sha256']
        curve=report.parent/'normalized_trajectory.csv';curves.append(pd.read_csv(curve))
        rows.append(dict(index=index,endpoint_right_mm=summary['endpoint_right_mm'],endpoint_forward_mm=summary['endpoint_forward_mm'],error_right_mm=r['endpoint_error_right_mm'],error_forward_mm=r['endpoint_error_forward_mm'],exit_error_deg=r['exit_travel_error_deg'],motion_observation_rate=summary['motion_observation_rate']))
        for p in (path,final,report,curve,trial_path,run/'marker_reference_used.json',run/'body_geometry_used.json',Path(capture['metadata_file'])): sources[str(p)]=sha(p)
    table=pd.DataFrame(rows);assert len(table)>0
    stats={key:dict(mean=float(table[key].mean()),range=float(np.ptp(table[key])),population_std=float(table[key].std(ddof=0)),maximum_absolute=float(abs(table[key]).max()),values=table[key].tolist()) for key in ('endpoint_right_mm','endpoint_forward_mm','error_right_mm','error_forward_mm','exit_error_deg')}
    guidance=dict(minimum_valid_repeats=3,maximum_abs_mean_axis_error_mm=1,maximum_abs_individual_axis_error_mm=2,maximum_abs_exit_error_deg=.5)
    numeric=bool(abs(stats['error_right_mm']['mean'])<=1 and abs(stats['error_forward_mm']['mean'])<=1 and stats['error_right_mm']['maximum_absolute']<=2 and stats['error_forward_mm']['maximum_absolute']<=2 and stats['exit_error_deg']['maximum_absolute']<=.5)
    result=dict(evaluation_frame='relative_origin_and_measured_entry_direction',board_fixed_goal_verified=False,recording_ids=sorted(recording_ids),detector_version='B_zoom2_v8',mouse='B',motion='r_v90',valid_trials=len(rows),status='THREE_VALID_SINGLE_CANDIDATE' if len(rows)>=3 and numeric else 'PARTIAL_REPEAT_CHECK' if len(rows)<3 else 'GUIDANCE_NOT_MET',statistics=stats,guidance=guidance,numerical_guidance_passed=numeric,repeatability_guidance_passed=bool(len(rows)>=3 and numeric),settings=baseline,unchanged_camera_reference_id=acquisition['reference']['reference_id'],camera_zoom_ratio=2,parameter_adoption_allowed=False,absolute_axle_position_verified=False,body_yaw_verified=False,independent_hardware_sync_verified=False,input_sha256={**sources,str(Path(__file__)):sha(__file__)})
    output.mkdir(exist_ok=False);table.to_csv(output/'trials.csv',index=False)
    (output/'review.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    fig,ax=plt.subplots(figsize=(6,5),layout='constrained')
    for row,curve in zip(rows,curves):
        good=curve.detected.to_numpy(bool);ax.plot(curve.right_mm[good],curve.forward_mm[good],label=f"Trial {row['index']}")
    ax.plot(127.27921030678928,190.91881030678928,'kx',label='Target')
    ax.set(xlabel='Right [mm]',ylabel='Forward [mm]',title=f"B r_V90, {len(rows)} unchanged trials")
    ax.axis('equal');ax.grid(alpha=.3);ax.legend();fig.savefig(output/'trajectories.png',dpi=150);plt.close(fig)
    print(json.dumps(dict(status=result['status'],valid_trials=len(rows),mean_error_right_mm=stats['error_right_mm']['mean'],mean_error_forward_mm=stats['error_forward_mm']['mean'],range_right_mm=stats['endpoint_right_mm']['range'],range_forward_mm=stats['endpoint_forward_mm']['range'],maximum_exit_error_deg=stats['exit_error_deg']['maximum_absolute'])))

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--reviews',type=Path,nargs='+',required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();review(args.reviews,args.output)
