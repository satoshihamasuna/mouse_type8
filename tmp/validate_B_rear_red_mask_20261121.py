"""淡い後部赤マーカーの候補を元画素で検証する。走行解・目標点は使わない。"""
from pathlib import Path
from datetime import datetime,timezone
import copy
import hashlib
import json
import sys
import cv2
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_rv90_20261120_v1'
OUT=BASE/'rear_red_profile_v4_validation'
assert not OUT.exists();OUT.mkdir()
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'tools/analysis')]
from video_marker_pose import colour_components,select_marker_pair,marker_registration,registration_quad,pose_world_corners,transform_points
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))


def save(path,value):
    with path.open('x',encoding='utf-8') as stream:
        json.dump(value,stream,ensure_ascii=False,indent=2,allow_nan=False)
        stream.write('\n')


runs=[BASE/f'geometry_ff0p0006_Ls3p3_Le18p9_c0p0_{i:02d}/local_execution/run_B_RV90_1600_geom3p3_18p9_c0p0_ff0p0006_20261121_{i:03d}' for i in (2,3)]
geometry=read(runs[0]/'body_geometry_used.json')
candidate=copy.deepcopy(geometry)
candidate['marker_detection']['red'].update(hue_ranges=[[0,10],[160,179]],minimum_saturation=30)
candidate['rear_marker_detection_revision']='2026-11-21: 後部赤の淡色欠測を元画像の色領域で検証する候補。赤Hue高側下限170→160、S40→30。他の色・V・面積・開演算・座標/時刻/採用条件は固定。角度/目標点の値は選択に使わない。'
profile=OUT/'body_geometry_B_green_s35_red_h160_s30_v4.json'
save(profile,candidate)
profile_sha=hashlib.sha256(profile.read_bytes()).hexdigest()
all_results=[]
for run_index,run in enumerate(runs):
    trial=read(run/'trial.json');reference=read(run/'marker_reference_used.json')
    original=read(run/'body_geometry_used.json')
    assert original['marker_detection']==geometry['marker_detection']
    pose=pd.read_csv(run/'full/apparent_floor_marker_pose.csv')
    valid=pose.marker_detected_raw.to_numpy(bool)
    bad=pose.loc[~valid]
    if run_index==0:
        targets=read(run/'missing_frames_diagnostic_v1/manifest.json')['records']
        frames=[row['frame'] for row in targets]
    else:
        nearest=[int(pose.loc[valid].iloc[int(np.argmin(abs(pose.loc[valid,'time_video_s']-stamp)))].frame)
                 for stamp in (float(bad.time_video_s.min())-.01,float(bad.time_video_s.max())+.01,
                               float(pose.time_video_s.min())+.15,float(pose.time_video_s.max())-.15)]
        frames=sorted(set(bad.frame.astype(int).tolist()+nearest))
    selected=pose.loc[pose.frame.isin(frames)].copy()
    target_times=selected.time_video_s.to_numpy()
    centers=np.column_stack([np.interp(target_times,pose.time_video_s[valid],pose.loc[valid,key])
        for key in ('midpoint_image_x_px','midpoint_image_y_px')])
    capture=cv2.VideoCapture(trial['video_file']);capture.set(cv2.CAP_PROP_ORIENTATION_AUTO,0)
    capture.set(cv2.CAP_PROP_POS_MSEC,float(target_times.min()*1000-60))
    decoded={}
    while True:
        ok,frame=capture.read()
        if not ok:break
        stamp=capture.get(cv2.CAP_PROP_POS_MSEC)/1000
        if stamp>target_times.max()+.01:break
        errors=np.abs(target_times-stamp)
        if errors.min()<.001:decoded[int(errors.argmin())]=(stamp,frame.copy())
    capture.release()
    assert len(decoded)==len(selected)
    cols=3;rows=(len(selected)+cols-1)//cols
    fig,axes=plt.subplots(rows,cols,figsize=(12,4*rows),layout='constrained',squeeze=False)
    results=[]
    for index,((_,row),center,axis) in enumerate(zip(selected.iterrows(),centers,axes.ravel())):
        stamp,frame=decoded[index]
        registration=marker_registration(frame,reference);quad=registration_quad(registration)
        assert quad is not None and len(set(registration['detected_marker_ids']))>=2
        matrix=cv2.getPerspectiveTransform(quad,pose_world_corners(reference['board_side_mm']))
        evaluated={}
        points={}
        for name,configuration in [('baseline',geometry),('red_v4',candidate)]:
            detection=configuration['marker_detection']
            pair=select_marker_pair(colour_components(frame,'green',detection),colour_components(frame,'red',detection),
                matrix,None,board_side_mm=reference['board_side_mm'],nominal_marker_span_mm=configuration['nominal_marker_span_mm'],
                marker_span_limits_mm=configuration['marker_span_limits_mm'])
            evaluated[name]=dict(observed=pair is not None)
            if pair is not None:
                pixel=transform_points(pair,np.linalg.inv(matrix));points[name]=pixel
                evaluated[name].update(pixel_centroids=pixel.tolist(),apparent_span_mm=float(np.linalg.norm(pair[0]-pair[1])))
        x,y=np.rint(center).astype(int);left,top=max(0,x-80),max(0,y-80)
        crop=frame[top:min(frame.shape[0],y+80),left:min(frame.shape[1],x+80)]
        axis.imshow(cv2.cvtColor(crop,cv2.COLOR_BGR2RGB),interpolation='nearest')
        if 'red_v4' in points:
            q=points['red_v4']-[left,top]
            axis.scatter(q[0,0],q[0,1],s=160,marker='o',facecolors='none',edgecolors='lime',linewidths=1.1)
            axis.scatter(q[1,0],q[1,1],s=160,marker='o',facecolors='none',edgecolors='white',linewidths=1.1)
        axis.set_title(f"frame {int(row.frame)} / {stamp:.6f}s\nbase={evaluated['baseline']['observed']} new={evaluated['red_v4']['observed']}")
        axis.axis('off')
        results.append(dict(frame=int(row.frame),pts_s=stamp,registered_aruco_ids=registration['detected_marker_ids'],
            original_cache_observed=bool(row.marker_detected_raw),**evaluated))
    for axis in axes.ravel()[len(selected):]:axis.axis('off')
    fig.suptitle(f"{'Primary' if run_index==0 else 'Holdout'} raw-pixel review: green/front, white/rear circles\nNo trajectory, timing or target-point fitting used",fontsize=13)
    fig.savefig(OUT/f'run{run_index+1}_raw_pair_review.png',dpi=120);plt.close(fig)
    save(OUT/f'run{run_index+1}_raw_pairs.json',dict(source_trial=str(run/'trial.json'),profile_sha256=profile_sha,rows=results,
        approximate_crop_centres_used_as_observations=False,temporal_imputation_used=False,physical_acceptance_changed=False))
    all_results.append(dict(source_trial=str(run/'trial.json'),sampled_frames=len(results),
        baseline_pairs=sum(a['baseline']['observed'] for a in results),candidate_pairs=sum(a['red_v4']['observed'] for a in results)))
save(OUT/'validation_pending_visual_review.json',dict(created_at=datetime.now(timezone.utc).isoformat(),
    profile=str(profile),profile_sha256=profile_sha,changes={'red.hue_ranges':[[[0,10],[170,179]],[[0,10],[160,179]]],'red.minimum_saturation':[40,30]},
    results=all_results,visual_review_pending=True,adoption_allowed=False,hardware_operations=0))
print(json.dumps(all_results,ensure_ascii=False))
