"""小さく写った実前マーカーの面積候補を、欠測と独立保存画像で検査する。実機操作なし。"""
from pathlib import Path
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
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_outr135_20261120_v1'
RUN=BASE/'baseline01/local_execution/run_B_OUTR135_1600_baseline_green_20261120_003'
OUT=BASE/'green_area10_validation_v1';assert not OUT.exists();OUT.mkdir()
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'tools/analysis')]
from video_marker_pose import marker_registration,registration_quad,pose_world_corners,colour_components,select_marker_pair,transform_points
original=read(RUN/'body_geometry_used.json');candidate=copy.deepcopy(original)
candidate['marker_detection']['green']['minimum_area_px']=10
candidate['marker_detection_source']='右135度出口の欠測9画像で実前ステッカーが10〜14pxになる箇所を視認。面積だけ15→10px、色/開演算/間隔/ArUco/観測95%/直線品質は維持。独立B入口動画と今回の全区間で別検証する。'
candidate_path=OUT/'body_geometry_B_green_area10_candidate.json'
with candidate_path.open('x',encoding='utf-8') as file:json.dump(candidate,file,ensure_ascii=False,indent=2)
reference=read(RUN/'marker_reference_used.json');pose=pd.read_csv(RUN/'full/apparent_floor_marker_pose.csv')
report=read(RUN/'full/report.json');lo,hi=report['required_motion_window_quality']['required_interval_pts_s']
missing=pose.index[~pose.marker_detected_raw.astype(bool)].tolist()
controls=pose.index[pose.time_video_s.between(lo,hi)].to_numpy()[::4].tolist()
indexes=sorted(set(missing+controls));rows=[];recovered=[]
cv2.setNumThreads(4)


def pairs(frame,ref,old_geometry,new_geometry):
    registration=marker_registration(frame,ref);quad=registration_quad(registration)
    if quad is None or len(set(registration['detected_marker_ids']))<2:return None,None,None,registration
    matrix=cv2.getPerspectiveTransform(quad,pose_world_corners(ref['board_side_mm']))
    def pair(geometry):
        return select_marker_pair(colour_components(frame,'green',geometry['marker_detection']),
            colour_components(frame,'red',geometry['marker_detection']),matrix,None,
            nominal_marker_span_mm=geometry['nominal_marker_span_mm'],marker_span_limits_mm=geometry['marker_span_limits_mm'])
    return pair(old_geometry),pair(new_geometry),matrix,registration


video=cv2.VideoCapture(read(RUN/'trial.json')['video_file']);video.set(cv2.CAP_PROP_ORIENTATION_AUTO,0)
fig,axes=plt.subplots(2,2,figsize=(8,8),layout='constrained')
for index in indexes:
    row=pose.iloc[index];video.set(cv2.CAP_PROP_POS_FRAMES,int(row.frame));ok,frame=video.read();assert ok
    old,new,matrix,registration=pairs(frame,reference,original,candidate)
    difference=float(np.linalg.norm(np.array(new)-np.array(old),axis=1).max()) if old is not None and new is not None else None
    rows.append(dict(frame=int(row.frame),time_video_s=float(row.time_video_s),original=old is not None,candidate=new is not None,
        old_new_pair_max_delta_mm=difference,aruco_ids=registration['detected_marker_ids']))
    if old is None and new is not None:
        recovered.append(int(row.frame));image_points=transform_points(np.array(new),np.linalg.inv(matrix))
        canvas=frame.copy()
        for point,colour in zip(image_points,[(0,255,0),(0,0,255)]):
            cv2.circle(canvas,tuple(np.rint(point).astype(int)),6,colour,1)
        center=np.rint(image_points.mean(0)).astype(int);x,y=center
        axes.flat[len(recovered)-1].imshow(cv2.cvtColor(canvas[max(0,y-70):y+70,max(0,x-70):x+70],cv2.COLOR_BGR2RGB))
        axes.flat[len(recovered)-1].set_title(f'frame {int(row.frame)} 実色成分を再検出');axes.flat[len(recovered)-1].axis('off')
video.release();fig.savefig(OUT/'recovered_actual_colour_pairs.jpg',dpi=145);plt.close(fig)
independent=[]
IN=ROOT/'tools/logs/pixel_camera/turn1600_B_inr135_20261120_v1'
for relative in ['geometry_c0p4_Ls10p5_Le17p2_02/local_execution/run_B_INR135_1600_c0p4_Ls10p5_Le17p2_green_20261120_002',
    'geometry_c0p4_Ls10p5_Le17p2_repeat_01/local_execution/run_B_INR135_1600_c0p4_Ls10p5_Le17p2_repeat_green_20261120_001',
    'geometry_c0p4_Ls10p5_Le17p2_repeat_02/local_execution/run_B_INR135_1600_c0p4_Ls10p5_Le17p2_repeat_green_20261120_002']:
    run=IN/relative;ref=read(run/'marker_reference_used.json');geo=read(run/'body_geometry_used.json');new_geo=copy.deepcopy(geo)
    new_geo['marker_detection']['green']['minimum_area_px']=10
    data=pd.read_csv(run/'full/apparent_floor_marker_pose.csv');quality=read(run/'full/report.json')['required_motion_window_quality']
    within=data[data.time_video_s.between(*quality['required_interval_pts_s'])]
    selected=np.linspace(0,len(within)-1,20).round().astype(int)
    cap=cv2.VideoCapture(read(run/'trial.json')['video_file']);cap.set(cv2.CAP_PROP_ORIENTATION_AUTO,0)
    checks=[]
    for _,row in within.iloc[selected].iterrows():
        cap.set(cv2.CAP_PROP_POS_FRAMES,int(row.frame));ok,frame=cap.read();assert ok
        old,new,_,registration=pairs(frame,ref,geo,new_geo)
        difference=float(np.linalg.norm(np.array(new)-np.array(old),axis=1).max()) if old is not None and new is not None else None
        checks.append(dict(frame=int(row.frame),old=old is not None,new=new is not None,max_delta_mm=difference,aruco_ids=registration['detected_marker_ids']))
    cap.release();independent.append(dict(source=str(run),checks=checks))
assert len(recovered)==4
assert all(not row['original'] or (row['candidate'] and row['old_new_pair_max_delta_mm']<1e-6) for row in rows)
assert all(not check['old'] or (check['new'] and check['max_delta_mm']<1e-6) for item in independent for check in item['checks'])
result=dict(state='SAMPLED_VALIDATION_PASSED_FULL_REPROCESS_REQUIRED',candidate=str(candidate_path),
    original_minimum_area_px=15,candidate_minimum_area_px=10,recovered_real_colour_pair_frames=recovered,
    current_samples=rows,independent_runs=independent,acceptance_observation_fraction_unchanged=.95,
    colour_hue_saturation_value_and_opening_unchanged=True,marker_span_limits_unchanged=True,
    full_current_motion_validation_pending=True,original_failed_quality_preserved=True,hardware_operations=0,
    source_sha256={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in [RUN/'body_geometry_used.json',RUN/'full/report.json']})
with (OUT/'sample_validation.json').open('x',encoding='utf-8') as file:json.dump(result,file,ensure_ascii=False,indent=2,allow_nan=False)
print(json.dumps(dict(state=result['state'],recovered_frames=recovered,current_samples=len(rows),independent_samples=60),ensure_ascii=False))
