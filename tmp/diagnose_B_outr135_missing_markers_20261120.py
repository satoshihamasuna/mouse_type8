"""取得済み動画の欠測箇所と色成分を可視化する。閾値・採用判定・原本は変更しない。"""
from pathlib import Path
import copy
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
OUT=BASE/'marker_diagnostic_v1';assert not OUT.exists();OUT.mkdir()
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'tools/analysis')]
from video_marker_pose import marker_registration,registration_quad,pose_world_corners,colour_components,select_marker_pair
trial=read(RUN/'trial.json');reference=read(RUN/'marker_reference_used.json');geometry=read(RUN/'body_geometry_used.json')
pose=pd.read_csv(RUN/'full/apparent_floor_marker_pose.csv')
bad=pose.index[~pose.marker_detected_raw.astype(bool)].tolist()
assert len(bad)==9
selected=sorted(set(bad+[max(0,min(bad)-2),min(len(pose)-1,max(bad)+2)]))
video=cv2.VideoCapture(trial['video_file']);video.set(cv2.CAP_PROP_ORIENTATION_AUTO,0)
fig,axes=plt.subplots(3,4,figsize=(12,9),layout='constrained');records=[]
for ax,index in zip(axes.flat,selected):
    row=pose.iloc[index];video.set(cv2.CAP_PROP_POS_FRAMES,int(row.frame));ok,frame=video.read();assert ok
    registration=marker_registration(frame,reference);quad=registration_quad(registration);assert quad is not None
    matrix=cv2.getPerspectiveTransform(quad,pose_world_corners(reference['board_side_mm']))
    center=np.array([row.midpoint_image_x_px,row.midpoint_image_y_px]);x,y=np.rint(center).astype(int)
    x0,x1=max(0,x-65),min(frame.shape[1],x+65);y0,y1=max(0,y-65),min(frame.shape[0],y+65)
    variants=[]
    for sat,area in [(25,15),(35,15),(45,15),(35,10),(35,5)]:
        detection=copy.deepcopy(geometry['marker_detection']);detection['green']['minimum_saturation']=sat;detection['green']['minimum_area_px']=area
        front=colour_components(frame,'green',detection);rear=colour_components(frame,'red',detection)
        pair=select_marker_pair(front,rear,matrix,None,nominal_marker_span_mm=geometry['nominal_marker_span_mm'],marker_span_limits_mm=geometry['marker_span_limits_mm'])
        near_front=[dict(x_px=float(p[0]),y_px=float(p[1]),area=int(a)) for p,a in front if np.linalg.norm(p-center)<80]
        near_rear=[dict(x_px=float(p[0]),y_px=float(p[1]),area=int(a)) for p,a in rear if np.linalg.norm(p-center)<80]
        variants.append(dict(green_min_saturation=sat,green_min_area=area,pair_found=pair is not None,front=near_front,rear=near_rear))
    crop=frame[y0:y1,x0:x1]
    ax.imshow(cv2.cvtColor(crop,cv2.COLOR_BGR2RGB));ax.set_title(f'frame {int(row.frame)} / {row.time_video_s:.4f}s\nraw pair {bool(row.marker_detected_raw)}',fontsize=9);ax.axis('off')
    cv2.imwrite(str(OUT/f'frame_{int(row.frame)}.jpg'),frame)
    records.append(dict(frame=int(row.frame),time_video_s=float(row.time_video_s),original_observed=bool(row.marker_detected_raw),
        aruco_ids=registration['detected_marker_ids'],crop_center_only_uses_interpolated_pose=True,variants=variants))
for ax in axes.flat[len(selected):]:ax.axis('off')
video.release();fig.savefig(OUT/'missing_frames_contact_sheet.jpg',dpi=145);plt.close(fig)
with (OUT/'diagnostic.json').open('x',encoding='utf-8') as file:json.dump(dict(state='DIAGNOSTIC_ONLY_NOT_ADOPTED',
    original_required_motion_observation_fraction=read(RUN/'full/report.json')['required_motion_window_quality']['observation_rate'],
    records=records,original_and_acceptance_unchanged=True,hardware_operations=0),file,ensure_ascii=False,indent=2)
print(json.dumps(dict(missing_frames=[int(pose.iloc[i].frame) for i in bad],sheet=str(OUT/'missing_frames_contact_sheet.jpg')),ensure_ascii=False))
