"""前面積候補の誤認を棄却し、見えている薄い後赤の色成分を診断。採用判定は別。"""
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
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
with (BASE/'green_area10_validation_v1/visual_review_rejected.json').open('x',encoding='utf-8') as file:
    json.dump(dict(state='REJECTED_FALSE_POSITIVE_COLOUR_PAIRS',frames=[2337,2347,2350,2353],
        reason='拡大画像で緑の格子背景と赤の基板を選んでいた。実前/実後マーカーではないため面積10px候補を不採用。',
        candidate_deployed=False,original_failed_quality_retained=True),file,ensure_ascii=False,indent=2)
OUT=BASE/'rear_red_saturation_diagnostic_v1';assert not OUT.exists();OUT.mkdir()
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'tools/analysis')]
from video_marker_pose import marker_registration,registration_quad,pose_world_corners,colour_components,select_marker_pair,transform_points
trial=read(RUN/'trial.json');geometry=read(RUN/'body_geometry_used.json');reference=read(RUN/'marker_reference_used.json')
pose=pd.read_csv(RUN/'full/apparent_floor_marker_pose.csv');bad=pose[~pose.marker_detected_raw.astype(bool)]
cap=cv2.VideoCapture(trial['video_file']);cap.set(cv2.CAP_PROP_ORIENTATION_AUTO,0)
plt.rcParams['font.family']='Yu Gothic'
fig,axes=plt.subplots(3,3,figsize=(10,10),layout='constrained');rows=[]
for ax,(_,row) in zip(axes.flat,bad.iterrows()):
    cap.set(cv2.CAP_PROP_POS_FRAMES,int(row.frame));ok,frame=cap.read();assert ok
    registration=marker_registration(frame,reference);quad=registration_quad(registration);assert quad is not None
    matrix=cv2.getPerspectiveTransform(quad,pose_world_corners(reference['board_side_mm']))
    variants=[];selected=None
    for sat in (40,35,30,25):
        detection=copy.deepcopy(geometry['marker_detection']);detection['red']['minimum_saturation']=sat
        pair=select_marker_pair(colour_components(frame,'green',detection),colour_components(frame,'red',detection),matrix,None,
            nominal_marker_span_mm=geometry['nominal_marker_span_mm'],marker_span_limits_mm=geometry['marker_span_limits_mm'])
        image_points=transform_points(np.array(pair),np.linalg.inv(matrix)) if pair is not None else None
        variants.append(dict(red_min_saturation=sat,pair=pair is not None,image_points_px=image_points.tolist() if image_points is not None else None))
        if sat==30:selected=image_points
    canvas=frame.copy()
    if selected is not None:
        for point,colour in zip(selected,[(0,255,0),(0,0,255)]):cv2.circle(canvas,tuple(np.rint(point).astype(int)),6,colour,1)
    center=np.array([row.midpoint_image_x_px,row.midpoint_image_y_px]);x,y=np.rint(center).astype(int)
    ax.imshow(cv2.cvtColor(canvas[max(0,y-70):y+70,max(0,x-70):x+70],cv2.COLOR_BGR2RGB))
    ax.set_title(f'frame {int(row.frame)} / 赤S30 pair {selected is not None}',fontsize=9);ax.axis('off')
    rows.append(dict(frame=int(row.frame),variants=variants,aruco_ids=registration['detected_marker_ids']))
cap.release();fig.savefig(OUT/'red_s30_pairs.jpg',dpi=145);plt.close(fig)
with (OUT/'diagnostic.json').open('x',encoding='utf-8') as file:json.dump(dict(state='VISUAL_REVIEW_REQUIRED_NOT_DEPLOYED',rows=rows,
    original_and_quality_thresholds_unchanged=True,hardware_operations=0),file,ensure_ascii=False,indent=2)
print(json.dumps(dict(red_s30_pair_count=sum(row['variants'][2]['pair'] for row in rows),missing_frames=len(rows)),ensure_ascii=False))
