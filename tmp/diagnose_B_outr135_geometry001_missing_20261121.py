"""欠測13画像の実色成分を診断。候補を採用/配備せず、原本・品質閾値を維持。"""
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
root=Path(__file__).resolve().parents[1]
base=root/'tools/logs/pixel_camera/turn1600_B_outr135_20261121_resume_v1'
run=base/'geometry_c1p0_Ls5p3_Le31_01/local_execution/run_B_OUTR135_1600_c1p0_Ls5p3_Le31_green_20261121_001'
out=run/'missing_marker_diagnostic_v1';assert not out.exists();out.mkdir()
read=lambda path:json.loads(path.read_text(encoding='utf-8-sig'))
sys.path[:0]=[str(root/'tools'),str(root/'tools/analysis')]
from video_marker_pose import marker_registration,registration_quad,pose_world_corners,colour_components,select_marker_pair,transform_points
cv2.setNumThreads(4)
trial=read(run/'trial.json');ref=read(run/'marker_reference_used.json');geo=read(run/'body_geometry_used.json')
pose=pd.read_csv(run/'full/apparent_floor_marker_pose.csv')
missing=pose[~pose.marker_detected_raw.astype(bool)]
assert len(missing)==13
variants={'original':{},'red_s25':{'red':25},'red_s20':{'red':20},'green_s30':{'green':30},'green_s25':{'green':25}}
figures={name:plt.subplots(4,4,figsize=(11,11),layout='constrained') for name in variants}
cap=cv2.VideoCapture(trial['video_file']);cap.set(cv2.CAP_PROP_ORIENTATION_AUTO,0)
records=[]
for number,(_,row) in enumerate(missing.iterrows()):
    cap.set(cv2.CAP_PROP_POS_FRAMES,int(row.frame));ok,frame=cap.read();assert ok
    registration=marker_registration(frame,ref);quad=registration_quad(registration)
    assert quad is not None and len(set(registration['detected_marker_ids']))>=2
    matrix=cv2.getPerspectiveTransform(quad,pose_world_corners(ref['board_side_mm']))
    center=np.array([row.midpoint_image_x_px,row.midpoint_image_y_px]);x,y=np.rint(center).astype(int)
    current=dict(frame=int(row.frame),time_video_s=float(row.time_video_s),
        registered_aruco_ids=registration['detected_marker_ids'],crop_center_only_is_interpolated=True,variants=[])
    for name,change in variants.items():
        detection=copy.deepcopy(geo['marker_detection'])
        for colour,saturation in change.items():detection[colour]['minimum_saturation']=saturation
        front=colour_components(frame,'green',detection);rear=colour_components(frame,'red',detection)
        pair=select_marker_pair(front,rear,matrix,None,nominal_marker_span_mm=geo['nominal_marker_span_mm'],marker_span_limits_mm=geo['marker_span_limits_mm'])
        pixels=transform_points(np.array(pair),np.linalg.inv(matrix)) if pair is not None else None
        canvas=frame.copy()
        if pixels is not None:
            for point,colour in zip(pixels,[(0,255,0),(0,0,255)]):cv2.circle(canvas,tuple(np.rint(point).astype(int)),6,colour,1)
        ax=figures[name][1].flat[number]
        ax.imshow(cv2.cvtColor(canvas[max(0,y-70):min(frame.shape[0],y+70),max(0,x-70):min(frame.shape[1],x+70)],cv2.COLOR_BGR2RGB))
        ax.set_title(f'{int(row.frame)} {row.time_video_s:.4f}s / pair {pixels is not None}',fontsize=8);ax.axis('off')
        current['variants'].append(dict(name=name,pair_found=pair is not None,image_points_px=pixels.tolist() if pixels is not None else None,
            near_front=[dict(x_px=float(point[0]),y_px=float(point[1]),area=int(area)) for point,area in front if np.linalg.norm(point-center)<90],
            near_rear=[dict(x_px=float(point[0]),y_px=float(point[1]),area=int(area)) for point,area in rear if np.linalg.norm(point-center)<90]))
    cv2.imwrite(str(out/f'frame_{int(row.frame)}.jpg'),frame)
    records.append(current)
cap.release()
for name,(fig,axes) in figures.items():
    for ax in axes.flat[len(missing):]:ax.axis('off')
    fig.savefig(out/f'{name}_pairs.jpg',dpi=140);plt.close(fig)
with (out/'diagnostic.json').open('x',encoding='utf-8') as stream:
    json.dump(dict(state='DIAGNOSTIC_ONLY_NOT_DEPLOYED',records=records,hardware_operations=0,
        original_profile_preserved=True,quality_threshold_unchanged=.95),stream,ensure_ascii=False,indent=2)
print(json.dumps(dict(missing_frames=missing.frame.astype(int).tolist(),
    pair_counts={name:sum(next(v for v in row['variants'] if v['name']==name)['pair_found'] for row in records) for name in variants},
    output=str(out)),ensure_ascii=False))
