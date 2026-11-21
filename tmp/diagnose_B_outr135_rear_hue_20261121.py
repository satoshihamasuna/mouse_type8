"""実後マーカーの桃色化に対する色相候補を診断。配備や採用は行わない。"""
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
out=run/'rear_hue_diagnostic_v1';assert not out.exists();out.mkdir()
read=lambda path:json.loads(path.read_text(encoding='utf-8-sig'))
sys.path[:0]=[str(root/'tools'),str(root/'tools/analysis')]
from video_marker_pose import marker_registration,registration_quad,pose_world_corners,colour_components,select_marker_pair,transform_points
cv2.setNumThreads(4)
trial=read(run/'trial.json');ref=read(run/'marker_reference_used.json');geo=read(run/'body_geometry_used.json')
pose=pd.read_csv(run/'full/apparent_floor_marker_pose.csv');missing=pose[~pose.marker_detected_raw.astype(bool)]
figures={low:plt.subplots(4,4,figsize=(11,11),layout='constrained') for low in (165,160,155)}
cap=cv2.VideoCapture(trial['video_file']);cap.set(cv2.CAP_PROP_ORIENTATION_AUTO,0);rows=[]
for index,(_,row) in enumerate(missing.iterrows()):
    cap.set(cv2.CAP_PROP_POS_FRAMES,int(row.frame));ok,frame=cap.read();assert ok
    registration=marker_registration(frame,ref);quad=registration_quad(registration);assert quad is not None
    matrix=cv2.getPerspectiveTransform(quad,pose_world_corners(ref['board_side_mm']))
    x,y=np.rint([row.midpoint_image_x_px,row.midpoint_image_y_px]).astype(int)
    record=dict(frame=int(row.frame),time_video_s=float(row.time_video_s),aruco_ids=registration['detected_marker_ids'],variants=[])
    for low in figures:
        detection=copy.deepcopy(geo['marker_detection']);detection['red']['hue_ranges']=[[0,10],[low,179]]
        pair=select_marker_pair(colour_components(frame,'green',detection),colour_components(frame,'red',detection),matrix,None,
            nominal_marker_span_mm=geo['nominal_marker_span_mm'],marker_span_limits_mm=geo['marker_span_limits_mm'])
        pixels=transform_points(np.array(pair),np.linalg.inv(matrix)) if pair is not None else None
        canvas=frame.copy()
        if pixels is not None:
            for point,colour in zip(pixels,[(0,255,0),(0,0,255)]):cv2.circle(canvas,tuple(np.rint(point).astype(int)),6,colour,1)
        ax=figures[low][1].flat[index];ax.imshow(cv2.cvtColor(canvas[max(0,y-70):y+70,max(0,x-70):x+70],cv2.COLOR_BGR2RGB))
        ax.set_title(f'{int(row.frame)} / H{low}..179 pair {pixels is not None}',fontsize=8);ax.axis('off')
        record['variants'].append(dict(red_high_hue_lower_bound=low,pair_found=pair is not None,
            image_points_px=pixels.tolist() if pixels is not None else None))
    if int(row.frame)==2252:
        # 写真を見て指定した実後マーカー内部の診断ROI。測定位置としては使用しない。
        hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        roi=hsv[169:177,590:598]
        record['manual_actual_rear_roi']=dict(box_xyxy=[590,169,598,177],
            median_hsv=np.median(roi.reshape(-1,3),axis=0).tolist(),purpose='colour_diagnostic_only')
    rows.append(record)
cap.release()
for low,(fig,axes) in figures.items():
    for ax in axes.flat[len(missing):]:ax.axis('off')
    fig.savefig(out/f'rear_hue{low}_pairs.jpg',dpi=140);plt.close(fig)
with (out/'diagnostic.json').open('x',encoding='utf-8') as stream:
    json.dump(dict(state='DIAGNOSTIC_ONLY_VISUAL_REVIEW_PENDING',rows=rows,hardware_operations=0,
        profile_deployed=False,original_failed_quality_preserved=True,acceptance_threshold_unchanged=.95),stream,ensure_ascii=False,indent=2)
print(json.dumps(dict(pair_counts={low:sum(next(v for v in row['variants'] if v['red_high_hue_lower_bound']==low)['pair_found'] for row in rows) for low in figures},output=str(out)),ensure_ascii=False))
