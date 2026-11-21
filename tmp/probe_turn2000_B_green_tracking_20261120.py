"""緑前の誤対応を保存動画だけで検証する。共有実装・撮影設定は変更しない。"""
from pathlib import Path
import copy
import inspect
import json
import sys

import cv2
import numpy as np
import pandas as pd

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn2000_B_r90_20261120_v1'
RUN=BASE/'cli_green_session01/run_B_R90_2000_omKi005_green_20261120_001'
OUT=RUN/'green_tracking_probe_v1'
OUT.mkdir(exist_ok=False)
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'tools/analysis')]
import video_marker_pose as pose

read=lambda p:json.loads(Path(p).read_text(encoding='utf-8'))
trial=read(RUN/'trial.json');reference=read(RUN/'marker_reference_used.json');geometry=read(RUN/'body_geometry_used.json')
source=inspect.getsource(pose.select_marker_pair)
anchor='            # The true circles are the largest similarly sized colour regions.'
assert source.count(anchor)==1
gates='''            if gate_previous_pair is not None:
                if (np.linalg.norm(blue_mm-gate_previous_pair[0]) > gate_max_step
                        or np.linalg.norm(yellow_mm-gate_previous_pair[1]) > gate_max_step):
                    continue
                if abs(separation_mm-np.linalg.norm(gate_previous_pair[0]-gate_previous_pair[1])) > 4.0:
                    continue
'''
scope=dict(vars(pose));exec(source.replace(anchor,gates+anchor),scope)
selector=scope['select_marker_pair']
variants={name:dict(previous=None,time=None,rows=[]) for name in ('original','low_threshold','low_threshold_tracked')}
settings=copy.deepcopy(geometry['marker_detection'])
settings['green'].update(minimum_saturation=15,minimum_area_px=3)
settings['red'].update(minimum_saturation=25)
tiles=[]
for index,frame,pts in pose.iter_video_frames(Path(trial['video_file']),True,(8.85,9.55),1):
    registration=pose.marker_registration(frame,reference)
    if not registration['available']:
        continue
    h=np.asarray(registration['homography_image_to_board_mm']);inv=np.linalg.inv(h)
    original_front=pose.colour_components(frame,'green',geometry['marker_detection'])
    original_rear=pose.colour_components(frame,'red',geometry['marker_detection'])
    front=pose.colour_components(frame,'green',settings);rear=pose.colour_components(frame,'red',settings)
    for name,state in variants.items():
        tracking=name.endswith('_tracked')
        age=pts-state['time'] if state['time'] is not None else None
        scope['gate_previous_pair']=state['previous'] if tracking and age is not None and age <= .05 else None
        scope['gate_max_step']=3.+4000.*age if age is not None else None
        components=(original_front,original_rear) if name=='original' else (front,rear)
        pair=selector(*components,h,None,board_side_mm=360,nominal_marker_span_mm=50,marker_span_limits_mm=[40,65])
        row=dict(frame=index,pts_s=pts,observed=pair is not None)
        if pair is not None:
            a,b=pair;pixels=pose.transform_points([a,b],inv)
            row.update(front_x=float(a[0]),front_y=float(a[1]),rear_x=float(b[0]),rear_y=float(b[1]),
                span_mm=float(np.linalg.norm(a-b)),front_x_px=float(pixels[0,0]),front_y_px=float(pixels[0,1]))
            state['previous']=np.array(pair);state['time']=pts
            if name=='low_threshold_tracked' and index in (2186,2187,2193,2194,2200,2210,2220,2231,2232,2240,2260,2300):
                annotated=frame.copy()
                for point,col in zip(pixels,[(0,255,0),(0,0,255)]):
                    cv2.circle(annotated,tuple(np.rint(point).astype(int)),7,col,2,cv2.LINE_AA)
                mid=pixels.mean(axis=0);x,y=np.rint(mid).astype(int);x0=max(0,min(frame.shape[1]-200,x-100));y0=max(0,min(frame.shape[0]-200,y-100))
                tile=cv2.resize(annotated[y0:y0+200,x0:x0+200],(300,300))
                cv2.putText(tile,str(index),(8,24),cv2.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),2,cv2.LINE_AA);tiles.append(tile)
        state['rows'].append(row)
    if index%120==0:print(f'frame{index} PTS{pts:.3f}',flush=True)
results=[]
for name,state in variants.items():
    data=pd.DataFrame(state['rows']);data.to_csv(OUT/(name+'.csv'),index=False)
    good=data[data.observed]
    # 固定背景を拾っていたx690/y139付近。診断だけに使い、マスク・採否条件には使わない。
    distractor=(np.hypot(good.front_x_px-689.96,good.front_y_px-139.04)<3).sum()
    results.append(dict(variant=name,frames=len(data),observed=int(data.observed.sum()),rate=float(data.observed.mean()),
        diagnostic_fixed_background_selections=int(distractor),span_range_mm=[float(good.span_mm.min()),float(good.span_mm.max())]))
if tiles:
    while len(tiles)%4:tiles.append(np.zeros_like(tiles[0]))
    cv2.imwrite(str(OUT/'tracked_contact_sheet.jpg'),np.vstack([np.hstack(tiles[i:i+4]) for i in range(0,len(tiles),4)]))
with (OUT/'report.json').open('x',encoding='utf-8') as stream:json.dump(dict(results=results,scope='Offline prototype on saved entry/turn footage; no shared source or acquisition settings changed',tracking=dict(max_marker_speed_mm_s=4000,position_margin_mm=3,max_span_change_mm=4,max_gap_s=.05),candidate_detection=settings),stream,indent=2)
print(json.dumps(results,ensure_ascii=True),flush=True)
