"""欠測フレームを可視化する。補間位置は切り出し専用で測定値には使用しない。"""
from pathlib import Path
import json
import sys
import cv2
import numpy as np
import pandas as pd

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'tools/analysis')]
from video_marker_pose import iter_video_frames,colour_components
from marker_detection import colour_mask
RUN=ROOT/'tools/logs/pixel_camera/turn1600_B_inr45_20261120_v1/geometry02_repeat_02/local_execution/run_B_INR45_1600_geometry02_repeat02_green_20261120_008'
OUT=RUN/'marker_loss_diagnostic_v2';OUT.mkdir(exist_ok=False)
read=lambda p:json.loads(p.read_text(encoding='utf-8'))
trial=read(RUN/'trial.json');geometry=read(RUN/'body_geometry_used.json')
pose=pd.read_csv(RUN/'independent_motion_window_v1/apparent_floor_marker_pose.csv')
times=[9.3529,9.3613,9.3821,9.4029,9.4322,9.4531]
selected={int(pose.iloc[(pose.time_video_s-time).abs().argmin()]['frame']) for time in times}
valid=pose.marker_detected_raw.to_numpy(bool)
good=pose.loc[valid]
rows=[];tiles=[];masktiles=[]
for index,frame,stamp in iter_video_frames(Path(trial['video_file']),True,(9.34,9.47),1):
    if index not in selected:continue
    center=np.array([np.interp(stamp,good.time_video_s,good[col]) for col in ('midpoint_image_x_px','midpoint_image_y_px')])
    x0,y0=np.rint(center-90).astype(int);x1,y1=x0+180,y0+180
    x0,y0=max(0,x0),max(0,y0);x1,y1=min(frame.shape[1],x1),min(frame.shape[0],y1)
    crop=frame[y0:y1,x0:x1].copy();row=pose.loc[pose.frame==index].iloc[0]
    tile=cv2.resize(crop,(480,480),interpolation=cv2.INTER_NEAREST)
    label=f'{stamp:.6f}s / frame {index} / '+('OBSERVED' if row['marker_detected_raw'] else 'MISSING')
    cv2.rectangle(tile,(0,0),(480,45),(0,0,0),-1);cv2.putText(tile,label,(8,28),cv2.FONT_HERSHEY_SIMPLEX,.54,(255,255,255),1,cv2.LINE_AA)
    tiles.append(tile)
    hsv=cv2.cvtColor(crop,cv2.COLOR_BGR2HSV)
    green,_=colour_mask(hsv,'green',geometry['marker_detection']);red,_=colour_mask(hsv,'red',geometry['marker_detection'])
    masks=np.zeros_like(crop);masks[:,:,1]=green;masks[:,:,2]=red
    masktile=cv2.resize(masks,(480,480),interpolation=cv2.INTER_NEAREST)
    cv2.rectangle(masktile,(0,0),(480,45),(0,0,0),-1);cv2.putText(masktile,label,(8,28),cv2.FONT_HERSHEY_SIMPLEX,.54,(255,255,255),1,cv2.LINE_AA)
    masktiles.append(masktile)
    components={colour:[dict(center_px=point.tolist(),area_px=int(area)) for point,area in colour_components(crop,colour,geometry['marker_detection'])]
                for colour in ('green','red')}
    predicted_front=np.array([np.interp(stamp,good.time_video_s,good[col]) for col in ('front_image_x_px','front_image_y_px')])
    predicted_rear=2*center-predicted_front
    rx,ry=predicted_rear-[x0,y0]
    yy,xx=np.ogrid[:crop.shape[0],:crop.shape[1]]
    rear_region=(xx-rx)**2+(yy-ry)**2<=10**2
    samples=hsv[rear_region]
    chromatic=samples[(samples[:,1]>=15)&(samples[:,2]>=50)]
    counts=np.bincount(chromatic[:,0],minlength=180) if len(chromatic) else np.zeros(180,dtype=int)
    top=np.argsort(counts)[-12:][::-1]
    rows.append(dict(frame=int(index),pts_s=float(stamp),observed=bool(row['marker_detected_raw']),crop_bounds_px=[int(v) for v in (x0,y0,x1,y1)],
        components=components,crop_center_interpolation_used_for_measurement=False,
        rear_roi_top_hues=[dict(hue=int(h),pixels=int(counts[h])) for h in top if counts[h]],
        rear_roi_hsv=chromatic.tolist(),predicted_rear_roi_used_for_measurement=False))
assert len(tiles)==6
cv2.imwrite(str(OUT/'frames.png'),np.vstack([np.hstack(tiles[:3]),np.hstack(tiles[3:])]))
cv2.imwrite(str(OUT/'masks.png'),np.vstack([np.hstack(masktiles[:3]),np.hstack(masktiles[3:])]))
with (OUT/'diagnostic.json').open('x',encoding='utf-8') as f:json.dump(dict(observations=rows,
    current_detector=geometry['marker_detection'],operator_report='とくになし：照明/マーカー/取付に特段の変更なし',
    acceptance_changed=False,missing_measurements_filled=False,hardware_commands_sent=0),f,ensure_ascii=False,indent=2)
print(OUT,flush=True)
