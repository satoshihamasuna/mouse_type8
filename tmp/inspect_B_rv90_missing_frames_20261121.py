"""欠測フレームの元画像を並べる。補間値は表示用の切出し位置だけに使う。"""
from pathlib import Path
import json
import cv2
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
RUN=ROOT/'tools/logs/pixel_camera/turn1600_B_rv90_20261120_v1/geometry_ff0p0006_Ls3p3_Le18p9_c0p0_02/local_execution/run_B_RV90_1600_geom3p3_18p9_c0p0_ff0p0006_20261121_002'
OUT=RUN/'missing_frames_diagnostic_v1'
assert not OUT.exists();OUT.mkdir()
trial=json.loads((RUN/'trial.json').read_text(encoding='utf-8'))
pose=pd.read_csv(RUN/'full/apparent_floor_marker_pose.csv')
valid=pose.marker_detected_raw.to_numpy(bool)
bad=pose.loc[~valid]
selected=pose.loc[pose.frame.isin(sorted(set(bad.frame.tolist()+[2267,2278,2287])))].copy()
times=pose.time_video_s.to_numpy()
centers=np.column_stack([np.interp(selected.time_video_s,times[valid],pose.loc[valid,key])
    for key in ('midpoint_image_x_px','midpoint_image_y_px')])
capture=cv2.VideoCapture(trial['video_file']);capture.set(cv2.CAP_PROP_ORIENTATION_AUTO,0)
capture.set(cv2.CAP_PROP_POS_MSEC,float(selected.time_video_s.min()*1000-60))
frames={}
while True:
    ok,frame=capture.read()
    if not ok:break
    stamp=capture.get(cv2.CAP_PROP_POS_MSEC)/1000
    if stamp>selected.time_video_s.max()+.01:break
    errors=np.abs(selected.time_video_s.to_numpy()-stamp)
    if errors.min()<.001:
        index=int(np.argmin(errors));frames[index]=(stamp,frame.copy())
capture.release()
assert len(frames)==len(selected),(len(frames),len(selected))
fig,axes=plt.subplots(4,3,figsize=(12,16),layout='constrained')
records=[]
for index,((_,row),center,axis) in enumerate(zip(selected.iterrows(),centers,axes.ravel())):
    stamp,frame=frames[index];x,y=np.rint(center).astype(int)
    crop=frame[max(0,y-80):min(frame.shape[0],y+80),max(0,x-80):min(frame.shape[1],x+80)]
    axis.imshow(cv2.cvtColor(crop,cv2.COLOR_BGR2RGB),interpolation='nearest')
    axis.set_title(f"frame {int(row.frame)} / {stamp:.6f}s\n{'OBSERVED' if row.marker_detected_raw else 'MISSED'}")
    axis.axis('off')
    records.append(dict(frame=int(row.frame),cached_pts_s=float(row.time_video_s),decoded_pts_s=stamp,
        marker_observed=bool(row.marker_detected_raw),approximate_crop_center_px=center.tolist()))
fig.suptitle('Original pixels around missing marker pairs (crop centers are display estimates)',fontsize=13)
fig.savefig(OUT/'missing_frame_contact_sheet.png',dpi=120);plt.close(fig)
(OUT/'manifest.json').write_text(json.dumps(dict(source_video=trial['video_file'],source_cache=str(RUN/'full/apparent_floor_marker_pose.csv'),
    records=records,estimated_crop_centers_used_as_observations=False,measurement_or_detector_changed=False),ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(str(OUT/'missing_frame_contact_sheet.png'))
