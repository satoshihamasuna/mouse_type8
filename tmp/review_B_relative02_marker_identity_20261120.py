"""保存済み走行の固定8コマで前後の検出点を重畳する。機器操作なし。"""
from pathlib import Path
import json
import sys
import cv2
import numpy as np
import pandas as pd

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_r90_20261120_v1'
RUN=BASE/'cli_session09/run_B_R90_1600_Lstart13p6_corr1p0_green_20261120_002'
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'tools/analysis')]
from video_marker_pose import marker_registration,board_to_pose_transform,transform_points

trial=json.loads((RUN/'trial.json').read_text(encoding='utf-8'))
reference=json.loads((RUN/'marker_reference_used.json').read_text(encoding='utf-8'))
pose=pd.read_csv(RUN/'full/apparent_floor_marker_pose.csv')
video=cv2.VideoCapture(trial['video_file'])
video.set(cv2.CAP_PROP_ORIENTATION_AUTO,0)
tiles=[]
rows=[]
for requested in [9.2,9.35,9.52,9.61,9.66,9.71,9.8,11.8]:
    row=pose.iloc[np.argmin(abs(pose.time_video_s-requested))]
    assert row.marker_detected_raw and row.calibration_valid
    video.set(cv2.CAP_PROP_POS_FRAMES,int(row.frame))
    ok,frame=video.read()
    assert ok
    registration=marker_registration(frame,reference)
    assert registration['available']
    matrix=board_to_pose_transform(reference)@np.asarray(registration['homography_image_to_board_mm'])
    pixels=transform_points([[row.blue_x_mm,row.blue_y_mm],[row.yellow_x_mm,row.yellow_y_mm]],np.linalg.inv(matrix))
    for point,color in zip(pixels,[(0,255,0),(255,0,255)]):
        cv2.circle(frame,tuple(np.rint(point).astype(int)),4,color,1,cv2.LINE_AA)
    center=pixels.mean(axis=0)
    left=int(np.clip(center[0]-65,0,frame.shape[1]-130))
    top=int(np.clip(center[1]-55,0,frame.shape[0]-110))
    tile=cv2.resize(frame[top:top+110,left:left+130],(325,275),interpolation=cv2.INTER_NEAREST)
    cv2.rectangle(tile,(0,0),(325,25),(0,0,0),-1)
    cv2.putText(tile,f'frame {int(row.frame)} / {row.time_video_s:.3f}s',(5,18),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1)
    tiles.append(tile)
    rows.append(dict(frame=int(row.frame),pts=float(row.time_video_s),front_px=pixels[0].tolist(),rear_px=pixels[1].tolist(),
        used_marker_ids=registration.get('used_marker_ids')))
video.release()
out=RUN/'marker_identity_fixed8.jpg'
assert not out.exists()
cv2.imwrite(str(out),np.vstack([np.hstack(tiles[:4]),np.hstack(tiles[4:])]))
with (RUN/'marker_identity_fixed8.json').open('x',encoding='utf-8') as output:
    json.dump(dict(front_green_rear_magenta=True,frames=rows,manual_identity_review='pending'),output,indent=2)
print(out)
