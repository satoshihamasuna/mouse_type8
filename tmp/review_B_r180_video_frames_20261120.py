"""B1600右180度の保存動画の代表フレームと実検出点を保存。実機操作なし。"""
from pathlib import Path
import json
import sys
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'tools/analysis')]
from video_marker_pose import (marker_registration,registration_quad,pose_world_corners,
    board_to_pose_transform,colour_components,select_marker_pair,transform_points)
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_r180_20261120_v1'
RUN=BASE/'baseline01/local_execution/run_B_R180_1600_baseline_green_20261120_001'
OUT=RUN/'representative_frames';OUT.mkdir(exist_ok=False)
read=lambda p:json.loads(Path(p).read_text(encoding='utf-8-sig'))
trial=read(RUN/'trial.json')
reference=read(RUN/'marker_reference_used.json')
geometry=read(RUN/'body_geometry_used.json')
capture=read(trial['capture_json'])
video=cv2.VideoCapture(capture['video_file']);video.set(cv2.CAP_PROP_ORIENTATION_AUTO,0)
rows=[];tiles=[]
for requested in [9.40,9.45,9.50,9.55,9.60,9.65,9.70,9.75,9.85,11.5]:
    video.set(cv2.CAP_PROP_POS_MSEC,requested*1000)
    ok,frame=video.read();assert ok
    pts=video.get(cv2.CAP_PROP_POS_MSEC)/1000
    registration=marker_registration(frame,reference);quad=registration_quad(registration)
    row=dict(requested_pts=requested,pts=pts,registered_aruco_ids=registration['detected_marker_ids'],
        calibration_valid=quad is not None)
    if quad is not None:
        matrix=cv2.getPerspectiveTransform(quad,pose_world_corners(reference['board_side_mm']))
        pair=select_marker_pair(colour_components(frame,'green',geometry['marker_detection']),
            colour_components(frame,'red',geometry['marker_detection']),matrix,None,
            nominal_marker_span_mm=geometry['nominal_marker_span_mm'],marker_span_limits_mm=geometry['marker_span_limits_mm'])
        row['detected']=pair is not None
        if pair is not None:
            pixels=transform_points(pair,np.linalg.inv(matrix))
            board=transform_points(pair,np.linalg.inv(board_to_pose_transform(reference)))
            row.update(front_board_mm=board[0].tolist(),rear_board_mm=board[1].tolist(),
                center_board_mm=((board[0]+board[1])/2).tolist())
            for point,colour in zip(pixels,[(0,255,0),(255,0,255)]):
                cv2.circle(frame,tuple(np.rint(point).astype(int)),4,colour,1)
    rows.append(row)
    tile=cv2.resize(frame[36:601,289:859],(342,339))
    cv2.rectangle(tile,(0,0),(342,23),(0,0,0),-1)
    cv2.putText(tile,f'{pts:.3f}s ArUco{len(row["registered_aruco_ids"])} body{row.get("detected",False)}',
        (4,16),cv2.FONT_HERSHEY_SIMPLEX,.42,(255,255,255),1)
    tiles.append(tile)
video.release()
while len(tiles)%4:tiles.append(np.zeros_like(tiles[0]))
assert cv2.imwrite(str(OUT/'motion_frames.jpg'),np.vstack([np.hstack(tiles[i:i+4]) for i in range(0,len(tiles),4)]))
with (OUT/'observations.json').open('x',encoding='utf-8') as output:
    json.dump(dict(scope='代表10フレームの視認確認。全区間の観測品質と独立幾何校正の代用にはしない。',rows=rows),output,ensure_ascii=False,indent=2)
print(OUT/'motion_frames.jpg')
