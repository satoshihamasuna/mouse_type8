"""合格した開始静止像を使い、実登録した盤面の開始中点を示す。実機操作なし。"""
from pathlib import Path
import json
import sys
import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools/analysis'))
from video_marker_pose import marker_registration,registration_quad,pose_world_corners,board_to_pose_transform,transform_points
BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_outr135_20261120_v1'
PRE=BASE/'c2p1_Ls0_Le37_03/local_execution/prepare_A_OUTR135_1800_c2p1_Ls0_Le37_20261120_003'
OUT=BASE/'telemetry_period_recovery_20261121_v1'
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
plan=read(PRE/'static_plan.json');check=read(PRE/'static_check.json');assert check['passed']
reference=read(Path(plan['reference']))
cap=cv2.VideoCapture(plan['video']);cap.set(cv2.CAP_PROP_ORIENTATION_AUTO,0)
cap.set(cv2.CAP_PROP_POS_MSEC,4000);ok,frame=cap.read();assert ok
frame_number=int(cap.get(cv2.CAP_PROP_POS_FRAMES))-1;cap.release()
registration=marker_registration(frame,reference)
assert {2,3}<=set(registration['detected_marker_ids'])
quad=registration_quad(registration);assert quad is not None
image_to_pose=cv2.getPerspectiveTransform(quad,pose_world_corners(reference.get('board_side_mm',360)))
board_to_image=np.linalg.inv(image_to_pose)@board_to_pose_transform(reference)
project=lambda p:transform_points(np.array(p,dtype=float),board_to_image)
centre=project([[180,135]])[0]
theta=np.linspace(0,2*np.pi,80);circle=project(np.c_[180+10*np.cos(theta),135+10*np.sin(theta)])
u=np.array([2**-.5,2**-.5]);direction=project([np.array([180,135])+35*u])[0]
plt.rcParams['font.family']='Yu Gothic'
fig,ax=plt.subplots(figsize=(11,7),layout='constrained')
ax.imshow(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB));ax.plot(*circle.T,c='lime',lw=2)
ax.scatter(*centre,c='lime',marker='+',s=180,lw=2)
ax.annotate('',xy=direction,xytext=centre,arrowprops=dict(arrowstyle='->',color='red',lw=2))
ax.annotate('中点：右180・下135 mm付近\n前の赤を右下へ（向きは大まかに）',xy=centre,
    xytext=(60,frame.shape[0]-90),color='white',fontsize=12,
    bbox=dict(facecolor='black',alpha=.7),arrowprops=dict(arrowstyle='->',color='lime',lw=2))
ax.set_title('機体A・out135の開始配置例\n移動前に合格した静止写真／緑：開始中点付近・赤矢印：前向き')
ax.set_axis_off();fig.savefig(OUT/'start_placement_photo_accepted_20261121.png',dpi=140);plt.close(fig)
with (OUT/'start_placement_photo_provenance.json').open('x',encoding='utf-8') as file:
    json.dump(dict(source_video=plan['video'],source_frame=frame_number,registered_ids=registration['detected_marker_ids'],
        used_aruco=[2,3],target_board_mm=[180,135],source_static_check=str(PRE/'static_check.json'),
        source_was_accepted=True,current_placement_confirmed=False,hardware_operations=0),file,indent=2)
print('Accepted placement example generated from one registered actual frame.')
