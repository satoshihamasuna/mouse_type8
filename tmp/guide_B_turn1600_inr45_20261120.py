"""保存画像とArUco登録で開始配置の案内図を作る。撮影や実機操作は行わない。"""
from pathlib import Path
import json
import sys
import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'tools/analysis')]
from video_marker_pose import marker_registration, registration_quad, pose_world_corners, board_to_pose_transform, transform_points
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr45_20261120_v1'
source=ROOT/'tools/logs/pixel_camera/turn1600_B_inr135_20261120_v1/geometry_c0p4_Ls10p5_Le17p2_repeat_01/local_execution/camera_before_ble_B_INR135_1600_c0p4_Ls10p5_Le17p2_repeat_green_20261120_001/static_plan.json'
plan=json.loads(source.read_text(encoding='utf-8'))
reference=json.loads((BASE/'marker_reference_B_current.json').read_text(encoding='utf-8'))
cap=cv2.VideoCapture(plan['video']);cap.set(cv2.CAP_PROP_POS_MSEC,2800)
ok,frame=cap.read();cap.release();assert ok
frame=cv2.rotate(frame,cv2.ROTATE_90_COUNTERCLOCKWISE)
registration=marker_registration(frame,reference)
assert len(set(registration['detected_marker_ids']))>=2
quad=registration_quad(registration);assert quad is not None
matrix=np.linalg.inv(cv2.getPerspectiveTransform(quad,pose_world_corners(reference['board_side_mm'])))@board_to_pose_transform(reference)
pix=lambda p:transform_points(np.array(p,dtype=float),matrix)
plt.rcParams['font.family']='Yu Gothic'
fig,ax=plt.subplots(figsize=(12.8,7.8),dpi=130)
ax.imshow(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB));ax.axis('off')
points=pix([[225,45],[225,135],[180,225],[90,315]])
ax.plot(points[:2,0],points[:2,1],color='cyan',linewidth=3)
t=np.linspace(0,1,60)[:,None]
curve=(1-t)**3*np.array([225,135])+3*(1-t)**2*t*np.array([225,190])+3*(1-t)*t**2*np.array([210,195])+t**3*np.array([180,225])
curve=pix(curve);ax.plot(curve[:,0],curve[:,1],color='cyan',linewidth=2,linestyle='--')
ax.plot(points[2:,0],points[2:,1],color='cyan',linewidth=2,linestyle='--')
start=pix([[225,45]])[0];front=pix([[225,70]])[0];rear=pix([[225,20]])[0]
ax.scatter(*start,s=150,marker='+',color='cyan',linewidth=3)
ax.scatter(*rear,s=85,color='red',edgecolor='white',zorder=5)
ax.scatter(*front,s=100,marker='v',color='lime',edgecolor='black',zorder=5)
ax.annotate('',xy=front,xytext=rear,arrowprops=dict(arrowstyle='->',lw=3,color='lime'))
ax.annotate('開始中点 (225,45) mm\n右端から3マス・上端から1マス\n前＝緑を下向き',xy=start,xytext=(170,150),color='white',fontsize=11,
    bbox=dict(facecolor='black',alpha=.7,pad=6),arrowprops=dict(arrowstyle='->',color='cyan',lw=2))
ax.annotate('停止目標 (90,315) mm',xy=points[3],xytext=(220,590),color='white',fontsize=10,
    bbox=dict(facecolor='black',alpha=.7,pad=4),arrowprops=dict(arrowstyle='->',color='cyan',lw=2))
ax.scatter(*points[3],s=90,marker='x',color='cyan',linewidth=2)
ax.set_title('機体B・1600 右45度入口：両方OFFで配置\n保存写真への配置案。点線は模式軌道で実測ではありません。',fontsize=13)
fig.text(.5,.025,'穴・段差・全接地点・車幅・旋回と停止余裕は、今回の配置で確認してください。',ha='center',fontsize=10)
fig.tight_layout(rect=[0,.045,1,1]);fig.savefig(BASE/'start_placement_guide.jpg',bbox_inches='tight');plt.close(fig)
with (BASE/'guide_source_upright.json').open('x',encoding='utf-8') as f:json.dump(dict(source_plan=str(source),registered_ids=registration['detected_marker_ids'],current_placement_measured=False,display_rotation_only='90_counterclockwise'),f,indent=2)
print(BASE/'start_placement_guide.jpg')
