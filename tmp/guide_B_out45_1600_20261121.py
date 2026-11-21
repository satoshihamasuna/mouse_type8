"""保存写真の実ArUco登録に、新しいout45の使用実績位置と模式経路を示す。"""
from pathlib import Path
import hashlib
import json
import sys
import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_outr45_20261121_v1'
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'tools/analysis')]
from video_marker_pose import marker_registration,registration_quad,pose_world_corners,board_to_pose_transform,transform_points
source=ROOT/'tools/logs/pixel_camera/turn1600_B_inr45_20261120_v1/geometry05_repeat01_correction_m1p2/local_execution/camera_before_ble_B_INR45_1600_m1p2_repeat01_green_20261121_012/static_check.jpg'
frame=cv2.imread(str(source));assert frame is not None
reference=json.loads((BASE/'marker_reference_B_current.json').read_text(encoding='utf-8'))
registration=marker_registration(frame,reference);quad=registration_quad(registration)
assert quad is not None and len(set(registration['detected_marker_ids']))>=2
matrix=np.linalg.inv(cv2.getPerspectiveTransform(quad,pose_world_corners(reference['board_side_mm'])))@board_to_pose_transform(reference)
pix=lambda p:transform_points(np.array(p,dtype=float),matrix)
plt.rcParams['font.family']='Yu Gothic';fig,ax=plt.subplots(figsize=(12.8,7.6),dpi=130)
ax.imshow(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB));ax.axis('off')
points=pix([[90,45],[180,135],[225,225],[225,315]])
ax.plot(points[:2,0],points[:2,1],c='cyan',lw=3)
t=np.linspace(0,1,80)[:,None]
curve=(1-t)**3*np.array([180,135])+3*(1-t)**2*t*np.array([215,170])+3*(1-t)*t**2*np.array([225,180])+t**3*np.array([225,225])
curve=pix(curve);ax.plot(*curve.T,c='cyan',ls='--',lw=2);ax.plot(*points[2:].T,c='cyan',ls='--',lw=2)
unit=np.array([1.,1.])/np.sqrt(2);front,rear=pix([np.array([90.,45.])+25*unit,np.array([90.,45.])-25*unit])
ax.scatter(*points[0],marker='+',s=160,c='cyan',lw=3);ax.scatter(*front,s=100,c='lime',edgecolor='black',zorder=5)
ax.scatter(*rear,s=90,c='red',edgecolor='white',zorder=5);ax.annotate('',xy=front,xytext=rear,arrowprops=dict(arrowstyle='->',lw=3,color='lime'))
box=dict(facecolor='black',alpha=.75,pad=5)
ax.annotate('開始中点 (90,45) mm\n格子左上から右2マス・下1マス\n前＝緑を右下へ（45°）',xy=points[0],xytext=(25,285),color='white',fontsize=11,bbox=box,arrowprops=dict(arrowstyle='->',color='cyan',lw=2))
ax.scatter(*points[2],marker='x',s=80,c='yellow',lw=2);ax.annotate('Lend目標 (225,225) mm',xy=points[2],xytext=(890,360),color='white',fontsize=10,bbox=box,arrowprops=dict(arrowstyle='->',color='yellow',lw=2))
ax.scatter(*points[3],marker='x',s=90,c='cyan',lw=2);ax.annotate('停止目標 (225,315) mm',xy=points[3],xytext=(890,540),color='white',fontsize=10,bbox=box,arrowprops=dict(arrowstyle='->',color='cyan',lw=2))
ax.set_title('機体B・1600 右45度出口：両方OFFで概略配置\n保存写真上の配置案（写真内の機体は前回位置）。点線は模式経路。',fontsize=13)
fig.text(.5,.025,'今回の穴・全接地点・車幅・旋回と停止余裕を確認してください。',ha='center',fontsize=10)
fig.tight_layout(rect=[0,.045,1,1]);destination=BASE/'start_placement_guide.jpg';assert not destination.exists();fig.savefig(destination,bbox_inches='tight');plt.close(fig)
with (BASE/'guide_source.json').open('x',encoding='utf-8') as f:json.dump(dict(source_image=str(source),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
 registered_ids=registration['detected_marker_ids'],current_placement_measured=False,nominal_path_not_a_footprint_verification=True,
 planned_start_mm=[90,45],front='green',front_heading_board_deg=45),f,ensure_ascii=False,indent=2)
print(str(destination))
