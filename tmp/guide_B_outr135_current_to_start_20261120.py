"""今回静止画像へ入口現在位置→出口開始点を表示する。機器操作なし。"""
from pathlib import Path
import json
import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import sys
ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_outr135_20261120_v1'
FOLDER=BASE/'baseline01/local_execution/camera_before_ble_B_OUTR135_1600_baseline_green_20261120_001'
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'tools/analysis')]
from video_marker_pose import marker_registration,registration_quad,pose_world_corners,board_to_pose_transform,transform_points
frame=cv2.imread(str(FOLDER/'static_check.jpg'));reference=read(BASE/'marker_reference_B_current.json')
registration=marker_registration(frame,reference);quad=registration_quad(registration)
assert quad is not None and len(set(registration['detected_marker_ids']))>=2
matrix=cv2.getPerspectiveTransform(pose_world_corners(reference['board_side_mm']),quad)@board_to_pose_transform(reference)
current=np.array(read(FOLDER/'static_check.json')['center_board_mm']);target=np.array([180.,45.]);v=np.array([1.,1.])/np.sqrt(2)
p=transform_points(np.array([current,target,target-v*25,target+v*25]),matrix)
plt.rcParams['font.family']='Yu Gothic'
fig,ax=plt.subplots(figsize=(12.8,7.2),layout='constrained');ax.imshow(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB))
ax.annotate('',xy=p[1],xytext=p[0],arrowprops=dict(arrowstyle='->',color='cyan',lw=3))
ax.scatter(*p[0],s=650,facecolors='none',edgecolors='orange',lw=2,label='今回画像の現在位置（入口用）')
ax.scatter(*p[1],s=220,marker='+',c='cyan',lw=2)
ax.plot(p[[2,3],0],p[[2,3],1],c='cyan',lw=2)
ax.scatter(*p[3],c='lime',s=100,edgecolors='black',label='移動先の前：緑（右下）')
ax.scatter(*p[2],c='red',s=90,edgecolors='white',label='移動先の後：赤')
ax.text(340,260,'出口の開始中点 (180,45) mm\n今の位置から左へ3マス・上へ2マス\n緑前を右下45度へ',color='white',fontsize=12,
    bbox=dict(facecolor='black',alpha=.8,edgecolor='none'))
ax.set_title('両方OFFで持ち上げて移動：右135度出口の配置\n今回の静止画像への案内／走行指令は未送信')
ax.legend(loc='lower left');ax.axis('off');fig.savefig(BASE/'current_to_outR135_start_guide.jpg',dpi=130);plt.close(fig)
print(str(BASE/'current_to_outR135_start_guide.jpg'))
