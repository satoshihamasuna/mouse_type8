"""最新OFF画像に、180度用の開始点と移動方向を重ねる。機器操作なし。"""
from pathlib import Path
import json
import sys
import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_r180_20261120_v1'
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'tools/analysis')]
from video_marker_pose import (marker_registration,registration_quad,pose_world_corners,
    board_to_pose_transform,transform_points)
read=lambda p:json.loads(Path(p).read_text(encoding='utf-8-sig'))
folder=BASE/'off_static03_s35'
frame=cv2.imread(str(folder/'static_frame.jpg'))
plan=read(folder/'plan.json');checked=read(folder/'static_check.json')
reference=read(plan['reference'])
registered=marker_registration(frame,reference);quad=registration_quad(registered)
assert quad is not None and len(registered['detected_marker_ids'])>=2
matrix=cv2.getPerspectiveTransform(pose_world_corners(reference['board_side_mm']),quad)@board_to_pose_transform(reference)
points=transform_points(np.array([checked['center_board_mm'],[315.,135.],[315.,110.],[315.,160.]]),matrix)
target=folder/'move_to_r180_start_guide_v2.jpg';assert not target.exists()
plt.rcParams['font.family']='Yu Gothic'
fig,ax=plt.subplots(figsize=(12.8,7.2),layout='constrained')
ax.imshow(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB))
ax.scatter(*points[0],c='orange',marker='x',s=100,label='今回の実測中点 (316,41) mm')
ax.scatter(*points[1],c='cyan',marker='+',s=180,label='180度の開始中点 (315,135) mm')
ax.annotate('',xy=points[1],xytext=points[0],arrowprops=dict(arrowstyle='->',color='cyan',lw=2.5))
ax.plot(points[[2,3],0],points[[2,3],1],color='cyan',lw=2)
ax.scatter(*points[3],c='lime',edgecolors='black',marker='v',s=90,label='前：緑を下へ')
ax.scatter(*points[2],c='red',edgecolors='white',s=60,label='後：赤')
ax.text(.25,.3,'両方OFFのまま下へ2マス\n右端から1マス・上端から3マス\n前後マーカー中点を合わせる',
    transform=ax.transAxes,va='top',color='white',fontsize=12,
    bbox=dict(facecolor='black',alpha=.7,edgecolor='none'))
ax.set_title('機体B・1600 右180度：最新OFF画像で開始位置を確認')
ax.legend(loc='lower left',fontsize=9);ax.axis('off')
fig.savefig(target,dpi=120);plt.close(fig)
print(target)
