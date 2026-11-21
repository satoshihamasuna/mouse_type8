"""検出器差分が大きい実フレームと回復フレームを目視確認用に並べる。"""
from pathlib import Path
import json
import sys
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr45_20261120_v1'
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'tools/analysis')]
from video_marker_pose import marker_registration,registration_quad,pose_world_corners,iter_video_frames
from pixel_camera_relative_pose import file_hash
VALIDATION=BASE/'red_s30_validation_v1';OUT=VALIDATION/'frame_audit_v1';OUT.mkdir(exist_ok=False)
read=lambda p:json.loads(Path(p).read_text(encoding='utf-8'))
fig,axes=plt.subplots(2,3,figsize=(12,8),layout='constrained');records=[];slot=0
for key in ('010','011','012'):
 new=pd.read_csv(VALIDATION/('run'+key)/'full/apparent_floor_marker_pose.csv')
 trial_path=Path(read(VALIDATION/('run'+key)/'full/report.json')['trial_json']);run=trial_path.parent;trial=read(trial_path)
 old=pd.read_csv(run/('full')/'apparent_floor_marker_pose.csv')
 joined=new.merge(old,on='frame',suffixes=('_new','_old'))
 both=joined.marker_detected_raw_new&joined.marker_detected_raw_old
 delta=np.linalg.norm(joined[['yellow_x_mm_new','yellow_y_mm_new']].to_numpy()-joined[['yellow_x_mm_old','yellow_y_mm_old']].to_numpy(),axis=1)
 order=joined.loc[both].assign(delta=delta[both]).sort_values('delta',ascending=False)
 chosen=[(order.iloc[0],'max common-frame change')]
 if key=='010':chosen.append((order.iloc[1],'second largest change'))
 if key=='012':
  recovered=joined.loc[~joined.marker_detected_raw_old&joined.marker_detected_raw_new]
  chosen.extend([(recovered.iloc[0],'actual recovered marker'),(recovered.iloc[len(recovered)//2],'actual recovered marker')])
 reference=read(run/'marker_reference_used.json')
 requested={int(row.frame) for row,label in chosen};frames={}
 for index,frame,pts in iter_video_frames(Path(trial['video_file']),True):
  if index in requested:frames[index]=(frame,pts)
  if index>=max(requested):break
 assert set(frames)==requested
 for row,label in chosen:
  index=int(row.frame);frame,pts=frames[index];assert abs(pts-row.time_video_s_new)<.001
  reg=marker_registration(frame,reference);quad=registration_quad(reg);assert quad is not None and len(set(reg['detected_marker_ids']))>=2
  inverse=np.linalg.inv(cv2.getPerspectiveTransform(quad,pose_world_corners(reference['board_side_mm'])))
  def point(which,suffix):
   xy=np.array([[[row[which+'_x_mm_'+suffix],row[which+'_y_mm_'+suffix]]]],dtype=np.float32)
   return cv2.perspectiveTransform(xy,inverse).reshape(2)
  front=point('blue','new');rear=point('yellow','new');mid=(front+rear)/2
  x,y=np.rint(mid).astype(int);x0=max(0,x-85);y0=max(0,y-90)
  ax=axes.flat[slot];slot+=1;ax.imshow(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB));ax.set_xlim(x0,min(frame.shape[1],x0+170));ax.set_ylim(min(frame.shape[0],y0+180),y0)
  ax.scatter(*front,s=110,facecolors='none',edgecolors='lime',linewidth=1.2,label='front actual')
  ax.scatter(*rear,s=100,facecolors='none',edgecolors='cyan',linewidth=1.2,label='rear S30')
  if row.marker_detected_raw_old:
   previous=point('yellow','old');ax.scatter(*previous,s=35,facecolors='none',edgecolors='red',linewidth=1.2,label='rear original')
  ax.set_title(f'{key} frame {index} / {pts:.4f}s\n{label}',fontsize=10);ax.axis('off');ax.legend(fontsize=7,loc='upper right')
  records.append(dict(run=key,frame=index,pts_s=pts,reason=label,registered_marker_ids=reg['detected_marker_ids'],
   candidate_front_rear_px=[front.tolist(),rear.tolist()],old_observed=bool(row.marker_detected_raw_old),
   video_file=trial['video_file'],source_trial_sha256=file_hash(trial_path)))
fig.savefig(OUT/'actual_markers.png',dpi=150);plt.close(fig)
with (OUT/'frames.json').open('x',encoding='utf-8') as f:json.dump(dict(frames=records,inferred_positions_used=False,source_videos_unchanged=True),f,ensure_ascii=False,indent=2)
print(str(OUT/'actual_markers.png'))
