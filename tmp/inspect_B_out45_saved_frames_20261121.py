"""指定された保存済みB out45の代表実画像を、解析と同じraw向きで確認用に出力。"""
from pathlib import Path
import hashlib,json,sys
import cv2,numpy as np,pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_outr45_20261121_v1'
RUN=(BASE/sys.argv[1]).resolve();OUT=(BASE/sys.argv[2]).resolve()
assert RUN.is_relative_to(BASE) and OUT.is_relative_to(BASE)
OUT.mkdir(exist_ok=True)
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'tools/analysis')]
from video_marker_pose import marker_registration,registration_quad
trial=read(RUN/'trial.json');reference=read(RUN/'marker_reference_used.json')
geo=read(RUN/'midpoint_raw_wheel_clock_v3/midpoint_review.json')
assert trial['mouse']=='B' and trial['motion']=='out_r45'
pose=pd.read_csv(RUN/'full/apparent_floor_marker_pose.csv');valid=pose.marker_detected_raw & pose.calibration_valid
stamps=[geo['independent_clock_required_motion_quality']['video_motion_start_s']-.08,
 geo['Lend_video_pts_s']-.035,float(np.mean(geo['exit_window_pts_s'])),float(np.mean(geo['late_stationary']['window_pts_s']))]
rows=[pose[valid].iloc[np.argmin(abs(pose[valid].time_video_s-t))] for t in stamps]
cap=cv2.VideoCapture(trial['video_file']);assert cap.isOpened();cap.set(cv2.CAP_PROP_ORIENTATION_AUTO,0)
plt.rcParams['font.family']='Yu Gothic';fig,axes=plt.subplots(2,4,figsize=(14,6),layout='constrained');evidence=[]
for col,row in enumerate(rows):
 cap.set(cv2.CAP_PROP_POS_FRAMES,int(row.frame));ok,frame=cap.read();assert ok and frame.shape[:2]==(720,1280)
 registration=marker_registration(frame,reference)
 assert registration_quad(registration) is not None and len(set(registration['detected_marker_ids']))>=2
 x,y=map(int,(row.midpoint_image_x_px,row.midpoint_image_y_px))
 axes[0,col].imshow(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB));axes[0,col].axis('off')
 axes[0,col].set_title(f'frame {int(row.frame)} / PTS {row.time_video_s:.4f}s',fontsize=10)
 crop=frame[max(0,y-65):min(frame.shape[0],y+65),max(0,x-65):min(frame.shape[1],x+65)]
 axes[1,col].imshow(cv2.cvtColor(crop,cv2.COLOR_BGR2RGB));axes[1,col].axis('off')
 evidence.append(dict(frame=int(row.frame),decoded_pts_s=float(row.time_video_s),registered_ids=registration['detected_marker_ids'],
  observed_colour_pair=True,inferred_points=False))
cap.release();correction=read(RUN/'saved_1ms.settings.json')['parameters']['correction']
fig.suptitle(f'B1600 右out45 補正{correction:+.1f}°：前緑・後赤の実画像（解析と同じraw向き）')
destination=OUT/'observed_frames_raw_orientation.jpg';assert not destination.exists();fig.savefig(destination,dpi=120);plt.close(fig)
with (OUT/'raw_orientation_frame_evidence.json').open('x',encoding='utf-8') as f:
 json.dump(dict(raw_orientation=True,frame_size=[1280,720],source_trial=str(RUN/'trial.json'),frames=evidence,
  source_cache_sha256=hashlib.sha256((RUN/'full/apparent_floor_marker_pose.csv').read_bytes()).hexdigest()),f,ensure_ascii=False,indent=2)
print(str(destination))
