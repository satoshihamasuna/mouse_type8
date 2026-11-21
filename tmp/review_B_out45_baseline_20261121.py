"""基準001の画像照合資料と、角度だけを次に比較する根拠を保存する。機器操作なし。"""
from pathlib import Path
from datetime import datetime,timezone
import hashlib,json,sys
import cv2,numpy as np,pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_outr45_20261121_v1'
RUN=BASE/'baseline01/local_execution/run_B_OUTR45_1600_baseline_green_20261121_001'
OUT=BASE/'baseline01_review_v1';OUT.mkdir(exist_ok=False)
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
def save(p,v):
 with p.open('x',encoding='utf-8') as f:json.dump(v,f,ensure_ascii=False,indent=2,allow_nan=False)
trial=read(RUN/'trial.json');geo=read(RUN/'midpoint_raw_wheel_clock_v3/midpoint_review.json')
track=read(RUN/'tracking_1ms_review_v3/verification.json');summary=read(RUN/'full/summary.json')
assert geo['independent_optical_geometry_eligible'] and summary['auto_stop_confirmed']
assert read(RUN/'operator_after_run.json')['both_powered_off']
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'tools/analysis')]
from video_marker_pose import marker_registration,registration_quad
reference=read(RUN/'marker_reference_used.json')
pose=pd.read_csv(RUN/'full/apparent_floor_marker_pose.csv')
valid=pose.marker_detected_raw & pose.calibration_valid
rows=[pose[valid].iloc[np.argmin(abs(pose[valid].time_video_s-t))] for t in (9.15,9.48,9.56,11.20)]
cap=cv2.VideoCapture(trial['video_file']);assert cap.isOpened()
plt.rcParams['font.family']='Yu Gothic';fig,axes=plt.subplots(2,4,figsize=(14,6),layout='constrained')
evidence=[]
for col,row in enumerate(rows):
 cap.set(cv2.CAP_PROP_POS_FRAMES,int(row.frame));ok,frame=cap.read();assert ok
 registration=marker_registration(frame,reference)
 assert registration_quad(registration) is not None and len(set(registration['detected_marker_ids']))>=2
 x,y=map(int,(row.midpoint_image_x_px,row.midpoint_image_y_px))
 axes[0,col].imshow(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB));axes[0,col].axis('off')
 axes[0,col].set_title(f'frame {int(row.frame)} / PTS {row.time_video_s:.4f}s',fontsize=10)
 crop=frame[max(0,y-65):min(frame.shape[0],y+65),max(0,x-65):min(frame.shape[1],x+65)]
 axes[1,col].imshow(cv2.cvtColor(crop,cv2.COLOR_BGR2RGB));axes[1,col].axis('off')
 evidence.append(dict(frame=int(row.frame),decoded_pts_s=float(row.time_video_s),
  registered_ids=registration['detected_marker_ids'],observed_colour_pair=True,inferred_points=False))
cap.release();fig.suptitle('B1600 右out45 基準001：実画像の前緑・後赤を確認（下段は拡大）')
fig.savefig(OUT/'observed_frames.jpg',dpi=120);plt.close(fig)
error=geo['exit_path_direction_change_deg']-45
result=dict(saved_at_utc=datetime.now(timezone.utc).isoformat(),mouse='B',motion='out_r45',preset=1600,
 executions=1,source_trial=str(RUN/'trial.json'),all_settings_match=True,operator_power='BOTH_OFF_USER_CONFIRMED',no_contact=True,
 camera_state=trial['camera_state'],auto_stop_confirmed=summary['auto_stop_confirmed'],
 necessary_interval_quality=geo['independent_clock_required_motion_quality'],entry_line=geo['entry_line'],exit_line=geo['exit_line'],
 angle_deg=geo['exit_path_direction_change_deg'],angle_error_deg=error,
 Lend_midpoint_mm=geo['midpoint_Lend_right_forward_mm'],Lend_residual_mm=geo['midpoint_Lend_residual_mm'],
 Lend_error_norm_mm=float(np.linalg.norm(geo['midpoint_Lend_residual_mm'])),
 stop_residual_mm=geo['midpoint_stop_residual_mm'],stop_error_norm_mm=float(np.linalg.norm(geo['midpoint_stop_residual_mm'])),
 battery_min_v=track['battery_min_v'],tracking_rms_rad_s=track['tracking']['tracking_rms_rad_s'],
 timing_sensitivity=geo['timing_sensitivity'],clock=geo['clock'],late_stationary=geo['late_stationary'],
 next_candidate=dict(changed_fields=['parameters.correction'],old_correction_deg=.3,new_correction_deg=-.9,
  selection='0.3 + (43.8091623 - 45) = -0.8908377 -> -0.9 degrees',
  reasoning='出口が1.191度浅い。長さはLend差1.165mmで、時刻1ms差だけでもLend解が約1.8mm変わる。まず角度一項目を比較する。',
  Lstart_mm=21.8,Lend_mm=16.5,FF_accel_decel=.000750,om_Ki=.01,
  response_slope_is_assumption=True,proportional_response_verified=False,
  next_run_is_required_for_validation=True),
 visual_frames=evidence,visual_review_completed=False,
 limitations=['実測入口相対の高さ補正マーカー中点。車軸・機体yaw・盤面固定の合格ではない。',
 '出口は事前固定のLend+10ms〜動作終了-10ms、減速を含む16点の進行方向。',
 '角速度のピーク/振れは残る。現時点でPIDやFFの最適化完了とはしない。',
 'BのEKF係数は今回同定できていない。左・反復・連続接続は未確認。'],
 source_params_changed=False,firmware_write_performed=False,
 source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in
 (RUN/'trial.json',RUN/'saved_1ms.csv',RUN/'saved_1ms.settings.json',RUN/'full/report.json',
 RUN/'midpoint_raw_wheel_clock_v3/midpoint_review.json',RUN/'tracking_1ms_review_v3/verification.json',Path(__file__))})
save(OUT/'review.json',result)
print(json.dumps({k:result[k] for k in ('angle_deg','angle_error_deg','Lend_error_norm_mm','stop_error_norm_mm','next_candidate')},ensure_ascii=False))
