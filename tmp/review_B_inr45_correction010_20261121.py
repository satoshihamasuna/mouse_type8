"""補正3条件を同じ実検出・入口相対測定で比較する。010は単発。"""
from pathlib import Path
from datetime import datetime,timezone
import hashlib
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr45_20261120_v1'
OUT=BASE/'correction_m0p9_single_review_v1';OUT.mkdir(exist_ok=False)
read=lambda p:json.loads(Path(p).read_text(encoding='utf-8-sig'))
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(p,v):
 with Path(p).open('x',encoding='utf-8') as f:json.dump(v,f,ensure_ascii=False,indent=2,allow_nan=False)
rows=[];paths=[]
for key in ('006','007','008','009','010'):
 if key in ('006','007','008'):
  geometry_folder=BASE/'red_hue160_validation_v1'/('run'+key)/'midpoint'
  geometry=read(geometry_folder/'midpoint_review.json');run=Path(geometry['source_trial']).parent
  full=geometry_folder.parent/'full'
 elif key=='009':
  run=BASE/'geometry03_correction_m0p4/local_execution/run_B_INR45_1600_correction_m0p4_green_20261121_009'
  geometry_folder=run/'midpoint_raw_wheel_clock_v3';geometry=read(geometry_folder/'midpoint_review.json');full=run/'full'
 else:
  run=BASE/'geometry04_correction_m0p9/local_execution/run_B_INR45_1600_correction_m0p9_green_20261121_010'
  geometry_folder=run/'midpoint_raw_wheel_clock_v3';geometry=read(geometry_folder/'midpoint_review.json');full=run/'full'
 trial=read(run/'trial.json');settings=read(run/'saved_1ms.settings.json');tracking=read(run/'tracking_1ms_review_v3/verification.json');operator=read(run/'operator_after_run.json')
 assert geometry['independent_optical_geometry_eligible'] and operator['both_powered_off'] and operator['no_contact']
 assert settings['mouse']=='B' and settings['motion_name']=='in_r45' and settings['preset_speed_mm_s']==1600
 assert geometry['independent_clock_required_motion_quality']['observation_rate']>=.95
 assert geometry['independent_clock_required_motion_quality']['calibration_rate']==1
 assert trial['execution_resend_count']==0
 if key=='008':assert trial['camera_state']=='FAILED'
 else:assert trial['camera_state']=='PASSED'
 rows.append(dict(run_id=key,source_trial=str(run/'trial.json'),settings=settings,geometry=geometry,tracking=tracking,operator=operator,
  capture_passed=trial['camera_state']=='PASSED',full_summary=read(full/'summary.json'),geometry_folder=str(geometry_folder)))
 paths.extend([run/'trial.json',run/'saved_1ms.settings.json',run/'tracking_1ms_review_v3/verification.json',geometry_folder/'midpoint_review.json'])
base=rows[0]['settings'];current=rows[-1]
for row in rows:
 settings=row['settings'];assert all(settings[group]==base[group] for group in ('pid','feedforward','suction','turn_count'))
 assert all(settings['parameters'][k]==base['parameters'][k] for k in base['parameters'] if k!='correction')
current_angle=current['geometry']['exit_path_direction_change_deg'];point=np.array(current['geometry']['midpoint_Lend_right_forward_mm']);goal=np.array([45.,180.])
old_mean=float(np.mean([row['geometry']['exit_path_direction_change_deg'] for row in rows[:3]]))
basis=read(BASE/'geometry04_correction_m0p9/angle_basis.json')
result=dict(schema_version=1,reviewed_at_utc=datetime.now(timezone.utc).isoformat(),mouse='B',motion='in_r45',preset=1600,
 state='M0P9_SINGLE_REVIEWED_REPRODUCIBILITY_PENDING',rows=rows,
 comparison=dict(old_m1p5_mean_exit_deg=old_mean,m0p4_exit_deg=rows[-2]['geometry']['exit_path_direction_change_deg'],
  m0p9_exit_deg=current_angle,m0p9_error_from45_deg=current_angle-45.,projected_exit_from_bracket_deg=basis['projected_exit_deg'],
  m0p9_point_right_forward_mm=point.tolist(),m0p9_point_residual_mm=(point-goal).tolist(),
  m0p9_point_residual_norm_mm=float(np.linalg.norm(point-goal)),same_m0p9_runs=1),
 observed_marker_geometry_only=True,absolute_axle_and_body_yaw_available=False,old_failed_008_reclassified=False,
 source_params_changed=False,firmware_written=False,ekf_identified=False,hardware_commands_sent=0,
 current_physical_power_state='BOTH_OFF_USER_CONFIRMED',independent_repeats_pending=True,
 limitations=['010は単発。目標に近くても再現性・左・連続接続の合格ではない。',
  '各フレームの実ArUcoと前後マーカーを使用。入口raw wheelだけで時計を照合。独立ハードウェア同期ではない。',
  'マーカー中点の入口相対形状。盤面固定の設置/出口/停止、絶対車軸/body yawを保証しない。',
  '角速度のピーク/後揺れを記録し、制御が理想追従したとは扱わない。',
  '元auto_stop判定を保持。取得時停止証拠、後半静止動画、操作者OFF/接触なしを区別。'],
 input_sha256={str(p):sha(p) for p in paths+[BASE/'geometry04_correction_m0p9/angle_basis.json',Path(__file__)]})
save(OUT/'review.json',result)
summary=[]
for row in rows:
 g=row['geometry'];t=row['tracking'];q=g['independent_clock_required_motion_quality'];p=g['midpoint_Lend_right_forward_mm']
 summary.append(dict(run_id=row['run_id'],correction_deg=row['settings']['parameters']['correction'],capture_passed=row['capture_passed'],
  observation_rate=q['observation_rate'],calibration_rate=q['calibration_rate'],exit_deg=g['exit_path_direction_change_deg'],
  right_mm=p[0],forward_mm=p[1],right_error_mm=p[0]-45,forward_error_mm=p[1]-180,
  om_rms_rad_s=t['tracking']['tracking_rms_rad_s'],om_peak_rad_s=t['tracking']['ego_peak_rad_s'],
  post_negative_peak_rad_s=t['tracking']['post_peak_neg_rad_s'],min_battery_v=t['battery_min_v']))
pd.DataFrame(summary).to_csv(OUT/'comparison.csv',index=False)
plt.rcParams['font.family']='Yu Gothic';fig,axes=plt.subplots(1,3,figsize=(13,4.5),layout='constrained')
for row in rows:
 key=row['run_id'];g=row['geometry'];data=pd.read_csv(Path(row['geometry_folder'])/'midpoint_relative_trajectory.csv')
 q=g['independent_clock_required_motion_quality'];time=data.time_video_s.to_numpy();valid=data.registered_midpoint_valid.to_numpy(bool)
 mask=(time>=q['video_motion_start_s']-.03)&(time<=q['video_motion_end_s']+.07)
 xy=np.where(valid[:,None],data[['right_mm','forward_mm']].to_numpy(),np.nan)
 label=key+f" 補正{row['settings']['parameters']['correction']:+.1f}°"+(' 区間のみ' if key=='008' else '')
 axes[0].plot(*xy[mask].T,lw=2 if key=='010' else 1,alpha=1 if key=='010' else .45,label=label)
 axes[1].scatter(*g['midpoint_Lend_right_forward_mm'],s=70 if key=='010' else 30,label=key)
 axes[2].scatter(row['settings']['parameters']['correction'],g['exit_path_direction_change_deg'],s=80 if key=='010' else 30,label=key)
axes[0].set(xlabel='入口に対する右 [mm]',ylabel='入口に対する前 [mm]',aspect='equal',title='長さ固定・角度補正3条件');axes[0].legend(fontsize=7)
axes[1].scatter(*goal,c='black',marker='x',label='目標45/180');axes[1].set(xlabel='右 [mm]',ylabel='前 [mm]',aspect='equal',title='Lendの相対中点');axes[1].legend(fontsize=8)
axes[2].plot([-1.5,-.4],[old_mean,rows[-2]['geometry']['exit_path_direction_change_deg']],c='gray',ls=':',label='010の予測に使った内挿')
axes[2].axhline(45,c='black',ls='--',label='目標45°');axes[2].set(xlabel='角度補正 [deg]',ylabel='出口方向の変化 [deg]',title='010は未使用の点で実測');axes[2].legend(fontsize=7)
for ax in axes:ax.grid(alpha=.25)
fig.savefig(OUT/'comparison.png',dpi=150);plt.close(fig)
g=current['geometry'];t=current['tracking'];q=g['independent_clock_required_motion_quality']
text='# 機体B1600 右in45：角度補正−0.9度の単発評価\n\n'
text+=f'010を一回取得。出口角{current_angle:.3f}度（45度との差{current_angle-45:+.3f}度）、Lend右{point[0]:.3f}/前{point[1]:.3f}mm（目標45/180との差{point[0]-45:+.3f}/{point[1]-180:+.3f}mm）。\n\n'
text+=f'前候補は−1.5度で平均{old_mean:.3f}度、−0.4度で{rows[-2]["geometry"]["exit_path_direction_change_deg"]:.3f}度。内挿した−0.9度の予測出口は{basis["projected_exit_deg"]:.3f}度で、010を予測に使用していない。全条件でLstart7.8/Lend34.5mm・半径−55mm・速度1.6m/s・PID/FF・吸引650を固定。\n\n'
text+=f"今回の必要区間実観測率{q['observation_rate']*100:.2f}%・ArUco登録100%。保存1ms {t['sample_count']}点/drop0、最低電圧{t['battery_min_v']:.3f}V。角速度RMS{t['tracking']['tracking_rms_rad_s']:.3f}rad/s、ピーク{t['tracking']['ego_peak_rad_s']:.3f}rad/s（目標{t['tracking']['ideal_peak_rad_s']:.3f}）、後揺れ負ピーク{t['tracking']['post_peak_neg_rad_s']:.3f}rad/s。\n\n"
text+='各フレームの実ArUco・実色マーカー、95%基準、20ms補間上限、入口raw wheel時計、同じ出口窓を維持。008全体FAILEDを保持し、必要連続区間のみの独立評価と明示した。\n\n'
text+='010は単発で再現性は未確認。相対マーカー中点の形状候補であり、盤面固定の出口/停止・絶対車軸/body yaw・左・連続接続・BのEKFは未確認。元summaryのauto_stop判定と取得時停止証拠・後半静止像を保持し、操作者の両方OFF/接触なしを記録。Params/ファームは変更していない。\n'
(OUT/'README.md').write_text(text,encoding='utf-8')
print(json.dumps(dict(output=str(OUT),exit_deg=current_angle,Lend=point.tolist(),residual=(point-goal).tolist(),quality=q['observation_rate'],same_candidate_runs=1),ensure_ascii=False))
