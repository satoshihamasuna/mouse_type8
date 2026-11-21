"""009と赤H160校正済み006/007/008を同じ入口相対基準で比較する。実機操作なし。"""
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
RUN=BASE/'geometry03_correction_m0p4/local_execution/run_B_INR45_1600_correction_m0p4_green_20261121_009'
OUT=BASE/'correction_m0p4_single_review_v1';OUT.mkdir(exist_ok=False)
read=lambda p:json.loads(Path(p).read_text(encoding='utf-8-sig'))
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(p,v):
 with Path(p).open('x',encoding='utf-8') as f:json.dump(v,f,ensure_ascii=False,indent=2,allow_nan=False)
previous=read(BASE/'red_hue160_validation_v1/review.json');trial=read(RUN/'trial.json')
geometry=read(RUN/'midpoint_raw_wheel_clock_v3/midpoint_review.json');tracking=read(RUN/'tracking_1ms_review_v3/verification.json')
summary=read(RUN/'full/summary.json');settings=read(RUN/'saved_1ms.settings.json');operator=read(RUN/'operator_after_run.json')
assert geometry['independent_optical_geometry_eligible'] and operator['both_powered_off'] and operator['no_contact']
assert trial['camera_state']=='PASSED' and trial['execution_resend_count']==0
assert geometry['independent_clock_required_motion_quality']['observation_rate']>=.95
detector=read(RUN/'body_geometry_used.json')['marker_detection']
for key in ('006','007','008'):
 assert detector==read(BASE/'red_hue160_validation_v1'/('run'+key)/'full/body_geometry_used.json')['marker_detection']
before=previous['rows'][0]['settings'];difference={group:{key:[before[group][key],value] for key,value in settings[group].items()
 if value!=before[group][key]} for group in ('parameters','pid','feedforward','suction')}
assert {g:d for g,d in difference.items() if d}=={'parameters':{'correction':[-1.5,-.4]}}
new_angle=geometry['exit_path_direction_change_deg'];new_point=np.array(geometry['midpoint_Lend_right_forward_mm']);goal=np.array([45.,180.])
old_angles=np.array(previous['relative_geometry']['exit_deg']);old_points=np.array(previous['relative_geometry']['Lend_right_forward_mm'])
new=dict(run_id='009',source_trial=str(RUN/'trial.json'),settings=settings,geometry=geometry,tracking=tracking,
 summary=summary,operator=operator,capture_passed=True)
result=dict(schema_version=1,reviewed_at_utc=datetime.now(timezone.utc).isoformat(),mouse='B',motion='in_r45',preset=1600,
 state='SINGLE_NEW_CORRECTION_REVIEWED_REPEATS_PENDING',before=previous['rows'],after=new,
 comparison=dict(changed_fields=difference,previous_mean_exit_deg=float(old_angles.mean()),previous_clean_capture_mean_exit_deg=float(old_angles[:2].mean()),
  new_exit_deg=new_angle,new_error_from45_deg=new_angle-45.,observed_exit_change_deg=float(new_angle-old_angles.mean()),
  commanded_clockwise_angle_change_deg=-1.1,expected_exit_by_unit_gain_deg=float(old_angles.mean()-1.1),
  new_Lend_right_forward_mm=new_point.tolist(),new_Lend_residual_mm=(new_point-goal).tolist(),new_Lend_residual_norm_mm=float(np.linalg.norm(new_point-goal)),
  previous_mean_Lend_right_forward_mm=old_points.mean(0).tolist(),previous_Lend_component_range_mm=np.ptp(old_points,axis=0).tolist(),
  control=dict(previous_tracking_rms_rad_s=[row['tracking']['tracking']['tracking_rms_rad_s'] for row in previous['rows']],
   new_tracking_rms_rad_s=tracking['tracking']['tracking_rms_rad_s'],new_target_peak_rad_s=tracking['tracking']['ideal_peak_rad_s'],
   new_actual_peak_rad_s=tracking['tracking']['ego_peak_rad_s'],new_post_negative_peak_rad_s=tracking['tracking']['post_peak_neg_rad_s'],
   saturation_pct=tracking['tracking']['saturation_pct'],tracking_tuned_in_this_trial=False)),
 qualifying_scope='入口実測方向へ合わせた相対マーカー中点。出口/turnを使わない前走raw wheel時計。',
 new_correction_same_condition_runs=1,independent_repeats_pending=True,old_failed_008_reclassified=False,
 auto_stop_original_preserved=summary.get('auto_stop_confirmed'),capture_controller_stop_evidence=trial['recording_stop'],
 physical_power_state='BOTH_OFF_USER_CONFIRMED',hardware_commands_sent_by_analysis=0,source_params_changed=False,firmware_written=False,
 ekf_status='B geometry calibration/tool support incomplete; coefficients not identified; no A values borrowed',
 limitations=['新補正は単発で、再現性と左/連続接続を確認していない。',
  '相対形状候補。絶対車軸/body yaw・盤面固定の設置/出口/停止の合格ではない。',
  '元summaryのauto_stop判定を変更していない。取得時停止証拠・後半静止動画・操作者確認は別に保持。',
  '制御PID/FFは固定。角速度のピーク/後揺れが残り、制御追従が理想化したとは扱わない。'],
 input_sha256={str(p):sha(p) for p in (BASE/'red_hue160_validation_v1/review.json',RUN/'trial.json',RUN/'saved_1ms.csv',RUN/'saved_1ms.settings.json',
 RUN/'body_geometry_used.json',RUN/'midpoint_raw_wheel_clock_v3/midpoint_review.json',RUN/'full/summary.json',RUN/'tracking_1ms_review_v3/verification.json',Path(__file__))})
save(OUT/'review.json',result)
plt.rcParams['font.family']='Yu Gothic';fig,axes=plt.subplots(1,3,figsize=(13,4.5),layout='constrained')
for key in ('006','007','008','009'):
 folder=RUN/'midpoint_raw_wheel_clock_v3' if key=='009' else BASE/'red_hue160_validation_v1'/('run'+key)/'midpoint'
 g=geometry if key=='009' else read(folder/'midpoint_review.json');data=pd.read_csv(folder/'midpoint_relative_trajectory.csv')
 q=g['independent_clock_required_motion_quality'];time=data.time_video_s.to_numpy();valid=data.registered_midpoint_valid.to_numpy(bool)
 mask=(time>=q['video_motion_start_s']-.03)&(time<=q['video_motion_end_s']+.07)
 xy=np.where(valid[:,None],data[['right_mm','forward_mm']].to_numpy(),np.nan)
 label='009 補正−0.4°' if key=='009' else key+' 補正−1.5°'+(' 区間のみ' if key=='008' else '')
 axes[0].plot(*xy[mask].T,lw=2 if key=='009' else 1,alpha=1 if key=='009' else .55,label=label)
 axes[1].scatter(*g['midpoint_Lend_right_forward_mm'],s=65 if key=='009' else 30,label=key)
axes[0].set(xlabel='入口に対する右 [mm]',ylabel='入口に対する前 [mm]',aspect='equal',title='長さ固定・角度補正のみ比較');axes[0].legend(fontsize=7)
axes[1].scatter(*goal,c='black',marker='x',label='目標45/180');axes[1].set(xlabel='右 [mm]',ylabel='前 [mm]',aspect='equal',title='Lendの相対中点');axes[1].legend(fontsize=8)
axes[2].bar(['006','007','008\n区間のみ','009'],list(old_angles)+[new_angle],color=['C0','C0','C0','C3'])
axes[2].axhline(45,c='black',ls='--',label='目標45°');axes[2].set(ylim=(min(44.5,new_angle-.3),max(46.5,new_angle+.3)),ylabel='出口方向の変化 [deg]',title='入口相対の出口角');axes[2].legend(fontsize=8)
for ax in axes:ax.grid(alpha=.25)
fig.savefig(OUT/'comparison.png',dpi=150);plt.close(fig)
text='# 機体B1600 右in45：角度補正−0.4度の単発比較\n\n'
text+=f'角度補正−1.5→−0.4度だけを変更し、一回撮影。新補正の入口相対出口角{new_angle:.3f}度、45度との差{new_angle-45:+.3f}度。旧補正の平均{old_angles.mean():.3f}度（録画合格006/007だけなら{old_angles[:2].mean():.3f}度）。前後長は7.8/34.5mmを固定。\n\n'
text+=f'Lendの高さ補正相対中点は右{new_point[0]:.3f}/前{new_point[1]:.3f}mm、目標45/180mmとの差は右{new_point[0]-45:+.3f}/前{new_point[1]-180:+.3f}mm。前走90mmを含む同一目標と既存の出口窓で比較した。\n\n'
q=geometry['independent_clock_required_motion_quality'];text+=f"今回の必要区間実観測率{q['observation_rate']*100:.2f}%、ArUco登録率{q['calibration_rate']*100:.2f}%。保存1ms {tracking['sample_count']}点/drop0、実測周期{tracking['measured_log_period_ms']:.6f}ms、最低電圧{tracking['battery_min_v']:.3f}V。全体録画PASSED、元のauto_stop={summary.get('auto_stop_confirmed')}は保持。取得時停止証拠と後半静止像・操作者の両方OFF/接触なしを別記録。\n\n"
text+=f"角速度追従RMS{tracking['tracking']['tracking_rms_rad_s']:.3f}rad/s、目標ピーク{tracking['tracking']['ideal_peak_rad_s']:.3f}に対して実測{tracking['tracking']['ego_peak_rad_s']:.3f}rad/s、後揺れ負ピーク{tracking['tracking']['post_peak_neg_rad_s']:.3f}rad/s、飽和{tracking['tracking']['saturation_pct']:.1f}%。制御値は固定で、追従が改善したとは扱わない。\n\n"
text+='同じB/検出器/高さ/撮影設定・実ArUco参照・95%基準・入口raw wheel時計・20ms補間上限を使用。旧008は必要連続区間のみの独立評価で、録画全体FAILEDを保持。新補正は一回だけで、反復・左・連続接続・EKF・ファーム反映は未実施。\n'
(OUT/'README.md').write_text(text,encoding='utf-8')
print(json.dumps(dict(output=str(OUT),new_exit_deg=new_angle,Lend=new_point.tolist(),residual=(new_point-goal).tolist(),quality=q['observation_rate'],new_rms=tracking['tracking']['tracking_rms_rad_s']),ensure_ascii=False))
