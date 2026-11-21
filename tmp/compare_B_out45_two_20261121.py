"""B out45の二回を同じ相対基準で比較。002の時刻照合不合格を維持する。"""
from pathlib import Path
from datetime import datetime,timezone
import hashlib,json
import numpy as np,pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_outr45_20261121_v1'
OUT=BASE/'correction02_review_v1'
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
runs=[BASE/'baseline01/local_execution/run_B_OUTR45_1600_baseline_green_20261121_001',
 BASE/'correction02_m0p9/local_execution/run_B_OUTR45_1600_correction_m0p9_green_20261121_002']
rows=[]
for index,RUN in enumerate(runs,1):
 trial=read(RUN/'trial.json');settings=read(RUN/'saved_1ms.settings.json')
 geo=read(RUN/'midpoint_raw_wheel_clock_v3/midpoint_review.json');summary=read(RUN/'full/summary.json')
 tracking=read(RUN/'tracking_1ms_review_v3/verification.json');operator=read(RUN/'operator_after_run.json')
 assert trial['camera_state']=='PASSED' and trial['execution_resend_count']==0 and summary['auto_stop_confirmed']
 assert operator['both_powered_off'] and operator['no_contact']
 assert settings['mouse']=='B' and settings['motion_name']=='out_r45' and settings['preset_speed_mm_s']==1600
 assert geo['evaluation_policy']==read(BASE/'analysis_policy.json')
 rows.append(dict(run=index,source_trial=str(RUN/'trial.json'),settings=settings,
  geometry=geo,standard_summary=summary,tracking=tracking,operator=operator,
  angle_deg=geo['exit_path_direction_change_deg'],angle_error_deg=geo['exit_path_error_from45_deg'],
  Lend_error_norm_mm=float(np.linalg.norm(geo['midpoint_Lend_residual_mm'])),
  stop_error_norm_mm=float(np.linalg.norm(geo['midpoint_stop_residual_mm'])),
  geometry_eligible=geo['independent_optical_geometry_eligible'],
  geometry_status='eligible_relative_candidate' if geo['independent_optical_geometry_eligible'] else 'reference_only_clock_failed'))
old,new=[row['settings'] for row in rows]
diff={g:{k:[old[g][k],v] for k,v in new[g].items() if old[g][k]!=v} for g in ('parameters','pid','feedforward','suction')}
assert {g:v for g,v in diff.items() if v}=={'parameters':{'correction':[.3,-.9]}}
assert rows[0]['geometry_eligible'] and not rows[1]['geometry_eligible']
assert 2<rows[1]['geometry']['clock']['half_window_zero_spread_ms']<2.2
assert read(OUT/'raw_orientation_frame_evidence.json')['raw_orientation']
result=dict(schema_version=1,created_at_utc=datetime.now(timezone.utc).isoformat(),mouse='B',motion='out_r45',preset=1600,
 executions=2,rows=rows,changed_fields=diff,visual_review_completed=True,
 visual_review='002の開始・旋回・出口・停止画像で前緑/後赤の実マーカーを確認。',
 selected_visual_evidence=str(OUT/'observed_frames_raw_orientation.jpg'),
 second_run_geometry_adopted=False,quality_thresholds_unchanged=True,
 second_run_rejection='incoming raw wheel clock half-window disagreement 2.1342616ms exceeds 2ms',
 next_decision=dict(action='repeat_same_candidate_once',correction_deg=-.9,Lstart_mm=21.8,Lend_mm=16.5,
  changed_parameter_fields=[],reason='出口の参考方向45.235度は目標付近。時刻照合を通過した同条件の確認一回を取得して判断する。',
  required_new_operator_ready=True,request_sent=False),
 parameter_adoption_allowed=False,source_params_changed=False,firmware_write_performed=False,
 limitations=['入口相対のマーカー中点。絶対車軸/機体yaw/盤面固定の合格ではない。',
  '002の45.235度・Lend位置/停止点は参考値。時刻照合基準を外れたまま精密採用しない。',
  'EKF係数・左・連続接続は未確認。'],
 input_sha256={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for RUN in runs for path in
 (RUN/'trial.json',RUN/'saved_1ms.settings.json',RUN/'midpoint_raw_wheel_clock_v3/midpoint_review.json',RUN/'tracking_1ms_review_v3/verification.json')})
with (OUT/'review.json').open('x',encoding='utf-8') as f:json.dump(result,f,ensure_ascii=False,indent=2,allow_nan=False)
plt.rcParams['font.family']='Yu Gothic';fig,axes=plt.subplots(1,2,figsize=(10,6),layout='constrained')
for row,RUN,color in zip(rows,runs,('tab:blue','tab:orange')):
 data=pd.read_csv(RUN/'midpoint_raw_wheel_clock_v3/midpoint_relative_trajectory.csv')
 start=row['geometry']['independent_clock_required_motion_quality']['video_motion_start_s']
 keep=data.time_video_s>=start-.04
 xy=data[['right_mm','forward_mm']].to_numpy();xy[~data.registered_midpoint_valid.to_numpy(bool)]=np.nan
 label=f"{row['run']:03d} 補正{row['settings']['parameters']['correction']:+.1f}° / {row['angle_deg']:.3f}°"
 if not row['geometry_eligible']:label+='（時刻条件外・参考）'
 for ax in axes:
  ax.plot(*xy[keep].T,c=color,label=label,ls='-' if row['geometry_eligible'] else '--')
  ax.scatter(*row['geometry']['midpoint_Lend_right_forward_mm'],c=color,s=35)
target=np.array(read(BASE/'analysis_policy.json')['target_Lend_right_forward_mm'])
for ax in axes:
 ax.scatter(*target,c='black',marker='x',s=70,label='Lend目標');ax.grid(alpha=.3)
 ax.set(xlabel='入口に対する右 [mm]',ylabel='入口に対する前 [mm]',aspect='equal')
axes[0].set_title('実測入口相対の中点軌跡');axes[1].set_title('Lend周辺（002は時刻条件外）')
axes[1].set_xlim(target[0]-7,target[0]+7);axes[1].set_ylim(target[1]-7,target[1]+7)
handles,labels=axes[0].get_legend_handles_labels();fig.legend(handles,labels,loc='outside lower center',fontsize=8)
fig.savefig(OUT/'comparison.png',dpi=140);plt.close(fig)
table='| 回 | 補正° | 出口° | Lend差mm | 時刻半窓差ms | 相対幾何判定 |\n|---|---:|---:|---:|---:|---|\n'
for r in rows:table+=f"| {r['run']:03d} | {r['settings']['parameters']['correction']:+.1f} | {r['angle_deg']:.3f} | {r['Lend_error_norm_mm']:.3f} | {r['geometry']['clock']['half_window_zero_spread_ms']:.3f} | {'通過' if r['geometry_eligible'] else '時刻条件外・参考'} |\n"
(OUT/'README.md').write_text('# B・1600 右out45：二回比較\n\n両方とも一回実行・再送0、撮影/停止確認通過、設定一致、保存ログ欠落0、操作者が両方OFF・接触なし。二回目の変更は角度補正+0.3→−0.9度のみ。\n\n'+table+
 '\n002は全910フレーム中908有効、必要区間の実観測98.67%、各評価フレーム実ArUco登録。入口37点/114.78mm/RMS0.389mm、出口16点/61.89mm/RMS0.187mm。ただし入口raw wheelによる前半/後半時計差2.134msが上限2msを超え、独立相対幾何判定は不合格のまま保持。出口45.235度、Lend差2.583/1.597mm（ノルム3.037mm）、停止点差6.576/4.243mmはいずれも参考値として扱う。\n\n'
 '次は−0.9度・Lstart21.8/Lend16.5mm・PID/FF同じで確認一回。時刻照合基準を緩めず、長さ変更も加えない。新しい配置・ON・手離し確認まで送信しない。基準001と002の元の判定・入力を保持。Params/実機書込み変更なし。\n\n[比較図](comparison.png)・[実画像](observed_frames_raw_orientation.jpg)・[全数値と元判定](review.json)。\n',encoding='utf-8')
for source,name in ((OUT/'review.json','comparison_two_review.json'),(OUT/'observed_frames_raw_orientation.jpg','observed_frames_raw_orientation.jpg'),
 (OUT/'raw_orientation_frame_evidence.json','visual_frame_evidence.json'),(OUT/'comparison.png','comparison_two.png')):
 with (runs[1]/name).open('xb') as f:f.write(source.read_bytes())
state_path=BASE/'resume_state.json';state=read(state_path)
state.update(state='OUT45_002_REFERENCE_ONLY_CLOCK_FAILED_OFF_CONFIRMED',latest_aggregate_review=str(OUT/'review.json'),
 latest_review=str(runs[1]/'midpoint_raw_wheel_clock_v3/midpoint_review.json'),latest_independent_relative_geometry_eligible=False,
 latest_standard_full_summary=rows[1]['standard_summary'],
 next_action='002の時刻半窓差2.134ms>2msを保持。設定を変えず−0.9度を確認一回するための003を準備。')
state_path.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
with (BASE/'README.md').open('a',encoding='utf-8') as f:
 f.write('\n## 比較002\n\n実走合計二回。002は両方OFF・接触なし、撮影と停止確認正常。参考出口45.235度。時刻照合半窓差2.134msが条件2ms以内を外れ、相対幾何の精密採用は見送り。長さ・FFなどは変えず−0.9度の同条件一回確認を準備する。[二回比較](correction02_review_v1/README.md)。\n')
print(json.dumps(dict(runs=2,second_angle_reference_deg=rows[1]['angle_deg'],geometry_eligible=False,
 clock_half_spread_ms=rows[1]['geometry']['clock']['half_window_zero_spread_ms'],next='same -0.9 degree once'),ensure_ascii=False))
