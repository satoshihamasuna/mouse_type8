"""三回でB out45の実走済み暫定値を選択。002の時刻不合格と未確認範囲を保持。"""
from pathlib import Path
from datetime import datetime,timezone
import hashlib,json
import numpy as np,pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_outr45_20261121_v1'
OUT=BASE/'selected_m0p9_three_review_v1'
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
def save(p,v):
 with p.open('x',encoding='utf-8') as f:json.dump(v,f,ensure_ascii=False,indent=2,allow_nan=False)
names=[('001','baseline01','B_OUTR45_1600_baseline_green_20261121_001'),
 ('002','correction02_m0p9','B_OUTR45_1600_correction_m0p9_green_20261121_002'),
 ('003','correction02_repeat01_m0p9','B_OUTR45_1600_m0p9_repeat01_green_20261121_003')]
rows=[];paths=[]
for run_id,stage,identifier in names:
 RUN=BASE/stage/'local_execution'/('run_'+identifier)
 trial=read(RUN/'trial.json');settings=read(RUN/'saved_1ms.settings.json')
 geo=read(RUN/'midpoint_raw_wheel_clock_v3/midpoint_review.json');summary=read(RUN/'full/summary.json')
 tracking=read(RUN/'tracking_1ms_review_v3/verification.json');operator=read(RUN/'operator_after_run.json')
 events=[json.loads(line) for line in (RUN/'events.jsonl').read_text(encoding='utf-8').splitlines()]
 assert len([e for e in events if e['event']=='execution_write_attempt'])==1
 assert trial['camera_state']=='PASSED' and trial['execution_resend_count']==0 and summary['auto_stop_confirmed']
 assert settings['mouse']=='B' and settings['motion_name']=='out_r45' and settings['preset_speed_mm_s']==1600
 assert operator['both_powered_off'] and operator['no_contact'] and trial['saved_log']['drop_count']==0
 assert geo['evaluation_policy']==read(BASE/'analysis_policy.json')
 paths += [RUN/name for name in ('trial.json','saved_1ms.csv','saved_1ms.settings.json','telemetry.csv','operator_after_run.json',
  'full/report.json','full/summary.json','midpoint_raw_wheel_clock_v3/midpoint_review.json','tracking_1ms_review_v3/verification.json')]
 rows.append(dict(run_id=run_id,source_trial=str(RUN/'trial.json'),settings=settings,
  geometry=geo,standard_summary=summary,tracking=tracking,operator=operator,
  angle_deg=geo['exit_path_direction_change_deg'],angle_error_deg=geo['exit_path_error_from45_deg'],
  Lend_error_norm_mm=float(np.linalg.norm(geo['midpoint_Lend_residual_mm'])),
  stop_error_norm_mm=float(np.linalg.norm(geo['midpoint_stop_residual_mm'])),
  relative_geometry_eligible=geo['independent_optical_geometry_eligible'],
  evaluation_status='eligible_relative_review' if geo['independent_optical_geometry_eligible'] else 'clock_failed_reference_only'))
assert [r['relative_geometry_eligible'] for r in rows]==[True,False,True]
for key in ('parameters','pid','feedforward','suction','turn_count'):
 assert rows[1]['settings'][key]==rows[2]['settings'][key],key
final=rows[2];geo=final['geometry'];assert geo['clock']['passed']
assert geo['independent_clock_required_motion_quality']['observation_rate']==1.
assert geo['independent_clock_required_motion_quality']['calibration_rate']==1.
assert abs(final['angle_error_deg'])<1. and final['Lend_error_norm_mm']<3.
selected=read(BASE/'correction02_repeat01_m0p9/candidate.settings.json')
selected['source']='B1600右out45、三回で実走済み暫定値を選択。003の相対幾何/時刻照合通過。002は参考のみ。左/連続/厳密停止点は未確認。'
save(OUT/'selected.settings.json',selected)
decision=dict(schema_version=1,decided_at_utc=datetime.now(timezone.utc).isoformat(),mouse='B',motion='out_r45',preset=1600,
 state='PROVISIONAL_RIGHT_SINGLE_TURN_SELECTED',executions_total=3,execution_resends=0,
 selected_settings=str(OUT/'selected.settings.json'),selected_from_run='003',rows=rows,
 validated_geometry_run_ids=['001','003'],validated_geometry_runs_at_selected_setting=['003'],
 reference_only_run_ids=['002'],validated_repetition_count_at_selected_setting=1,repeatability_established=False,
 selected_parameters=selected['parameters'],selected_pid=selected['pid'],selected_feedforward=selected['feedforward'],selected_suction=selected['suction'],
 final_relative_exit_deg=final['angle_deg'],final_relative_exit_error_deg=final['angle_error_deg'],
 final_Lend_midpoint_mm=geo['midpoint_Lend_right_forward_mm'],final_Lend_residual_mm=geo['midpoint_Lend_residual_mm'],
 final_Lend_error_norm_mm=final['Lend_error_norm_mm'],final_stop_residual_mm=geo['midpoint_stop_residual_mm'],
 final_stop_error_norm_mm=final['stop_error_norm_mm'],
 reason='003の品質/時計照合を通過し、出口誤差+0.828度・Lend差2.767mm。実走済みの−0.9度/21.8/16.5mmを単体暫定値に選択。未走行の長さ解は採用しない。',
 selected_same_setting_angular_observations=[dict(run_id=r['run_id'],angle_deg=r['angle_deg'],relative_geometry_eligible=r['relative_geometry_eligible']) for r in rows[1:]],
 untested_length_solution_not_selected=geo['length_only_candidate_at_current_angle'],
 quality_thresholds_unchanged=True,missing_points_inferred_as_observed=False,visual_review_completed=True,
 visual_evidence=str(OUT/'observed_frames_raw_orientation.jpg'),
 precise_absolute_geometry_adoption_allowed=False,source_params_changed_in_this_out45_tuning=False,
 current_source_correction_deg=.3,selected_correction_applied_to_firmware_source=False,firmware_write_performed=False,
 previous_shared_FF_source_adoption_retained=True,physical_power_state='BOTH_OFF_USER_CONFIRMED',
 next_execution_scheduled=False,next_request_id=None,
 remaining=['減速/制動後停止中点差約6.9mm。Lend位置と区別。盤面固定・車軸の合格ではない。',
  '同条件002は時刻照合不合格を保持。採用設定で厳密な有効反復二本を確認したとはしない。',
  '左ターン・連続接続は未走行。BのEKF係数は未同定。',
  '今回の角度補正−0.9度はRAM試験・採用資料のみ。ファームのソース/実機へ未反映。'],
 input_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths+[BASE/'analysis_policy.json',Path(__file__)]})
save(OUT/'decision.json',decision)
plt.rcParams['font.family']='Yu Gothic';fig,axes=plt.subplots(1,2,figsize=(10,6),layout='constrained')
for row,(_,stage,identifier),color in zip(rows,names,('tab:blue','tab:orange','tab:green')):
 RUN=BASE/stage/'local_execution'/('run_'+identifier)
 data=pd.read_csv(RUN/'midpoint_raw_wheel_clock_v3/midpoint_relative_trajectory.csv')
 keep=data.time_video_s>=row['geometry']['independent_clock_required_motion_quality']['video_motion_start_s']-.04
 xy=data[['right_mm','forward_mm']].to_numpy();xy[~data.registered_midpoint_valid.to_numpy(bool)]=np.nan
 label=f"{row['run_id']} 補正{row['settings']['parameters']['correction']:+.1f}° / {row['angle_deg']:.3f}°"
 if not row['relative_geometry_eligible']:label+='（時刻条件外・参考）'
 for ax in axes:
  ax.plot(*xy[keep].T,c=color,label=label,ls='-' if row['relative_geometry_eligible'] else '--')
  ax.scatter(*row['geometry']['midpoint_Lend_right_forward_mm'],c=color,s=35)
target=np.array(geo['target_midpoint_Lend_right_forward_mm'])
for ax in axes:
 ax.scatter(*target,c='black',marker='x',s=70,label='Lend目標');ax.grid(alpha=.3)
 ax.set(xlabel='入口に対する右 [mm]',ylabel='入口に対する前 [mm]',aspect='equal')
axes[0].set_title('実測入口相対のマーカー中点');axes[1].set_title('Lend周辺（002は参考）')
axes[1].set_xlim(target[0]-7,target[0]+7);axes[1].set_ylim(target[1]-7,target[1]+7)
handles,labels=axes[0].get_legend_handles_labels();fig.legend(handles,labels,loc='outside lower center',fontsize=8)
fig.savefig(OUT/'comparison.png',dpi=140);plt.close(fig)
table='| 回 | 補正° | 出口° | Lend差mm | 時刻半窓差ms | 相対幾何 |\n|---|---:|---:|---:|---:|---|\n'
for r in rows:table+=f"| {r['run_id']} | {r['settings']['parameters']['correction']:+.1f} | {r['angle_deg']:.3f} | {r['Lend_error_norm_mm']:.3f} | {r['geometry']['clock']['half_window_zero_spread_ms']:.3f} | {'通過' if r['relative_geometry_eligible'] else '不合格・参考'} |\n"
(OUT/'README.md').write_text('# 機体B・1600 右out45：三回で単体暫定値を選択\n\n'
 '暫定値は速度1.6m/s、半径−60mm、Lstart21.8/Lend16.5mm、degree−45度、補正−0.9度、前走加速6.5m/s²。速度PID4/0.05/0、角速度PID0.20/0.01/0、角加減速FF各0.000750、吸引650。全FFはselected.settings.json。今回の補正はRAM試験と保存資料のみで、ファームソース/実機への反映は未実施。以前の入口/出口共通FFソース反映は保持。\n\n'+table+
 '\n全三回が一回実行・再送0、撮影PASSED・停止確認通過、実機設定一致、保存ログ欠落0。操作者が全三回で両方OFF・接触なしを明示。BLE E1:7F:86:21:84:73 / 8767を各回のプロセス引数まで照合。\n\n'
 '003は全908コマ中907有効、必要区間は実観測100%/ArUco登録100%、推定点0。入口37点/116.44mm/RMS0.337mm、出口17点/67.35mm/RMS0.210mm。入口raw wheel時計RMS0.508mm、半窓差0.491msで既定基準を通過。出口45.828度、Lend中点33.073/225.206mm、目標との差+1.254/+2.467mm（ノルム2.767mm）。停止後中点差+5.576/+4.114mm（約6.930mm）は別の残差。最低電圧7.695V。\n\n'
 '002は参考出口45.235度だが、時刻半窓差2.134ms>2msで相対幾何不合格のまま保持。同設定で有効反復二本を確認済みとは説明しない。003で品質と時刻照合が成立した実走済み設定を選び、未走行の長さ解20.55/14.75mmへは変更しない。\n\n'
 '相対マーカー中点と進行方向の単体暫定候補。絶対車軸・機体yaw・盤面固定の合格ではない。角速度の振れ、停止点残差、左・連続接続、BのEKF同定は未解決/未確認。追加走行は予約せず、両方OFFで区切る。\n\n'
 '[採用設定](selected.settings.json)・[判断と三回の元判定](decision.json)・[比較図](comparison.png)・[実画像](observed_frames_raw_orientation.jpg)。\n',encoding='utf-8')
LAST=BASE/names[-1][1]/'local_execution'/('run_'+names[-1][2])
for source,name in ((OUT/'decision.json','provisional_single_turn_decision.json'),(OUT/'selected.settings.json','selected_provisional.settings.json'),
 (OUT/'observed_frames_raw_orientation.jpg','observed_frames_raw_orientation.jpg'),(OUT/'raw_orientation_frame_evidence.json','visual_frame_evidence.json'),
 (OUT/'comparison.png','comparison_three.png')):
 with (LAST/name).open('xb') as f:f.write(source.read_bytes())
state_path=BASE/'resume_state.json';state=read(state_path);assert state['execution_commands_sent']==3
state.update(state='PROVISIONAL_RIGHT_OUT45_SELECTED_BOTH_OFF',selected_settings=str(OUT/'selected.settings.json'),
 provisional_decision=str(OUT/'decision.json'),latest_aggregate_review=str(OUT/'decision.json'),
 latest_independent_relative_geometry_eligible=True,latest_standard_full_summary=final['standard_summary'],
 latest_review=str(LAST/'midpoint_raw_wheel_clock_v3/midpoint_review.json'),
 physical_power_state='BOTH_OFF_USER_CONFIRMED',operator_ready=False,current_placement_confirmed=False,
 execution_pending=False,next_stage=None,next_request_id=None,next_request_sent=False,next_operator_ready=False,
 next_settings=None,readiness_question_pending=None,next_execution_scheduled=False,
 next_action='右out45の暫定値を保存して終了。今回の角度補正のソース反映/実機書込み、左/連続接続は未実施。再開は新規の配置/ON/手離し確認から。')
state['ble_binding'].update(pending_request_id=None,request_sent=True)
state_path.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
with (BASE/'README.md').open('a',encoding='utf-8') as f:
 f.write('\n## 三回での暫定決定\n\n実走合計三回。003は必要区間実観測/ArUco登録100%・時計半窓差0.491msで相対幾何判定通過。最終出口45.828度、Lend差2.767mm、停止点差約6.93mm。補正−0.9度・Lstart21.8/Lend16.5mm・PID/FF据え置きを単体暫定値に選択。002の不合格は保持し、採用設定で有効二本の再現性確認済みとはしない。[三回の決定記録](selected_m0p9_three_review_v1/README.md)。両方OFF。追加走行予約なし。今回の補正ソース反映/実機書込み・左/連続は未実施。\n')
print(json.dumps(dict(state=decision['state'],runs=3,correction=-.9,Lstart=21.8,Lend=16.5,
 final_angle=final['angle_deg'],Lend_error_mm=final['Lend_error_norm_mm'],stop_error_mm=final['stop_error_norm_mm'],
 next_run_scheduled=False,firmware_source_updated=False),ensure_ascii=False))
