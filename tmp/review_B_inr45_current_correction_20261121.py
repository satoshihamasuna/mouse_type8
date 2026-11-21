"""完了済みの現在試行を既存の補正比較へ追加。元の撮影・標準判定を保持する。"""
from pathlib import Path
from datetime import datetime,timezone
import argparse
import copy
import hashlib
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr45_20261120_v1'
parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',required=True);args=parser.parse_args()
read=lambda p:json.loads(Path(p).read_text(encoding='utf-8-sig'))
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(p,v):
 with Path(p).open('x',encoding='utf-8') as f:json.dump(v,f,ensure_ascii=False,indent=2,allow_nan=False)
state=read(BASE/'resume_state.json');assert state['physical_power_state']=='BOTH_OFF_USER_CONFIRMED'
RUN=Path(state['source_trial']).parent;assert RUN.is_relative_to(BASE)
parent_path=Path(state['latest_aggregate_review']);parent=read(parent_path);rows=copy.deepcopy(parent['rows'])
identifier=state['request_id'];key=identifier.rsplit('_',1)[1];assert RUN.name=='run_'+identifier and all(row['run_id']!=key for row in rows)
trial=read(RUN/'trial.json');settings=read(RUN/'saved_1ms.settings.json');operator=read(RUN/'operator_after_run.json')
geometry_folder=RUN/'midpoint_raw_wheel_clock_v3';geometry=read(geometry_folder/'midpoint_review.json')
tracking=read(RUN/'tracking_1ms_review_v3/verification.json');summary=read(RUN/'full/summary.json');postprocess=read(RUN/'postprocess_status.json')
assert settings['mouse']=='B' and settings['motion_name']=='in_r45' and settings['preset_speed_mm_s']==1600 and settings['turn_count']==1
assert trial['camera_state']=='PASSED' and trial['execution_resend_count']==0 and operator['both_powered_off'] and operator['no_contact']
assert geometry['independent_optical_geometry_eligible'] and postprocess['midpoint_returncode']==0 and postprocess['tracking_returncode']==0
quality=geometry['independent_clock_required_motion_quality'];assert quality['observation_rate']>=.95 and quality['calibration_rate']>=.95
base=rows[0]['settings'];assert all(settings[g]==base[g] for g in ('pid','feedforward','suction','turn_count'))
assert all(settings['parameters'][k]==base['parameters'][k] for k in settings['parameters'] if k!='correction')
current=dict(run_id=key,source_trial=str(RUN/'trial.json'),settings=settings,geometry=geometry,tracking=tracking,operator=operator,
 capture_passed=True,full_summary=summary,postprocess_status=postprocess,geometry_folder=str(geometry_folder))
rows.append(current)
same=[row for row in rows if all(row['settings'][g]==settings[g] for g in ('parameters','pid','feedforward','suction','turn_count'))]
angles=np.array([row['geometry']['exit_path_direction_change_deg'] for row in same])
points=np.array([row['geometry']['midpoint_Lend_right_forward_mm'] for row in same]);goal=np.array([45.,180.])
OUT=(BASE/args.output).resolve();assert OUT.is_relative_to(BASE);OUT.mkdir(exist_ok=False)
result=dict(schema_version=1,reviewed_at_utc=datetime.now(timezone.utc).isoformat(),mouse='B',motion='in_r45',preset=1600,
 state='CURRENT_RELATIVE_CANDIDATE_REVIEWED_REPRODUCIBILITY_PENDING',rows=rows,current_run_id=key,
 current_condition=dict(parameters=settings['parameters'],pid=settings['pid'],feedforward=settings['feedforward'],suction=settings['suction'],
  run_ids=[row['run_id'] for row in same],runs=len(same),exit_deg=angles.tolist(),mean_exit_deg=float(angles.mean()),
  exit_error_from45_deg=(angles-45.).tolist(),exit_range_deg=float(np.ptp(angles)),Lend_right_forward_mm=points.tolist(),
  mean_Lend_right_forward_mm=points.mean(0).tolist(),Lend_residual_mm=(points-goal).tolist(),Lend_component_range_mm=np.ptp(points,axis=0).tolist(),
  observation_rates=[row['geometry']['independent_clock_required_motion_quality']['observation_rate'] for row in same],
  minimum_battery_v=[row['tracking']['battery_min_v'] for row in same]),
 standard_full_summary=summary,standard_full_returncode=postprocess['full_returncode'],independent_relative_geometry_eligible=True,
 old_failed_008_reclassified=False,absolute_axle_and_body_yaw_available=False,hardware_commands_sent=0,source_params_changed=False,firmware_written=False,
 current_physical_power_state='BOTH_OFF_USER_CONFIRMED',ekf_identified=False,
 limitations=['各評価コマの実ArUco登録・前後マーカー観測・95%基準を維持。欠測を観測値に補完しない。',
  '入口raw wheel時計・20ms補間上限・既存の出口窓を保持。独立ハードウェア同期ではない。',
  '相対マーカー中点の形状。絶対車軸/body yaw、盤面固定の設置/出口/停止、左・連続接続は未確認。',
  '標準summaryの要確認/auto_stop判定と元returncodeを保持。取得時停止証拠・後半静止像・操作者OFF/接触なしは別記録。',
  '各runの電圧を併記。BのEKFは識別未完了。制御ピーク・後揺れは残っている。'],
 input_sha256={str(p):sha(p) for p in (parent_path,RUN/'trial.json',RUN/'saved_1ms.csv',RUN/'saved_1ms.settings.json',
  RUN/'body_geometry_used.json',geometry_folder/'midpoint_review.json',RUN/'full/summary.json',RUN/'postprocess_status.json',
  RUN/'tracking_1ms_review_v3/verification.json',Path(__file__))})
save(OUT/'review.json',result)
table=[]
for row in rows:
 g=row['geometry'];t=row['tracking'];p=g['midpoint_Lend_right_forward_mm'];q=g['independent_clock_required_motion_quality']
 table.append(dict(run_id=row['run_id'],correction_deg=row['settings']['parameters']['correction'],capture_passed=row['capture_passed'],
  observation_rate=q['observation_rate'],calibration_rate=q['calibration_rate'],exit_deg=g['exit_path_direction_change_deg'],
  right_mm=p[0],forward_mm=p[1],right_error_mm=p[0]-45,forward_error_mm=p[1]-180,
  om_rms_rad_s=t['tracking']['tracking_rms_rad_s'],om_peak_rad_s=t['tracking']['ego_peak_rad_s'],min_battery_v=t['battery_min_v']))
pd.DataFrame(table).to_csv(OUT/'comparison.csv',index=False)
plt.rcParams['font.family']='Yu Gothic';fig,axes=plt.subplots(1,3,figsize=(13,4.5),layout='constrained')
for row in rows:
 label=row['run_id']+f" 補正{row['settings']['parameters']['correction']:+.1f}°"+(' 区間のみ' if not row['capture_passed'] else '')
 g=row['geometry'];data=pd.read_csv(Path(row['geometry_folder'])/'midpoint_relative_trajectory.csv')
 time=data.time_video_s.to_numpy();valid=data.registered_midpoint_valid.to_numpy(bool);q=g['independent_clock_required_motion_quality']
 mask=(time>=q['video_motion_start_s']-.03)&(time<=q['video_motion_end_s']+.07)
 xy=np.where(valid[:,None],data[['right_mm','forward_mm']].to_numpy(),np.nan)
 active=row['run_id']==key
 axes[0].plot(*xy[mask].T,lw=2 if active else 1,alpha=1 if active else .4,label=label)
 axes[1].scatter(*g['midpoint_Lend_right_forward_mm'],s=80 if active else 30,label=row['run_id'])
 axes[2].scatter(row['settings']['parameters']['correction'],g['exit_path_direction_change_deg'],s=80 if active else 30,label=row['run_id'])
axes[0].set(xlabel='入口に対する右 [mm]',ylabel='入口に対する前 [mm]',aspect='equal',title='長さ固定・角度補正比較');axes[0].legend(fontsize=6.5)
axes[1].scatter(*goal,c='black',marker='x',label='目標45/180');axes[1].set(xlabel='右 [mm]',ylabel='前 [mm]',aspect='equal',title='Lendの相対中点');axes[1].legend(fontsize=7)
axes[2].axhline(45,c='black',ls='--',label='目標45°');axes[2].set(xlabel='角度補正 [deg]',ylabel='出口方向の変化 [deg]',title='補正値と実測出口角');axes[2].legend(fontsize=7)
for ax in axes:ax.grid(alpha=.25)
fig.savefig(OUT/'comparison.png',dpi=150);plt.close(fig)
point=np.array(geometry['midpoint_Lend_right_forward_mm']);angle=geometry['exit_path_direction_change_deg']
text=f"# 機体B1600 右in45：{key}の相対評価\n\n補正{settings['parameters']['correction']:+.1f}度、Lstart7.8/Lend34.5mm、半径−55mm、速度1.6m/s。制御と吸引650を固定。\n\n"
text+=f'今回の出口角{angle:.3f}度、45度との差{angle-45:+.3f}度。Lend右{point[0]:.3f}/前{point[1]:.3f}mm、目標45/180との差{point[0]-45:+.3f}/{point[1]-180:+.3f}mm。\n\n'
text+=f"必要区間の実観測率{quality['observation_rate']*100:.2f}%・ArUco登録率{quality['calibration_rate']*100:.2f}%。登録不成立のフレームは位置評価から除外。1ms保存{tracking['sample_count']}点/drop0、最低電圧{tracking['battery_min_v']:.3f}V。全体録画PASSED、標準full returncode={postprocess['full_returncode']}・auto_stop={summary.get('auto_stop_confirmed')}を保持。"
if summary.get('error'):text+='標準summaryの理由：'+summary['error']+'。'
text+='入口raw wheel時計の相対形状評価は通過。取得時停止証拠・後半静止動画・操作者の両方OFF/接触なしは別に保存。\n\n'
text+='| 試行 | 補正 ° | 出口 ° | Lend右/前 mm | 実観測率 | 最低電圧 V |\n|---|---:|---:|---:|---:|---:|\n'
for row in table:text+=f"| {row['run_id']}{' 区間のみ' if not row['capture_passed'] else ''} | {row['correction_deg']:+.1f} | {row['exit_deg']:.3f} | {row['right_mm']:.3f}/{row['forward_mm']:.3f} | {row['observation_rate']*100:.2f}% | {row['min_battery_v']:.3f} |\n"
text+=f'\n現在の同一設定は{len(same)}本。反復状況はreview.jsonのcurrent_conditionを参照。008全体のFAILEDを保持し、連続必要区間のみの独立評価と区別する。\n\n'
text+='相対マーカー中点の形状候補。盤面固定の設置/出口/停止、絶対車軸/body yaw、左・連続接続、BのEKFは未確認。Params/ファームは変更していない。\n'
(OUT/'README.md').write_text(text,encoding='utf-8')
print(json.dumps(dict(output=str(OUT),run=key,correction=settings['parameters']['correction'],exit_deg=angle,
 Lend=point.tolist(),residual=(point-goal).tolist(),quality=quality['observation_rate'],same_condition_runs=len(same)),ensure_ascii=False))
