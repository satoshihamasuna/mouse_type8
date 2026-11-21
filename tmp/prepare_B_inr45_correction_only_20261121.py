"""実映像色域の検証結果を保存し、角度補正だけを変更する未実行候補を用意する。"""
from pathlib import Path
from datetime import datetime,timezone
import copy
import hashlib
import json
import shutil
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr45_20261120_v1'
VALIDATION=BASE/'red_hue160_validation_v1';PREVIOUS=BASE/'geometry02_repeat_02';OUT=BASE/'geometry03_correction_m0p4'
OUT.mkdir(exist_ok=False)
read=lambda p:json.loads(Path(p).read_text(encoding='utf-8'))
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(p,v):
 with Path(p).open('x',encoding='utf-8') as f:json.dump(v,f,ensure_ascii=False,indent=2,allow_nan=False)
now=datetime.now(timezone.utc).isoformat();rows=[];angles=[];points=[]
for key in ('006','007','008'):
 folder=VALIDATION/('run'+key);validation=read(folder/'validation_result.json');geom=read(folder/'midpoint/midpoint_review.json')
 run=Path(geom['source_trial']).parent;trial=read(run/'trial.json');settings=read(run/'saved_1ms.settings.json')
 operator=read(run/'operator_after_run.json');assert operator['both_powered_off'] and operator['no_contact']
 assert validation['relative_geometry_eligible'] and validation['original_inputs_unchanged']
 assert validation['motion_quality']['observation_rate']>=.95 and validation['motion_quality']['calibration_rate']==1
 if key=='008':assert not validation['global_capture_passed'] and trial['state']=='FAILED'
 else:assert validation['global_capture_passed']
 tracking=read(run/'tracking_1ms_review_v3/verification.json')
 oldgeom=read(run/'midpoint_raw_wheel_clock_v3/midpoint_review.json') if key!='008' else None
 rows.append(dict(run_id=key,source_trial=str(run/'trial.json'),settings=settings,geometry=geom,tracking=tracking,
  capture_passed=validation['global_capture_passed'],operator=operator,old_geometry=oldgeom,
  detector_comparison=read(folder/'full/same_frame_detector_comparison.json'),qualification=read(folder/'full/window_qualification.json')))
 angles.append(geom['exit_path_direction_change_deg']);points.append(geom['midpoint_Lend_right_forward_mm'])
assert all(all(row['settings'][k]==rows[0]['settings'][k] for k in ('parameters','pid','feedforward','suction','turn_count')) for row in rows)
mean_angle=float(np.mean(angles));old=rows[0]['settings'];correction=round(old['parameters']['correction']+mean_angle-45,1)
assert correction==-.4
report=dict(schema_version=1,reviewed_at_utc=now,mouse='B',motion='in_r45',preset=1600,
 status='OFFLINE_DETECTOR_VALIDATED_ANGLE_REFINEMENT_PENDING',rows=rows,
 two_globally_passed_captures=['006','007'],independent_contiguous_window_only=['008'],
 failed_capture_reclassified=False,thresholds_changed=False,
 operator_conditions_reply=dict(source='ユーザー回答「とくになし」',lighting_changed=False,markers_changed=False,new_B_HY0020_mounting_changed=False),
 marker_profile_change=dict(front_unchanged=True,rear_hue_before=[[0,10],[170,179]],rear_hue_after=[[0,10],[160,179]],
  saturation_value_area_opening_and_span_unchanged=True,missing_positions_filled=False,
  diagnostic_actual_frame_audit=str(VALIDATION/'frame_audit_v2/actual_markers.png'),
  manual_review='最大差分・欠測回復の実6フレームを目視。両点は実マーカー上。006の赤点最大4.17mm差は動体ぼけ内で、背景や基板部品への乗換えではない。',
  heldout_exit_delta_deg=[row['geometry']['exit_path_direction_change_deg']-row['old_geometry']['exit_path_direction_change_deg'] for row in rows[:2]],
  heldout_Lend_delta_mm=[(np.array(row['geometry']['midpoint_Lend_right_forward_mm'])-row['old_geometry']['midpoint_Lend_right_forward_mm']).tolist() for row in rows[:2]]),
 relative_geometry=dict(exit_deg=angles,mean_exit_deg=mean_angle,clean_capture_mean_exit_deg=float(np.mean(angles[:2])),
  exit_range_deg=float(np.ptp(angles)),Lend_right_forward_mm=points,mean_Lend_right_forward_mm=np.mean(points,axis=0).tolist(),
  median_Lend_right_forward_mm=np.median(points,axis=0).tolist(),Lend_component_range_mm=np.ptp(points,axis=0).tolist(),target_mm=[45.,180.]),
 next_candidate=dict(parameters_changed={'correction':[old['parameters']['correction'],correction]},
  basis='3本の相対出口方向の平均差を指令角へ1対1で戻す小角度近似。録画合格2本のみの平均でも丸め候補は同じ−0.4度。',
  expected_mean_exit_deg=mean_angle+old['parameters']['correction']-correction,validation_state='NOT_EXECUTED',
  length_change_deferred=True,Lstart_mm=7.8,Lend_mm=34.5),
 limitations=['相対的なマーカー中点の形状。盤面固定の出口/停止・絶対車軸・body yawの合格ではない。',
  '008全体は停止後のPTS欠落でFAILED。連続した必要区間だけの独立評価で、元の撮影判定とauto_stop=falseは保持。',
  'BのEKF係数は識別未完了。Aの係数を流用しない。右in45単体のみで左・連続接続は未検証。',
  '色域は今回B・撮影条件の実映像校正。新しい照明/マーカー/カメラ条件への汎用合格ではない。'],
 physical_power_state='BOTH_OFF_USER_CONFIRMED',next_request_sent=False,firmware_written=False,source_params_changed=False,
 input_sha256={str(p):sha(p) for p in [Path(__file__),ROOT/'tools/analysis/pixel_camera_relative_pose.py',ROOT/'tools/analysis/video_marker_pose.py',
 ROOT/'tools/marker_detection.py',VALIDATION/'frame_audit_v2/frames.json']+[VALIDATION/('run'+key)/'validation_result.json' for key in ('006','007','008')]})
save(VALIDATION/'review.json',report)
profile=copy.deepcopy(read(BASE/'body_geometry_B_green_red_h160_candidate_v4.json'))
profile.update(analysis_profile_adoption_state='VALIDATED_OFFLINE_FOR_THIS_B_CAPTURE_CONDITION',
 marker_detection_source='B実映像の赤H160〜169を含める。008実画素・過去006/007の独立映像・実フレーム差分監査で検証。95%基準、ArUco実登録、欠測方針は維持。',
 led_on_marker_validation='006/007/008の走行中LEDを含む全フレーム窓を検証。各runのrequired_motion_window_qualityとframe_auditを参照。',
 validation_report=str(VALIDATION/'review.json'),validation_report_sha256=sha(VALIDATION/'review.json'))
profile_path=BASE/'body_geometry_B_green_red_h160_validated_v4.json';save(profile_path,profile)
candidate=copy.deepcopy(read(PREVIOUS/'candidate.settings.json'));candidate['parameters']['correction']=correction
candidate['source']='B1600 inR45：相対出口角46.167/46.103/45.945度に基づく角度補正だけの未実行候補。Ls7.8/Le34.5・制御/吸引は固定。'
save(OUT/'candidate.settings.json',candidate)
config=read(PREVIOUS/'config.json');assert config['address']=='E1:7F:86:21:84:73' and config['ble_port']==8767
config['geometry']=str(profile_path);save(OUT/'config.json',config)
identifier='B_INR45_1600_correction_m0p4_green_20261121_009'
source=(PREVIOUS/'run_recorded_once.py').read_text(encoding='utf-8').replace('B_INR45_1600_geometry02_repeat02_green_20261120_008',identifier)
compile(source,str(OUT/'run_recorded_once.py'),'exec');(OUT/'run_recorded_once.py').write_text(source,encoding='utf-8')
shutil.copyfile(PREVIOUS/'coarse_placement_check.py',OUT/'coarse_placement_check.py')
plan=read(PREVIOUS/'placement_plan.json');plan.update(request_id=identifier,request_sent=False,operator_ready=False,current_placement_confirmed=False,
 physical_power_state='BOTH_OFF_USER_CONFIRMED',power_state='BOTH_OFF_USER_CONFIRMED',execution_commands_sent=0,settings_write_commands_sent=0,
 placement_basis='006/007/008のB1600右in45で225/45・前緑下向きの走行実績と操作者の接触なし確認あり。新規試行の干渉余裕は操作者が再確認。',
 comparison=dict(phase='angle_correction_only',changed_fields=['parameters.correction'],previous=old['parameters'],requested=candidate['parameters'],basis=str(VALIDATION/'review.json')),
 relative_Lend_from_pre_run_start_right_forward_mm=[45.,180.],geometry_profile=str(profile_path),new_one_run_readiness_required=True)
save(OUT/'placement_plan.json',plan)
diff={group:{k:[old[group][k],v] for k,v in candidate[group].items() if old[group][k]!=v} for group in ('parameters','pid','feedforward','suction')}
assert {g:d for g,d in diff.items() if d}=={'parameters':{'correction':[-1.5,-.4]}}
save(OUT/'offline_validation.json',dict(state='PREPARED_NOT_EXECUTED',differences=diff,request_id=identifier,request_sent=False,
 ble_address=config['address'],ble_port=8767,camera_port=8765,front='green',rear='red',new_hardware_readiness_required=True,
 hardware_commands_sent=0,source_params_changed=False,firmware_written=False,review_sha256=sha(VALIDATION/'review.json'),
 input_sha256={str(p):sha(p) for p in (OUT/'config.json',OUT/'candidate.settings.json',OUT/'run_recorded_once.py',profile_path)}))
plt.rcParams['font.family']='Yu Gothic';fig,axes=plt.subplots(1,3,figsize=(13,4.5),layout='constrained')
for row in rows:
 key=row['run_id'];g=row['geometry'];data=pd.read_csv(VALIDATION/('run'+key)/'midpoint/midpoint_relative_trajectory.csv')
 t=data.time_video_s.to_numpy();valid=data.registered_midpoint_valid.to_numpy(bool);q=g['independent_clock_required_motion_quality']
 mask=(t>=q['video_motion_start_s']-.03)&(t<=q['video_motion_end_s']+.07)
 xy=np.where(valid[:,None],data[['right_mm','forward_mm']].to_numpy(),np.nan)
 axes[0].plot(*xy[mask].T,label=key+(' 区間のみ' if key=='008' else ''))
 axes[1].scatter(*g['midpoint_Lend_right_forward_mm'],label=key)
axes[0].set(xlabel='入口に対する右 [mm]',ylabel='入口に対する前 [mm]',aspect='equal',title='同一設定の相対軌道')
axes[0].legend(fontsize=8);axes[1].scatter(45,180,c='black',marker='x',label='目標45/180');axes[1].set(xlabel='右 [mm]',ylabel='前 [mm]',aspect='equal',title='Lendの相対中点')
axes[1].legend(fontsize=8);axes[2].bar(['006','007','008\n区間のみ'],angles);axes[2].axhline(45,c='black',ls='--',label='目標45°')
axes[2].set(ylim=(44.5,46.5),ylabel='出口方向の変化 [deg]',title='出口角：補正前の残差約+1.1°');axes[2].legend(fontsize=8)
for ax in axes:ax.grid(alpha=.25)
fig.savefig(VALIDATION/'comparison.png',dpi=150);plt.close(fig)
text='# 機体B turn1600 右in45：赤色域検証と角度候補\n\n'
text+='前緑/後赤、1500mm、EV−1.5、1.8倍、ライトOFF、AF固定。操作者は照明・マーカー・B用HY0020取付の変更なしと回答。全実行後に両方OFF・接触なしを確認。\n\n'
text+='後赤がH160〜169に写る実画素を確認し、赤の色相だけ170〜179から160〜179へ拡張（0〜10は保持）。S/V・面積・開演算・緑・前後間隔・95%観測率・各フレームArUco2個以上の実登録は変更していない。欠測を推定位置へ置き換えていない。\n\n'
text+='| 試行 | 録画全体 | 必要区間の実観測率 | 相対出口角 | Lend右/前 mm |\n|---|---|---:|---:|---:|\n'
for row in rows:
 g=row['geometry'];text+=f"| {row['run_id']} | {'PASSED' if row['capture_passed'] else 'FAILED・連続区間のみ独立評価'} | {g['independent_clock_required_motion_quality']['observation_rate']*100:.2f}% | {g['exit_path_direction_change_deg']:.3f}° | {g['midpoint_Lend_right_forward_mm'][0]:.3f}/{g['midpoint_Lend_right_forward_mm'][1]:.3f} |\n"
text+='\n全3本のArUco登録率100%。008は停止後12.6388〜12.6765秒の37.64ms間隔で全体FAILEDを保持。必要区間9.0486〜9.6496秒はPTSが連続し、独立した入口wheel時計の品質も通過。正常録画3本に数え直していない。動画・ログ・元判定を変更していない。\n\n'
text+='006/007の旧解析との差は出口角−0.037/+0.055度、Lend各成分最大0.227mm。後点の同一フレーム位置差95%点は0.305/0.460mm、最大4.17/1.28mm。最大差分と回復フレーム6枚を目視し、実マーカーの動体ぼけ内であることを確認。全体像と個別判定はreview.json、画像はframe_audit_v2/actual_markers.png。\n\n'
text+=f'相対出口角平均{mean_angle:.3f}度（録画合格2本だけでも{np.mean(angles[:2]):.3f}度）。次候補は角度補正−1.5→−0.4度のみ。Lstart7.8/Lend34.5mm・半径−55mm・速度1.6m/s・Ki0.005・角加減速FF0.000750・吸引650を固定。角度応答を1対1と近似した候補で、次の一回で確認し、その後に長さを再評価する。\n\n'
text+='入口実測方向に合わせた相対中点の評価。盤面固定の設置/出口/停止、絶対車軸/body yaw、左・連続接続、BのEKFは未確認。元summaryのauto_stop=falseを保持し、取得時の停止証拠・後半静止像・操作者OFF/接触なしと区別した。ファーム・Paramsは変更していない。\n\n'
text+=f'次の準備先は ../{OUT.name}/。要求ID {identifier} は未送信。B専用E1:7F:86:21:84:73、BLE8767（カメラ8765）。新しいON・手離し・一回可の確認が必要。\n'
(VALIDATION/'README.md').write_text(text,encoding='utf-8')
state_path=BASE/'resume_state.json';shutil.copyfile(state_path,VALIDATION/'resume_state.before_completion.json');state=read(state_path)
state.update(state='ANGLE_REFINEMENT_009_PREPARED_NOT_EXECUTED',stage=str(PREVIOUS),physical_power_state='BOTH_OFF_USER_CONFIRMED',
 operator_ready=False,current_physical_power_state_unconfirmed=False,execution_pending=False,execution_commands_sent=6,
 next_action='赤の色域校正は保存映像で検証済み。B専用割当を保持。元225/45・前緑下向きへ概略配置し、新規の両方ON・手離し・一回可を確認後、角度補正だけ−0.4度の009を取得する。',
 next_stage=str(OUT),next_request_id=identifier,next_request_sent=False,next_operator_ready=False,next_settings=str(OUT/'candidate.settings.json'),
 latest_review=str(VALIDATION/'run008/midpoint/midpoint_review.json'),latest_aggregate_review=str(VALIDATION/'review.json'),
 same_candidate_runs=2,same_candidate_independent_window_reviews=1,remaining_same_candidate_repeats=0,
 latest_capture_passed=False,latest_capture_independent_contiguous_window_eligible=True,failed_capture_reclassified=False,
 archive=read(Path(rows[-1]['source_trial']).parent/'archive.json'),next_geometry_basis=str(VALIDATION/'review.json'))
for entry in state['repeat_stages']:
 if entry['request_id'].endswith('_008'):entry.update(request_sent=True,global_capture_passed=False,independent_contiguous_window_eligible=True)
state['ble_binding'].update(request_sent=True,pending_request_id=identifier,hardware_connections_started=1,
 current_binding_verified=True,current_binding_evidence=state['ble_binding_evidence'])
state_path.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
with (BASE/'README.md').open('a',encoding='utf-8') as f:f.write('\n2026-11-21：008実行済み・両方OFF/接触なし。停止後PTS欠落によるFAILEDは保持。赤H160〜169の実画素に基づく検出を過去006/007とともに検証し、必要区間の観測率99.32/100/97.22%。相対出口46.167/46.103/45.945度。赤色域検証と角度だけ−0.4度の未実行009候補はred_hue160_validation_v1/README.md。ソース/ファーム変更なし。\n')
print(json.dumps(dict(stage=str(OUT),parameters=candidate['parameters'],request_sent=False,mean_exit_deg=mean_angle),ensure_ascii=False))
