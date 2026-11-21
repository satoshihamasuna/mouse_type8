"""実赤画素と独立映像でS30を検証。取得時S40の不成立は変更しない。"""
from pathlib import Path
from datetime import datetime,timezone
import copy
import hashlib
import json
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr45_20261120_v1'
OUT=BASE/'red_s30_validation_v1';read=lambda p:json.loads(Path(p).read_text(encoding='utf-8-sig'))
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(p,v):
 with Path(p).open('x',encoding='utf-8') as f:json.dump(v,f,ensure_ascii=False,indent=2,allow_nan=False)
rows=[];inputs=[Path(__file__),OUT/'frame_audit_v1/frames.json']
for key in ('010','011','012'):
 folder=OUT/('run'+key);validation=read(folder/'validation_result.json');geometry=read(folder/'midpoint/midpoint_review.json')
 run=Path(geometry['source_trial']).parent;trial=read(run/'trial.json');tracking=read(run/'tracking_1ms_review_v3/verification.json')
 assert validation['relative_geometry_eligible'] and validation['global_capture_passed'] and validation['original_inputs_unchanged']
 assert geometry['independent_clock_required_motion_quality']['observation_rate']>=.95
 assert geometry['independent_clock_required_motion_quality']['calibration_rate']>=.95
 original_geometry=read(run/'midpoint_raw_wheel_clock_v3/midpoint_review.json') if key!='012' else None
 rows.append(dict(run_id=key,source_trial=str(run/'trial.json'),settings=read(run/'saved_1ms.settings.json'),geometry=geometry,
  tracking=tracking,operator=read(run/'operator_after_run.json'),capture_passed=True,full_summary=read(folder/'full/summary.json'),
  geometry_folder=str(folder/'midpoint'),detector_validation=validation,original_geometry=original_geometry,
  original_detector_quality=read(run/'full/report.json')['required_motion_window_quality'],
  charging_condition=read(run/'operator_charging_condition.json') if key=='012' else None))
 inputs.extend([folder/'validation_result.json',folder/'midpoint/midpoint_review.json',run/'trial.json',run/'saved_1ms.settings.json'])
assert not rows[-1]['charging_condition']['charging_completed_before_trial']
same=rows[1:];angles=np.array([r['geometry']['exit_path_direction_change_deg'] for r in same]);points=np.array([r['geometry']['midpoint_Lend_right_forward_mm'] for r in same])
report=dict(schema_version=1,reviewed_at_utc=datetime.now(timezone.utc).isoformat(),mouse='B',motion='in_r45',preset=1600,
 state='S30_ANALYSIS_VALIDATED_SAME_SETTINGS_REPEATS_BEFORE_CHARGING_COMPLETION',rows=rows,
 current_condition=dict(parameters=rows[-1]['settings']['parameters'],run_ids=['011','012'],runs=2,exit_deg=angles.tolist(),
  mean_exit_deg=float(angles.mean()),exit_range_deg=float(np.ptp(angles)),Lend_right_forward_mm=points.tolist(),
  mean_Lend_right_forward_mm=points.mean(0).tolist(),Lend_component_range_mm=np.ptp(points,axis=0).tolist(),
  minimum_battery_v=[r['tracking']['battery_min_v'] for r in same],charged_reproducibility_confirmed=False),
 detector_change=dict(red_minimum_saturation_before=40,red_minimum_saturation_after=30,other_detector_fields_unchanged=True,
  threshold95_unchanged=True,per_frame_real_aruco_required=True,missing_positions_filled=False,
  manual_frame_audit='最大差分/回復の実6フレームを目視。前後点は実マーカー上で、赤点差分はぼけ像内。背景・基板部品への乗換えは見られない。',
  frame_audit=str(OUT/'frame_audit_v1/actual_markers.png'),heldout_run_ids=['010','011'],
  heldout_exit_delta_deg=[r['geometry']['exit_path_direction_change_deg']-r['original_geometry']['exit_path_direction_change_deg'] for r in rows[:2]],
  heldout_Lend_delta_mm=[(np.array(r['geometry']['midpoint_Lend_right_forward_mm'])-r['original_geometry']['midpoint_Lend_right_forward_mm']).tolist() for r in rows[:2]]),
 original012_quality_rejection_preserved=True,original012_capture_and_profile_unchanged=True,
 original012_rejection_record=str(Path(rows[-1]['source_trial']).parent/'quality_rejection_record.json'),
 fresh_s30_analysis_separately_qualified=True,source_params_changed=False,firmware_written=False,hardware_commands_sent=0,
 physical_power_state='BOTH_OFF_USER_CONFIRMED',charging_completion_confirmed=False,
 limitations=['今回はBの同じ撮影条件の解析用色校正。別の照明/機体へ自動適用しない。',
  '012は明示回答により充電未完了での反復。充電完了後の再現性・採用値は未確認。',
  '元S40の不成立、標準summary/auto_stop判定、取得時条件を上書きしていない。',
  '相対中点のみ。絶対車軸/body yaw、盤面固定接続、左・連続、BのEKFは未確認。'],
 input_sha256={str(p):sha(p) for p in inputs})
save(OUT/'review.json',report)
profile=copy.deepcopy(read(BASE/'body_geometry_B_green_red_h160_s30_candidate_v5.json'))
profile.update(analysis_profile_adoption_state='VALIDATED_OFFLINE_FOR_THIS_B_CAPTURE_CONDITION',
 marker_detection_source='012実赤画素に基づき赤S40→30のみ変更。010/011独立映像と最大差分/回復実フレームを検証。95%・実ArUco・欠測方針は保持。',
 validation_report=str(OUT/'review.json'),validation_report_sha256=sha(OUT/'review.json'))
save(BASE/'body_geometry_B_green_red_h160_s30_validated_v5.json',profile)
plt.rcParams['font.family']='Yu Gothic';fig,axes=plt.subplots(1,3,figsize=(12,4.5),layout='constrained')
for row in rows:
 g=row['geometry'];data=pd.read_csv(Path(row['geometry_folder'])/'midpoint_relative_trajectory.csv');t=data.time_video_s.to_numpy();q=g['independent_clock_required_motion_quality']
 mask=(t>=q['video_motion_start_s']-.03)&(t<=q['video_motion_end_s']+.07);xy=np.where(data.registered_midpoint_valid.to_numpy(bool)[:,None],data[['right_mm','forward_mm']].to_numpy(),np.nan)
 axes[0].plot(*xy[mask].T,label=row['run_id']+f" 補正{row['settings']['parameters']['correction']:+.1f}°")
 axes[1].scatter(*g['midpoint_Lend_right_forward_mm'],label=row['run_id'])
 axes[2].scatter(row['settings']['parameters']['correction'],g['exit_path_direction_change_deg'],label=row['run_id'])
axes[0].set(xlabel='入口に対する右 [mm]',ylabel='入口に対する前 [mm]',aspect='equal',title='S30で揃えた相対軌道');axes[1].scatter(45,180,c='black',marker='x',label='目標45/180')
axes[1].set(xlabel='右 [mm]',ylabel='前 [mm]',aspect='equal',title='Lendの相対中点');axes[2].axhline(45,c='black',ls='--',label='目標45°')
axes[2].set(xlabel='角度補正 [deg]',ylabel='出口方向の変化 [deg]',title='同一設定011/012は充電前')
for ax in axes:ax.grid(alpha=.25);ax.legend(fontsize=8)
fig.savefig(OUT/'comparison.png',dpi=150);plt.close(fig)
text='# 機体B1600 右in45：赤彩度S30の検証と充電前反復\n\n'
text+='012は充電未完了との明示回答を保存。取得時S40で必要区間90.28%となった不成立を保持し、別出力へS30で再解析した。実赤画素H160〜179/S30〜39を確認し、赤の彩度だけ40→30。色相・明度・面積・開演算・緑・前後間隔・95%基準・実ArUco・20ms補間上限は変更していない。\n\n'
text+='| 試行 | 補正 ° | 出口 ° | Lend右/前 mm | 実観測/ArUco率 | 最低電圧 V |\n|---|---:|---:|---:|---:|---:|\n'
for row in rows:
 g=row['geometry'];q=g['independent_clock_required_motion_quality'];p=g['midpoint_Lend_right_forward_mm']
 text+=f"| {row['run_id']} | {row['settings']['parameters']['correction']:+.1f} | {g['exit_path_direction_change_deg']:.3f} | {p[0]:.3f}/{p[1]:.3f} | {q['observation_rate']*100:.2f}/{q['calibration_rate']*100:.2f}% | {row['tracking']['battery_min_v']:.3f} |\n"
text+='\n012の検証候補は必要区間100%。独立の010/011では出口角差−0.022/−0.173度、Lend各成分の変化は最大0.457mm。最大差分と回復実フレーム6枚を目視し、実マーカー上の点であることを確認。元画像・座標・全フレームprovenance・入力ハッシュを保存。\n\n'
text+=f'同じ補正−1.2度の011/012の出口平均{angles.mean():.3f}度、範囲{np.ptp(angles):.3f}度。Lend成分範囲は{np.ptp(points,axis=0)[0]:.3f}/{np.ptp(points,axis=0)[1]:.3f}mm。両方とも最新の充電依頼後に充電完了を確認したデータではない。次は充電完了を明示確認して同設定を検証する。\n\n'
text+='S30は今回条件の解析用校正候補として検証済み。制御値は未採用、Params/ファーム変更なし。元summary/auto_stopと元S40判定を保持。相対中点のみで、絶対車軸/body yaw、盤面固定接続、左・連続、BのEKFは未確認。\n'
(OUT/'README.md').write_text(text,encoding='utf-8')
state_path=BASE/'resume_state.json';state=read(state_path);state.update(latest_aggregate_review=str(OUT/'review.json'),
 latest_review=str(OUT/'run012/midpoint/midpoint_review.json'),latest_independent_relative_geometry_eligible=True,
 latest_relative_geometry_qualification='NEW_S30_ANALYSIS_ONLY_ORIGINAL_S40_REJECTION_PRESERVED',current_correction_same_condition_runs=2,
 current_repeat_charging_completed=False,charging_completion_confirmed=False,archive=read(Path(rows[-1]['source_trial']).parent/'archive.json'))
state_path.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(analysis_profile='S30 validated for current B conditions',new012_exit_deg=angles[-1],new012_point=points[-1].tolist(),mean_m1p2_exit_deg=float(angles.mean()),charging_completed=False),ensure_ascii=False))
