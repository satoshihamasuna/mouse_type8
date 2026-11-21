"""補正+1.0°の保存データ評価と、未送信の同条件再確認を準備する。実機通信なし。"""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json
import shutil

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT/'tools/logs/pixel_camera/turn2000_B_r90_20261120_v1'
PREP = BASE/'next_correction_plus1p0_20261120_v1'
RUN = PREP/'run_session01/run_B_R90_2000_correction1p0_20261120_001'
NEXT = BASE/'next_correction_plus1p0_repeat01_20261120_v1'
IDENTIFIER = 'B_R90_2000_correction1p0_repeat01_20261120_001'
now = datetime.now(timezone.utc).isoformat()
read = lambda p: json.loads(Path(p).read_text(encoding='utf-8'))
digest = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(path, obj):
    with Path(path).open('x',encoding='utf-8') as stream:
        json.dump(obj,stream,ensure_ascii=False,indent=2,allow_nan=False)
        stream.write('\n')

trial = read(RUN/'trial.json')
pose = pd.read_csv(RUN/'full/apparent_floor_marker_pose.csv')
report = read(RUN/'full/report.json')
summary = read(RUN/'full/summary.json')
geometry = read(RUN/'geometry_review_v1/video_comparison.json')['comparisons'][0]
track = read(RUN/'tracking_1ms_review_v2/verification.json')
sensitivity = read(RUN/'direction_window_sensitivity.json')
manual = read(RUN/'marker_identity_review_v2/sampled_points.json')
assert len(manual)==56 and geometry['relative_review_eligible']
assert trial['mouse']=='B' and trial['motion']=='long_r90' and trial['preset']==2000
assert trial['execution_resend_count']==0
assert read(RUN/'operator_stop.json')['both_off'] is True
save(RUN/'marker_identity_review_v2/review.json',dict(at=now,raw_orientation=True,
    samples=len(manual),whole_turn_frames=28,pages_reviewed=[1,2,3,4],
    sampled_marker_identity_passed=True,scope='Visual review of both physical marker disks in all56 sampled images, including every turn frame; not an exhaustive manual review of1236 frames.',
    excluded_prior_render='marker_identity_review: incorrect display rotation used for crops; not used as marker evidence. Numerical detector had raw orientation enabled from the start.'))
save(RUN/'marker_identity_review/INVALID_RENDER.json',dict(reason='Contact-sheet crop renderer used automatic display orientation against raw-frame coordinates. Images are not marker-identity evidence. Numerical analysis unaffected.',replacement='marker_identity_review_v2'))

paths = [BASE/'resume_entry_relative_20261120_v1/run_session01/run_B_R90_2000_omKi005_green_relative_20261120_003',
    BASE/'next_correction_plus1p5_20261120_v1/run_session01/run_B_R90_2000_correction1p5_20261120_001',RUN]
rows=[]
fingerprints=[]
for folder in paths:
    settings=read(folder/'saved_1ms.settings.json')
    canonical={key:settings[key] for key in ('mouse','motion_name','preset_speed_mm_s','turn_count','parameters','pid','feedforward','suction')}
    canonical=json.loads(json.dumps(canonical))
    canonical['parameters']['correction']='compared_separately'
    fingerprints.append(json.dumps(canonical,sort_keys=True))
    geom=read(folder/'geometry_review_v1/video_comparison.json')['comparisons'][0]
    tracking_file=folder/('tracking_1ms_review_v3' if folder==paths[1] else 'tracking_1ms_review_v2')/'verification.json'
    tracking=read(tracking_file)
    rows.append(dict(correction_deg=settings['parameters']['correction'],source_trial=str(folder/'trial.json'),
        angle_deg=geom['entry_to_exit_path_rotation_deg'],error_deg=geom['path_rotation_error_from90_deg'],
        entry_board_heading_deg=geom['entry_board_heading_deg'],exit_board_heading_deg=geom['exit_board_heading_deg'],
        exit_forward_offset_mm=geom['exit_line_offset_from270_mm'],
        peak_rad_s=tracking['tracking']['ego_peak_rad_s'],rms_rad_s=tracking['tracking']['tracking_rms_rad_s'],
        post_rms_rad_s=tracking['tracking']['post_rms_rad_s'],gyro_through_lend_deg=tracking['gyro_through_lend_deg'],
        minimum_battery_v=tracking['battery_min_v'],motion_observation_rate=geom['motion_observation_rate'],
        source_settings_sha256=digest(folder/'saved_1ms.settings.json')))
assert len(set(fingerprints))==1
for name in ('video_marker_pose.py','body_appearance_model.py','marker_detection.py'):
    assert digest(paths[1]/'analysis_source_used'/name)==digest(RUN/'analysis_source_used'/name)
comparison=RUN/'correction_comparison';comparison.mkdir(exist_ok=False)
save(comparison/'comparison.json',dict(at=now,mouse='B',motion='long_r90',preset_mm_s=2000,
    comparison_kind='different_angle_corrections_one_run_each',all_other_turn_settings_identical=True,
    cases=rows,monotonic_response_supported=False,
    previous_interpolation_prediction_deg=89.9508696605133,
    current_difference_from_prediction_deg=geometry['entry_to_exit_path_rotation_deg']-89.9508696605133,
    next_action='Repeat correction+1.0 once under current charged conditions; hold PID/FF/radius/Lstart/Lend. No new interpolation or length adoption.',
    limitations=['One run per correction, charging and measured entrance differed; not a controlled correction-only causal estimate.',
        'Measured-entry-relative marker midpoint; fixed-board placement and axle geometry not accepted.'],parameter_adoption_allowed=False))
save(RUN/'actual_comparison_settings.json',dict(all_other_turn_settings_identical=True,
    compared_fields=list(canonical),corrections=[row['correction_deg'] for row in rows],
    source_settings_sha256=[row['source_settings_sha256'] for row in rows]))
fig,axes=plt.subplots(1,3,figsize=(14,4.4),layout='constrained')
labels=['correction0','correction1.5','correction1.0 (charged)']
for index,(folder,row,label) in enumerate(zip(paths,rows,labels)):
    csv=folder/'geometry_review_v1'/f"{['before','correction1p5','correction1p0'][index]}_relative_trajectory.csv"
    if not csv.exists():
        candidates=list((folder/'geometry_review_v1').glob('*_relative_trajectory.csv'));assert len(candidates)==1;csv=candidates[0]
    data=pd.read_csv(csv);valid=data.detected
    axes[0].plot(data.loc[valid,'right_mm'],data.loc[valid,'forward_mm'],label=label)
axes[0].axhline(270,color='black',ls='--',lw=.7);axes[0].set(xlabel='Right [mm]',ylabel='Forward [mm]');axes[0].axis('equal');axes[0].legend(fontsize=7)
axes[1].bar(labels,[row['angle_deg'] for row in rows]);axes[1].axhline(90,color='black',ls='--');axes[1].set(ylim=(87,93),ylabel='Entry to exit path rotation [deg]')
axes[2].bar(labels,[row['exit_forward_offset_mm'] for row in rows]);axes[2].set(ylabel='Exit-line forward offset at right90 [mm]')
for ax in axes:ax.grid(alpha=.2);ax.tick_params(axis='x',labelsize=7)
fig.savefig(comparison/'comparison.png',dpi=160);plt.close(fig)

# A fresh request and exclusive output directory prevent execution replay.
NEXT.mkdir(exist_ok=False)
for name in ('candidate.settings.json','marker_reference.json','body_geometry_candidate.json','body_model_B_green_red.npz'):
    shutil.copyfile(PREP/name,NEXT/name)
body=read(NEXT/'body_geometry_candidate.json')
body['marker_detection']['body_model']['model_file']=str(NEXT/'body_model_B_green_red.npz')
(NEXT/'body_geometry_candidate.json').write_text(json.dumps(body,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
config=read(PREP/'config.candidate.json');config['reference']=str(NEXT/'marker_reference.json');config['geometry']=str(NEXT/'body_geometry_candidate.json')
save(NEXT/'config.candidate.json',config)
shutil.copytree(PREP/'analysis_source_snapshot',NEXT/'analysis_source_snapshot')
controller=ROOT/'tmp/run_turn2000_B_correction1p0_repeat01_once_20261120.py'
assert not controller.exists()
source=(ROOT/'tmp/run_turn2000_B_correction1p0_once_20261120.py').read_text(encoding='utf-8')
source=source.replace('next_correction_plus1p0_20261120_v1','next_correction_plus1p0_repeat01_20261120_v1').replace('B_R90_2000_correction1p0_20261120_001',IDENTIFIER)
source=source.replace('設定照合・短い通電静止検査・録画付き右90°','設定照合・録画付き右90°')
controller.write_text(source,encoding='utf-8')
ready=read(PREP/'ready_plan.json')
for name in ('pixel_camera_tuning.py','pixel_camera_tuning_cli.py','pixel_camera_turn_capture.py','myshell_debug.py'):
    source_path=ROOT/'tools'/name
    assert digest(source_path)==ready['source_sha256'][str(source_path)], 'Acquisition code changed; review before preparing another run: '+name
ready.update(at=now,request_id=IDENTIFIER,change={},source_comparison=str(comparison/'comparison.json'),
    evaluation_scope='Repeat correction+1.0 under current charged conditions, relative to measured entry. Absolute pose is not a quality gate.',
    operator_current_on_hands_clear_path_confirmation_required=True,execution_commands_sent=0)
guard_sources=[NEXT/name for name in ('config.candidate.json','body_geometry_candidate.json','body_model_B_green_red.npz','marker_reference.json','candidate.settings.json')]
guard_sources += [controller]+[ROOT/'tools'/name for name in ('pixel_camera_tuning.py','pixel_camera_tuning_cli.py','pixel_camera_turn_capture.py','myshell_debug.py')]
ready['source_sha256']={str(p):digest(p) for p in guard_sources}
ready['offline_analysis_sources']={str(p):digest(p) for p in (NEXT/'analysis_source_snapshot').glob('*.py')}
ready['post_charge_acquisition_review']=str(PREP/'post_charge_acquisition_review/review.json')
save(NEXT/'ready_plan.json',ready)
preparation=dict(at=now,state='PREPARED_NOT_SENT_NOT_EXECUTED',mouse='B',motion='long_r90',preset_mm_s=2000,
    request_id=IDENTIFIER,change={},chosen_correction_deg=1.0,source_comparison=str(comparison/'comparison.json'),
    reason='Current88.876deg differs from the prior two-point prediction89.951deg. One charged run cannot identify a new correction. Repeat before changing geometry.',
    hold_lengths=dict(lstart_mm=12.57,lend_mm=42.89),hold_angular_pid=dict(kp=.2,ki=.005,kd=0),
    settings_write_commands_sent=0,execution_commands_sent=0,parameter_adoption_allowed=False)
save(NEXT/'preparation.json',preparation)
save(RUN/'next_candidate_preparation.json',preparation)
shutil.copyfile(NEXT/'candidate.settings.json',RUN/'next_candidate.settings.json')
ekf=read(paths[1]/'ekf_input_review.json')
ekf['source_trial']=str(RUN/'trial.json')
control=read(PREP/'run_session01/controller_result.json')
ekf['turn_ekf_device_response']=control.get('turn_ekf')
for field,item in ekf['input_fields'].items():
    frame=pd.read_csv(RUN/'telemetry.csv');item['present']=field in frame
    item['finite']=bool(field in frame and np.isfinite(frame[field]).all())
save(RUN/'ekf_input_review.json',ekf)
save(RUN/'acquisition_validation.json',dict(capture_metadata_complete=True,metadata_recovery_required=False,
    source_review=str(PREP/'post_charge_acquisition_review/review.json'),
    host_test_command='D:\\venv\\venv3-11\\Scripts\\python.exe -m unittest discover -s tools/tests -p test_pixel_camera_tuning.py -v',
    host_tests_passed_before_capture=25,host_tests_skipped=0,
    saved_log_capture_phase=trial['saved_log_capture_phase'],hardware_execution_commands=1,execution_resends=0))
valid=pose.marker_detected_raw & pose.calibration_valid
review=dict(at=now,mouse='B',motion='long_r90',preset_mm_s=2000,correction_deg=1.0,executions=1,
    operator_stopped=True,contact=False,both_off=True,ble_disconnected=True,
    full_frames=len(pose),raw_observed=int(valid.sum()),registered=int(pose.calibration_valid.sum()),
    excluded_frames=pose.loc[~valid,'frame'].astype(int).tolist(),excluded_frames_interpolated_for_measurement=False,
    manual_samples=len(manual),manual_marker_identity_passed=True,manual_review='marker_identity_review_v2/review.json',
    path_rotation_deg=geometry['entry_to_exit_path_rotation_deg'],angle_error_deg=geometry['path_rotation_error_from90_deg'],
    entry_board_direction_deg=geometry['entry_board_heading_deg'],exit_board_direction_deg=geometry['exit_board_heading_deg'],
    exit_relative_forward_error_mm=geometry['exit_line_offset_from270_mm'],direction_window_sensitivity=sensitivity,
    relative_review_eligible=True,late_static_rms_mm=geometry['stationary_scatter_rms_mm'],
    actual_capture_stop_evidence=geometry['actual_capture_stop_evidence'],original_standard_auto_stop_confirmed=summary['auto_stop_confirmed'],
    metadata_recovery_required=False,analysis_source_manifest='analysis_source_used/manifest.json',
    next_correction_candidate=1.0,next_action='Same-condition repeat; no additional angle or length change',candidate_adoption_allowed=False)
save(RUN/'final_review.json',review)
deferred=read(RUN/'deferred_analysis.json');deferred.update(state='ANALYZED_REVIEWED',frame_strides=[1],
    final_review='final_review.json',parameter_adoption_allowed=False,at=now)
(RUN/'deferred_analysis.json').write_text(json.dumps(deferred,indent=2)+'\n',encoding='utf-8')
table='\n'.join(f"| {row['correction_deg']:+.1f}° | {row['angle_deg']:.3f}° | {row['error_deg']:+.3f}° | {row['exit_forward_offset_mm']:+.3f} mm | {row['minimum_battery_v']:.3f} V |" for row in rows)
text=f'''# 機体B turn2000 右90°：角度補正+1.0°の1回確認

充電後、補正+1.0°・Ki0.005で録画付き右90°を1回取得した。停止・接触なし・機体B／HY0020両方OFFを操作者が確認済み。BLE切断、実行再送0回。ファームウェア・Paramsへ反映していない。

| 補正 | 実測入口からの旋回量 | 90°との差 | 相対出口線の前方ずれ | 最低電圧 |
| --- | --- | --- | --- | --- |
{table}

[比較図](correction_comparison/comparison.png)、[比較JSON](correction_comparison/comparison.json)、[最終評価](final_review.json)。出口線は入口相対座標の右90 mmで、前方270 mmとの差。固定盤面の停止点合格とは扱わない。

今回は旋回量88.876°、回り不足1.124°。前の2点による予測89.951°とは約1.075°異なり、補正に対する単調な応答は3本では確認できなかった。充電前後の電圧、入口方向（今回盤面92.370°、前回90.278°）、独立試行の差を含む。原因を補正値だけに決めず、**+1.0°の同条件再確認**を次の1回とした。半径・Lstart12.57 mm・Lend42.89 mm・PID・FF・吸引650は保持する。時刻±5 msでの旋回量は88.828〜89.009°。

全1236コマを間引きなしで処理し、前後の実色観測・ArUco登録が同時成立した1234コマを使用した。2332／2333コマは登録が成立せず除外し、固定較正や補間で観測点を作っていない。必要動作窓の観測率98.82%は既存条件を満たす。検出器・モデルは前回と同じ保存済み版で、推定点のみ・CNN補正は0件。定間隔画像・旋回中の全28コマ・後半静止画像、計56枚を[正しい向きの画像](marker_identity_review_v2/sample_page1.png)で目視照合した。初回の表示用画像は回転指定が不適切だったため無効と明記し保持した。数値解析の画像向きは当初からraw指定で変更していない。

入口／出口の直線残差RMS0.413／0.183 mm、連続ログとの剛体合わせ全体1.748 mm・動作中4.284 mm。時刻合わせはモデル推定で独立同期ではなく、距離スケールは調整していない。後半静止197点の散らばりRMS0.196 mm、連続ログでは1414 ms／283点の停止を確認した。標準summaryのauto_stop_confirmed=falseも保持している。

保存ログ276件、欠落・CRC異常0件、対応行から求めた周期1.01542 ms。角速度ピーク40.938 rad/s、追従RMS5.707 rad/s、旋回後RMS3.284 rad/s、出力飽和0%。今回は停止確認・後録画の後、動画転送前に保持ログを保存した。保存メタデータは完全で、復元は不要。取得前に関連ホストテスト25件が成功、スキップ0件。解析区間は後録画完了の12.403147 sまでで、追加のRAM読出し中の静止尾部だけを除いた。

実測入口を基準にした見掛けのマーカー中点の相対形状である。絶対機軸位置・高さ補正済みyaw・B2000のEKF滑り係数は未校正。Aの係数を流用していない。同条件の反復・連続接続も未確認で、候補は未採用。未送信の次候補・準備根拠をこのフォルダーにも保存した。
'''
(RUN/'README.md').write_text(text,encoding='utf-8')
state=read(BASE/'resume_state.json')
state.update(at=now,state='CORRECTION_PLUS1P0_REVIEWED_REPEAT_PREPARED_BOTH_OFF',power_state='BOTH_OFF_USER_CONFIRMED',
    execution_pending=False,ble_disconnected=True,last_trial_json=str((RUN/'trial.json').relative_to(ROOT)),
    latest_relative_path_rotation_deg=geometry['entry_to_exit_path_rotation_deg'],latest_relative_exit_error_mm=geometry['exit_line_offset_from270_mm'],
    latest_final_review=str((RUN/'final_review.json').relative_to(ROOT)),next_candidate=str((NEXT/'candidate.settings.json').relative_to(ROOT)),
    next_ready_plan=str((NEXT/'ready_plan.json').relative_to(ROOT)),next_candidate_preparation=str((NEXT/'preparation.json').relative_to(ROOT)),
    next_controller_script=str(controller.relative_to(ROOT)),next_cli_session=str((NEXT/'run_session01').relative_to(ROOT)),
    next_request_id=IDENTIFIER,next_requested_change={},
    next_action='Operator returns the robot while OFF, then confirms ON/hands clear/path for a fresh single correction+1.0 repeat. Read voltage/settings before execution.',
    current_placement_confirmed=False,current_hands_clear_confirmed=False,
    latest_user_steering='After charged correction+1.0: stopped, no contact, both OFF confirmed.',
    repeatability='Correction0,+1.5,+1.0 each one current relative run. Charged+1.0 needs repeat before another correction.',
    charging_completion_confirmed=True,parameter_adoption_allowed=False)
(BASE/'resume_state.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(review=review,next_preparation=str(NEXT),next_controller=str(controller)),ensure_ascii=True),flush=True)
