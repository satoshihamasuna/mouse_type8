"""B右90度・補正1.0度の3本目を根拠付きで保存する。機器操作なし。"""
from pathlib import Path
import hashlib
import json
import shutil
import sys

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_r90_20261120_v1'
RUN=BASE/'cli_session10/run_B_R90_1600_Lstart13p6_corr1p0_green_20261120_003'
REVIEW=BASE/'green_correction1p0_three_video_review_v1'
sys.path.insert(0,str(ROOT/'tools'))
from pixel_camera_tuning import setting_differences
from pixel_camera_archive import archive_trial,verify_archive,write_index
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def save(path,value):
    with path.open('x',encoding='utf-8') as output:
        json.dump(value,output,ensure_ascii=False,indent=2,allow_nan=False)
        output.write('\n')


assert not (RUN/'archive.json').exists()
assert read(RUN/'operator_postrun_confirmation.json')['both_off']
assert read(BASE/'device_prepare_green_corr1p0_04/handoff_closure.json')['ble_disconnected']
report=read(RUN/'full/report.json')
summary=read(RUN/'full/summary.json')
tracking=read(RUN/'tracking_1ms_review_v2/verification.json')
pose=pd.read_csv(RUN/'full/apparent_floor_marker_pose.csv')
excluded=pose.loc[~(pose.marker_detected_raw & pose.calibration_valid),
    ['frame','time_video_s','marker_detected_raw','calibration_valid']].to_dict('records')
comparison=read(REVIEW/'video_comparison.json')
assert comparison['repeat_count']==3 and comparison['all_relative_reviews_eligible']
current=comparison['comparisons'][2]
assert current['label']=='corr1p0_003'
assert not setting_differences(read(RUN/'saved_1ms.settings.json'),read(BASE/'device_prepare_green_corr1p0_01/actual.settings.json'),quantized=True)
prepared=read(BASE/'preparation.json')['source_hashes']
source_names=('Params/Param_B/turn_1600/turn_1600.cpp','Params/Param_B/run_param_B.cpp','Params/mouse_config.h','System/Src/myshell.cpp')
checks={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==prepared[name.replace('/','\\')] for name in source_names}
assert all(checks.values())
baseline=read(BASE/'green_Lstart13p6_three_review_v1/video_comparison.json')
assert baseline['repeat_count']==3 and baseline['all_relative_reviews_eligible']
numeric_differences=[]
profile_keys=('front_colour','rear_colour','marker_detection','nominal_marker_span_mm','marker_span_limits_mm')
for group in (baseline,comparison):
    for case in group['comparisons']:
        folder=Path(case['source_trial']).parent
        value=read(folder/'saved_1ms.settings.json')
        differences=setting_differences(value,read(RUN/'saved_1ms.settings.json'),quantized=True)
        expected=['parameters.correction'] if group is baseline else []
        assert differences==expected
        profile=read(folder/'full/body_geometry_used.json')
        current_profile=read(RUN/'full/body_geometry_used.json')
        assert all(profile.get(key)==current_profile.get(key) for key in profile_keys)
        numeric_differences.append(dict(trial=case['source_trial'],changed_fields=differences))
contrast=dict(mouse='B',motion='long_r90',baseline_correction_deg=1.5,candidate_correction_deg=1.,
    baseline_count=3,candidate_count=3,groups_pooled=False,numeric_differences=numeric_differences,
    baseline_statistics=baseline['repeat_statistics'],candidate_statistics=comparison['repeat_statistics'],
    rotation_mean_delta_deg=comparison['repeat_statistics']['entry_to_exit_path_rotation_deg']['mean']-baseline['repeat_statistics']['entry_to_exit_path_rotation_deg']['mean'],
    exit_offset_mean_delta_mm=comparison['repeat_statistics']['exit_line_offset_from270_mm']['mean']-baseline['repeat_statistics']['exit_line_offset_from270_mm']['mean'],
    battery_and_initial_conditions_randomized=False,causal_improvement_verified=False,
    parameter_adoption_allowed=False,source_params_changed=False,
    evaluation_frame='relative to each measured entry line; marker midpoint, not absolute axle',
    interpretation='各設定3本の観測比較。電圧・開始条件が異なるため補正0.5度変更の効果を単独で断定しない。')
save(REVIEW/'baseline3_vs_correction1p0_3.json',contrast)
fig,axes=plt.subplots(1,2,figsize=(9,4),layout='constrained')
for column,key,target,label in [(0,'entry_to_exit_path_rotation_deg',90.,'Entry-to-exit rotation [deg]'),(1,'exit_line_offset_from270_mm',0.,'Exit-line offset [mm]')]:
    for position,group in enumerate((baseline,comparison)):
        values=[case[key] for case in group['comparisons']]
        axes[column].scatter(np.full(3,position),values)
        axes[column].plot([position-.12,position+.12],[np.mean(values)]*2,color='black',lw=2)
    axes[column].axhline(target,color='gray',ls='--',lw=.8)
    axes[column].set(xticks=[0,1],xticklabels=['correction1.5 (n=3)','correction1.0 (n=3)'],ylabel=label,xlim=(-.4,1.4))
    axes[column].grid(alpha=.25)
fig.savefig(REVIEW/'baseline3_vs_correction1p0_3.png',dpi=150)
plt.close(fig)
text=['# 機体B・右90°・1600 mm/s 補正1.0度の3本比較','',
    'Lstart 13.6 mm / Lend 26.31 mm / 半径 −52 mm / 吸引duty650。PID・FFは実機読返しで同一を確認。',
    '開始は右上付近・前緑がおおむね下向き。実際の入口直線を前方軸にして比較し、厳密な初期位置・角度合わせを要求しない。',
    '','| 試行 | 入口→出口角度 [deg] | 出口線ずれ [mm] | 最低電圧 [V] |','| --- | ---: | ---: | ---: |']
for index,case in enumerate(comparison['comparisons'],1):
    folder=Path(case['source_trial']).parent
    voltage=read(folder/'tracking_1ms_review_v2/verification.json')['battery_min_v']
    text.append(f"| {index} | {case['entry_to_exit_path_rotation_deg']:.3f} | {case['exit_line_offset_from270_mm']:+.3f} | {voltage:.3f} |")
text+=['',f"角度平均 {comparison['repeat_statistics']['entry_to_exit_path_rotation_deg']['mean']:.3f}°、出口線ずれ平均 {comparison['repeat_statistics']['exit_line_offset_from270_mm']['mean']:+.3f} mm。",'',
    '[3本の相対軌跡](video_path_comparison.png) / [補正1.5度群との比較](baseline3_vs_correction1p0_3.png) / [数値・入力ハッシュ](video_comparison.json)',
    '', '今回の3本目は1回のみ実行し、332点の1 msログ・動画・実機設定を保存。操作者が停止・接触なし・両方OFFを確認。',
    f"3本目の評価フレーム {report['quality']['valid_frames']}/{report['quality']['frames']}、動作区間観測率 {current['motion_observation_rate']*100:.2f}%。欠測3フレームは補完せず除外。各フレームの実ArUco登録と生検出を検査し、片側推定点・学習検出器は使っていない。",
    '元summaryの自動停止未成立は保持し、録画終了判断時の停止ログ、後半の動画静止窓、操作者確認を別に記録する。',
    '動画同期は軌跡当てはめによる推定。カメラ高さは概算で、盤面固定目標・絶対車軸位置・連続接続の合格を意味しない。',
    '補正1.5度群とは別集計。電圧・開始条件の差があり、補正値変更だけの因果効果は未確定。ファームウェアへ未反映。',
    '機体BのEKF独立同定は幾何校正とB対応ツールが不足しており未実施。Aの係数は流用していない。','']
with (REVIEW/'README.md').open('x',encoding='utf-8') as output:output.write('\n'.join(text))
quality=dict(mouse='B',motion='long_r90',preset_mm_s=1600,correction_deg=1.,Lstart_mm=13.6,Lend_mm=26.31,
    full_evaluated_frames=report['quality']['frames'],valid_frames=report['quality']['valid_frames'],
    motion_observation_fraction=current['motion_observation_rate'],motion_calibration_fraction=current['motion_calibration_rate'],
    detector=report['quality'].get('detector'),detection_profile_changed=False,body_model_enabled=False,
    fixed8_actual_front_green_rear_red_identity_manually_confirmed=True,
    manual_identity_evidence='marker_identity_fixed8.jpg',
    placement_mode='relative_to_measured_motion_entry_direction',exact_start_target_required=False,
    static_full_frame_alignment_claimed=False,preflight='pre_run_relative_start_review.json',
    original_auto_stop_confirmed=summary['auto_stop_confirmed'],original_summary_preserved=True,
    capture_stop_evidence=current['actual_capture_stop_evidence'],
    end_capture_stop_poll_evidence=read(RUN/'trial.json')['log_stop_evidence'],
    excluded_frames=excluded,excluded_frames_interpolated_for_measurement=False,
    late_static_observations=current['stationary_observations'],late_static_scatter_rms_mm=current['stationary_scatter_rms_mm'],
    relative_shape_review_eligible=current['relative_review_eligible'],
    relative_exit_rotation_deg=current['entry_to_exit_path_rotation_deg'],
    relative_exit_line_offset_mm=current['exit_line_offset_from270_mm'],
    correction1p0_group_count=3,baseline_correction1p5_count=3,groups_pooled=False,
    identical_numeric_settings_with_candidate_trial1_verified=True,
    repeatability_target_count=3,repeatability_target_reached=True,
    battery_min_v=tracking['battery_min_v'],operator_stopped_no_contact_both_off=True,ble_closed=True,
    source_params_changed=False,firmware_write_performed=False,firmware_sources_unchanged=checks,
    parameter_adoption_allowed=False,execution_commands_sent=1,execution_resend_count=0,
    fixed_board_alignment_accepted=False,absolute_axle_position_verified=False,
    EKF_identification='B geometry calibration and B tool support remain insufficient; no A coefficients reused.',
    next_action='Three identical-setting trials reviewed. Keep both OFF; no further execution pending.')
save(RUN/'manual_quality_review.json',quality)
save(REVIEW/'repeat03_manual_quality_review.json',quality)
shutil.copytree(REVIEW,RUN/'video_shape_review_v1')
sources=RUN/'analysis_sources'
sources.mkdir()
names=('tmp/finalize_B_corr1p0_repeat03_20261120.py','tmp/postprocess_B_corr1p0_repeat03_20261120.py',
    'tmp/review_B_relative03_marker_identity_20261120.py','tmp/resume_B_r90_relative_preflight_repeat03_20261120.py',
    'tmp/review_turn1600_B_video_20261120.py','tools/analysis/video_marker_pose.py',
    'tools/analysis/pixel_camera_relative_pose.py','tools/analysis/pixel_camera_tuning_report.py',
    'tools/marker_detection.py','tools/pixel_camera_detection.py')
for name in names:
    shutil.copy2(ROOT/name,sources/Path(name).name)
save(sources/'source_hashes.json',{name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in names})
pending=read(RUN/'deferred_analysis.json')
pending.update(state='ANALYZED_REVIEWED_NOT_ADOPTED',quality_review='manual_quality_review.json',
    both_power_off_user_confirmed=True,archive_completion_receipt='archive.json')
(RUN/'deferred_analysis.json').write_text(json.dumps(pending,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
archive=archive_trial(RUN/'trial.json')
manifest=verify_archive(archive)
write_index(archive.parent)
state=read(BASE/'resume_state.json')
state.update(state='CORRECTION1P0_REPEAT03_REVIEWED_ARCHIVED_BOTH_OFF',power_state='BOTH_OFF_USER_CONFIRMED',
    last_quality_review=str(RUN/'manual_quality_review.json'),last_video_review=str(REVIEW/'video_comparison.json'),
    last_relative_exit_rotation_deg=current['entry_to_exit_path_rotation_deg'],relative_exit_rotation_deg=current['entry_to_exit_path_rotation_deg'],
    last_relative_exit_forward_offset_mm=current['exit_line_offset_from270_mm'],relative_exit_forward_offset_mm=current['exit_line_offset_from270_mm'],
    last_battery_min_v=tracking['battery_min_v'],same_setting_repeat_count=3,next_candidate_group_run_count=3,
    baseline_same_setting_repeat_count=3,exact_start_target_required=False,firmware_source_hashes_unchanged=checks,
    comparison_summary=str(REVIEW/'README.md'),baseline_vs_candidate_review=str(REVIEW/'baseline3_vs_correction1p0_3.json'),
    next_request_sent=False,next_unsent_request_id=None,
    tuning_stage='relative_single_turn_candidate_three_runs; firmware and continuous connection not validated',
    operator_power_confirmation='User confirmed stopped/no contact/both OFF after correction1.0_003',
    same_setting_repeat_count_note='補正1.0度群3本。補正1.5度基準3本とは別群。',
    archive_folder=str(archive),last_archive_folder=str(archive),archive_verified=True,last_archive_verified=True,
    next_action='Three-run comparison complete. Keep both OFF. No precise initial alignment and no further execution pending; retain current candidate for review.',
    next_static_tag='green_corr1p0_future_relative01',execution_pending=False,pending_request_id=None,pending_request_sent=False)
(BASE/'resume_state.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(archived=str(archive),file_count=len(manifest['files']),verified=True,
    angle_deg=current['entry_to_exit_path_rotation_deg'],exit_offset_mm=current['exit_line_offset_from270_mm'],
    candidate_count=3,parameter_adoption_allowed=False),ensure_ascii=False))
