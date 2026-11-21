"""補償1.0度の初回を品質確認し、基準3本とは別群で比較する。機器操作なし。"""
from pathlib import Path
import hashlib
import json
import shutil
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from pixel_camera_tuning import setting_differences

BASE = ROOT/'tools/logs/pixel_camera/turn1600_B_r90_20261120_v1'
RUN = BASE/'cli_session08/run_B_R90_1600_Lstart13p6_corr1p0_green_20261120_001'
REVIEW = BASE/'green_correction1p0_001_video_review_v1'
BASELINE = BASE/'green_Lstart13p6_three_review_v1'


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def new(path,value):
    with path.open('x',encoding='utf-8') as output:
        json.dump(value,output,ensure_ascii=False,indent=2,allow_nan=False)
        output.write('\n')


assert not (RUN/'archive.json').exists()
assert read(RUN/'operator_postrun_confirmation.json')['both_off']
result = read(REVIEW/'video_comparison.json')['comparisons'][0]
baseline = read(BASELINE/'video_comparison.json')['comparisons']
assert len(baseline)==3 and result['relative_review_eligible']
report = read(RUN/'full/report.json')
summary = read(RUN/'full/summary.json')
tracking = read(RUN/'tracking_1ms_review_v2/verification.json')
preflight = read(RUN/'pre_run_green_static_validation.json')
actual_path = BASE/'device_prepare_green_corr1p0_01/actual.settings.json'
actual = read(actual_path)
previous = read(BASE/'device_prepare_green_Lstart13p6_05/actual.settings.json')
assert setting_differences(previous,actual,quantized=True)==['parameters.correction']
assert actual['parameters']['correction']==1.0
assert not read(RUN/'body_geometry_used.json')['marker_detection'].get('body_model')
prepared = read(BASE/'preparation.json')['source_hashes']
checks = {name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==prepared[name.replace('/','\\')]
    for name in ('Params/Param_B/turn_1600/turn_1600.cpp','Params/Param_B/run_param_B.cpp',
                 'Params/mouse_config.h','System/Src/myshell.cpp')}
assert all(checks.values())

# 基準3本と候補1本を同一設定の反復として混ぜない。
angles = np.array([case['entry_to_exit_path_rotation_deg'] for case in baseline])
offsets = np.array([case['exit_line_offset_from270_mm'] for case in baseline])
contrast = dict(mouse='B',motion='long_r90',preset_mm_s=1600,
    evaluation_frame='Relative marker midpoint and measured motion-entry direction; absolute axle/fixed-board targets unverified.',
    baseline_count=3,candidate_count=1,groups_pooled=False,
    changed_fields={'parameters.correction':{'from_deg':1.5,'to_deg':1.0}},
    all_other_numeric_settings_unchanged=True,
    baseline_rotation_deg=dict(mean=float(angles.mean()),median=float(np.median(angles)),minimum=float(angles.min()),maximum=float(angles.max())),
    baseline_exit_offset_mm=dict(mean=float(offsets.mean()),median=float(np.median(offsets)),minimum=float(offsets.min()),maximum=float(offsets.max())),
    candidate_rotation_deg=result['entry_to_exit_path_rotation_deg'],
    candidate_exit_offset_mm=result['exit_line_offset_from270_mm'],
    rotation_delta_from_baseline_mean_deg=float(result['entry_to_exit_path_rotation_deg']-angles.mean()),
    rotation_delta_from_baseline_median_deg=float(result['entry_to_exit_path_rotation_deg']-np.median(angles)),
    candidate_entry_board_heading_deg=result['entry_board_heading_deg'],
    candidate_exit_board_heading_deg=result['exit_board_heading_deg'],
    candidate_battery_min_v=tracking['battery_min_v'],
    candidate_repeatability_established=False,improvement_verified=False,parameter_adoption_allowed=False,
    interpretation='候補1本と基準3本の観測比較。基準の角度範囲は約1.17度で、単発から0.5度変更の効果を確定しない。入口姿勢と電圧差も残る。',
    next_action='角度補正1.0度・長さ13.6/26.31mmを維持して候補2本目を取得。',
    inputs={str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest() for path in
        [BASELINE/'video_comparison.json',REVIEW/'video_comparison.json',actual_path]})
new(REVIEW/'baseline3_vs_correction1p0_001.json',contrast)
fig,axes = plt.subplots(1,3,figsize=(14,4),layout='constrained')
for case in baseline:
    source = BASELINE/(case['label']+'_relative_trajectory.csv')
    shutil.copy2(source,REVIEW/source.name)
    path = pd.read_csv(source)
    valid = path.detected.to_numpy(bool)
    axes[0].plot(path.right_mm[valid],path.forward_mm[valid],color='gray',alpha=.6,lw=1,label='correction1.5 baseline' if case==baseline[0] else None)
path = pd.read_csv(REVIEW/'corr1p0_001_relative_trajectory.csv')
valid = path.detected.to_numpy(bool)
axes[0].plot(path.right_mm[valid],path.forward_mm[valid],label='correction1.0 trial1')
axes[0].axhline(270,color='black',lw=.7,ls='--')
axes[0].set(xlabel='Right [mm]',ylabel='Forward [mm]')
axes[0].axis('equal')
axes[0].legend(fontsize=8)
for ax,values,current,target,ylabel in [
    (axes[1],angles,result['entry_to_exit_path_rotation_deg'],90,'Path rotation [deg]'),
    (axes[2],offsets,result['exit_line_offset_from270_mm'],0,'Exit-line offset [mm]')]:
    ax.scatter(np.zeros(3),values,color='gray',label='3 baseline runs')
    ax.scatter(1,current,label='1 candidate run')
    ax.axhline(target,color='black',lw=.7,ls='--')
    ax.set(xticks=[0,1],xticklabels=['correction1.5','correction1.0'],ylabel=ylabel,xlim=(-.4,1.4))
for ax in axes:
    ax.grid(alpha=.25)
fig.savefig(REVIEW/'baseline3_vs_correction1p0_001.png',dpi=160)
plt.close(fig)

qa = dict(mouse='B',motion='long_r90',preset_mm_s=1600,Lstart_mm=13.6,Lend_mm=26.31,
    correction_deg=1.0,suction=tracking['suction'],front_colour='green',rear_colour='red',
    full_evaluated_frames=report['quality']['frames'],valid_frames=report['quality']['valid_frames'],
    per_frame_registration_fraction=report['quality']['marker_calibration']['calibration_frame_rate'],
    body_detection_given_calibration=report['quality']['body_detection_rate_given_calibration'],
    motion_observation_fraction=result['motion_observation_rate'],
    fixed8_actual_round_marker_identity_and_LED_on_off_manually_confirmed=True,
    strict_start_static_alignment_passed=preflight['strict_static_alignment_passed'],
    strict_static_failure_reasons=preflight['static_check']['failure_reasons'],
    static_marker_yaw_not_used_as_motion_entry_axis=True,
    original_auto_stop_confirmed=summary['auto_stop_confirmed'],original_summary_preserved=True,
    original_false_stop_reason='motion.csvが短く、撮影終了時の長い静止区間を含まない。別の連続ログ停止証拠と遅い動画静止窓を評価。',
    capture_stop_evidence=result['actual_capture_stop_evidence'],
    late_static_observations=result['stationary_observations'],late_static_scatter_rms_mm=result['stationary_scatter_rms_mm'],
    operator_stopped_no_contact_both_off=True,ble_closed=True,relative_shape_review_eligible=True,
    fixed_board_trajectory_accepted=False,absolute_axle_position_verified=False,
    relative_exit_rotation_deg=result['entry_to_exit_path_rotation_deg'],
    relative_exit_line_offset_mm=result['exit_line_offset_from270_mm'],
    correction1p0_group_count=1,baseline_correction1p5_count=3,repeatability_established=False,
    baseline_comparison=contrast,body_model_enabled=False,LED_independent_sync_used=False,
    EKF_identification='B用の幾何校正とツール対応が不足。AのK・雑音を流用せず、取得時のモードを維持。',
    firmware_sources_unchanged=checks,source_params_changed=False,firmware_write_performed=False,
    parameter_adoption_allowed=False,execution_commands_sent=1,execution_resend_count=0)
new(RUN/'manual_quality_review.json',qa)
new(REVIEW/'manual_quality_review.json',qa)
shutil.copytree(REVIEW,RUN/'video_shape_review_v1')
source_folder = RUN/'analysis_sources'
source_folder.mkdir()
source_paths = [Path(__file__),ROOT/'tmp/review_turn1600_B_video_20261120.py',ROOT/'tools/analysis/video_marker_pose.py',ROOT/'tools/marker_detection.py']
for source in source_paths:
    shutil.copy2(source,source_folder/source.name)
new(source_folder/'source_hashes.json',{str(source.relative_to(ROOT)):hashlib.sha256(source.read_bytes()).hexdigest() for source in source_paths})
deferred = read(RUN/'deferred_analysis.json')
deferred.update(state='ANALYZED_REVIEWED_NOT_ADOPTED',quality_review='manual_quality_review.json',
    archive_completion_receipt='archive.json',both_power_off_user_confirmed=True,source_params_changed=False)
(RUN/'deferred_analysis.json').write_text(json.dumps(deferred,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
state = read(BASE/'resume_state.json')
state.update(state='CORRECTION1P0_001_REVIEWED_BOTH_OFF_READY_FOR_ARCHIVE',power_state='BOTH_OFF_USER_CONFIRMED',
    last_quality_review=str(RUN/'manual_quality_review.json'),last_video_review=str(REVIEW/'video_comparison.json'),
    last_relative_exit_rotation_deg=result['entry_to_exit_path_rotation_deg'],
    last_relative_exit_forward_offset_mm=result['exit_line_offset_from270_mm'],
    relative_exit_rotation_deg=result['entry_to_exit_path_rotation_deg'],
    relative_exit_forward_offset_mm=result['exit_line_offset_from270_mm'],
    last_battery_min_v=tracking['battery_min_v'],same_setting_repeat_count=1,baseline_same_setting_repeat_count=3,
    same_setting_repeat_count_note='角度補正1.0度群の初回。補正1.5度基準3本と別群。',
    next_candidate_group_run_count=1,next_candidate=str(actual_path),next_candidate_applied=False,
    current_RAM_candidate_applied=False,next_candidate_change=None,
    next_static_tag='green_correction1p0_repeat02_placed01',
    next_unsent_request_id='B_R90_1600_Lstart13p6_corr1p0_green_20261120_002',next_request_sent=False,
    execution_pending=False,pending_request_id=None,pending_request_sent=False,archive_verified=False,last_archive_verified=False,
    next_action='一式保存後、両方OFFで同じ登録開始点315/45mm・緑前が下向きに戻した返答を待つ。次は同じ角度補正1.0度の2本目。ONはまだ依頼しない。')
(BASE/'resume_state.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(finalized=True,candidate_count=1,baseline_count=3,
    candidate_angle_deg=result['entry_to_exit_path_rotation_deg'],candidate_exit_offset_mm=result['exit_line_offset_from270_mm'],
    baseline_angle_median_deg=float(np.median(angles)),improvement_verified=False,both_off=True),ensure_ascii=False))
