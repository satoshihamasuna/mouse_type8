"""前後色検出候補とLendだけの次候補を保存する。機器操作なし。"""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json
import shutil
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tools/logs/pixel_camera/turn1800_A_r180_20261120_v1'
stage = BASE / 'radius47p6_c1p5_s15_01'
run = stage / 'local_execution/run_A_R180_1800_radius47p6_c1p5_s15_20261120_001'
out = stage / 'rear_visibility_review_v1'
read = lambda path: json.loads(path.read_text(encoding='utf-8-sig'))
current = read(out / 'both15_current_midpoint/report.json')
previous = read(out / 'both15_previous_charged_midpoint/report.json')
old_current = read(stage / 'entry_clock_midpoint_review_v1/report.json')
old_previous = read(BASE / 'radius46p8_c2p0_03/marker_visibility_review_v1/s15_previous_charged_midpoint/report.json')
assert all(r['relative_geometry_eligible'] and all(r['checks'].values()) for r in (current, previous))
vector = lambda r: np.array([r['return_spacing_at_forward45_mm'], r['path_angle_deg'], *r['lend_right_forward_mm']])
runtime = read(stage / 'runtime_command_review.json')
raw = pd.read_csv(run / 'saved_1ms.csv')
turn_rows = raw.index[raw['motion.pattern'] == 5]
resets = raw.index[(raw['ego.length'].diff() < -20) & (raw.index >= int(runtime.get('start_row', 100))) & (raw.index < turn_rows[-1])]
reset = int(resets[-1])
completion = int(turn_rows[-1]) + 1
assert raw['motion.pattern'].iloc[completion] == 30
# 実装は完了時に超過分を残す。角度区間のlengthリセットを跳躍と混同しない。
counter_at_completion = 31.5 + float(raw['ego.length'].iloc[completion])
counter_per_tick = counter_at_completion / (completion - reset)
velocity = (np.array(current['lend_sensitivity_plus1ms_mm']) - current['lend_sensitivity_minus1ms_mm']) / .002
tick_seconds = current['timing']['measured_tick_ms'] / 1000
gain = -velocity[1] * tick_seconds / counter_per_tick
extra = (current['lend_right_forward_mm'][1] - 90) / gain
next_lend = round((31.5 + extra) * 2) / 2
assert next_lend == 40.5
delta_time = (next_lend - 31.5) / counter_per_tick * tick_seconds
prediction = np.array(current['lend_right_forward_mm']) + velocity * delta_time
refined = dict(runtime, post_boundary_reset_count=reset, completion_count=completion,
    counter_at_completion_before_rebase_mm=counter_at_completion, counter_increment_mean_mm_per_tick=counter_per_tick,
    raw_tail_negative_step_is_angular_phase_length_reset=True,
    provisional_forward_change_per_extra_lend_mm=float(gain), estimated_extra_lend_mm=float(extra),
    candidate_lend_mm=next_lend, predicted_lend_right_forward_mm=prediction.tolist(),
    prediction_is_measurement=False, source_runtime_review=str(stage / 'runtime_command_review.json'))
with (out / 'runtime_command_refined_review.json').open('x', encoding='utf-8') as f:
    json.dump(refined, f, ensure_ascii=False, indent=2)
geometry = out / 'body_geometry_A_red15_yellow15_candidate.json'
next_stages = [BASE / f'lend40p5_r47p6_c1p5_both15_{i:02d}' for i in range(1, 4)]
for next_stage in next_stages:
    target = next_stage / geometry.name
    assert not target.exists()
    shutil.copyfile(geometry, target)
    config = read(next_stage / 'config.json')
    config['geometry'] = str(target)
    (next_stage / 'config.json').write_text(json.dumps(config, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    with (next_stage / 'detector_candidate_setup.json').open('x', encoding='utf-8') as f:
        json.dump(dict(status='PROSPECTIVE_BOTH_S15_DETECTOR_CANDIDATE', geometry=str(target),
            original_profiles_preserved=True, red_minimum_saturation=15, yellow_minimum_saturation=15,
            physical_geometry_changed=False, camera_settings_changed=False,
            inferred_points_used=False, statistical_quality_thresholds_changed=False), f, ensure_ascii=False, indent=2)
proposal = dict(mouse='A', motion='long_r180', status='LEND_ONLY_CANDIDATE_NOT_EXECUTED',
    source_review=str(out / 'both15_current_midpoint/report.json'), source_runtime=str(out / 'runtime_command_refined_review.json'),
    physical_parameter_changes=dict(lend=dict(before=31.5, after=40.5)),
    unchanged_parameters=dict(r_min=-47.6, correction=1.5, lstart=7.52, velo=1.8, pre_accel=6.5),
    pid_ff_suction_unchanged=True, charged_same_type_battery_required=True,
    entry_radius_compensation_unchanged=True,
    source_effective_command_radius_estimate_mm=runtime['inferred_effective_command_r_min_mm'],
    predicted_lend_right_forward_mm=prediction.tolist(), prediction_is_measurement=False,
    prediction_limitations='Lend付近の画像速度と実機lengthカウンタからの局所推定。電池・進入補正・機体状態が変わるため独立再検証が必要。',
    new_operator_ready_required=True, source_params_changed=False, firmware_changed=False,
    parameter_adoption_allowed=False, fixed_board_accepted=False,
    working_candidate_targets=dict(width_at_forward45_mm=90., width_tolerance_mm=2., path_angle_deg=180., path_tolerance_deg=1.,
        nominal_lend_right_forward_mm=[90., 90.], lend_right_tolerance_mm=2., lend_forward_tolerance_mm=3.))
with (next_stages[0] / 'geometry_proposal.json').open('x', encoding='utf-8') as f:
    json.dump(proposal, f, ensure_ascii=False, indent=2)
comparison = dict(recorded_at_utc=datetime.now(timezone.utc).isoformat(), status='BOTH_S15_RELATIVE_CANDIDATE_REVIEWED_NOT_ADOPTED',
    current_geometry=vector(current).tolist(), previous_geometry_same_detector=vector(previous).tolist(),
    current_change_from_original_detector=(vector(current) - vector(old_current)).tolist(),
    previous_change_from_red15_yellow30=(vector(previous) - vector(old_previous)).tolist(),
    current_required_observation_rate=current['required_observation_rate'], current_motion_observation_rate=current['motion_observation_rate'],
    previous_required_observation_rate=previous['required_observation_rate'], previous_motion_observation_rate=previous['motion_observation_rate'],
    original_rejected_trial_and_analysis_preserved=True, actual_registered_aruco_required=True,
    inferred_observations=False, red_clock_reused_because_red_configuration_and_height_unchanged=True,
    formal_false_positive_rate_verified=False, fixed_board_accepted=False, parameter_adoption_allowed=False,
    source_params_changed=False, firmware_changed=False, both_powered_off_confirmed=True,
    next_stage=str(next_stages[0]), next_lend_mm=40.5, next_prediction=prediction.tolist(), next_execution_pending=False)
with (out / 'candidate_review.json').open('x', encoding='utf-8') as f:
    json.dump(comparison, f, ensure_ascii=False, indent=2)
text = f'''# 機体A・1800 R180：前後マーカーの観測候補とLend候補

元の47.6 mm・補正+1.5度・Lstart7.52 mm・Lend31.5 mmの試走は、純走行窓の前後マーカー観測率93.48%で採用不可として保存済み。元の資料・解析・不合格は変更していない。
今回の画像では後方黄色も白っぽく映り、彩度30で色領域が消えるフレームがあった。赤15は保持し、黄色の彩度下限だけ30から15へ変更した候補を今回と直前の比較用動画へ適用した。色相・面積・モルフォロジー・前後の組選択・実ArUco・95%採用条件は変更していない。
緑十字を重ねた欠測例で、新たに観測した重心が実際の後方黄色い円上にあることを目視確認した。推定点の補完はしていない。正式な偽陽性率の検証は未実施なので、恒久採用ではなく次の独立試走用の検出候補とする。
赤の検出条件・高さは変わらないため、入口時計の赤観測は同じ動画の保存済み結果を再利用した。

| 入口基準・実観測マーカー中点 | 今回 |
| --- | ---: |
| 返り幅 [mm] | {vector(current)[0]:.3f} |
| 返り角 [deg] | {vector(current)[1]:.3f} |
| Lend右 [mm] | {vector(current)[2]:.3f} |
| Lend前 [mm] | {vector(current)[3]:.3f} |
| 必要窓・純走行窓の観測率 [%] | {current['required_observation_rate']*100:.2f} / {current['motion_observation_rate']*100:.2f} |

今回と比較用動画の全フレームで実観測を確認し、個別の入口時計・直線・静止の条件も通過。比較用動画での黄色変更差（幅/角/右/前）は{comparison['previous_change_from_red15_yellow30']}。
標準全軌跡モデルはREVIEW_REQUIREDのままで、入口時計だけを使う相対診断と区別する。カメラ足位置・絶対車軸・body yaw・盤面固定接続・連続最短接続・EKF係数の採用は未確認。

## 実行時の半径補正

`Task/Src/init_motion.cpp`のR180には進入時`ego.x_point`による半径補正がある。今回の設定-47.6 mm、進入推定x={runtime['entry_estimated_x_mm']:.3f} mmから、指令上の半径は{runtime['inferred_effective_command_r_min_mm']:.3f} mmと推定される。
この式からの角速度ピーク{runtime['planned_peak_from_source_rad_s']:.4f} rad/sは保存された指令ピーク{runtime['observed_ideal_peak_rad_s']:.4f} rad/sと整合する。実機から実行半径を直接読み出した値でも、物理的な曲率半径でもない。ファームウェアや補正処理は変更していない。

## 次の一回

幅・角度は作業目標内。Lend前位置だけ目標90 mmより約{vector(current)[3]-90:.1f} mm先に残るため、半径47.6 mm・補正+1.5度・Lstart7.52 mmを保持し、Lendだけ31.5から40.5 mmへ延ばす。
Lend付近の画像速度と、長さの角度区間リセットを除いた実機カウンタから、変更後Lendを右{prediction[0]:.2f} mm・前{prediction[1]:.2f} mmと予測する。これは予測であり測定値ではない。進入補正や電池条件の影響を含めて再検証する。
直前の最低電圧は7.18 Vなので、次は同種類・容量・取付位置の充電済み電池を準備する。現在は両方OFF確認済み、次候補は未実行。新しい配置・余裕・両方ON・手離しの確認後に一回だけ実行する。

- [今回の相対軌跡](both15_current_midpoint/relative_trajectory.png)
- [今回の中点診断](both15_current_midpoint/report.json)
- [比較用動画の中点診断](both15_previous_charged_midpoint/report.json)
- [黄色の実観測例](rear_observed_candidate_crops.png)
- [実行指令とLend予測の根拠](runtime_command_refined_review.json)
'''
with (out / 'README.md').open('x', encoding='utf-8') as f:
    f.write(text)
for path in (BASE / 'preparation.json', stage / 'phase.json'):
    state = read(path)
    state.update(state='LEND40P5_PREPARED_WAITING_CHARGED_BATTERY_BOTH_OFF', phase='LEND40P5_PREPARED_WAITING_CHARGED_BATTERY_BOTH_OFF',
        operator_ready=False, execution_pending=False, power_state='Both_OFF_confirmed', ble_disconnected=True,
        candidate=str(next_stages[0] / 'candidate.settings.json'), next_candidate=str(next_stages[0] / 'candidate.settings.json'),
        config=str(next_stages[0] / 'config.json'), next_request_id=read(next_stages[0] / 'phase.json')['request_id'],
        latest_detector_candidate_review=str(out / 'candidate_review.json'), battery_replacement_required=True,
        charged_battery_ready_confirmed=False, current_placement_confirmed=False, current_clearance_confirmed=False,
        current_hands_clear_confirmed=False, parameter_adoption_allowed=False)
    path.write_text(json.dumps(state, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
manifest = dict(source_trials=[str(read(out / name / 'candidate_analysis_provenance.json')['source_trial'])
    for name in ('both15_current', 'both15_previous_charged')], source_data_changed=False, hardware_commands_sent=0,
    files={path.relative_to(out).as_posix(): dict(size_bytes=path.stat().st_size, sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        for path in sorted(out.rglob('*')) if path.is_file()})
with (out / 'supplemental_manifest.json').open('x', encoding='utf-8') as f:
    json.dump(manifest, f, ensure_ascii=False, indent=2)
print(json.dumps(dict(saved=True, next_lend=next_lend, predicted_lend=prediction.tolist(),
    effective_command_radius_mm=runtime['inferred_effective_command_r_min_mm']), ensure_ascii=False))
