"""色検出候補の比較と、未実行の次ターン候補を保存する。実機操作なし。"""
from pathlib import Path
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import shutil
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tools/logs/pixel_camera/turn1800_A_r180_20261120_v1'
stage = BASE / 'radius46p8_c2p0_03'
out = stage / 'marker_visibility_review_v1'
read = lambda p: json.loads(p.read_text(encoding='utf-8-sig'))
current_path, previous_path = out / 's15_current_midpoint/report.json', out / 's15_previous_charged_midpoint/report.json'
current, previous = read(current_path), read(previous_path)
assert all(r['relative_geometry_eligible'] and all(r['checks'].values()) for r in (current, previous))
old_previous = read(BASE / 'radius48p2_c2p0_entrycheck_02/entry_clock_midpoint_review_v1/report.json')
old_c4 = read(BASE / 'radius46p2_c4p25_01/entry_clock_midpoint_review_v1/report.json')
old_c2 = read(BASE / 'angle2p0_r46p2_01/entry_clock_midpoint_review_v1/report.json')
vector = lambda r: np.array([r['return_spacing_at_forward45_mm'], r['path_angle_deg'], *r['lend_right_forward_mm']])
radius_gradient = (vector(previous) - vector(current)) / 1.4
correction_gradient = (vector(old_c4) - vector(old_c2)) / 2.25
prediction = vector(current) + .8 * radius_gradient - .5 * correction_gradient
geometry_source = out / 'body_geometry_A_red_s15_candidate.json'
next_stages = [BASE / f'radius47p6_c1p5_s15_{i:02d}' for i in range(1, 4)]
for next_stage in next_stages:
    target = next_stage / 'body_geometry_A_red_s15_candidate.json'
    assert not target.exists()
    shutil.copyfile(geometry_source, target)
    config = read(next_stage / 'config.json')
    original_geometry_path = config['geometry']
    config['geometry'] = str(target)
    (next_stage / 'config.json').write_text(json.dumps(config, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    with (next_stage / 'detector_candidate_setup.json').open('x', encoding='utf-8') as f:
        json.dump(dict(status='PROSPECTIVE_DETECTOR_CANDIDATE_TEST_NOT_PERMANENT_ADOPTION',
            source_geometry=str(geometry_source), previous_config_geometry=original_geometry_path,
            candidate_geometry=str(target), candidate_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),
            red_minimum_saturation=15, physical_geometry_changed=False, camera_settings_changed=False,
            observation_quality_thresholds_changed=False, inferred_observations=False), f, ensure_ascii=False, indent=2)
goals = dict(width_at_forward45_mm=90., width_tolerance_mm=2., path_angle_deg=180., path_tolerance_deg=1.,
    nominal_lend_right_forward_mm=[90., 90.], lend_right_tolerance_mm=2., lend_forward_tolerance_mm=3.)
proposal = dict(mouse='A', motion='long_r180', status='OFFLINE_CANDIDATE_NOT_EXECUTED',
    source_midpoint_reviews=[str(current_path), str(previous_path)],
    source_original_rejection_preserved=True, detector_candidate_test=True,
    parameter_changes=dict(r_min=dict(before=-46.8, after=-47.6), correction=dict(before=2., after=1.5)),
    unchanged_parameters=dict(lstart=7.52, lend=31.5, velo=1.8, pre_accel=6.5),
    same_current_battery_requested=True, pid_ff_suction_unchanged=True,
    model_order=['width_mm', 'path_angle_deg', 'lend_right_mm', 'lend_forward_mm'],
    provisional_radius_sensitivity=radius_gradient.tolist(), provisional_correction_sensitivity=correction_gradient.tolist(),
    predicted_geometry=prediction.tolist(), prediction_is_measurement=False,
    sensitivity_limitations='電圧・時刻・旧色検出条件が完全には揃わない差分モデル。独立した物理同定値ではなく次の一回を選ぶ暫定予測。',
    working_candidate_targets=goals, new_operator_ready_required=True,
    parameter_adoption_allowed=False, fixed_board_accepted=False, firmware_changed=False)
with (next_stages[0] / 'geometry_proposal.json').open('x', encoding='utf-8') as f:
    json.dump(proposal, f, ensure_ascii=False, indent=2)
comparison = dict(recorded_at_utc=datetime.now(timezone.utc).isoformat(),
    status='DETECTOR_CANDIDATE_RELATIVE_REANALYSIS_PASSED_NOT_ADOPTED',
    original_trial_still_rejected_under_original_detector=True,
    current_geometry=vector(current).tolist(), previous_geometry_same_detector=vector(previous).tolist(),
    previous_good_trial_change_from_original_detector=(vector(previous) - vector(old_previous)).tolist(),
    current_required_observation_rate=current['required_observation_rate'],
    current_motion_observation_rate=current['motion_observation_rate'],
    previous_required_observation_rate=previous['required_observation_rate'],
    actual_registered_aruco_required=True, inferred_points_used=False, statistical_quality_thresholds_changed=False,
    formal_false_positive_rate_verified=False, relative_geometry_only=True, fixed_board_accepted=False,
    parameter_adoption_allowed=False, source_params_changed=False, both_powered_off_confirmed=True,
    next_stage=str(next_stages[0]), next_request_id=read(next_stages[0] / 'phase.json')['request_id'],
    next_execution_pending=False, next_geometry_prediction=prediction.tolist(),
    source_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in
        (current_path, previous_path, geometry_source, out / 'front_observation_comparison.json')})
with (out / 'candidate_review.json').open('x', encoding='utf-8') as f:
    json.dump(comparison, f, ensure_ascii=False, indent=2)
plt.rcParams['font.family'] = 'Yu Gothic'
fig, ax = plt.subplots(figsize=(7, 8), layout='constrained')
for name, label, review in [('s15_previous_charged', '半径48.2 mm・補正+2.0°', previous),
                            ('s15_current', '半径46.8 mm・補正+2.0°', current)]:
    frame = pd.read_csv(out / (name + '_midpoint') / 'relative_trajectory.csv')
    seen = frame[frame.detected]
    line, = ax.plot(seen.right_mm, seen.forward_mm, label=label)
    ax.scatter(*review['lend_right_forward_mm'], color=line.get_color(), s=35)
ax.axvline(90, color='.5', ls='--', label='幅の目標90 mm')
ax.scatter(90, 90, marker='x', c='black', label='名目Lend (90,90) mm')
ax.set(xlabel='実測入口に対する右 [mm]', ylabel='実測入口に対する前 [mm]',
    title='機体A・1800 R180 / 同じ赤S15候補で比較', aspect='equal')
ax.grid(); ax.legend(fontsize=8)
fig.savefig(out / 'same_detector_geometry_comparison.png', dpi=140); plt.close(fig)
text = f'''# 機体A・1800 R180の色検出候補の再解析

元の半径46.8 mm・補正+2.0度の試走は、赤マーカーのLend付近の20 ms超欠測により不合格として保存済み。元の動画・設定・CSV・解析・保存先は変更していない。
実画像の赤が薄いピンクに映り、欠測例の彩度medianは19だった。赤の彩度下限だけ35から15へ変更した解析候補を、今回と直前の良好な動画へ同じように適用した。
実際の色領域のみを検出し、各使用フレームの実ArUco2/3登録、95%観測率、直線・入口時計・静止条件は保持。点の推定補完はしていない。

| 同じ候補検出器による相対マーカー中点 | 半径48.2 mm | 半径46.8 mm |
| --- | ---: | ---: |
| 返り幅 [mm] | {vector(previous)[0]:.3f} | {vector(current)[0]:.3f} |
| 返り角 [deg] | {vector(previous)[1]:.3f} | {vector(current)[1]:.3f} |
| Lend右 [mm] | {vector(previous)[2]:.3f} | {vector(current)[2]:.3f} |
| Lend前 [mm] | {vector(previous)[3]:.3f} | {vector(current)[3]:.3f} |

良好だった動画への変更差（幅/角/右/前）は{comparison['previous_good_trial_change_from_original_detector']}。
今回の必要区間の中点観測率は{current['required_observation_rate']*100:.2f}%、純走行窓は{current['motion_observation_rate']*100:.2f}%。全て実観測。今回の前側で新たに拾えた10点によりLendも観測できた。
目視確認は欠測例と周辺画像で行った。正式な偽陽性率・カメラ足位置・絶対機軸姿勢は未校正。標準の全軌跡モデルはREVIEW_REQUIREDのままで、入口だけの時計による相対診断と区別する。
この候補は恒久採用ではなく、次の独立試走で使う検出候補。物理ターン値・ファームウェアの採用は未実施。

次の物理候補は半径47.6 mm・補正+1.5度、Lstart7.52 mm・Lend31.5 mmを保持。既存差分モデルの予測（幅/角/右/前）は{prediction.tolist()}。電圧や旧色条件の交絡があり、予測は測定値ではない。
新しい開始配置の撮影を毎回行い、過去の配置を次回へ流用しない。概略配置だけの確認として、位置5 mm・向き5度・実観測30点以上と静止ばらつきで実行可否を判定する。精密判定の不合格原本を保持し、走行中の採用条件は変更しない。
現在は両方OFF確認済み、次の候補は未実行。配置・干渉余裕・両方ON・手離しの新しい確認後に一回だけ実行する。

- [同じ検出候補での比較図](same_detector_geometry_comparison.png)
- [今回の中点診断](s15_current_midpoint/report.json)
- [比較用動画の中点診断](s15_previous_charged_midpoint/report.json)
- [観測の増減と重心差](front_observation_comparison.json)
'''
with (out / 'README.md').open('x', encoding='utf-8') as f:
    f.write(text)
for path in (BASE / 'preparation.json', stage / 'phase.json'):
    state = read(path)
    state.update(state='S15_CANDIDATE_REVIEWED_NEXT_PREPARED_BOTH_OFF', phase='S15_CANDIDATE_REVIEWED_NEXT_PREPARED_BOTH_OFF',
        operator_ready=False, execution_pending=False, power_state='Both_OFF_confirmed', ble_disconnected=True,
        candidate=str(next_stages[0] / 'candidate.settings.json'), next_candidate=str(next_stages[0] / 'candidate.settings.json'),
        config=str(next_stages[0] / 'config.json'), next_request_id=comparison['next_request_id'],
        latest_detector_candidate_review=str(out / 'candidate_review.json'), current_placement_confirmed=False,
        current_clearance_confirmed=False, current_hands_clear_confirmed=False, parameter_adoption_allowed=False)
    path.write_text(json.dumps(state, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
manifest = dict(source_trials=[str(read(out / name / 'candidate_analysis_provenance.json')['source_trial'])
        for name in ('s15_current', 's15_previous_charged')],
    source_data_changed=False, hardware_commands_sent=0, files={})
for path in sorted(out.rglob('*')):
    if path.is_file():
        manifest['files'][path.relative_to(out).as_posix()] = dict(size_bytes=path.stat().st_size,
            sha256=hashlib.sha256(path.read_bytes()).hexdigest())
with (out / 'supplemental_manifest.json').open('x', encoding='utf-8') as f:
    json.dump(manifest, f, ensure_ascii=False, indent=2)
print(json.dumps(dict(saved=True, current_geometry=vector(current).tolist(),
    next_prediction=prediction.tolist(), next_stage=str(next_stages[0])), ensure_ascii=False))
