"""R180の単体結果と次の幾何候補を保存する。接続・走行は行わない。"""
from pathlib import Path
from datetime import datetime, timezone
import argparse
import hashlib
import json
import shutil

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tools/logs/pixel_camera/turn1800_A_r180_20261120_v1'
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--stage', type=Path, required=True)
parser.add_argument('--trial-id', required=True)
parser.add_argument('--next-stage', type=Path)
parser.add_argument('--tracking', default='tracking_1ms_review_v1')
parser.add_argument('--host-tests', type=int)
parser.add_argument('--save-rejected', action='store_true', help='品質不合格の参考結果を合格扱いせず保存する')
args = parser.parse_args()
stage = args.stage.resolve()
assert stage.is_relative_to(BASE)
run = stage / 'local_execution' / ('run_' + args.trial_id)
assert run.resolve().is_relative_to(stage)
read = lambda p: json.loads(p.read_text(encoding='utf-8-sig'))
trial, settings = read(run / 'trial.json'), read(run / 'saved_1ms.settings.json')
review_path = stage / 'entry_clock_midpoint_review_v1/report.json'
review = read(review_path)
tracking = read(run / args.tracking / 'verification.json')
operator_path = stage / 'local_execution/operator_results' / (args.trial_id + '.json')
operator = read(operator_path)
assert operator['both_powered_off'] and operator['contact_detected'] is False
assert trial['state'] == 'CAPTURED_REVIEW_REQUIRED' and trial['saved_log']['status'] == 'saved'
assert trial['saved_log']['drop_count'] == 0 and trial['execution_resend_count'] == 0
quality_passed = bool(review['relative_geometry_eligible'] and all(review['checks'].values()))
if args.save_rejected:
    assert not quality_passed, '品質不合格の記録だけを指定してください'
else:
    assert quality_passed
goals = dict(width_at_forward45_mm=90., width_tolerance_mm=2., path_angle_deg=180.,
    path_tolerance_deg=1., nominal_lend_right_forward_mm=[90., 90.],
    lend_right_tolerance_mm=2., lend_forward_tolerance_mm=3.)
if (stage / 'geometry_proposal.json').is_file():
    goals = read(stage / 'geometry_proposal.json').get('working_candidate_targets', goals)
elif (stage / 'working_targets.json').is_file():
    goals = read(stage / 'working_targets.json')
target_checks = dict(
    width=abs(review['return_spacing_at_forward45_mm'] - goals['width_at_forward45_mm']) <= goals['width_tolerance_mm'],
    path_angle=abs(review['path_angle_deg'] - goals['path_angle_deg']) <= goals['path_tolerance_deg'],
    lend_right=abs(review['lend_right_forward_mm'][0] - goals['nominal_lend_right_forward_mm'][0]) <= goals['lend_right_tolerance_mm'],
    lend_forward=abs(review['lend_right_forward_mm'][1] - goals['nominal_lend_right_forward_mm'][1]) <= goals['lend_forward_tolerance_mm'])
exports = run / 'review_exports'
exports.mkdir(exist_ok=False)
for source, name in ((stage / 'entry_clock_midpoint_review_v1', 'midpoint'),
                     (stage / 'front_phase_review_v1', 'front')):
    shutil.copytree(source, exports / name)
shutil.copyfile(operator_path, exports / 'operator_result.json')
result = dict(recorded_at_utc=datetime.now(timezone.utc).isoformat(), mouse='A', motion='long_r180',
    source_trial=str(run / 'trial.json'), parameters=settings['parameters'],
    physical_executions=1, execution_resend_count=0, saved_1ms_frames=trial['saved_log']['frame_count'],
    saved_1ms_drops=0, relative_geometry_quality_passed=quality_passed, working_targets=goals,
    geometry_values_are_diagnostic_only=not quality_passed, quality_checks=review['checks'],
    working_target_checks=target_checks, all_working_targets_met=quality_passed and all(target_checks.values()),
    width45_mm=review['return_spacing_at_forward45_mm'], path_angle_deg=review['path_angle_deg'],
    lend_right_forward_mm=review['lend_right_forward_mm'], minimum_battery_v=tracking['battery_min_v'],
    tracking_rms_rad_s=tracking['nominal_1ms_tracking_metrics']['tracking_rms_rad_s'],
    saturation_pct=tracking['nominal_1ms_tracking_metrics']['saturation_pct'],
    original_auto_stop_confirmed=review['original_auto_stop_confirmed'], original_failures_preserved=True,
    both_powered_off=True, contact_detected=False, firmware_changed=False,
    source_params_changed=False, parameter_adoption_allowed=False, fixed_board_accepted=False,
    ekf_candidate_adopted=False, host_tests_passed=args.host_tests,
    source_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in
        (run / 'trial.json', run / 'saved_1ms.csv', run / 'saved_1ms.settings.json', review_path, operator_path)})
next_stage = args.next_stage.resolve() if args.next_stage else None
if next_stage is not None:
    assert next_stage.is_relative_to(BASE)
    candidate = read(next_stage / 'candidate.settings.json')
    assert all(candidate[key] == settings[key] for key in ('mouse', 'motion_name', 'preset_speed_mm_s', 'pid', 'feedforward', 'suction', 'turn_count'))
    changes = {key: dict(before=value, after=candidate['parameters'][key])
        for key, value in settings['parameters'].items() if value != candidate['parameters'][key]}
    result.update(next_stage=str(next_stage), next_geometry_changes=changes,
        next_operator_ready=False, next_execution_pending=False)
    with (next_stage / 'working_targets.json').open('x', encoding='utf-8') as file:
        json.dump(goals, file, indent=2)
with (stage / 'stage_review_summary.json').open('x', encoding='utf-8') as file:
    json.dump(result, file, ensure_ascii=False, indent=2, allow_nan=False)
    file.write('\n')
shutil.copyfile(stage / 'stage_review_summary.json', exports / 'stage_review_summary.json')
text = f'''# 機体A・1800 R180の単体幾何試験

実機設定: {json.dumps(settings['parameters'], ensure_ascii=False)}

動画・1msログ{trial['saved_log']['frame_count']}件・実機設定を保存。欠落0、実行指令1回で再送なし。
操作者が接触なし・停止・機体/HY0020両方OFFを確認。採用値とファームウェアは未変更。

| 入口基準の相対マーカー中点 | 今回 | 名目目標 |
| --- | ---: | ---: |
| 前45mm断面の返り幅 [mm] | {review['return_spacing_at_forward45_mm']:.3f} | 90 |
| 返り角 [deg] | {review['path_angle_deg']:.3f} | 180 |
| Lendの右 [mm] | {review['lend_right_forward_mm'][0]:.3f} | 90 |
| Lendの前 [mm] | {review['lend_right_forward_mm'][1]:.3f} | 90 |

実ArUco2/3を各使用フレームで登録し、必要窓の観測率{100 * review['required_observation_rate']:.2f}%、
純走行窓{100 * review['motion_observation_rate']:.2f}%。95%条件、入口/退出直線、入口時計、静止の個別基準を維持。
品質判定は{quality_passed}、個別結果は{json.dumps(review['checks'], ensure_ascii=False)}。
品質不合格時の数値は参考値であり、設定の採用・追い込みには使用しない。
標準解析の不合格statusとsummaryの生成有無を保持し、独立入口時計の相対診断と区別する。
今回の作業目標の判定: {json.dumps(target_checks, ensure_ascii=False)}。
固定盤の開始/停止点・絶対車軸位置・body yaw・連続最短接続・採用の合格ではない。

追従RMS {tracking['nominal_1ms_tracking_metrics']['tracking_rms_rad_s']:.3f}rad/s、
出力飽和{tracking['nominal_1ms_tracking_metrics']['saturation_pct']:.1f}%、最低電圧{tracking['battery_min_v']:.2f}V。
EKF係数の採用も行っていない。元の観測と制御条件、カメラ幾何の限界を保持する。

次候補: {str(next_stage) if next_stage else '未設定'}。
変更内容: {json.dumps(result.get('next_geometry_changes', {}), ensure_ascii=False)}。
次候補は準備のみで未実行。現在の配置・余裕・ON・手離しの新規確認が必要。

- [相対中点レポート](review_exports/midpoint/report.json)
- [相対軌跡図](review_exports/midpoint/relative_trajectory.png)
- [角速度追従]({args.tracking}/verification.json)
- [実機設定](saved_1ms.settings.json)
- [元の取得記録](trial.json)
'''
if args.host_tests:
    text += f'\n解析側の角度補正込み区間選択を修正し、ホストテスト{args.host_tests}件を通過。実機設定には影響しない。\n'
with (run / 'stage_review_README.md').open('x', encoding='utf-8') as file:
    file.write(text)
for path in (stage / 'phase.json', BASE / 'preparation.json'):
    phase = read(path)
    saved_state = 'GEOMETRY_STAGE_SAVED_NEXT_PREPARED' if quality_passed else 'QUALITY_REJECTED_TRIAL_SAVED_NEXT_PREPARED'
    phase.update(state=saved_state, phase=saved_state,
        operator_ready=False, execution_pending=False, power_state='Both_OFF_confirmed',
        ble_disconnected=True, operator_contact=False, source_params_changed=False,
        current_placement_confirmed=False, current_clearance_confirmed=False,
        current_hands_clear_confirmed=False, current_physical_clearance_verified=False,
        next_candidate_executed=False, parameter_adoption_allowed=False,
        last_completed_trial=str(run / 'trial.json'), latest_stage_review=str(stage / 'stage_review_summary.json'))
    if next_stage is not None:
        phase.update(candidate=str(next_stage / 'candidate.settings.json'), next_candidate=str(next_stage / 'candidate.settings.json'),
            config=str(next_stage / 'config.json'), next_request_id=read(next_stage / 'phase.json')['request_id'])
    else:
        phase['next_request_id'] = None
    path.write_text(json.dumps(phase, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
print(json.dumps(dict(stage=str(stage), width45_mm=result['width45_mm'], angle_deg=result['path_angle_deg'],
    working_target_checks=target_checks, next_changes=result.get('next_geometry_changes', {})), ensure_ascii=False))
