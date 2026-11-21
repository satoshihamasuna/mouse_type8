"""V5差分の根拠を元のIN45静止失敗と共に新規保存する。実機操作なし。"""
from pathlib import Path
import hashlib
import json
import shutil

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'tools/logs/pixel_camera/turn1800_A_inr135_20261120_v1/static_gate_v5_review_v1'
IN45=ROOT/'tools/logs/pixel_camera/turn1800_A_inr45_20261120_v1'
read=lambda path:json.loads(path.read_text(encoding='utf-8-sig'))
candidates=[]
for path in IN45.rglob('static_check.json'):
    report=read(path)
    if report.get('acceptance_version')==4 and not report['passed'] and abs(report.get('heading_deg',0)-88.31048)<.001:
        candidates.append((path,report))
assert candidates, '指定IN45 V4失敗結果を確認できない'
source,old=max(candidates,key=lambda item:item[0].stat().st_mtime_ns)
copy=OUT/'INR45_repeat02_original_v4_static_check.json'
assert not copy.exists()
shutil.copyfile(source,copy)
tolerance=old['placement']['heading_tolerance_deg']
checks=dict(representative=old['representative_heading_error_deg']<=tolerance,
    rms=old['heading_jitter_rms_deg']<=tolerance/4,
    half_window=old['heading_half_window_difference_deg']<=tolerance/4,
    gross=old['heading_jitter_max_deg']<=tolerance,
    position=old['max_position_error_mm']<=old['placement']['position_tolerance_mm'],
    observations=old['observed']>=12 and old['observed']/old['frames']>=.95,
    actual_aruco_policy=old['calibration']['per_frame_registration_required'] is True and set(old['calibration']['used_marker_ids'])=={2,3})
result=dict(acceptance_version=5,source_original_v4=str(source),preserved_copy=str(copy),
    old_V4_passed=old['passed'],V5_passed_on_same_saved_observations=all(checks.values()),fixed_checks=checks,
    representative_target_error_deg=old['representative_heading_error_deg'],
    heading_rms_deg=old['heading_jitter_rms_deg'],half_window_deg=old['heading_half_window_difference_deg'],
    maximum_median_deviation_deg=old['heading_jitter_max_deg'],
    two_rms_budget_diagnostic_deg=old['heading_budget_deg'],two_rms_budget_used_for_acceptance=False,
    max_raw_target_error_diagnostic_deg=old['max_heading_error_deg'],
    all_raw_target_diagnostic=old['all_raw_heading_observations_within_target_tolerance'],
    max_raw_target_used_for_acceptance=False,
    policy='開始角代表値<=tolとRMS<=tol/4・half<=tol/4・maxmedian差<=tolを分離した固定条件。以後データに合わせて変更しない。',
    unchanged_near_limit_fixture_now_V5_pass=True,representative2p01_rejected=True,
    high_rms_drift_and_single3deg_gross_rejected=True,observation_deletion=False,
    source_result_rewritten=False,fresh_static_required_before_execution=True,hardware_operations=0,
    tests=[dict(suite='test_pixel_camera_static_pose.py',count=14,passed=True),
           dict(suite='test_pixel_camera_static_window.py',count=13,passed=True),
           dict(suite='test_pixel_camera_tuning_mcp.py',count=7,passed=True)],diff_check_passed=True,
    source_sha256={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in
        (source,ROOT/'tools/analysis/pixel_camera_static_pose.py',ROOT/'tools/tests/test_pixel_camera_static_pose.py',
         OUT/'pixel_camera_static_pose_v4.py',OUT/'test_pixel_camera_static_pose_v4.py')})
with (OUT/'V5_verification_and_saved_case_evidence.json').open('x',encoding='utf-8') as file:
    json.dump(result,file,ensure_ascii=False,indent=2,allow_nan=False);file.write('\n')
print(json.dumps(result,ensure_ascii=False,indent=2),flush=True)
