"""既存の入口車輪時計・相対中点解析を今回B1600の明示入力へ限定する。機器操作なし。"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT/'tmp/parallel_inr135_midpoint005_20261120.py').read_text(encoding='utf-8-sig')
replacements = [
    ("BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_inr135_20261120_v1'", "BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr135_20261120_v1'"),
    ("RUN=BASE/'baseline01/local_execution/run_A_INR135_1800_baseline_20261120_005'", "RUN=BASE/(sys.argv[1] if len(sys.argv)>1 else 'baseline01/local_execution/run_B_INR135_1600_baseline_green_20261120_002')"),
    ("OUT=BASE/'parallel_analysis/A_INR135_1800_baseline_20261120_005/midpoint_raw_wheel_clock_v1'", "OUT=RUN/'midpoint_raw_wheel_clock_v1'"),
    ("assert set(provenance['marker_calibration']['used_marker_ids'])=={2,3}", "assert len(set(provenance['marker_calibration']['used_marker_ids']))>=2"),
    ("request_Lstart_mm=float(22.85+delta_ls),request_Lend_mm=float(34.81+delta_le)", "request_Lstart_mm=float(settings['parameters']['lstart']+delta_ls),request_Lend_mm=float(settings['parameters']['lend']+delta_le)"),
    ("result=dict(schema_version=1,mouse='A',motion='in_r135'", "result=dict(schema_version=1,mouse='B',motion='in_r135'"),
    ("heights=dict(camera_height_mm=1500.,front_height_mm=10.,rear_height_mm=1.,camera_foot_unknown=True,camera_foot_constant_cancels=True)", "heights=dict(camera_height_mm=trial['camera_height_mm'],front_height_mm=geometry['front_height_mm'],rear_height_mm=geometry['rear_height_mm'],camera_foot_unknown=True,camera_foot_constant_cancels=True)"),
    ("existing_angle_correction_candidate_deg=3.5", "existing_angle_correction_candidate_deg=settings['parameters']['correction']"),
    ("'長さ解は現在138度出口方向を保つ仮定。補正+3.5度は終端を変えるため再計測が必要。'", "'長さ解は今回の出口方向を保つ仮定。角度補正を変えた後は長さを再計測する。'"),
    ("'元summaryの初期直進窓不足と軌道モデルRMS17.77mmを別評価として保持。'", "'元summaryと軌道モデルの品質判定をそのまま保持。BのEKF係数は未同定。'"),
    ("title=f'IN135005 midpoint / {angle:.3f}°'", "title=f'機体B1600 右135度入口 / {angle:.3f}°'"),
]
for before, after in replacements:
    assert source.count(before) == 1, before
    source = source.replace(before, after)
guard = "assert trial['motion']==settings['motion_name']=='in_r135'"
source = source.replace(guard, guard + "\nassert settings['mouse']=='B' and settings['preset_speed_mm_s']==1600")
compile(source, '<B1600 in_r135 analysis>', 'exec')
output = ROOT/'tmp/review_turn1600_B_inr135_midpoint_20261120.py'
assert not output.exists()
output.write_text(source, encoding='utf-8', newline='\n')
print('Prepared B1600 in_r135 saved-data analysis; hardware operations: 0')
