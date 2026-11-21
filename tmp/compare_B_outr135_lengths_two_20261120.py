"""B1600右135度出口の同一長さ候補2本を入口相対で比較する。機器操作なし。"""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json
import math
import statistics
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import pandas as pd

root = Path(__file__).resolve().parents[1]
base = root/'tools/logs/pixel_camera/turn1600_B_outr135_20261120_v1'
out = base/'lengths_same_candidate_two_review_v1'
assert not out.exists()
read = lambda path: json.loads(path.read_text(encoding='utf-8-sig'))
runs = [base/f'lengths_Ls6_Le27p9_c1p9_{number:02d}/local_execution/run_B_OUTR135_1600_Ls6_Le27p9_c1p9_green_20261120_{number:03d}' for number in (2, 3)]
rows = []
settings = []
for run in runs:
    geometry_path = run/'midpoint_raw_wheel_clock_v1/midpoint_review.json'
    geometry = read(geometry_path)
    tracking = read(run/'tracking_1ms_review_v3/verification.json')
    actual = read(run/'saved_1ms.settings.json')
    settings.append({key:actual[key] for key in ('mouse', 'motion_name', 'preset_speed_mm_s', 'parameters', 'pid', 'feedforward', 'suction', 'turn_count')})
    assert geometry['independent_optical_geometry_eligible']
    assert tracking['execution_resend_count'] == 0
    report = read(run/'full/report.json')
    rows.append(dict(source_trial=str(run/'trial.json'), geometry_review=str(geometry_path),
        geometry_review_sha256=hashlib.sha256(geometry_path.read_bytes()).hexdigest(),
        exit_angle_deg=geometry['exit_path_direction_change_deg'],
        angle_error_deg=geometry['exit_path_direction_change_deg']-135,
        Lend_right_forward_mm=geometry['midpoint_Lend_right_forward_mm'],
        Lend_error_mm=math.dist(geometry['midpoint_Lend_right_forward_mm'], geometry['target_midpoint_Lend_right_forward_mm']),
        required_observation_fraction=report['required_motion_window_quality']['observation_rate'],
        registration_fraction=report['required_motion_window_quality']['calibration_rate'],
        tracking_rms_rad_s=tracking['tracking']['tracking_rms_rad_s'],
        saturation_pct=tracking['tracking']['saturation_pct'], battery_min_v=tracking['battery_min_v'],
        clock_fit_rms_mm=geometry['clock']['incoming_raw_wheel_fit_rms_mm'],
        raw_clock_half_spread_ms=geometry['clock']['half_window_zero_spread_ms'],
        length_only_candidate_at_current_angle=geometry['length_only_candidate_at_current_angle']))
assert settings[0] == settings[1]
baseline = read(base/'baseline003_red30_review.json')['geometry']
result = dict(recorded_at=datetime.now(timezone.utc).isoformat(), mouse='B', motion='out_r135',
    preset_mm_s=1600, successful_same_candidate_runs=2, settings=settings[0], runs=rows,
    mean_exit_angle_deg=statistics.mean(row['exit_angle_deg'] for row in rows),
    exit_angle_range_deg=max(row['exit_angle_deg'] for row in rows)-min(row['exit_angle_deg'] for row in rows),
    mean_Lend_right_forward_mm=[statistics.mean(row['Lend_right_forward_mm'][axis] for row in rows) for axis in (0,1)],
    maximum_Lend_error_mm=max(row['Lend_error_mm'] for row in rows),
    baseline_angle_deg=baseline['exit_path_direction_change_deg'],
    baseline_Lend_error_mm=math.dist(baseline['midpoint_Lend_right_forward_mm'], baseline['target_midpoint_Lend_right_forward_mm']),
    reference='実測入口基準の高さ補正前後マーカー中点。固定盤面/絶対車軸・機軸ではない。',
    source_params_changed=False, firmware_write_performed=False, adopted=False,
    limitations=['同一候補2本。絶対位置・独立ハード同期・車軸/機軸幾何・BのEKF係数・連続接続は未確認。',
        '元full/summaryの判定は維持。静止は取得側連続ログ、動画、操作者OFF/接触なしを別に記録。'])
out.mkdir()
with (out/'review.json').open('x',encoding='utf-8') as stream:
    json.dump(result,stream,ensure_ascii=False,indent=2,allow_nan=False)
plt.rcParams['font.family']='Yu Gothic'
fig,ax=plt.subplots(figsize=(6,7),layout='constrained')
for number,(run,row) in enumerate(zip(runs,rows),1):
    pose=pd.read_csv(run/'midpoint_raw_wheel_clock_v1/midpoint_relative_trajectory.csv')
    mask=pose.registered_midpoint_valid.astype(bool)
    ax.plot(pose.right_mm.where(mask),pose.forward_mm.where(mask), label=f'{number}本目 {row["exit_angle_deg"]:.3f}°')
    ax.scatter(*row['Lend_right_forward_mm'])
ax.scatter(*baseline['target_midpoint_Lend_right_forward_mm'],marker='x',color='black',label='Lend目標')
ax.set(xlabel='入口に対する右 [mm]',ylabel='入口に対する前 [mm]',aspect='equal',title='機体B1600 右135度出口 / 同一候補2本')
ax.grid(alpha=.3);ax.legend(fontsize=8)
fig.savefig(out/'two_trajectories.png',dpi=140);plt.close(fig)
text = '# 機体B1600 右135度出口・同一長さ候補2本\n\n'
text += '半径−41 mm、Lstart 6.0 mm、Lend 27.9 mm、補正＋1.9°、吸引650。機体BのPID/FFを同じ実機値で照合。\n\n'
text += '|試行|入口相対出口角 [deg]|Lend右/前 [mm]|目標との差 [mm]|最低電圧 [V]|\n|---|---:|---:|---:|---:|\n'
for number,row in enumerate(rows,1):
    text += f'|{number}|{row["exit_angle_deg"]:.3f}|{row["Lend_right_forward_mm"][0]:.3f}/{row["Lend_right_forward_mm"][1]:.3f}|{row["Lend_error_mm"]:.3f}|{row["battery_min_v"]:.3f}|\n'
text += f'\n平均角{result["mean_exit_angle_deg"]:.3f}°、2本間の角度差{result["exit_angle_range_deg"]:.3f}°。元の長さ1.0/31.5 mmの初回に対する目標誤差は{result["baseline_Lend_error_mm"]:.3f} mm。\n\n'
text += '基準は実測入口に対する高さ補正マーカー中点。各フレームの実ArUco登録と観測率95%以上を維持。原本・元品質判定を変更していない。ファームへの出口値反映・書込みは未実施。BのEKF係数と連続走行は未確認。\n'
(out/'README.md').write_text(text,encoding='utf-8')
print(json.dumps({key:result[key] for key in ('runs','mean_exit_angle_deg','exit_angle_range_deg','mean_Lend_right_forward_mm')},ensure_ascii=False),flush=True)
