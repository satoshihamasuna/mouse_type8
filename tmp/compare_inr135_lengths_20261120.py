"""A1800右135入口の基準と完全ログ002を同じ相対座標で比較する。実機操作なし。"""
from pathlib import Path
import json
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tools/logs/pixel_camera/turn1800_A_inr135_20261120_v1'
OUT = BASE / 'lengths_Ls2p5_Le24_preparation_v1'
read = lambda p: json.loads(p.read_text(encoding='utf-8-sig'))
sources = [
    ('基準：22.85 / 34.81 mm・補正0°', BASE / 'parallel_analysis/A_INR135_1800_baseline_20261120_005/midpoint_raw_wheel_clock_v1'),
    ('今回：7.85 / 34.81 mm・補正＋3.5°', BASE / 'parallel_analysis/A_INR135_1800_c3p5_Ls7p85_req_20261120_002/midpoint_raw_wheel_clock_v1'),
]
plt.rcParams['font.family'] = 'Yu Gothic'
fig, ax = plt.subplots(figsize=(6.8, 7), layout='constrained')
rows = []
for label, folder in sources:
    review = read(folder / 'midpoint_review.json')
    frame = pd.read_csv(folder / 'midpoint_relative_trajectory.csv')
    quality = review['independent_clock_required_motion_quality']
    mask = (frame.time_video_s >= quality['video_motion_start_s'] - .02) & (frame.time_video_s <= quality['video_motion_end_s'] + .05)
    valid = frame.registered_midpoint_valid.astype(bool).to_numpy()
    xy = np.where(valid[:, None], frame[['right_mm', 'forward_mm']].to_numpy(), np.nan)
    item, = ax.plot(*xy[mask].T, label=label)
    ax.scatter(*review['midpoint_Lend_right_forward_mm'], c=item.get_color(), s=35)
    rows.append(dict(label=label, source=str(folder / 'midpoint_review.json'),
        exit_angle_deg=review['exit_path_direction_change_deg'],
        Lend_right_forward_mm=review['midpoint_Lend_right_forward_mm'],
        Lend_residual_mm=review['midpoint_Lend_residual_mm'],
        independent_optical_geometry_eligible=review['independent_optical_geometry_eligible'],
        original_summary=review['original_summary']))
ax.plot([90, 180], [135, 45], '--', color='gray', label='目標出口線')
ax.scatter(90, 135, c='black', marker='x', s=65, label='Lend目標 (90,135) mm')
ax.scatter(180, 45, c='black', marker='+', s=65, label='停止目標 (180,45) mm')
ax.set(xlabel='入口に対する右 [mm]', ylabel='入口に対する前 [mm]', aspect='equal',
       title='機体A / 右135°入口 / 1800 mm/s\n相対マーカー中点・採用前の比較')
ax.grid(alpha=.3)
ax.legend(fontsize=8)
fig.savefig(OUT / 'baseline_and_c3p5_Ls7p85_comparison.png', dpi=150)
plt.close(fig)
summary = dict(mouse='A', motion='in_r135', preset_mm_s=1800,
    coordinate_basis='relative marker midpoint aligned to measured entry; no absolute axle calibration',
    rows=rows, next_requested_parameters=read(BASE / 'c3p5_Ls2p5_Le24_01/candidate.settings.json')['parameters'],
    next_stage=str(BASE / 'c3p5_Ls2p5_Le24_01'), next_candidate_executed=False,
    current_power_state='Both_OFF_confirmed_after_Ls7p85_002', current_operator_ready=False,
    source_params_changed=False, hardware_operations=0,
    EKF_identification=dict(status='NOT_IDENTIFIED',
        reason='135-degree/1800 is outside current firmware calibration eligibility. Absolute body yaw/axle geometry is not independently calibrated; no K/noise candidate is adopted.'))
with (OUT / 'comparison_and_resume_state.json').open('x', encoding='utf-8') as file:
    json.dump(summary, file, ensure_ascii=False, indent=2, allow_nan=False)
    file.write('\n')
print('Comparison and next-trial state saved; hardware operations: 0')
