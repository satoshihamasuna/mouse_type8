"""実測の出口線について、Lstart/Lendが動かす方向を示す。実機操作なし。"""
from pathlib import Path
import json
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties

ROOT = Path(__file__).resolve().parents[1]
FOLDER = ROOT / 'tools/logs/pixel_camera/turn1600_A_r90_20261118_frame18_v1/comparison_corr1p0_v1'
data = pd.read_csv(FOLDER / 'corr1p0_relative_trajectory.csv')
document = json.loads((FOLDER / 'video_comparison.json').read_text(encoding='utf-8'))
case = next(x for x in document['comparisons'] if x['correction_deg'] == 1.)
valid = data.detected.to_numpy(bool)
mask = valid & data.time_video_s.between(*case['exit_window_pts_s'])
points = data.loc[mask, ['right_mm', 'forward_mm']].to_numpy()
centre = points.mean(axis=0)
direction = np.linalg.svd(points - centre, full_matrices=False)[2][0]
slope = direction[1] / direction[0]
intercept = centre[1] - slope * centre[0]
at90 = slope * 90 + intercept
plt.rcParams['font.family'] = FontProperties(fname='C:/Windows/Fonts/meiryo.ttc').get_name()
fig, axes = plt.subplots(1, 2, figsize=(11, 5), layout='constrained', gridspec_kw={'width_ratios': [1, 1.4]})
axes[0].plot(-data.loc[valid, 'right_mm'], data.loc[valid, 'forward_mm'], color='tab:orange', lw=2)
axes[0].axhline(270, color='black', ls='--', lw=1)
axes[0].annotate('入口方向', xy=(0, 155), xytext=(12, 105), arrowprops={'arrowstyle': '->', 'color': 'tab:blue'}, color='tab:blue')
axes[0].annotate('出口方向', xy=(-190, 274), xytext=(-125, 243), arrowprops={'arrowstyle': '->', 'color': 'tab:orange'}, color='tab:orange')
axes[0].set(xlim=(-215, 30), ylim=(300, -10), title='右90°：入口は下向き、出口は左向き', xlabel='入口基準の横位置 [mm]', ylabel='ターン前の進行方向 Y [mm]')
axes[0].set_aspect('equal', adjustable='box')
x = np.linspace(50, 175, 100)
axes[1].plot(-x, np.full_like(x, 270), color='black', ls='--', label='目標の出口線：Y=270 mm')
axes[1].plot(-x, slope * x + intercept, color='tab:orange', lw=2, label='実測の出口線：Y≒274 mm')
axes[1].annotate('', xy=(-90, at90), xytext=(-90, 270), arrowprops={'arrowstyle': '<->', 'color': 'tab:blue', 'lw': 1.7})
axes[1].text(-83, 271.8, f'約{at90 - 270:.1f} mm\n入口方向の差', color='tab:blue', fontsize=10, va='center')
axes[1].annotate('Lend が増減させる方向', xy=(-150, 279), xytext=(-98, 279), arrowprops={'arrowstyle': '->', 'color': 'tab:green', 'lw': 1.5}, color='tab:green', fontsize=10, ha='center', va='center')
axes[1].set(xlim=(-178, -48), ylim=(286, 260), title='出口付近の拡大：線の位置と線に沿う長さを区別', xlabel='入口基準の横位置 [mm]（出口は左）', ylabel='ターン前の進行方向 Y [mm]')
axes[1].legend(loc='lower left', fontsize=9)
for ax in axes:
    ax.grid(alpha=.2)
fig.savefig(FOLDER / 'Lstart_Lend_axis_explanation.png', dpi=170)
plt.close(fig)
print('Saved axis explanation with Japanese labels.', flush=True)
