"""今回の未実行候補の配置図を生成する。実機操作なし。"""
from pathlib import Path
import json
import re
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parents[1]
base = root / 'tools/logs/pixel_camera/turn1800_A_inr135_20261120_v1'
out = base / 'resume_20261120_inr135_v1'
read = lambda path: json.loads(path.read_text(encoding='utf-8-sig'))
candidate = read(base / 'c3p5_Ls7p85_01/candidate.settings.json')
p = candidate['parameters']
text = (root / 'Params/turn_table.h').read_text(encoding='utf-8')
wave = np.fromstring(re.search(r'accel_table\[1001\]\s*=\s*\{([^}]+)\}', text)[1], sep=',')
integral = float(re.search(r'accel_Integral\s*=\s*([.\d]+)', text)[1])
duration = np.deg2rad(abs(p['degree'] + p['correction'])) / (1800 / abs(p['r_min']) * integral)
t = np.linspace(0, duration, 1001)
w = 1800 / abs(p['r_min']) * wave
theta = np.r_[0., np.cumsum((w[:-1] + w[1:]) / 2 * np.diff(t))]
middle = (theta[:-1] + theta[1:]) / 2
right = np.r_[0., np.cumsum(1800 * np.sin(middle) * np.diff(t))]
forward = 90 + p['lstart'] + np.r_[0., np.cumsum(1800 * np.cos(middle) * np.diff(t))]
direction = np.array([np.sin(theta[-1]), np.cos(theta[-1])])
end = np.array([right[-1], forward[-1]]) + p['lend'] * direction
stop = end + 90 * np.sqrt(2) * direction
local = np.vstack(([0., 0.], [0., 90 + p['lstart']], np.c_[right, forward], end, stop))
board = np.c_[315 - local[:, 0], 135 + local[:, 1]]
plt.rcParams['font.family'] = 'Yu Gothic'
fig, ax = plt.subplots(figsize=(7, 7.5), layout='constrained')
ax.set(xlim=(-8, 368), ylim=(368, -8), aspect='equal', xlabel='盤面右 [mm]', ylabel='盤面下 [mm]')
ax.set_xticks(np.arange(0, 361, 45))
ax.set_yticks(np.arange(0, 361, 45))
ax.grid(alpha=.35)
for x in range(0, 361, 90):
    ax.scatter([x] * 5, range(0, 361, 90), s=17, color='#3e474b')
ax.plot(board[:, 0], board[:, 1], label='今回候補の名目中心軌跡（未実走）')
ax.scatter(315, 135, c='black', marker='+', s=130)
ax.plot([315, 315], [110, 160], c='black')
ax.scatter(315, 160, c='red', s=65, label='前：赤・下向き')
ax.scatter(315, 110, c='gold', edgecolors='black', s=65, label='後：黄')
ax.annotate('開始中点 (315,135) mm', xy=(315, 135), xytext=(135, 80), arrowprops=dict(arrowstyle='->'))
ax.scatter(225, 270, marker='x', s=65, label='Lend格子目標 (225,270)')
ax.scatter(135, 180, marker='+', s=85, label='停止格子目標 (135,180)')
ax.set_title('機体A / 1800 mm/s / 右135°入口・次の1回\n半径40 / Lstart7.85 / Lend34.81 mm / 補正＋3.5°')
ax.text(.02, .02, '前走90 mm・後走127.28 mm・吸引650\n図は名目中心のみ。穴・接地点・車幅・停止余裕は実物で確認。',
        transform=ax.transAxes, fontsize=9, bbox=dict(facecolor='white', alpha=.9, edgecolor='lightgray'))
ax.legend(loc='upper left', fontsize=8)
fig.savefig(out / 'start_placement_guide.png', dpi=150)
plt.close(fig)
print('Placement guide saved; hardware operations: 0')
