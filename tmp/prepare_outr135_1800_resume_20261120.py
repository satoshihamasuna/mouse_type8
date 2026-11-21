"""A1800右135出口の未実行baselineを現在の撮影・大まかな配置確認で再開準備する。"""
from pathlib import Path
from datetime import datetime, timezone
import ast
import hashlib
import json
import re
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tools/logs/pixel_camera/turn1800_A_outr135_20261120_v1'
STAGE = BASE / 'baseline01'
OUT = BASE / 'resume_20261120_v1'
RID = 'A_OUTR135_1800_baseline_20261120_001'
read = lambda p: json.loads(p.read_text(encoding='utf-8-sig'))
assert not (STAGE / 'local_execution/jobs' / (RID + '.json')).exists()
candidate = read(STAGE / 'candidate.settings.json')
assert candidate['mouse'] == 'A' and candidate['motion_name'] == 'out_r135'
assert candidate['preset_speed_mm_s'] == 1800 and candidate['turn_count'] == 1
p = candidate['parameters']
assert p == dict(velo=1.8, r_min=-41., lstart=15.61, lend=46.65, degree=-135., correction=0., pre_accel=6.5)
source = ROOT / 'Params/Param_A/turn_1800/turn_1800.cpp'
code = re.sub(r'/\*.*?\*/|//[^\n]*', '', source.read_text(encoding='utf-8'), flags=re.S)
assert '1.80f,-41.0f,15.61,46.65,-135.0f,Turn_R' in re.sub(r'\s+', '', code)
config = read(STAGE / 'config.json')
assert config['height_mm'] == 1500. and config['mouse'] == 'A'
for name in ('reference', 'geometry'):
    assert Path(config[name]).is_file()
OUT.mkdir(exist_ok=False)
for path in (BASE / 'preparation.json', STAGE / 'placement_plan.json', STAGE / 'run_recorded_once.py'):
    (OUT / ('before_' + path.name)).write_bytes(path.read_bytes())
wrapper_path = STAGE / 'run_recorded_once.py'
wrapper = wrapper_path.read_text(encoding='utf-8-sig')
old = 'position_tolerance_mm=5., heading_tolerance_deg=2.'
assert wrapper.count(old) == 1
wrapper = wrapper.replace(old, 'position_tolerance_mm=10., heading_tolerance_deg=5.')
assert 'placement, None, 20.)' in wrapper and 'heading_deg=45.0' in wrapper
ast.parse(wrapper)
wrapper_path.write_text(wrapper, encoding='utf-8', newline='\n')
plan = read(STAGE / 'placement_plan.json')
plan.update(prepared_at=datetime.now(timezone.utc).isoformat(),
    state='UNEXECUTED_A1800_OUTR135_BASELINE_READY_FOR_CURRENT_OPERATOR_CONFIRMATION',
    static_apparent_reference_heading_deg=45.,
    static_reference_basis='Nominal planned45deg; older observed44.15deg retained only in historical snapshots.',
    initial_placement_policy='User requested approximate check; position10mm and apparent heading5deg. Per-frame ArUco/observations unchanged.',
    source_profile_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    source_params_changed=False, next_candidate_executed=False, execution_pending=False,
    operator_ready=False, current_placement_confirmed=False, current_clearance_confirmed=False,
    current_hands_clear_confirmed=False, power_state='Both_OFF_last_confirmed_after_in135',
    guide=str(OUT / 'start_placement_guide.png'), next_request_id=RID, request_id=RID,
    candidate=str(STAGE / 'candidate.settings.json'), config=str(STAGE / 'config.json'))
plan['placement'] = dict(x_mm=180., y_mm=135., heading_deg=45., position_tolerance_mm=10., heading_tolerance_deg=5.)
for path in (STAGE / 'placement_plan.json', BASE / 'preparation.json'):
    path.write_text(json.dumps(plan, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')

table_text = (ROOT / 'Params/turn_table.h').read_text(encoding='utf-8')
wave = np.fromstring(re.search(r'accel_table\[1001\]\s*=\s*\{([^}]+)\}', table_text)[1], sep=',')
integral = float(re.search(r'accel_Integral\s*=\s*([.\d]+)', table_text)[1])
duration = np.deg2rad(135) / (1800 / 41 * integral)
time = np.linspace(0, duration, 1001)
omega = 1800 / 41 * wave
theta = np.r_[0., np.cumsum((omega[:-1]+omega[1:])/2 * np.diff(time))]
middle = (theta[:-1]+theta[1:])/2
right = np.r_[0., np.cumsum(1800*np.sin(middle)*np.diff(time))]
forward = 90*np.sqrt(2)+p['lstart']+np.r_[0., np.cumsum(1800*np.cos(middle)*np.diff(time))]
direction = np.array([np.sin(theta[-1]), np.cos(theta[-1])])
lend = np.array([right[-1], forward[-1]])+p['lend']*direction
stop = lend+90*direction
local = np.vstack(([0., 0.], [0., 90*np.sqrt(2)+p['lstart']], np.c_[right, forward], lend, stop))
h = np.deg2rad(45)
basis = np.array([[-np.sin(h), np.cos(h)], [np.cos(h), np.sin(h)]])
board = local @ basis.T + [180., 135.]
plt.rcParams['font.family'] = 'Yu Gothic'
fig, ax = plt.subplots(figsize=(7, 7.5), layout='constrained')
ax.set(xlim=(-8, 368), ylim=(368, -8), aspect='equal', xlabel='盤面右 [mm]', ylabel='盤面下 [mm]')
ax.set_xticks(np.arange(0, 361, 45)); ax.set_yticks(np.arange(0, 361, 45)); ax.grid(alpha=.35)
for x in range(0, 361, 90):
    ax.scatter([x]*5, range(0, 361, 90), s=17, color='#3e474b')
ax.plot(board[:, 0], board[:, 1], label='初期値の名目中心軌跡（未実走）')
ax.scatter(180, 135, c='black', marker='+', s=130)
front = np.array([180., 135.])+25*np.array([np.cos(h), np.sin(h)])
rear = np.array([180., 135.])-25*np.array([np.cos(h), np.sin(h)])
ax.plot([rear[0], front[0]], [rear[1], front[1]], c='black')
ax.scatter(*front, c='red', s=65, label='前：赤・右下向き')
ax.scatter(*rear, c='gold', edgecolors='black', s=65, label='後：黄')
ax.annotate('開始中点 (180,135) mm', xy=(180, 135), xytext=(20, 70), arrowprops=dict(arrowstyle='->'))
ax.scatter(225, 315, marker='x', s=65, label='Lend格子目標 (225,315)')
ax.scatter(135, 315, marker='+', s=85, label='停止格子目標 (135,315)')
ax.set_title('機体A / 1800 mm/s / 右135°出口・初回\n半径41 / Lstart15.61 / Lend46.65 mm / 補正0°')
ax.text(.02, .02, '斜め前走127.28 mm・直線後走90 mm・吸引650\n穴・全接地点・車幅・旋回と停止の空きを実物で確認。\n図は名目中心のみ。初期配置は大まかに確認。',
    transform=ax.transAxes, fontsize=9, bbox=dict(facecolor='white', alpha=.9, edgecolor='lightgray'))
ax.legend(loc='upper right', fontsize=8)
fig.savefig(OUT / 'start_placement_guide.png', dpi=150)
plt.close(fig)
report = dict(mouse='A', motion='out_r135', preset_mm_s=1800, request_id=RID,
    candidate=candidate, placement=plan['placement'], config=config,
    nominal_center_bounds_board_mm=[board.min(0).tolist(), board.max(0).tolist()],
    nominal_stop_board_mm=board[-1].tolist(), physical_clearance_verified=False,
    source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    hardware_operations=0, firmware_parameters_changed=False, operator_ready=False,
    next_step='Current placement, clearance, both ON and hands-clear confirmation, then one recorded baseline.',
    guide=str(OUT / 'start_placement_guide.png'))
with (OUT / 'resume_plan.json').open('x', encoding='utf-8') as file:
    json.dump(report, file, ensure_ascii=False, indent=2, allow_nan=False)
print(json.dumps(dict(prepared=True, nominal_center_bounds=report['nominal_center_bounds_board_mm'],
    nominal_stop=report['nominal_stop_board_mm'], hardware_operations=0), ensure_ascii=False))
