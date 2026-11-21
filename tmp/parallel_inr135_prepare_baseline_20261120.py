"""機体A turn1800右135度入口の初回を保存資料だけで準備する。"""
from pathlib import Path
import ast
import hashlib
import json
import re
import shutil

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tools/logs/pixel_camera/turn1800_A_inr135_20261120_v1'
OUT = BASE / 'baseline01'
TEMPLATE_BASE = ROOT / 'tools/logs/pixel_camera/turn1800_A_inr45_20261120_v1'
TEMPLATE = TEMPLATE_BASE / 'baseline01'
SOURCE = ROOT / 'Params/Param_A/turn_1800/turn_1800.cpp'
FLAGS = ROOT / 'Params/Param_A/run_param_A.h'
REQUEST = 'A_INR135_1800_baseline_20261120_001'
PRIOR_BASE = ROOT / 'tools/logs/pixel_camera/turn1600_A_inr135_20261119_v1'
PRIOR_PLAN = PRIOR_BASE / 'next_Ls5p1_c2p1/placement_plan.json'
OPERATOR = ROOT / 'tools/logs/pixel_camera/mcp_tuning/operator_results/A_INR135_1600_Ls5p1_Le20p56_c2p1_20261119_MCP045.json'
CURRENT_PLACE = ROOT / 'tools/logs/pixel_camera/turn1800_A_r180_20261120_v1/finish02_01/placement_plan.json'


def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def save(path, value):
    with path.open('x', encoding='utf-8', newline='\n') as file:
        json.dump(value, file, ensure_ascii=False, indent=2, allow_nan=False)
        file.write('\n')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def declaration(source, kind, name):
    match = re.search(r'const\s+' + kind + r'\s+' + name + r'\s*=\s*\{([^}]+)\}', source)
    assert match, name
    return match[0] + ';', [float(item.strip().removesuffix('f')) for item in match[1].split(',')]


assert not BASE.exists(), '新規出力先のみ使用する'
flag = int(re.search(r'^#define PARAM_A_USE_PINK_LED_TURN_LENGTHS (\d+)', FLAGS.read_text(encoding='utf-8'), re.M)[1])
assert flag == 0
source_text = SOURCE.read_text(encoding='utf-8')
# コメントを除いた後、対象の#ifの#else（flag0）だけを抽出する。
source = re.sub(r'/\*.*?\*/|//[^\n]*', '', source_text, flags=re.S)
block = next(m.group(2) for m in re.finditer(
    r'#if PARAM_A_USE_PINK_LED_TURN_LENGTHS\s+(.*?)#else\s+(.*?)#endif', source, re.S)
    if 'slalom_inR135_1800_table' in m.group(2))
table_match = re.search(r'const t_turn_param_table slalom_inR135_1800_table\s*=\s*\{([^}]+)\}', block)
raw_table = [item.strip() for item in table_match[1].split(',')]
assert raw_table[-1] == 'Turn_R' and len(raw_table) == 6
velo, r_min, lstart, lend, degree = [float(value.removesuffix('f')) for value in raw_table[:5]]
assert (velo, r_min, lstart, lend, degree) == (1.8, -40.0, 22.85, 34.81, -135.0)
sp_decl, sp = declaration(source, 't_pid_gain', 'sp_gain_turnIn135_1800')
om_decl, om = declaration(source, 't_pid_gain', 'om_gain_turnIn135_1800')
ff_decl, ff = declaration(source, 't_ff_gain', 'ff_gain_in135_R_1800')
assert sp == [2.0, .05, 0] and om == [.1, .002, 0]
assert len(ff) == 10
assert re.search(r'param_inR135_1800\s*=\s*\{&slalom_inR135_1800_table,&sp_gain_turnIn135_1800,&om_gain_turnIn135_1800,&ff_gain_in135_R_1800\}', source)
left_match = re.search(r'slalom_inL135_1800_table\s*=\s*\{([^}]+)\}', block)
left = [item.strip() for item in left_match[1].split(',')]
assert float(left[1].removesuffix('f')) == 40 and float(left[4].removesuffix('f')) == 135 and left[-1] == 'Turn_L'
prior = read(PRIOR_PLAN)
assert read(OPERATOR)['contact_detected'] is False and read(OPERATOR)['both_powered_off'] is True
current = read(CURRENT_PLACE)
heading = current['static_apparent_reference_heading_deg']
assert current['planned_start_board_mm'] == [315., 135.]

BASE.mkdir()
OUT.mkdir()
for filename in ('body_geometry_A_yellow_v2.json', 'marker_reference_A_current.json'):
    shutil.copyfile(TEMPLATE_BASE / filename, BASE / filename)
config = read(TEMPLATE / 'config.json')
config.update(reference=str(BASE / 'marker_reference_A_current.json'), geometry=str(BASE / 'body_geometry_A_yellow_v2.json'))
assert config['height_mm'] == 1500.
assert config['camera_settings'] == dict(ev=-1.5, torch=False, focus_lock=True, zoom_ratio=1.8)
save(OUT / 'config.json', config)

candidate = dict(schema_version=1,
    source='未実行の機体A turn1800 in_r135初回。PARAM_A_USE_PINK_LED_TURN_LENGTHS=0の形状専用テーブル・PID・FFをソースから抽出。A1600の調整値は使用しない。',
    motion_name='in_r135',
    parameters=dict(velo=velo, r_min=r_min, lstart=lstart, lend=lend, degree=degree, correction=0., pre_accel=6.5),
    pid=dict(zip(('sp_kp', 'sp_ki', 'sp_kd', 'om_kp', 'om_ki', 'om_kd'), sp + om)),
    feedforward=dict(zip(('ff_sp_velo', 'ff_sp_accel', 'ff_sp_bias', 'ff_om_velo', 'ff_om_accel', 'ff_om_decel', 'ff_om_bias',
                         'ff_om_jerk', 'ff_sp_turn_accel_mag', 'ff_sp_turn_velo_sq'), ff)),
    suction=dict(enabled=True, duty=650), preset_speed_mm_s=1800, turn_count=1, mouse='A', mouse_identity_source='user_supplied')
save(OUT / 'candidate.settings.json', candidate)
placement = dict(x_mm=315., y_mm=135., heading_deg=heading, position_tolerance_mm=5., heading_tolerance_deg=2.)
plan = dict(mouse='A', motion='in_r135', preset_mm_s=1800, count=1,
    planned_start_board_mm=[315., 135.], planned_heading_image_deg=90.,
    static_apparent_reference_heading_deg=heading,
    static_reference_basis='現在の撮影・同じ315/135mm地点で使用したA1800右180度の見かけマーカー参照。機軸の絶対yawではない。',
    static_reference_source=str(CURRENT_PLACE), placement=placement, front_colour='red', rear_colour='yellow',
    pre_run_type='straight', pre_run_mm=90., requested_pre_accel_m_s2=6.5, effective_pre_accel_m_s2=18.,
    post_run_type='diagonal', post_run_mm=2 * 90 / np.sqrt(2), requested_post_diagonal_accel_m_s2=20.,
    stopping_distance_at20_mm=81.,
    nominal_Lend_from_start_right_forward_mm=[90., 135.], nominal_stop_right_forward_mm=[180., 45.],
    reference_nominal_Lend_board_mm=[225., 270.], reference_nominal_stop_board_mm=[135., 180.], nominal_exit_heading_board_deg=225.,
    camera_height_mm=1500., camera_settings=config['camera_settings'],
    placement_basis='A1600 in_r135で実走した315/135mm近傍の同じ格子地域を再使用する提案。1600の設定値・軌道余裕は1800へ転用しない。',
    historical_placement_plan=str(PRIOR_PLAN), historical_placement=prior['placement'], historical_operator_no_contact_record=str(OPERATOR),
    source_profile=str(SOURCE), source_profile_sha256=digest(SOURCE), source_flag_name='PARAM_A_USE_PINK_LED_TURN_LENGTHS', source_flag_value=flag,
    source_params_changed=False, next_candidate_executed=False, execution_pending=False, operator_ready=False,
    current_placement_confirmed=False, current_clearance_confirmed=False, current_hands_clear_confirmed=False,
    current_physical_clearance_verified=False, physical_clearance_note='穴・段差・全接地点・機体全幅・旋回の膨らみ・停止余裕を操作者が今回確認する。図は名目中心軌道のみ。',
    power_state='Both_OFF_confirmed_after_R180_finish02_by_parent', next_request_id=REQUEST,
    candidate=str(OUT / 'candidate.settings.json'), config=str(OUT / 'config.json'),
    guide=str(BASE / 'start_placement_guide.png'), time_target_minutes=20,
    workflow='今回の配置/両方ON/手離し確認と新規静止照合を経て1回のみ。保存解析はMCPとサブエージェントで他ターン取得と並行し、通電の解析待ちを作らない。',
    evaluation_scope='相対マーカー中点・実測入口方向を基本とする。機軸/車軸の絶対姿勢と盤面固定接続の合格は別に確認。',
    reference_policy='各評価フレームで実復号した登録ArUco2/3を参照。登録・観測95%以上、V2静止安定性、motion全フレームを維持。',
    analysis_plan='後マーカーが弱い条件を記録し、前マーカー基準の補助解析を併用予定。品質を下げず、補助指標をマーカー中点の採用証拠と混同しない。',
    static_sampling=dict(seconds=5., stationary_video_window_s=[3.3, 4.95], stationary_frame_stride=4, acceptance_version=2,
        observation_minimum_fraction=.95, minimum_observations=12, motion_analysis_stride=1))
save(OUT / 'placement_plan.json', plan)
save(BASE / 'preparation.json', plan)

wrapper = (TEMPLATE / 'run_recorded_once.py').read_text(encoding='utf-8')
assert wrapper.count('A_INR45_1800_baseline_20261120_001') == 1
wrapper = wrapper.replace('A_INR45_1800_baseline_20261120_001', REQUEST)
old_placement = 'placement = dict(x_mm=225., y_mm=45., heading_deg=89.50081634521484,'
assert old_placement in wrapper
wrapper = wrapper.replace(old_placement, f'placement = dict(x_mm=315., y_mm=135., heading_deg={heading},')
tree = ast.parse(wrapper)
prepare = next(node for node in ast.walk(tree) if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and node.func.attr == 'prepare')
assert ast.literal_eval(prepare.args[-2]) == 20. and ast.literal_eval(prepare.args[-1]) is None
with (OUT / 'run_recorded_once.py').open('x', encoding='utf-8', newline='\n') as file:
    file.write(wrapper)
for filename in ('static_pose_after_settle.py', 'static_settle_policy.json'):
    shutil.copyfile(TEMPLATE / filename, OUT / filename)
shutil.copyfile(TEMPLATE_BASE / 'static_stride4_validation.json', BASE / 'static_stride4_validation_source.json')

save(BASE / 'source_extraction.json', dict(mouse='A', motion='in_r135', flag=flag,
    declarations=dict(table=table_match[0] + ';', sp_pid=sp_decl, om_pid=om_decl, ff=ff_decl),
    right_signs=dict(r_min=r_min, degree=degree, direction='Turn_R'),
    left_signs=dict(r_min=40., degree=135., direction='Turn_L'),
    omitted_degree_correction_zero_initialized=True, source_params_changed=False,
    historical1600_parameters_used=False, historical_placement_only_used=True,
    historical_contact_detected=read(OPERATOR)['contact_detected'],
    input_sha256={str(path.relative_to(ROOT)):digest(path) for path in
        (SOURCE, FLAGS, ROOT / 'Params/typedef_run_param.h', ROOT / 'System/Src/myshell.cpp', PRIOR_PLAN, OPERATOR, CURRENT_PLACE,
         TEMPLATE / 'run_recorded_once.py', TEMPLATE / 'config.json', TEMPLATE_BASE / 'body_geometry_A_yellow_v2.json', TEMPLATE_BASE / 'marker_reference_A_current.json')}))

# ファーム角速度テーブルを積分した名目中心軌道を図示する。物理余裕の検証ではない。
table_text = (ROOT / 'Params/turn_table.h').read_text(encoding='utf-8')
table = np.fromstring(re.search(r'accel_table\[1001\]\s*=\s*\{([^}]+)\}', table_text)[1], sep=',')
integral = float(re.search(r'accel_Integral\s*=\s*([.\d]+)', table_text)[1])
assert len(table) == 1001
duration = abs(np.deg2rad(degree)) / (velo * 1000 / abs(r_min) * integral)
t = np.linspace(0, duration, 1001)
w = velo * 1000 / abs(r_min) * table
theta = np.r_[0., np.cumsum((w[:-1] + w[1:]) / 2 * np.diff(t))]
right = np.r_[0., np.cumsum(velo * 1000 * np.sin((theta[:-1] + theta[1:]) / 2) * np.diff(t))]
forward = 90 + lstart + np.r_[0., np.cumsum(velo * 1000 * np.cos((theta[:-1] + theta[1:]) / 2) * np.diff(t))]
turn_end = np.array([right[-1] + lend * np.sin(theta[-1]), forward[-1] + lend * np.cos(theta[-1])])
post_end = turn_end + np.array([np.sin(theta[-1]), np.cos(theta[-1])]) * plan['post_run_mm']
local = np.vstack(([0., 0.], [0., 90 + lstart], np.column_stack((right, forward)), turn_end, post_end))
board = np.column_stack((315 - local[:, 0], 135 + local[:, 1]))
save(BASE / 'nominal_path_for_placement.json', dict(method='1800 source angular table integration, no measured-path fit',
    nominal_turn_end_right_forward_mm=turn_end.tolist(), nominal_command_stop_right_forward_mm=post_end.tolist(),
    nominal_path_board_mm=board.tolist(), physical_clearance_verified=False,
    limitations=['名目中心軌道であり車幅/全接地点の掃引や実停止を保証しない。', '入口方向は盤面下90度とし、見かけ静止yaw89.26036度を物理yawとして使わない。']))
font_names = {font.name for font in font_manager.fontManager.ttflist}
plt.rcParams['font.family'] = next((name for name in ('Yu Gothic', 'Meiryo', 'MS Gothic', 'Noto Sans CJK JP') if name in font_names), 'sans-serif')
fig, ax = plt.subplots(figsize=(8, 8.7), layout='constrained')
ax.set(xlim=(-8, 368), ylim=(368, -8), aspect='equal', xlabel='盤面 X [mm]（右が正）', ylabel='盤面 Y [mm]（下が正）')
ax.set_xticks(np.arange(0, 361, 45)); ax.set_yticks(np.arange(0, 361, 45)); ax.grid(alpha=.35)
for h in np.arange(0, 361, 90):
    ax.scatter(np.full(5, h), np.arange(0, 361, 90), color='#303e43', s=18, zorder=2)
ax.plot(board[:, 0], board[:, 1], color='#187ca7', lw=2, label='1800ソースの名目中心軌道')
ax.scatter([315], [135], color='black', marker='+', s=130, zorder=5)
ax.plot([315, 315], [110, 160], color='black', lw=2)
ax.scatter([315], [110], color='#f3d65b', edgecolor='black', s=64, zorder=5, label='後ろ：黄')
ax.scatter([315], [160], color='#d54236', edgecolor='black', s=64, zorder=5, label='前：赤（盤面下へ）')
ax.annotate('開始中点 (315,135) mm\n赤が下、黄が上', xy=(315,135), xytext=(184,72), arrowprops=dict(arrowstyle='->'))
ax.scatter([225], [270], marker='x', s=70, color='#364e59', zorder=5, label='Lend格子目標 (225,270)')
ax.scatter([135], [180], marker='+', s=90, color='#765f3b', zorder=5, label='停止格子目標 (135,180)')
ax.set_title('機体A turn1800 右135°入口：初回配置案\nr=-40 mm / Lstart22.85 mm / Lend34.81 mm / 補正0°')
ax.text(.02, .02, '前走90 mm → 右135° → 後走斜め127.28 mm / 後走加速度20 m/s²\n穴・全接地点・車幅・旋回・停止余裕は今回の操作者確認が必要\n図は名目中心軌道。絶対機軸姿勢・物理クリアランスは未検証',
        transform=ax.transAxes, fontsize=9, va='bottom', bbox=dict(facecolor='white', alpha=.9, edgecolor='#ccd5da'))
ax.legend(loc='upper left', fontsize=8)
fig.savefig(BASE / 'start_placement_guide.png', dpi=150)
plt.close(fig)
print(json.dumps(dict(stage=str(OUT), request_id=REQUEST, parameters=candidate['parameters'],
    placement=placement, nominal_turn_end=turn_end.tolist(), nominal_stop=post_end.tolist()), ensure_ascii=False))
