"""B1600右V90の初回を保存資料から準備する。実機への指令は送らない。"""
from pathlib import Path
from datetime import datetime, timezone
import ast
import copy
import hashlib
import json
import re
import shutil
import sys

import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
TEMPLATE = ROOT/'tools/logs/pixel_camera/turn1600_B_outr135_20261120_v1/baseline01'
BASE = ROOT/'tools/logs/pixel_camera/turn1600_B_rv90_20261120_v1'
STAGE = BASE/'baseline01'
RID = 'B_RV90_1600_baseline_green_20261120_001'
SOURCE = ROOT/'Params/Param_B/turn_1600/turn_1600.cpp'
HISTORY = ROOT/'tools/turn_analysis/20261114_B_turn500_seed_from_A/B_rV90_start315_90_plan.json'
PHOTO = ROOT/'tools/logs/pixel_camera/turn1600_B_inr135_20261120_v1/angle_cm0p4_01/local_execution/prepare_B_INR135_1600_cm0p4_green_20261120_001/static_check.jpg'
read = lambda path: json.loads(path.read_text(encoding='utf-8-sig'))


def save(path, value):
    with path.open('x', encoding='utf-8') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')


assert not BASE.exists(), '既存測定資料は上書きしない'
source = re.sub(r'/\*.*?\*/|//[^\n]*', '', SOURCE.read_text(encoding='utf-8'), flags=re.S)
raw = re.search(r'const t_turn_param_table slalom_RV90_1600_table\s*=\s*\{([^}]+)\}', source)[1].split(',')
assert len(raw) == 7 and raw[5].strip() == 'Turn_R'
values = [float(value.strip().removesuffix('f')) for value in raw[:5] + raw[6:]]
assert values == [1.6, -40., 2.2, 20.83, -90., 1.6]
candidate = copy.deepcopy(read(TEMPLATE/'candidate.settings.json'))
candidate.update(motion_name='r_v90', turn_count=1)
candidate['parameters'] = dict(zip(('velo', 'r_min', 'lstart', 'lend', 'degree', 'correction'), values), pre_accel=6.5)
for prefix in ('sp', 'om'):
    gains = re.search(r'const t_pid_gain '+prefix+r'_gain_turnV90_1600\s*=\s*\{([^}]+)\}', source)[1].split(',')
    candidate['pid'].update(dict(zip((prefix+'_'+term for term in ('kp', 'ki', 'kd')),
                                    (float(value.strip().removesuffix('f')) for value in gains))))
candidate['source'] = 'B現行turn1600右V90の初回。FFは同日のB1600既定値を仮置きし、実行前に対象プリセットを読み戻して照合する。'
STAGE.mkdir(parents=True)
config = read(TEMPLATE/'config.json')
for key, name in [('reference', 'marker_reference_B_current.json'), ('geometry', 'body_geometry_B_green_front_s35_v3.json')]:
    shutil.copyfile(config[key], BASE/name)
    config[key] = str(BASE/name)
save(STAGE/'config.json', config)
save(STAGE/'candidate.settings.json', candidate)

# 既存の1回実行・実機モード/FF照合を再利用する。位置照合は共通実装の静止品質判定を使う。
wrapper = (TEMPLATE/'run_recorded_once.py').read_text(encoding='utf-8-sig')
tree = ast.parse(wrapper)
lines = wrapper.splitlines(keepends=True)
remove = []
for node in tree.body:
    if isinstance(node, ast.FunctionDef) and node.name == 'dense_static_check':
        remove.append((node.lineno-1, node.end_lineno))
    if isinstance(node, ast.ClassDef) and node.name == 'RecordedService':
        method = next(child for child in node.body if isinstance(child, ast.FunctionDef) and child.name == 'perform')
        remove.append((method.lineno-1, method.end_lineno))
for start, end in sorted(remove, reverse=True):
    del lines[start:end]
wrapper = ''.join(lines).replace('tuning.static_check = dense_static_check\n', '')
for before, after in [
    ('B_OUTR135_1600_baseline_green_20261120_001', RID),
    ('B135度出口の実機FF', 'B右V90の実機FF'),
    ('Current operator readiness for B1600 out_r135; coarse placement then measured-entry-relative review',
     'Current operator readiness for B1600 r_v90; standard static quality gate then measured-entry-relative review'),
    ('x_mm=180., y_mm=45., heading_deg=45.', 'x_mm=315., y_mm=90., heading_deg=135.'),
    ('position_tolerance_mm=5., heading_tolerance_deg=2.', 'position_tolerance_mm=10., heading_tolerance_deg=5.'),
    ('placement, None, 20.', 'placement, 25., None'),
]:
    assert wrapper.count(before) == 1, before
    wrapper = wrapper.replace(before, after)
compile(wrapper, '<B1600 RV90 wrapper>', 'exec')
(STAGE/'run_recorded_once.py').write_text(wrapper, encoding='utf-8', newline='\n')

placement = dict(x_mm=315., y_mm=90., heading_deg=135., position_tolerance_mm=10., heading_tolerance_deg=5.)
plan = dict(prepared_at=datetime.now(timezone.utc).isoformat(), mouse='B', motion='r_v90', preset_mm_s=1600,
    count=1, parameters=candidate['parameters'], suction=candidate['suction'], placement=placement,
    front_colour='green', rear_colour='red', camera_height_mm=1500., camera_settings=config['camera_settings'],
    pre_run_mm=float(90*np.sqrt(2)), post_run_mm=float(45*np.sqrt(2)),
    effective_pre_accel_m_s2=float(1.6**2/(2*.09*np.sqrt(2))), post_diagonal_accel_m_s2=25.,
    stopping_distance_at25_mm=51.2,
    relative_Lend_target_right_forward_mm=[float(45*np.sqrt(2)),float(135*np.sqrt(2))],
    relative_stop_target_right_forward_mm=[float(90*np.sqrt(2)),float(135*np.sqrt(2))],
    target_Lend_board_mm=[135.,180.], target_stop_board_mm=[90.,135.], target_exit_heading_board_deg=225.,
    historical_placement_source=str(HISTORY),
    placement_basis='B500右V90で3本の実走実績がある315/90mm・左下135度を起点とする。速度1600・半径40・前緑への変更後の穴/全接地点/車幅/旋回/停止余裕は今回確認する。',
    static_policy='共通pixel_camera_static_pose.pyで今回動画の各フレームArUco登録と静止安定度・位置/方向を照合する。',
    reference_policy='走行の位置評価は各フレームで実復号した登録ArUco2個以上。全走行の欠測と誤認を検査する。',
    evaluation_scope='前後マーカー中点の実測入口相対軌道と盤面固定差を区別。車軸の絶対幾何・機体BのEKF係数は未校正。',
    ekf_scope='モードを実機読取りして固定比較する。B・斜めの適用/同定条件は最新実装を確認し、不足入力を既定値で埋めない。',
    current_placement_confirmed=False, current_clearance_confirmed=False, current_hands_clear_confirmed=False,
    operator_ready=False, power_state='CURRENT_USER_CONFIRMATION_PENDING', next_request_id=RID,
    execution_pending=False, execution_commands_sent=0, settings_write_commands_sent=0,
    source_params_changed=False, firmware_write_performed=False,
    candidate=str(STAGE/'candidate.settings.json'), config=str(STAGE/'config.json'),
    guide=str(BASE/'start_placement_guide.jpg'),
    source_hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (SOURCE,HISTORY)},
    camera_health_json=str(ROOT/'tmp/B_v90_1600_camera_health_20261120_02.json'))
save(BASE/'preparation.json', plan)
save(BASE/'resume_state.json', dict(plan, state='PREPARED_NOT_EXECUTED', ble_connected=False))
save(STAGE/'placement_plan.json', plan)
sys.path[:0] = [str(ROOT/'tools'), str(ROOT/'tools/analysis')]
from pixel_camera_tuning_mcp import McpTuningService
payload = McpTuningService(BASE/'offline_validation', {}).prepare(str(STAGE/'config.json'), str(STAGE/'candidate.settings.json'), placement, 25., None)
save(BASE/'offline_validation.json', dict(state='VALIDATED_NOT_EXECUTED', payload=payload, hardware_operations=0))

from video_marker_pose import marker_registration, registration_quad, pose_world_corners, board_to_pose_transform, transform_points
frame = cv2.imread(str(PHOTO))
reference = read(Path(config['reference']))
registration = marker_registration(frame, reference)
quad = registration_quad(registration)
assert quad is not None and len(set(registration['detected_marker_ids'])) >= 2
matrix = cv2.getPerspectiveTransform(pose_world_corners(reference['board_side_mm']), quad) @ board_to_pose_transform(reference)
start = np.array([315.,90.])
direction = np.array([-1.,1.])/np.sqrt(2)
points = transform_points(np.array([start,start+direction*25,start-direction*25]), matrix)
route = transform_points(np.array([start,[225.,180.],[135.,180.],[90.,135.]]), matrix)
plt.rcParams['font.family'] = 'Yu Gothic'
fig, ax = plt.subplots(figsize=(12.8,7.2), layout='constrained')
ax.imshow(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
ax.plot(route[:,0], route[:,1], '--', c='cyan', lw=1.5, label='予定の接続点（実軌道ではありません）')
ax.plot(points[[1,2],0], points[[1,2],1], c='white', lw=2)
ax.scatter(*points[0], s=240, marker='+', c='cyan', linewidths=2)
ax.scatter(*points[1], s=120, c='lime', edgecolors='black', label='前：緑（左下135度）')
ax.scatter(*points[2], s=90, c='red', edgecolors='white', label='後：赤')
ax.annotate('開始中点 (315,90) mm\n左から7マス・上から2マス', xy=points[0], xytext=(850,230),
    color='white', fontsize=12, arrowprops=dict(arrowstyle='->',color='cyan'),
    bbox=dict(facecolor='black',alpha=.8,edgecolor='none'))
ax.set_title('機体B・turn1600 右V90：両方OFFで配置\n保存写真への案内です。今回の位置・穴や全幅の干渉は別途確認します。')
ax.legend(loc='lower left'); ax.axis('off')
fig.savefig(BASE/'start_placement_guide.jpg',dpi=130); plt.close(fig)
(BASE/'README.md').write_text('機体B・turn1600右V90の単体初回準備。現行値：半径−40 mm、Lstart 2.20 mm、Lend 20.83 mm、補正＋1.60度、吸引duty650。\n\n'
    '開始候補は中点315/90 mm、前緑が左下135度。45 mm格子で左から7マス・上から2マス。B500の実使用位置を参照するが、今回の高速条件での全接地点・車幅・穴・旋回と停止余裕を確認する。\n\n'
    '斜め前走127.28 mm・後走63.64 mm。後走加減速度は25 m/s²で読み戻し照合する（1600 mm/sからの停止距離51.2 mm）。前走の実効加速度は10.06 m/s²。\n\n'
    '通常起動はオフライン検証だけ。今回の配置・干渉なし・両方ON・手離し確認後にbaseline01/run_recorded_once.py --operator-readyで1回実行する。要求IDは再送しない。取得後の解析中は両方OFF。\n', encoding='utf-8')
print(json.dumps(dict(state='VALIDATED_NOT_EXECUTED',folder=str(BASE),request_id=RID,parameters=candidate['parameters'],hardware_operations=0),ensure_ascii=False))
