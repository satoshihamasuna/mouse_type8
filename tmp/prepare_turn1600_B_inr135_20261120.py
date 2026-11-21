"""B1600右135度入口の初回を保存資料だけで準備。実機へ接続しない。"""
from pathlib import Path
from datetime import datetime, timezone
import copy
import hashlib
import json
import re
import shutil
import sys

ROOT = Path(__file__).resolve().parents[1]
OLD = ROOT / 'tools/logs/pixel_camera/turn1600_B_r180_20261120_v1'
TEMPLATE = OLD / 'angle_c0p5_s35_02'
BASE = ROOT / 'tools/logs/pixel_camera/turn1600_B_inr135_20261120_v1'
STAGE = BASE / 'baseline01'
REQUEST = 'B_INR135_1600_baseline_green_20261120_001'
SOURCE = ROOT / 'Params/Param_B/turn_1600/turn_1600.cpp'


def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def save(path, value):
    with path.open('x', encoding='utf-8', newline='\n') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')


assert not BASE.exists(), '既存資料は上書きしない'
source = re.sub(r'/\*.*?\*/|//[^\n]*', '', SOURCE.read_text(encoding='utf-8'), flags=re.S)
raw = re.search(r'const t_turn_param_table slalom_inR135_1600_table\s*=\s*\{([^}]+)\}', source)[1]
values = [part.strip() for part in raw.split(',')]
assert len(values) == 7 and values[5] == 'Turn_R'
numeric = [float(value.removesuffix('f')) for value in values[:5] + values[6:]]
assert numeric == [1.6, -42.5, 5.1, 20.56, -135., 2.1]
assert re.search(r'param_inR135_1600\s*=\s*\{&slalom_inR135_1600_table,&sp_gain_turnIn135_1600,&om_gain_turnIn135_1600\}', source)
pid = {}
for suffix, prefix in [('sp', 'sp'), ('om', 'om')]:
    raw_gain = re.search(r'const t_pid_gain ' + suffix + r'_gain_turnIn135_1600\s*=\s*\{([^}]+)\}', source)[1]
    pid.update(dict(zip([prefix + '_' + term for term in ['kp', 'ki', 'kd']],
                        [float(value.strip().removesuffix('f')) for value in raw_gain.split(',')])))
observed_path = TEMPLATE / 'local_execution/run_B_R180_1600_c0p5_s35_green_20261120_002/saved_1ms.settings.json'
observed = read(observed_path)
observed = observed.get('actual_settings', observed)
assert observed['mouse'] == 'B' and observed['preset_speed_mm_s'] == 1600
candidate = {key: copy.deepcopy(observed[key]) for key in ('schema_version', 'mouse',
    'mouse_identity_source', 'motion_name', 'preset_speed_mm_s', 'turn_count', 'parameters',
    'pid', 'feedforward', 'suction')}
candidate.update(motion_name='in_r135', turn_count=1, pid=pid)
candidate['parameters'].update(dict(zip(['velo', 'r_min', 'lstart', 'lend', 'degree', 'correction'], numeric)), pre_accel=6.5)
candidate['source'] = '機体Bの現行turn1600右135度入口。FFは同日のB1600実機既定値から引用し、走行前に対象プリセットと照合。初回は幾何・PID/FFを変更せず測定する。'
STAGE.mkdir(parents=True)
config = read(TEMPLATE / 'config.json')
for key, filename in [('reference', 'marker_reference_B_current.json'),
                       ('geometry', 'body_geometry_B_green_front_s35_v3.json')]:
    shutil.copyfile(Path(config[key]), BASE / filename)
    config[key] = str(BASE / filename)
assert config['mouse'] == 'B' and config['height_mm'] == 1500
save(STAGE / 'config.json', config)
save(STAGE / 'candidate.settings.json', candidate)
for filename in ('coarse_placement_check.py', 'static_pose_after_settle.py'):
    shutil.copyfile(TEMPLATE / filename, STAGE / filename)
wrapper = (TEMPLATE / 'run_recorded_once.py').read_text(encoding='utf-8')
assert wrapper.count('B_R180_1600_c0p5_s35_green_20261120_002') == 1
wrapper = wrapper.replace('B_R180_1600_c0p5_s35_green_20261120_002', REQUEST)
assert wrapper.count('placement, None, 20.') == 1
wrapper = wrapper.replace('placement, None, 20.', 'placement, 20., None')
wrapper = wrapper.replace('B180度の実機FF', 'B135度入口の実機FF')
wrapper = wrapper.replace('User currently powered and explicitly requested one run after approximate placement check, then measured-entry-relative video correction',
    'Current operator readiness for B1600 in_r135; coarse placement then measured-entry-relative review')
compile(wrapper, '<prepared wrapper>', 'exec')
(STAGE / 'run_recorded_once.py').write_text(wrapper, encoding='utf-8', newline='\n')
placement = dict(x_mm=315., y_mm=135., heading_deg=90., position_tolerance_mm=5., heading_tolerance_deg=2.)
plan = dict(prepared_at=datetime.now(timezone.utc).isoformat(), mouse='B', motion='in_r135', preset_mm_s=1600,
    count=1, placement=placement, planned_start_board_mm=[315., 135.], planned_heading_image_deg=90.,
    front_colour='green', rear_colour='red', camera_height_mm=1500., camera_settings=config['camera_settings'],
    pre_run_type='straight', pre_run_mm=90., post_run_type='diagonal', post_run_mm=127.27922061357854,
    requested_turn_pre_accel_m_s2=6.5, effective_pre_accel_m_s2=1.6**2/(2*.09),
    requested_post_diagonal_accel_m_s2=20., stop_distance_at20_mm=64.,
    relative_Lend_target_right_forward_mm=[90., 135.], relative_stop_target_right_forward_mm=[180., 45.],
    target_Lend_board_mm=[225., 270.], target_stop_board_mm=[135., 180.], target_exit_heading_board_deg=225.,
    placement_basis='同じ格子盤のA1600右135度入口と直近B1600右180度で使用した315/135mm近傍を候補にする。B135度の全接地点・全幅・穴・旋回・停止余裕は今回の確認が必要。',
    placement_policy='概略位置を照合後、走行動画の実測入口に対する相対形状と盤面固定差を区別して評価。既存B1600運用を引き継ぐ。',
    precise_static_pose_gate_used=False, coarse_position_tolerance_mm=10., coarse_heading_tolerance_deg=5.,
    current_placement_confirmed=False, current_clearance_confirmed=False, current_hands_clear_confirmed=False,
    operator_ready=False, power_state='CURRENT_USER_CONFIRMATION_PENDING', hole_clearance_verified=False,
    execution_pending=False, execution_commands_sent=0, settings_write_commands_sent=0,
    next_request_id=REQUEST, next_request_sent=False, candidate=str(STAGE/'candidate.settings.json'),
    config=str(STAGE/'config.json'), source_actual_settings=str(observed_path), source_params_changed=False,
    firmware_write_performed=False, workflow_order=['in_r135', 'out_r135'],
    runtime_modes='未読。実行前にwall_ekf/wall_edge/turn_ekfを記録。BのEKF未校正。',
    evaluation_scope='前後マーカー中点の入口相対形状。盤面固定の設置/出口/停止差を併記。機軸/車軸の絶対校正と連続接続の合格は別。',
    reference_policy='各評価フレームで実復号したArUco2個以上を使用。走行全区間の欠測・誤認・登録品質を別に検査。',
    ekf_scope='Bの幾何・校正未完了と解析ツールのA制約を保持。識別できない係数を既定値で埋めない。',
    source_hashes={str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in [SOURCE, ROOT/'Params/Param_B/run_param_B.cpp', ROOT/'System/Src/myshell.cpp', ROOT/'Task/Src/motion.cpp', observed_path]},
    next_action='現在の撮影条件/前緑後赤を確認し、両方OFFで315/135mmへ配置。現在の穴・接地点・全幅・旋回・停止余裕を確認してからON・手離しを確認し、固有IDで1回だけ撮影走行。')
save(STAGE / 'placement_plan.json', plan)
save(BASE / 'preparation.json', plan)
save(BASE / 'resume_state.json', dict(plan, state='PREPARED_NOT_EXECUTED', ble_connected=False))
sys.path.insert(0, str(ROOT/'tools'))
from pixel_camera_tuning_mcp import McpTuningService
service = McpTuningService(BASE / 'offline_validation', {})
payload = service.prepare(str(STAGE/'config.json'), str(STAGE/'candidate.settings.json'), placement, 20., None)
assert payload['settings'] == candidate and payload['post_diagonal_accel_m_s2'] == 20.
save(BASE/'offline_validation.json', dict(state='VALIDATED_NOT_EXECUTED', payload=payload,
    physical_execution=False, all_source_hashes_unchanged=all(hashlib.sha256((ROOT/path).read_bytes()).hexdigest() == value
        for path, value in plan['source_hashes'].items())))
(BASE/'README.md').write_text('機体B・1600右135度入口から右135度出口の順で調整。\n\n'
    '初回は現行値、半径−42.5 mm・Lstart5.10 mm・Lend20.56 mm・補正＋2.10度。'
    'BのPID/FFと吸引650、前走90 mm・後斜め走127.28 mm、後走加減速度20 m/s²。'
    '前緑/後赤・高さ1500 mm・EV−1.5/1.8倍の既存条件を候補として複製。現在の条件は操作者確認待ち。\n\n'
    '配置候補はマーカー中点315/135 mm・緑前が下90度。盤左上が原点、45 mm格子。'
    '穴・段差・全接地点・車幅・旋回と停止余裕は今回確認する。概略配置の照合と走行全フレームの品質検証を区別し、入口相対形状と盤面固定差を併記する。\n\n'
    'run_recorded_once.pyの通常起動は検証のみ。今回の配置・干渉なし・両方ON・手離しを確認後に--operator-readyで1回実行する。'
    '固有IDは再送しない。取得後の長い解析中は両方OFF。設定と走行コマンドはまだ0件、ファーム/パラメータソース未変更。\n', encoding='utf-8')
print(json.dumps(dict(state='VALIDATED_NOT_EXECUTED', folder=str(BASE), request_id=REQUEST,
    parameters=candidate['parameters'], robot_commands_sent=0), ensure_ascii=False))
