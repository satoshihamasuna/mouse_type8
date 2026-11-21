"""B1600右45度入口の準備資料を作る。実機接続・走行はしない。"""
from pathlib import Path
import copy
import hashlib
import json
import shutil

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT/'tools/logs/pixel_camera/turn1600_B_inr45_20261120_v1'
SOURCE = ROOT/'tools/logs/pixel_camera/turn1600_B_inr135_20261120_v1'
STAGE = BASE/'baseline01'
STAGE.mkdir(parents=True, exist_ok=False)
read = lambda p: json.loads(p.read_text(encoding='utf-8-sig'))
def save(path, value):
    with path.open('x', encoding='utf-8') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')

config = read(SOURCE/'geometry_c0p4_Ls10p5_Le17p2_02/config.json')
for key in ('reference', 'geometry'):
    source = Path(config[key])
    target = BASE/source.name
    shutil.copyfile(source, target)
    config[key] = str(target)
save(STAGE/'config.json', config)
settings_source = SOURCE/'geometry_c0p4_Ls10p5_Le17p2_02/candidate.settings.json'
settings = copy.deepcopy(read(settings_source))
settings['motion_name'] = 'in_r45'
settings['parameters'] = dict(velo=1.6, r_min=-55., lstart=5.80, lend=39.86,
                              degree=-45., correction=-.90, pre_accel=6.5)
settings['pid'] = dict(sp_kp=4., sp_ki=.05, sp_kd=0., om_kp=.20, om_ki=.01, om_kd=0.)
settings['source'] = 'B turn1600現行inR45形状。FFはB共通値の準備用複製で、inR45の実機preset応答と一致を確認するまで未検証。'
save(STAGE/'candidate.settings.json', settings)
plan = dict(mouse='B', motion='in_r45', preset_mm_s=1600, count=1,
    placement=dict(x_mm=225., y_mm=45., heading_deg=90., position_tolerance_mm=5., heading_tolerance_deg=2.),
    front_colour='green', rear_colour='red', pre_run_mm=90., post_run_mm=127.27922061357854,
    requested_pre_accel_m_s2=6.5, effective_pre_accel_m_s2=1.6**2/(2*.09),
    post_diagonal_accel_m_s2=20., stop_distance_at20_mm=64.,
    relative_turn_end_right_forward_mm=[45.,90.],
    target_Lend_board_mm=[180.,225.], target_stop_board_mm=[90.,315.], exit_heading_board_deg=135.,
    placement_basis='B右45入口の使用済み225/45mm・前下向き。速度500時の実績であり1600での実走は今回確認する。',
    full_footprint_and_hole_clearance_verified=False, operator_ready=False,
    physical_power_state='OFF_REQUESTED_NOT_CONFIRMED', execution_commands_sent=0,
    settings_write_commands_sent=0, request_id='B_INR45_1600_baseline_green_20261120_001',
    request_sent=False, firmware_written=False, source_parameters_changed=False,
    evaluation='マーカー中点の入口相対軌道と盤面固定の設置/出口/停止差を分ける。各評価コマでArUco2個以上を実復号。',
    ekf='Bの幾何・校正と解析ツール対応が未完了。AのK・雑音は流用しない。',
    source_hashes={str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in (
        ROOT/'Params/Param_B/turn_1600/turn_1600.cpp', ROOT/'System/Src/myshell.cpp',
        settings_source, Path(config['reference']), Path(config['geometry']))})
save(STAGE/'placement_plan.json', plan)

# 既存のカメラ先行・実機値照合・一回限定送信を再使用する。
old = (SOURCE/'geometry_c0p4_Ls10p5_Le17p2_02/run_recorded_once.py').read_text(encoding='utf-8')
a_stage = ROOT/'tools/logs/pixel_camera/turn1800_A_inr45_20261120_v1/mid_final_repeat_01'
a_text = (a_stage/'run_recorded_once.py').read_text(encoding='utf-8')
dense = a_text[a_text.index('def dense_static_check('):a_text.index('tuning.static_check = dense_static_check')]
dense = dense.replace("    original = tuning.read_json(result)\n    checked = relative_setup_acceptance(original, result)\n    tuning.save_json(folder/'relative_setup_check.json', checked)", "    checked = tuning.read_json(result)")
start, end = old.index('def dense_static_check('), old.index('tuning.static_check = dense_static_check')
old = old[:start] + dense + old[end:]
old = old.replace('B1600 in_r135', 'B1600 in_r45').replace('B135度入口', 'B45度入口')
old = old.replace('coarse placement then measured-entry-relative review', 'current strict static check and measured-entry-relative review')
old = old.replace('B_INR135_1600_c0p4_Ls10p5_Le17p2_green_20261120_002', plan['request_id'])
old = old.replace('x_mm=315., y_mm=135.', 'x_mm=225., y_mm=45.')
(STAGE/'run_recorded_once.py').write_text(old, encoding='utf-8')
shutil.copyfile(a_stage/'static_pose_after_settle.py', STAGE/'static_pose_after_settle.py')
save(BASE/'resume_state.json', dict(state='PREPARED_NOT_EXECUTED', stage=str(STAGE),
    operator_ready=False, physical_power_state='OFF_REQUESTED_NOT_CONFIRMED',
    execution_commands_sent=0, request_id=plan['request_id'], request_sent=False,
    next_action='撮影条件が継続かと現在の配置・全接地点/全幅/穴/停止余裕・両方ON・手離しを確認後、固有IDで一回取得。'))
(BASE/'README.md').write_text(
    '# 機体B turn1600 右45度入口\n\n'
    '現行形状から開始。半径−55 mm、Lstart5.80 mm、Lend39.86 mm、論理角−45度、補正−0.90度。'
    'BのPID/FF、吸引650、回数1。実機inR45 preset1600応答と照合するまで準備値。\n\n'
    '前緑/後赤・高さ1500 mm・露出−1.5 EV・1.8倍・ライトOFF・AF固定。盤面左上を原点、45 mm格子。'
    '使用済み配置225/45 mm・前下90度。前走90 mm、後斜め127.279 mm・加減速度20 m/s²。'
    'Lend目標180/225 mm、全動作停止目標90/315 mm。穴・全接地点・車幅・旋回・停止余裕は今回の操作者確認が必要。\n\n'
    'baseline01/run_recorded_once.pyの通常起動はオフライン検証のみ。今回の設置確認後だけ--operator-readyを付ける。'
    '静止区間品質と既存の固定姿勢条件を照合し、各評価フレームでArUco2個以上を実復号する。'
    '実行要求は再送せず、取得後は停止・両方OFFを確認してから詳細解析。BのEKF係数は未同定。'
    'ファーム/パラメータソース変更・書込み・今回の走行は0件。\n', encoding='utf-8')
print(STAGE)
