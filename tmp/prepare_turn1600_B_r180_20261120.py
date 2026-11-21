"""B1600右180度の未実行候補と配置図を準備する。実機へ接続しない。"""
from pathlib import Path
from datetime import datetime, timezone
import copy
import hashlib
import json
import shutil
import sys

import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
OLD = ROOT / 'tools/logs/pixel_camera/turn1600_B_r90_20261120_v1'
BASE = ROOT / 'tools/logs/pixel_camera/turn1600_B_r180_20261120_v1'
STAGE = BASE / 'baseline01'
STAGE.mkdir(exist_ok=False)
sys.path[:0] = [str(ROOT / 'tools'), str(ROOT / 'tools/analysis')]
from pixel_camera_tuning_mcp import McpTuningService
from video_marker_pose import (marker_registration, registration_quad,
    pose_world_corners, board_to_pose_transform, transform_points)


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def save(path, value):
    with path.open('x', encoding='utf-8') as output:
        json.dump(value, output, ensure_ascii=False, indent=2, allow_nan=False)
        output.write('\n')


config = read(OLD / 'config.green_front.v1.json')
for key, name in [('reference', 'marker_reference_B_current.json'),
                  ('geometry', 'body_geometry_B_green_front_v1.json')]:
    target = BASE / name
    shutil.copyfile(Path(config[key]), target)
    config[key] = str(target)
save(STAGE / 'config.json', config)
settings_source = OLD / 'device_prepare_green_Lstart13p6_04/actual.settings.json'
observed = read(settings_source)
candidate = {key: copy.deepcopy(observed[key]) for key in (
    'schema_version', 'mouse', 'mouse_identity_source', 'motion_name',
    'preset_speed_mm_s', 'turn_count', 'parameters', 'pid', 'feedforward', 'suction')}
candidate['motion_name'] = 'long_r180'
candidate['parameters'].update(r_min=-45., lstart=13.12, lend=27.38,
    degree=-180., correction=2., pre_accel=6.5)
candidate['source'] = ('機体B現行turn1600右180度の形状とPID。FFは同日のB1600実機既定値を引用。'
    '未接続・未送信・未実走。走行前に右180度の実機プリセットFFと照合する。')
candidate['feedforward_source'] = str(settings_source)
save(STAGE / 'candidate.settings.json', candidate)
placement = dict(x_mm=315., y_mm=135., heading_deg=90.,
    position_tolerance_mm=5., heading_tolerance_deg=2.)
plan = dict(state='PREPARED_OFF_PLACEMENT_REQUIRED', prepared_at=datetime.now(timezone.utc).isoformat(),
    mouse='B', motion='long_r180', preset_mm_s=1600, count=1,
    planned_start_board_mm=[315.,135.], planned_heading_image_deg=90.,
    placement=placement, front_colour='green', rear_colour='red',
    pre_run_mm=90., post_run_mm=90., requested_turn_pre_accel_m_s2=6.5,
    effective_pre_accel_from_source_m_s2=1.6**2/(2*.09),
    requested_post_straight_accel_m_s2=20., stop_distance_at20_mm=64.,
    stop_distance_at6p5_mm=1000*1.6**2/(2*6.5),
    suction=candidate['suction'], reference_nominal_stop_board_mm=[225.,135.],
    placement_basis='同じ盤で実使用したA1600/1800右180度の315/135mm配置をBの候補とする。B1600右90度の315/45mmから下へ90mmで穴格子位相を維持。Bの全接地点・全幅・旋回・停止余裕は今回確認する。',
    hole_clearance_verified=False, current_placement_confirmed=False,
    current_hands_clear_confirmed=False, current_clearance_confirmed=False,
    power_state='BOTH_OFF_USER_CONFIRMED', same_camera_board_user_confirmed=True,
    current_marker_colours_user_confirmed=True, camera_height_mm=1500.,
    camera_settings=config['camera_settings'], camera_health=str(BASE/'camera_health02.json'),
    candidate=str(STAGE/'candidate.settings.json'), config=str(STAGE/'config.json'),
    next_unsent_request_id='B_R180_1600_baseline_green_20261120_001',
    execution_commands_sent=0, settings_write_commands_sent=0, execution_pending=False,
    source_params_changed=False, firmware_write_performed=False,
    runtime_modes='未読。走行前にwall_ekf/wall_edge/turn_ekfを記録。BのEKF未校正。',
    evaluation_scope='前後マーカー中点と実測入口を用いた相対形状。盤面固定の開始・出口・停止点は別に記録。絶対車軸位置と機首方向は未校正。',
    long180_identifiability='平行な入口と出口の終点だけではLstart/Lendを分離できない。旋回開始・頂点と時刻対応した軌跡も評価する。',
    ekf_identification='B条件の有効動画・IMUと静止データがそろってから識別可否を評価。AのKや雑音をBへ流用しない。',
    next_action='両方OFFで315/135mm・緑前下向きへ配置。新しい静止撮影でArUcoとマーカーを照合し、現在の全接地点・車幅・穴・停止余裕を確認してからONを依頼。',
    source_hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
        for p in [ROOT/'Params/Param_B/turn_1600/turn_1600.cpp',
                  ROOT/'Params/Param_B/run_param_B.cpp', ROOT/'System/Src/myshell.cpp',
                  ROOT/'Task/Src/motion.cpp', settings_source]})
save(BASE/'preparation.json',plan)
save(STAGE/'placement_plan.json',plan)
save(BASE/'operator_conditions01.json',dict(source='current_user_reply',
    quote='同じ条件・両方OFF', same_board_and_camera_height=True,
    front_green_rear_red=True, both_powered_off=True,
    placement_confirmed=False, hands_clear_confirmed=False))
service = McpTuningService(BASE/'offline_validation',{})
validated = service.prepare(str(STAGE/'config.json'),str(STAGE/'candidate.settings.json'),
    placement,None,20.)
save(BASE/'offline_validation.json',dict(state='VALIDATED_NO_HARDWARE_CONNECTION',
    settings=validated['settings'],placement=validated['placement'],
    post_straight_accel_m_s2=validated['post_straight_accel_m_s2']))

# 同じ撮影条件の保存像を背景として使う。今回の実配置を測った画像ではない。
frame_path = OLD/'green_Lstart13p6_after_ble_diagnosis_placed01_static_frame.jpg'
frame = cv2.imread(str(frame_path))
reference = read(config['reference'])
registration = marker_registration(frame,reference)
quad = registration_quad(registration)
if quad is None or len(registration['detected_marker_ids']) < 2:
    raise RuntimeError('保存画像のArUco登録を確認できない')
transform = cv2.getPerspectiveTransform(pose_world_corners(reference['board_side_mm']),quad) @ board_to_pose_transform(reference)
points = transform_points(np.array([[315.,135.],[315.,110.],[315.,160.],[315.,45.]]),transform)
plt.rcParams['font.family'] = 'Yu Gothic'
fig, ax = plt.subplots(figsize=(12.8,7.2),layout='constrained')
ax.imshow(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB))
ax.plot(points[[1,2],0],points[[1,2],1],color='cyan',lw=2)
ax.scatter(*points[0],s=210,marker='+',c='cyan',lw=2)
ax.scatter(*points[2],s=105,marker='v',c='lime',edgecolor='black',label='前：緑（下向き）')
ax.scatter(*points[1],s=90,c='red',edgecolor='white',label='後：赤')
ax.annotate('開始中点 (315,135) mm\n右端から1マス・上端から3マス',
    xy=points[0],xytext=(390,260),color='white',fontsize=12,
    arrowprops=dict(arrowstyle='->',color='cyan'),
    bbox=dict(facecolor='black',alpha=.75,edgecolor='none'))
ax.annotate('右90度の開始点から下へ2マス（90 mm）',xy=points[0],xytext=(430,190),
    color='white',fontsize=10,bbox=dict(facecolor='black',alpha=.65,edgecolor='none'))
ax.set_title('機体B・1600 右180度：両方OFFで配置\n保存済み写真への配置案（現在位置の測定ではありません）')
ax.legend(loc='lower left');ax.axis('off')
fig.savefig(BASE/'start_placement_guide.jpg',dpi=130);plt.close(fig)
print(json.dumps(dict(state='VALIDATED_NOT_EXECUTED',folder=str(BASE),
    placement=placement,parameters=candidate['parameters'],
    execution_commands_sent=0),ensure_ascii=False),flush=True)
