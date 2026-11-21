"""機体B1600 outR45を既存GUI/BLE経路で単発撮影する準備。実機接続なし。"""
from pathlib import Path
from datetime import datetime,timezone
import copy
import hashlib
import json
import math
import re
import shutil
ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_outr45_20261121_v1';BASE.mkdir(exist_ok=False)
STAGE=BASE/'baseline01';STAGE.mkdir()
IN=ROOT/'tools/logs/pixel_camera/turn1600_B_inr45_20261120_v1'
TEMPLATE=IN/'geometry05_repeat02_correction_m1p2'
identifier='B_OUTR45_1600_baseline_green_20261121_001'
read=lambda p:json.loads(Path(p).read_text(encoding='utf-8-sig'))
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(p,v):
 with Path(p).open('x',encoding='utf-8') as f:json.dump(v,f,ensure_ascii=False,indent=2,allow_nan=False)
code_path=ROOT/'Params/Param_B/turn_1600/turn_1600.cpp';code=code_path.read_text(encoding='utf-8')
def fields(name,typ):return [v.strip() for v in re.search(r'^const '+typ+' '+name+r' = \{([^}]+)\}',code,re.M)[1].split(',')]
number=lambda v:float(v.rstrip('f'))
shape=fields('slalom_outR45_1600_table','t_turn_param_table');assert shape[5]=='Turn_R'
parameters=dict(zip(('velo','r_min','lstart','lend','degree'),map(number,shape[:5])),correction=number(shape[6]),pre_accel=6.5)
assert parameters==dict(velo=1.6,r_min=-60.,lstart=21.8,lend=16.5,degree=-45.,correction=.3,pre_accel=6.5)
sp=list(map(number,fields('sp_gain_turnOut45_1600','t_pid_gain')));om=list(map(number,fields('om_gain_turnOut45_1600','t_pid_gain')))
ffkeys=('ff_sp_velo','ff_sp_accel','ff_sp_bias','ff_om_velo','ff_om_accel','ff_om_decel','ff_om_bias','ff_om_jerk','ff_sp_turn_accel_mag','ff_sp_turn_velo_sq')
ff=dict(zip(ffkeys,map(number,fields('ff_gain_turnIn45_1600','t_ff_gain'))))
assert ff['ff_om_accel']==ff['ff_om_decel']==.00075 and om==[.2,.01,0.]
candidate=dict(schema_version=1,mouse='B',mouse_identity_source='user_supplied',motion_name='out_r45',preset_speed_mm_s=1600,
 parameters=parameters,pid=dict(zip(('sp_kp','sp_ki','sp_kd','om_kp','om_ki','om_kd'),sp+om)),feedforward=ff,
 suction=dict(enabled=True,duty=650),turn_count=1,
 source='現Bソースの左右out45共有FF反映後の初回基準。実機書き込み未実施のため走行時にGUI経路でRAMへ復元し読み返す。未実行。')
save(STAGE/'candidate.settings.json',candidate)
for src,name in ((IN/'marker_reference_B_current.json','marker_reference_B_current.json'),
 (IN/'body_geometry_B_green_red_h160_s30_validated_v5.json','body_geometry_B_green_red_s30_v5.json')):
 shutil.copyfile(src,BASE/name)
config=read(TEMPLATE/'config.json');config.update(mouse='B',address='E1:7F:86:21:84:73',ble_port=8767,
 reference=str(BASE/'marker_reference_B_current.json'),geometry=str(BASE/'body_geometry_B_green_red_s30_v5.json'))
assert config['height_mm']==1500 and config['camera_settings']==dict(ev=-1.5,torch=False,focus_lock=True,zoom_ratio=1.8)
save(STAGE/'config.json',config)
shutil.copyfile(TEMPLATE/'coarse_placement_check.py',STAGE/'coarse_placement_check.py')
source=(TEMPLATE/'run_recorded_once.py').read_text(encoding='utf-8')
for old,new in {'B_INR45_1600_m1p2_repeat02_green_20261121_013':identifier,'in_r45':'out_r45','B45度入口':'B45度出口',
 'placement = dict(x_mm=225., y_mm=45., heading_deg=90.,':'placement = dict(x_mm=90., y_mm=45., heading_deg=45.,',
 'placement, 20., None)':'placement, None, 20.)'}.items():
 assert old in source,old;source=source.replace(old,new)
compile(source,str(STAGE/'run_recorded_once.py'),'exec');(STAGE/'run_recorded_once.py').write_text(source,encoding='utf-8')
now=datetime.now(timezone.utc).isoformat()
save(STAGE/'charging_completion_confirmed.json',dict(confirmed_at_utc=now,request_id=identifier,charging_completed=True,
 explicit_user_confirmation=True,user_reply='充電完了・両方OFF',physical_power_state='BOTH_OFF_USER_CONFIRMED',
 source='Current answer to B out45 preparation charging question; this is not run readiness'))
placement=dict(x_mm=90.,y_mm=45.,heading_deg=45.,position_tolerance_mm=5.,heading_tolerance_deg=2.)
save(STAGE/'placement_plan.json',dict(mouse='B',motion='out_r45',preset_mm_s=1600,count=1,placement=placement,
 front_colour='green',rear_colour='red',pre_run_diagonal_mm=90*math.sqrt(2),post_run_straight_mm=90.,
 requested_pre_accel_m_s2=6.5,effective_pre_accel_m_s2=1.6**2/(2*.09*math.sqrt(2)),post_straight_accel_m_s2=20.,stopping_distance_at20_mm=64.,
 target_Lend_board_mm=[225.,225.],target_stop_board_mm=[225.,315.],target_Lend_right_forward_mm=[45/math.sqrt(2),315/math.sqrt(2)],
 target_stop_right_forward_mm=[135/math.sqrt(2),405/math.sqrt(2)],exit_heading_board_deg=90.,
 placement_basis='B右out45・500時の実使用90/45mm・前右下45度を再利用する案。1600の全接地点/幅/停止余裕は今回の操作者確認が必要。',
 historical_placement_source=str(ROOT/'tools/turn_analysis/20261114_B_turn500_seed_from_A/B_outR45_Lstart32p8_Lend18_plan.json'),
 historical_speed_and_yellow_front_not_used_as_current_validation=True,full_footprint_and_hole_clearance_verified=False,
 coarse_position_tolerance_mm=10.,coarse_heading_tolerance_deg=5.,precise_static_pose_gate_used=False,
 evaluation='実測入口相対の高さ補正中点。各評価コマ実ArUco2個以上。盤面固定/車軸/body yawは別評価。',
 request_id=identifier,request_sent=False,operator_ready=False,charging_completion_confirmed=True,
 physical_power_state='BOTH_OFF_USER_CONFIRMED',hardware_commands_sent=0))
policy=dict(method='incoming_raw_wheel_clock_and_real_registered_marker_midpoint',minimum_observation_rate=.95,
 per_frame_real_aruco_minimum=2,maximum_point_interpolation_gap_ms=20,minimum_line_points=15,minimum_line_span_mm=30,
 maximum_line_rms_mm=.75,exit_window_start_ms_after_Lend=10,exit_window_end_ms_before_motion_end=10,
 exit_window_reason='後直進90mm・1600mm/s・減速20m/s²の名目所要時間96.25msに対し、15/25msを除くと約14フレームとなる。初回撮影前に10/10msの固定窓を定義し、15点/30mm/RMS条件を維持する。減速を含む出口進行方向。',
 target_Lend_right_forward_mm=[45/math.sqrt(2),315/math.sqrt(2)],target_stop_right_forward_mm=[135/math.sqrt(2),405/math.sqrt(2)],
 nominal_post_run_duration_ms=96.25,body_yaw_available=False,EKF_coefficients_identified=False,
 iteration_policy='基準一回→必要な項目だけ補正→確認を基本とし、実走回数と残差を毎回示す。追加の広い条件探索を避ける。')
save(BASE/'analysis_policy.json',policy)
save(BASE/'resume_state.json',dict(state='OUT45_BASELINE_PREPARED_OFF_CONFIRMED',mouse='B',motion='out_r45',preset=1600,
 stage=str(STAGE),request_id=identifier,request_sent=False,execution_commands_sent=0,operator_ready=False,
 charging_completion_confirmed=True,physical_power_state='BOTH_OFF_USER_CONFIRMED',execution_pending=False,
 ble_binding=dict(address='E1:7F:86:21:84:73',ble_port=8767,camera_port=8765),selected_settings=str(STAGE/'candidate.settings.json'),
 current_placement_confirmed=False,source_params_changed=False,firmware_write_performed=False,
 next_action='写真案内と読み取り専用のカメラ/ポート確認を完了後、今回の概略配置・干渉余裕・両方ON・手離し・一回可を確認する。'))
inputs=[code_path,ROOT/'System/Src/myshell.cpp',Path(config['reference']),Path(config['geometry']),STAGE/'candidate.settings.json',STAGE/'run_recorded_once.py',Path(__file__)]
save(BASE/'preparation.json',dict(prepared_at_utc=now,source_parameters=parameters,source_pid=candidate['pid'],feedforward=ff,
 firmware_not_written=True,device_settings_to_be_applied_and_verified_at_run=True,
 original_machine_B_data_only=True,hardware_commands_sent=0,input_sha256={str(p):sha(p) for p in inputs}))
(BASE/'README.md').write_text('# 機体B turn1600 右out45：初回基準の準備\n\n'
 'ユーザー確認済みB・1600のout45。現ソースの半径−60mm・Lstart21.8/Lend16.5mm・補正+0.3度、角速度PID0.20/0.01/0、入口と共通の角加減速FF0.000750、吸引650を基準に一回取得する準備。実機書き込み未実施のため、実行時に既存GUI/BLE経路でRAMへ復元し全設定を読み返す。\n\n'
 'B用HY0020 E1:7F:86:21:84:73・BLE8767、カメラ8765。使用中ポートは操作せず停止。高さ1500mm・EV−1.5/1.8倍・ライトOFF/AF固定、前緑/後赤、実画像で検証したS30プロファイルを使用。今回out45動画の品質は取得後に必ず確認する。\n\n'
 '充電完了・両方OFFの明示回答を保存。Bで使用実績のある中点90/45mm・前右下45度を今回再確認する。穴/全接地点/幅/旋回/停止余裕は今回の操作者確認が必要。位置は概略配置、入口方向を動画で補正する方針を継続。\n\n'
 '前斜め127.279mm・後直進90mm/減速20m/s²。Lend目標は盤面225/225、停止225/315mm。入口相対目標はLend31.820/222.739、停止95.459/286.378mm。初回撮影前にanalysis_policy.jsonの固定窓・品質条件を設定。\n\n'
 '走行0回・設定送信0回・ファーム書込み0回。過去500の形状/ゲイン・別機体Aの資料を今回の採用値へ流用していない。まず右単体を評価し、左/連続/EKFは別確認。\n',encoding='utf-8')
print(json.dumps(dict(stage=str(STAGE),request_sent=False,parameters=parameters,charging_completed=True,hardware_commands_sent=0),ensure_ascii=False))
