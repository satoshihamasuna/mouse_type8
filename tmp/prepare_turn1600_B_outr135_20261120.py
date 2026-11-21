"""B1600右135度出口の初回と配置図を保存資料だけで準備する。機器操作なし。"""
from pathlib import Path
from datetime import datetime, timezone
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

ROOT=Path(__file__).resolve().parents[1]
IN=ROOT/'tools/logs/pixel_camera/turn1600_B_inr135_20261120_v1'
TEMPLATE=IN/'angle_cm0p4_01'
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_outr135_20261120_v1'
STAGE=BASE/'baseline01'
RID='B_OUTR135_1600_baseline_green_20261120_001'
SOURCE=ROOT/'Params/Param_B/turn_1600/turn_1600.cpp'
HISTORY=ROOT/'tools/turn_analysis/20261114_B_turn500_seed_from_A/B_outR135_start180_45_plan.json'
read=lambda path:json.loads(path.read_text(encoding='utf-8-sig'))


def save(path,value):
    with path.open('x',encoding='utf-8') as file:json.dump(value,file,ensure_ascii=False,indent=2,allow_nan=False)


assert not BASE.exists()
text=re.sub(r'/\*.*?\*/|//[^\n]*','',SOURCE.read_text(encoding='utf-8'),flags=re.S)
raw=[p.strip() for p in re.search(r'const t_turn_param_table slalom_outR135_1600_table\s*=\s*\{([^}]+)\}',text)[1].split(',')]
assert len(raw)==7 and raw[5]=='Turn_R'
numbers=[float(p.removesuffix('f')) for p in raw[:5]+raw[6:]]
assert numbers==[1.6,-41.,1.,31.5,-135.,1.9]
assert re.search(r'param_outR135_1600\s*=\s*\{&slalom_outR135_1600_table,&sp_gain_turnOut135_1600,&om_gain_turnOut135_1600\}',text)
actual_path=IN/'baseline01/local_execution/run_B_INR135_1600_baseline_green_20261120_002/saved_1ms.settings.json'
actual=read(actual_path)
assert actual['mouse']=='B' and actual['preset_speed_mm_s']==1600
candidate={key:copy.deepcopy(actual[key]) for key in ('schema_version','mouse','mouse_identity_source','motion_name',
    'preset_speed_mm_s','turn_count','parameters','pid','feedforward','suction')}
candidate.update(motion_name='out_r135',turn_count=1)
candidate['parameters']=dict(zip(('velo','r_min','lstart','lend','degree','correction'),numbers),pre_accel=6.5)
for prefix in ('sp','om'):
    gains=[float(p.strip().removesuffix('f')) for p in re.search(r'const t_pid_gain '+prefix+r'_gain_turnOut135_1600\s*=\s*\{([^}]+)\}',text)[1].split(',')]
    candidate['pid'].update(dict(zip([prefix+'_'+p for p in ('kp','ki','kd')],gains)))
candidate['source']='未実行のB1600右135度出口初回。形状/PIDはB現行ソース、FFは同日のB1600実測既定値。対象プリセットのFFを走行前に読み戻す。'
STAGE.mkdir(parents=True)
config=read(TEMPLATE/'config.json')
for key,name in [('reference','marker_reference_B_current.json'),('geometry','body_geometry_B_green_front_s35_v3.json')]:
    shutil.copyfile(Path(config[key]),BASE/name);config[key]=str(BASE/name)
save(STAGE/'config.json',config);save(STAGE/'candidate.settings.json',candidate)
for name in ('coarse_placement_check.py','static_pose_after_settle.py'):
    shutil.copyfile(TEMPLATE/name,STAGE/name)
wrapper=(TEMPLATE/'run_recorded_once.py').read_text(encoding='utf-8-sig')
for before,after in [('B_INR135_1600_cm0p4_green_20261120_001',RID),
    ('x_mm=315., y_mm=135., heading_deg=90.','x_mm=180., y_mm=45., heading_deg=45.'),
    ('placement, 20., None','placement, None, 20.'),('B135度入口の実機FF','B135度出口の実機FF'),
    ('Current operator readiness for B1600 in_r135','Current operator readiness for B1600 out_r135')]:
    assert wrapper.count(before)==1,before
    wrapper=wrapper.replace(before,after)
compile(wrapper,'<B1600 out_r135>','exec')
(STAGE/'run_recorded_once.py').write_text(wrapper,encoding='utf-8',newline='\n')
placement=dict(x_mm=180.,y_mm=45.,heading_deg=45.,position_tolerance_mm=5.,heading_tolerance_deg=2.)
plan=dict(prepared_at=datetime.now(timezone.utc).isoformat(),mouse='B',motion='out_r135',preset_mm_s=1600,count=1,
    placement=placement,planned_start_board_mm=[180.,45.],planned_heading_image_deg=45.,front_colour='green',rear_colour='red',
    pre_run_type='diagonal',pre_run_mm=float(90*np.sqrt(2)),post_run_type='straight',post_run_mm=90.,
    requested_turn_pre_accel_m_s2=6.5,effective_pre_accel_m_s2=float(1.6**2/(2*.09*np.sqrt(2))),
    requested_post_straight_accel_m_s2=20.,stop_distance_at20_mm=64.,
    target_Lend_board_mm=[225.,225.],target_stop_board_mm=[135.,225.],target_exit_heading_board_deg=180.,
    relative_Lend_target_right_forward_mm=[float(67.5*np.sqrt(2)),float(112.5*np.sqrt(2))],
    relative_stop_target_right_forward_mm=[float(112.5*np.sqrt(2)),float(67.5*np.sqrt(2))],
    historical_placement_source=str(HISTORY),
    placement_basis='B500右135度出口で実使用し接触なしを確認した180/45mm近傍を今回の候補とする。速度・半径・撮影/マーカー変更後の全接地点・全幅・旋回・停止余裕を保証しない。135/45mmの撤回案は使用しない。',
    camera_height_mm=config['height_mm'],camera_settings=config['camera_settings'],
    placement_policy='今回の概略開始位置を新規撮影で確認後、走行動画の実測入口基準で相対評価。盤面固定差も別に記録。',
    precise_static_pose_gate_used=False,coarse_position_tolerance_mm=10.,coarse_heading_tolerance_deg=5.,
    hole_clearance_verified=False,current_placement_confirmed=False,current_clearance_confirmed=False,
    current_hands_clear_confirmed=False,operator_ready=False,power_state='BOTH_OFF_USER_CONFIRMED_AFTER_INR135',
    next_request_id=RID,next_request_sent=False,execution_pending=False,execution_commands_sent=0,settings_write_commands_sent=0,
    source_params_changed=False,firmware_write_performed=False,candidate=str(STAGE/'candidate.settings.json'),
    config=str(STAGE/'config.json'),guide=str(BASE/'start_placement_guide.jpg'),
    evaluation_scope='前後マーカー中点の入口相対形状。固定盤面の設置・出口・停止は別評価。Bの車軸/機軸絶対幾何・EKF係数は未校正。',
    source_hashes={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in (SOURCE,actual_path,HISTORY)},
    next_action='右入口の調整後、両方OFFで180/45mm・緑前右下45度へ配置。今回の全接地点/車幅/穴/旋回/停止余裕・両方ON・手離し確認後に1回のみ。')
save(BASE/'preparation.json',plan);save(STAGE/'placement_plan.json',plan)
save(BASE/'resume_state.json',dict(plan,state='PREPARED_NOT_EXECUTED',ble_connected=False))
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'tools/analysis')]
from pixel_camera_tuning_mcp import McpTuningService
payload=McpTuningService(BASE/'offline_validation',{}).prepare(str(STAGE/'config.json'),str(STAGE/'candidate.settings.json'),placement,None,20.)
save(BASE/'offline_validation.json',dict(state='VALIDATED_NOT_EXECUTED',payload=payload,hardware_operations=0))
from video_marker_pose import marker_registration,registration_quad,pose_world_corners,board_to_pose_transform,transform_points
image=IN/'angle_cm0p4_01/local_execution/prepare_B_INR135_1600_cm0p4_green_20261120_001/static_check.jpg'
frame=cv2.imread(str(image));reference=read(Path(config['reference']))
registration=marker_registration(frame,reference);quad=registration_quad(registration)
assert quad is not None and len(set(registration['detected_marker_ids']))>=2
matrix=cv2.getPerspectiveTransform(pose_world_corners(reference['board_side_mm']),quad)@board_to_pose_transform(reference)
v=np.array([1.,1.])/np.sqrt(2);p=np.array([180.,45.])
points=transform_points(np.array([p,p-v*25,p+v*25]),matrix)
plt.rcParams['font.family']='Yu Gothic'
fig,ax=plt.subplots(figsize=(12.8,7.2),layout='constrained');ax.imshow(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB))
ax.plot(points[[1,2],0],points[[1,2],1],c='cyan',lw=2)
ax.scatter(*points[0],s=220,marker='+',c='cyan',linewidths=2)
ax.scatter(*points[2],s=110,marker='v',c='lime',edgecolors='black',label='前：緑（右下45度）')
ax.scatter(*points[1],s=90,c='red',edgecolors='white',label='後：赤')
ax.annotate('開始中点 (180,45) mm\n左から4マス・上から1マス',xy=points[0],xytext=(330,260),color='white',fontsize=12,
    arrowprops=dict(arrowstyle='->',color='cyan'),bbox=dict(facecolor='black',alpha=.8,edgecolor='none'))
ax.set_title('機体B・1600 右135度出口：両方OFFで配置\n保存写真への案内（現在位置の測定ではありません）')
ax.legend(loc='lower left');ax.axis('off');fig.savefig(BASE/'start_placement_guide.jpg',dpi=130);plt.close(fig)
(BASE/'README.md').write_text('機体B1600右135度出口の初回を現行値で準備。半径−41mm・Lstart1mm・Lend31.5mm・補正+1.9度。'
    'BのPID/FF、吸引650、斜め前走127.28mm・直進後走90mm・後走20m/s²。\n\n'
    '開始候補は中点180/45mm・緑前右下45度。B500の実使用配置を起点にするが、今回の1600での穴・全接地点・全幅・旋回/停止余裕を確認する。'
    '撤回済みの135/45mm案は使用しない。右入口の後に実施し、現在の操作者確認前に送信しない。\n',encoding='utf-8')
print(json.dumps(dict(state='VALIDATED_NOT_EXECUTED',folder=str(BASE),parameters=candidate['parameters'],placement=placement,hardware_operations=0),ensure_ascii=False))
