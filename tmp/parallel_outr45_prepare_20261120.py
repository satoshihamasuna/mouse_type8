"""A1800右45°出口のソース基準・既使用配置・MCP候補をオフライン準備する。"""
from pathlib import Path
import ast
import hashlib
import json
import re
import shutil
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_outr45_20261120_v1'
IN=ROOT/'tools/logs/pixel_camera/turn1800_A_inr45_20261120_v1'
R180=ROOT/'tools/logs/pixel_camera/turn1800_A_r180_20261120_v1/baseline01'
OLD=ROOT/'tools/logs/pixel_camera/turn1600_A_outr45_20261119_v1'
PROFILE=ROOT/'Params/Param_A/turn_1800/turn_1800.cpp'
FLAG=ROOT/'Params/Param_A/run_param_A.h'
TYPEDEF=ROOT/'Params/typedef_run_param.h'
REQUEST='A_OUTR45_1800_baseline_20261120_001'
if BASE.exists(): raise FileExistsError('保存済み成果物を上書きしません')


def read(path): return json.loads(path.read_text(encoding='utf-8-sig'))
def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest()
def save(path,value):
    with path.open('x',encoding='utf-8') as f:
        json.dump(value,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')


source=PROFILE.read_text(encoding='utf-8')
assert re.search(r'^#define\s+PARAM_A_USE_PINK_LED_TURN_LENGTHS\s+0\s*$',FLAG.read_text(encoding='utf-8'),re.M)
branch=source.split('//k = 250\nconst t_pid_gain sp_gain_turnOut45_1800',1)[1].split('//k = 250\nconst t_pid_gain sp_gain_turnIn135_1800',1)[0]
active=branch.split('#else',1)[1].split('#endif',1)[0]
row=re.search(r'^const t_turn_param_table slalom_outR45_1800_table\s*=\s*\{([^}]+)\}',active,re.M).group(1)
numeric=lambda text:[float(x.strip().rstrip('f')) for x in text.split(',')]
table=numeric(','.join(row.split(',')[:5]))
def values(symbol,typename):
    return numeric(re.search(r'^const '+typename+r' '+symbol+r'\s*=\s*\{([^}]+)\}',source,re.M).group(1))
sp=values('sp_gain_turnOut45_1800','t_pid_gain')
om=values('om_gain_turnOut45_1800','t_pid_gain')
ff=values('ff_gain_out45_R_1800','t_ff_gain')
assert len(ff)==8 and 'float sp_turn_accel_mag = 0.0f;' in TYPEDEF.read_text(encoding='utf-8')
ff+= [0.,0.]
assert table==[1.8,-55.,22.66,26.,-45.] and sp==[2.,.05,0.] and om==[.12,.004,0.]
candidate=read(IN/'baseline01/candidate.settings.json')
candidate.update(source='Unexecuted A1800 out_r45 source baseline, PARAM_A_USE_PINK_LED_TURN_LENGTHS=0; dedicated out45_R feedforward and zero default SP-turn terms. Fresh device show required.',motion_name='out_r45')
candidate['parameters']=dict(zip(('velo','r_min','lstart','lend','degree'),table),correction=0.,pre_accel=6.5)
candidate['pid']=dict(zip(('sp_kp','sp_ki','sp_kd','om_kp','om_ki','om_kd'),sp+om))
ffkeys=['ff_sp_velo','ff_sp_accel','ff_sp_bias','ff_om_velo','ff_om_accel','ff_om_decel','ff_om_bias','ff_om_jerk','ff_sp_turn_accel_mag','ff_sp_turn_velo_sq']
candidate['feedforward']=dict(zip(ffkeys,ff))
candidate.update(turn_count=1,preset_speed_mm_s=1800,mouse='A',mouse_identity_source='user_supplied')
candidate['suction']=dict(enabled=True,duty=650)

BASE.mkdir();stage=BASE/'baseline01';stage.mkdir()
for name in ('marker_reference_A_current.json','body_geometry_A_yellow_v2.json'):
    shutil.copy2(IN/name,BASE/name)
config=read(IN/'baseline01/config.json')
config.update(reference=str(BASE/'marker_reference_A_current.json'),geometry=str(BASE/'body_geometry_A_yellow_v2.json'))
save(stage/'config.json',config);save(stage/'candidate.settings.json',candidate)
for name in ('static_gate_v2_source.py','static_gate_v2_validation.json','static_pose_after_settle.py'):
    shutil.copy2(IN/'baseline01'/name,stage/name)
policy=read(IN/'baseline01/static_settle_policy.json')
policy.update(position_tolerance_mm_unchanged=3.5,
    placement_policy_source='A1600 out_r45 operational placement315/90mm, static heading133.5deg, position3.5mm, heading2deg; tighter than inherited5mm maximum.')
save(stage/'static_settle_policy.json',policy)
wrapper=(R180/'run_recorded_once.py').read_text(encoding='utf-8')
assert wrapper.count('x_mm=315.')==1 and wrapper.count('y_mm=135.')==1
wrapper=wrapper.replace('y_mm=135.','y_mm=90.').replace('heading_deg=89.26036071777344','heading_deg=133.5')
wrapper=wrapper.replace('position_tolerance_mm=5.','position_tolerance_mm=3.5').replace('stationary_frame_stride=2','stationary_frame_stride=4')
wrapper=wrapper.replace('A_R180_1800_baseline_20261120_001',REQUEST)
assert 'placement, None, 20.)' in wrapper
ast.parse(wrapper)
(stage/'run_recorded_once.py').write_text(wrapper,encoding='utf-8')

old_static=ROOT/'tools/logs/pixel_camera/mcp_tuning/prepare_A_OUTR45_1600_target135_225_Ls21p8_Le16p5_c0p3_20261119_MCP027/static_check.json'
old_contact=OLD/'target135_225_candidate_v1/entry135_recheck_v1/recheck_summary.json'
old_500_contact=ROOT/'tools/logs/pixel_camera/cli_out_r45_c05_repeat03_ready_20261111_153140/operator_stop_out_r45_c05_cli_003.json'
static=read(old_static);contact=read(old_contact)
assert static['passed'] and contact['trials'][0]['contact_detected'] is False
placement=dict(x_mm=315.,y_mm=90.,heading_deg=133.5,position_tolerance_mm=3.5,heading_tolerance_deg=2.)
plan=dict(mouse='A',motion='out_r45',preset_mm_s=1800,count=1,
    planned_start_board_mm=[315.,90.],planned_heading_image_deg=135.,
    static_apparent_reference_heading_deg=133.5,placement=placement,
    static_reference_basis='A1600 out_r45 successful operational apparent-marker envelope, not calibrated body yaw. Latest successful static133.776deg; static-to-dynamic offset varies and is not folded into turn correction.',
    front_colour='red',rear_colour='yellow',pre_run_diagonal_mm=float(90*np.sqrt(2)),post_run_straight_mm=90.,
    requested_pre_accel_m_s2=6.5,effective_pre_accel_m_s2=float(1.8**2/(2*.09*np.sqrt(2))),
    requested_post_straight_accel_m_s2=20.,stopping_distance_at20_mm=81.,
    nominal_Lend_board_mm=[135.,225.],nominal_stop_board_mm=[45.,225.],
    nominal_Lend_right_forward_mm=[float(45/np.sqrt(2)),float(315/np.sqrt(2))],
    nominal_stop_right_forward_mm=[float(135/np.sqrt(2)),float(405/np.sqrt(2))],
    placement_basis='A1600 same motion repeatedly used315/90mm, red front toward lower-left. No B placement copied. New1800 source profile and stopping behavior require current operator clearance confirmation.',
    successful_placement_evidence=dict(static_json=str(old_static),static_center_mm=static['center_board_mm'],
        static_heading_deg=static['heading_deg'],contact_summary=str(old_contact),
        contact_detected=False,previous_A500_operator_record=str(old_500_contact)),
    rejected_or_not_adopted=dict(old_135_45_lower_right='Prior requested135/45mm lower-right diagonal placement was superseded after hole-contact concern; do not reuse.',
        source=str(ROOT/'tools/logs/pixel_camera/cli_out_r45_20261111_132440/static_probe_rejection_and_setup.json'),
        B90_45='B out_r45 placement only; not used for A.'),
    camera_height_mm=1500.,camera_settings=config['camera_settings'],
    reference_policy='Each evaluated frame requires actually decoded registered ArUcos2and3; exclude missing registration; observation95% and static v2 stability unchanged.',
    current_placement_confirmed=False,current_clearance_confirmed=False,current_hands_clear_confirmed=False,
    operator_ready=False,power_state='Not checked by this preparation; no hardware connection',
    source_params_changed=False,next_candidate_executed=False,execution_pending=False,parameter_adoption_allowed=False,
    next_request_id=REQUEST,candidate=str(stage/'candidate.settings.json'),config=str(stage/'config.json'),
    guide=str(BASE/'start_placement_guide.png'),time_target_minutes=20,
    limits=['A1600 values are placement evidence only; no tuning value copied.',
        'Hole/full contact geometry and body width not mechanically verified by path sketch.',
        'Camera foot/marker height/body yaw calibration remain unverified.',
        'Yellow midpoint dropouts observed in in_r45; front-only calibration is separate work and not silently substituted.'])
save(BASE/'preparation.json',plan);save(stage/'placement_plan.json',plan)
save(stage/'phase.json',dict(state='PREPARED_NOT_EXECUTED',request_id=REQUEST,operator_ready=False,
    source_profile=str(PROFILE),source_flag=0,current_placement_confirmed=False,current_clearance_confirmed=False,
    next_candidate_executed=False,execution_pending=False))
save(BASE/'source_extraction.json',dict(profile=str(PROFILE),flag=str(FLAG),active_flag_value=0,
    table_symbol='slalom_outR45_1800_table',table=table,
    sp_symbol='sp_gain_turnOut45_1800',sp=sp,om_symbol='om_gain_turnOut45_1800',om=om,
    ff_symbol='ff_gain_out45_R_1800',feedforward=dict(zip(ffkeys,ff)),
    omitted_aggregate_defaults={'degree_correction':0.,'sp_turn_accel_mag':0.,'sp_turn_velo_sq':0.},
    fresh_device_show_verified=False,
    sources_sha256={str(p):sha(p) for p in [PROFILE,FLAG,TYPEDEF,ROOT/'System/Src/myshell.cpp',old_static,old_contact,old_500_contact]}))

plt.rcParams['font.family']='Yu Gothic'
fig,ax=plt.subplots(figsize=(7,7),layout='constrained')
for value in range(0,361,45):
    ax.axvline(value,color='.87',lw=.7);ax.axhline(value,color='.87',lw=.7)
holes=np.array([(x,y) for x in range(0,361,90) for y in range(0,361,90)])
ax.scatter(*holes.T,color='.25',s=12,label='穴位置の参考格子（90 mm周期）')
ax.plot([315,225],[90,180],lw=2,label='斜め前走127.3 mm')
u=np.linspace(0,1,100)[:,None]
curve=(1-u)**3*np.array([225,180])+3*(1-u)**2*u*np.array([200,205])+3*(1-u)*u**2*np.array([165,225])+u**3*np.array([135,225])
ax.plot(*curve.T,'--',lw=2,label='右45°出口・設計接続の模式線')
ax.plot([135,45],[225,225],lw=2,label='直線後走90 mm（acc 20）')
front=np.array([315.,90.])+25/np.sqrt(2)*np.array([-1,1])
rear=np.array([315.,90.])-25/np.sqrt(2)*np.array([-1,1])
ax.plot([front[0],rear[0]],[front[1],rear[1]],c='.3',lw=4)
ax.scatter(*front,c='red',s=80,label='前：赤、左下135°');ax.scatter(*rear,c='gold',edgecolors='black',s=70,label='後：黄')
ax.scatter(315,90,c='blue',s=30);ax.annotate('開始中点 (315,90) mm',xy=(315,90),xytext=(90,45),arrowprops=dict(arrowstyle='->'))
ax.scatter(135,225,marker='x',s=60,c='purple',label='設計Lend点 (135,225)')
ax.scatter(45,225,marker='x',s=60,c='black',label='参考停止点 (45,225)')
ax.set(xlim=(-12,372),ylim=(372,-12),aspect='equal',xlabel='盤面右方向 [mm]',ylabel='盤面下方向 [mm]',
    title='機体A / 1800 mm/s / 右45°出口\nA1600の既使用配置。穴・接地点・全幅・停止余裕を現在の実物で確認')
ax.legend(loc='lower right',fontsize=7.5)
fig.savefig(BASE/'start_placement_guide.png',dpi=150);plt.close(fig)
print('A1800 out_r45 source/placement prepared; hardware operations:0',flush=True)
