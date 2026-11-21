"""A1800右45度入口の初回設定を準備する。実機接続なし。"""
from pathlib import Path
import ast
import json
import shutil
import hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
OLD=ROOT/'tools/logs/pixel_camera/turn1800_A_r180_20261120_v1'
SRC=OLD/'baseline01'
BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_inr45_20261120_v1'
BASE.mkdir(exist_ok=False); STAGE=BASE/'baseline01'; STAGE.mkdir()


def save(path,value):
    with path.open('x',encoding='utf-8') as f: json.dump(value,f,ensure_ascii=False,indent=2)


for name in ('marker_reference_A_current.json','body_geometry_A_yellow_v2.json'):
    shutil.copy2(OLD/name,BASE/name)
config=json.loads((SRC/'config.json').read_text(encoding='utf-8'))
config.update(reference=str(BASE/'marker_reference_A_current.json'),geometry=str(BASE/'body_geometry_A_yellow_v2.json'))
save(STAGE/'config.json',config)
c=json.loads((SRC/'candidate.settings.json').read_text(encoding='utf-8'))
c.update(motion_name='in_r45',source='Unexecuted source A1800 in_r45 baseline, PARAM_A_USE_PINK_LED_TURN_LENGTHS=0; A1600 values are not adopted.')
c['parameters'].update(r_min=-53.,lstart=1.33,lend=46.18,degree=-45.,correction=0.)
c['pid'].update(sp_ki=.05,om_kp=.12,om_ki=.004)
c['feedforward'].update(ff_sp_accel=.08297866,ff_sp_bias=.08061591,ff_om_accel=.00094,ff_om_decel=.00082,ff_om_jerk=.0000024)
save(STAGE/'candidate.settings.json',c)
for name in ('static_gate_v2_source.py','static_gate_v2_validation.json'):
    shutil.copy2(SRC/name,STAGE/name)
policy=json.loads((SRC/'static_settle_policy.json').read_text(encoding='utf-8'))
policy.update(stationary_frame_stride=4,sampling_revision='Use original MCP60Hz static sampling over the same settled1.65s window. Recognition95%, minimum12, registration and v2 stability limits unchanged. Motion analysis remains all frames.')
save(STAGE/'static_settle_policy.json',policy)
helper=(SRC/'static_pose_after_settle.py').read_text(encoding='utf-8').replace("plan['stationary_frame_stride'] == 2","plan['stationary_frame_stride'] == 4").replace("kwargs['frame_stride'] = 2","kwargs['frame_stride'] = 4")
with (STAGE/'static_pose_after_settle.py').open('x',encoding='utf-8') as f:f.write(helper)
rid='A_INR45_1800_baseline_20261120_001'
code=(SRC/'run_recorded_once.py').read_text(encoding='utf-8')
assert code.count('x_mm=315.')==1 and code.count('y_mm=135.')==1
code=code.replace('x_mm=315.','x_mm=225.').replace('y_mm=135.','y_mm=45.')
code=code.replace('heading_deg=89.26036071777344','heading_deg=89.50081634521484')
code=code.replace('placement, None, 20.)','placement, 20., None)').replace('stationary_frame_stride=2','stationary_frame_stride=4')
code=code.replace('A_R180_1800_baseline_20261120_001',rid)
ast.parse(code); ast.parse(helper)
with (STAGE/'run_recorded_once.py').open('x',encoding='utf-8') as f:f.write(code)
plan=dict(mouse='A',motion='in_r45',preset_mm_s=1800,count=1,planned_start_board_mm=[225.,45.],
    planned_heading_image_deg=90.,static_apparent_reference_heading_deg=89.50081634521484,
    static_reference_basis='Recorded A1600 in_r45 successful placement at225/45; apparent marker reference only, not calibrated body yaw.',
    front_colour='red',rear_colour='yellow',pre_run_mm=90.,post_run_mm=float(90*np.sqrt(2)),
    effective_pre_accel_m_s2=18.,requested_post_diagonal_accel_m_s2=20.,stopping_distance_at20_mm=81.,
    nominal_Lend_from_start_right_forward_mm=[45.,180.],nominal_stop_right_forward_mm=[135.,270.],
    reference_nominal_stop_board_mm=[90.,315.],camera_height_mm=1500.,camera_settings=config['camera_settings'],
    placement_basis='A1600 in_r45225/45mm operator no-contact record. Current A1800 vehicle contact points/hole/full-width/turn/stopping margins still require current operator confirmation.',
    source_profile='Params/Param_A/turn_1800/turn_1800.cpp',source_profile_sha256=hashlib.sha256((ROOT/'Params/Param_A/turn_1800/turn_1800.cpp').read_bytes()).hexdigest(),
    source_params_changed=False,next_candidate_executed=False,execution_pending=False,operator_ready=False,
    power_state='BothOFF_confirmed_after_right180',next_request_id=rid,candidate=str(STAGE/'candidate.settings.json'),
    config=str(STAGE/'config.json'),time_target_minutes=20,
    workflow='One baseline, candidate, two identical repeats with concurrent MCP saved analysis. Combine stop/OFF-reposition/ON readiness for same-candidate repeats.',
    guide=str(BASE/'start_placement_guide.png'))
save(BASE/'preparation.json',plan); save(STAGE/'placement_plan.json',plan)
plt.rcParams['font.family']='Yu Gothic'
fig,ax=plt.subplots(figsize=(6.5,6.5),layout='constrained')
for v in range(0,361,45):
    ax.axvline(v,color='.85',lw=.7); ax.axhline(v,color='.85',lw=.7)
ax.plot([225,225],[45,135],label='前走90 mm')
t=np.linspace(0,1,100)[:,None]
curve=(1-t)**3*np.array([225,135])+3*(1-t)**2*t*np.array([225,185])+3*(1-t)*t**2*np.array([205,200])+t**3*np.array([180,225])
ax.plot(curve[:,0],curve[:,1],'--',label='45°入口の目標接続線（模式）')
ax.plot([180,90],[225,315],label='斜め後走127.3 mm')
ax.plot([225,225],[20,70],c='.35',lw=3)
ax.scatter(225,70,c='red',marker='v',s=90,label='前：赤、画像下向き'); ax.scatter(225,20,c='gold',edgecolors='black',s=70,label='後：黄')
ax.scatter(225,45,c='blue',s=35); ax.annotate('開始中点 (225,45) mm',xy=(225,45),xytext=(15,95),arrowprops=dict(arrowstyle='->'))
ax.scatter(90,315,c='black',marker='x',s=50,label='目標停止点（基準）')
ax.set(xlim=(-10,370),ylim=(370,-10),aspect='equal',xlabel='盤面右方向 [mm]',ylabel='盤面下方向 [mm]',title='機体A / 1800 mm/s / 右45°入口\n穴・車幅・旋回と停止余裕を実物で確認')
ax.legend(loc='lower right',fontsize=8); fig.savefig(BASE/'start_placement_guide.png',dpi=140); plt.close(fig)
print('A1800 in_r45 source baseline and layout prepared; hardware operations:0')
