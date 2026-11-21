"""A1800右180度の未実行基準設定と配置図を準備する。実機接続なし。"""
from pathlib import Path
import json
import shutil
import ast
import hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
OLD=ROOT/'tools/logs/pixel_camera/turn1800_A_r90_20261119_v1'
BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_r180_20261120_v1'
BASE.mkdir(exist_ok=False); STAGE=BASE/'baseline01'; STAGE.mkdir()


def save(path,value):
    with path.open('x',encoding='utf-8') as f:
        json.dump(value,f,ensure_ascii=False,indent=2)


for name in ('marker_reference_A_current.json','body_geometry_A_yellow_v2.json'):
    shutil.copy2(OLD/name,BASE/name)
config=json.loads((OLD/'repeat_c1p25_03/config.json').read_text(encoding='utf-8'))
config.update(reference=str(BASE/'marker_reference_A_current.json'),geometry=str(BASE/'body_geometry_A_yellow_v2.json'))
save(STAGE/'config.json',config)
candidate=json.loads((OLD/'repeat_c1p25_03/candidate.settings.json').read_text(encoding='utf-8'))
candidate.update(source='Unexecuted A1800 long_r180 source baseline. Separate coefficients from right90; no A1600 tuning values adopted.',motion_name='long_r180')
candidate['parameters'].update(r_min=-48.,lstart=7.52,lend=37.68,degree=-180.,correction=0.)
candidate['pid'].update(sp_ki=.2,om_kp=.1,om_ki=.002)
candidate['feedforward'].update(ff_sp_velo=.9030126,ff_sp_accel=.08101898,ff_sp_bias=.2138578,
    ff_om_accel=.00088,ff_om_decel=.00090)
save(STAGE/'candidate.settings.json',candidate)
for name in ('static_pose_after_settle.py','static_settle_policy.json','static_gate_v2_source.py','static_gate_v2_validation.json'):
    shutil.copy2(OLD/'repeat_c1p25_03'/name,STAGE/name)
rid='A_R180_1800_baseline_20261120_001'
code=(OLD/'repeat_c1p25_03/run_recorded_once.py').read_text(encoding='utf-8')
assert code.count('y_mm=45.')==1
code=code.replace('y_mm=45.','y_mm=135.').replace('A_R90_1800_c1p25_repeat03_20261120_001',rid)
ast.parse(code)
with (STAGE/'run_recorded_once.py').open('x',encoding='utf-8') as f: f.write(code)
source=ROOT/'Params/Param_A/turn_1800/turn_1800.cpp'
plan=dict(mouse='A',motion='long_r180',preset_mm_s=1800,count=1,
    planned_start_board_mm=[315.,135.],planned_heading_image_deg=90.,
    static_apparent_reference_heading_deg=89.26036071777344,
    static_reference_basis='Same camera setup and X315 incoming line as current successful right90; this is a repeatable apparent marker reference, not calibrated body yaw.',
    front_colour='red',rear_colour='yellow',pre_run_mm=90.,post_run_mm=90.,
    requested_pre_accel_m_s2=6.5,effective_pre_accel_from_source_m_s2=18.,
    requested_post_straight_accel_m_s2=20.,stopping_distance_at20_mm=81.,
    suction_enabled=True,suction_duty=650,camera_settings=config['camera_settings'],camera_height_mm=1500.,
    nominal_turn_displacement_right_forward_mm=[90.,0.],
    nominal_Lend_from_start_right_forward_mm=[90.,90.],
    reference_nominal_stop_board_mm=[225.,135.],
    placement_basis='A1600 right180 used315/135mm with90mm pre/post and operator no-contact confirmations. Source1800 radius48 and effective18m/s2 acceleration are different; current vehicle/hole/full-width/turn/stopping clearance must be confirmed.',
    current_placement_confirmed=False,current_clearance_confirmed=False,operator_ready=False,
    source_params_changed=False,next_candidate_executed=False,execution_pending=False,
    baseline_source=str(source),baseline_source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    candidate=str(STAGE/'candidate.settings.json'),config=str(STAGE/'config.json'),next_request_id=rid,
    power_state='BothOFF_confirmed_after_right90',time_target_minutes=20,
    workflow='Baseline, adjust only required geometry, capture two identical repeats without waiting for analysis. Analyze saved data concurrently; archive sequentially.',
    reference_policy='Each evaluated frame requires actually decoded registered ArUcos2and3; missing frames excluded; existing quality limits retained.',
    evaluation_scope='Relative marker midpoint and measured incoming course. Fixed-board target remains separate.',
    identifiability='Parallel input/output does not independently identify Lstart/Lend from terminal position; also evaluate forward apex and phase-aligned trajectory.',
    guide=str(BASE/'start_placement_guide.png'))
save(BASE/'preparation.json',plan); save(STAGE/'placement_plan.json',plan)
plt.rcParams['font.family']='Yu Gothic'
fig,ax=plt.subplots(figsize=(6.5,6.5),layout='constrained')
for v in range(0,361,45):
    ax.axvline(v,color='.85',lw=.7); ax.axhline(v,color='.85',lw=.7)
ax.plot([315,315],[135,225],color='tab:blue',label='前走90 mm')
t=np.linspace(0,np.pi,101)
ax.plot(270+45*np.cos(t),225+45*np.sin(t),'--',color='tab:green',label='右180°の目標接続線（模式）')
ax.plot([225,225],[225,135],color='tab:orange',label='後走90 mm')
ax.plot([315,315],[110,160],color='.35',lw=3)
ax.scatter(315,160,c='red',marker='v',s=90,label='前：赤、画像下向き')
ax.scatter(315,110,c='gold',edgecolors='black',s=80,label='後：黄')
ax.scatter(315,135,c='blue',s=35)
ax.annotate('開始中点 (315,135) mm',xy=(315,135),xytext=(105,80),arrowprops=dict(arrowstyle='->'))
ax.scatter(225,135,marker='x',c='black',s=60,label='目標停止点（基準）')
ax.set(xlim=(-10,370),ylim=(370,-10),aspect='equal',xlabel='盤面右方向 [mm]',ylabel='盤面下方向 [mm]',title='機体A / 1800 mm/s / 右180°\n穴・車幅・旋回と停止余裕を実物で確認')
ax.legend(loc='lower left',fontsize=8)
fig.savefig(BASE/'start_placement_guide.png',dpi=140); plt.close(fig)
print(json.dumps(dict(stage=str(STAGE),request_id=rid,hardware_commands_sent=0),ensure_ascii=False))
