"""A1800 out_r135の専用ソース値と実使用配置を保存する。実機操作なし。"""
from pathlib import Path
import ast
import hashlib
import json
import re
import shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT/'tools/logs/pixel_camera/turn1800_A_inr45_20261120_v1/baseline01'
BASE = ROOT/'tools/logs/pixel_camera/turn1800_A_outr135_20261120_v1'
STAGE = BASE/'baseline01'
PROFILE = ROOT/'Params/Param_A/turn_1800/turn_1800.cpp'
PROFILE_FLAG = ROOT/'Params/Param_A/run_param_A.h'
HISTORY = ROOT/'tools/logs/pixel_camera/turn1600_A_outr135_20261119_v1/next_Ls1p0_Le31p5_c1p9'
OLD = ROOT/'tools/logs/pixel_camera/cli_out_r135_lstart40_7_aftercharge_ready_20261112_082139'
RID = 'A_OUTR135_1800_baseline_20261120_001'


def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def save(path, value):
    with path.open('x', encoding='utf-8') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


if BASE.exists():
    raise FileExistsError(BASE)
assert re.search(r'^#define PARAM_A_USE_PINK_LED_TURN_LENGTHS 0$', PROFILE_FLAG.read_text(encoding='utf-8'), re.M)
text = re.sub(r'//[^\n]*', '', PROFILE.read_text(encoding='utf-8'))


def numbers(name, last=False):
    blocks = re.findall(r'\b'+re.escape(name)+r'\s*=\s*\{([^}]+)\}', text)
    assert blocks, name
    return [float(v.strip().rstrip('f')) for v in blocks[-1 if last else 0].split(',')
            if v.strip() not in ('Turn_R', 'Turn_L')]


table = numbers('slalom_outR135_1800_table', last=True)
sp, om = numbers('sp_gain_turnOut135_1800'), numbers('om_gain_turnOut135_1800')
ff = numbers('ff_gain_out135_R_1800')
assert table == [1.8, -41., 15.61, 46.65, -135.]
assert sp == [2., .05, 0.] and om == [.1, .002, 0.]
assert len(ff) == 8
ff += [0., 0.]  # t_ff_gainの末尾2項は宣言時既定値0。
ffkeys = ('ff_sp_velo', 'ff_sp_accel', 'ff_sp_bias', 'ff_om_velo', 'ff_om_accel',
          'ff_om_decel', 'ff_om_bias', 'ff_om_jerk', 'ff_sp_turn_accel_mag', 'ff_sp_turn_velo_sq')
sources = [PROFILE, PROFILE_FLAG, HISTORY/'resume_plan.json', HISTORY/'repeat_validation_v1/summary.json',
           HISTORY/'repeat_validation_v1/placement_A_OUTR135_1600_Ls1p0_Le31p5_c1p9_20261119_LOCAL010.json',
           OLD/'out_r135_hole_avoidance_layout_plan.json', OLD/'operator_stop_out_r135_lstart40_7_cli_004.json']
hashes = {str(path): sha(path) for path in sources}
history = read(HISTORY/'repeat_validation_v1/summary.json')
assert len(history['runs']) == 3
assert all(v['operator_result']['contact_detected'] is False for v in history['runs'])
BASE.mkdir(); STAGE.mkdir()
for name in ('marker_reference_A_current.json', 'body_geometry_A_yellow_v2.json'):
    shutil.copyfile(SRC.parent/name, BASE/name)
config = read(SRC/'config.json')
config.update(reference=str(BASE/'marker_reference_A_current.json'), geometry=str(BASE/'body_geometry_A_yellow_v2.json'))
save(STAGE/'config.json', config)
candidate = read(SRC/'candidate.settings.json')
candidate.update(motion_name='out_r135', source='Unexecuted source A1800 out_r135 baseline; PARAM_A_USE_PINK_LED_TURN_LENGTHS=0; no A1600 geometry/PID values adopted.')
candidate['parameters'] = dict(zip(('velo','r_min','lstart','lend','degree'), table), correction=0., pre_accel=6.5)
candidate['pid'] = dict(zip(('sp_kp','sp_ki','sp_kd','om_kp','om_ki','om_kd'), sp+om))
candidate['feedforward'] = dict(zip(ffkeys, ff))
save(STAGE/'candidate.settings.json', candidate)
for name in ('static_gate_v2_source.py','static_gate_v2_validation.json','static_pose_after_settle.py','static_settle_policy.json'):
    shutil.copyfile(SRC/name, STAGE/name)
code = (SRC/'run_recorded_once.py').read_text(encoding='utf-8-sig')
for before, after in (('A_INR45_1800_baseline_20261120_001', RID),
                      ('x_mm=225., y_mm=45., heading_deg=89.50081634521484', 'x_mm=180., y_mm=135., heading_deg=44.15'),
                      ('placement, 20., None)', 'placement, None, 20.)')):
    assert code.count(before) == 1, before
    code = code.replace(before, after)
ast.parse(code); compile(code, str(STAGE/'run_recorded_once.py'), 'exec')
with (STAGE/'run_recorded_once.py').open('x', encoding='utf-8') as stream:
    stream.write(code)
placement = dict(x_mm=180., y_mm=135., heading_deg=44.15, position_tolerance_mm=5., heading_tolerance_deg=2.)
plan = dict(mouse='A', motion='out_r135', preset_mm_s=1800, count=1,
    placement=placement, planned_start_board_mm=[180.,135.], planned_heading_image_deg=45.,
    static_apparent_reference_heading_deg=44.15,
    static_reference_basis='Latest A1600 LOCAL010 accepted apparent heading44.15; marker appearance, not calibrated body yaw. Historical source has older y131 plan then accepted y135.3 actual envelope.',
    latest_accepted_A1600_placement=read(sources[4])['placement'],
    front_colour='red', rear_colour='yellow', pre_run_mm=float(90*np.sqrt(2)), post_run_mm=90.,
    effective_pre_accel_m_s2=float(18/np.sqrt(2)), requested_post_straight_accel_m_s2=20.,
    requested_pre_accel_m_s2=6.5, stopping_distance_at20_mm=81.,
    nominal_Lend_board_mm=[225.,315.], reference_nominal_stop_board_mm=[135.,315.],
    source_profile=str(PROFILE), source_profile_sha256=sha(PROFILE), source_flag=0,
    camera_height_mm=1500., camera_settings=config['camera_settings'], suction=candidate['suction'],
    source_params_changed=False, next_candidate_executed=False, execution_pending=False,
    operator_ready=False, current_placement_confirmed=False, current_clearance_confirmed=False,
    power_state='BothOFF_reported_by_root; current confirmation required for future run',
    request_id=RID, next_request_id=RID, candidate=str(STAGE/'candidate.settings.json'), config=str(STAGE/'config.json'),
    placement_basis='A1600 same-candidate three contact-free runs and historical A500 hole-avoidance atapproximately180/135. Reuse this vicinity; do not reuse withdrawn A/B proposals. A1800 dedicated radius/lengths/speed differ and clearance remains unverified.',
    clearance_model_verified=False, current_contact_points_holes_full_width_turn_bulge_stopping_room_required=True,
    geometry_prediction='Connection schematic only; no fitted A1800 trajectory or whole-body clearance guarantee',
    source_hashes=hashes, guide=str(BASE/'start_placement_guide.png'), hardware_commands_sent=0)
save(BASE/'preparation.json', plan); save(STAGE/'placement_plan.json', plan)
save(STAGE/'phase.json', dict(state='BASELINE_PREPARED_NOT_EXECUTED', operator_ready=False, request_id=RID,
     candidate=plan['candidate'], config=plan['config'], turn_count=1, execution_write_attempted=False))
plt.rcParams['font.family'] = 'Yu Gothic'
fig, ax = plt.subplots(figsize=(7,7), constrained_layout=True)
for v in range(0,361,45):
    ax.axvline(v,color='.86',lw=.7); ax.axhline(v,color='.86',lw=.7)
holes = np.array([(x,y) for x in range(0,361,90) for y in range(0,361,90)])
ax.scatter(holes[:,0],holes[:,1],s=45,c='black',label='穴の格子位置（実径/接地余裕は未検証）')
ax.plot([180,270],[135,225],label='斜め前走127.3 mm')
t=np.linspace(0,1,100)[:,None]
curve=(1-t)**3*np.array([270,225])+3*(1-t)**2*t*np.array([310,265])+3*(1-t)*t**2*np.array([290,315])+t**3*np.array([225,315])
ax.plot(curve[:,0],curve[:,1],'--',label='右135° 接続模式（予測軌跡ではない）')
ax.plot([225,135],[315,315],label='直線後走90 mm')
direction=np.array([1.,1.])/np.sqrt(2); start=np.array([180.,135.])
front,rear=start+25*direction,start-25*direction
ax.plot([rear[0],front[0]],[rear[1],front[1]],c='.4',lw=3)
ax.scatter(*front,c='red',s=100,marker='>',label='前：赤、右下45°付近')
ax.scatter(*rear,c='gold',s=80,edgecolors='black',label='後：黄')
ax.scatter(*start,c='blue',s=30)
ax.annotate('開始中点 (180,135) mm\n見かけ方位44.15°',xy=start,xytext=(10,65),arrowprops=dict(arrowstyle='->'))
ax.scatter(135,315,c='blue',marker='x',s=60,label='名目停止点（基準）')
ax.set(xlim=(-10,370),ylim=(370,-10),aspect='equal',xlabel='盤面右方向 [mm]',ylabel='盤面下方向 [mm]',
       title='機体A / 1800 mm/s / 右135°出口\n過去の実使用付近。A1800の穴・全幅・停止余裕は現物確認が必要')
ax.legend(loc='upper right',fontsize=7)
fig.savefig(BASE/'start_placement_guide.png',dpi=150); plt.close(fig)
assert all(sha(Path(path)) == value for path,value in hashes.items())
print('PREPARED_NOT_EXECUTED', BASE)
