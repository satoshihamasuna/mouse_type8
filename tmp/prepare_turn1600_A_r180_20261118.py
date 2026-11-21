"""機体A・右180度の未実行計画と配置図を保存する。機器操作なし。"""
from pathlib import Path
import hashlib
import json
import shutil

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OLD = ROOT / 'tools/logs/pixel_camera/turn1600_A_r90_20261118_frame18_v1'
BASE = ROOT / 'tools/logs/pixel_camera/turn1600_A_r180_20261118_v1'
BASE.mkdir(exist_ok=False)
STAGE = BASE / 'baseline01'
STAGE.mkdir()


def create(path, value):
    with path.open('x', encoding='utf-8') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')


reference = BASE / 'marker_reference_A_current.json'
geometry = BASE / 'body_geometry_A_yellow_v2.json'
shutil.copyfile(OLD / 'marker_reference_A_current.json', reference)
shutil.copyfile(OLD / 'analysis_yellow_v2/body_geometry_A_yellow_v2.json', geometry)
create(STAGE / 'candidate.settings.json', dict(schema_version=1, mouse='A',
    mouse_identity_source='user_supplied', motion_name='long_r180',
    preset_speed_mm_s=1600, turn_count=1,
    parameters=dict(velo=1.6, r_min=-48., lstart=13.12, lend=27.38,
        degree=-180., correction=0., pre_accel=6.5),
    pid=dict(sp_kp=2., sp_ki=.016, sp_kd=0., om_kp=.20, om_ki=.012, om_kd=0.),
    feedforward=dict(ff_sp_velo=.9030126, ff_sp_accel=.08101898, ff_sp_bias=.2138578,
        ff_om_velo=.0168, ff_om_accel=.00088, ff_om_decel=.00085,
        ff_om_bias=.0613, ff_om_jerk=.0000032,
        ff_sp_turn_accel_mag=-.00024186, ff_sp_turn_velo_sq=.00008612),
    suction=dict(enabled=True, duty=650),
    source='Current A long180 compiled table baseline; unexecuted. Fresh device readback required.'))
create(STAGE / 'config.json', dict(mouse='A', address='C1:B5:60:87:91:73',
    height_mm=1500., reference=str(reference), geometry=str(geometry),
    char_delay=.08, seconds=20,
    camera_settings=dict(ev=-2., zoom_ratio=1.8, torch=False, focus_lock=True)))
plan = dict(status='OFF_PLACEMENT_REQUIRED_UNEXECUTED', mouse='A', motion='long_r180',
    preset_mm_s=1600, planned_start_board_mm=[315., 135.],
    planned_heading_image_deg=90., front_colour='red', rear_colour='yellow',
    planned_request_id='A_R180_1600_baseline_20261118_001', count=1,
    pre_run_mm=90., post_run_mm=90.,
    configured_turn_pre_accel_m_s2=6.5, effective_pre_accel_from_source_m_s2=1.6**2/(2*.09),
    requested_post_straight_accel_m_s2=20., post_condition_applied=False,
    stop_distance_at_default6p5_mm=1000*1.6**2/(2*6.5),
    stop_distance_at20_mm=1000*1.6**2/(2*20),
    post_setup_reason='The90mm post-run cannot ramp1.6m/s down to zero at6.5m/s2. Set only debug-straight acceleration to20 after reading/preserving its other fields; record full actual response. Do not infer Lend from stopping overrun.',
    reference_turn_end_displacement_right_forward_mm=[90., 0.],
    reference_forward_apex_from_turn_entry_mm=95.,
    reference_nominal_stop_board_mm=[225.,135.],
    reference_nominal_forward_apex_board_y_mm=320.,
    planned_start_basis='Same X315 incoming grid line used successfully for A right90; translate start90mm down for upper stopping clearance. Historical A500 right180 start was approximately225/45; its speed/suction/framing differ and it is not a clearance proof for1600.',
    placement_status='Proposed; not yet statically or mechanically confirmed for A1600 right180.',
    hole_clearance_verified=False, body_width_and_contact_points_verified=False,
    current_placement_confirmed=False, current_clearance_confirmed=False,
    operator_current_readiness_confirmed=False, power_state='Both OFF requested; last confirmed after right90 batch',
    ble_disconnected=True, execution_commands_sent=0, execution_pending=False,
    capture_defer_analysis=True, source_params_changed=False,
    analysis_scope='Relative marker midpoint and measured entry direction; not fixed-board/absolute axle acceptance.',
    long180_identifiability='Parallel input/output makes terminal longitudinal displacement insufficient to separate Lstart/Lend. Review forward apex and actual phase-aligned trajectory in addition to output-line spacing/angle.',
    next_step='Operator places with bothOFF at315/135mm, redfront down. Capture camera-only static pose. Confirm hole/wall/all contact-point/full-width/turn bulge/stopping clearance before bothON/hands-clear request. Fresh battery/modes/straight-profile/device readback; set post condition and baseline with verification; exactly one unique capture request.',
    source_hashes={str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest()
        for path in (ROOT/'Params/Param_A/turn_1600/turn_1600.cpp',
                     ROOT/'System/Src/myshell.cpp', ROOT/'Task/Src/motion.cpp')})
create(BASE / 'preparation.json', plan)
create(STAGE / 'preparation.json', plan)

plt.rcParams['font.family'] = ['Yu Gothic', 'Meiryo', 'DejaVu Sans']
fig, ax = plt.subplots(figsize=(7, 7), layout='constrained')
ax.set(xlim=(-15,375), ylim=(375,-15), aspect='equal',
    xlabel='盤面左端から X [mm]', ylabel='盤面上端から Y [mm]')
ax.set_xticks(np.arange(0,361,45)); ax.set_yticks(np.arange(0,361,45))
ax.grid(color='.8', linewidth=.7)
ax.plot([0,360,360,0,0],[0,0,360,360,0],color='black')
ax.plot([315,315],[135,225],color='tab:blue',label='前走90 mm')
theta=np.linspace(0,np.pi,150)
ax.plot(270+45*np.cos(theta),225+95*np.sin(theta),'--',color='tab:green',
    label='目標のU字形状（実軌道予測ではない）')
ax.plot([225,225],[225,135],color='tab:orange',label='後走90 mm')
ax.scatter(315,135,color='tab:blue',s=70,zorder=4)
ax.plot([315,315],[110,160],color='.4',linewidth=2)
ax.scatter(315,160,color='red',marker='v',s=100,zorder=5,label='前：赤（画像下向き）')
ax.scatter(315,110,color='gold',edgecolor='black',s=85,zorder=5,label='後：黄')
ax.annotate('開始：前後マーカー中点\n(315, 135) mm',xy=(315,135),xytext=(105,70),
    arrowprops=dict(arrowstyle='->',color='tab:blue'),fontsize=11)
ax.scatter(225,135,marker='x',s=70,color='black')
ax.set_title('機体A・turn1600 右180°：両方OFFで配置\n45 mm格子／右端から1マス・上端から3マス\n穴・全接地点・車幅・旋回・停止余裕は実配置で確認',fontsize=11)
ax.legend(loc='lower left',fontsize=8)
fig.savefig(BASE/'start_placement_plan.png',dpi=160)
plt.close(fig)
print(BASE, flush=True)
