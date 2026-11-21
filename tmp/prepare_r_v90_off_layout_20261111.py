"""右V90の電源OFF配置確認だけを準備する。BLE接続・走行指令は行わない。"""
from pathlib import Path
from datetime import datetime, timezone, timedelta
import hashlib
import json
import math
import py_compile
import shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt, font_manager

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera'
SOURCE=BASE/'cli_out_l45_cminus05_pose_repeat03_retry_ready_20261111_171826'
checkpoint=BASE/'current_ui_tuning_session.json'
state=json.loads(checkpoint.read_text(encoding='utf-8'))
assert state['operator_power_off_confirmed'] and state['gateway_stopped'] and not state['any_execution_pending']
stamp=datetime.now(timezone(timedelta(hours=9))).strftime('%Y%m%d_%H%M%S')
folder=BASE/('cli_r_v90_off_layout_ready_'+stamp);folder.mkdir()
shutil.copy2(SOURCE/'config.json',folder/'config.json')
diagonal=63.6396
target=dict(right_mm=90/math.sqrt(2)+diagonal,forward_mm=90/math.sqrt(2)+2*diagonal,exit_right_positive_deg=90)
plan=dict(recorded_at=datetime.now(timezone.utc).isoformat(),mouse='A',motion='r_v90',preset=500,
    expected_source_parameters=dict(velo_m_s=.5,r_min_mm=-41.,lstart_mm=6.36,lend_mm=12.60,degree_left_positive_deg=-90.,correction_deg=0.),
    actual_settings_verified=False,pre_run_diagonal_mm=2*diagonal,post_run_diagonal_mm=diagonal,
    target_endpoint_in_initial_heading_frame=target,
    target_geometry_basis_ja='90 mmセルのV90設計変位を開始斜め方向へ変換し、シェルの斜め前走2区画・後走1区画を加える。',
    source_parameters='Params/Param_A/turn_500.h:slalom_RV90_500_table',
    execution_geometry_source='System/Src/myshell.cpp:shell_debug_turn pre/post run, Params/mouse_config.h:DIAG_SECTION',
    nominal_proposed_start_pose_xy_mm=[180,35],nominal_proposed_heading_deg=135,
    image_view_start_from_left_mm=180,image_view_start_from_top_mm=35,
    proposed_image_view_heading='red front down-right, yellow rear up-left',
    setup_proposal_image=str(folder/'r_v90_setup_proposal.png'),camera_height_mm=800,exposure_ev=-1.5,
    focus_lock_required=True,current_power='OFF_CONFIRMED',current_ble='CLOSED',
    first_step='Keep A/HY0020 OFF; place robot, confirm holes, then passive static capture only',
    body_width_and_stopping_margin_operator_confirmation_required=True,hole_clearance_verified=False,
    one_motion_authorized=False,execution_sent=False,
    note_ja='配置図は概略で、穴の位置を記載していない。実物で機体幅・旋回のふくらみ・停止余裕を確認し、穴にかかるなら別の位置を選ぶ。右V90の実機500プリセットは後で新規取得する。')
with (folder/'r_v90_hole_avoidance_layout_plan.json').open('x',encoding='utf-8') as file:
    json.dump(plan,file,ensure_ascii=False,indent=2)

code=(SOURCE/'static_probe.py').read_text(encoding='utf-8')
def replace_once(old,new):
    global code
    assert code.count(old)==1,(old,code.count(old))
    code=code.replace(old,new)
replace_once("choices=['setup','powered']","choices=['setup']")
replace_once("identifier='out_l45_cminus05_pose_cli_005'","identifier='r_v90_baseline_cli_001'")
replace_once("r_min']==50 and parameters['degree']==45 and parameters['correction']==-.5", "r_min']==-41 and parameters['degree']==-90 and parameters['correction']==0")
replace_once("exit_forward=(forward-right)/math.sqrt(2)","exit_forward=right")
replace_once("assert target['right_mm']<0 and target['exit_right_positive_deg']==-45", "assert target['right_mm']>0 and target['exit_right_positive_deg']==90")
start=code.index("    pose_reference_path=SESSION/'initial_pose_reference.json'")
end=code.index("    report=dict(",start)
code=code[:start]+"    accepted=bool(len(pose)>=40 and calibration>=.99 and recognition>=.99 and rms<=.5 and distance.max()<=1.5 and abs(diagonal_error)<=5 and inside)\n"+code[end:]
start=code.index('        initial_pose_matches_reference=pose_matches,')
end=code.index('        expected_parameters_are_actual_device_readback=',start)
code=code[:start]+code[end:]
code=code.replace(", 'initial_centre_difference_mm'",'')
code=code.replace("'initial_centre_difference_mm','initial_heading_difference_deg','initial_pose_matches_reference',",'')
code=code.replace('out_l45_hole_avoidance_layout_plan.json','r_v90_hole_avoidance_layout_plan.json')
code=code.replace('post_run_straight_mm','post_run_diagonal_mm')
code=code.replace('out_l45','r_v90').replace('out_L45','R V90')
code=code.replace('左45度出口なので右の式の+rightを流用せず、-rightで左へ向ける。','右V90なので出口方向は開始時のright軸。')
code=code.replace('Static recognition, reference initial pose or nominal footprint failed; do not execute motion','Static recognition or nominal footprint failed; keep robot power OFF')
(folder/'static_probe.py').write_text(code,encoding='utf-8')
py_compile.compile(str(folder/'static_probe.py'),doraise=True)
assert "initial_pose_matches_reference" not in code

for name in ['Yu Gothic','MS Gothic']:
    try:
        font_manager.findfont(name,fallback_to_default=False)
        plt.rcParams['font.family']=name
        break
    except ValueError:
        pass
plt.rcParams['axes.unicode_minus']=False
start=np.array([180.,35.]);forward=np.array([1.,1.])/math.sqrt(2);right=np.array([-1.,1.])/math.sqrt(2)
end=start+forward*target['forward_mm']+right*target['right_mm']
pre=start+forward*(2*diagonal+6.36);post=end-right*(diagonal+12.60)
alpha,_=np.linalg.solve(np.column_stack([forward,right]),post-pre)
control=pre+alpha*forward;t=np.linspace(0,1,80)[:,None]
curve=(1-t)**2*pre+2*(1-t)*t*control+t**2*post
fig,ax=plt.subplots(figsize=(7.2,8.0),dpi=160)
ax.set(xlim=(0,360),ylim=(360,0),aspect='equal',xticks=np.arange(0,361,45),yticks=np.arange(0,361,45),xlabel='画面の盤面左端から [mm]',ylabel='盤面上端から [mm]')
ax.grid(color='#cccccc',linewidth=.7)
path=np.vstack([start,pre,curve,post,end]);ax.plot(path[:,0],path[:,1],color='#008e55',linewidth=2.5,label='名目経路の概略')
front=start+27.5*forward;rear=start-27.5*forward
ax.plot([rear[0],front[0]],[rear[1],front[1]],color='#444444',linewidth=4)
ax.scatter(*start,s=95,facecolors='white',edgecolors='#008e55',linewidths=2,zorder=5)
ax.scatter(*front,s=65,color='red',zorder=6);ax.scatter(*rear,s=65,color='#e5bf00',zorder=6)
ax.annotate('前F（赤）',front,xytext=(218,66),arrowprops=dict(arrowstyle='->',color='red'))
ax.annotate('後R（黄）',rear,xytext=(70,12),arrowprops=dict(arrowstyle='->',color='#a08000'))
ax.annotate('機体中心：左右中央・上端から約35 mm',start,xytext=(12,96),fontsize=9,arrowprops=dict(arrowstyle='->'))
ax.annotate('',xy=pre,xytext=start+forward*48,arrowprops=dict(arrowstyle='->',color='#008e55',lw=2))
ax.annotate('',xy=end,xytext=post,arrowprops=dict(arrowstyle='->',color='#008e55',lw=2))
ax.scatter(*end,marker='x',s=90,color='#a000b0',linewidths=2);ax.text(end[0]-105,end[1]+20,'停止目標（概略）',color='#9000a0')
ax.set_title('右V90の配置案：電源OFFで設置確認\n赤い前マーカーは画面右下、黄色い後マーカーは左上',fontsize=12)
fig.text(.10,.015,'穴は図に未記載。機体幅・旋回のふくらみ・停止余裕は現物で確認してください。',fontsize=9)
fig.tight_layout(rect=(0,.04,1,1));fig.savefig(folder/'r_v90_setup_proposal.png');plt.close(fig)
validation=dict(recorded_at=datetime.now(timezone.utc).isoformat(),syntax_compilation_passed=True,
    no_hardware_operations=True,prepared_while_mouse_and_hy0020_off=True,passive_static_only=True,
    motor_execution_authorized=False,prior_out_l45_reference_not_applied_to_new_motion=True,
    input_sha256={str(folder/name):hashlib.sha256((folder/name).read_bytes()).hexdigest() for name in ['static_probe.py','config.json','r_v90_hole_avoidance_layout_plan.json','r_v90_setup_proposal.png']})
with (folder/'prepared_helpers_validation.json').open('x',encoding='utf-8') as file: json.dump(validation,file,ensure_ascii=False,indent=2)
state.update(state='OUT_L45_CANDIDATE_SAVED_OFF_RV90_PASSIVE_LAYOUT_PREPARED',next_motion='r_v90',prepared_next_cli_folder=str(folder),
    next_candidate_plan_json=str(folder/'r_v90_hole_avoidance_layout_plan.json'),next_setup_static_accepted=False,
    next_setup_actual_hole_confirmation_required=True,conditional_one_shot_authorized=False,
    updated_at=validation['recorded_at'])
checkpoint.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(prepared_folder=str(folder),static_only=True,motor_authorized=False,nominal_endpoint_image_view_mm=end.tolist(),target=target),ensure_ascii=False))
