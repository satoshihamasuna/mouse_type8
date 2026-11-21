from pathlib import Path
from datetime import datetime, timezone
import json, hashlib, copy, sys
import numpy as np
import cv2
from PIL import Image, ImageDraw, ImageFont
ROOT=Path.cwd()
BASE=ROOT/'tools/turn_analysis/20261114_B_turn500_seed_from_A'
sys.path[:0]=[str(BASE),str(ROOT/'tools'),str(ROOT/'tools/analysis')]
from nominal_inR135 import make_path
from video_marker_pose import marker_registration, transform_points
def load(p):return json.loads(Path(p).read_text(encoding='utf-8'))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p,d):
    with Path(p).open('x',encoding='utf-8',newline='\n') as f:
        json.dump(d,f,ensure_ascii=False,indent=2);f.write('\n')

oldmeta_path=BASE/'next_session_B_outR135_corr1p5_Lstart33p7_repeat03_v8.json'
oldmeta=load(oldmeta_path)
assert oldmeta['hardware_power']=='BOTH_OFF_USER_CONFIRMED' and oldmeta['service_closed']
session=ROOT/'tools/logs/pixel_camera/tuning_cli_B_rV90_Aseed_20261115_prepared'
session.mkdir(exist_ok=False)
meta_path=BASE/'next_session_B_rV90_Aseed_v1.json'
plan_path=BASE/'B_rV90_Aseed_plan.json'
helper=BASE/'preflight_rV90_Aseed_zoom2.py'
runner=BASE/'prepare_rV90_candidate_fast.py'
A_path=ROOT/'tools/logs/pixel_camera/cli_r_v90_10_6_3_6_c05_repeat03_ready_20261111_182538/r_v90_candidate.json'
A=load(A_path); assert A['count']==3
plan=copy.deepcopy(load(oldmeta['plan']))
for key in ['baseline_actual_settings','baseline_final_validity','baseline_actual_settings_sha256','baseline_final_validity_sha256',
            'changes_from_last_trial','next_candidate_model','same_settings_valid_trials_before_capture','candidate_bundled_compensation','individual_parameter_effect_isolation']:
    plan.pop(key,None)
compiled=dict(velo=.5,r_min=-41.,lstart=6.36,lend=12.60,degree=-90.,correction=0,pre_accel=6.5)
plan.update(mouse='B',motion='r_v90',state='UNEXECUTED_FIRST_B_RV90_A_GEOMETRY_B_CONTROL',first_B_trial=True,
    expected_compiled_B=compiled,expected_candidate_parameters=A['actual_settings']['parameters'],
    initial_geometry_from_A=A['actual_settings']['parameters'],pre_run_mm=127.2792,post_run_mm=63.6396,
    target_right_forward_mm=[A['target_endpoint_right_mm'],A['target_endpoint_forward_mm']],
    target_exit_right_forward_unit_vector=[1,0],target_exit_angle_in_right_forward_plane_deg=0,
    planned_stop_board_xy_mm=[225,270],planned_exit_direction_board_deg=135,
    planned_entry_line_board_xy_mm=[[180,45],[270,135]],planned_exit_line_board_xy_mm=[[270,225],[225,270]],
    A_reference=dict(source=str(A_path),sha256=sha(A_path),valid_trials=3,status='A measured geometry reference only; no A PID or FF'),
    B_feedforward_policy='Preserve all freshly read B r_v90 FF, SP PID and OM I/D. Seed OM P0.10 from B measured turns; first B r_v90 still unverified.',
    B_control_reference=[dict(path=str(BASE/'B_outR135_single_turn_candidate.settings.json'),sha256=sha(BASE/'B_outR135_single_turn_candidate.settings.json'))],
    comparison_reason='Initial B V90 using measured A geometry; preserve fresh B control and all FF, with B OM P0.10 reference.',
    geometry_source=['System/Src/myshell.cpp: diagonal pre127.2792/post63.6396 mm',
        'A r_v90 3-trial candidate: fixed target R127.2792103/F190.9188103 mm; exit+90deg relative right-positive'],
    changes_from_B_preset={k:[v,A['actual_settings']['parameters'][k]] for k,v in compiled.items() if v!=A['actual_settings']['parameters'][k]},
    start_change_reason='Retain B used start180/45 and45deg; A accepted V90 start approx315/90 and135deg maps to same90mm hole-grid phase under a90deg rotation. Wheel/body clearance must be confirmed for B and this route.',
    A_actual_used_start_board_xy_mm=[317.9294891357422,88.06889343261719],A_actual_used_heading_board_deg=134.76422000594135,
    A_operator_hole_clearance_evidence=str(A_path.parent/'operator_stop_r_v90_10_6_3_6_c05_cli_003.json'))
for key in ['source_A','source_B']:plan[key]['sha256']=sha(ROOT/plan[key]['path'])
plan['start_placement_registry_sha256']='PENDING_REGISTRY_UPDATE'
save(plan_path,plan)
meta=copy.deepcopy(oldmeta)
for k in ['recording_id','trial','final_validity','service_pid','service_closed','latest_final_validity','candidate_meta','same_settings_valid_trials']:
    meta.pop(k,None)
meta.update(motion='r_v90',state='OFFLINE_PREPARATION_IN_PROGRESS_NO_HARDWARE',session=str(session),plan=str(plan_path),
    capture_request_id='B_rV90_Aseed_run01',operator_ready='PENDING_FRESH_CONFIRMATION',hardware_power='BOTH_OFF_USER_CONFIRMED',
    execution_commands_sent=0,execution_resends=0,preflight_ready=False,resume_allowed=True,operator_hole_clearance_pending=True,
    offline_preflight_helper_check_passed=False,preflight_helper=str(helper),preparation_runner=str(runner),
    offline_reference_check=str(BASE/'A_rV90_reference_check/validation.json'),
    preparation_validation=str(BASE/'rV90_Aseed_preparation_validation.json'),next_target='r_v90')
for key,name in [('settings_source','B_rV90_candidate_from_fresh_B.settings.json'),('preset_readback','B_rV90_preset500.settings.json'),('settings','B_rV90_candidate.settings.json')]:meta[key]=str(session/name)
save(meta_path,meta)
source=Path(oldmeta['preflight_helper']).read_text(encoding='utf-8').replace(oldmeta_path.name,meta_path.name).replace('out_r135','r_v90').replace('out_R135','r_V90').replace('右135度出口','右V90')
helper.write_text(source,encoding='utf-8',newline='\n')
source=Path(oldmeta['preparation_runner']).read_text(encoding='utf-8').replace('out_r135','r_v90').replace('B_outR135','B_rV90').replace('右135出口','右V90')
begin=source.index("    for field in ('baseline_actual_settings', 'baseline_final_validity'):")
end=source.index('    return reference, plan',begin)
source=source[:begin]+"    assert reference['count'] == 3 and plan['first_B_trial'] is True\n    assert {key:[value,plan['expected_candidate_parameters'][key]] for key,value in plan['expected_compiled_B'].items() if value != plan['expected_candidate_parameters'][key]} == plan['changes_from_B_preset']\n"+source[end:]
source=source.replace("    assert baseline['feedforward'] == load(plan['baseline_actual_settings'])['feedforward']\n    assert baseline['suction'] == load(plan['baseline_actual_settings'])['suction']\n",'')
runner.write_text(source,encoding='utf-8',newline='\n')
tests=(BASE/'test_prepare_outR135_candidate_fast.py').read_text(encoding='utf-8')
tests=tests.replace('prepare_outR135_candidate_fast','prepare_rV90_candidate_fast').replace('preflight_outR135_r32p3_zoom2','preflight_rV90_Aseed_zoom2').replace('B_outR135_r32p3_plan.json','B_rV90_Aseed_plan.json').replace('OutR135PreparationGates','RV90PreparationGates').replace("motion_name='out_r135'","motion_name='r_v90'")
(BASE/'test_prepare_rV90_candidate_fast.py').write_text(tests,encoding='utf-8',newline='\n')
control=(BASE/'review_outR135_control.py').read_text(encoding='utf-8').replace('out_r135','r_v90').replace('右135','右V90')
(BASE/'review_rV90_control.py').write_text(control,encoding='utf-8',newline='\n')

# Reuse a recent image only as a clearly labelled placement guide; fresh static capture still required.
frame_path=Path(oldmeta['session'])/'static_preflight_frame.png';frame=cv2.imread(str(frame_path));assert frame is not None
config=load(meta['config']);ref=load(config['reference']);board=marker_registration(frame,ref);assert board['available']
inverse=np.linalg.inv(np.asarray(board['homography_image_to_board_mm']))
path,_=make_path(plan['expected_candidate_parameters'],plan['pre_run_mm'],plan['post_run_mm'])
forward=np.array([1.,1.])/2**.5;right=np.array([-1.,1.])/2**.5;start=np.array([180.,45.])
world=start+path.forward_mm.to_numpy()[:,None]*forward+path.right_mm.to_numpy()[:,None]*right
holes=np.array([[x,y] for x in range(0,361,90) for y in range(0,361,90)])
minimum=float(np.linalg.norm(world[:,None,:]-holes[None,:,:],axis=2).min())
plan.update(nominal_centreline_to_visible_grid_point_min_mm=minimum,nominal_start_and_route_guide=str(BASE/'B_rV90_start180_45_route_guide.png'),
    physical_hole_clearance_verified_by_model=False,nominal_board_centre_min_xy_mm=world.min(0).tolist(),nominal_board_centre_max_xy_mm=world.max(0).tolist())
plan_path.write_text(json.dumps(plan,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
pixels=transform_points(world,inverse).astype(np.int32);cv2.polylines(frame,[pixels],False,(60,220,60),3,cv2.LINE_AA)
def point(p):return tuple(np.rint(transform_points([p],inverse)[0]).astype(int))
cv2.circle(frame,point(start),11,(255,255,0),3,cv2.LINE_AA)
cv2.arrowedLine(frame,point(start),point(start+45*forward),(0,220,255),3,cv2.LINE_AA,tipLength=.25)
cv2.circle(frame,point(start+27.5*forward),7,(0,255,255),-1,cv2.LINE_AA)
cv2.circle(frame,point(start-27.5*forward),7,(0,0,255),-1,cv2.LINE_AA)
cv2.drawMarker(frame,point(plan['planned_stop_board_xy_mm']),(230,0,230),cv2.MARKER_TILTED_CROSS,18,3,cv2.LINE_AA)
image=Image.new('RGB',(1280,840),'white');image.paste(Image.fromarray(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)),(0,0));draw=ImageDraw.Draw(image)
font=ImageFont.truetype('C:/Windows/Fonts/YuGothM.ttc',21)
for y,text in [(725,'B 右V90：中点を右180 mm・下45 mm（右4マス・下1マス）、黄色前を右下45°'),(757,'緑＝名目経路（水色○が開始）。写真は前回の静止像で、今回の配置確認は別途行います。'),(789,'穴・タイヤ等の全接地点・機体幅・旋回と停止の余裕は現物で確認してください。')]:draw.text((18,y),text,font=font,fill='black')
image.save(BASE/'B_rV90_start180_45_route_guide.png')
save(BASE/'rV90_initial_nominal_layout.json',dict(start_board_xy_mm=[180,45],forward_board_deg=45,target_stop_board_xy_mm=[225,270],
    nominal_centreline_min_black_grid_distance_mm=minimum,full_physical_clearance_verified=False,
    source_image=str(frame_path),source_image_sha256=sha(frame_path),A_actual_used_board_pose=dict(xy_mm=[317.9294891357422,88.06889343261719],heading_deg=134.76422000594135),
    A_previous_proposed180_35_not_used=True,geometry_source=str(A_path)))
print(json.dumps(dict(state='OFFLINE_ASSETS_CREATED',meta=str(meta_path),nominal_min_grid_distance_mm=minimum),ensure_ascii=False))
