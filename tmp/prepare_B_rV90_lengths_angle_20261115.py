from pathlib import Path
import copy,json,hashlib,sys,importlib.util
import numpy as np
import cv2
from PIL import Image,ImageDraw,ImageFont
ROOT=Path.cwd();BASE=ROOT/'tools/turn_analysis/20261114_B_turn500_seed_from_A'
sys.path[:0]=[str(BASE),str(ROOT/'tools'),str(ROOT/'tools/analysis')]
from nominal_inR135 import make_path
from video_marker_pose import marker_registration,transform_points
def load(p):return json.loads(Path(p).read_text(encoding='utf-8'))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p,d):
    with Path(p).open('x',encoding='utf-8',newline='\n') as f:json.dump(d,f,ensure_ascii=False,indent=2);f.write('\n')
oldpath=BASE/'next_session_B_rV90_start315_90_v1.json';old=load(oldpath);assert old['hardware_power']=='BOTH_OFF_USER_CONFIRMED' and old['service_closed']
source_session=Path(old['session']);source_run=source_session/('run_'+old['capture_request_id']);actual=load(source_run/'motion.settings.json')
fv=Path(old['final_validity']);assert load(fv)['full_trajectory_review_passed']
model=load(BASE/'rV90_start315_90_run01_review/next_candidate_model.json')
session=ROOT/'tools/logs/pixel_camera/tuning_cli_B_rV90_Ls9_Le11p6_c1_20261115_prepared';session.mkdir(exist_ok=False)
mp=BASE/'next_session_B_rV90_Ls9_Le11p6_c1_v1.json';pp=BASE/'B_rV90_Ls9_Le11p6_c1_plan.json'
helper=BASE/'preflight_rV90_Ls9_Le11p6_c1_zoom2.py';runner=BASE/'prepare_rV90_adjustment_fast.py'
plan=copy.deepcopy(load(old['plan']));plan.update(state='UNEXECUTED_B_RV90_LENGTHS_AND_ANGLE_COMPARISON',first_B_trial=False,
    expected_candidate_parameters=model['candidate_parameters'],baseline_actual_settings=str(source_run/'motion.settings.json'),
    baseline_actual_settings_sha256=sha(source_run/'motion.settings.json'),baseline_final_validity=str(fv),baseline_final_validity_sha256=sha(fv),
    changes_from_last_trial={k:[v,model['candidate_parameters'][k]] for k,v in actual['parameters'].items() if v!=model['candidate_parameters'][k]},
    comparison_reason='Correct measured7.51mm right deficit via exit length; add0.5deg correction to reduce0.60deg over-rotation; compensate coupled forward increment via entry length. Bundle three fields to reduce trials; effects not isolated.',
    next_candidate_model=model,individual_parameter_effect_isolation=False,
    start_placement_status='REGISTERED_STATIC_REFERENCE_REPOSITION_CONFIRMATION_REQUIRED',
    fresh_repositioning_required=True,start_reference_scope='Historical registered start315/90 and135deg. Robot is now OFF at previous endpoint; fresh reset and operator confirmation required.',
    nominal_start_and_route_guide=str(BASE/'B_rV90_Ls9_Le11p6_c1_route_guide.png'),
    start_placement_registry_sha256='PENDING_REGISTRY_STATUS_UPDATE')
plan.pop('changes_from_B_preset',None)
new=copy.deepcopy(old)
for k in ['service_pid','service_closed','final_validity','recording_id','trial']:new.pop(k,None)
new.update(session=str(session),plan=str(pp),state='OFFLINE_PREPARATION_NO_HARDWARE',capture_request_id='B_rV90_Ls9_Le11p6_c1_run01',
    operator_ready='PENDING_FRESH_CONFIRMATION',hardware_power='BOTH_OFF_USER_CONFIRMED',resume_allowed=True,preflight_ready=False,
    execution_commands_sent=0,execution_resends=0,offline_preflight_helper_check_passed=False,preflight_helper=str(helper),preparation_runner=str(runner),
    preparation_validation=str(BASE/'rV90_Ls9_Le11p6_c1_preparation_validation.json'),
    start_placement_status=plan['start_placement_status'],operator_hole_clearance_pending=True)
for k in ['settings_source','preset_readback','settings']:new[k]=str(session/Path(old[k]).name)
helper.write_text(Path(old['preflight_helper']).read_text(encoding='utf-8').replace(oldpath.name,mp.name),encoding='utf-8',newline='\n')
code=(BASE/'prepare_outR135_candidate_fast.py').read_text(encoding='utf-8').replace('out_r135','r_v90').replace('B_outR135','B_rV90').replace('右135出口','右V90')
code=code.replace('OFF_POSITION_CONFIRMED_OPERATOR_CONTACT_CLEARANCE_REQUIRED',plan['start_placement_status']).replace('off_static_position_report','position_confirmation_report')
code=code.replace("assert off['state'] == 'OFF_POSITION_CONFIRMED_PENDING_OPERATOR_CONTACT_CLEARANCE'","assert off['state'] == 'POSITION_CONFIRMED_UNCHANGED_AND_OFF_BY_OPERATOR'\n    assert off['operator_position_maintained_and_power_off'] is True")
code=code.replace('placement_matches_current_off_picture_confirmed','placement_matches_registered_static_picture_confirmed').replace('OFF確認画像と今回の配置の照合が必要','登録静止画像と今回の配置の照合が必要')
code=code.replace('unapplied_A_geometry_B_OM_P010_preserving_fresh_B_FF','unapplied_B_geometry_comparison_preserving_fresh_B_FF')
runner.write_text(code,encoding='utf-8',newline='\n')
tests=(BASE/'test_prepare_rV90_registered_fast.py').read_text(encoding='utf-8').replace('prepare_rV90_registered_fast','prepare_rV90_adjustment_fast').replace('preflight_rV90_start315_90_zoom2','preflight_rV90_Ls9_Le11p6_c1_zoom2').replace('B_rV90_start315_90_plan.json',pp.name)
(BASE/'test_prepare_rV90_adjustment_fast.py').write_text(tests,encoding='utf-8',newline='\n')
save(pp,plan);save(mp,new)
config=load(new['config']);ref=load(config['reference']);geometry=load(config['geometry']);frame_path=source_session/'static_preflight_frame.png';frame=cv2.imread(str(frame_path));assert frame is not None
spec=importlib.util.spec_from_file_location('rv90_adjusted_static',helper);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
check=module.inspect_frame(frame,ref,geometry,plan['expected_candidate_parameters'],plan);assert check['ready']
forward=np.array([-1.,1.])/2**.5;right=np.array([-1.,-1.])/2**.5;start=np.array([315.,90.])
path,_=make_path(plan['expected_candidate_parameters'],127.2792,63.6396);world=start+path.forward_mm.to_numpy()[:,None]*forward+path.right_mm.to_numpy()[:,None]*right
board=marker_registration(frame,ref);assert board['available'];inverse=np.linalg.inv(np.asarray(board['homography_image_to_board_mm']))
def pt(v):return tuple(np.rint(transform_points([v],inverse)[0]).astype(int))
cv2.polylines(frame,[transform_points(world,inverse).astype(np.int32)],False,(60,220,60),3,cv2.LINE_AA)
cv2.circle(frame,pt(start),11,(255,255,0),3,cv2.LINE_AA);cv2.arrowedLine(frame,pt(start),pt(start+45*forward),(0,220,255),3,cv2.LINE_AA,tipLength=.25)
cv2.circle(frame,pt(start+27.5*forward),7,(0,255,255),-1,cv2.LINE_AA);cv2.circle(frame,pt(start-27.5*forward),7,(0,0,255),-1,cv2.LINE_AA)
cv2.drawMarker(frame,pt(plan['planned_stop_board_xy_mm']),(230,0,230),cv2.MARKER_TILTED_CROSS,18,3,cv2.LINE_AA)
image=Image.new('RGB',(1280,840),'white');image.paste(Image.fromarray(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)),(0,0));draw=ImageDraw.Draw(image);font=ImageFont.truetype('C:/Windows/Fonts/YuGothM.ttc',21)
for y,text in [(725,'B 右V90：同じ開始点へ戻す（中点は右315 mm・下90 mm、黄色前は左下向き）'),(757,'次候補：半径−41 mm、Lstart9.0 mm、Lend11.6 mm、補正＋1.0°。緑は名目経路。'),(789,'穴・全接地点・全幅・旋回と停止の余裕を現物で確認。背景写真は前回の開始位置です。')]:draw.text((18,y),text,font=font,fill='black')
image.save(BASE/'B_rV90_Ls9_Le11p6_c1_route_guide.png')
save(BASE/'rV90_Ls9_Le11p6_c1_offline_frame_check.json',dict(state='PASSED_PREVIOUS_REAL_FRAME_NOMINAL_GATE',ready=check['ready'],nominal_centreline_min_grid_distance_mm=check['nominal_centreline_to_visible_grid_point_min_mm'],physical_hole_clearance_verified=False,source_frame=str(frame_path),source_frame_sha256=sha(frame_path)))
print(json.dumps(dict(state='NEW_COMPARISON_ASSETS_READY_NO_HARDWARE',meta=str(mp),changes=plan['changes_from_last_trial']),ensure_ascii=False))
