from pathlib import Path
import json,copy,hashlib,sys,importlib.util
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
oldpath=BASE/'next_session_B_rV90_Aseed_v1.json';old=load(oldpath);source_session=Path(old['session'])
assert old['hardware_power']=='BOTH_OFF_USER_CONFIRMED' and old['execution_commands_sent']==0 and old['service_closed']
power=source_session/'power_off_confirmation.json';assert load(power)['position_and_heading_maintained']
session=ROOT/'tools/logs/pixel_camera/tuning_cli_B_rV90_start315_90_20261115_prepared';session.mkdir(exist_ok=False)
mp=BASE/'next_session_B_rV90_start315_90_v1.json';pp=BASE/'B_rV90_start315_90_plan.json'
helper=BASE/'preflight_rV90_start315_90_zoom2.py';runner=BASE/'prepare_rV90_registered_fast.py'
plan=load(old['plan']);plan.update(state='UNEXECUTED_B_RV90_OBSERVED_A_USED_LAYOUT',initial_position_board_xy_mm=[315,90],
    initial_forward_board_xy=[-2**-.5,2**-.5],initial_heading_board_deg=135,
    planned_stop_board_xy_mm=[90,135],planned_exit_direction_board_deg=225,
    planned_entry_line_board_xy_mm=[[315,90],[225,180]],planned_exit_line_board_xy_mm=[[135,180],[90,135]],
    start_placement_status='STATIC_POSITION_CONFIRMED_USER_MAINTAINED_OFF_CONTACT_CLEARANCE_REQUIRED',
    start_change_reason='Fresh B static capture is approx316/88 mm and134.5deg, matching actual historical A V90 layout. Operator explicitly maintained pose and turned both OFF. Retain observed position; do not move to earlier proposed180/45.',
    original_static_only_session=str(source_session),original_static_only_motion_commands=0,
    initial180_45_proposal_not_used=True,position_confirmation_report=str(BASE/'rV90_start315_90_position_confirmation.json'),
    start_placement_registry_sha256='PENDING_REGISTRY_UPDATE',nominal_start_and_route_guide=str(BASE/'B_rV90_start315_90_route_guide.png'))
for k in ['off_static_position_report','off_static_position_report_sha256']:plan.pop(k,None)
new=copy.deepcopy(old)
for k in ['service_pid','service_closed','off_static_position_ready']:new.pop(k,None)
new.update(session=str(session),plan=str(pp),state='OFFLINE_PREPARATION_NO_HARDWARE',capture_request_id='B_rV90_start315_90_run01',
    operator_ready='PENDING_FRESH_CONFIRMATION',hardware_power='BOTH_OFF_USER_CONFIRMED',resume_allowed=True,
    preflight_ready=False,offline_preflight_helper_check_passed=False,preflight_helper=str(helper),preparation_runner=str(runner),
    preparation_validation=str(BASE/'rV90_start315_90_preparation_validation.json'),
    start_placement_status=plan['start_placement_status'],registered_static_position_ready=True,operator_hole_clearance_pending=True)
for k in ['settings_source','preset_readback','settings']:new[k]=str(session/Path(old[k]).name)
code=Path(old['preflight_helper']).read_text(encoding='utf-8').replace(oldpath.name,mp.name)
helper.write_text(code,encoding='utf-8',newline='\n')
code=Path(old['preparation_runner']).read_text(encoding='utf-8')
code=code.replace('OFF_POSITION_CONFIRMED_OPERATOR_CONTACT_CLEARANCE_REQUIRED',plan['start_placement_status'])
code=code.replace('off_static_position_report','position_confirmation_report')
code=code.replace("off = load(plan['position_confirmation_report'])","position = load(plan['position_confirmation_report'])")
code=code.replace("assert off['state'] == 'OFF_POSITION_CONFIRMED_PENDING_OPERATOR_CONTACT_CLEARANCE'", "assert position['state'] == 'POSITION_CONFIRMED_UNCHANGED_AND_OFF_BY_OPERATOR'\n    assert position['operator_position_maintained_and_power_off'] is True\n    assert position['source_capture_power'] == 'ON_STATIC_NO_MOTION'")
code=code.replace("for item in off['frames']","for item in position['frames']")
code=code.replace('placement_matches_current_off_picture_confirmed','placement_matches_registered_static_picture_confirmed')
code=code.replace('OFF確認画像と今回の配置の照合が必要','登録静止画像と今回の配置の照合が必要')
runner.write_text(code,encoding='utf-8',newline='\n')
test=(BASE/'test_prepare_rV90_candidate_fast.py').read_text(encoding='utf-8').replace('prepare_rV90_candidate_fast','prepare_rV90_registered_fast').replace('preflight_rV90_Aseed_zoom2','preflight_rV90_start315_90_zoom2').replace('B_rV90_Aseed_plan.json',pp.name).replace('placement_matches_current_off_picture_confirmed','placement_matches_registered_static_picture_confirmed')
test=test.replace('self.inspect_pose([180,45],[2**-.5,2**-.5])','self.inspect_pose([315,90],[-2**-.5,2**-.5])').replace('self.inspect_pose([180,45],[0,1])','self.inspect_pose([315,90],[0,1])')
(BASE/'test_prepare_rV90_registered_fast.py').write_text(test,encoding='utf-8',newline='\n')
spec=importlib.util.spec_from_file_location('new_rv90_preflight',helper);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
config=load(new['config']);ref=load(config['reference']);geometry=load(config['geometry'])
frame_path=source_session/'static_preflight_frame.png';frame=cv2.imread(str(frame_path));assert frame is not None
check=module.inspect_frame(frame,ref,geometry,plan['expected_candidate_parameters'],plan);assert check['ready']
original=load(source_session/'static_preflight_check.json');frames=copy.deepcopy(original['frames'])
for item in frames:
    f=np.asarray(item['forward_board_xy']);centre=np.asarray(item['centre_board_xy_mm'])
    item['heading_error_from_planned_deg']=float((np.degrees(np.arctan2(f[1],f[0]))-135+180)%360-180)
    item['planned_start_distance_mm']=float(np.linalg.norm(centre-[315,90]))
    item['ready']=bool(item['nominal_center_inside_board'] and item['nominal_markers_inside_board'] and abs(item['heading_error_from_planned_deg'])<=5 and item['planned_start_distance_mm']<=12)
assert all(i['ready'] for i in frames)
proof=dict(state='POSITION_CONFIRMED_UNCHANGED_AND_OFF_BY_OPERATOR',source_capture_power='ON_STATIC_NO_MOTION',
    source_image=str(frame_path),source_video=str(source_session/'static_preflight.json'),operator_position_maintained_and_power_off=True,
    operator_power_off_confirmation=str(power),frames=frames,independent_saved_image_recheck_ready=check['ready'],
    physical_hole_full_contact_clearance_pending=True,robot_motion_commands_sent=0,
    scope='Image was acquired powered ON, motion0. User then explicitly maintained position/heading and switched both OFF. This is not a newly acquired OFF video.',
    input_sha256={str(p):sha(p) for p in [frame_path,power,source_session/'static_preflight_check.json',Path(config['reference']),Path(config['geometry'])]})
save(plan['position_confirmation_report'],proof);plan['position_confirmation_report_sha256']=sha(plan['position_confirmation_report'])
f=np.array([-1.,1.])/2**.5;r=np.array([-1.,-1.])/2**.5;start=np.array([315.,90.]);path,_=make_path(plan['expected_candidate_parameters'],127.2792,63.6396)
world=start+path.forward_mm.to_numpy()[:,None]*f+path.right_mm.to_numpy()[:,None]*r
holes=np.array([[x,y] for x in range(0,361,90) for y in range(0,361,90)])
plan.update(nominal_centreline_to_visible_grid_point_min_mm=float(np.linalg.norm(world[:,None,:]-holes[None,:,:],axis=2).min()),
    nominal_board_centre_min_xy_mm=world.min(0).tolist(),nominal_board_centre_max_xy_mm=world.max(0).tolist(),physical_hole_clearance_verified_by_model=False)
save(pp,plan);save(mp,new)
board=marker_registration(frame,ref);assert board['available'];inverse=np.linalg.inv(np.asarray(board['homography_image_to_board_mm']))
def pt(v):return tuple(np.rint(transform_points([v],inverse)[0]).astype(int))
cv2.polylines(frame,[transform_points(world,inverse).astype(np.int32)],False,(60,220,60),3,cv2.LINE_AA)
cv2.circle(frame,pt(start),11,(255,255,0),3,cv2.LINE_AA);cv2.arrowedLine(frame,pt(start),pt(start+45*f),(0,220,255),3,cv2.LINE_AA,tipLength=.25)
cv2.circle(frame,pt(start+27.5*f),7,(0,255,255),-1,cv2.LINE_AA);cv2.circle(frame,pt(start-27.5*f),7,(0,0,255),-1,cv2.LINE_AA)
cv2.drawMarker(frame,pt(plan['planned_stop_board_xy_mm']),(230,0,230),cv2.MARKER_TILTED_CROSS,18,3,cv2.LINE_AA)
image=Image.new('RGB',(1280,840),'white');image.paste(Image.fromarray(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)),(0,0));draw=ImageDraw.Draw(image);font=ImageFont.truetype('C:/Windows/Fonts/YuGothM.ttc',21)
for y,text in [(725,'B 右V90：現在位置を維持（中点は右約315 mm・下90 mm、黄色前は左下向き）'),(757,'緑＝今回位置からの名目経路。写真は今回の静止像（走行指令0回）です。'),(789,'穴・全接地点・全幅・旋回と停止の余裕を確認し、機体の位置と向きは動かさないでください。')]:draw.text((18,y),text,font=font,fill='black')
image.save(BASE/'B_rV90_start315_90_route_guide.png')
print(json.dumps(dict(state='CURRENT_POSITION_REGISTERED_PENDING_CLEARANCE',meta=str(mp),nominal_min_grid_distance_mm=plan['nominal_centreline_to_visible_grid_point_min_mm']),ensure_ascii=False))
