"""今回の静止像を相対評価用に記録する。厳密な目標位置合わせ・機器指令なし。"""
from pathlib import Path
import json
import sys
import cv2
import numpy as np
import pandas as pd

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_r90_20261120_v1'
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'tools/analysis')]
from video_marker_pose import marker_registration,transform_points


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def save(path,value):
    with Path(path).open('x',encoding='utf-8') as stream:
        json.dump(value,stream,ensure_ascii=False,indent=2,allow_nan=False)
        stream.write('\n')


tag='green_corr1p0_repeat03_relative01'
capture=read(BASE/(tag+'_static_capture.json'))
points=read(BASE/(tag+'_detection_points.json'))['frames']
assert capture['state']=='PASSED' and len(points)==10
config=read(BASE/'config.green_front.v1.json')
reference=read(config['reference'])
video=cv2.VideoCapture(capture['video_file'])
video.set(cv2.CAP_PROP_ORIENTATION_AUTO,0)
centres=[]
headings=[]
registrations=[]
image=None
for point in points:
    video.set(cv2.CAP_PROP_POS_MSEC,point['pts']*1000)
    ok,frame=video.read()
    assert ok
    registration=marker_registration(frame,reference)
    assert registration['available']
    matrix=np.asarray(registration['homography_image_to_board_mm'])
    front,rear=transform_points([point['front_image_px'],point['rear_image_px']],matrix)
    centres.append((front+rear)/2)
    headings.append(np.degrees(np.arctan2(*(front-rear)[::-1])))
    registrations.append(dict(pts=point['pts'],used_marker_ids=registration.get('used_marker_ids')))
    image=frame
video.release()
center=np.median(centres,axis=0)
heading=float(np.median(headings))
theta=np.radians(heading)
forward=np.array([np.cos(theta),np.sin(theta)])
right=np.array([-np.sin(theta),np.cos(theta)])
template=BASE/'green_correction1p0_001_video_review_v1/corr1p0_001_relative_trajectory.csv'
trajectory=pd.read_csv(template)
trajectory=trajectory[trajectory.detected.astype(bool)]
route=center+trajectory.forward_mm.to_numpy()[:,None]*forward+trajectory.right_mm.to_numpy()[:,None]*right
assert np.isfinite(route).all()
inverse=np.linalg.inv(matrix)
pixel_path=np.rint(transform_points(route,inverse)).astype(int)
cv2.polylines(image,[pixel_path],False,(255,220,0),2,cv2.LINE_AA)
cv2.putText(image,'Relative-entry review / previous measured path projected here',(20,30),cv2.FONT_HERSHEY_SIMPLEX,.62,(255,255,255),2,cv2.LINE_AA)
cv2.putText(image,'Check all contacts, holes, width and stopping space',(20,690),cv2.FONT_HERSHEY_SIMPLEX,.62,(255,255,255),2,cv2.LINE_AA)
guide=BASE/(tag+'_route_preview.jpg')
assert not guide.exists()
cv2.imwrite(str(guide),image)
review=dict(mouse='B',motion='long_r90',request_id='B_R90_1600_Lstart13p6_corr1p0_green_20261120_003',
    placement_mode='relative_to_measured_motion_entry_direction',
    exact_start_target_required=False,static_marker_yaw_not_used_as_motion_axis=True,
    initial_center_board_mm=center.tolist(),initial_apparent_marker_heading_deg=heading,
    static_sample_count=10,static_full_frame_acceptance_claimed=False,
    registrations=registrations,route_template=str(template),route_preview=str(guide),
    projected_marker_midpoint_bounds_mm=[route.min(axis=0).tolist(),route.max(axis=0).tolist()],
    footprint_hole_clearance_verified_by_image=False,operator_clearance='pending fresh reply',
    detection_mode='profile',detector_changed_from_previous_trial=False,
    learned_detector_work='paused at user request; not applied to this trial',
    motion_quality_policy='Same full-frame registration and raw observation >=95%, fitted trajectory RMSE <=8 mm.',
    robot_commands_sent=0,execution_resend_count=0,
    limitations=['Preview rotates the previous measured marker-midpoint path by current apparent marker yaw; this is an approximate clearance guide, not a footprint guarantee.',
        'This trial measures relative turn shape. Fixed-board alignment and absolute axle accuracy are not claimed.'])
save(BASE/'preflight_green_corr1p0_repeat03_relative.json',review)
state=read(BASE/'resume_state.json')
state.update(state='RELATIVE_START_REVIEWED_ALREADY_ON_READY',power_state='BOTH_ON_USER_CONFIRMED',
    next_action='Keep current rough start. Fresh ON/clearance/hands-clear reply, then restore and verify actual candidate and execute once.',
    evaluation_frame=review['placement_mode'],exact_start_target_required=False,
    current_position='Current right-upper start vicinity; green roughly downward. User ON/placement/hands-clear confirmed.',
    current_placement_confirmed=True,current_hands_clear_confirmed=True,
    relative_setup_review=str(BASE/'preflight_green_corr1p0_repeat03_relative.json'),
    placement_guide=str(guide),current_relative_static_sample_count=10,
    current_relative_static_xy_mm=center.tolist(),current_relative_static_heading_deg=heading,
    vision_improvement_status='PAUSED_BY_USER_TO_RESUME_TURN90; experimental models not applied',
    next_request_sent=False,execution_pending=False,pending_request_id=None,pending_request_sent=False)
(BASE/'resume_state.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps({key:review[key] for key in ('initial_center_board_mm','initial_apparent_marker_heading_deg','projected_marker_midpoint_bounds_mm','route_preview')},ensure_ascii=False))
