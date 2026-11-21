"""B1600の保存済み静止動画から今回の盤面基準と配置図を作る。機器操作なし。"""
from pathlib import Path
import copy
import json
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT/'tools/logs/pixel_camera/turn1600_B_r90_20261120_v1'
sys.path[:0] = [str(ROOT/'tools'),str(ROOT/'tools/analysis')]
from pixel_camera_reference import create_marker_reference, detect_registered_vertices, marker_corners_by_id
from video_marker_pose import marker_registration, transform_points
from pixel_camera_static_pose import measure


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def save(path,value):
    with Path(path).open('x',encoding='utf-8') as stream:
        json.dump(value,stream,ensure_ascii=False,indent=2,allow_nan=False)
        stream.write('\n')


capture = read(BASE/'initial_static_capture.json')
frame = cv2.imread(str(BASE/'initial_static_frame.jpg'))
old = read(BASE/'marker_reference_current.json')
search = copy.deepcopy(old)
search.pop('marker_search_rois_px',None)
markers = detect_registered_vertices(frame,search)
corners = marker_corners_by_id(markers)
assert set(corners)=={0,1,2,3}
center = np.mean([v.mean(axis=0) for v in corners.values()],axis=0)
inner = {k:v[np.argmin(np.linalg.norm(v-center,axis=1))].tolist() for k,v in corners.items()}
ids = dict(TL=1,TR=3,BR=2,BL=0)
inspection = dict(corners=markers,board_reference=dict(available=True,method='current_actual_decoded_four_ArUcos',
    points_px={name:inner[k] for name,k in ids.items()}),source=dict(video=capture['video_file'],time_video_s=2.,
    rotation_note='盤面は前回基準から90度回転している。今回は画面左上原点・右X正・下Y正で新規登録。',
    actual_decoded_ids=sorted(corners),camera_height_mm=1500.,zoom_ratio=1.8))
ref = create_marker_reference(inspection,board_corner_ids=ids)
ref['corner_refinement']='subpix_original_win3'
ref['grid_refinement']=copy.deepcopy(old['grid_refinement'])
registered=marker_registration(frame,ref)
save(BASE/'current_registration_frame.json',registered)
assert registered['available'], registered
save(BASE/'marker_reference_B_current_rotated.json',ref)
body=read(BASE/'body_geometry_B_current.json')
body['previous_reference_id']=body['reference_id']
body['reference_id']=ref['reference_id']
body['reference_binding_source']='今回静止動画の実復号4ArUco＋同一固定格子点・独立検査から新規登録。'
save(BASE/'body_geometry_B_current_rotated.json',body)
config=read(BASE/'config.json')
config.update(reference=str(BASE/'marker_reference_B_current_rotated.json'),geometry=str(BASE/'body_geometry_B_current_rotated.json'))
save(BASE/'config.current.json',config)
placement=dict(x_mm=315.,y_mm=45.,heading_deg=90.,position_tolerance_mm=5.,heading_tolerance_deg=2.)
check=measure(Path(capture['video_file']),ref,body,placement)
save(BASE/'initial_static_check_current_reference.json',check)
print(json.dumps({k:check.get(k) for k in ('passed','frames','observed','center_board_mm','heading_deg','heading_budget_deg','failure_reasons')},ensure_ascii=False),flush=True)
inverse=np.linalg.inv(np.asarray(registered['homography_image_to_board_mm']))
def px(points):
    return np.rint(transform_points(points,inverse)).astype(int)
def line(points,color,width=3):
    cv2.polylines(frame,[px(points)],False,color,width,cv2.LINE_AA)
# この線は設置案内。ターン形状・穴との非干渉を保証する軌道ではない。
start,front,rear=px([[315,45],[315,70],[315,20]])
cv2.circle(frame,tuple(start),12,(255,220,0),3,cv2.LINE_AA)
cv2.circle(frame,tuple(front),7,(0,255,255),2,cv2.LINE_AA)
cv2.circle(frame,tuple(rear),7,(0,0,255),2,cv2.LINE_AA)
cv2.arrowedLine(frame,tuple(start),tuple(px([[315,103]])[0]),(0,255,255),3,cv2.LINE_AA,tipLength=.2)
line([[315,45],[315,225]],(255,220,0),2)
cv2.putText(frame,'B RIGHT90 / 1600 / SUCTION650',(30,35),cv2.FONT_HERSHEY_SIMPLEX,.8,(255,255,255),2,cv2.LINE_AA)
cv2.putText(frame,'START midpoint (315,45) mm',(820,195),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1,cv2.LINE_AA)
cv2.putText(frame,'YELLOW front DOWN',(820,220),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,255),1,cv2.LINE_AA)
cv2.putText(frame,'RED rear UP',(820,245),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,0,255),1,cv2.LINE_AA)
cv2.putText(frame,'Placement guide; check tyres / holes / stopping space',(25,688),cv2.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),2,cv2.LINE_AA)
cv2.imwrite(str(BASE/'start_placement_guide.jpg'),frame)
