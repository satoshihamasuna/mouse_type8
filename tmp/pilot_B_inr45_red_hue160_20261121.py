"""実マーカーの色相境界による欠測を診断し、B専用の解析候補を作る。"""
from pathlib import Path
import copy
import hashlib
import json
import sys
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'tools/analysis')]
from video_marker_pose import iter_video_frames,marker_registration,registration_quad,pose_world_corners,colour_components,select_marker_pair
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr45_20261120_v1'
RUN=BASE/'geometry02_repeat_02/local_execution/run_B_INR45_1600_geometry02_repeat02_green_20261120_008'
OUT=RUN/'red_hue160_pilot_v1';OUT.mkdir(exist_ok=False)
read=lambda p:json.loads(p.read_text(encoding='utf-8'))
trial=read(RUN/'trial.json');old=read(RUN/'body_geometry_used.json');reference=read(RUN/'marker_reference_used.json')
candidate=copy.deepcopy(old)
candidate['marker_detection']['red']['hue_ranges']=[[0,10],[160,179]]
candidate['marker_detection_source']='解析候補v4：008の後赤マーカーにH160〜169・S40以上・V65以上の実画素を確認。赤色相の下限170→160のみ拡張し、S/V/面積/開演算、緑、前後間隔、実ArUco参照、観測率基準は保持。過去独立2動画で検証するまで候補。'
candidate['analysis_profile_adoption_state']='CANDIDATE_PENDING_HELDOUT_VALIDATION'
profile=BASE/'body_geometry_B_green_red_h160_candidate_v4.json'
with profile.open('x',encoding='utf-8') as f:json.dump(candidate,f,ensure_ascii=False,indent=2)
rows=[];previous={'old':None,'candidate':None}
for index,frame,stamp in iter_video_frames(Path(trial['video_file']),True,(9.34,9.47),1):
    registration=marker_registration(frame,reference);quad=registration_quad(registration)
    row=dict(frame=int(index),pts_s=float(stamp),aruco_ids=registration['detected_marker_ids'])
    assert quad is not None and len(set(registration['detected_marker_ids']))>=2
    matrix=cv2.getPerspectiveTransform(quad,pose_world_corners(reference['board_side_mm']))
    for name,geometry in [('old',old),('candidate',candidate)]:
        pair=select_marker_pair(colour_components(frame,'green',geometry['marker_detection']),
            colour_components(frame,'red',geometry['marker_detection']),matrix,previous[name],
            nominal_marker_span_mm=geometry['nominal_marker_span_mm'],marker_span_limits_mm=geometry['marker_span_limits_mm'])
        row[name+'_observed']=pair is not None
        if pair is not None:
            previous[name]=(pair[0]+pair[1])/2
            row[name+'_span_mm']=float(np.linalg.norm(pair[0]-pair[1]))
            row[name+'_front_rear_mm']=[pair[0].tolist(),pair[1].tolist()]
            if name=='candidate' and index in (2245,2250,2255,2270):
                inverse=np.linalg.inv(matrix)
                points=cv2.perspectiveTransform(np.array(pair,dtype=np.float32).reshape(-1,1,2),inverse).reshape(-1,2)
                annotated=frame.copy()
                for point,colour in zip(points,[(0,255,0),(0,0,255)]):cv2.circle(annotated,tuple(np.rint(point).astype(int)),7,colour,1)
                cv2.imwrite(str(OUT/f'frame_{index}_actual_colour_pair.jpg'),annotated)
    rows.append(row)
result=dict(source_trial=str(RUN/'trial.json'),candidate_geometry=str(profile),
    frames=len(rows),old_observed=sum(r['old_observed'] for r in rows),candidate_observed=sum(r['candidate_observed'] for r in rows),
    observations=rows,actual_colour_pairs_only=True,missing_positions_interpolated=False,
    global_capture_failure_preserved=True,acceptance_thresholds_changed=False,
    heldout_validation_required=True,source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (
        RUN/'trial.json',RUN/'body_geometry_used.json',RUN/'marker_reference_used.json',profile,Path(__file__))})
with (OUT/'pilot.json').open('x',encoding='utf-8') as f:json.dump(result,f,ensure_ascii=False,indent=2,allow_nan=False)
print(json.dumps({k:result[k] for k in ('frames','old_observed','candidate_observed','candidate_geometry')},ensure_ascii=False),flush=True)
