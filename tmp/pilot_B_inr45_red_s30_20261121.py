"""012の実赤画素に基づくS30解析候補。品質の採用基準は変えない。"""
from pathlib import Path
import copy
import hashlib
import json
import sys
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr45_20261120_v1'
RUN=BASE/'geometry05_repeat01_correction_m1p2/local_execution/run_B_INR45_1600_m1p2_repeat01_green_20261121_012'
OUT=RUN/'red_s30_pilot_v1';OUT.mkdir(exist_ok=False)
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'tools/analysis')]
from video_marker_pose import iter_video_frames,marker_registration,registration_quad,pose_world_corners,colour_components,select_marker_pair
read=lambda p:json.loads(Path(p).read_text(encoding='utf-8-sig'))
trial=read(RUN/'trial.json');old=read(RUN/'body_geometry_used.json');reference=read(RUN/'marker_reference_used.json')
candidate=copy.deepcopy(old);candidate['marker_detection']['red']['minimum_saturation']=30
candidate['analysis_profile_adoption_state']='CANDIDATE_PENDING_HELDOUT_VALIDATION'
candidate['marker_detection_source']='012の実後赤画素H160〜179/S30〜39/V65以上が旧S40から外れる。赤Sだけ40→30の候補。色相/明度/面積/開演算/前後間隔/緑/ArUco/95%基準/欠測方針は維持。過去動画での検証は未完了。'
profile=BASE/'body_geometry_B_green_red_h160_s30_candidate_v5.json'
with profile.open('x',encoding='utf-8') as f:json.dump(candidate,f,ensure_ascii=False,indent=2)
rows=[];previous={'old':None,'candidate':None}
for index,frame,stamp in iter_video_frames(Path(trial['video_file']),True,(9.47,9.55),1):
 registration=marker_registration(frame,reference);quad=registration_quad(registration)
 assert quad is not None and len(set(registration['detected_marker_ids']))>=2
 matrix=cv2.getPerspectiveTransform(quad,pose_world_corners(reference['board_side_mm']))
 row=dict(frame=int(index),pts_s=float(stamp),aruco_ids=registration['detected_marker_ids'])
 for name,geometry in [('old',old),('candidate',candidate)]:
  pair=select_marker_pair(colour_components(frame,'green',geometry['marker_detection']),colour_components(frame,'red',geometry['marker_detection']),
   matrix,previous[name],nominal_marker_span_mm=geometry['nominal_marker_span_mm'],marker_span_limits_mm=geometry['marker_span_limits_mm'])
  row[name+'_observed']=pair is not None
  if pair is not None:
   previous[name]=(pair[0]+pair[1])/2
   row[name+'_front_rear_mm']=[p.tolist() for p in pair];row[name+'_span_mm']=float(np.linalg.norm(pair[0]-pair[1]))
   if name=='candidate' and index in (2274,2278,2281,2285):
    points=cv2.perspectiveTransform(np.asarray(pair,dtype=np.float32).reshape(-1,1,2),np.linalg.inv(matrix)).reshape(-1,2)
    center=np.mean(points,axis=0);x0,y0=np.maximum(0,np.rint(center-[80,85]).astype(int));x1,y1=np.minimum([frame.shape[1],frame.shape[0]],[x0+160,y0+170])
    crop=frame[y0:y1,x0:x1].copy()
    for p,c in zip(points,[(0,255,0),(0,255,255)]):cv2.circle(crop,tuple(np.rint(p-[x0,y0]).astype(int)),5,c,1)
    cv2.imwrite(str(OUT/f'frame_{index}_actual_pair.jpg'),cv2.resize(crop,(480,510),interpolation=cv2.INTER_NEAREST))
 rows.append(row)
result=dict(source_trial=str(RUN/'trial.json'),candidate_geometry=str(profile),frames=len(rows),
 old_observed=sum(r['old_observed'] for r in rows),candidate_observed=sum(r['candidate_observed'] for r in rows),observations=rows,
 missing_positions_filled=False,acceptance_thresholds_changed=False,heldout_validation_required=True,old_rejected_result_preserved=True,
 input_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (RUN/'trial.json',RUN/'body_geometry_used.json',profile,Path(__file__))})
with (OUT/'pilot.json').open('x',encoding='utf-8') as f:json.dump(result,f,ensure_ascii=False,indent=2,allow_nan=False)
print(json.dumps({k:result[k] for k in ('frames','old_observed','candidate_observed')},ensure_ascii=False))
