"""元動画の実色成分だけで欠測原因を比較する。位置補間を観測として使わない。"""
from pathlib import Path
import copy
import json
import sys
import cv2
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/analysis'))
from video_marker_pose import colour_components, marker_registration, registration_quad, pose_world_corners, select_marker_pair, transform_points
BASE = ROOT / 'tools/logs/pixel_camera/turn1800_A_outr135_20261120_v1'
RUN = BASE / 'baseline01/local_execution/run_A_OUTR135_1800_baseline_20261120_001'
OUT = BASE / 'resume_20261120_v1/missing_frames_review_v1/variant_check_v2'
OUT.mkdir(exist_ok=False)
read = lambda p: json.loads(p.read_text(encoding='utf-8-sig'))
trial, geometry, reference = [read(RUN / n) for n in ('trial.json', 'body_geometry_used.json', 'marker_reference_used.json')]
pose = pd.read_csv(RUN / 'full/apparent_floor_marker_pose.csv').set_index('frame')
missing = pose.index[~pose.marker_detected_raw.to_numpy(bool)].tolist()
indices = sorted(set(missing + [k+d for k in missing for d in (-2,-1,1,2)] + [1800,1900,2200,2350,2400,2600]))
variants = {}
for label in ('original','red_open1','yellow_open1','both_open1','red_S25','yellow_S20','both_S25_20'):
    detection = copy.deepcopy(geometry['marker_detection'])
    if label in ('red_open1','both_open1'): detection['red']['opening_kernel_size'] = 0
    if label in ('yellow_open1','both_open1'): detection['yellow']['opening_kernel_size'] = 0
    if label in ('red_S25','both_S25_20'): detection['red']['minimum_saturation'] = 25
    if label in ('yellow_S20','both_S25_20'): detection['yellow']['minimum_saturation'] = 20
    variants[label] = detection
cap = cv2.VideoCapture(trial['video_file']); cap.set(cv2.CAP_PROP_ORIENTATION_AUTO,0); cv2.setNumThreads(4)
rows = []
for index in indices:
    cap.set(cv2.CAP_PROP_POS_FRAMES,index); ok, frame = cap.read()
    assert ok
    registration = marker_registration(frame,reference); quad = registration_quad(registration)
    if quad is None:
        rows.append(dict(frame=index,variant='all',registered=False));continue
    matrix = cv2.getPerspectiveTransform(quad,pose_world_corners(reference.get('board_side_mm',360)))
    for label,detection in variants.items():
        red = colour_components(frame,'red',detection); yellow = colour_components(frame,'yellow',detection)
        pair = select_marker_pair(red,yellow,matrix,None,nominal_marker_span_mm=50.,marker_span_limits_mm=(40.,65.))
        row = dict(frame=index,originally_missing=index in missing,variant=label,registered=True,
            red_components=len(red),yellow_components=len(yellow),pair=pair is not None)
        if pair is not None:
            px = transform_points(pair,np.linalg.inv(matrix))
            row.update(front_x_px=float(px[0,0]),front_y_px=float(px[0,1]),rear_x_px=float(px[1,0]),rear_y_px=float(px[1,1]))
            if index in pose.index:
                prior = pose.loc[index];front = prior[['front_image_x_px','front_image_y_px']].to_numpy(float)
                rear = 2*prior[['midpoint_image_x_px','midpoint_image_y_px']].to_numpy(float)-front
                row['distance_from_prior_track_front_px'] = float(np.linalg.norm(px[0]-front))
                row['distance_from_prior_track_rear_px'] = float(np.linalg.norm(px[1]-rear))
        rows.append(row)
cap.release()
frame = pd.DataFrame(rows);frame.to_csv(OUT/'component_pairs.csv',index=False)
summary=[]
for label in variants:
    part=frame[frame.variant==label];bad=part[part.originally_missing.fillna(False)]
    selected=part[part.pair.fillna(False)]
    summary.append(dict(variant=label,registered_frames=len(part),pairs=int(part.pair.sum()),
        recovered_missing=int(bad.pair.sum()),max_front_distance_px=float(selected.distance_from_prior_track_front_px.max()),
        max_rear_distance_px=float(selected.distance_from_prior_track_rear_px.max())))
with (OUT/'variants.json').open('x',encoding='utf-8') as file:json.dump(dict(summary=summary,detection=variants,observations_are_actual_colour_components=True,track_prediction_used_only_for_diagnostic_distance=True,hardware_operations=0),file,ensure_ascii=False,indent=2)
print(json.dumps(summary,indent=2))
