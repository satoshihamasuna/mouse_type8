"""保存済みの独立動画で従来・外観学習・局所精密化・CNNを同じ実登録条件で比較する。"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import pandas as pd

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from body_appearance_model import LearnedBodyKeypointModel,detect_pair_with_body_model
from pixel_camera_detection import file_sha256
from video_marker_pose import (marker_registration,registration_quad,pose_world_corners,iter_video_frames,
    colour_components,select_marker_pair,transform_points,board_to_pose_transform)


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def evaluate(manifest,appearance_file,cnn_file,output):
    output = Path(output)
    output.mkdir(parents=True,exist_ok=False)
    started = time.perf_counter()
    appearance = LearnedBodyKeypointModel.load(appearance_file)
    cnn = None
    if cnn_file:
        from body_keypoint_cnn import BodyKeypointCNN
        cnn = BodyKeypointCNN.load(cnn_file)
    loaded_seconds = time.perf_counter()-started
    training = set()
    for source in appearance.metadata.get('sources',[]):
        training.add(file_sha256(source['source_video']))
    if cnn:
        for source in cnn.metadata.get('sources',[]):
            training.add(file_sha256(source['source_video']))
    variants = manifest.get('variants', ['legacy','appearance','appearance_refined']+(['cnn'] if cnn else []))
    if not variants or set(variants)-{'legacy','appearance','appearance_refined','cnn','learned_assist'}:
        raise ValueError('unknown detector evaluation variant')
    results = []
    all_rows = []
    montage = []
    for case in manifest['cases']:
        geometry = read(case['geometry'])
        reference = read(case['reference'])
        if file_sha256(case['video']) in training:
            raise ValueError('学習動画を独立評価に使いません: '+case['label'])
        front,rear = geometry['front_colour'],geometry.get('rear_colour','yellow')
        appearance.validate_identity(geometry['mouse'],front,rear)
        if cnn and any(cnn.metadata.get(key)!=value for key,value in
            [('mouse',geometry['mouse']),('front_colour',front),('rear_colour',rear)]):
            raise ValueError('CNNの機体・色が評価動画と異なります')
        side = reference.get('board_side_mm',360)
        world = pose_world_corners(side)
        board_inverse = np.linalg.inv(board_to_pose_transform(reference))
        states = {name:dict(center=None,heading=None,velocity=None) for name in variants}
        case_rows = []
        stride = case.get('stride',1)
        window = case['window']
        warmup_start = case.get('warmup_start',window[0])
        if not 0 <= warmup_start <= window[0]:
            raise ValueError('warmup_start must precede the evaluation window')
        for frame_index,frame,pts in iter_video_frames(Path(case['video']),True,(warmup_start,window[1]),stride):
            registration = marker_registration(frame,reference)
            quad = registration_quad(registration)
            H = cv2.getPerspectiveTransform(quad,world) if quad is not None else None
            marked = frame.copy() if frame_index in case.get('review_frames',[]) else None
            marked_points = []
            for variant in variants:
                clock = time.perf_counter()
                row = dict(case=case['label'],kind=case['kind'],variant=variant,frame=frame_index,
                    time_video_s=pts,calibration_valid=H is not None,observed=False,estimated=False,
                    method='not_detected',body_ncc=None,cnn_applied=False,
                    decoded_aruco_ids=json.dumps(registration.get('detected_marker_ids',[])),
                    used_aruco_ids=json.dumps(registration.get('used_marker_ids',[])))
                pair,info = None,{}
                state = states[variant]
                if H is not None:
                    if variant in ('legacy', 'learned_assist'):
                        pair = select_marker_pair(colour_components(frame,front,geometry.get('marker_detection')),
                            colour_components(frame,rear,geometry.get('marker_detection')),H,state['center'],
                            board_side_mm=side,nominal_marker_span_mm=geometry.get('nominal_marker_span_mm'),
                            marker_span_limits_mm=geometry.get('marker_span_limits_mm'))
                        info = dict(method='colour_pair',front_colour_observed=pair is not None,rear_colour_observed=pair is not None)
                    if variant != 'legacy' and pair is None:
                        found = detect_pair_with_body_model(frame,front,rear,geometry.get('marker_detection'),H,np.linalg.inv(H),
                            appearance,state['center'],state['heading'],state['velocity'],board_side_mm=side,
                            nominal_span_mm=geometry.get('nominal_marker_span_mm'),span_limits_mm=geometry.get('marker_span_limits_mm'),
                            require_colour_pair=variant!='learned_assist',refine_standard=variant=='appearance_refined',
                            center_margin_mm=0. if variant=='learned_assist' else 25.,
                            keypoint_model=cnn if variant in ('cnn','learned_assist') else None)
                        if found is not None:
                            pair,info = found[:2],found[2]
                row['processing_ms'] = (time.perf_counter()-clock)*1000
                if pair is not None:
                    points = np.asarray(pair)
                    center = points.mean(axis=0)
                    heading = math.atan2(*(points[0]-points[1])[::-1])
                    board_points = transform_points(points,board_inverse)
                    board_center = board_points.mean(axis=0)
                    board_heading = math.atan2(*(board_points[0]-board_points[1])[::-1])
                    observed = bool(info['front_colour_observed'] and info['rear_colour_observed'])
                    pixels = transform_points(points,np.linalg.inv(H))
                    row.update(observed=observed,estimated=not observed,method=info['method'],
                        body_ncc=info.get('body_ncc'),cnn_applied=info.get('keypoint_cnn_applied',False),
                        center_x_mm=float(board_center[0]),center_y_mm=float(board_center[1]),
                        heading_deg=float(np.degrees(board_heading)),front_x_px=float(pixels[0,0]),front_y_px=float(pixels[0,1]),
                        rear_x_px=float(pixels[1,0]),rear_y_px=float(pixels[1,1]))
                    if state['center'] is not None:
                        velocity = center-state['center']
                        state['velocity'] = .5*velocity if state['velocity'] is None else .5*(velocity+state['velocity'])
                    state.update(center=center,heading=heading)
                    if marked is not None:
                        colour = {'legacy':(0,0,255),'appearance':(255,255,0),'appearance_refined':(0,255,255),'cnn':(0,255,0),'learned_assist':(0,165,255)}[variant]
                        cv2.line(marked,tuple(np.rint(pixels[1]).astype(int)),tuple(np.rint(pixels[0]).astype(int)),colour,1)
                        for point in pixels:
                            cv2.circle(marked,tuple(np.rint(point).astype(int)),3,colour,1)
                        marked_points.extend(pixels.tolist())
                else:
                    state['velocity'] = None
                if pts >= window[0]:
                    case_rows.append(row)
            if marked is not None and marked_points:
                center = np.median(marked_points,axis=0)
                x = int(np.clip(center[0]-90,0,frame.shape[1]-180))
                y = int(np.clip(center[1]-70,0,frame.shape[0]-140))
                tile = cv2.resize(marked[y:y+140,x:x+180],(360,280))
                cv2.rectangle(tile,(0,0),(360,26),(0,0,0),-1)
                cv2.putText(tile,f'{case["label"]} frame{frame_index}',(5,18),cv2.FONT_HERSHEY_SIMPLEX,.43,(255,255,255),1)
                montage.append(tile)
            if case_rows and len(case_rows)%(len(variants)*30)==0:
                print(f'{case["label"]}: {len(case_rows)//len(variants)} frames',flush=True)
        table = pd.DataFrame(case_rows)
        table.to_csv(output/(case['label']+'.csv'),index=False)
        all_rows.extend(case_rows)
        for variant in variants:
            selected = table[table.variant==variant]
            observed = selected[selected.observed]
            covered = selected[selected.observed | selected.estimated]
            calibration_count = int(selected.calibration_valid.sum())
            xy = covered[['center_x_mm','center_y_mm']].to_numpy() if len(covered) else np.empty((0,2))
            heading = np.unwrap(np.radians(covered.heading_deg.to_numpy())) if len(covered) else np.empty(0)
            stamps = covered.time_video_s.to_numpy()
            dt = np.diff(stamps)
            steps = np.linalg.norm(np.diff(xy,axis=0),axis=1)
            rotations = np.abs(np.degrees(np.diff(heading)))
            invalid_steps = steps>(2500*dt+5)
            invalid_turns = rotations>np.degrees(35*dt)+6
            result = dict(case=case['label'],kind=case['kind'],variant=variant,frames=len(selected),
                calibrated_frames=calibration_count,observed_frames=len(observed),
                observed_given_calibration=len(observed)/calibration_count if calibration_count else 0,
                estimated_frames=int(selected.estimated.sum()),
                coverage_given_calibration=len(covered)/calibration_count if calibration_count else 0,
                maximum_step_mm=float(steps.max()) if len(steps) else None,
                maximum_heading_step_deg=float(rotations.max()) if len(rotations) else None,
                implausible_position_steps=int(invalid_steps.sum()),implausible_heading_steps=int(invalid_turns.sum()),
                processing_median_ms=float(selected.processing_ms.median()),processing_p95_ms=float(selected.processing_ms.quantile(.95)),
                cnn_applied_frames=int(selected.cnn_applied.sum()),
                mean_xy_mm=xy.mean(axis=0).tolist() if len(xy) else None)
            if case['kind']=='static' and len(covered):
                result.update(heading_jitter_rms_deg=float(np.sqrt(np.mean(np.degrees(heading-np.median(heading))**2))),
                    midpoint_jitter_rms_mm=float(np.sqrt(np.sum(np.var(xy,axis=0)))))
            results.append(result)
    pd.DataFrame(all_rows).to_csv(output/'all_frames.csv',index=False)
    report = dict(schema_version=1,results=results,model_load_seconds=loaded_seconds,
        appearance_model_file=str(Path(appearance_file).resolve()),appearance_sha256=file_sha256(appearance_file),
        cnn_model_file=str(Path(cnn_file).resolve()) if cnn_file else None,
        cnn_sha256=file_sha256(cnn_file) if cnn_file else None,
        training_video_sha256=sorted(training),manifest=manifest,independent_video_split_verified=True,
        absolute_keypoint_accuracy_verified=False,
        scope='同じ実復号ArUco登録での未平滑化検出比較。静止散らばりと飛びを評価。手動の独立サブピクセル正解はなく、絶対精度の保証ではない。')
    (output/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    if montage:
        while len(montage)%3:
            montage.append(np.zeros_like(montage[0]))
        cv2.imwrite(str(output/'comparison_montage.jpg'),np.vstack([np.hstack(montage[i:i+3]) for i in range(0,len(montage),3)]))
    print(json.dumps(results,ensure_ascii=False),flush=True)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path,required=True)
    parser.add_argument('--appearance-model',type=Path,required=True)
    parser.add_argument('--cnn-model',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    evaluate(read(args.manifest),args.appearance_model,args.cnn_model,args.output)


if __name__=='__main__':
    main()
