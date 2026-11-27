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


ROOT = Path(__file__).resolve().parents[2]
BUILTIN_VARIANTS = {'legacy','appearance','appearance_refined','cnn','learned_assist'}
APPEARANCE_REQUIRED_VARIANTS = {'appearance','appearance_refined','cnn','learned_assist'}
MONTAGE_COLOURS = {
    'legacy':(0,0,255),
    'appearance':(255,255,0),
    'appearance_refined':(0,255,255),
    'cnn':(0,255,0),
    'learned_assist':(0,165,255),
}
EXTRA_MONTAGE_PALETTE = ((255,0,255),(255,128,0),(180,255,120),(255,120,180),(120,180,255))


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def resolve_portable_path(raw_path,base_dir=None):
    text = str(raw_path)
    normalized = text.replace('\\','/')
    candidates = [Path(text)]
    if normalized != text:
        candidates.append(Path(normalized))
    rel = Path(normalized)
    if not rel.is_absolute():
        if base_dir is not None:
            candidates.append(Path(base_dir)/rel)
        candidates.append(ROOT/rel)
    for candidate in candidates:
        if candidate.exists():
            return candidate
    raise FileNotFoundError('ファイルが見つかりません: '+text)


def _is_sha256_hex(value):
    return isinstance(value,str) and len(value)==64 and all(ch in '0123456789abcdefABCDEF' for ch in value)


def collect_training_hashes(sources,*,base_dir=None):
    hashes = set()
    for source in sources or []:
        recorded = source.get('video_sha256')
        if _is_sha256_hex(recorded):
            hashes.add(recorded.lower())
            continue
        raw_video = source.get('source_video')
        if not raw_video:
            raise ValueError('学習元メタデータにvideo_sha256またはsource_videoがありません')
        hashes.add(file_sha256(resolve_portable_path(raw_video,base_dir)).lower())
    return hashes


def max_consecutive_missing(selected):
    calibrated = selected[selected.calibration_valid]
    if not len(calibrated):
        return 0
    missing = ~(calibrated.observed | calibrated.estimated)
    longest = 0
    run = 0
    for is_missing in missing.to_numpy(dtype=bool):
        if is_missing:
            run += 1
            if run > longest:
                longest = run
        else:
            run = 0
    return int(longest)


def evaluate(manifest,appearance_file,cnn_file,output,*,base_dir=None,extra_detectors=None,device='cpu'):
    output = Path(output)
    output.mkdir(parents=True,exist_ok=False)
    extra_detectors = dict(extra_detectors or {})
    started = time.perf_counter()
    variants = manifest.get('variants')
    if variants is None:
        variants = ['legacy']
        if appearance_file is not None:
            variants += ['appearance','appearance_refined']
        if cnn_file is not None:
            variants.append('cnn')
        variants += [name for name in extra_detectors if name not in variants]
    allowed_variants = BUILTIN_VARIANTS | set(extra_detectors)
    if not variants or set(variants)-allowed_variants:
        raise ValueError('unknown detector evaluation variant')
    need_appearance = bool(set(variants) & APPEARANCE_REQUIRED_VARIANTS)
    if need_appearance and appearance_file is None:
        raise ValueError('外観・CNN系variantの評価にはappearance_fileが必要です')
    appearance = LearnedBodyKeypointModel.load(resolve_portable_path(appearance_file,base_dir)) if appearance_file else None
    cnn = None
    if cnn_file:
        from body_keypoint_cnn import BodyKeypointCNN
        cnn = BodyKeypointCNN.load(resolve_portable_path(cnn_file,base_dir),device=device)
    loaded_seconds = time.perf_counter()-started
    training = set()
    if appearance:
        training |= collect_training_hashes(appearance.metadata.get('sources',[]),base_dir=base_dir)
    if cnn:
        training |= collect_training_hashes(cnn.metadata.get('sources',[]),base_dir=base_dir)
    for name,detector in extra_detectors.items():
        if name not in variants:
            continue
        meta = getattr(detector,'metadata',None)
        if isinstance(meta,dict):
            training |= collect_training_hashes(meta.get('sources',[]),base_dir=base_dir)
    results = []
    all_rows = []
    montage = []
    extra_colour_map = {name:EXTRA_MONTAGE_PALETTE[idx%len(EXTRA_MONTAGE_PALETTE)]
        for idx,name in enumerate(name for name in variants if name not in MONTAGE_COLOURS)}
    for case in manifest['cases']:
        video_path = resolve_portable_path(case['video'],base_dir)
        geometry_path = resolve_portable_path(case['geometry'],base_dir)
        reference_path = resolve_portable_path(case['reference'],base_dir)
        geometry = read(geometry_path)
        reference = read(reference_path)
        if file_sha256(video_path).lower() in training:
            raise ValueError('学習動画を独立評価に使いません: '+case['label'])
        front,rear = geometry['front_colour'],geometry.get('rear_colour','yellow')
        if appearance:
            appearance.validate_identity(geometry['mouse'],front,rear)
        if cnn and any(cnn.metadata.get(key)!=value for key,value in
            [('mouse',geometry['mouse']),('front_colour',front),('rear_colour',rear)]):
            raise ValueError('CNNの機体・色が評価動画と異なります')
        for name,detector in extra_detectors.items():
            if name not in variants:
                continue
            meta = getattr(detector,'metadata',None)
            if isinstance(meta,dict) and any(meta.get(key) is not None and meta.get(key)!=value for key,value in
                [('mouse',geometry['mouse']),('front_colour',front),('rear_colour',rear)]):
                raise ValueError(f'追加検出器 {name} の機体・色が評価動画と異なります')
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
        case_context = dict(case=case,geometry=geometry,reference=reference,board_side_mm=side,
            front_colour=front,rear_colour=rear,appearance=appearance,cnn=cnn)
        for frame_index,frame,pts in iter_video_frames(video_path,True,(warmup_start,window[1]),stride):
            registration = marker_registration(frame,reference)
            quad = registration_quad(registration)
            H = cv2.getPerspectiveTransform(quad,world) if quad is not None else None
            H_inv = np.linalg.inv(H) if H is not None else None
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
                    if variant in extra_detectors:
                        found = extra_detectors[variant](frame,H,H_inv,state,case_context)
                        if found is not None:
                            pair = (found[0],found[1])
                            raw_info = dict(found[2]) if len(found)>2 and isinstance(found[2],dict) else {}
                            info = dict(method=raw_info.get('method',variant),
                                front_colour_observed=bool(raw_info.get('front_colour_observed',False)),
                                rear_colour_observed=bool(raw_info.get('rear_colour_observed',False)),
                                ** {k:v for k,v in raw_info.items() if k not in ('method','front_colour_observed','rear_colour_observed')})
                    else:
                        if variant in ('legacy', 'learned_assist'):
                            pair = select_marker_pair(colour_components(frame,front,geometry.get('marker_detection')),
                                colour_components(frame,rear,geometry.get('marker_detection')),H,state['center'],
                                board_side_mm=side,nominal_marker_span_mm=geometry.get('nominal_marker_span_mm'),
                                marker_span_limits_mm=geometry.get('marker_span_limits_mm'))
                            info = dict(method='colour_pair',front_colour_observed=pair is not None,rear_colour_observed=pair is not None)
                        if variant != 'legacy' and pair is None:
                            found = detect_pair_with_body_model(frame,front,rear,geometry.get('marker_detection'),H,H_inv,
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
                    pixels = transform_points(points,H_inv)
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
                        colour = MONTAGE_COLOURS.get(variant,extra_colour_map.get(variant,(255,0,255)))
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
            method_counts = {str(k):int(v) for k,v in selected.method.value_counts().items()} if len(selected) else {}
            result = dict(case=case['label'],kind=case['kind'],variant=variant,frames=len(selected),
                calibrated_frames=calibration_count,
                aruco_calibration_rate=calibration_count/len(selected) if len(selected) else 0.,
                observed_frames=len(observed),
                observed_given_calibration=len(observed)/calibration_count if calibration_count else 0.,
                estimated_frames=int(selected.estimated.sum()),
                coverage_given_calibration=len(covered)/calibration_count if calibration_count else 0.,
                missing_given_calibration=int(calibration_count-len(covered)),
                max_consecutive_missing_frames=max_consecutive_missing(selected),
                method_counts=method_counts,
                maximum_step_mm=float(steps.max()) if len(steps) else None,
                maximum_heading_step_deg=float(rotations.max()) if len(rotations) else None,
                implausible_position_steps=int(invalid_steps.sum()),implausible_heading_steps=int(invalid_turns.sum()),
                processing_median_ms=float(selected.processing_ms.median()) if len(selected) else None,
                processing_p95_ms=float(selected.processing_ms.quantile(.95)) if len(selected) else None,
                cnn_applied_frames=int(selected.cnn_applied.sum()) if len(selected) else 0,
                mean_xy_mm=xy.mean(axis=0).tolist() if len(xy) else None)
            if case['kind']=='static' and len(covered):
                result.update(heading_jitter_rms_deg=float(np.sqrt(np.mean(np.degrees(heading-np.median(heading))**2))),
                    midpoint_jitter_rms_mm=float(np.sqrt(np.sum(np.var(xy,axis=0)))))
            results.append(result)
    pd.DataFrame(all_rows).to_csv(output/'all_frames.csv',index=False)
    resolved_appearance = resolve_portable_path(appearance_file,base_dir) if appearance_file else None
    resolved_cnn = resolve_portable_path(cnn_file,base_dir) if cnn_file else None
    report = dict(schema_version=1,results=results,model_load_seconds=loaded_seconds,
        appearance_model_file=str(resolved_appearance.resolve()) if resolved_appearance else None,
        appearance_sha256=file_sha256(resolved_appearance) if resolved_appearance else None,
        cnn_model_file=str(resolved_cnn.resolve()) if resolved_cnn else None,
        cnn_sha256=file_sha256(resolved_cnn) if resolved_cnn else None,
        extra_variants=sorted(name for name in extra_detectors if name in variants),
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
    parser.add_argument('--appearance-model',type=Path,default=None)
    parser.add_argument('--cnn-model',type=Path,default=None)
    parser.add_argument('--base-dir',type=Path,default=None,help='manifest内相対パスの基準ディレクトリ（省略時はmanifest親ディレクトリ）')
    parser.add_argument('--device',default='cpu',help='CNN推論デバイス: cpu / cuda / auto')
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    base_dir = args.base_dir if args.base_dir is not None else args.manifest.resolve().parent
    evaluate(read(args.manifest),args.appearance_model,args.cnn_model,args.output,base_dir=base_dir,device=args.device)


if __name__=='__main__':
    main()
