"""保存済み動画の実観測ペアから機体別の外観を学習する。機器操作なし。"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import sys
import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from body_appearance_model import LearnedBodyKeypointModel, extract_canonical_patch, FRONT_CANON_XY, REAR_CANON_XY
from pixel_camera_detection import file_sha256
from pixel_camera_position import prepare_reference
from video_marker_pose import marker_registration, registration_quad, pose_world_corners, colour_components, select_marker_pair, transform_points


def train(trials, geometry_path, output, *, samples_per_trial=30):
    geometry=json.loads(Path(geometry_path).read_text(encoding='utf-8'))
    output=Path(output); output.mkdir(parents=True,exist_ok=False)
    samples=[]; sources=[]; labels=[]; spans_px=[]; spans_mm=[]
    for trial_path in map(Path,trials):
        trial=json.loads(trial_path.read_text(encoding='utf-8'))
        if trial.get('mouse')!=geometry.get('mouse'):
            raise ValueError('機体の異なる動画を一緒に学習できません')
        reference=json.loads((trial_path.parent/'marker_reference_used.json').read_text(encoding='utf-8'))
        if geometry.get('reference_id')!=reference.get('reference_id'):
            raise ValueError('学習用の盤面参照が一致しません')
        start=max(0.,trial.get('video_motion_start_estimate_s',1.)-.5)
        end=start+4.5
        reference,diagnostic=prepare_reference(Path(trial['video_file']),reference,start,end)
        if not diagnostic['available']:
            raise ValueError('学習動画のAR登録に失敗しました: '+str(trial_path))
        case_samples=[]
        cap=cv2.VideoCapture(trial['video_file']); cap.set(cv2.CAP_PROP_ORIENTATION_AUTO,0)
        try:
            for stamp in np.linspace(start,end,samples_per_trial*2):
                cap.set(cv2.CAP_PROP_POS_MSEC,float(stamp)*1000)
                ok,frame=cap.read()
                if not ok:continue
                registration=marker_registration(frame,reference); quad=registration_quad(registration)
                if quad is None:continue
                matrix=cv2.getPerspectiveTransform(quad,pose_world_corners(reference['board_side_mm']))
                pair=select_marker_pair(colour_components(frame,geometry['front_colour'],geometry.get('marker_detection')),
                    colour_components(frame,geometry['rear_colour'],geometry.get('marker_detection')),matrix,None,
                    board_side_mm=reference['board_side_mm'],nominal_marker_span_mm=geometry.get('nominal_marker_span_mm'),
                    marker_span_limits_mm=geometry.get('marker_span_limits_mm'))
                if pair is None:continue
                native=transform_points(pair,np.linalg.inv(matrix)); patch,_=extract_canonical_patch(frame,*native)
                case_samples.append((patch,native,float(cap.get(cv2.CAP_PROP_POS_MSEC))/1000,float(np.linalg.norm(pair[0]-pair[1]))))
        finally:cap.release()
        if len(case_samples)<3:
            raise ValueError('学習に使える前後の実観測が不足しています: '+str(trial_path))
        chosen=np.linspace(0,len(case_samples)-1,min(samples_per_trial,len(case_samples)),dtype=int)
        for index in chosen:
            patch,native,pts,span=case_samples[index]
            name=f'patch_{len(samples):04d}.png';cv2.imwrite(str(output/name),patch)
            labels.append(dict(patch=name,source_video=trial['video_file'],video_pts_s=pts,
                front_px=native[0].tolist(),rear_px=native[1].tolist(),label_source='decoded_aruco_and_observed_colour_pair'))
            samples.append((patch,FRONT_CANON_XY,REAR_CANON_XY))
            spans_px.append(float(np.linalg.norm(native[0]-native[1])));spans_mm.append(span)
        sources.append(dict(source_trial=str(trial_path.resolve()),source_video=trial['video_file'],
            video_sha256=file_sha256(trial['video_file']),reference_search=diagnostic))
        print(f'{trial["mouse"]}: {len(chosen)} observed training patches from {trial_path.parent.name}',flush=True)
    metadata=dict(mouse=geometry['mouse'],front_colour=geometry['front_colour'],rear_colour=geometry['rear_colour'],
        sources=sources,marker_span_mm_median=float(np.median(spans_mm)),
        labels_are_independent_ground_truth=False,backend='learned_body_appearance',robot_commands_sent=False)
    model=LearnedBodyKeypointModel.fit_from_frames(samples,metadata)
    model.span_px_median=float(np.median(spans_px));model.save(output/'appearance.npz')
    (output/'training.json').write_text(json.dumps(dict(metadata=model.metadata,labels=labels),ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    return output/'appearance.npz'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial',type=Path,nargs='+',required=True)
    parser.add_argument('--geometry',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--samples-per-trial',type=int,default=30)
    args=parser.parse_args()
    if not 3<=args.samples_per_trial<=100:parser.error('samples-per-trial must be 3..100')
    print(train(args.trial,args.geometry,args.output,samples_per_trial=args.samples_per_trial))


if __name__=='__main__':main()
