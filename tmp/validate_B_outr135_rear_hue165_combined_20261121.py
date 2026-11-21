"""後赤の色相候補を今回と独立4動画で検証。原本と95%基準は保持。"""
from pathlib import Path
import copy
import hashlib
import json
import sys
import cv2
import numpy as np
import pandas as pd
root=Path(__file__).resolve().parents[1]
base=root/'tools/logs/pixel_camera/turn1600_B_outr135_20261121_resume_v1'
run=base/'geometry_c1p0_Ls5p3_Le31_01/local_execution/run_B_OUTR135_1600_c1p0_Ls5p3_Le31_green_20261121_001'
out=base/'rear_hue165_validation_v2';assert not out.exists();out.mkdir()
read=lambda path:json.loads(path.read_text(encoding='utf-8-sig'))
sys.path[:0]=[str(root/'tools'),str(root/'tools/analysis')]
from video_marker_pose import marker_registration,registration_quad,pose_world_corners,colour_components,select_marker_pair
cv2.setNumThreads(4)
candidate=copy.deepcopy(read(run/'body_geometry_used.json'))
candidate['marker_detection']['red']['hue_ranges']=[[0,10],[165,179]]
candidate['marker_detection_source']='2026-11-21右135度出口の実後赤が桃色化した欠測13画像を診断。赤の高色相域下限170→165だけ変更。彩度/明度/面積/前緑/開演算/間隔/実ArUco/観測95%/直線品質は維持。11画像の実後マーカーを視認。独立4動画と今回全区間で別検証。'
candidate_path=out/'body_geometry_B_green35_rear_hue165_s30.json'
with candidate_path.open('x',encoding='utf-8') as stream:json.dump(candidate,stream,ensure_ascii=False,indent=2)
def check(folder,indices,apply_candidate_to_original):
    ref=read(folder/'marker_reference_used.json');old_geo=read(folder/'body_geometry_used.json')
    new_geo=copy.deepcopy(old_geo);new_geo['marker_detection']=copy.deepcopy(candidate['marker_detection'])
    if apply_candidate_to_original:
        assert old_geo['marker_detection']['red']['minimum_saturation']==30
        new_geo=candidate
    pose=pd.read_csv(folder/'full/apparent_floor_marker_pose.csv')
    cap=cv2.VideoCapture(read(folder/'trial.json')['video_file']);cap.set(cv2.CAP_PROP_ORIENTATION_AUTO,0)
    rows=[]
    for index in indices:
        row=pose.iloc[index];cap.set(cv2.CAP_PROP_POS_FRAMES,int(row.frame));ok,frame=cap.read();assert ok
        registration=marker_registration(frame,ref);quad=registration_quad(registration)
        assert quad is not None and len(set(registration['detected_marker_ids']))>=2
        matrix=cv2.getPerspectiveTransform(quad,pose_world_corners(ref['board_side_mm']))
        def pair(geometry):
            return select_marker_pair(colour_components(frame,'green',geometry['marker_detection']),
                colour_components(frame,'red',geometry['marker_detection']),matrix,None,
                nominal_marker_span_mm=geometry['nominal_marker_span_mm'],marker_span_limits_mm=geometry['marker_span_limits_mm'])
        before,after=pair(old_geo),pair(new_geo)
        change=float(np.linalg.norm(np.array(before)-np.array(after),axis=1).max()) if before is not None and after is not None else None
        rows.append(dict(frame=int(row.frame),time_video_s=float(row.time_video_s),original=before is not None,candidate=after is not None,
            maximum_pair_change_mm=change,registered_aruco_ids=registration['detected_marker_ids']))
    cap.release();return rows
pose=pd.read_csv(run/'full/apparent_floor_marker_pose.csv');quality=read(run/'full/report.json')['required_motion_window_quality']
missing=pose.index[~pose.marker_detected_raw.astype(bool)].tolist()
controls=pose.index[pose.time_video_s.between(*quality['required_interval_pts_s'])].to_numpy()[::4].tolist()
indices=sorted(set(missing+controls+[0,12,24,len(pose)-1,len(pose)-13,len(pose)-25]))
current=check(run,indices,True)
inlet=root/'tools/logs/pixel_camera/turn1600_B_inr135_20261120_v1'
folders=[inlet/relative for relative in [
    'geometry_c0p4_Ls10p5_Le17p2_02/local_execution/run_B_INR135_1600_c0p4_Ls10p5_Le17p2_green_20261120_002',
    'geometry_c0p4_Ls10p5_Le17p2_repeat_01/local_execution/run_B_INR135_1600_c0p4_Ls10p5_Le17p2_repeat_green_20261120_001',
    'geometry_c0p4_Ls10p5_Le17p2_repeat_02/local_execution/run_B_INR135_1600_c0p4_Ls10p5_Le17p2_repeat_green_20261120_002']]
folders.append(root/'tools/logs/pixel_camera/turn1600_B_outr135_20261120_v1/angle_c1p0_Ls6_Le27p9_01/local_execution/run_B_OUTR135_1600_c1p0_Ls6_Le27p9_green_20261120_001')
independent=[]
for folder in folders:
    data=pd.read_csv(folder/'full/apparent_floor_marker_pose.csv')
    window=read(folder/'full/report.json')['required_motion_window_quality']['required_interval_pts_s']
    indices=data.index[data.time_video_s.between(*window)].to_numpy()
    chosen=indices[np.linspace(0,len(indices)-1,20).round().astype(int)].tolist()
    independent.append(dict(source=str(folder),checks=check(folder,chosen,False)))
all_rows=current+[row for item in independent for row in item['checks']]
old_valid=[row for row in all_rows if row['original']]
lost=[row for row in old_valid if not row['candidate']]
changed=[row for row in old_valid if row['candidate'] and row['maximum_pair_change_mm']>1.5]
recovered=[row['frame'] for row in current if not row['original'] and row['candidate']]
passed=not lost and not changed and len(recovered)==11
result=dict(state='SAMPLED_VALIDATION_PASSED_FULL_REPROCESS_REQUIRED' if passed else 'SAMPLED_VALIDATION_REJECTED',
    candidate=str(candidate_path),current_samples=current,independent_runs=independent,
    maximum_allowed_pair_change_mm=1.5,old_valid_pairs_lost=lost,old_valid_pairs_changed_over_limit=changed,
    maximum_observed_pair_change_mm=max(row['maximum_pair_change_mm'] for row in old_valid if row['candidate']),
    recovered_actual_marker_frames=recovered,
    visual_review='rear_hue165_pairs.jpgの11ペアすべてで実前緑/後桃色マーカーを視認。160/155候補より範囲の小さい165を選択。',
    source_visual_review=str(run/'rear_hue_diagnostic_v1/rear_hue165_pairs.jpg'),
    original_failed_quality_preserved=True,acceptance_threshold_unchanged=.95,full_current_motion_validation_pending=True,
    hardware_operations=0,candidate_deployed=False)
with (out/'sample_validation.json').open('x',encoding='utf-8') as stream:json.dump(result,stream,ensure_ascii=False,indent=2)
print(json.dumps(dict(state=result['state'],current_samples=len(current),independent_samples=sum(len(item['checks']) for item in independent),
    maximum_observed_pair_change_mm=result['maximum_observed_pair_change_mm'],lost=len(lost),changed_over_limit=len(changed),recovered_frames=recovered),ensure_ascii=False),flush=True)
raise SystemExit(0 if passed else 1)

