"""004基準のsource training10点を固定し、別OFF動画で格子の全基準を再検証する。"""
from pathlib import Path
import copy
import csv
import hashlib
import json
import sys

import cv2

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT/'tools/logs/pixel_camera/turn1800_A_inr135_20261120_v1'
SOURCE = BASE/'camera_reference_004_v1'
OUT = BASE/'camera_reference_004_fixed10_v1'
REFERENCE = OUT/'marker_reference_A_current_rotated004_fixed10.json'
GEOMETRY = OUT/'body_geometry_A_yellow_v2_rotated004_fixed10.json'
MANIFEST = ROOT/'tools/logs/pixel_camera/20261120_085933_f0946d18.capture.json'
sys.path[:0] = [str(ROOT/'tools'), str(ROOT/'tools/analysis')]
from video_marker_pose import iter_video_frames, marker_registration


def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def digest(path):
    result=hashlib.sha256()
    with path.open('rb') as file:
        for chunk in iter(lambda:file.read(1024*1024),b''):
            result.update(chunk)
    return result.hexdigest()


def save(path,value):
    with path.open('x',encoding='utf-8') as file:
        json.dump(value,file,ensure_ascii=False,indent=2,allow_nan=False);file.write('\n')


OUT.mkdir(exist_ok=False)
original = read(SOURCE/'marker_reference_A_current_rotated004.json')
profile = copy.deepcopy(original)
indices = profile['grid_calibration']['training_grid_indices']
assert len(indices)==10 and len({tuple(pair) for pair in indices})==10
profile['grid_refinement']['training_policy']='fixed_indices'
profile['grid_refinement']['training_grid_indices']=indices
assert profile['grid_refinement']['minimum_training_points']==8
assert profile['grid_refinement']['minimum_heldout_points']==12
assert set(profile['markers_px'])=={'2','3'}
profile['fixed_training_binding']=dict(source_reference=str(SOURCE/'marker_reference_A_current_rotated004.json'),
    source_reference_sha256=digest(SOURCE/'marker_reference_A_current_rotated004.json'),
    source_calibration_time_video_s=profile['source']['time_video_s'], fixed_training_grid_indices=indices,
    per_frame_training_group_changes_allowed=False, fixed_points_must_be_visible_at_least=8,
    framewise_heldout_minimum=12, existing_reference_id_preserved=True)
save(REFERENCE,profile)
geometry = read(SOURCE/'body_geometry_A_yellow_v2_rotated004.json')
assert geometry['reference_id']==profile['reference_id']
geometry['reference_binding_source']=str(REFERENCE)
geometry['reference_rebinding_note']='同じmarker/metric geometryとrefID。source10点のfixed_indices候補へ結合。物理値・高さ・検出閾値・camerafoot未知は維持。'
save(GEOMETRY,geometry)
print(json.dumps(dict(reference=str(REFERENCE),geometry=str(GEOMETRY),reference_id=profile['reference_id'],indices=indices)),flush=True)
manifest=read(MANIFEST);video=Path(manifest['video_file'])
assert manifest['state']=='PASSED' and manifest['video_inspection']['sha256_rechecked']
assert digest(video)==manifest['sha256']
cv2.setNumThreads(2)
rows=[]
fixed_set={tuple(pair) for pair in indices}
for index,frame,stamp in iter_video_frames(video,raw_orientation=True,video_window=(3.3,4.95),frame_stride=4):
    correction=marker_registration(frame,profile)
    ids=set(correction['detected_marker_ids']);used=set(correction.get('used_marker_ids',[]))
    quality=correction.get('grid_quality',{})
    training_set={tuple(pair) for pair in quality.get('training_grid_indices',[])}
    heldout_set={tuple(pair) for pair in quality.get('heldout_grid_indices',[])}
    fixed_groups=training_set<=fixed_set and not heldout_set&fixed_set
    passed=bool(correction['available'] and ids>={2,3} and used=={2,3} and fixed_groups
        and quality.get('training_policy')=='fixed_indices' and quality.get('training_points',0)>=8
        and quality.get('heldout_points',0)>=12 and quality.get('position_rmse_mm',float('inf'))<=.75
        and quality.get('position_p95_mm',float('inf'))<=1.5 and quality.get('position_max_mm',float('inf'))<=2.)
    rows.append(dict(frame=index,time_video_s=stamp,passed=passed,actual_decoded_ids=sorted(ids),
        used_marker_ids=sorted(used),training_heldout_fixed_assignment_verified=fixed_groups,
        failure_reason=correction.get('reason'),grid_quality=quality))
    if len(rows)%30==0: print(f'fixed10 {len(rows)}frames / {stamp:.3f}s',flush=True)
good=[row['grid_quality'] for row in rows if row['passed']]
passed_count=len(good)
result=dict(schema_version=1,purpose='FIXED_TRAINING_10_INDEPENDENT_OFF_VIDEO_PREFLIGHT',
    source_video=str(video),source_manifest=str(MANIFEST),source_reference=str(REFERENCE),geometry=str(GEOMETRY),
    reference_id=profile['reference_id'],reference_id_same_as_visible_candidate=True,
    window_s=[3.3,4.95],frame_stride=4,frames=len(rows),registered_grid_passed_frames=passed_count,
    registered_grid_passed_fraction=passed_count/len(rows),passed=bool(len(rows)>=12 and passed_count/len(rows)>=.95),
    fixed_training_grid_indices=indices,frame_group_changes_allowed=False,
    all_frames_fixed_assignment_verified=all(row['training_heldout_fixed_assignment_verified'] for row in rows),
    training_counts=sorted({value['training_points'] for value in good}),heldout_counts=sorted({value['heldout_points'] for value in good}),
    maximum_grid_rms_mm=max(value['position_rmse_mm'] for value in good) if good else None,
    maximum_grid_p95_mm=max(value['position_p95_mm'] for value in good) if good else None,
    maximum_grid_error_mm=max(value['position_max_mm'] for value in good) if good else None,
    required_actual_registration_ids=[2,3],ids0_1_used_for_missing_marker_substitution=False,
    thresholds=dict(registration_fraction=.95,training_points_minimum=8,heldout_points_minimum=12,rms_mm=.75,p95_mm=1.5,max_mm=2.),
    old_visible_reference_and_failures_preserved=True,body_pair_required=False,physical_clearance_verified=False,
    hardware_operations=0,common_source_changed=False,adoption_pending_root_review=True,
    input_sha256={str(path):digest(path) for path in (MANIFEST,REFERENCE,GEOMETRY,video,Path(__file__))},frames_detail=rows)
save(OUT/'independent_off_registration_preflight.json',result)
with (OUT/'independent_off_registration_frames.csv').open('x',encoding='utf-8',newline='') as file:
    flat=[dict(frame=row['frame'],time_video_s=row['time_video_s'],passed=row['passed'],
        fixed_assignment=row['training_heldout_fixed_assignment_verified'],reason=row['failure_reason'],
        train=row['grid_quality'].get('training_points'),heldout=row['grid_quality'].get('heldout_points'),
        rms_mm=row['grid_quality'].get('position_rmse_mm'),p95_mm=row['grid_quality'].get('position_p95_mm'),
        max_mm=row['grid_quality'].get('position_max_mm')) for row in rows]
    writer=csv.DictWriter(file,fieldnames=list(flat[0]));writer.writeheader();writer.writerows(flat)
print(json.dumps({key:value for key,value in result.items() if key not in ('input_sha256','frames_detail')},ensure_ascii=False,indent=2),flush=True)
