"""004保存静止動画から既存2/3登録・格子APIで90度回転後の独立参照を作る。"""
from pathlib import Path
import copy
import hashlib
import json
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tools/logs/pixel_camera/turn1800_A_inr135_20261120_v1'
OUT = BASE / 'camera_reference_004_v1'
VIDEO = Path('D:/TYPE8_Camera/recordings/remote/20261120_090228_240fps_1e721ca2.mp4')
OLD_REFERENCE = BASE / 'baseline01/local_execution/prepare_A_INR135_1800_baseline_20261120_004/reference.used.json'
OLD_GEOMETRY = BASE / 'baseline01/local_execution/prepare_A_INR135_1800_baseline_20261120_004/geometry.used.json'
sys.path[:0] = [str(ROOT / 'tools'), str(ROOT / 'tools/analysis')]
from pixel_camera_reference import (CORNER_NAMES, refine_marker_vertices, marker_corners_by_id,
    register_marker_reference, create_marker_reference, fit_grid_reference, split_grid_checkpoints,
    refine_board_from_checkpoints, detect_registered_vertices, detect_board_reference)
from pixel_camera_framing import read_frame
from pixel_camera_accuracy import find_grid_checkpoints


def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def save(name, value):
    with (OUT / name).open('x', encoding='utf-8') as file:
        json.dump(value, file, ensure_ascii=False, indent=2, allow_nan=False)
        file.write('\n')


def checks(frame, matrix, config):
    return find_grid_checkpoints(frame, matrix, config['pitch_mm'],
        occlusion_kernel_px=config['occlusion_kernel_px'],
        minimum_colored_component_area_px=config['minimum_colored_component_area_px'],
        minimum_colored_value=config['minimum_colored_value'])


OUT.mkdir(exist_ok=False)
cv2.setNumThreads(2)
old, geometry = read(OLD_REFERENCE), read(OLD_GEOMETRY)
old_bytes, geometry_bytes = OLD_REFERENCE.read_bytes(), OLD_GEOMETRY.read_bytes()
frame, stamp = read_frame(VIDEO, 4.)
corners, ids, _ = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)).detectMarkers(frame)
found = {int(key): quad.reshape(4, 2).tolist() for key, quad in zip(ids.ravel(), corners)}
assert {2, 3} <= set(found)
markers = refine_marker_vertices(frame, dict(actual_2_3=dict(ids=[2,3], corners_px=[found[2], found[3]])))
assert set(marker_corners_by_id(markers)) == {2, 3}
old_alignment = register_marker_reference(markers, old)
assert old_alignment['available'], old_alignment
projected = np.array([old_alignment['points_px'][key] for key in CORNER_NAMES])
new_corners = projected[[1,2,3,0]]
# 90度変換の意味を画像位置とは独立に確認する。
coordinate_change = np.array([[0., 1., 0.],[-1., 0., 360.],[0., 0., 1.]])
assert np.allclose(cv2.perspectiveTransform(np.float64([[[0,0],[360,0],[360,360],[0,360]]]), coordinate_change),
    [[[0,360],[0,0],[360,0],[360,360]]])
corner_ids = dict(TL=3, TR=2, BR=0, BL=1)
inspection = dict(corners=markers, board_reference=dict(available=True,
    method='old board corners projected through actual decoded2/3 vertices; quarter-turn seed only',
    points_px=dict(zip(CORNER_NAMES, new_corners.tolist()))))
seed = create_marker_reference(inspection, board_corner_ids=corner_ids)
seed['corner_refinement'] = 'subpix_original_win3'
seed['grid_refinement'] = copy.deepcopy(old['grid_refinement'])
config = seed['grid_refinement']
assert config['minimum_training_points'] == 8 and config['minimum_heldout_points'] == 12
assert config['maximum_heldout_rmse_mm'] == .75 and config['maximum_heldout_p95_mm'] == 1.5 and config['maximum_heldout_error_mm'] == 2.
config.setdefault('training_point_count', 10)
for candidate in (seed,):
    candidate['marker_search_rois_px'] = {str(key):[max(0,int(np.floor(np.min(quad,axis=0)[0]-55))),
        max(0,int(np.floor(np.min(quad,axis=0)[1]-55))), min(frame.shape[1],int(np.ceil(np.max(quad,axis=0)[0]+55))),
        min(frame.shape[0],int(np.ceil(np.max(quad,axis=0)[1]+55)))] for key,quad in marker_corners_by_id(markers).items()}
save('seed_reference_projected.json', seed)
save('seed_correspondence.json', dict(source_video=str(VIDEO), time_video_s=stamp,
    old_reference=str(OLD_REFERENCE), old_reference_id=old['reference_id'], full_decoded_ids=sorted(found),
    registration_ids=[2,3], other_ids_diagnostic_only=[0,1], old_projected_corners_px=projected.tolist(),
    new_corner_source_order=['old_TR','old_BR','old_BL','old_TL'], new_corners_px=new_corners.tolist(),
    old_board_to_new_board=coordinate_change.tolist(), formula='new=(old_y,360-old_x)',
    marker_vertices_are_board_corners=False, source_sha256=hashlib.sha256(old_bytes).hexdigest()))
cv2.imwrite(str(OUT / 'calibration_source_frame.png'), frame)
grid_checks = checks(frame, seed['image_to_board_mm'], config)
save('calibration_grid_checkpoints.json', grid_checks)
training, heldout = split_grid_checkpoints(grid_checks, config)
assert len(training) >= 8 and len(heldout) >= 12, (len(training), len(heldout))
seed_registration = register_marker_reference(markers, seed)
validated = refine_board_from_checkpoints(seed_registration, grid_checks, seed)
save('calibration_grid_quality.json', validated)
assert validated['available'], validated
source = dict(video_file=str(VIDEO), time_video_s=stamp, camera_height_mm=1500., zoom_ratio=1.8,
    old_reference=str(OLD_REFERENCE), old_reference_sha256=hashlib.sha256(old_bytes).hexdigest(),
    coordinate_transform_from_old='new=(old_y,360-old_x)', old_corners_used_as_seed_only=True,
    actual_decoded_registration_ids=[2,3], diagnostic_only_ids=[0,1], grid_calibration=validated['grid_quality'])
profile = fit_grid_reference(training, markers, source=source, minimum_points=8)
profile['board_corner_ids'] = corner_ids
profile['corner_refinement'] = 'subpix_original_win3'
profile['grid_refinement'] = config
profile['marker_search_rois_px'] = seed['marker_search_rois_px']
profile['grid_calibration']['training_grid_indices'] = [[point['col'],point['row']] for point in training]
profile['calibration_source_method'] = 'registered_markers_plus_distributed_grid'
profile['board_axes'] = 'Current004 image board TL origin; x to image TR, y to image BL; grid45mm, side360mm'
profile['metric_accuracy_verified'] = False
assert set(profile['markers_px']) == {'2','3'}

validation = []
for requested_stamp in (3.35,3.55,3.75,4.25,4.45,4.65,4.85):
    observed, actual_stamp = read_frame(VIDEO, requested_stamp)
    actual_markers = detect_registered_vertices(observed, profile)
    correction = detect_board_reference(observed, actual_markers, profile)
    entry = dict(time_video_s=actual_stamp, available=correction['available'],
        actual_decoded_ids=sorted(marker_corners_by_id(actual_markers)), correction=correction)
    validation.append(entry)
    print(json.dumps(dict(time_video_s=actual_stamp, available=entry['available'],
        grid_quality=correction.get('grid_quality'), reason=correction.get('reason')),ensure_ascii=False),flush=True)
save('independent_frame_grid_validation.json', dict(frames=validation, all_passed=all(item['available'] for item in validation),
    reference_frame_not_used_for_validation=True, hardware_operations=0))
assert all(item['available'] for item in validation), '独立frame格子検証で不合格。新refはまだ保存/配布しない。'
save('marker_reference_A_current_rotated004.json', profile)
new_geometry = copy.deepcopy(geometry)
new_geometry['reference_id'] = profile['reference_id']
new_geometry['reference_binding_source'] = str(OUT / 'marker_reference_A_current_rotated004.json')
new_geometry['reference_rebinding_note'] = '004保存画像の90deg新TL座標への参照再結合のみ。高さ/物理係数/検出閾値/camerafoot未知を維持。'
save('body_geometry_A_yellow_v2_rotated004.json', new_geometry)
assert OLD_REFERENCE.read_bytes() == old_bytes and OLD_GEOMETRY.read_bytes() == geometry_bytes
save('completion.json', dict(reference=str(OUT / 'marker_reference_A_current_rotated004.json'),
    geometry=str(OUT / 'body_geometry_A_yellow_v2_rotated004.json'), reference_id=profile['reference_id'],
    old_raw_config_changed=False, hardware_operations=0, registration_ids=[2,3],
    corner_ids_diagnostic=corner_ids, calibration_training_points=len(training), calibration_heldout_points=len(heldout),
    calibration_quality=validated['grid_quality'], independent_frames=len(validation), independent_frames_all_passed=True,
    original_capture_failure_preserved=True, physical_metric_accuracy_verified=False))
print(json.dumps(read(OUT / 'completion.json'),ensure_ascii=False,indent=2),flush=True)
