"""保存済み9×9基準から今回のB単体調整用プロファイルを作る。機器操作なし。"""
from pathlib import Path
from itertools import permutations
import hashlib
import json
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'tools'))
from pixel_camera_reference import create_marker_reference


def save_new(path, value):
    with path.open('x', encoding='utf-8') as stream:
        stream.write(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')


def main():
    folder = Path(__file__).resolve().parent
    source = ROOT / 'tools/logs/pixel_camera/board_9x9_reference.json'
    reference = json.loads(source.read_text(encoding='utf-8'))
    if reference.get('schema') != 1 or (reference['rows'], reference['columns']) != (9, 9):
        raise ValueError('登録済み9×9基準が必要です')
    keys = sorted(reference['source_markers_image_px'])
    if len(keys) != 4:
        raise ValueError('盤面の4隅に対応する実検出IDが必要です')
    image = np.concatenate([reference['source_markers_image_px'][key] for key in keys]).astype(float)
    board = np.concatenate([reference['markers_cell'][key] for key in keys]).astype(float)*90.
    matrix, _ = cv2.findHomography(image, board, method=0)
    fitted = cv2.perspectiveTransform(image.reshape(-1, 1, 2), matrix).reshape(-1, 2)
    if np.max(np.linalg.norm(fitted-board, axis=1)) > .001:
        raise ValueError('登録頂点と盤面座標の尺度が一致しません')
    targets = np.array([[0, 0], [810, 0], [810, 810], [0, 810]], dtype=float)
    means = {key: np.mean(reference['markers_cell'][key], axis=0)*90 for key in keys}
    assignment = min(permutations(keys), key=lambda ids: sum(
        np.linalg.norm(means[key]-target) for key, target in zip(ids, targets)))
    quad = cv2.perspectiveTransform(targets.reshape(-1, 1, 2), np.linalg.inv(matrix)).reshape(-1, 2)
    names = ('TL', 'TR', 'BR', 'BL')
    inspection = dict(corners={'registered_source': dict(ids=[int(key) for key in keys],
        corners_px=[reference['source_markers_image_px'][key] for key in keys])},
        board_reference=dict(available=True, method='saved_9x9_grid_reference',
            points_px=dict(zip(names, quad.tolist()))))
    profile = create_marker_reference(inspection,
        image_size=(reference['image_width'], reference['image_height']), board_side_mm=810.,
        source_inspection=str(source), board_corner_ids=dict(zip(names, map(int, assignment))))
    profile['image_to_board_mm'] = (matrix/matrix[2, 2]).tolist()
    profile['source'] = dict(reference_json=str(source),
        reference_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        source_frame=reference['source_frame'], source_frame_sha256=reference['source_sha256'],
        cell_mm=90., rows=9, columns=9)
    profile['limitations'] = reference['limitations']
    profile['metric_accuracy_verified'] = False
    profile['corner_refinement'] = 'subpix_original_win3'
    geometry = dict(schema_version=1, mouse='B', front_colour='yellow', rear_colour='red',
        reference_id=profile['reference_id'], front_height_mm=10., rear_height_mm=1.,
        rear_height_is_approximate=True,
        marker_height_source='2026-11-14 user: front/rear heights unchanged; front 10 mm, rear about 1 mm',
        marker_colour_source='2026-11-14 user: B front yellow / rear red',
        nominal_marker_span_mm=50., nominal_marker_span_is_approximate=True,
        nominal_marker_span_source='2026-11-14 user: approximately the same separation as Mouse A',
        camera_height_policy='per_measurement', camera_height_mm=1500.,
        camera_height_is_approximate=True, camera_height_measurement_confirmed=False,
        camera_height_source='2026-11-14 user: current lens-to-floor height approximately 1500 mm',
        camera_center_board_mm=None, camera_foot_board_mm=None,
        axle_offset_from_marker_midpoint_mm=None, preferred_analysis_mode='relative_marker_midpoint',
        hardware_height_correction_verified=False,
        marker_detection=dict(red=dict(minimum_saturation=60, minimum_value=100),
                              yellow=dict(minimum_saturation=60, minimum_value=65)),
        marker_detection_source='Saved B frame review: red marker merges with red maze walls at value 65; value 100 separates it. New static preview required.',
        analysis_capabilities=dict(relative_midpoint_displacement=True, absolute_position=False,
                                   height_corrected_body_heading=False))
    save_new(folder/'marker_reference_9x9.json', profile)
    save_new(folder/'body_geometry_B.json', geometry)
    print(json.dumps(dict(board_side_mm=profile['board_side_mm'], reference_id=profile['reference_id'],
                         board_corner_ids=profile['board_corner_ids']), ensure_ascii=False))


if __name__ == '__main__':
    main()
