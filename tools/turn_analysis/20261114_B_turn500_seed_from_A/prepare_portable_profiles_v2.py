"""Reproduce the corrected B reference and detector without replacing capture inputs."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys

import cv2

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'tools'))
from pixel_camera_reference import refine_marker_vertices, marker_corners_by_id


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    folder = Path(__file__).resolve().parent
    original = folder / 'portable_marker_reference_B.json'
    geometry_path = folder / 'body_geometry_B_portable.json'
    reference = json.loads(original.read_text(encoding='utf-8'))
    geometry = json.loads(geometry_path.read_text(encoding='utf-8'))
    frame_path = ROOT / reference['source']['frame']
    frame = cv2.imread(str(frame_path))
    if frame is None or digest(frame_path) != reference['source']['frame_sha256']:
        raise ValueError('source frame is missing or changed')
    decoded = {'source': dict(ids=list(map(int, reference['markers_px'])),
                              corners_px=list(reference['markers_px'].values()))}
    refined = refine_marker_vertices(frame, decoded)
    vertices = marker_corners_by_id(refined)
    if set(vertices) != set(map(int, reference['markers_px'])):
        raise ValueError('source corner refinement rejected a registered ID')
    reference['markers_px'] = {str(key): value.tolist() for key, value in sorted(vertices.items())}
    reference['corner_refinement'] = 'subpix_original_win3'
    reference['source']['revision'] = dict(
        previous_profile=str(original.relative_to(ROOT)), previous_sha256=digest(original),
        reason='Apply the declared subpixel refinement to source vertices as well as current vertices',
        refined_source=deepcopy(refined),
        original_trial_inputs_preserved=True)
    # The old 25 px exclusion at ~800 mm subtended ~13 px at ~1500 mm.
    # Actual coloured occlusions remain excluded; held-out accuracy limits stay unchanged.
    reference['grid_refinement']['occlusion_kernel_px'] = 13
    reference['source']['revision']['occlusion_margin_policy'] = '25 * 800 / 1500 rounded to odd 13 px'
    shape = {key: reference[key] for key in ('markers_px', 'board_points_px', 'board_side_mm')}
    reference['reference_id'] = hashlib.sha256(json.dumps(shape, sort_keys=True).encode('utf-8')).hexdigest()[:20]
    geometry['reference_id'] = reference['reference_id']
    geometry['marker_span_limits_mm'] = [40, 65]
    geometry['marker_span_limits_source'] = 'User approximately 50 mm; reject short PCB pairs; never fit image scale to span'
    geometry['marker_detection'] = {colour: dict(minimum_saturation=25, minimum_value=65,
        minimum_area_px=6, opening_kernel_size=0) for colour in ('red', 'yellow')}
    geometry['marker_detection_source'] = 'Pale small dots in saved baseline video at ~1500 mm; all motion frames require validation'
    geometry['previous_geometry'] = dict(path=str(geometry_path.relative_to(ROOT)), sha256=digest(geometry_path))
    for name, content in (('portable_marker_reference_B_v2.json', reference),
                          ('body_geometry_B_portable_v2.json', geometry)):
        with (folder / name).open('x', encoding='utf-8') as stream:
            json.dump(content, stream, ensure_ascii=False, indent=2)
    print(json.dumps(dict(reference_id=reference['reference_id'], output=str(folder)), ensure_ascii=False))


if __name__ == '__main__':
    main()
