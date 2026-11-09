"""既知の盤面格子で登録基準を再校正する。カメラ・機体には接続しない。"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path
import sys

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pixel_camera_framing import inspect_rois, read_frame, write_json
from pixel_camera_reference import (fit_grid_reference, marker_corners_by_id, register_marker_reference,
                                    refine_board_from_checkpoints)
from pixel_camera_accuracy import find_grid_checkpoints, evaluate_checkpoints, write_csv


def training_location(col, row):
    """予め固定した約1/4の点だけを使い、残りを空間的な検査用に残す。"""
    return col % 2 == 1 and row % 2 == 1


def position_metrics(points):
    if not points:
        return dict(observations=0, position_rmse_mm=None, position_p95_mm=None, position_max_mm=None)
    errors = np.asarray([point['error_mm'] for point in points])
    return dict(observations=len(points), unique_points=len({(p['col'], p['row']) for p in points}),
                position_rmse_mm=float(np.sqrt(np.mean(errors**2))),
                position_p95_mm=float(np.percentile(errors, 95)), position_max_mm=float(errors.max()))


def calibration_from_samples(samples, seed, capture, pitch_mm=45.0):
    """前半だけで校正し、後半の未使用格子点で検査する。"""
    if len(samples) < 6:
        raise ValueError('at least six samples are required')
    training, validation = samples[:len(samples)//2], samples[len(samples)//2:]
    grouped = {}
    for sample in training:
        for point in sample['checks']:
            if point['accepted'] and training_location(point['col'], point['row']):
                grouped.setdefault((point['col'], point['row']), []).append(point)
    points = []
    for key, observations in sorted(grouped.items()):
        if len(observations) < max(2, len(training)//2):
            continue
        point = copy.deepcopy(observations[0])
        point['image_x_px'], point['image_y_px'] = np.median(
            [[p['image_x_px'], p['image_y_px']] for p in observations], axis=0).tolist()
        point['training_observations'] = len(observations)
        points.append(point)
    ids = sorted(map(int, seed['markers_px']))
    vertices = {}
    for marker_id in ids:
        values = [marker_corners_by_id(sample['markers'])[marker_id] for sample in training
                  if marker_id in marker_corners_by_id(sample['markers'])]
        if len(values) >= max(2, len(training)//2):
            vertices[marker_id] = np.median(values, axis=0).tolist()
    markers = {'training_median': dict(ids=list(vertices), corners_px=list(vertices.values()))}
    profile = fit_grid_reference(points, markers, image_size=seed['image_size'], board_side_mm=pitch_mm*8,
                                 source=dict(capture_json=capture, training_samples=len(training)))
    profile['board_corner_ids'] = seed['board_corner_ids']
    profile['grid_calibration'].update(pitch_mm=pitch_mm, pitch_source='user_confirmed',
        training_grid_indices=[[p['col'], p['row']] for p in points],
        training_rule='col and row both odd; remaining interior grid points are held out',
        training_pts_s=[sample['pts'] for sample in training],
        physical_reference_uncertainty_mm=None)
    profile['grid_refinement'] = dict(enabled=True, pitch_mm=pitch_mm,
        training_grid_indices=[[col, row] for row in range(1, 8) for col in range(1, 8) if training_location(col, row)],
        minimum_training_points=8, minimum_heldout_points=12,
        maximum_heldout_rmse_mm=.75, maximum_heldout_p95_mm=1.5, maximum_heldout_error_mm=2.0)
    search_vertices = {}
    for marker_id in ids:
        values = [marker_corners_by_id(sample.get('coarse_markers', sample['markers']))[marker_id]
                  for sample in training if marker_id in marker_corners_by_id(sample.get('coarse_markers', sample['markers']))]
        if len(values) >= max(2, len(training)//2):
            search_vertices[str(marker_id)] = np.median(values, axis=0).tolist()
    profile['markers_search_px'] = search_vertices
    evaluated, transferred, fixed = [], [], []
    frames = []
    for sample in validation:
        result = register_marker_reference(sample['markers'], profile, seed['image_size'])
        heldout = [p for p in sample['checks'] if not training_location(p['col'], p['row'])]
        fixed.extend(evaluate_checkpoints(heldout, profile['image_to_board_mm']))
        if result['available']:
            transferred.extend(evaluate_checkpoints(heldout, result['homography_image_to_board_mm']))
        refined = refine_board_from_checkpoints(result, sample['checks'], profile)
        frames.append(dict(sample=sample['sample'], pts=sample['pts'], accepted=refined['available'],
                           reason=refined.get('reason', ''), grid_quality=refined.get('grid_quality')))
        if refined['available']:
            evaluated.extend(dict(p, sample=sample['sample'], pts=sample['pts']) for p in
                             evaluate_checkpoints(heldout, refined['homography_image_to_board_mm']))
    report = dict(method='later frames, grid points never used to fit this profile',
                  registration_mode='per_frame_grid',
                  validation_frames=frames, heldout=position_metrics(evaluated),
                  per_frame_marker_registration=position_metrics(transferred),
                  fixed_first_half_grid=position_metrics(fixed),
                  scope='visible interior board surface only; body marker height not evaluated',
                  training_fit_is_accuracy_validation=False)
    return profile, points, evaluated, report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--seed-reference', type=Path, required=True)
    parser.add_argument('--roi-settings', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--pitch-mm', type=float, default=45)
    parser.add_argument('--frames', type=int, default=12)
    args = parser.parse_args()
    if args.frames < 6 or not np.isfinite(args.pitch_mm) or args.pitch_mm <= 0:
        parser.error('frames >= 6 and finite positive pitch required')
    args.output.mkdir(parents=True, exist_ok=False)
    capture = json.loads(args.capture.read_text(encoding='utf-8'))
    if capture.get('state') != 'PASSED':
        raise ValueError('source capture must have passed transfer/decode/hash checks')
    seed = json.loads(args.seed_reference.read_text(encoding='utf-8'))
    rois = json.loads(args.roi_settings.read_text(encoding='utf-8'))['rois']
    samples = []
    for index, stamp in enumerate(np.linspace(1, 2.8, args.frames)):
        frame, pts = read_frame(capture['video_file'], float(stamp))
        # 旧基準は格子探索の窓にだけ使う。校正には実画像から求めた線中心を使う。
        coarse = inspect_rois(frame, rois)
        search = register_marker_reference(coarse, seed, seed['image_size'])
        if not search['available']:
            raise ValueError('seed registration failed: ' + str(search))
        checks = find_grid_checkpoints(frame, search['homography_image_to_board_mm'], args.pitch_mm)
        samples.append(dict(sample=index, pts=pts, checks=checks, coarse_markers=coarse,
                            markers=inspect_rois(frame, rois, refine_corners=True)))
        print(f'grid sample {index+1}/{args.frames}', flush=True)
    profile, training, evaluated, report = calibration_from_samples(
        samples, seed, str(args.capture.resolve()), args.pitch_mm)
    profile['source'].update(video_file=capture['video_file'], video_sha256=capture['sha256'])
    report.update(reference_id=profile['reference_id'], source_capture=str(args.capture.resolve()),
                  input_sha256={str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest()
                                for p in (args.capture, args.seed_reference, args.roi_settings)},
                  source_video_sha256=capture['sha256'])
    profile['calibration_validation'] = dict(report_file=str((args.output/'calibration_report.json').resolve()),
                                            **report['heldout'])
    write_json(args.output/'reference.json', profile)
    write_json(args.output/'calibration_report.json', report)
    write_csv(args.output/'training_points.csv', training)
    write_csv(args.output/'heldout_points.csv', evaluated)
    write_csv(args.output/'detected_checkpoints.csv', [dict(p, sample=s['sample'], pts=s['pts'])
                                                     for s in samples for p in s['checks']])
    print(json.dumps(report['heldout']), flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
