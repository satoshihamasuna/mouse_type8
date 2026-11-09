"""マーカー補正に未使用の格子交点を、ユーザー確認の45 mm寸法で検査する。"""
from __future__ import annotations

import argparse
import csv
import hashlib
from itertools import combinations
import json
from pathlib import Path
import sys

import cv2
import numpy as np

TOOLS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))
from pixel_camera_framing import inspect_rois, read_frame, write_json
from pixel_camera_reference import register_marker_reference, refine_board_grid

IMPLEMENTATION_SHA256 = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()


def transform(points, matrix):
    return cv2.perspectiveTransform(np.float64(points).reshape(-1, 1, 2), np.float64(matrix)).reshape(-1, 2)


def line_near_grid_point(gray, inverse, world_point, axis):
    """予測位置は探索窓にだけ使い、実画像の白線中心を独立に当てはめる。"""
    centre = np.asarray(world_point, dtype=float)
    axis = np.asarray(axis, dtype=float)
    along = np.r_[np.linspace(-20, -8, 18), np.linspace(8, 20, 18)]
    predicted = transform(centre+along[:, None]*axis, inverse)
    ends = transform([centre-20*axis, centre+20*axis], inverse)
    direction = ends[1]-ends[0]
    direction /= np.linalg.norm(direction)
    normal = np.array([-direction[1], direction[0]])
    offsets = np.linspace(-8, 8, 65)
    sample = predicted[:, None, :]+offsets[None, :, None]*normal
    values = cv2.remap(gray, sample[:, :, 0].astype('float32'), sample[:, :, 1].astype('float32'),
                       cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT).astype(float)
    # 36本の濃度断面をまとめて処理する。探索窓・閾値・重み付き線中心は従来と同じ。
    background = np.percentile(values, 20, axis=1)
    peaks = np.argmax(values, axis=1)
    contrasts = values[np.arange(len(values)), peaks]-background
    near = abs(offsets[None, :]-offsets[peaks, None]) <= 1.5
    weights = np.maximum(values-background[:, None], 0)*near
    sums = weights.sum(axis=1)
    accepted = (contrasts >= 8) & (peaks >= 4) & (peaks < len(offsets)-4) & (sums > 0)
    indices = np.flatnonzero(accepted)
    centres = (weights[accepted]*offsets).sum(axis=1)/sums[accepted]
    points = predicted[accepted]+centres[:, None]*normal
    contrasts = contrasts[accepted]
    if len(points) < 24:
        return None
    fit = cv2.fitLine(points.astype('float32'), cv2.DIST_HUBER, 0, .01, .01).ravel()
    vector, origin = fit[:2].astype(float), fit[2:].astype(float)
    perpendicular = np.array([-vector[1], vector[0]])
    keep = abs((points-origin) @ perpendicular) < .75
    indices = np.asarray(indices)
    if keep.sum() < 24 or sum(indices[keep] < 18) < 10 or sum(indices[keep] >= 18) < 10:
        return None
    fit = cv2.fitLine(points[keep].astype('float32'), cv2.DIST_L2, 0, .01, .01).ravel()
    vector, origin = fit[:2].astype(float), fit[2:].astype(float)
    perpendicular = np.array([-vector[1], vector[0]])
    rms = float(np.sqrt(np.mean(((points[keep]-origin) @ perpendicular)**2)))
    if abs(vector @ direction) < np.cos(np.deg2rad(8)) or rms > .5:
        return None
    return dict(normal=perpendicular, constant=float(perpendicular @ origin), rms_px=rms,
                samples=int(keep.sum()), contrast_median=float(np.median(np.asarray(contrasts)[keep])))


def find_grid_checkpoints(frame, image_to_board, pitch_mm=45.0, intervals=8):
    """内部交点49個を探索。中心の黒丸や機体の色は線の当てはめに使わない。"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    colored = ((hsv[:, :, 1] > 90) & (hsv[:, :, 2] > 35)).astype('uint8')
    occlusion = cv2.dilate(colored, np.ones((25, 25), 'uint8'))
    inverse = np.linalg.inv(np.asarray(image_to_board))
    results = []
    for row in range(1, intervals):
        for col in range(1, intervals):
            world = [col*pitch_mm, row*pitch_mm]
            predicted = transform([world], inverse)[0]
            x, y = np.rint(predicted).astype(int)
            record = dict(col=col, row=row, expected_x_mm=world[0], expected_y_mm=world[1])
            if not (8 <= x < frame.shape[1]-8 and 8 <= y < frame.shape[0]-8):
                results.append(dict(record, accepted=False, reason='outside_frame'))
                continue
            if occlusion[y, x]:
                results.append(dict(record, accepted=False, reason='colored_object_occlusion'))
                continue
            horizontal = line_near_grid_point(gray, inverse, world, [1, 0])
            vertical = line_near_grid_point(gray, inverse, world, [0, 1])
            if horizontal is None or vertical is None:
                results.append(dict(record, accepted=False, reason='line_quality'))
                continue
            matrix = np.array([horizontal['normal'], vertical['normal']])
            if abs(np.linalg.det(matrix)) < .3:
                results.append(dict(record, accepted=False, reason='parallel_lines'))
                continue
            point = np.linalg.solve(matrix, [horizontal['constant'], vertical['constant']])
            if np.linalg.norm(point-predicted) > 10:
                results.append(dict(record, accepted=False, reason='outside_search_radius'))
                continue
            results.append(dict(record, accepted=True, reason='', image_x_px=float(point[0]), image_y_px=float(point[1]),
                                horizontal_rms_px=horizontal['rms_px'], vertical_rms_px=vertical['rms_px']))
    return results


def evaluate_checkpoints(checkpoints, image_to_board):
    results = []
    for point in checkpoints:
        if not point['accepted']:
            continue
        measured = transform([[point['image_x_px'], point['image_y_px']]], image_to_board)[0]
        dx = float(measured[0]-point['expected_x_mm'])
        dy = float(measured[1]-point['expected_y_mm'])
        results.append(dict(point, measured_x_mm=float(measured[0]), measured_y_mm=float(measured[1]),
                            error_x_mm=dx, error_y_mm=dy, error_mm=float(np.hypot(dx, dy))))
    return results


def heldout_checkpoints(checkpoints, correction, reference_training_points=()):
    """可視点分割でも、その変換に実際に使った点を検査へ混ぜない。"""
    used = set(map(tuple, reference_training_points))
    used.update(tuple(point) for point in correction.get('grid_quality', {}).get('training_grid_indices', []))
    return [dict(point, accepted=False, reason='used_for_reference_calibration')
            if (point['col'], point['row']) in used else dict(point) for point in checkpoints]


def summarize(points, distances):
    errors = np.array([p['error_mm'] for p in points])
    signed = np.array([[p['error_x_mm'], p['error_y_mm']] for p in points])
    delta = np.array([p['error_mm'] for p in distances])
    result = dict(observations=len(points), unique_points=len({(p['col'], p['row']) for p in points}),
                position_rmse_mm=float(np.sqrt(np.mean(errors**2))), position_median_mm=float(np.median(errors)),
                position_p95_mm=float(np.percentile(errors, 95)), position_max_mm=float(errors.max()),
                mean_bias_x_mm=float(signed[:, 0].mean()), mean_bias_y_mm=float(signed[:, 1].mean()),
                distance_observations=len(distances), distance_bias_mm=float(delta.mean()) if len(delta) else None,
                distance_rmse_mm=float(np.sqrt(np.mean(delta**2))) if len(delta) else None,
                distance_max_abs_mm=float(np.max(abs(delta))) if len(delta) else None)
    result['distance_by_intervals'] = {}
    for intervals in sorted({value['intervals'] for value in distances}):
        selected = np.array([value['error_mm'] for value in distances if value['intervals'] == intervals])
        result['distance_by_intervals'][str(intervals)] = dict(observations=len(selected),
            rmse_mm=float(np.sqrt(np.mean(selected**2))), mean_error_mm=float(selected.mean()),
            p95_abs_mm=float(np.percentile(abs(selected), 95)), max_abs_mm=float(abs(selected).max()))
    point_stats = []
    for col, row in sorted({(point['col'], point['row']) for point in points}):
        selected = [point for point in points if point['col'] == col and point['row'] == row]
        coordinates = np.array([[point['measured_x_mm'], point['measured_y_mm']] for point in selected])
        expected = np.array([selected[0]['expected_x_mm'], selected[0]['expected_y_mm']])
        median_error = np.median(coordinates, axis=0)-expected
        point_stats.append(dict(col=col, row=row, observations=len(selected), expected_mm=expected.tolist(),
            median_error_x_mm=float(median_error[0]), median_error_y_mm=float(median_error[1]),
            median_error_mm=float(np.linalg.norm(median_error)),
            repeatability_std_x_mm=float(coordinates[:, 0].std()), repeatability_std_y_mm=float(coordinates[:, 1].std())))
    result['point_statistics'] = point_stats
    result['median_point_error_rmse_mm'] = float(np.sqrt(np.mean([value['median_error_mm']**2 for value in point_stats])))
    return result


def distance_checks(points, pitch_mm):
    results = []
    for first, second in combinations(points, 2):
        if first['row'] != second['row'] and first['col'] != second['col']:
            continue
        intervals = abs(first['row']-second['row'])+abs(first['col']-second['col'])
        measured = np.linalg.norm(np.array([first['measured_x_mm'], first['measured_y_mm']])-
                                  [second['measured_x_mm'], second['measured_y_mm']])
        results.append(dict(intervals=intervals, expected_mm=intervals*pitch_mm, measured_mm=float(measured),
                            error_mm=float(measured-intervals*pitch_mm)))
    return results


def angle_checks(points):
    found = {(point['col'], point['row']): np.array([point['measured_x_mm'], point['measured_y_mm']]) for point in points}
    results = []
    for (col, row), origin in found.items():
        if (col+1, row) not in found or (col, row+1) not in found:
            continue
        horizontal = found[col+1, row]-origin
        vertical = found[col, row+1]-origin
        cosine = np.clip((horizontal @ vertical)/(np.linalg.norm(horizontal)*np.linalg.norm(vertical)), -1, 1)
        angle = float(np.degrees(np.arccos(cosine)))
        results.append(dict(col=col, row=row, angle_deg=angle, error_deg=angle-90))
    return results


def draw_accuracy_plots(output, summaries, first_frame, first_points, image_to_board):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(2, 2, figsize=(12, 10), layout='constrained')
    maximum = max(value['position_p95_mm'] for value in summaries.values())
    for axis, (case, summary) in zip(axes.ravel(), summaries.items()):
        values = summary['point_statistics']
        x = np.array([value['expected_mm'][0] for value in values])
        y = np.array([value['expected_mm'][1] for value in values])
        dx = np.array([value['median_error_x_mm'] for value in values])
        dy = np.array([value['median_error_y_mm'] for value in values])
        scatter = axis.scatter(x, y, c=np.hypot(dx, dy), cmap='magma', vmin=0, vmax=maximum, s=45)
        axis.quiver(x, y, dx*10, dy*10, angles='xy', scale_units='xy', scale=1, width=.004)
        axis.set(title=f'IDs {case} / position RMS {summary["position_rmse_mm"]:.2f} mm',
                 xlabel='Grid x [mm]', ylabel='Grid y [mm]', xlim=(0, 360), ylim=(360, 0), aspect='equal')
        axis.set_xticks(range(0, 361, 90)); axis.set_yticks(range(0, 361, 90)); axis.grid(alpha=.25)
        figure.colorbar(scatter, ax=axis, label='Median position error [mm]')
    figure.suptitle('Independent 45 mm grid checks / median vectors enlarged 10x')
    figure.savefig(Path(output)/'position_error_map.png', dpi=160)
    plt.close(figure)
    overlay = first_frame.copy()
    inverse = np.linalg.inv(image_to_board)
    for point in first_points:
        measured = tuple(np.rint([point['image_x_px'], point['image_y_px']]).astype(int))
        predicted = tuple(np.rint(transform([[point['expected_x_mm'], point['expected_y_mm']]], inverse)[0]).astype(int))
        cv2.circle(overlay, measured, 3, (0, 255, 0), 1)
        cv2.drawMarker(overlay, predicted, (0, 0, 255), cv2.MARKER_TILTED_CROSS, 7, 1)
        cv2.line(overlay, measured, predicted, (0, 180, 255), 1)
        cv2.putText(overlay, f'{point["col"]},{point["row"]}', (measured[0]+4, measured[1]-3), cv2.FONT_HERSHEY_SIMPLEX, .3, (255, 255, 255), 1)
    cv2.imwrite(str(Path(output)/'independent_checkpoints.png'), overlay)


def write_csv(path, records):
    keys = list(dict.fromkeys(key for record in records for key in record))
    with Path(path).open('w', encoding='utf-8', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        writer.writerows(records)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', required=True, type=Path)
    parser.add_argument('--reference', required=True, type=Path)
    parser.add_argument('--roi-settings', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--pitch-mm', type=float, default=45)
    parser.add_argument('--frames', type=int, default=12)
    args = parser.parse_args()
    if args.frames < 1 or not np.isfinite(args.pitch_mm) or args.pitch_mm <= 0:
        parser.error('frames must be positive and pitch-mm must be finite and positive')
    args.output.mkdir(parents=True, exist_ok=False)
    capture = json.loads(args.capture.read_text(encoding='utf-8'))
    profile = json.loads(args.reference.read_text(encoding='utf-8'))
    rois = json.loads(args.roi_settings.read_text(encoding='utf-8'))['rois']
    cases = [tuple(sorted(map(int, profile['markers_px'])))]
    if len(cases[0]) > 2:
        cases.extend(combinations(cases[0], 2))
    points, distances, detections, frames, angles = [], [], [], [], []
    main_registrations = []
    first_frame = first_points = first_matrix = None
    training_points = {tuple(point) for point in profile.get('grid_calibration', {}).get('training_grid_indices', [])}
    dynamic_grid = profile.get('grid_refinement', {}).get('enabled', False)
    training_points.update(tuple(point) for point in profile.get('grid_refinement', {}).get('training_grid_indices', []))
    samples = [read_frame(capture['video_file'], float(stamp)) for stamp in np.linspace(1.0, 2.8, args.frames)]
    for frame_index, (frame, pts) in enumerate(samples):
        markers = inspect_rois(frame, rois, refine_corners=profile.get('corner_refinement') == 'subpix_original_win3')
        correction = register_marker_reference(markers, profile)
        if dynamic_grid:
            correction = refine_board_grid(frame, correction, profile)
        main_registrations.append(dict(sample=frame_index, accepted=correction['available'],
            used_marker_ids=correction.get('used_marker_ids', []), reason=correction.get('reason', ''),
            grid_quality=correction.get('grid_quality')))
        if not correction['available']:
            frames.extend(dict(sample=frame_index, pts=pts, case=','.join(map(str, ids)), accepted=False,
                               reason='main_registration_' + correction['reason']) for ids in cases)
            continue
        checks = find_grid_checkpoints(frame, correction['homography_image_to_board_mm'], args.pitch_mm)
        detections.extend(dict(point, sample=frame_index, pts=pts)
                          for point in heldout_checkpoints(checks, correction, training_points))
        for ids in cases:
            subset = {key: value for key, value in markers.items() if value['ids'] and value['ids'][0] in ids}
            result = register_marker_reference(subset, profile)
            if dynamic_grid:
                result = refine_board_grid(frame, result, profile)
            if not result['available'] or set(result['used_marker_ids']) != set(ids):
                frames.append(dict(sample=frame_index, pts=pts, case=','.join(map(str, ids)), accepted=False,
                                   reason=result.get('reason', 'requested_marker_set_unavailable')))
                continue
            measured = evaluate_checkpoints(heldout_checkpoints(checks, result, training_points),
                                            result['homography_image_to_board_mm'])
            label = ','.join(map(str, ids))
            points.extend(dict(point, sample=frame_index, pts=pts, case=label) for point in measured)
            distances.extend(dict(value, sample=frame_index, pts=pts, case=label) for value in distance_checks(measured, args.pitch_mm))
            angles.extend(dict(value, sample=frame_index, pts=pts, case=label) for value in angle_checks(measured))
            frames.append(dict(sample=frame_index, pts=pts, case=label, accepted=True, checkpoints=len(measured)))
            if first_frame is None and ids == cases[0]:
                first_frame, first_points, first_matrix = frame, measured, np.asarray(result['homography_image_to_board_mm'])
        print(f'frame {frame_index+1}/{args.frames}: {sum(p["accepted"] for p in checks)}/49 checkpoints', flush=True)
        if frame_index == 0:
            cv2.imwrite(str(args.output/'source_frame.png'), frame)
    summaries = {}
    for ids in cases:
        label = ','.join(map(str, ids))
        selected = [p for p in points if p['case'] == label]
        if selected:
            summaries[label] = summarize(selected, [p for p in distances if p['case'] == label])
            summaries[label]['accepted_frames'] = sum(bool(p['accepted']) for p in frames if p.get('case') == label)
            summaries[label]['sampled_frames'] = args.frames
            selected_angles = np.array([p['error_deg'] for p in angles if p['case'] == label])
            summaries[label]['orthogonality_rmse_deg'] = float(np.sqrt(np.mean(selected_angles**2))) if len(selected_angles) else None
            summaries[label]['orthogonality_max_abs_deg'] = float(abs(selected_angles).max()) if len(selected_angles) else None
    write_csv(args.output/'checkpoints.csv', points)
    write_csv(args.output/'distances.csv', distances)
    write_csv(args.output/'detection_quality.csv', detections)
    write_csv(args.output/'frames.csv', frames)
    write_csv(args.output/'angles.csv', angles)
    if first_frame is not None and summaries:
        draw_accuracy_plots(args.output, summaries, first_frame, first_points, first_matrix)
    all_label = ','.join(map(str, cases[0]))
    status = 'EVALUATED' if summaries.get(all_label, {}).get('unique_points', 0) >= 25 else 'INSUFFICIENT_CHECKPOINTS'
    report = dict(status=status, capture_json=str(args.capture.resolve()), reference_json=str(args.reference.resolve()),
                  reference_id=profile['reference_id'], pitch_mm=args.pitch_mm,
                  registration_mode='per_frame_grid' if dynamic_grid else 'per_frame_marker_registration',
                  pitch_source='user_supplied_pitch', physical_reference_uncertainty_mm=None,
                  evaluation='interior grid intersections not used to fit this reference or marker registration',
                  excluded_reference_training_points=sorted(training_points),
                  coordinate_origin='outer-grid TL, x to TR, y to BL', cases=summaries,
                  scope='board surface only; body marker height/parallax not validated',
                  acceptance_tolerance_mm=None, evaluated_grid_coordinate_range_mm=[args.pitch_mm, args.pitch_mm*7],
                  sampled_frames=args.frames, candidate_checkpoints_per_frame=49,
                  main_registration_frames=main_registrations,
                  input_sha256={str(path.resolve()): hashlib.sha256(path.read_bytes()).hexdigest()
                                for path in (args.capture, args.reference, args.roi_settings)},
                  implementation_sha256=IMPLEMENTATION_SHA256,
                  software_versions=dict(opencv=cv2.__version__, numpy=np.__version__))
    write_json(args.output/'reference_used.json', profile)
    write_json(args.output/'roi_settings_used.json', dict(rois=rois))
    write_json(args.output/'accuracy_report.json', report)
    print(json.dumps({case: {key: value for key, value in summary.items() if key not in {'point_statistics', 'distance_by_intervals'}} for case, summary in summaries.items()}, ensure_ascii=False), flush=True)
    return 0 if status == 'EVALUATED' else 2


if __name__ == '__main__':
    raise SystemExit(main())
