"""登録済みの盤面座標で、走行前の静止位置を測定する。"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from video_marker_pose import board_to_pose_transform, read_video_pose, transform_points
from pixel_camera_detection import resolve_geometry_detection


def wrap_degrees(value):
    return (value + 180) % 360 - 180


def measure(video, reference, geometry, placement):
    geometry = resolve_geometry_detection(geometry)
    pose, _, _, _ = read_video_pose(video, geometry['front_colour'],
        marker_reference=reference, video_window=(1., 2.95), frame_stride=4,
        marker_detection=geometry.get('marker_detection'), rear_colour=geometry.get('rear_colour', 'yellow'),
        nominal_marker_span_mm=geometry.get('nominal_marker_span_mm'),
        marker_span_limits_mm=geometry.get('marker_span_limits_mm'))
    valid = pose.marker_detected_raw.to_numpy(bool) & pose.calibration_valid.to_numpy(bool)
    tolerance = placement['heading_tolerance_deg']
    stability_limits = dict(rms=tolerance/4, half_window=tolerance/4, maximum_median_deviation=tolerance)
    result = dict(passed=False, frames=len(pose), observed=int(valid.sum()),
                  detector=pose.attrs.get('body_detection',{}),
                  calibration=pose.attrs['marker_calibration'], placement=placement,
                  acceptance_version=5, heading_statistic='circular_median',
                  heading_reference='apparent_marker_pair_on_board; body_yaw_uncalibrated',
                  heading_acceptance_basis='circular_median_target_error_with_separate_stability_guards',
                  heading_target_max_limit_deg=tolerance,
                  raw_heading_tolerance_is_diagnostic_only=True,
                  heading_budget_rms_multiplier=2., heading_budget_limit_deg=tolerance,
                  heading_budget_is_diagnostic_only=True,
                  heading_budget_is_statistical_guarantee=False,
                  heading_budget_deg=None, max_heading_error_deg=None,
                  all_raw_heading_observations_within_target_tolerance=None,
                  heading_stability_limits_deg=stability_limits,
                  heading_jitter_max_is_diagnostic_only=False,
                  acceptance_reason='開始角の円周median目標差<=tolと、最大median差<=tol・RMS<=tol/4・半窓差<=tol/4の静止安定度を分けて判定する。2RMS予算と全raw目標内は診断のみで、判定に使わない。以後この固定条件を使用する。',
                  failure_reasons=[])
    if valid.sum() < 12:
        result['failure_reasons'].append('insufficient_observations')
    if len(valid) == 0 or valid.mean() < .95:
        result['failure_reasons'].append('insufficient_observation_fraction')
    if not valid.any():
        return result
    board_transform = np.linalg.inv(board_to_pose_transform(reference))
    front = transform_points(pose.loc[valid, ['blue_x_mm', 'blue_y_mm']].to_numpy(), board_transform)
    rear = transform_points(pose.loc[valid, ['yellow_x_mm', 'yellow_y_mm']].to_numpy(), board_transform)
    midpoint = (front + rear) / 2
    heading = np.degrees(np.arctan2((front - rear)[:, 1], (front - rear)[:, 0]))
    heading_error = wrap_degrees(heading - placement['heading_deg'])
    # +/-180度をまたぐ向きも、反対向きを0度と誤認せず代表値を求める。
    anchor = np.degrees(np.angle(np.mean(np.exp(1j * np.radians(heading)))))
    representative = float((anchor + np.median(wrap_degrees(heading-anchor))) % 360)
    representative_error = float(abs(wrap_degrees(representative-placement['heading_deg'])))
    jitter = wrap_degrees(heading-representative)
    jitter_rms = float(np.sqrt(np.mean(jitter**2)))
    jitter_max = float(np.abs(jitter).max())
    heading_budget = representative_error+2*jitter_rms
    half = len(jitter)//2
    drift = float(abs(np.median(jitter[-half:])-np.median(jitter[:half]))) if half else 0.
    target = [placement['x_mm'], placement['y_mm']]
    position_error = float(np.linalg.norm(midpoint - target, axis=1).max())
    angle_error = float(np.abs(heading_error).max())
    checks = {
        'position_tolerance_exceeded': position_error <= placement['position_tolerance_mm'],
        'heading_tolerance_exceeded': representative_error <= tolerance,
        'heading_gross_outlier_exceeded': jitter_max <= stability_limits['maximum_median_deviation'],
        'heading_jitter_rms_exceeded': jitter_rms <= stability_limits['rms'],
        'heading_drift_exceeded': drift <= stability_limits['half_window'],
    }
    result['failure_reasons'].extend(reason for reason, passed in checks.items() if not passed)
    result.update(center_board_mm=np.median(midpoint, axis=0).tolist(),
        heading_deg=representative, representative_heading_error_deg=representative_error,
        heading_budget_deg=heading_budget,
        max_position_error_mm=position_error, max_heading_error_deg=angle_error,
        all_raw_heading_observations_within_target_tolerance=angle_error <= tolerance,
        heading_jitter_rms_deg=jitter_rms, heading_jitter_max_deg=jitter_max,
        heading_half_window_difference_deg=drift, heading_stability_limits_deg=stability_limits,
        observed_fraction=float(valid.mean()),
        passed=not result['failure_reasons'])
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('plan', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    plan = json.loads(args.plan.read_text(encoding='utf-8'))
    reference = json.loads(Path(plan['reference']).read_text(encoding='utf-8'))
    geometry = json.loads(Path(plan['geometry']).read_text(encoding='utf-8'))
    result = measure(Path(plan['video']), reference, geometry, plan['placement'])
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)


if __name__ == '__main__':
    main()
