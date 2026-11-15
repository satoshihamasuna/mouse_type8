"""同一設定の右135度入口を集計する。機器への接続なし。"""
import argparse
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--review', type=Path, action='append', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--mouse', choices=('A', 'B'), default='B')
    parser.add_argument('--camera-comparison', type=Path)
    args = parser.parse_args()
    reviews = [json.loads(p.read_text(encoding='utf-8')) for p in args.review]
    assert len(reviews) >= 2
    assert len({item['summary']['trial_json'] for item in reviews}) == len(reviews), 'Duplicate underlying trial'
    setup_reports = {}
    if args.mouse == 'B':
        for path in args.review:
            dense = path.parent/'final_validity.json'
            sparse = path.parent/'recorded_start_pose_check.json'
            gate = dense if dense.exists() else sparse
            setup = json.loads(gate.read_text(encoding='utf-8'))
            if gate == dense:
                assert setup['full_trajectory_review_passed']
                if not setup['dense_static_confirmation_passed']:
                    assert setup.get('independent_body_setup_confirmation_passed')
                    body_path = Path(setup['body_setup_report'])
                    body = json.loads(body_path.read_text(encoding='utf-8'))
                    static_path = Path(setup['dense_static_report'])
                    static = json.loads(static_path.read_text(encoding='utf-8'))
                    assert static['frames'] >= 280 and static['detection_and_calibration_rate'] >= .99
                    assert static['stationary_scatter_rms_mm'] <= .5 and static['maximum_stationary_deviation_mm'] <= 1.5
                    assert static['centre_change_since_dense_static_mm'] <= 3
                    assert body['state'] == 'PASSED' and body['correlation'] >= .85
                    assert abs(body['relative_rotation_deg']) <= 1
                    assert np.linalg.norm(body['centre_translation_mm']) <= 3
                    assert body['protocol']['mask_ellipse_half_axes_mm'] == [10, 20]
                    assert max(s['error_deg'] for s in body['synthetic_validation']) <= .15
                    mask_path = Path(setup['body_mask_review'])
                    assert json.loads(mask_path.read_text(encoding='utf-8'))['state'] == 'PASSED'
                    for report in (body_path, static_path, mask_path):
                        setup_reports[str(report)] = hashlib.sha256(report.read_bytes()).hexdigest()
            else:
                assert setup['state'] == 'PASSED'
            setup_reports[str(gate)] = hashlib.sha256(gate.read_bytes()).hexdigest()

    reference_ids = []
    camera_comparison = None
    if args.mouse == 'B':
        for item in reviews:
            report = json.loads(Path(item['summary']['report_json']).read_text(encoding='utf-8'))
            reference_ids.append(report['quality']['marker_calibration']['reference_id'])
        if len(set(reference_ids)) > 1:
            assert args.camera_comparison, 'Mixed camera conditions require explicit static comparison'
            camera_comparison = json.loads(args.camera_comparison.read_text(encoding='utf-8'))
            assert camera_comparison['state'] == 'PASSED_INDEPENDENT_STATIC_VALIDATION'
            check = camera_comparison['comparison_to_1x']
            assert check['midpoint_difference_mm'] <= .75 and abs(check['apparent_heading_difference_deg']) <= 1
            for path, expected in camera_comparison['input_sha256'].items():
                assert hashlib.sha256(Path(path).read_bytes()).hexdigest() == expected, path
            setup_reports[str(args.camera_comparison)] = hashlib.sha256(args.camera_comparison.read_bytes()).hexdigest()

    keys = ('parameters', 'pid', 'feedforward', 'suction', 'preset_speed_mm_s', 'turn_count', 'motion_name')
    for item in reviews:
        assert item['mouse'] == args.mouse and item['motion'] == 'in_r135'
        assert item['target_right_forward_mm'] == [180, 45]
        assert item['status'] == 'VALID_SINGLE_TRIAL_REVIEW' and item['summary']['frame_stride'] == 1
        assert all(item['settings'][key] == reviews[0]['settings'][key] for key in keys)
    endpoints = np.array([[r['summary']['endpoint_right_mm'], r['summary']['endpoint_forward_mm']]
                          for r in reviews])
    errors = endpoints-np.array([180, 45])
    exits = np.array([r['exit_travel_error_deg'] for r in reviews])
    mean = endpoints.mean(axis=0)
    result = dict(mouse=args.mouse, motion='in_r135', valid_trials=len(reviews),
        settings=reviews[0]['settings'], target_right_forward_mm=[180, 45],
        endpoints_right_forward_mm=endpoints.tolist(), endpoint_errors_right_forward_mm=errors.tolist(),
        mean_right_forward_mm=mean.tolist(), median_right_forward_mm=np.median(endpoints, axis=0).tolist(),
        mean_errors_right_forward_mm=(mean-np.array([180, 45])).tolist(),
        range_right_forward_mm=np.ptp(endpoints, axis=0).tolist(),
        sample_std_right_forward_mm=endpoints.std(axis=0, ddof=1).tolist(),
        max_abs_individual_errors_right_forward_mm=abs(errors).max(axis=0).tolist(),
        exit_travel_errors_deg=exits.tolist(), mean_exit_travel_error_deg=float(exits.mean()),
        exit_travel_error_sample_std_deg=float(exits.std(ddof=1)),
        maximum_abs_exit_travel_error_deg=float(abs(exits).max()),
        provisional_guidance=dict(minimum_trials=3, maximum_abs_mean_axis_error_mm=1,
            maximum_abs_individual_axis_error_mm=2, maximum_abs_exit_travel_error_deg=1,
            note='Exploratory relative-marker guidance; no absolute axle or continuous-run acceptance'),
        guidance_passed=bool(len(reviews) >= 3 and np.all(abs(errors.mean(axis=0)) <= 1)
                             and np.all(abs(errors) <= 2) and np.all(abs(exits) <= 1)),
        parameter_adoption_allowed=False, firmware_parameters_modified=False,
        absolute_axle_position_verified=False, body_yaw_verified=False,
        independent_hardware_sync_verified=False,
        acquisition_reference_ids=reference_ids, mixed_camera_conditions=len(set(reference_ids)) > 1,
        camera_comparison=str(args.camera_comparison) if args.camera_comparison else None,
        acquisition_scope='Mixed camera trials retain explicit static comparison; no absolute or continuous-run guarantee',
        setup_gate_reports=setup_reports,
        reviews=[str(p) for p in args.review],
        input_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                      for p in [*args.review, Path(__file__)]})
    args.output.mkdir(exist_ok=False)
    (args.output/'review.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    fig, ax = plt.subplots(figsize=(6, 5), layout='constrained')
    for index, review in enumerate(reviews):
        folder = Path(review['summary']['report_json']).parent
        trajectory = pd.read_csv(folder/'normalized_trajectory.csv')
        valid = trajectory.detected.to_numpy(bool)
        xy = trajectory[['right_mm', 'forward_mm']].to_numpy()
        ax.plot(*xy[valid].T, label=f'Trial {index+1}')
    ax.plot(180, 45, 'kx', label='Target 180 / 45 mm')
    ax.set(xlabel='Right [mm]', ylabel='Forward [mm]', title=f'{args.mouse} in_R135 repeatability, 500 mm/s')
    ax.axis('equal')
    ax.legend(fontsize=8)
    fig.savefig(args.output/'trajectories.png', dpi=150)
    plt.close(fig)
    print(json.dumps({k: result[k] for k in ['valid_trials', 'mean_right_forward_mm',
        'range_right_forward_mm', 'exit_travel_errors_deg', 'guidance_passed']}, ensure_ascii=False))


if __name__ == '__main__':
    main()
