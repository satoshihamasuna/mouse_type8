"""同一設定の保存済みB右90度を集計する。機器への接続なし。"""
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
    args = parser.parse_args()
    reviews = [json.loads(p.read_text(encoding='utf-8')) for p in args.review]
    assert len(reviews) >= 2
    keys = ('parameters', 'pid', 'feedforward', 'suction', 'preset_speed_mm_s', 'turn_count', 'motion_name')
    for item in reviews:
        assert item['status'] == 'VALID_SINGLE_TRIAL_REVIEW'
        assert item['summary']['frame_stride'] == 1
        assert all(item['settings'][key] == reviews[0]['settings'][key] for key in keys)
    endpoints = np.array([[r['summary']['endpoint_right_mm'], r['summary']['endpoint_forward_mm']]
                          for r in reviews])
    errors = endpoints-270
    exits = np.array([r['exit_travel_error_deg'] for r in reviews])
    mean = endpoints.mean(axis=0)
    result = dict(mouse='B', motion='long_r90', valid_trials=len(reviews),
        settings=reviews[0]['settings'], target_right_forward_mm=[270, 270],
        endpoints_right_forward_mm=endpoints.tolist(), endpoint_errors_right_forward_mm=errors.tolist(),
        mean_right_forward_mm=mean.tolist(), median_right_forward_mm=np.median(endpoints, axis=0).tolist(),
        mean_errors_right_forward_mm=(mean-270).tolist(),
        range_right_forward_mm=np.ptp(endpoints, axis=0).tolist(),
        sample_std_right_forward_mm=endpoints.std(axis=0, ddof=1).tolist(),
        max_abs_individual_errors_right_forward_mm=abs(errors).max(axis=0).tolist(),
        exit_travel_errors_deg=exits.tolist(), mean_exit_travel_error_deg=float(exits.mean()),
        exit_travel_error_sample_std_deg=float(exits.std(ddof=1)),
        maximum_abs_exit_travel_error_deg=float(abs(exits).max()),
        provisional_guidance=dict(minimum_trials=3, maximum_abs_mean_axis_error_mm=1,
            maximum_abs_individual_axis_error_mm=2, maximum_abs_exit_travel_error_deg=1,
            note='Exploratory relative-marker guidance, not absolute axle or continuous-run acceptance'),
        guidance_passed=bool(len(reviews) >= 3 and np.all(abs(mean-270) <= 1)
                             and np.all(abs(errors) <= 2) and np.all(abs(exits) <= 1)),
        parameter_adoption_allowed=False, firmware_parameters_modified=False,
        absolute_axle_position_verified=False, body_yaw_verified=False,
        independent_hardware_sync_verified=False,
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
    ax.plot(270, 270, 'kx', label='Target 270 / 270 mm')
    ax.set(xlabel='Right [mm]', ylabel='Forward [mm]', title='B R90 repeatability, 500 mm/s')
    ax.axis('equal')
    ax.legend(fontsize=8)
    fig.savefig(args.output/'trajectories.png', dpi=150)
    plt.close(fig)
    print(json.dumps({k: result[k] for k in ['valid_trials', 'mean_right_forward_mm',
        'range_right_forward_mm', 'exit_travel_errors_deg', 'guidance_passed']}, ensure_ascii=False))


if __name__ == '__main__':
    main()
