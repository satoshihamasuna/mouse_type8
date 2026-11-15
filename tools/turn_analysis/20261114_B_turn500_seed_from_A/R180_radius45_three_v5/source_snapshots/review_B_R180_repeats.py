"""Review saved B radius45 captures with the common v5 detector; no device access."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from compare_B_R180_radius import read_trial

ROOT = Path(__file__).resolve().parents[3]
SEED = Path(__file__).resolve().parent
RUNS = [
    ('baseline', 'tuning_cli_B_portable_20261114_164124/run_B_R180_baseline_run01'),
    ('radius45_01', 'tuning_cli_B_radius_20261114_172808/run_B_R180_A_radius_candidate_run01'),
    ('radius45_02', 'tuning_cli_B_radius_repeat02_20261114_checked/run_B_R180_radius_repeat02_checked'),
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--include-third', action='store_true')
    args = parser.parse_args()
    runs = list(RUNS)
    if args.include_third:
        runs.append(('radius45_03', 'tuning_cli_B_radius_repeat03_20261114_prepared/run_B_R180_radius_repeat03_checked'))
    output = SEED/('R180_radius45_three_v5' if args.include_third else 'R180_radius45_repeats_v5')
    output.mkdir(exist_ok=False)
    entries = {}
    figure, axis = plt.subplots(figsize=(6, 5), layout='constrained')
    for name, relative in runs:
        folder = ROOT/'tools/logs/pixel_camera'/relative/('full' if name == 'radius45_03' else 'full_common_v5')
        entry, xy, valid = read_trial(folder)
        entries[name] = entry
        points = xy.copy()
        points[~valid] = np.nan
        axis.plot(*points.T, label=name)
    baseline = entries['baseline']
    candidates = [entry for name,entry in entries.items() if name.startswith('radius45_')]
    assert all(e['settings']['parameters']['r_min'] == -45 for e in candidates)
    for key in ('parameters', 'pid', 'feedforward', 'suction'):
        assert all(candidates[0]['settings'][key] == entry['settings'][key] for entry in candidates[1:])
    assert all(e['marker_detection'] == baseline['marker_detection'] for e in candidates)
    valid_candidates = [e for e in candidates if e['relative_summary']['auto_stop_confirmed']]
    lateral = np.array([e['relative_summary']['endpoint_right_mm'] for e in valid_candidates])
    result = dict(mouse='B', motion='long_r180', preset=500, entries=entries,
        same_colour_detector=True, candidate_capture_count=len(candidates), candidate_valid_count=len(valid_candidates),
        candidate_summary=dict(mean_right_mm=float(lateral.mean()) if len(lateral) else None,
            median_right_mm=float(np.median(lateral)) if len(lateral) else None,
            range_right_mm=[float(lateral.min()), float(lateral.max())] if len(lateral) else None,
            individual_target_errors_mm=(lateral-90).tolist(),
            maximum_abs_target_error_mm=float(abs(lateral-90).max()) if len(lateral) else None,
            sample_std_right_mm=float(lateral.std(ddof=1)) if len(lateral)>1 else None),
        parameter_adoption_allowed=False, three_valid_candidate_trials_complete=len(valid_candidates)>=3,
        firmware_parameters_modified=False, camera_height_mm=1500, camera_height_is_approximate=True,
        physical_motion_executions_this_session=1,
        historical_interruption_before_execution=dict(request_id='B_R180_A_radius_candidate_run02',
            execution_write_attempts=0, record='tuning_cli_B_radius_repeat02_20261114_prepared/interrupted_before_execution.json'),
        condition_notes=['portable board moved in the camera image between captures; update actual ID 2/3 ROIs',
            'relative entry frame and marker midpoint, not absolute axle position',
            'no independent hardware synchronization or fresh wall EKF mode readback'],
        next_step='Review remaining lateral bias before a radius-only fine comparison' if len(valid_candidates)>=3
            else 'One more radius45 capture, retain B lengths/PID/FF; verify current static board before execution')
    if len(valid_candidates)>=3 and baseline['relative_summary']['auto_stop_confirmed']:
        sensitivity=(lateral.mean()-baseline['relative_summary']['endpoint_right_mm'])/2.5
        result['radius_fine_comparison_hypothesis']=dict(
            method='local linear hypothesis from one baseline and three radius45 trials; not identified physics',
            measured_lateral_mm_per_radius_mm=float(sensitivity),
            candidate_r_min_mm=-44.5,
            predicted_lateral_mm=float(lateral.mean()-.5*sensitivity),
            tested=False, only_parameter_to_change='r_min')
    with (output/'review.json').open('x',encoding='utf-8') as stream:
        json.dump(result,stream,ensure_ascii=False,indent=2)
    axis.axvline(90,color='gray',linestyle='--',label='90 mm target')
    axis.set(xlabel='Right [mm]',ylabel='Forward [mm]',title='B R180, common v5 detector; gaps are unobserved')
    axis.axis('equal')
    axis.legend(fontsize=8)
    figure.savefig(output/'trajectories.png',dpi=160)
    plt.close(figure)
    print(json.dumps(dict(valid_count=len(valid_candidates), summary=result['candidate_summary'],
        measurements={name:dict(right=e['relative_summary']['endpoint_right_mm'],
            forward=e['relative_summary']['endpoint_forward_mm'],
            exit_deg=e['exit_travel_direction_error_deg'],
            observation=e['relative_summary']['motion_observation_rate'],
            accepted=e['relative_summary']['auto_stop_confirmed'],span=e['marker_span_range_mm'])
            for name,e in entries.items()}),ensure_ascii=True))


if __name__ == '__main__':
    main()
