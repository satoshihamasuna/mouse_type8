"""Review the B baseline and first two radius45 captures with the common v5 detector."""
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
    output = SEED/'R180_radius45_repeats_v5'
    output.mkdir(exist_ok=False)
    entries = {}
    figure, axis = plt.subplots(figsize=(6, 5), layout='constrained')
    for name, relative in RUNS:
        folder = ROOT/'tools/logs/pixel_camera'/relative/'full_common_v5'
        entry, xy, valid = read_trial(folder)
        entries[name] = entry
        points = xy.copy()
        points[~valid] = np.nan
        axis.plot(*points.T, label=name)
    baseline = entries['baseline']
    candidates = [entries['radius45_01'], entries['radius45_02']]
    assert all(e['settings']['parameters']['r_min'] == -45 for e in candidates)
    for key in ('parameters', 'pid', 'feedforward', 'suction'):
        assert candidates[0]['settings'][key] == candidates[1]['settings'][key]
    assert all(e['marker_detection'] == baseline['marker_detection'] for e in candidates)
    valid_candidates = [e for e in candidates if e['relative_summary']['auto_stop_confirmed']]
    lateral = np.array([e['relative_summary']['endpoint_right_mm'] for e in valid_candidates])
    result = dict(mouse='B', motion='long_r180', preset=500, entries=entries,
        same_colour_detector=True, candidate_capture_count=2, candidate_valid_count=len(valid_candidates),
        candidate_summary=dict(mean_right_mm=float(lateral.mean()) if len(lateral) else None,
            range_right_mm=[float(lateral.min()), float(lateral.max())] if len(lateral) else None,
            sample_std_right_mm=float(lateral.std(ddof=1)) if len(lateral)>1 else None),
        parameter_adoption_allowed=False, three_valid_candidate_trials_complete=False,
        firmware_parameters_modified=False, camera_height_mm=1500, camera_height_is_approximate=True,
        physical_motion_executions_this_session=1,
        interrupted_before_execution=dict(request_id='B_R180_A_radius_candidate_run02',
            execution_write_attempts=0, record='tuning_cli_B_radius_repeat02_20261114_prepared/interrupted_before_execution.json'),
        condition_notes=['portable board moved in the camera image between captures; update actual ID 2/3 ROIs',
            'relative entry frame and marker midpoint, not absolute axle position',
            'no independent hardware synchronization or fresh wall EKF mode readback'],
        next_step='One more radius45 capture, retain B lengths/PID/FF; verify current static board before execution')
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
