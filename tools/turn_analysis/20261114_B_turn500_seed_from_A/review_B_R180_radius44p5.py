"""Compare the saved first radius44.5 trial with the three radius45 trials."""
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from compare_B_R180_radius import read_trial

ROOT = Path(__file__).resolve().parents[3]
SEED = Path(__file__).resolve().parent


def main():
    previous_path = SEED/'R180_radius45_three_v5/review.json'
    previous = json.loads(previous_path.read_text(encoding='utf-8'))
    folder = ROOT/'tools/logs/pixel_camera/tuning_cli_B_r44p5_20261114_prepared/run_B_R180_radius44p5_run01/full'
    current, xy, valid = read_trial(folder)
    prior = previous['entries']['radius45_03']
    assert previous['candidate_valid_count'] == 3
    assert current['relative_summary']['auto_stop_confirmed']
    assert current['marker_detection'] == prior['marker_detection']
    changes = {key:[value,current['settings']['parameters'][key]]
               for key,value in prior['settings']['parameters'].items()
               if value != current['settings']['parameters'][key]}
    assert changes == {'r_min':[-45.0,-44.5]}
    assert all(current['settings'][key] == prior['settings'][key] for key in ('pid','feedforward','suction'))
    result = dict(mouse='B',motion='long_r180',status='ONE_VALID_RADIUS44P5_TRIAL_NOT_ADOPTED',
        previous_three_trial_review=str(previous_path), current=current, settings_changes=changes,
        lateral_change_from_radius45_mean_mm=current['relative_summary']['endpoint_right_mm']
            -previous['candidate_summary']['mean_right_mm'],
        valid_radius44p5_trials=1, parameter_adoption_allowed=False,
        relative_trial_review_guidance=dict(minimum_valid_trials=3,maximum_abs_mean_lateral_error_mm=1,
            maximum_abs_individual_lateral_error_mm=2,maximum_abs_exit_travel_direction_error_deg=1,
            scope='Provisional relative marker midpoint guidance, not absolute axle or continuous-run acceptance'),
        next_step='Repeat the same radius44.5 settings to assess three-trial scatter')
    output=SEED/'R180_radius44p5_run01_review'
    output.mkdir(exist_ok=False)
    with (output/'review.json').open('x',encoding='utf-8') as stream:
        json.dump(result,stream,ensure_ascii=False,indent=2)
    figure,axes=plt.subplots(1,2,figsize=(10,4.5),layout='constrained')
    for i,name in enumerate(('radius45_01','radius45_02','radius45_03')):
        entry=previous['entries'][name]
        trajectory=pd.read_csv(Path(entry['analysis_folder'])/'normalized_trajectory.csv')
        points=trajectory[['right_mm','forward_mm']].to_numpy()
        points[~trajectory.detected.to_numpy(bool)]=np.nan
        axes[0].plot(*points.T,color='gray',alpha=.5,label='Radius45, three trials' if i==0 else None)
        summary=entry['relative_summary']
        axes[1].plot(summary['endpoint_right_mm'],summary['endpoint_forward_mm'],'o',color='gray')
    points=xy.copy();points[~valid]=np.nan
    axes[0].plot(*points.T,label='Radius44.5, first trial')
    summary=current['relative_summary']
    axes[1].plot(summary['endpoint_right_mm'],summary['endpoint_forward_mm'],'o',label='Radius44.5')
    for axis in axes:
        axis.axvline(90,color='black',linestyle='--',label='90 mm target')
        axis.set(xlabel='Right [mm]',ylabel='Forward [mm]')
        axis.set_aspect('equal',adjustable='box')
    axes[0].legend(fontsize=8)
    axes[1].set(xlim=(89,93),ylim=(-2,2),title='Stopped marker midpoint')
    axes[1].legend(fontsize=8)
    figure.savefig(output/'comparison.png',dpi=160)
    plt.close(figure)
    print(json.dumps(dict(right_mm=summary['endpoint_right_mm'],forward_mm=summary['endpoint_forward_mm'],
        exit_deg=current['exit_travel_direction_error_deg'],observation=summary['motion_observation_rate'],
        marker_span=current['marker_span_range_mm'],lateral_delta=result['lateral_change_from_radius45_mean_mm'],
        minimum_battery=current['minimum_battery_V']),ensure_ascii=True))


if __name__ == '__main__':
    main()
