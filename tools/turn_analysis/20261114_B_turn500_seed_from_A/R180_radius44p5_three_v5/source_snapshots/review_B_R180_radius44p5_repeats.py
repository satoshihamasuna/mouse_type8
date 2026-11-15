"""Review saved B radius44.5 repetitions with the common v5 detector; no device access."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from compare_B_R180_radius import read_trial

ROOT=Path(__file__).resolve().parents[3]
SEED=Path(__file__).resolve().parent
RUNS=[
    ('radius44p5_01','tuning_cli_B_r44p5_20261114_prepared/run_B_R180_radius44p5_run01'),
    ('radius44p5_02','tuning_cli_B_r44p5_repeat02_20261114_reconnect/run_B_R180_radius44p5_run02'),
]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--include-third',action='store_true')
    args=parser.parse_args()
    runs=list(RUNS)
    if args.include_third:
        runs.append(('radius44p5_03','tuning_cli_B_r44p5_repeat03_20261114_prepared/run_B_R180_radius44p5_run03'))
    entries={}
    figure,axes=plt.subplots(1,2,figsize=(10,4.5),layout='constrained')
    for name,relative in runs:
        entry,xy,valid=read_trial(ROOT/'tools/logs/pixel_camera'/relative/'full')
        entries[name]=entry
        observed=xy.copy();observed[~valid]=np.nan
        axes[0].plot(*observed.T,label=name)
        s=entry['relative_summary']
        axes[1].plot(s['endpoint_right_mm'],s['endpoint_forward_mm'],'o',label=name)
    first=entries['radius44p5_01']
    assert first['settings']['parameters']['r_min']==-44.5
    for entry in entries.values():
        assert entry['marker_detection']==first['marker_detection']
        assert all(entry['settings'][key]==first['settings'][key]
                   for key in ('parameters','pid','feedforward','suction'))
    accepted=[entry for entry in entries.values() if entry['relative_summary']['auto_stop_confirmed']]
    lateral=np.array([e['relative_summary']['endpoint_right_mm'] for e in accepted])
    forward=np.array([e['relative_summary']['endpoint_forward_mm'] for e in accepted])
    exit_error=np.array([e['exit_travel_direction_error_deg'] for e in accepted])
    enough=len(accepted)>=3
    relative_guidance_passed=bool(enough and abs(lateral.mean()-90)<=1 and abs(lateral-90).max()<=2
                                  and abs(exit_error).max()<=1)
    result=dict(mouse='B',motion='long_r180',r_min_mm=-44.5,preset=500,
        entries=entries,capture_count=len(entries),valid_count=len(accepted),same_colour_detector=True,
        summary=dict(mean_right_mm=float(lateral.mean()) if len(lateral) else None,
            median_right_mm=float(np.median(lateral)) if len(lateral) else None,
            range_right_mm=[float(lateral.min()),float(lateral.max())] if len(lateral) else None,
            sample_std_right_mm=float(lateral.std(ddof=1)) if len(lateral)>1 else None,
            individual_target_errors_mm=(lateral-90).tolist(),
            maximum_abs_target_error_mm=float(abs(lateral-90).max()) if len(lateral) else None,
            range_forward_mm=[float(forward.min()),float(forward.max())] if len(forward) else None,
            maximum_abs_exit_travel_error_deg=float(abs(exit_error).max()) if len(exit_error) else None),
        three_valid_trials_complete=enough,relative_guidance_passed=relative_guidance_passed,
        relative_guidance=dict(minimum_valid_trials=3,maximum_abs_mean_lateral_error_mm=1,
            maximum_abs_individual_lateral_error_mm=2,maximum_abs_exit_travel_direction_error_deg=1),
        parameter_adoption_allowed=False,firmware_parameters_modified=False,
        limitations=['relative marker midpoint, not absolute axle position','camera height approximately 1500 mm',
            'no independent hardware synchronization or fresh wall EKF mode readback'],
        next_step='Review for provisional single-turn candidate and next left-turn measurement' if relative_guidance_passed
            else 'Retain settings and collect the remaining valid single-turn repetitions')
    output=SEED/('R180_radius44p5_three_v5' if args.include_third else 'R180_radius44p5_two_v5')
    output.mkdir(exist_ok=False)
    with (output/'review.json').open('x',encoding='utf-8') as stream:
        json.dump(result,stream,ensure_ascii=False,indent=2)
    for axis in axes:
        axis.axvline(90,color='black',linestyle='--',label='90 mm target')
        axis.set(xlabel='Right [mm]',ylabel='Forward [mm]')
        axis.set_aspect('equal',adjustable='box')
        axis.legend(fontsize=8)
    axes[1].set(xlim=(88,93),ylim=(-2.5,2.5),title='Stopped marker midpoint')
    figure.savefig(output/'trajectories.png',dpi=160)
    plt.close(figure)
    print(json.dumps(dict(valid_count=len(accepted),summary=result['summary'],
        measurements={name:dict(right=e['relative_summary']['endpoint_right_mm'],
            forward=e['relative_summary']['endpoint_forward_mm'],exit_deg=e['exit_travel_direction_error_deg'],
            observation=e['relative_summary']['motion_observation_rate'],accepted=e['relative_summary']['auto_stop_confirmed'])
            for name,e in entries.items()}),ensure_ascii=True))


if __name__=='__main__':
    main()
