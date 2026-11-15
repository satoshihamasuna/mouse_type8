"""全フレーム確認済みB右135度入口の、半径と入口長の組合せ候補の比較。機器操作なし。"""
import argparse
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import pandas as pd

from compare_inR135_control_v2 import read


def load(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--candidate', type=Path, required=True)
    parser.add_argument('--baseline-review', type=Path, required=True)
    parser.add_argument('--candidate-review', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    baseline, bt, bi, ba = read(args.baseline)
    candidate, ct, ci, ca = read(args.candidate)
    differences = {group+'.'+key: [value, candidate['settings'][group][key]]
        for group in ('parameters', 'pid', 'feedforward', 'suction')
        for key, value in baseline['settings'][group].items()
        if value != candidate['settings'][group][key]}
    assert set(differences) == {'parameters.r_min', 'parameters.lstart'}, differences
    reviews = [load(args.baseline_review), load(args.candidate_review)]
    for item, actual in zip(reviews, [baseline, candidate]):
        assert item['mouse'] == 'B' and item['motion'] == 'in_r135'
        assert item['valid_trials'] == 1 and item['summary']['auto_stop_confirmed']
        for group in ('parameters', 'pid', 'feedforward', 'suction'):
            assert item['settings'][group] == actual['settings'][group]
    result = dict(mouse='B', motion='in_r135', state='TWO_VALID_SINGLE_TRIAL_SHAPE_COMPARISON',
        differences=differences, baseline=baseline, candidate=candidate,
        baseline_review=str(args.baseline_review), candidate_review=str(args.candidate_review),
        endpoints_right_forward_mm=[[item['summary']['endpoint_right_mm'],
            item['summary']['endpoint_forward_mm']] for item in reviews],
        exit_travel_errors_deg=[item['exit_travel_error_deg'] for item in reviews],
        battery_minimum_V=[item['minimum_battery_V'] for item in reviews],
        video_full_review_complete=True, repeatability_verified=False, parameter_adoption_allowed=False,
        windows='Approximate ideal omega boundary plus commanded length durations; sample RMS, actual MCU time_ms',
        note='Coupled radius/Lstart candidate; individual effects are not isolated. One trial per condition; battery differs.',
        input_sha256={str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in
            [args.baseline_review, args.candidate_review, Path(__file__),
             Path(__file__).with_name('compare_inR135_control_v2.py')]})
    args.output.mkdir(exist_ok=False)
    (args.output/'comparison.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    fig, axes = plt.subplots(1, 2, figsize=(11, 4), layout='constrained')
    for item, label in zip(reviews, ['Baseline', 'Candidate']):
        pose = pd.read_csv(Path(item['summary']['report_json']).parent/'normalized_trajectory.csv')
        good = pose.detected
        axes[0].plot(pose.right_mm[good], pose.forward_mm[good], label=label)
    axes[0].plot(180, 45, 'x', label='Target 180 / 45 mm')
    axes[0].set(xlabel='Right [mm]', ylabel='Forward [mm]', title='B in_R135 relative marker midpoint')
    axes[0].axis('equal')
    axes[1].plot(bt, bi, '--', label='Baseline target')
    axes[1].plot(bt, ba, label='Baseline measured')
    axes[1].plot(ct, ci, '--', label='Candidate target')
    axes[1].plot(ct, ca, label='Candidate measured')
    axes[1].set(xlim=(-.12, .40), xlabel='Time from angular turn start [s]',
        ylabel='Angular velocity [rad/s]', title='True MCU time, each turn start aligned')
    for axis in axes:
        axis.grid(alpha=.2)
        axis.legend(fontsize=8)
    fig.savefig(args.output/'trajectories_and_control.png', dpi=150)
    plt.close(fig)
    print(json.dumps({key: result[key] for key in ['differences', 'endpoints_right_forward_mm',
        'exit_travel_errors_deg', 'battery_minimum_V']}, ensure_ascii=False))


if __name__ == '__main__':
    main()
