"""保存済みB右135度入口の制御応答を比較する。機器への接続なし。"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'tools/analysis'))
from video_marker_pose import log_elapsed_seconds


def read(folder):
    trial_path = folder/'trial.json'
    trial = json.loads(trial_path.read_text(encoding='utf-8'))
    settings_path = Path(trial['type8_settings_json'])
    settings = json.loads(settings_path.read_text(encoding='utf-8'))
    assert trial['mouse'] == settings['mouse'] == 'B'
    assert settings['motion_name'] == 'in_r135' and settings['preset_speed_mm_s'] == 500
    assert settings['turn_count'] == 1 and trial['log_quality']['full_motion_retained']
    csv = Path(trial['type8_log_csv'])
    log = pd.read_csv(csv)
    time = log_elapsed_seconds(log)
    target, actual = [log[k].to_numpy() for k in ('ideal.rad_velo', 'ego.rad_velo')]
    turn = abs(target) >= .02*abs(target).max()
    begin, end = time[turn][[0, -1]]
    p = settings['parameters']
    entry, exit = p['lstart']/1000/p['velo'], p['lend']/1000/p['velo']
    windows = dict(
        entry_length_near_zero_target=(time >= begin-entry) & (time < begin-.005) & (abs(target) < .2),
        exit_length_near_zero_target=(time > end+.005) & (time <= end+exit) & (abs(target) < .2),
        later_diagonal_straight=(time > end+exit+.01) & (time < end+exit+.12) & (abs(target) < .2))
    metrics = {}
    for name, mask in windows.items():
        values = actual[mask]
        assert len(values) >= 3
        metrics[name] = dict(rows=len(values), rms_rad_s=float(np.sqrt(np.mean(values**2))),
            peak_abs_rad_s=float(abs(values).max()), sign_changes=int(np.count_nonzero(np.diff(np.sign(values)))))
    metrics.update(turn_omega_rmse_rad_s=float(np.sqrt(np.mean((actual[turn]-target[turn])**2))),
        turn_speed_rmse_m_s=float(np.sqrt(np.mean((log['ego.velo'][turn]-log['ideal.velo'][turn])**2))),
        minimum_battery_V=float(log.Battery.min()),
        peak_wheel_output_V=float(abs(log[['V_r', 'V_l']]).max().max()))
    paths = (trial_path, settings_path, csv)
    return dict(trial_json=str(trial_path), settings=settings, metrics=metrics,
        log_quality=trial['log_quality'], input_sha256={str(f): hashlib.sha256(f.read_bytes()).hexdigest() for f in paths}), time-begin, target, actual


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--candidate', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    baseline, bt, bi, ba = read(args.baseline)
    candidate, ct, ci, ca = read(args.candidate)
    differences = {group+'.'+key: [value, candidate['settings'][group][key]]
        for group in ('parameters', 'pid', 'feedforward', 'suction')
        for key, value in baseline['settings'][group].items()
        if candidate['settings'][group][key] != value}
    assert differences == {'pid.om_kp': [.3, .1]}, differences
    result = dict(mouse='B', motion='in_r135', differences=differences,
        status='SINGLE_TRIAL_CONTROL_COMPARISON', baseline=baseline, candidate=candidate,
        windows='Approximate from ideal omega boundaries and commanded Lstart/Lend durations; controller state flags not independently logged',
        video_full_review_complete=False, parameter_adoption_allowed=False, repeatability_verified=False,
        analysis_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    args.output.mkdir(exist_ok=False)
    (args.output/'comparison.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    fig, axes = plt.subplots(1, 2, figsize=(10, 4), layout='constrained')
    for axis in axes:
        axis.plot(bt, bi, color='gray', linestyle='--', label='Target, Kp0.30')
        axis.plot(bt, ba, label='Measured, Kp0.30')
        axis.plot(ct, ca, label='Measured, Kp0.10')
        axis.set(xlabel='Time from angular turn start [s]', ylabel='Angular velocity [rad/s]')
    axes[0].set(xlim=(-.05, .28), title='B in_R135, unchanged geometry and FF')
    axes[1].set(xlim=(.10, .25), ylim=(-3, 3), title='Near-zero target after angular turn')
    axes[0].legend(fontsize=8)
    fig.savefig(args.output/'angular_tracking.png', dpi=150)
    plt.close(fig)
    print(json.dumps(dict(differences=differences, baseline=baseline['metrics'], candidate=candidate['metrics']), ensure_ascii=False))


if __name__ == '__main__':
    main()
