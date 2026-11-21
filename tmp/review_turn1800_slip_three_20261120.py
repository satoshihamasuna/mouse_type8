"""同一設定のA1800右90度3本を使う滑り係数・雑音の事後評価。機器操作なし。"""
from pathlib import Path
import hashlib
import json
import sys

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT/'tools/logs/pixel_camera/turn1800_A_r90_20261119_v1'
OUT = BASE/'slip_corr1p0_three_v1'
sys.path.insert(0, str(ROOT/'tools/analysis'))
from turn_slip_conditions import observe, force_fit
from turn_ff_report import settings_fingerprint

CASES = [
    ('01', BASE/'next_corr1p0/local_execution/run_A_R90_1800_corr1p0_20261120_002'),
    ('02', BASE/'repeat_corr1p0_02/local_execution/run_A_R90_1800_corr1p0_repeat02_20261120_001'),
    ('03', BASE/'repeat_corr1p0_03/local_execution/run_A_R90_1800_corr1p0_repeat03_20261120_001'),
]
read = lambda p: json.loads(p.read_text(encoding='utf-8'))
OUT.mkdir(exist_ok=False)
frames, metrics, noise, sensitivity, sources, rejected = {}, [], [], [], [], []
fingerprint = None
for label, run in CASES:
    full = run/'full'
    settings = read(run/'motion.settings.json')
    this = settings_fingerprint(settings)
    if fingerprint is None:
        fingerprint = this
    assert this == fingerprint, '異なる設定を同一候補として混ぜない'
    raw, pose = pd.read_csv(run/'motion.csv'), pd.read_csv(full/'apparent_floor_marker_pose.csv')
    geometry, report = read(full/'body_geometry_used.json'), read(full/'report.json')
    assert report['required_motion_window_quality']['accepted_for_relative_review']
    sources.append(dict(trial=label, source_trial=str(run/'trial.json'),
        input_sha256={str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (run/'motion.csv', run/'motion.settings.json', full/'report.json',
                      full/'apparent_floor_marker_pose.csv', full/'body_geometry_used.json')}))
    try:
        observation = observe(raw, settings, pose, geometry, report)
    except (ValueError, KeyError) as error:
        rejected.append(dict(trial=label, reason=str(error)))
        continue
    observation.frame.to_csv(OUT/f'observations_{label}.csv', index=False)
    frames[label] = observation.frame
    metrics.append(dict(trial=label, **observation.diagnostics))
    for window in (.015, .025):
        try:
            other = observe(raw, settings, pose, geometry, report, window)
            sensitivity.append(dict(trial=label, half_window_ms=window*1000,
                k_m_s2=other.diagnostics['force']['k_m_s2'],
                ay_rmse_m_s2=other.diagnostics['force']['rmse_m_s2']))
        except ValueError as error:
            sensitivity.append(dict(trial=label, half_window_ms=window*1000, reason=str(error)))
    relative_time = (raw.time_ms-settings['log']['motion_start_time_ms'])/1000
    quiet = relative_time.between(-.2, -.04) & (raw['ideal.velo'].abs()<.01)
    quiet &= (raw['ego.velo']-.010*raw['ego.accel']).abs()<.02
    noise.append(dict(trial=label, window_before_motion_s=[-.2, -.04], samples=int(quiet.sum()),
        gyro_std_rad_s=float(raw.loc[quiet, 'ego.rad_velo'].std(ddof=1)),
        gyro_bias_median_rad_s=float(raw.loc[quiet, 'ego.rad_velo'].median()),
        lateral_accel_std_m_s2=float(raw.loc[quiet, 'ego.horizon_accel.get'].std(ddof=1)),
        lateral_accel_bias_median_m_s2=float(raw.loc[quiet, 'ego.horizon_accel.get'].median())))

existing_k = 254.90725  # Params/turn_pose_param.hのA1600吸引。A1800への外挿比較のみ。
cross_validation, pooled = [], None
if len(frames) >= 2:
    all_frames = pd.concat(list(frames.values()), ignore_index=True)
    pooled = force_fit(all_frames.averaged_beta_rad, all_frames.imu_left_m_s2, all_frames['fit'])
    for label, test in frames.items():
        train = pd.concat([value for name, value in frames.items() if name != label], ignore_index=True)
        fit = force_fit(train.averaged_beta_rad, train.imu_left_m_s2, train['fit'])
        keep = test['fit'].to_numpy(bool)
        def error(k):
            difference = -test.imu_left_m_s2.to_numpy()/k-test.averaged_beta_rad.to_numpy()
            return float(np.degrees(np.sqrt(np.mean(difference[keep]**2))))
        cross_validation.append(dict(heldout_trial=label, training_trials=len(frames)-1,
            training_k_m_s2=fit['k_m_s2'], training_fit_at_bound=fit['at_bound'],
            candidate_beta_rmse_deg=error(fit['k_m_s2']),
            extrapolated_A1600_k_beta_rmse_deg=error(existing_k)))

result = dict(mouse='A', motion='long_r90', speed_mm_s=1800,
    suction=dict(enabled=True, duty=650), correction_deg=1., settings_fingerprint=fingerprint,
    trial_count=len(CASES), accepted_trials=len(frames), rejected=rejected,
    pooled_force_fit=pooled, per_trial=metrics, sensitivity=sensitivity,
    late_pre_motion_noise=noise, leave_one_trial_out=cross_validation,
    existing_comparison=dict(k_m_s2=existing_k, calibration_speed_mm_s=1600,
        label='A1600吸引係数の外挿比較。A1800で実機EKFに適用した値ではない。'),
    q_identified=False, initial_bias_covariance_adopted=False,
    gyro_optical_gain_is_not_sensor_calibration=True,
    firmware_application_allowed=False, source_params_changed=False,
    limitations=['3本の右90度・同一速度・吸引650のみ。左・別形状・別dutyは未検証。',
        '機体中心IMU・中点中心とマーカー高さを仮定した相対評価。カメラ真下位置は未校正。',
        '静止雑音は走行前の末尾160msから算出。実dutyの独立計測履歴はない。',
        'Kの試行除外評価と因果的ファーム再生・実機採用は別。Qと初期共分散は確定していない。'],
    sources=sources, source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
(OUT/'report.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
pd.DataFrame(cross_validation).to_csv(OUT/'leave_one_trial_out.csv', index=False)
pd.DataFrame(noise).to_csv(OUT/'stationary_noise.csv', index=False)
if pooled:
    fig, axes = plt.subplots(1, 2, figsize=(10, 4), layout='constrained')
    for label, frame in frames.items():
        selected = frame.loc[frame['fit']]
        axes[0].scatter(np.degrees(selected.averaged_beta_rad), selected.imu_left_m_s2, s=16, label=label)
    beta = np.linspace(-.35, .35, 100)
    axes[0].plot(np.degrees(beta), -pooled['k_m_s2']*beta, label='Pooled candidate')
    axes[0].plot(np.degrees(beta), -existing_k*beta, '--', label='A1600 extrapolation')
    axes[0].set(xlabel='Video averaged slip [deg]', ylabel='Measured lateral acceleration [m/s²]')
    table = pd.DataFrame(cross_validation)
    indices = np.arange(len(table))
    axes[1].bar(indices-.18, table.extrapolated_A1600_k_beta_rmse_deg, .36, label='A1600 extrapolation')
    axes[1].bar(indices+.18, table.candidate_beta_rmse_deg, .36, label='Held-out candidate')
    axes[1].set(xticks=indices, xticklabels=table.heldout_trial, xlabel='Held-out trial', ylabel='Slip residual RMS [deg]')
    for ax in axes:
        ax.grid(alpha=.25)
        ax.legend(fontsize=8)
    fig.savefig(OUT/'force_and_holdout.png', dpi=160)
    plt.close(fig)
print(json.dumps(dict(accepted_trials=len(frames), pooled_force_fit=pooled,
    leave_one_trial_out=cross_validation, rejected=rejected), ensure_ascii=False), flush=True)
