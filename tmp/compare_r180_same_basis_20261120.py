"""半径48mmの前回と今回を同じ赤前点・断面・退出phaseで比較する。実機操作なし。"""
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
BASE = ROOT / 'tools/logs/pixel_camera/turn1800_A_r180_20261120_v1'
OUT = BASE / 'comparison_before_radius54'
RUNS = [BASE / 'finish02_01/local_execution/run_A_R180_1800_finish02_c3p5_r48_Le31p5_20261120_001',
    BASE / 'angle_c4p25_03/local_execution/run_A_R180_1800_angle_c4p25_r48_Le31p5_20261120_003']
FRONTS = [OUT / 'old_front_same_phase_v1', BASE / 'angle_c4p25_03/front_line_diagnostic_v2']
read = lambda p: json.loads(p.read_text(encoding='utf-8-sig'))
metrics = []
evidence = [Path(__file__)]
settings = [read(run / 'saved_1ms.settings.json') for run in RUNS]
for key in ('pid', 'feedforward', 'suction', 'mouse', 'motion_name', 'preset_speed_mm_s', 'turn_count'):
    assert settings[0][key] == settings[1][key], key
assert dict(settings[0]['parameters'], correction=4.25) == settings[1]['parameters']
plt.rcParams['font.family'] = 'Yu Gothic'
fig, ax = plt.subplots(figsize=(6.5, 7.5), layout='constrained')
for label, run, front_dir, actual in zip(('前回＋3.5°', '今回＋4.25°'), RUNS, FRONTS, settings):
    front = read(front_dir / 'front_reference.json')
    section = front['return_line']['sections'][0]
    assert section['forward_mm'] == 0 and not section['extrapolated']
    assert front['clock']['passed'] and front['actual_registered_aruco_per_frame']
    raw, stream = pd.read_csv(run / 'saved_1ms.csv'), pd.read_csv(run / 'telemetry.csv')
    signature = ['ideal.velo', 'ego.velo', 'ideal.rad_velo', 'ego.rad_velo', 'V_r', 'V_l']
    matched = raw.merge(stream[['time_ms'] + signature], on=signature)
    tick, _ = np.polyfit(matched.cnt, matched.time_ms, 1)
    target = np.abs(raw['ideal.rad_velo'].to_numpy())
    active = target > .02 * target.max()
    wheel = (raw['Encoder_GetProperty_Right().sp_pulse'] - raw['Encoder_GetProperty_Left().sp_pulse']) * 15.02 * np.pi / 172 / 2 / tick
    metrics.append(dict(label=label, source_trial=str(run / 'trial.json'),
        correction_deg=actual['parameters']['correction'], radius_mm=abs(actual['parameters']['r_min']),
        front_width_at_forward0_mm=section['return_spacing_mm'],
        front_path_angle_deg=front['path_direction_change_deg'],
        front_motion_observation_rate=front['motion_only_observation_rate'],
        front_required_observation_rate=front['observation_rate'],
        old_failure_not_reclassified=True,
        wheel_speed_mean_m_s=float(wheel[active].mean()),
        gyro_peak_rad_s=float(np.abs(raw['ego.rad_velo']).max()),
        battery_min_v=float(raw.Battery.min()), battery_max_v=float(raw.Battery.max()),
        measured_period_ms=float(tick)))
    curve = pd.read_csv(front_dir / 'front_relative_trajectory.csv')
    xy = curve[['right_mm', 'forward_mm']].to_numpy()
    xy[~curve.front_relative_valid.to_numpy(bool)] = np.nan
    ax.plot(xy[:, 0], xy[:, 1], label=label)
    evidence.extend([run / 'saved_1ms.csv', run / 'saved_1ms.settings.json', run / 'telemetry.csv',
        run / 'trial.json', run / 'marker_reference_used.json', run / 'body_geometry_used.json',
        front_dir / 'front_reference.json', front_dir / 'front_relative_trajectory.csv'])
ax.set(xlabel='実測入口に対する右 [mm]', ylabel='実測入口に対する前 [mm]',
    title='A / 1800 / R180 / 半径48mm\n同じ赤前点・前0mm断面で比較', aspect='equal')
ax.axvline(90, ls='--', color='.5', label='返り幅の名目目標90mm')
ax.grid(); ax.legend(fontsize=9)
plot_name = 'same_front_comparison_verified.png' if '--plot-only' in sys.argv else 'same_front_comparison.png'
fig.savefig(OUT / plot_name, dpi=140)
plt.close(fig)
if '--plot-only' in sys.argv:
    print(str(OUT / plot_name))
    raise SystemExit(0)
report = dict(state='HOLD_RADIUS54_VERIFY_48MM_REPEATABILITY', mouse='A', motion='long_r180',
    comparison_basis='赤前マーカー、入口方向基準、前0mmの実観測範囲内断面。退出窓は両方ともLend seed+5msから走行停止seed+10ms。',
    metrics=metrics, width_difference_mm=metrics[1]['front_width_at_forward0_mm'] - metrics[0]['front_width_at_forward0_mm'],
    angle_difference_deg=metrics[1]['front_path_angle_deg'] - metrics[0]['front_path_angle_deg'],
    unchanged=['radius48', 'Lstart7.52', 'Lend31.5', 'preset1800', 'PID', 'FF', 'suction650',
        'camera_height1500', 'EV-1.5', 'zoom1.8', 'torchOFF', 'focus_lockON', 'wall/turn EKF modes'],
    changed=['correction3.5->4.25', 'operator replaced battery'],
    reference_changes='盤面・頂点・尺度・幾何は同じ。ArUco探索ROI制限の削除と参照パスの変更。',
    reporting_correction='前回は赤前点、今回の79.2mm/182.4degは中点であり、そのまま同一指標の比較ではなかった。',
    cause_identified=False, battery_causality_identified=False,
    old_motion_quality_passed=False, midpoint_comparison_allowed=False,
    radius54_executed=False, source_params_changed=False, parameter_adoption_allowed=False,
    next_test='半径48mm・補正+3.5degの前回設定を電池交換後の条件で再確認。fresh readiness未確認のため未実行。',
    next_stage=str(BASE / 'restore_c3p5_r48_01'),
    source_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in evidence})
with (OUT / 'comparison.json').open('x', encoding='utf-8') as file:
    json.dump(report, file, ensure_ascii=False, indent=2, allow_nan=False)
    file.write('\n')
old, new = metrics
text = f'''# 機体A・1800 R180の前回差確認

半径54mm案は未実行のまま保留。前回と今回を赤前点・前0mm断面・同じ退出phaseへ揃えた。

| 指標 | 前回 | 今回 |
| --- | ---: | ---: |
| 半径 [mm] | 48 | 48 |
| 角度補正 [deg] | 3.5 | 4.25 |
| 返り幅・赤前点 [mm] | {old['front_width_at_forward0_mm']:.2f} | {new['front_width_at_forward0_mm']:.2f} |
| 返り角・赤前点 [deg] | {old['front_path_angle_deg']:.2f} | {new['front_path_angle_deg']:.2f} |
| 車輪換算速度・旋回平均 [m/s] | {old['wheel_speed_mean_m_s']:.3f} | {new['wheel_speed_mean_m_s']:.3f} |
| ジャイロ角速度ピーク [rad/s] | {old['gyro_peak_rad_s']:.2f} | {new['gyro_peak_rad_s']:.2f} |
| 保存ログの最低電圧 [V] | {old['battery_min_v']:.2f} | {new['battery_min_v']:.2f} |

幅の差は{report['width_difference_mm']:.2f}mm、角度差は{report['angle_difference_deg']:+.2f}deg。
以前の説明の79.2mm/182.4degは中点の値であり、赤前点の前回値との直接比較ではなかった。
比較基準を揃えても幅の差は残る。ログ側にも応答差があり、電池交換の因果関係は未確定。

前回の純走行赤観測率は94.67%で、95%条件に不合格の診断値という扱いを保持する。中点の直接比較や採用はしない。
PID/FF・吸引650・半径と前後長・撮影条件は一致。設定で変えたのは補正+0.75degだけ。

次は半径48mm・補正+3.5degの前回設定を新しい電池条件で1回確認する候補を準備済み。
今回の配置・ON・手離し確認は未取得で、接続・設定送信・走行はしていない。Params変更もなし。
'''
with (OUT / 'README.md').open('x', encoding='utf-8') as file:
    file.write(text)
print(json.dumps(dict(report=str(OUT / 'comparison.json'), delta_width_mm=report['width_difference_mm'],
    delta_angle_deg=report['angle_difference_deg'], next_stage=report['next_stage'], executed=False), ensure_ascii=False))
