"""B1600右V90の取得済みFF比較を保存する。実機操作なし。"""
from pathlib import Path
import hashlib
import json

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT/'tools/logs/pixel_camera/turn1600_B_rv90_20261120_v1'
OUT = BASE/'ff_comparison_v1'
assert not OUT.exists()
OUT.mkdir()
runs = [
    BASE/'baseline01/local_execution/run_B_RV90_1600_baseline_green_20261120_002',
    BASE/'ff_accdec_0p0006_01/local_execution/run_B_RV90_1600_ff0p0006_green_20261120_001',
]
read = lambda path: json.loads(path.read_text(encoding='utf-8'))
reports = [read(run/'tracking_1ms_review_v3/verification.json') for run in runs]
before, after = reports
for name in ('parameters', 'pid', 'suction'):
    assert before[name] == after[name]
changes = {key: [value, after['feedforward'][key]] for key, value in before['feedforward'].items()
           if value != after['feedforward'][key]}
assert set(changes) == {'ff_om_accel', 'ff_om_decel'}
plt.rcParams['font.family'] = 'Yu Gothic'
fig, axes = plt.subplots(3, 1, figsize=(10,10), layout='constrained')
labels = ['初回FF 0.000378', '比較FF 0.000600']
colors = ['tab:blue', 'tab:orange']
for index, (run, report, label, color) in enumerate(zip(runs,reports,labels,colors)):
    frame = pd.read_csv(run/'saved_1ms.csv')
    metrics = report['tracking']
    start, end = int(metrics['start']), int(metrics['end'])
    t = (frame.cnt.to_numpy()-frame.cnt.iloc[start])*report['measured_log_period_ms']
    keep = np.arange(start, len(frame))
    if index == 0:
        axes[0].plot(t[keep], -frame['ideal.rad_velo'].to_numpy()[keep], 'k--', label='目標')
    axes[0].plot(t[keep], -frame['ego.rad_velo'].to_numpy()[keep], color=color, label=label)
    post = np.arange(end+1, len(frame))
    axes[1].plot(t[post]-t[end], -frame['ego.rad_velo'].to_numpy()[post], color=color, label=label)
    maximum = np.maximum(abs(frame['V_r']),abs(frame['V_l']))
    axes[2].plot(t[keep], maximum.iloc[keep], color=color, label=label+' モータ電圧最大')
    axes[2].plot(t[keep], frame['Battery'].iloc[keep], '--', color=color, alpha=.55)
axes[0].set(xlabel='旋回波形開始から [ms]', ylabel='右正の角速度 [rad/s]', title='機体B・1600右V90：形状/PID固定のFF比較')
axes[1].set(xlabel='旋回波形終了から [ms]', ylabel='右正の角速度 [rad/s]', title='波形終了後からLendまで（14標本）')
axes[2].set(xlabel='旋回波形開始から [ms]', ylabel='電圧 [V]', title='左右モータの最大絶対電圧 / 破線は電池電圧')
for axis in axes:
    axis.grid(alpha=.3);axis.legend(fontsize=9)
fig.savefig(OUT/'response_comparison.png',dpi=140);plt.close(fig)
keys = ['tracking_rms_rad_s','first_rms_rad_s','second_rms_rad_s','peak_error_rad_s','post_rms_rad_s','saturation_pct']
table = {key: dict(before=before['tracking'][key],after=after['tracking'][key]) for key in keys}
ratio = after['tracking']['tracking_rms_rad_s']/before['tracking']['tracking_rms_rad_s']
result = dict(mouse='B',motion='r_v90',preset_mm_s=1600,changes=changes,parameters=before['parameters'],
    pid=before['pid'],suction=before['suction'],metrics=table,tracking_rms_reduction_pct=(1-ratio)*100,
    battery_min_v=[before['battery_min_v'],after['battery_min_v']],
    gyro_through_lend_deg=[before['gyro_through_lend_deg'],after['gyro_through_lend_deg']],
    fixed_voltage_comparison=False,parameter_adoption_allowed=False,
    note='旋回中のRMS/ピークは改善、波形終了後のRMSは悪化。各1本で電圧も異なるため再現性や最終採用を意味しない。動画の相対出口と軌道も併せて判断する。',
    source_trials=[str(run/'trial.json') for run in runs],
    input_sha256={str(run/'saved_1ms.csv'):hashlib.sha256((run/'saved_1ms.csv').read_bytes()).hexdigest() for run in runs})
(OUT/'comparison.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(rms_reduction_pct=result['tracking_rms_reduction_pct'],metrics=table,battery_min_v=result['battery_min_v']),ensure_ascii=False))
