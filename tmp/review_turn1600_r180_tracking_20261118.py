"""指定済み走行の保存ログを、実機へ接続せず明示入力で評価する。"""
from pathlib import Path
import json
import sys
import hashlib

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/analysis'))
from turn_ff_report import analyze_group

RUN = (ROOT / sys.argv[1]).resolve()
OUT = RUN / 'tracking_1ms_review_v1'
if OUT.exists():
    raise RuntimeError('既存の評価を上書きしません')
settings = json.loads((RUN / 'saved_1ms.settings.json').read_text(encoding='utf-8'))
trial = json.loads((RUN / 'trial.json').read_text(encoding='utf-8'))
if settings.get('mouse') != 'A' or settings.get('motion_name') != 'long_r180' or settings['log']['period_ms'] != 1:
    raise ValueError('機体・motion・公称周期が今回の測定と一致しません')
retained = pd.read_csv(RUN / 'saved_1ms.csv')
stream = pd.read_csv(RUN / 'telemetry.csv')
signature = ['ideal.velo', 'ego.velo', 'ideal.rad_velo', 'ego.rad_velo', 'V_r', 'V_l']
matches = retained.merge(stream[['time_ms'] + signature], on=signature, how='inner')
if len(matches) < 12 or matches.cnt.max() - matches.cnt.min() < len(retained) * .7:
    raise ValueError('保存ログとMCU時刻の対応が不足しています')
period_ms, origin_ms = np.polyfit(matches.cnt, matches.time_ms, 1)
timing_error = matches.time_ms - (period_ms * matches.cnt + origin_ms)
metrics = analyze_group([(RUN / 'saved_1ms.csv', settings)], OUT)
matches.to_csv(OUT / 'retained_live_matches.csv', index=False)
index0 = int(metrics['metrics_mean']['start'])
index1 = int(metrics['metrics_mean']['end'])
core = np.arange(index0, index1 + 1)
direction = 1 if settings['parameters']['degree'] > 0 else -1
gyro = direction * retained['ego.rad_velo'].to_numpy()
target = direction * retained['ideal.rad_velo'].to_numpy()
pulse_velocity = (retained['Encoder_GetProperty_Right().sp_pulse'] - retained['Encoder_GetProperty_Left().sp_pulse']) * (15.02 * np.pi / 172) / 2
result = dict(mouse='A', motion='long_r180', preset_mm_s=1600,
    correction_deg=settings['parameters']['correction'],
    target_profile_degree=settings['parameters']['degree'] + settings['parameters']['correction'],
    execution_write_attempted=trial['execution_write_attempted'], execution_resend_count=trial['execution_resend_count'],
    source_trial=str(RUN / 'trial.json'), source_csv=str(RUN / 'saved_1ms.csv'),
    source_settings=str(RUN / 'saved_1ms.settings.json'), nominal_period_ms=1,
    measured_period_ms=float(period_ms), matched_stream_rows=len(matches),
    measured_period_fit_rmse_ms=float(np.sqrt(np.mean(timing_error ** 2))),
    irq_period_mean_us=float(retained['irq.period_us'].mean()), sample_count=len(retained),
    cnt_contiguous=bool(np.array_equal(retained.cnt, np.arange(len(retained)))),
    nominal_1ms_tracking_metrics=metrics['metrics_mean'],
    gyro_turn_profile_measured_time_deg=float(np.sum(gyro[core]) * period_ms / 1000 * 180 / np.pi),
    gyro_through_lend_measured_time_deg=float(np.sum(gyro[index0:]) * period_ms / 1000 * 180 / np.pi),
    target_profile_integral_nominal_time_deg=float(np.sum(target[core]) * .001 * 180 / np.pi),
    fused_turn_speed_nominal_mean_mm_s=float(retained['ego.velo'].iloc[core].mean() * 1000),
    pulse_speed_measured_time_mean_mm_s=float(pulse_velocity.iloc[core].mean() * 1000 / period_ms),
    battery_min_v=float(retained.Battery.min()), saved_log_status=trial['saved_log'],
    parameter_adoption_allowed=False,
    limitations=['実時間角度はMCU時刻との一致から求めた周期によるジャイロ積分。独立外部時計による校正ではない。',
        '保存ログはLendまで。最終停止と相対軌道は動画および連続CSVで別途評価する。',
        '単発比較。制御周期・固定速度のモデルからFF係数を独立に同定したとは扱わない。'],
    source_sha256={name: hashlib.sha256((RUN / name).read_bytes()).hexdigest() for name in ['saved_1ms.csv', 'saved_1ms.settings.json', 'telemetry.csv', 'trial.json']})
(OUT / 'verification.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), flush=True)
