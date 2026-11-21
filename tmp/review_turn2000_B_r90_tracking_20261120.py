"""B2000右90°の保存済み1msログを評価する。機器へ接続しない。"""
from pathlib import Path
import hashlib
import json
import sys

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools/analysis'))
from turn_ff_report import analyze_group

BASE=ROOT/'tools/logs/pixel_camera/turn2000_B_r90_20261120_v1'
RUN=BASE/'cli_session01/run_B_R90_2000_baseline_20261120_001'
if len(sys.argv)>1:
    RUN=(BASE/sys.argv[1]).resolve()
    if not RUN.is_relative_to(BASE.resolve()):
        raise ValueError('今回のB2000フォルダ内の試行を指定する')
OUT=RUN/'tracking_1ms_review_v2'
if OUT.exists():
    raise RuntimeError('既存の評価を上書きしない')
settings=json.loads((RUN/'saved_1ms.settings.json').read_text(encoding='utf-8'))
trial=json.loads((RUN/'trial.json').read_text(encoding='utf-8'))
assert settings['mouse']=='B' and settings['motion_name']=='long_r90'
assert settings['preset_speed_mm_s']==2000 and settings['log']['period_ms']==1
retained=pd.read_csv(RUN/'saved_1ms.csv')
stream=pd.read_csv(RUN/'telemetry.csv')
assert len(retained)>100 and retained.cnt.is_unique
assert np.array_equal(retained.cnt.to_numpy(),np.arange(len(retained)))
signature=['ideal.velo','ego.velo','ideal.rad_velo','ego.rad_velo','V_r','V_l']
matches=retained.merge(stream[['time_ms']+signature],on=signature,how='inner')
if len(matches)<12 or matches.cnt.max()-matches.cnt.min()<len(retained)*.7:
    raise RuntimeError('保存ログとMCU時刻の対応不足')
period_ms,origin_ms=np.polyfit(matches.cnt,matches.time_ms,1)
timing_error=matches.time_ms-(period_ms*matches.cnt+origin_ms)
if not .95<period_ms<1.05 or abs(timing_error).max()>1.5:
    raise RuntimeError('保存周期の実測照合が不成立')
metrics=analyze_group([(RUN/'saved_1ms.csv',settings)],OUT)
matches.to_csv(OUT/'retained_live_matches.csv',index=False)
first=int(metrics['metrics_mean']['start']);last=int(metrics['metrics_mean']['end'])
idx=np.arange(first,last+1)
direction=-1.
gyro=direction*retained['ego.rad_velo'].to_numpy()
target=direction*retained['ideal.rad_velo'].to_numpy()
speed=retained['ego.velo'].to_numpy()*1000
speed_target=retained['ideal.velo'].to_numpy()*1000
result=dict(mouse='B',motion='long_r90',preset_mm_s=2000,
    parameters=settings['parameters'],pid=settings['pid'],feedforward=settings['feedforward'],
    suction=settings['suction'],execution_resend_count=trial['execution_resend_count'],
    nominal_log_period_ms=1,measured_log_period_ms=float(period_ms),
    measured_period_fit_rmse_ms=float(np.sqrt(np.mean(timing_error**2))),
    matched_stream_rows=len(matches),sample_count=len(retained),cnt_contiguous=True,
    irq_period_mean_us=float(retained['irq.period_us'].mean()),
    tracking=metrics['metrics_mean'],
    target_integral_deg=float(target[idx].sum()*.001*180/np.pi),
    gyro_turn_profile_deg=float(gyro[idx].sum()*period_ms/1000*180/np.pi),
    gyro_through_lend_deg=float(gyro[first:].sum()*period_ms/1000*180/np.pi),
    turn_speed_mean_mm_s=float(speed[idx].mean()),
    turn_speed_target_mean_mm_s=float(speed_target[idx].mean()),
    turn_speed_error_rms_mm_s=float(np.sqrt(np.mean((speed[idx]-speed_target[idx])**2))),
    turn_speed_min_mm_s=float(speed[idx].min()),turn_speed_max_mm_s=float(speed[idx].max()),
    battery_min_v=float(retained.Battery.min()),saved_log_status=trial['saved_log'],
    parameter_adoption_allowed=False,
    limitations=['単発のB実測。FF係数は独立同定していない。',
        '周期は保存行とMCU配信行の対応による確認。外部時計校正ではない。',
        '保存ログはLendまで。停止・出口方向は連続CSVと動画で確認する。',
        'BのEKF係数同定にはB用幾何校正とツール対応が別途必要。'],
    source_sha256={name:hashlib.sha256((RUN/name).read_bytes()).hexdigest()
        for name in ['saved_1ms.csv','saved_1ms.settings.json','telemetry.csv','trial.json']})
with (OUT/'verification.json').open('x',encoding='utf-8') as output:
    json.dump(result,output,ensure_ascii=False,indent=2,allow_nan=False)
    output.write('\n')
print(json.dumps(result,ensure_ascii=False),flush=True)
