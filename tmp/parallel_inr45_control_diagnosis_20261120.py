"""保存済みA1800 in_r45の飽和寄与と角速度応答遅れを診断する。"""
from pathlib import Path
import json
import hashlib
import numpy as np
import pandas as pd

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_inr45_20261120_v1'
RUN=BASE/'baseline01/local_execution/run_A_INR45_1800_baseline_20261120_002'
OUT=BASE/'parallel_analysis/control_diagnosis_v1'
OUT.mkdir(exist_ok=False)
frame=pd.read_csv(RUN/'saved_1ms.csv')
settings=json.loads((RUN/'saved_1ms.settings.json').read_text(encoding='utf-8'))
timing=json.loads((BASE/'parallel_analysis/tracking_1ms_review_v1/verification.json').read_text(encoding='utf-8'))
assert settings['mouse']=='A' and settings['motion_name']=='in_r45'
target=-frame['ideal.rad_velo'].to_numpy()
gyro=-frame['ego.rad_velo'].to_numpy()
threshold=float(target.max()/2)
tu=float(np.interp(threshold,target[98:112],np.arange(98,112)))
gu=float(np.interp(threshold,gyro[98:121],np.arange(98,121)))
td=float(np.interp(threshold,target[111:132][::-1],np.arange(111,132)[::-1]))
gd=float(np.interp(threshold,gyro[120:134][::-1],np.arange(120,134)[::-1]))
shifts=[(float(k),float(np.sqrt(np.mean((target[98:132]-np.interp(np.arange(98,132)+k,
    np.arange(len(frame)),gyro))**2)))) for k in np.linspace(-3,8,111)]
best=min(shifts,key=lambda x:x[1])
mask=(frame.V_r.abs()>=frame.Battery)|(frame.V_l.abs()>=frame.Battery)
rows=[]
for _,r in frame[mask].iterrows():
    om=float(r.om_feedforward+r.om_feedback)
    corr_l=max(om-float(r.Battery),0)
    corr_r=min(float(r.Battery)+om,0)
    ff_r=float(r.sp_feedforward+r.om_feedforward+corr_r-corr_l)
    ff_l=float(-r.sp_feedforward+r.om_feedforward+corr_l-corr_r)
    model_r=ff_r+float(r.sp_feedback+r.om_feedback)
    model_l=ff_l+float(-r.sp_feedback+r.om_feedback)
    model_r+=np.sign(model_r)*.05
    model_l+=np.sign(model_l)*.05
    rows.append(dict(cnt=int(r.cnt),battery_v=float(r.Battery),
        sp_ff_v=float(r.sp_feedforward),sp_fb_v=float(r.sp_feedback),om_ff_v=float(r.om_feedforward),
        om_fb_v=float(r.om_feedback),angular_only_limit_correction_right_v=corr_r,
        angular_only_limit_correction_left_v=corr_l,dead_voltage_per_wheel_v=.05,
        requested_right_v=float(r.V_r),requested_left_v=float(r.V_l),
        reconstructed_right_v=float(model_r),reconstructed_left_v=float(model_l),
        ff_only_max_abs_with_dead_v=float(max(abs(ff_r),abs(ff_l))+.05),
        ff_alone_saturates=bool(max(abs(ff_r),abs(ff_l))+.05>=r.Battery),
        feedback_pushes_combined_request_to_saturation=True))
pd.DataFrame(rows).to_csv(OUT/'saturation_contributions.csv',index=False)
result=dict(mouse='A',motion='in_r45',source_trial=str(RUN/'trial.json'),
    measured_period_ms=timing['measured_period_ms'],saturation_samples=len(rows),
    target_half_height_rad_s=threshold,rise_half_height_delay_samples=gu-tu,
    fall_half_height_delay_samples=gd-td,best_shape_shift_samples=best[0],
    best_shifted_omega_rmse_rad_s=best[1],argmax_peak_delay_samples=9,
    target_flat_peak_samples=list(range(111,119)),
    gyro_software_20_sample_average=False,gyro_lpf1_enabled_in_source=False,
    gyro_dma_read_timing='IMU_read_DMA_Start at previous interrupt postprocess, data consumed at current preprocess.',
    gyro_odr_source='LSM6DSV16X G_ODR_2KHZ, HAODR_SEL1, LPF1 disabled. Sensor internal group delay unmeasured.',
    angular_only_limiter_activated_at_saturated_samples=False,
    clipped_motor_duty=True,logged_V_is_pre_duty_clipping_request=True,
    saturation_contributions=rows,
    candidate_recommendation='Do not add9ms feedforward lead. Preserve actual PID/FF until optical quality/geometry is evaluated; peak argmax is confounded by8sample target plateau.',
    parameter_adoption_allowed=False,
    limitations=['約1制御回のDMAサンプル遅れに実応答とセンサ内部遅れが重なる。半高・最適shiftから独立分離はできない。',
        'FFだけの仮想電圧は今回ログを代入した成分確認。FF/PIDを変えた後の閉ループ予測ではない。'],
    sources_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in
        [RUN/'saved_1ms.csv',RUN/'saved_1ms.settings.json',ROOT/'Module/Src/interrupt.cpp',
         ROOT/'Peripheral/Src/imu.c',ROOT/'Task/Src/ctrl_task.cpp',ROOT/'Params/mouse_config.h',Path(__file__)]})
(OUT/'diagnosis.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
print(json.dumps({k:result[k] for k in ['rise_half_height_delay_samples','fall_half_height_delay_samples',
    'best_shape_shift_samples','best_shifted_omega_rmse_rad_s','saturation_samples']},ensure_ascii=False))
