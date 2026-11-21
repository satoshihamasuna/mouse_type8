"""A1800右45度入口の相対形状。保存入力のみ・前走距離だけで時刻を再照合。"""
from pathlib import Path
import argparse
import hashlib
import json
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.optimize import minimize_scalar

from review_turn1600_A_inr45_20261119 import align, line_fit, onboard_positions

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools/analysis'))
from turn_slip_conditions import observe


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    run, out = args.run.resolve(), args.out.resolve()
    if out.exists():
        raise FileExistsError('既存の解析結果を上書きしません')
    full = run/'full'
    trial, settings = read(run/'trial.json'), read(run/'saved_1ms.settings.json')
    quality = read(full/'report.json')
    summary = read(full/'summary.json') if (full/'summary.json').is_file() else {}
    assert trial['motion'] == settings['motion_name'] == 'in_r45'
    assert settings['mouse'] == trial['mouse'] == 'A' and settings['preset_speed_mm_s'] == 1800
    assert settings['log']['period_ms'] == 1
    saved, stream, pose, relative = map(pd.read_csv, (run/'saved_1ms.csv',run/'telemetry.csv',
        full/'apparent_floor_marker_pose.csv',full/'relative_marker_midpoint.csv'))
    assert np.array_equal(saved.cnt, np.arange(len(saved)))
    assert np.array_equal(relative.time_video_s, pose.time_video_s)
    xy = relative[['center_dx_mm','center_dy_mm']].to_numpy()
    valid = relative.detected.to_numpy(bool)
    time = relative.time_video_s.to_numpy()
    # Legacy helper supplies a coarse model alignment only. The final clock is
    # estimated below from the incoming wheel distance, without turning data.
    coarse_pts, period, origin, model_rms, motion_rms = align(trial,saved,stream,pose,xy,valid,
        read(run/'marker_reference_used.json'))
    pattern = saved['motion.pattern'].to_numpy()
    start, stop = (trial['log_quality'][k] for k in ('motion_start_time_ms','motion_end_time_ms'))
    turn_start = origin+period*np.flatnonzero(pattern == 7)[0]
    old_zero = coarse_pts(origin)
    entry_window = [coarse_pts(start+8),coarse_pts(turn_start-3)]
    entry_mask = valid & (time >= entry_window[0]) & (time <= entry_window[1])
    entry, entry_stats = line_fit(xy[entry_mask])
    basis = np.column_stack([[-entry[1],entry[0]],entry])
    still_before = valid & (time >= coarse_pts(start)-.25) & (time < coarse_pts(start)-.025)
    assert still_before.sum() >= 12
    video_origin = np.median(xy[still_before],axis=0)
    local = (xy-video_origin)@basis
    saved_time = saved.cnt.to_numpy()*period/1000
    mm_per_pulse = 15.02*np.pi/172
    distance = np.cumsum((saved['Encoder_GetProperty_Right().sp_pulse'].to_numpy()
        -saved['Encoder_GetProperty_Left().sp_pulse'].to_numpy())*mm_per_pulse/2)

    def fit_clock(mask):
        def residual(zero):
            return np.interp(time[mask]-zero,saved_time,distance)-local[mask,1]
        fitted = minimize_scalar(lambda zero:np.mean(residual(zero)**2),
            bounds=(old_zero-.025,old_zero+.025),method='bounded',options={'xatol':1e-10})
        return float(fitted.x),residual(fitted.x)

    zero, clock_errors = fit_clock(entry_mask)
    middle = np.median(time[entry_mask])
    halves = [fit_clock(entry_mask & (time <= middle))[0],fit_clock(entry_mask & (time > middle))[0]]
    clock_rms = float(np.sqrt(np.mean(clock_errors**2)))
    spread_ms = float(abs(halves[0]-halves[1])*1000)
    clock_shift_ms = float((zero-old_zero)*1000)
    clock_eligible = bool(clock_rms <= 1.5 and spread_ms <= 2 and abs(clock_shift_ms) < 24)
    def pts(mcu):
        return float(zero+(mcu-origin)/1000)
    # Re-estimate the line in the corrected incoming window, then preserve the
    # same clock. This changes axes by observation only and fits no scale.
    entry_window = [pts(start+8),pts(turn_start-3)]
    entry_mask = valid & (time >= entry_window[0]) & (time <= entry_window[1])
    entry, entry_stats = line_fit(xy[entry_mask])
    basis = np.column_stack([[-entry[1],entry[0]],entry])
    local = (xy-video_origin)@basis
    lend_time = pts(origin+period*saved.cnt.iloc[-1])
    exit_window = [lend_time+.008,pts(stop-8)]
    exit_mask = valid & (time >= exit_window[0]) & (time <= exit_window[1])
    exiting, exit_stats = line_fit(xy[exit_mask])
    exit_direction = exiting@basis
    angle = float(np.degrees(np.arctan2(exit_direction[0],exit_direction[1])))
    assert 0 < angle < 90
    target = np.array([45.,180.])

    def at(stamp):
        good_time = time[valid]
        left, right = np.searchsorted(good_time,stamp)-1,np.searchsorted(good_time,stamp)
        if left < 0 or right >= len(good_time) or good_time[right]-good_time[left] > .020:
            raise ValueError('Lend評価点の前後に20ms以内の登録済み観測がありません')
        return np.array([np.interp(stamp,good_time,axis[valid]) for axis in local.T])

    def lengths(stamp):
        point = at(stamp)
        extra = (target[0]-point[0])/exit_direction[0]
        return dict(endpoint_right_forward_mm=point.tolist(),
            Lstart_mm=float(settings['parameters']['lstart']+target[1]-point[1]-extra*exit_direction[1]),
            Lend_mm=float(settings['parameters']['lend']+extra))

    stop_info = trial['recording_stop']
    steady_window = [stop_info['confirmation_camera_seconds']+.1,stop_info['confirmation_camera_seconds']+.54]
    steady = valid & (time >= steady_window[0]) & (time <= steady_window[1])
    assert steady.sum() >= 12
    final = np.median(local[steady],axis=0)
    scatter = float(np.sqrt(np.sum(np.var(local[steady],axis=0))))
    normal = exit_direction[::-1]*[1,-1]
    shift_forward = float((target@normal-np.median(local[exit_mask]@normal))/normal[1])
    z = quality['required_motion_window_quality']
    optical_eligible = bool(z['accepted_for_relative_review'] and clock_eligible
        and entry_stats['rms_mm'] <= .75 and exit_stats['rms_mm'] <= .75 and scatter <= 1
        and trial['log_quality']['lost_frames'] == 0 and trial['log_quality']['crc_errors_during_trial'] == 0)
    onboard, reset = onboard_positions(saved,settings['parameters']['lend'])
    try:
        slip = dict(state='DIAGNOSTIC_ONLY',diagnostics=observe(pd.read_csv(run/'motion.csv'),
            read(run/'motion.settings.json'),pose,read(full/'body_geometry_used.json'),quality,.020).diagnostics)
    except ValueError as error:
        slip = dict(state='UNAVAILABLE',reason=str(error))
    result = dict(schema_version=1,mouse='A',motion='in_r45',source_trial=str(run/'trial.json'),settings=settings,
        reference='登録済みマーカー中点の開始位置と実測入口直線。尺度固定。絶対車軸・body yaw・盤面接続は未検証。',
        one_ms_to_mcu_period_ms=float(period),one_ms_to_mcu_origin_ms=float(origin),
        coarse_trajectory_fit_zero_video_s=old_zero,saved_zero_video_s=zero,
        clock_shift_from_trajectory_fit_ms=clock_shift_ms,pre_straight_distance_rms_mm=clock_rms,
        half_window_zero_spread_ms=spread_ms,timing_review_eligible=clock_eligible,
        clock_fit_uses_turn_or_exit=False,scale_fitted=False,independent_hardware_sync=False,
        entry_window_pts_s=entry_window,exit_window_pts_s=exit_window,entry_line=entry_stats,exit_line=exit_stats,
        entry_board_path_heading_deg=float(np.degrees(np.arctan2(entry[1],entry[0]))),
        exit_board_path_heading_deg=float(np.degrees(np.arctan2(exiting[1],exiting[0]))),
        exit_direction_deg=angle,exit_error_deg=angle-45,
        estimated_Lend_pts_s=lend_time,length_only_estimates=lengths(lend_time),
        length_sensitivity={label:lengths(lend_time+ms/1000) for label,ms in
            [('minus5ms',-5),('minus1ms',-1),('plus1ms',1),('plus5ms',5)]},
        Lstart_from_exit_line_mm=float(settings['parameters']['lstart']+shift_forward),
        target_end_right_forward_mm=target.tolist(),target_stop_right_forward_mm=[135.,270.],
        stopped_right_forward_mm=final.tolist(),stationary_scatter_mm=scatter,stationary_observations=int(steady.sum()),
        stationary_window_pts_s=steady_window,estimated_brake_entry_right_forward_mm=at(pts(stop)).tolist(),
        onboard_Lend_end_right_forward_mm=onboard[-1].tolist(),
        optical_rate=z['observation_rate'],registration_rate=z['calibration_rate'],
        relative_observation_quality_passed=z['accepted_for_relative_review'],
        encoder_model_rmse_mm=model_rms,encoder_model_motion_rmse_mm=motion_rms,
        relative_optical_geometry_eligible=optical_eligible,
        original_encoder_model_review_eligible=bool(z['accepted_for_relative_review'] and model_rms <= 8 and motion_rms <= 8),
        original_auto_stop_confirmed=summary.get('auto_stop_confirmed'),slip=slip,
        parameter_adoption_allowed=False,source_params_changed=False,
        limitations=['単発試行。再現性未検証。','Lstart/Lendは現角度の出口方向を保持した長さだけの解。角度補正後には再測定が必要。',
            '前走のタイヤ積算距離とマーカー中点移動を同一と仮定。ピッチ・視差・露光時刻とタイヤ尺度誤差は残る。',
            'gyroの20点平均はない。IMUは前回IRQ後のDMA値。絶対センサ遅れの同定ではない。'],
        sources_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in
            [run/'trial.json',run/'saved_1ms.csv',run/'saved_1ms.settings.json',run/'telemetry.csv',
             full/'report.json',full/'relative_marker_midpoint.csv',Path(__file__)]})
    out.mkdir(parents=True,exist_ok=False)
    (out/'review.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    pd.DataFrame(dict(time_video_s=time,detected=valid,right_mm=local[:,0],forward_mm=local[:,1])).to_csv(out/'relative_trajectory.csv',index=False)
    pd.DataFrame(dict(time_video_s=time[entry_mask],video_forward_mm=local[entry_mask,1])).to_csv(out/'entry_fit_points.csv',index=False)
    mask = (time >= pts(start)-.035) & (time <= pts(stop)+.33)
    fig, ax = plt.subplots(figsize=(6.5,7),layout='constrained')
    plot = np.where(valid[:,None],local,np.nan)
    ax.plot(*plot[mask].T,label='Registered video midpoint',lw=2)
    ax.plot(*onboard[:reset].T,label='Recorded onboard position',alpha=.7)
    ax.plot(*onboard[reset-1:].T,'--',alpha=.7,label='Onboard Lend joined by gyro')
    ax.scatter(*at(lend_time),label='Observed Lend time',s=36)
    ax.scatter(*target,label='Target Lend',marker='x',s=44)
    ax.scatter(*final,label='Observed stop',s=36,facecolors='none',edgecolors='teal')
    ax.scatter(135,270,label='Reference stop',marker='x',s=44)
    ax.set(xlabel='Right [mm]',ylabel='Forward [mm]',aspect='equal')
    ax.grid(alpha=.25); ax.legend(fontsize=8)
    fig.savefig(out/'trajectory.png',dpi=160); plt.close(fig)
    text = f'''<!doctype html><html lang="ja"><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>A1800 右45度入口</title><style>body{{max-width:850px;margin:24px auto;padding:0 16px;font:16px/1.7 system-ui;color:#183442}}img{{max-width:100%}}</style><h1>機体A turn1800 右45度入口</h1><p>入口方向基準の相対マーカー中点。絶対車軸、body yaw、盤面固定の接続は未検証。</p><p>出口 {angle:.3f}°（目標45°、差 {angle-45:+.3f}°）。Lend時点 {at(lend_time)[0]:.2f} / {at(lend_time)[1]:.2f} mm（右/前、目標45/180）。停止 {final[0]:.2f} / {final[1]:.2f} mm（参考135/270）。</p><p>実周期 {period:.6f} ms。前走距離のみの時計照合RMSE {clock_rms:.3f} mm、半区間差 {spread_ms:.3f} ms。光学観測 {z['observation_rate']:.1%}、ArUco登録 {z['calibration_rate']:.1%}。品質判定 {optical_eligible}。</p><img src="trajectory.png" alt="動画と機体内部の相対軌跡"><p><a href="review.json">評価値・入力ハッシュ・制約</a> / <a href="relative_trajectory.csv">登録済み観測マスク付き座標</a></p></html>'''
    (out/'compare.html').write_text(text,encoding='utf-8')
    print(json.dumps({k:result[k] for k in ['exit_direction_deg','exit_error_deg','length_only_estimates',
        'Lstart_from_exit_line_mm','stopped_right_forward_mm','optical_rate','registration_rate',
        'pre_straight_distance_rms_mm','half_window_zero_spread_ms','relative_optical_geometry_eligible','slip']},ensure_ascii=False),flush=True)


if __name__ == '__main__':
    main()
