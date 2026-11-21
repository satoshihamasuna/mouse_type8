"""旋回形状を使わず、前走の生エンコーダ積算距離で動画時刻を照合する。"""
from pathlib import Path
import argparse
import hashlib
import json

import numpy as np
import pandas as pd
from scipy.optimize import minimize_scalar

parser = argparse.ArgumentParser()
parser.add_argument('--review', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--labels', nargs='+', required=True)
parser.add_argument('--optical-r180', action='store_true', help='入口だけの時刻照合で右180度の光学形状を別に評価する')
args = parser.parse_args()
document = json.loads((args.review/'video_comparison.json').read_text(encoding='utf-8'))
assert len(args.labels) == len(document['comparisons'])
args.output.mkdir(exist_ok=False)
results = []
for label, old in zip(args.labels, document['comparisons']):
    run = Path(old['source_trial']).parent
    trial = json.loads((run/'trial.json').read_text(encoding='utf-8'))
    raw = pd.read_csv(run/'saved_1ms.csv')
    stream = pd.read_csv(run/'telemetry.csv')
    curve = pd.read_csv(args.review/(label+'_relative_trajectory.csv'))
    signature = ['ideal.velo', 'ego.velo', 'ideal.rad_velo', 'ego.rad_velo', 'V_r', 'V_l']
    matches = raw.merge(stream[['time_ms']+signature], on=signature, how='inner')
    tick, origin = np.polyfit(matches.cnt, matches.time_ms, 1)
    live_start = stream.loc[stream.time_ms >= trial['log_quality']['motion_start_time_ms']-500, 'time_ms'].iloc[0]
    old_zero = old['fitted_video_time_offset_s']+(origin-live_start)/1000
    saved_time = raw.cnt.to_numpy()*tick/1000
    mm_per_pulse = 15.02*np.pi/172  # Params/mouse_config.h: AのMMPP。
    distance = np.cumsum((raw['Encoder_GetProperty_Right().sp_pulse'].to_numpy()
        -raw['Encoder_GetProperty_Left().sp_pulse'].to_numpy())*mm_per_pulse/2)
    video_time = curve.time_video_s.to_numpy()
    forward = curve.forward_mm.to_numpy()
    valid = curve.detected.to_numpy(bool)
    window = old['entry_window_pts_s']
    selected = valid & (video_time >= window[0]) & (video_time <= window[1])
    assert selected.sum() >= 15

    def fit(mask):
        def residual(zero):
            return np.interp(video_time[mask]-zero, saved_time, distance)-forward[mask]
        value = minimize_scalar(lambda zero: np.mean(residual(zero)**2),
            bounds=(old_zero-.025, old_zero+.025), method='bounded', options={'xatol':1e-10})
        return float(value.x), residual(value.x)

    zero, errors = fit(selected)
    median = np.median(video_time[selected])
    halves = [fit(selected & (video_time <= median))[0], fit(selected & (video_time > median))[0]]
    lend_time = zero+saved_time[-1]
    def point(stamp):
        return [float(np.interp(stamp, video_time[valid], curve.loc[valid, key]))
                for key in ('right_mm', 'forward_mm')]
    result = dict(label=label, source_trial=str(run/'trial.json'), measured_tick_ms=float(tick),
        samples=int(selected.sum()), old_saved_zero_video_s=float(old_zero),
        saved_zero_video_s=zero, clock_shift_from_trajectory_fit_ms=float((zero-old_zero)*1000),
        pre_straight_distance_rms_mm=float(np.sqrt(np.mean(errors**2))),
        half_window_zero_spread_ms=float(abs(halves[0]-halves[1])*1000),
        lend_video_pts_s=float(lend_time), lend_right_forward_mm=point(lend_time),
        lend_minus1ms_right_forward_mm=point(lend_time-.001),
        lend_plus1ms_right_forward_mm=point(lend_time+.001),
        encoder_scale_fitted=False, exit_or_turn_used_for_clock_fit=False,
        independent_hardware_sync=False,
        timing_review_eligible=bool(np.sqrt(np.mean(errors**2)) <= 1.5
            and abs(halves[0]-halves[1]) <= .002 and abs(zero-old_zero) < .024),
        limitations=['前走のタイヤ転がり距離とマーカー中点を同一と仮定。',
            '15.02mmのタイヤ径、視差・ピッチ・フレーム露光時刻の系統差は残る。',
            '±1msは感度表示であり、絶対同期精度の保証ではない。'],
        input_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (run/'saved_1ms.csv', run/'telemetry.csv', args.review/(label+'_relative_trajectory.csv'))})
    if args.optical_r180:
        assert old['motion']=='long_r180'
        xy=curve[['right_mm','forward_mm']].to_numpy()
        shift=zero-old_zero
        entry_mask=valid & (video_time>=old['entry_window_pts_s'][0]+shift) & (video_time<=old['entry_window_pts_s'][1]+shift)
        exit_mask=valid & (video_time>=old['exit_window_pts_s'][0]+shift) & (video_time<=old['exit_window_pts_s'][1]+shift)
        def line(mask):
            points=xy[mask]
            if len(points)<15:
                raise ValueError('独立した光学評価に必要な直線観測15点がありません')
            direction=np.linalg.svd(points-points.mean(axis=0),full_matrices=False)[2][0]
            if direction@(points[-1]-points[0])<0: direction=-direction
            span=float(np.ptp(points@direction))
            if span<30: raise ValueError('光学直線の観測範囲が30mm未満です')
            residual=points-points.mean(axis=0)
            residual-=np.outer(residual@direction,direction)
            return direction,dict(observations=len(points),span_mm=span,rms_mm=float(np.sqrt(np.mean(np.sum(residual**2,axis=1)))))
        entry,entry_q=line(entry_mask); exiting,exit_q=line(exit_mask)
        basis=np.column_stack(([entry[1],-entry[0]],entry))
        optical_xy=xy@basis
        exiting=exiting@basis
        assert abs(exiting[1])>.9
        angle=float(np.degrees(np.arctan2(exiting[0],exiting[1]))%360)
        mean=optical_xy[exit_mask].mean(axis=0)
        width=float(mean[0]+(45-mean[1])*exiting[0]/exiting[1])
        lend=np.asarray(point(lend_time))@basis
        optical_eligible=bool(result['timing_review_eligible'] and old['relative_observation_quality_passed']
            and entry_q['rms_mm']<=.75 and exit_q['rms_mm']<=.75
            and old['stationary_observations']>=12 and old['stationary_scatter_rms_mm']<=1
            and trial['log_quality']['lost_frames']==0 and trial['log_quality']['crc_errors_during_trial']==0
            and np.array_equal(raw.cnt.to_numpy(),np.arange(len(raw))))
        result.update(optical_measurement_method='Observed entry/exit lines, clock fitted to raw incoming wheel distance only; no turning-path fit or scale fit.',
            optical_path_angle_deg=angle,optical_return_spacing_at_forward45_mm=width,
            optical_Lend_right_forward_mm=lend.tolist(),optical_apex_forward_mm=float(optical_xy[valid,1].max()),
            optical_entry_line=entry_q,optical_exit_line=exit_q,
            independent_optical_geometry_eligible=optical_eligible,
            encoder_path_reconstruction_motion_rms_mm=old['encoder_model_motion_rmse_mm'],
            original_encoder_path_review_eligible=old['relative_review_eligible'],
            optical_geometry_scope='Relative marker midpoint and measured entry; absolute body/axle pose remains unverified.')
    pd.DataFrame(dict(time_video_s=video_time[selected], video_forward_mm=forward[selected],
        encoder_forward_mm=forward[selected]+errors, residual_mm=errors)).to_csv(
            args.output/(label+'_entry_alignment.csv'), index=False)
    results.append(result)
(args.output/'report.json').write_text(json.dumps(dict(results=results,
    parameter_adoption_allowed=False, source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()),
    ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
print(json.dumps([{k:r[k] for k in ('label','clock_shift_from_trajectory_fit_ms',
    'pre_straight_distance_rms_mm','half_window_zero_spread_ms','lend_right_forward_mm','timing_review_eligible')}
    for r in results], ensure_ascii=False), flush=True)
