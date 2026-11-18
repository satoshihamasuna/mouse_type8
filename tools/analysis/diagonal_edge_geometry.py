"""斜めの既知柱と左右の特徴点から姿勢を推定するオフライン幾何モデル。

座標は前方x・左y [mm]、角度は左正 [rad]。画像座標を推定入力に
含めない。部品の取付角とIR波形の実効特徴点方向は別の校正項目。
"""
from dataclasses import dataclass
import math

import numpy as np
from scipy.optimize import least_squares


@dataclass(frozen=True)
class Geometry:
    angle_rad: float
    effective_offset_mm: float

    def __post_init__(self):
        if not 0 < self.angle_rad <= math.pi / 2 or not math.isfinite(self.effective_offset_mm):
            raise ValueError('Invalid sensor geometry')


@dataclass(frozen=True)
class Observation:
    label: str
    side: int  # 1=left, 2=right
    corner_x_mm: float
    corner_y_mm: float
    displacement_x_mm: float
    displacement_y_mm: float
    gyro_angle_rad: float
    sigma_mm: float = 4.

    def __post_init__(self):
        if self.side not in (1, 2) or self.sigma_mm <= 0:
            raise ValueError('Invalid observation')
        if not all(math.isfinite(v) for v in (self.corner_x_mm, self.corner_y_mm,
                self.displacement_x_mm, self.displacement_y_mm, self.gyro_angle_rad, self.sigma_mm)):
            raise ValueError('Non-finite observation')


def state_at(initial, displacement_x_mm, displacement_y_mm, gyro_angle_rad):
    """一定の入口姿勢を、距離と相対gyroで伝播する。横滑りゼロが仮定。"""
    x0, e0, psi0 = np.asarray(initial, dtype=float)
    c, s = math.cos(psi0), math.sin(psi0)
    return np.array([x0+c*displacement_x_mm-s*displacement_y_mm,
                     e0+s*displacement_x_mm+c*displacement_y_mm, psi0+gyro_angle_rad])


def residuals(initial, observations, geometry, weighted=False):
    """柱がセンサの実効中心線上に来る条件。出力は法線方向のmm。

    左右対称の取付位置(a, +/-b)、角度+/-thetaについて、必要な
    取付位置の組はA=a-b*cot(theta)。a/bを別々に確定した意味ではない。
    """
    result = []
    for o in observations:
        sigma = 1 if o.side == 1 else -1
        x, e, psi = state_at(initial, o.displacement_x_mm, o.displacement_y_mm, o.gyro_angle_rad)
        beta = psi+sigma*geometry.angle_rad
        value = (-math.sin(beta)*(o.corner_x_mm-x)
                 + math.cos(beta)*(o.corner_y_mm-e)
                 + sigma*math.sin(geometry.angle_rad)*geometry.effective_offset_mm)
        result.append(value/o.sigma_mm if weighted else value)
    return np.array(result)


def jacobian(initial, observations, geometry, weighted=False):
    psi0 = float(initial[2]); c, s = math.cos(psi0), math.sin(psi0)
    result = []
    for o in observations:
        sign = 1 if o.side == 1 else -1
        x, e, psi = state_at(initial, o.displacement_x_mm, o.displacement_y_mm, o.gyro_angle_rad)
        beta = psi+sign*geometry.angle_rad
        dx_dpsi = -s*o.displacement_x_mm-c*o.displacement_y_mm
        de_dpsi = c*o.displacement_x_mm-s*o.displacement_y_mm
        row = np.array([math.sin(beta), -math.cos(beta),
            -math.cos(beta)*(o.corner_x_mm-x)-math.sin(beta)*(o.corner_y_mm-e)
            +math.sin(beta)*dx_dpsi-math.cos(beta)*de_dpsi])
        result.append(row/o.sigma_mm if weighted else row)
    return np.array(result)


def fit(observations, geometry):
    """最低3点から入口x/e/psiを推定。事前の画像位置・機体e/psiは使わない。"""
    if len(observations) < 3:
        return dict(valid=False, reason='fewer_than_three_events', count=len(observations))
    first = observations[0]
    x_guess = first.corner_x_mm-abs(first.corner_y_mm)/math.tan(geometry.angle_rad)-geometry.effective_offset_mm-first.displacement_x_mm
    opt = least_squares(residuals, [np.clip(x_guess, -190, 190), 0, 0],
        jac=jacobian, args=(observations, geometry, True),
        bounds=([-200, -40, -math.radians(15)], [200, 40, math.radians(15)]),
        x_scale=[40, 10, .05], ftol=1e-12, xtol=1e-12, gtol=1e-12)
    j = jacobian(opt.x, observations, geometry, True)
    scaled = j @ np.diag([40, 10, .05]); sv = np.linalg.svd(scaled, compute_uv=False)
    rank = int(np.sum(sv > max(sv)*1e-6))
    reason = 'ready' if opt.success and rank == 3 and not np.any(opt.active_mask) else ('rank_deficient' if rank < 3 else 'bound_or_solver_failure')
    covariance = np.linalg.pinv(j.T @ j) if rank == 3 else np.full((3, 3), np.nan)
    return dict(valid=reason == 'ready', reason=reason, count=len(observations), initial=opt.x.tolist(),
        residual_mm=residuals(opt.x, observations, geometry).tolist(),
        residual_rms_mm=float(np.sqrt(np.mean(residuals(opt.x, observations, geometry)**2))),
        rank=rank, scaled_singular_values=sv.tolist(), condition=float(sv[0]/sv[-1]) if rank == 3 else None,
        covariance=covariance.tolist(), e0_std_mm=float(np.sqrt(max(0, covariance[1, 1]))) if rank == 3 else None,
        psi0_std_deg=math.degrees(float(np.sqrt(max(0, covariance[2, 2])))) if rank == 3 else None)


def point_exclusion(row):
    """飽和・回転履歴などを、点の教師値として通さない。"""
    flags = int(row['flags'])
    if flags:
        return 'firmware_quality_flags_'+str(flags)
    side = 'l' if int(row['side']) == 1 else 'r'
    if float(row['s'+side+'_mm']) <= 25.5 or int(row['adc_'+side]) >= 4000:
        return 'side_floor_or_adc_saturation'
    return ''


def motion_history(times_ms, lengths_mm, velocities_mps, gyro_radps):
    """ego.lengthの座標リセットを跨ぐ距離を再構成。

    通常は生エンコーダで積算したego.lengthの差を使う。リセットを
    含む最大6msの区間だけ速度積分を用い、その箇所を必ず返す。
    """
    t, length, velocity, gyro = [np.asarray(v, dtype=float) for v in
        (times_ms, lengths_mm, velocities_mps, gyro_radps)]
    if len(t) < 2 or any(len(v) != len(t) for v in (length, velocity, gyro)):
        raise ValueError('Invalid motion arrays')
    if not all(np.isfinite(v).all() for v in (t, length, velocity, gyro)):
        raise ValueError('Non-finite motion sample')
    dt = np.diff(t)/1000
    if np.any(dt <= 0) or np.max(dt) > .0061:
        raise ValueError('Missing or unordered telemetry')
    raw = np.diff(length); velocity_step = np.maximum(0, (velocity[1:]+velocity[:-1])*.5*dt*1000)
    resets = (raw < -.5) | (raw > np.maximum(2., velocity_step*2.5+1.))
    ds = np.where(resets, velocity_step, np.maximum(0, raw))
    gamma = np.r_[0, np.cumsum((gyro[1:]+gyro[:-1])*.5*dt)]
    mid_gamma = (gamma[1:]+gamma[:-1])*.5
    return dict(time_ms=t, distance_mm=np.r_[0, np.cumsum(ds)],
        dx_mm=np.r_[0, np.cumsum(ds*np.cos(mid_gamma))],
        dy_mm=np.r_[0, np.cumsum(ds*np.sin(mid_gamma))], gamma_rad=gamma,
        reset_bridges=[dict(start_ms=float(t[i]), end_ms=float(t[i+1]), estimated_distance_mm=float(velocity_step[i]), raw_difference_mm=float(raw[i])) for i in np.flatnonzero(resets)])


def sample_time(history, time_ms):
    if not history['time_ms'][0] <= time_ms <= history['time_ms'][-1]:
        raise ValueError('Motion sample outside recorded range')
    return [float(np.interp(time_ms, history['time_ms'], history[k])) for k in ('dx_mm', 'dy_mm', 'gamma_rad')]


def sample_distance(history, distance_mm):
    if not history['distance_mm'][0] <= distance_mm <= history['distance_mm'][-1]:
        raise ValueError('Feature outside recorded distance')
    return [float(np.interp(distance_mm, history['distance_mm'], history[k])) for k in ('dx_mm', 'dy_mm', 'gamma_rad')]


def constrain_gyro_integrals(history, snapshots, available_ms):
    """確認済みの1ms積算角度差で、5ms gyro再積分の総和誤差を補う。

    区間内の角速度波形そのものは復元できない。角度差の誤差を過去の
    スナップショット間で線形配分し、最後から先は既知の補正量を保持。
    カメラ座標と、available_msより後に確認された値は使わない。
    """
    observed = sorted((s for s in snapshots if s['confirmation_ms'] <= available_ms), key=lambda s: s['minimum_ms'])
    result = dict(history)
    correction = np.zeros(len(history['time_ms']))
    if len(observed) >= 2:
        first = observed[0]; times = np.array([s['minimum_ms'] for s in observed])
        target = np.array([s['angle_rad']-first['angle_rad'] for s in observed])+np.interp(first['minimum_ms'], history['time_ms'], history['gamma_rad'])
        errors = target-np.interp(times, history['time_ms'], history['gamma_rad'])
        correction = np.interp(history['time_ms'], times, errors, left=0., right=errors[-1])
    gamma = history['gamma_rad']+correction; ds=np.diff(history['distance_mm']); mid=(gamma[:-1]+gamma[1:])*.5
    result.update(gamma_rad=gamma, dx_mm=np.r_[0,np.cumsum(ds*np.cos(mid))], dy_mm=np.r_[0,np.cumsum(ds*np.sin(mid))],
        integral_constraints=len(observed), maximum_correction_deg=float(np.degrees(abs(correction).max())))
    return result


def ekf_replay(timed_observations, history, geometry, prior, prior_time_ms,
               prior_available_ms, query_ms, lateral_walk=.15, yaw_walk_deg=.03):
    """確認済み観測だけで固定遅れ再生。元の特徴点時刻で更新し再伝播する。

    priorは既存機体推定を使う仮の事前分布。画像や未確認エッジを使わない。
    確認が来る前の制御へ過去の補正を適用した、とは扱わない。
    """
    if query_ms < prior_available_ms:
        return None
    x = np.asarray(prior, float).copy(); p = np.diag([60.**2, 15.**2, math.radians(5.)**2])
    current_time = float(prior_time_ms); records = []
    def propagate(end):
        nonlocal x, p, current_time
        if end < current_time-1e-6: raise ValueError('Out-of-order feature times')
        knots = np.r_[current_time, history['time_ms'][(history['time_ms'] > current_time) & (history['time_ms'] < end)], end]
        for a, b in zip(knots, knots[1:]):
            ds = float(np.interp(b, history['time_ms'], history['distance_mm'])-np.interp(a, history['time_ms'], history['distance_mm']))
            dg = float(np.interp(b, history['time_ms'], history['gamma_rad'])-np.interp(a, history['time_ms'], history['gamma_rad']))
            mid = x[2]+dg*.5
            f = np.array([[1, 0, -ds*math.sin(mid)], [0, 1, ds*math.cos(mid)], [0, 0, 1.]])
            q = np.diag([.1**2*ds, lateral_walk**2*ds, math.radians(yaw_walk_deg)**2*ds])
            x += [ds*math.cos(mid), ds*math.sin(mid), dg]; p = f@p@f.T+q
        current_time = end
    available = [v for v in timed_observations if v['confirmation_ms'] <= query_ms]
    for item in sorted(available, key=lambda v: v['feature_ms']):
        propagate(item['feature_ms']); o = item['observation']
        pose_observation = Observation(o.label, o.side, o.corner_x_mm, o.corner_y_mm, 0, 0, 0, o.sigma_mm)
        h = jacobian(x, [pose_observation], geometry)[0]
        innovation = -float(residuals(x, [pose_observation], geometry)[0])
        variance = float(h@p@h+o.sigma_mm**2); nis = innovation**2/variance
        accepted = nis <= 9.
        if accepted:
            gain = p@h/variance; x += gain*innovation
            a = np.eye(3)-np.outer(gain, h); p = a@p@a.T+np.outer(gain, gain)*o.sigma_mm**2
        records.append(dict(pillar=o.label, feature_ms=item['feature_ms'], confirmation_ms=item['confirmation_ms'],
                            innovation_normal_mm=innovation, nis=nis, accepted=accepted))
    propagate(query_ms)
    count = sum(r['accepted'] for r in records)
    return dict(state=x.tolist(), covariance=p.tolist(), events=records, considered=len(records), accepted=count,
        ready=count >= 3, e_std_mm=float(math.sqrt(max(0, p[1, 1]))), psi_std_deg=math.degrees(math.sqrt(max(0, p[2, 2]))),
        maximum_confirmation_ms=max((v['confirmation_ms'] for v in records), default=prior_available_ms))


def main():
    import argparse
    import csv
    import hashlib
    import json
    from pathlib import Path
    import shutil

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--review', type=Path, required=True, help='前回のpillar_comparison.csv/cases.jsonを含む固定済み解析')
    parser.add_argument('--photo-geometry', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True, help='未作成の出力フォルダー')
    parser.add_argument('--effective-offset-mm', type=float, default=20., help='仮のA=a-b*cot(theta)。未校正のため0..40mmも比較する')
    args = parser.parse_args()
    out = args.output.resolve(); out.mkdir(parents=True, exist_ok=False)
    read = lambda p: json.loads(Path(p).read_text(encoding='utf-8-sig'))
    def rows(p):
        with Path(p).open(encoding='utf-8-sig', newline='') as source:
            return list(csv.DictReader(source))
    def save(name, value):
        def clean(v):
            if isinstance(v, dict): return {k: clean(x) for k, x in v.items()}
            if isinstance(v, (list, tuple)): return [clean(x) for x in v]
            if isinstance(v, (float, np.floating)) and not math.isfinite(v): return None
            if isinstance(v, np.generic): return v.item()
            return v
        (out/name).write_text(json.dumps(clean(value), ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    def write(name, data):
        if not data: return
        with (out/name).open('w', encoding='utf-8-sig', newline='') as f:
            w = csv.DictWriter(f, list(dict.fromkeys(k for r in data for k in r)))
            w.writeheader(); w.writerows(data)
    def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
    review = args.review.resolve(); photo = read(args.photo_geometry)
    angle_deg = float(photo['angle_for_shadow_deg']); base_geometry = Geometry(math.radians(angle_deg), args.effective_offset_mm)
    cases = read(review/'cases.json'); comparison = rows(review/'pillar_comparison.csv')
    previous_provenance = {r['run']: r for r in read(review/'deep_provenance.json')}
    source_manifest = []; predictions = []; summaries = []; sweeps = []; prefix_results = []; ekf_audits = []; gyro_audits = []
    (out/'inputs').mkdir()
    for source in (review/'pillar_comparison.csv', review/'map_verified.json', review/'deep_summary.json', review/'deep_provenance.json', args.photo_geometry):
        shutil.copyfile(source, out/'inputs'/source.name)
    shutil.copyfile(review/'cases.json',out/'inputs/source_cases.json')
    frozen_cases=[dict(c,input_directory=str(out/'inputs'/c['name'])) for c in cases]
    (out/'inputs/cases.json').write_text(json.dumps(frozen_cases,ensure_ascii=False,indent=2),encoding='utf-8')
    photo_image = args.photo_geometry.with_name('sensor_mount_angles.png')
    if photo_image.exists(): shutil.copyfile(photo_image, out/'sensor_mount_angles.png')
    shutil.copyfile(Path(__file__), out/'analysis_source.py')
    forward = np.array([-1, 1])/math.sqrt(2); left = np.array([1, 1])/math.sqrt(2); anchor = np.array([4.5, 1.])
    for case in cases:
        name = case['name']; inp = Path(case['input_directory']); setting = read(inp/'settings.json')
        if setting.get('firmware', {}).get('mouse') != 'A': raise ValueError('This input review is for machine A only')
        for filename in ('telemetry.csv', 'settings.json'):
            if sha(inp/filename) != previous_provenance[name]['input_sha256'][filename]:
                raise ValueError('Frozen input changed: '+str(inp/filename))
        frozen = out/'inputs'/name; frozen.mkdir()
        for filename in ('telemetry.csv', 'settings.json', 'wall_corrections.csv'):
            shutil.copyfile(inp/filename, frozen/filename)
        events = [r for r in comparison if r['run'] == name]
        if len(events) != 6 or [r['pillar'] for r in events] != ['R1', 'L2', 'R3', 'L4', 'R5', 'L6']:
            raise ValueError('Physical pillar association must be reviewed first')
        t0 = min(float(r['minimum_ms']) for r in events)-20
        t1 = max(float(r['ms']) for r in events)+20
        tr = rows(frozen/'telemetry.csv')
        if any(int(b['sequence'])-int(a['sequence']) != 1 for a,b in zip(tr,tr[1:])):
            raise ValueError('Telemetry sequence gap')
        begin = max(0, next(i for i, r in enumerate(tr) if float(r['time_ms']) >= t0)-1)
        end = next(i for i, r in enumerate(tr) if float(r['time_ms']) >= t1)+1
        tr = tr[begin:end]
        history = motion_history(*[[float(r[key]) for r in tr] for key in ('time_ms', 'ego.length', 'ego.velo', 'ego.rad_velo')])
        snapshots=[dict(minimum_ms=float(r['minimum_ms']), confirmation_ms=float(r['ms']), angle_rad=float(r['model_psi_rad'])) for r in events]
        first_snapshot,last_snapshot=snapshots[0],snapshots[-1]
        raw_angle=float(np.interp(last_snapshot['minimum_ms'],history['time_ms'],history['gamma_rad'])-np.interp(first_snapshot['minimum_ms'],history['time_ms'],history['gamma_rad']))
        integrated_angle=last_snapshot['angle_rad']-first_snapshot['angle_rad']
        gyro_audits.append(dict(run=name, pc_5ms_integral_deg=math.degrees(raw_angle), mcu_1ms_integral_deg=math.degrees(integrated_angle),difference_deg=math.degrees(raw_angle-integrated_angle)))
        # Use only reviewed real event IDs; no "divide a gap by 3" recovery.
        differences = {(r['ms'], r['side']): r for r in rows(frozen/'wall_corrections.csv')}
        source_manifest.append(dict(run=name, mouse='A', telemetry_sha256=sha(frozen/'telemetry.csv'), settings_sha256=sha(frozen/'settings.json'),
            sequence_range=[tr[0]['sequence'], tr[-1]['sequence']], gyro_period_ms='actual time_ms, max 6 ms', reset_bridges=history['reset_bridges']))
        for kind in ('minimum', 'difference'):
            observations = []; metadata = []
            for event in events:
                exclusion = point_exclusion(event); confirmation = float(event['ms'])
                if kind == 'minimum':
                    tm = float(event['minimum_ms'])
                    distance = float(np.interp(tm, history['time_ms'], history['distance_mm']))+float(event['feature_mm'])-float(event['minimum_length_mm'])
                else:
                    key = (event['difference_event_ms'], event['side'])
                    if not key[0] or key not in differences:
                        metadata.append(dict(event=event, exclusion='no_reviewed_difference_event', observation=None, confirmation_ms=None)); continue
                    w = differences[key]; confirmation = float(w['ms'])
                    distance = float(np.interp(confirmation, history['time_ms'], history['distance_mm']))-float(w['delay_mm'])
                    if int(w['position_quality_flags']): exclusion = 'difference_quality_flags_'+w['position_quality_flags']
                dx, dy, gamma = sample_distance(history, distance)
                point = (np.array([float(event['pillar_x']), float(event['pillar_y'])])-anchor)*90
                std = math.sqrt(4**2+float(event['width_mm'])**2/12)*math.sin(base_geometry.angle_rad)
                o = Observation(event['pillar'], int(event['side']), float(point@forward), float(point@left), dx, dy, gamma, std)
                metadata.append(dict(event=event, exclusion=exclusion, observation=o, confirmation_ms=confirmation,
                    feature_distance_mm=distance, feature_time_interpolated_ms=float(np.interp(distance, history['distance_mm'], history['time_ms']))))
                if not exclusion: observations.append(o)
            solved = fit(observations, base_geometry)
            entry = dict(run=name, profile=case['options']['profile'], mode=case['options']['wall_ekf_mode'], kind=kind,
                quality_events=len(observations), fit=solved, excluded={m['event']['pillar']: m['exclusion'] for m in metadata if m['exclusion']})
            summaries.append(entry)
            eligible = [m for m in metadata if not m['exclusion']]
            ekf_prior = None
            if eligible:
                first = min(eligible, key=lambda m: m['feature_time_interpolated_ms']); start = first['feature_time_interpolated_ms']
                sample_ms = float(first['event']['minimum_ms'])
                dgamma = float(np.interp(start, history['time_ms'], history['gamma_rad'])-np.interp(sample_ms, history['time_ms'], history['gamma_rad']))
                psi_prior = float(first['event']['model_psi_rad'])+dgamma
                ds_prior = float(np.interp(start, history['time_ms'], history['distance_mm'])-np.interp(sample_ms, history['time_ms'], history['distance_mm']))
                ekf_prior = [0., float(first['event']['model_e_mm'])+ds_prior*math.sin(psi_prior), psi_prior]
                timed_obs = [dict(observation=m['observation'], feature_ms=m['feature_time_interpolated_ms'], confirmation_ms=m['confirmation_ms']) for m in eligible]
            def kalman_at(query, integral=False):
                if ekf_prior is None:return None
                h=constrain_gyro_integrals(history,snapshots,query) if integral else history
                gamma_delta=float(np.interp(start,h['time_ms'],h['gamma_rad'])-np.interp(sample_ms,h['time_ms'],h['gamma_rad']))
                psi=float(first['event']['model_psi_rad'])+gamma_delta
                prior=[0.,float(first['event']['model_e_mm'])+ds_prior*math.sin(psi),psi]
                return ekf_replay(timed_obs,h,base_geometry,prior,start,first['confirmation_ms'],query)
            accepted = []; available = []; heldout = {}
            for m in sorted(metadata, key=lambda v: v['confirmation_ms'] if v['confirmation_ms'] is not None else math.inf):
                if m['exclusion']: continue
                prior = available[-1]['fit'] if available else None
                if prior and prior['valid']:
                    heldout[m['event']['pillar']] = float(residuals(prior['initial'], [m['observation']], base_geometry)[0])
                accepted.append(m['observation']); current = fit(accepted, base_geometry)
                rec = dict(run=name, kind=kind, through=m['event']['pillar'], available_ms=m['confirmation_ms'], fit=current)
                available.append(rec); prefix_results.append(rec)
            for m in metadata:
                r = m['event']; camera_time = float(r['minimum_ms'])+float(r['camera_gap_ms'])
                camera_motion = sample_time(history, camera_time)
                row = dict(run=name, profile=entry['profile'], mode=entry['mode'], kind=kind, pillar=r['pillar'], side=r['side'],
                    point_exclusion=m['exclusion'], event_confirmation_ms=m['confirmation_ms'], camera_time_ms=camera_time,
                    camera_qualified=r['camera_grid_refined']=='True', camera_method=r['camera_method'],
                    camera_e_mm=r['camera_left_mm'], camera_psi_deg=r['camera_psi_left_deg'],
                    mcu_e_mm=r['model_e_mm'], mcu_psi_deg=r['model_psi_left_deg'],
                    floor_range_mm=r['sl_mm' if r['side']=='1' else 'sr_mm'], width_mm=r['width_mm'])
                if m['observation']:
                    row.update(feature_distance_mm=m['feature_distance_mm'], feature_time_interpolated_ms=m['feature_time_interpolated_ms'])
                if solved['valid']:
                    x, e, psi = state_at(solved['initial'], *camera_motion)
                    row.update(retrospective_x_mm=x, retrospective_e_mm=e, retrospective_psi_deg=math.degrees(psi))
                previous = [v for v in available if v['available_ms'] <= camera_time and v['fit']['valid']]
                if previous:
                    used = previous[-1]; x, e, psi = state_at(used['fit']['initial'], *camera_motion)
                    row.update(causal_through=used['through'], causal_available_ms=used['available_ms'], causal_e_mm=e, causal_psi_deg=math.degrees(psi))
                if r['pillar'] in heldout: row['heldout_normal_mm'] = heldout[r['pillar']]
                for variant in ('ekf','ekf_integral'):
                    kalman=kalman_at(camera_time,variant=='ekf_integral')
                    if kalman:
                        ekf_audits.append(dict(run=name, kind=kind, variant=variant, camera_pillar=r['pillar'], query_ms=camera_time, **kalman))
                        row.update({variant+'_edges':kalman['accepted'],variant+'_ready':kalman['ready'],variant+'_e_std_mm':kalman['e_std_mm'],variant+'_psi_std_deg':kalman['psi_std_deg']})
                        if kalman['ready']:
                            row.update({variant+'_e_mm':kalman['state'][1],variant+'_psi_deg':math.degrees(kalman['state'][2])})
                    if not m['exclusion']:
                        before=kalman_at(m['feature_time_interpolated_ms'],variant=='ekf_integral')
                        if before and before['ready']:
                            o=m['observation'];point=Observation(o.label,o.side,o.corner_x_mm,o.corner_y_mm,0,0,0,o.sigma_mm)
                            row[variant+'_heldout_normal_mm']=float(residuals(before['state'],[point],base_geometry)[0])
                predictions.append(row)
            for angle in photo['sensitivity_angles_deg']:
                for offset in (0., 10., 20., 30., 40.):
                    g = Geometry(math.radians(angle), offset); solution = fit(observations, g)
                    sr = dict(run=name, kind=kind, angle_deg=angle, offset_mm=offset, valid=solution['valid'], reason=solution['reason'])
                    if solution['valid']:
                        sr.update(e0_mm=solution['initial'][1], psi0_deg=math.degrees(solution['initial'][2]), residual_rms_mm=solution['residual_rms_mm'])
                    sweeps.append(sr)
        print(name, [(r['kind'], r['quality_events'], r['fit']['reason'], round(r['fit'].get('residual_rms_mm', -1), 2)) for r in summaries if r['run']==name], flush=True)
    write('predictions.csv', predictions); write('geometry_sweep.csv', sweeps)
    save('fits.json', summaries); save('prefix_fits.json', prefix_results)
    save('ekf_audits.json', ekf_audits)
    save('gyro_integral_comparison.json',gyro_audits)
    save('provenance.json', dict(input_review=str(review), photo_geometry=str(args.photo_geometry.resolve()), photo_sha256=photo['source_sha256'],
        review_files_sha256={p.name:sha(p) for p in (review/'pillar_comparison.csv',review/'map_verified.json',review/'cases.json',args.photo_geometry)},
        source_sha256=sha(Path(__file__)), sources=source_manifest, trajectory_video_in_estimation=False, trajectory_video_in_geometry_selection=False,sensor_photo_for_geometry=True,
        baseline_geometry=dict(angle_deg=angle_deg, effective_offset_mm=args.effective_offset_mm, calibrated=False),
        estimation_kind='Blind sequential nonlinear geometry fit and a separate offline delayed-observation EKF replay; no firmware implementation',
        ekf_prior='First eligible MCU model e/psi snapshot, available only at its confirmation; std x=60mm, e=15mm, psi=5deg',
        ekf_process_noise='std per sqrt(mm): x=.1mm, e=.15mm, psi=.03deg; fixed before comparison, not tuned to camera',
        integral_constrained_variant='Only already-confirmed model_psi snapshots constrain relative 1ms gyro integrals. Unknown within-interval waveforms are not recovered.',
        motion_time_interpolation='Offline linear interpolation between 5ms samples may use the next sample within 6ms. Confirmed-event gating does not establish an exact 1ms online replay.',
        current_state_validity='Causal estimates use only events confirmed by the camera measurement time; retrospective estimates are separate.',
        assumptions=['no lateral slip','gyro between 5ms telemetry samples interpolated','A/beam angle is an effective point-feature model, not optical calibration'],
        firmware_changed=False, parameters_adopted=False))
    make_review(out, predictions, summaries, sweeps, photo)


def make_review(out, predictions, summaries, sweeps, photo):
    """映像を推定後の比較にだけ使い、事後解と因果的な推定を区別する。"""
    import html
    import json
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from collections import defaultdict

    def stats(values):
        if not values: return dict(n=0)
        a = np.asarray(values, float)
        return dict(n=len(a), bias=float(a.mean()), mae=float(abs(a).mean()), rmse=float(np.sqrt(np.mean(a*a))), maximum=float(abs(a).max()))
    evaluations = {}
    for kind in ('minimum', 'difference'):
        for mode in ('causal', 'retrospective', 'ekf', 'ekf_integral'):
            rr = [r for r in predictions if r['kind']==kind and r['camera_qualified'] and mode+'_e_mm' in r]
            automatic = [r for r in rr if r['camera_method']=='automatic_image_pair']
            evaluations[kind+'_'+mode] = dict(e_mm=stats([r[mode+'_e_mm']-float(r['camera_e_mm']) for r in rr]),
                psi_deg_automatic_only=stats([math.degrees(math.atan2(math.sin(math.radians(r[mode+'_psi_deg']-float(r['camera_psi_deg']))), math.cos(math.radians(r[mode+'_psi_deg']-float(r['camera_psi_deg']))))) for r in automatic]),
                mcu_e_mm_same_points=stats([float(r['mcu_e_mm'])-float(r['camera_e_mm']) for r in rr]),
                mcu_psi_deg_same_automatic_points=stats([float(r['mcu_psi_deg'])-float(r['camera_psi_deg']) for r in automatic]),
                heldout_normal_mm=stats([r[mode+'_heldout_normal_mm' if mode.startswith('ekf') else 'heldout_normal_mm'] for r in predictions if r['kind']==kind and (mode+'_heldout_normal_mm' if mode.startswith('ekf') else 'heldout_normal_mm') in r]))
    (out/'evaluation.json').write_text(json.dumps(evaluations, ensure_ascii=False, indent=2), encoding='utf-8')
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    for ki, kind in enumerate(('minimum', 'difference')):
        for ci, key in enumerate(('e_mm', 'psi_deg')):
            ax=axes[ki, ci]
            limits_data=[]
            for prefix, marker, label in [('retrospective','o','all-event fit (retrospective)'),('causal','x','blind fit, confirmed events'),('ekf_integral','^','EKF, confirmed 1ms angle totals')]:
                rr=[r for r in predictions if r['kind']==kind and r['camera_qualified'] and prefix+'_'+key in r
                    and (key!='psi_deg' or r['camera_method']=='automatic_image_pair')]
                ax.scatter([float(r['camera_'+key]) for r in rr],[r[prefix+'_'+key] for r in rr],s=30,marker=marker,label=label,alpha=.75)
                limits_data.extend(float(r['camera_'+key]) for r in rr);limits_data.extend(r[prefix+'_'+key] for r in rr)
            extent=max(15 if key=='e_mm' else 5, math.ceil(max(map(abs,limits_data),default=0)+1))
            limits=(-extent,extent)
            ax.plot(limits,limits,ls='--',c='#999');ax.set(xlim=limits,ylim=limits,xlabel='Camera '+key,ylabel='Geometry '+key,title=kind)
            ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.tight_layout();fig.savefig(out/'camera_comparison.png',dpi=160);plt.close(fig)
    fig, ax=plt.subplots(figsize=(11,4))
    names=list(dict.fromkeys(r['run'] for r in predictions))
    for i,name in enumerate(names):
        rr=[r for r in sweeps if r['kind']=='minimum' and r['run']==name and r['valid'] and r['offset_mm']==20]
        if rr:ax.plot([r['angle_deg'] for r in rr],[r['e0_mm'] for r in rr],marker='o',label=name[4:10])
    ax.set(xlabel='Assumed effective sensor angle [deg]',ylabel='Estimated entrance lateral e [mm]',title='Angle sensitivity, no camera used for model selection');ax.grid(alpha=.2);ax.legend(ncol=4,fontsize=8)
    fig.tight_layout();fig.savefig(out/'angle_sensitivity.png',dpi=160);plt.close(fig)
    def number(x):return '—' if x is None else f'{float(x):.2f}'
    def table(headers, rr):return '<div class="scroll"><table><tr>'+''.join('<th>'+str(x)+'</th>' for x in headers)+'</tr>'+''.join('<tr>'+''.join('<td>'+str(x)+'</td>' for x in r)+'</tr>' for r in rr)+'</table></div>'
    minimum_ekf=evaluations['minimum_ekf_integral']
    content=['<h1>左右の斜め特徴点による横位置・角度のShadow検証</h1>',
        '<p>機体A・変更後の9×9迷路・保存済み7走行。実機走行、ファーム書込み、Params変更なし。センサ写真から約55度を基準にしたが、光軸・取付位置・特徴点の位相は未校正。画像位置は推定器へ入力せず、推定後の評価だけに使った。</p>',
        '<p style="background:#fff0d4;padding:16px"><b>判断：</b>幾何推定は5/7走行で解けたが、2つの高速走行は観測品質不足。確認済みの積算角を使ったEKFと映像を比較できたのは横位置7点・角度6点で、横MAE '+number(minimum_ekf['e_mm'].get('mae'))+'mm、角度MAE '+number(minimum_ekf['psi_deg_automatic_only'].get('mae'))+'度。同じ点の既存MCU横MAEは'+number(minimum_ekf['mcu_e_mm_same_points'].get('mae'))+'mm。横位置の改善は確認できていない。55度の取付角だけで、左右の実効検出位相まで校正済みにはならない。</p>',
        '<h2>写真で測った取付方向</h2><img src="sensor_mount_angles.png"><p>赤マーカー側が前。写真の右側が機体左。斜めの組は左右約55度、外側の組は前方から約5〜8度。白い部品の長辺に垂直な方向を仮の光軸とし、4個の矩形を使った局所的な斜視補正を行った。単一写真から真の光学的なピーク方向まで確定したものではない。取付角の手読みに由来する幅は概ね51〜59度で、モデル全体の誤差範囲ではない。</p>',
        '<p>左右対称の点センサ・角度ずれゼロという理想モデルでは、交互間隔のずれは約2*cot(theta)*e。55度では係数が約1.40となり、同じ間隔差から求める横ずれは45度モデルの約1.43倍になる。この比を現行補正へ単純に掛けるのではなく、特徴点別の位相・角度との連成を含む観測式で検証した。</p>',
        '<h2>推定方法と使用条件</h2><p>状態は入口の前方位置x・左正の横位置e・左正の角度psi。相対ジャイロとエンコーダ距離で伝播し、既知柱が左右センサの実効中心線に来る条件を時系列で観測する。未知の前後・左右位置は A=a−b*cot(theta) とまとめ、仮に20mm、感度確認は0〜40mmとした。左右別の未校正オフセットは絶対横位置と分離できないので、絶対補正値の採用にはならない。</p>',
        '<p>最低3イベントが必要。2点だけ・同側だけ・真横90度の点センサでは、横位置と角度を十分に分離できない場合を単体テストで確認した。回転履歴フラグ・距離下限・ADC飽和は点の観測から除外した。検出確認より前にその観測を使わない。全イベントを使った事後解は、因果的なShadow解と別に表示する。極小と差分交差を混ぜていない。</p>',
        '<p>さらにPythonのEKFで再生した。初期横位置・角度は最初の品質通過イベントに保存されたMCUモデル値を使い、その確認時刻より前には利用しない。事前標準偏差は縦60mm・横15mm・角度5度。距離当たりの過程雑音はsqrt(mm)当たり縦0.1mm・横0.15mm・角度0.03度、観測は進行方向4mm相当を仮定した。画像に合わせて雑音を調整していない。確認済みの観測を特徴点時刻へ戻して更新し、評価時刻まで再伝播する。3イベント以上が採用された後を評価する。ファームへの反映・実走による改善確認ではない。</p>',
        '<p>高速の5ms角速度再積分は、機体が1msで積算したmodel_psiの差から最大約2.5度ずれた。そのためekf_integralでは、確認済みの角度スナップショット間の積算量で過去の角度を補って再生した。未確認の将来値は使わない。区間内の1ms波形を復元した意味ではない。<a href="gyro_integral_comparison.json">積算量の比較</a>。</p>',
        '<p>運動ログ自体は5ms記録の時刻補間であり、評価時刻より最大6ms後の運動サンプルを補間に使う場合がある。「確認済みのみ」はエッジ・角度スナップショットの採用時刻についての保証で、実機の1msオンライン処理を厳密再現した意味ではない。角度や位置の数値差が小さいことから実走改善を断定しない。</p>',
        '<h2>走行ごとの成立条件</h2>',
        table(['走行','特徴点','使用数','判定','法線残差RMS mm','入口e mm','入口psi deg','e標準偏差mm','psi標準偏差deg'],[[s['run'][4:10],s['kind'],s['quality_events'],s['fit']['reason'],number(s['fit'].get('residual_rms_mm')),number(s['fit'].get('initial',[None]*3)[1]),number(math.degrees(s['fit']['initial'][2])) if s['fit'].get('initial') else '—',number(s['fit'].get('e0_std_mm')),number(s['fit'].get('psi0_std_deg'))] for s in summaries]),
        '<p>誤差・標準偏差は、この仮の幾何・横滑りゼロ・観測ノイズ仮定の内部値。未校正のセンサ偏りやスリップまで含む保証値ではない。品質除外のため解けない走行をゼロへ置き換えない。</p>',
        '<h2>映像との比較</h2><img src="camera_comparison.png">',
        table(['特徴点 / 評価','e比較数','幾何e MAE mm','同じ点のMCU e MAE mm','自動画像の角度比較数','幾何psi MAE deg','同じ点のMCU psi MAE deg','未使用次イベント数','次イベント法線MAE mm'],[[key,v['e_mm']['n'],number(v['e_mm'].get('mae')),number(v['mcu_e_mm_same_points'].get('mae')),v['psi_deg_automatic_only']['n'],number(v['psi_deg_automatic_only'].get('mae')),number(v['mcu_psi_deg_same_automatic_points'].get('mae')),v['heldout_normal_mm']['n'],number(v['heldout_normal_mm'].get('mae'))] for key,v in evaluations.items()]),
        '<p>角度の数値評価は自動で端点が得られた画像だけを使う。横位置は手動画像を含むが、毎フレームの盤面格子品質を通った点のみ。カメラの高さ・車軸位置・マーカー取付角は未校正。同期は前回の画像角度差とgyroによる値を引き継ぎ、今回空間座標を回転・平行移動して誤差を消していない。</p>',
        '<h2>角度・取付位置の未確定さ</h2><img src="angle_sensitivity.png"><p>45/50/55/60度、A=0/10/20/30/40mmをすべて記録した。映像に合うものを選んで採用していない。柱の接続壁による光学ピークの偏りはこのモデルに残る。相対位置の追従が良くても、絶対横位置と機体角度の校正完了とはしない。</p>',
        '<p><a href="predictions.csv">各柱・確認時刻・推定・画像の対応</a> / <a href="geometry_sweep.csv">全感度計算</a> / <a href="fits.json">解と除外理由</a> / <a href="prefix_fits.json">確認後の逐次更新</a> / <a href="ekf_audits.json">EKFの採用・NIS・共分散</a> / <a href="evaluation.json">評価値</a> / <a href="provenance.json">入力・仮定・SHA256</a></p>']
    page='<!doctype html><html lang="ja"><meta charset="utf-8"><title>斜め柱 幾何Shadow検証</title><style>body{max-width:1200px;margin:30px auto;padding:0 20px;font:16px/1.8 system-ui;color:#243542}img{max-width:100%}table{border-collapse:collapse;font-size:13px}td,th{border:1px solid #ccd4df;padding:6px;white-space:nowrap}.scroll{overflow:auto}h2{margin-top:2em}</style>'+''.join(content)+'</html>'
    (out/'report.html').write_text(page,encoding='utf-8')
    print(json.dumps(evaluations,ensure_ascii=False,indent=2))


if __name__ == '__main__':
    main()
