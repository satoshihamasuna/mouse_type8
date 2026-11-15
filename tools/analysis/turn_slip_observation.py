"""保存済み動画・ジャイロ・横加速度から機体A turn500の横滑りを再評価する。

モデルが計算したego.turn_slip_thetaを教師値には使わない。
前後の直進を使うジャイロ尺度校正と、同期・微分窓の感度を分けて保存する。
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.integrate import cumulative_trapezoid
from scipy.optimize import least_squares, minimize_scalar
from scipy.signal import lfilter


ROOT = Path(__file__).resolve().parents[2]


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def local_polynomial(time, xy, valid, half_window_s=.025):
    """実PTSで局所3次式を解く。20 ms超の欠測をまたぐ点は採用しない。"""
    time, xy, valid = np.asarray(time), np.asarray(xy), np.asarray(valid, bool)
    result = np.full((len(time), 3, 2), np.nan)
    source = np.flatnonzero(valid & np.isfinite(xy).all(axis=1))
    if len(source) < 7 or not np.isfinite(time).all() or np.any(np.diff(time) <= 0):
        raise ValueError("Insufficient observations or non-monotonic video PTS")
    observed_time = time[source]
    for i in np.flatnonzero(valid):
        a, b = np.searchsorted(observed_time, [time[i] - half_window_s, time[i] + half_window_s])
        indices = source[a:b]
        if (len(indices) < 7 or np.max(np.diff(time[indices])) > .020
                or time[i] - time[indices[0]] < half_window_s * .6
                or time[indices[-1]] - time[i] < half_window_s * .6):
            continue
        u = (time[indices] - time[i]) / half_window_s
        coeff = np.linalg.lstsq(np.vander(u, 4, increasing=True), xy[indices], rcond=None)[0]
        result[i, 0] = coeff[0]
        result[i, 1] = coeff[1] / half_window_s
        result[i, 2] = 2 * coeff[2] / half_window_s ** 2
    return result


def heading_from_velocity(time, derivatives, raw, shift_s=0.):
    """右回り正。yawは実測角速度を連続積分し、区間リセット列は使わない。"""
    rt = raw.elapsed_s.to_numpy()
    wc = -raw["ideal.rad_velo"].to_numpy()
    w = -raw["ego.rad_velo"].to_numpy()
    commanded_v = raw["ideal.velo"].to_numpy()
    peak = float(np.max(abs(wc)))
    turn = abs(wc) > .02 * peak
    first, last = rt[turn][[0, -1]]
    stationary = ((rt < -.04) | (rt > raw.attrs["motion_end_s"] + .2)) & (abs(raw["ego.velo"].to_numpy()) < .02)
    if stationary.sum() < 20:
        raise ValueError("Too few stationary gyro samples")
    bias = float(np.median(w[stationary]))
    integral = cumulative_trapezoid(w - bias, rt, initial=0)
    observed = np.interp(time + shift_s, rt, integral)
    speed = np.linalg.norm(derivatives[:, 1], axis=1) / 1000
    moving = np.isfinite(speed) & (speed > commanded_v.max() * .5)
    course = np.full(len(time), np.nan)
    course[moving] = np.unwrap(np.arctan2(derivatives[moving, 1, 0], derivatives[moving, 1, 1]))
    vc = np.interp(time, rt, commanded_v)
    entry = moving & (time > max(.03, first - .18)) & (time < first - .035) & (vc > .85 * commanded_v.max())
    exit_mask = moving & (time > last + .035) & (vc > .85 * commanded_v.max()) & (time < raw.attrs["motion_end_s"] - .04)
    if entry.sum() < 8 or exit_mask.sum() < 6:
        raise ValueError(f"Too few straight samples: entry={entry.sum()}, exit={exit_mask.sum()}")
    course0, yaw0 = float(np.median(course[entry])), float(np.median(observed[entry]))
    delta_course = float(np.median(course[exit_mask]) - course0)
    delta_yaw = float(np.median(observed[exit_mask]) - yaw0)
    gain = delta_course / delta_yaw
    if not .9 < gain < 1.1:
        raise ValueError(f"Gyro scale inconsistent with straight directions: {gain}")
    psi_raw = observed - yaw0 + course0
    psi_cal = gain * (observed - yaw0) + course0
    yaw = np.interp(time, rt, wc)
    core = moving & (abs(yaw) > .5 * peak)
    fit_mask = moving & (time > first - .01) & (time < last + .03) & (abs(yaw) > .1 * peak)
    sign = float(np.sign(delta_yaw))
    return dict(course=course, psi_raw=psi_raw, psi_cal=psi_cal, beta_raw=course-psi_raw, beta=course-psi_cal,
                core=core, fit_mask=fit_mask, entry=entry, exit=exit_mask, speed=speed,
                gyro_bias_rad_s=bias, gyro_gain=gain, raw_angle_deg=float(np.degrees(delta_yaw)),
                path_angle_deg=float(np.degrees(delta_course)), turn_first_s=float(first), turn_last_s=float(last),
                turn_sign=sign, moving=moving)


def beta_model(time, raw, tau, gyro_gain, gyro_bias):
    rt = raw.elapsed_s.to_numpy()
    grid = np.arange(rt[0], rt[-1] + .0001, .001)
    omega = gyro_gain * np.interp(grid, rt, -raw["ego.rad_velo"].to_numpy() - gyro_bias)
    if tau <= 1e-8:
        return np.zeros(len(time))
    decay = np.exp(-.001 / tau)
    beta = -tau * lfilter([1 - decay], [1, -decay], omega)
    return np.interp(time+raw.attrs.get("heading_shift_s",0.), grid, beta)


def fit_tau(time, heading, raw):
    mask = heading["fit_mask"] & np.isfinite(heading["beta"])
    def error(tau):
        simulated = beta_model(time, raw, tau, heading["gyro_gain"], heading["gyro_bias_rad_s"])
        residual = np.degrees(simulated[mask] - heading["beta"][mask])
        return float(np.mean(2 * (np.sqrt(1 + residual ** 2) - 1)))
    fit = minimize_scalar(error, bounds=(0, .05), method="bounded", options=dict(xatol=1e-7))
    tau = float(fit.x) if error(fit.x) < error(0) else 0.
    simulated = beta_model(time, raw, tau, heading["gyro_gain"], heading["gyro_bias_rad_s"])
    return tau, simulated, float(np.sqrt(np.mean(np.degrees(simulated[mask]-heading["beta"][mask]) ** 2)))


def compare_point_offset(time,heading,raw):
    """横滑りと目印中点の車軸からの前後ずれを、同じ観測で比較する。"""
    rt=raw.elapsed_s.to_numpy()
    query=time+raw.attrs.get("heading_shift_s",0.)
    omega=heading["gyro_gain"]*np.interp(query,rt,-raw["ego.rad_velo"]-heading["gyro_bias_rad_s"])
    velocity=np.maximum(.1,np.interp(query,rt,raw["ego.velo"]))
    mask=heading["fit_mask"] & np.isfinite(heading["beta"])
    target=heading["beta"][mask]
    def predicted(tau,offset_mm):
        slip=beta_model(time,raw,tau,heading["gyro_gain"],heading["gyro_bias_rad_s"])
        return np.arctan(np.tan(slip)+offset_mm/1000*omega/velocity)
    def residual(p):
        return np.degrees(predicted(p[0],p[1])[mask]-target)
    point=least_squares(lambda p:residual([0,p[0]]),[-4.],bounds=([-20],[20]),loss="soft_l1",f_scale=1)
    fits=[least_squares(residual,start,bounds=([0,-20],[.05,20]),x_scale=[.01,5],loss="soft_l1",f_scale=1)
          for start in ([.001,-4],[.01,0],[.02,5])]
    best=min(fits,key=lambda f:np.mean(residual(f.x)**2))
    normalized=best.jac/np.maximum(np.linalg.norm(best.jac,axis=0),1e-12)
    scenarios=[]
    for offset in (-5.,-2.5,0.,2.5,5.):
        fit=minimize_scalar(lambda tau:float(np.mean((np.sqrt(1+residual([tau,offset])**2)-1))),bounds=(0,.05),method="bounded")
        tau=float(fit.x)
        if np.mean(residual([0,offset])**2)<np.mean(residual([tau,offset])**2): tau=0.
        scenarios.append(dict(marker_forward_offset_mm=offset,tau_ms=tau*1000,residual_deg=float(np.sqrt(np.mean(residual([tau,offset])**2)))))
    return dict(point_only_offset_mm=float(point.x[0]),point_only_residual_deg=float(np.sqrt(np.mean(residual([0,point.x[0]])**2))),
                combined_tau_ms=float(best.x[0]*1000),combined_offset_mm=float(best.x[1]),
                combined_residual_deg=float(np.sqrt(np.mean(residual(best.x)**2))),
                combined_condition=float(np.linalg.cond(normalized)),offset_scenarios=scenarios)


def imu_comparison(time, derivatives, heading, raw, shift_s=0.):
    # ログ横加速度は左正。出力の右正へ反転する。
    rt = raw.elapsed_s.to_numpy()
    stationary = (rt < -.04) & (abs(raw["ego.velo"].to_numpy()) < .02)
    ay_raw = -raw["ego.horizon_accel.get"].to_numpy()
    bias = float(np.median(ay_raw[stationary]))
    observed = np.interp(time+shift_s, rt, ay_raw - bias)
    psi = heading["psi_cal"]
    acceleration = (derivatives[:, 2, 0] * np.cos(psi) - derivatives[:, 2, 1] * np.sin(psi)) / 1000
    periods = raw["irq.period_us"].to_numpy()
    period_s = float(np.median(periods[np.isfinite(periods) & (periods > 500) & (periods < 2000)]) / 1e6)
    finite = np.isfinite(acceleration)
    # IMU側の20点移動平均と同じ遅れを予測加速度に与える。
    filtered = np.mean([np.interp(time-j*period_s, time[finite], acceleration[finite]) for j in range(20)], axis=0)
    velocity = np.interp(time+shift_s, rt, raw["ego.velo"])
    omega = heading["gyro_gain"] * np.interp(time+shift_s, rt, -raw["ego.rad_velo"] - heading["gyro_bias_rad_s"])
    centripetal = velocity * omega
    core = heading["core"] & finite
    # 旋回全域の比較。位相のない中央平坦部だけで相関を過大評価しない。
    window = heading["moving"] & finite & (time > heading["turn_first_s"]-.03) & (time < heading["turn_last_s"]+.03)
    rmse = float(np.sqrt(np.mean((observed[window]-filtered[window]) ** 2)))
    corr = float(np.corrcoef(observed[window], filtered[window])[0, 1])
    scale = float(filtered[window] @ observed[window] / (filtered[window] @ filtered[window]))
    # -ay/betaは符号・励起を満たす点だけを使う診断。0付近で割らない。
    beta = heading["beta"]
    usable = core & (abs(beta) > np.deg2rad(1)) & (observed * beta < 0)
    k = float(-beta[usable] @ observed[usable] / (beta[usable] @ beta[usable])) if usable.sum() >= 10 else None
    return dict(observed=observed, video_filtered=filtered, centripetal=centripetal,
                imu_video_rmse_m_s2=rmse, imu_video_correlation=corr, imu_video_scale=scale,
                imu_centripetal_ratio=float(np.median(observed[core] / centripetal[core])),
                imu_stationary_bias_m_s2=bias, slip_force_k_m_s2=k, force_fit_samples=int(usable.sum()))


def fit_marker_circle(vectors):
    """固定カメラの未知真下位置を、剛体上のマーカー間距離一定で消去する。"""
    vectors = np.asarray(vectors, float)
    if len(vectors) < 7 or not np.isfinite(vectors).all() or np.linalg.matrix_rank(vectors-vectors.mean(axis=0)) < 2:
        raise ValueError("Insufficient marker vectors")
    a = np.column_stack([2*vectors, np.ones(len(vectors))])
    algebraic = np.linalg.lstsq(a, np.sum(vectors**2, axis=1), rcond=None)[0]
    center = algebraic[:2]
    radius = np.sqrt(max(.01, algebraic[2]+center@center))
    def residual(parameters):
        return np.linalg.norm(vectors-parameters[:2], axis=1)-parameters[2]
    fit = least_squares(residual, [*center, radius], loss="soft_l1", f_scale=.25)
    normalized = fit.jac/np.linalg.norm(fit.jac, axis=0)
    return dict(center=fit.x[:2], radius_mm=float(fit.x[2]),
                rms_mm=float(np.sqrt(np.mean(residual(fit.x)**2))),
                condition=float(np.linalg.cond(normalized)))


def fit_heading_time(time,optical,heading,raw):
    rt=raw.elapsed_s.to_numpy()
    angle=cumulative_trapezoid(-raw["ego.rad_velo"].to_numpy()-heading["gyro_bias_rad_s"],rt,initial=0)
    mask=heading["moving"] & np.isfinite(optical["heading"]) & (time>heading["turn_first_s"]-.10) & (time<heading["turn_last_s"]+.09)
    target=optical["heading"]
    reference=float(np.median(angle))
    def residual(p,selected=mask):
        return p[0]*(np.interp(time[selected]+p[1],rt,angle)-reference)+p[2]-target[selected]
    first=[heading["gyro_gain"],0.,float(np.nanmedian(target[mask]))]
    fit=least_squares(residual,first,bounds=([.9,-.05,-8],[1.1,.05,8]),x_scale=[1,.02,1],loss="soft_l1",f_scale=.01)
    estimate=fit.x[0]*(np.interp(time+fit.x[1],rt,angle)-reference)+fit.x[2]
    return dict(gain=float(fit.x[0]),shift_s=float(fit.x[1]),heading=estimate,
                rmse_deg=float(np.sqrt(np.mean(np.degrees(estimate[mask]-target[mask])**2))))


def optical_heading(pose, pose_path, heading):
    """同一フレームのマーカー向きと軌跡接線を比較。ジャイロ尺度を使わない。"""
    apparent = pd.read_csv(pose_path.parent/"apparent_floor_marker_pose.csv")
    geometry = read_json(pose_path.parent/"body_geometry_used.json")
    height = geometry["camera_height_mm"]
    af, ab = 1-geometry["front_height_mm"]/height, 1-geometry["rear_height_mm"]/height
    if len(pose) != len(apparent) or np.ptp(apparent.time_video_s.to_numpy()-pose.time_from_motion_start_s.to_numpy()) > 1e-8:
        raise ValueError("Marker cache and normalized PTS rows do not correspond")
    aligned = pose[["right_mm","forward_mm"]].reset_index(drop=True).join(apparent.reset_index(drop=True))
    front = aligned[["blue_x_mm","blue_y_mm"]].to_numpy()
    rear = aligned[["yellow_x_mm","yellow_y_mm"]].to_numpy()
    centre = (af*front+ab*rear)/2
    finite = np.isfinite(centre).all(axis=1) & aligned.marker_detected_raw.fillna(False).to_numpy(bool)
    # 保存済み正規化座標への線形変換。軌跡の角度・終点を合わせ直す処理ではない。
    design = np.column_stack([centre[finite],np.ones(finite.sum())])
    conversion = np.linalg.lstsq(design,aligned.loc[finite,["right_mm","forward_mm"]],rcond=None)[0]
    transform = conversion[:2]
    if not np.allclose(transform.T@transform,np.eye(2),atol=2e-4):
        raise ValueError("Marker coordinate conversion is not an isometry")
    vectors = (af*front-ab*rear)@transform
    t = pose.time_from_motion_start_s.to_numpy()
    usable = finite & (t >= heading["turn_first_s"]-.02) & (t <= heading["turn_last_s"]+.02)
    circle = fit_marker_circle(vectors[usable])
    relative = vectors-circle["center"]
    angles = np.full(len(t),np.nan)
    angles[finite] = np.unwrap(np.arctan2(relative[finite,0],relative[finite,1]))
    angles += np.nanmedian(heading["course"][heading["entry"]]-angles[heading["entry"]])
    beta = heading["course"]-angles
    lag = -heading["turn_sign"]*np.degrees(beta[heading["core"] & finite])
    # 異なる30 ms区間を丸ごと除いた2解で、円弧が短いときの不安定さも残す。
    block = np.floor((t-heading["turn_first_s"])/.030).astype(int)
    split_lags=[]
    for parity in (0,1):
        fitted=fit_marker_circle(vectors[usable & (block%2==parity)])
        vec=vectors-fitted["center"]
        angle=np.full(len(t),np.nan)
        angle[finite]=np.unwrap(np.arctan2(vec[finite,0],vec[finite,1]))
        angle+=np.nanmedian(heading["course"][heading["entry"]]-angle[heading["entry"]])
        split_lags.append(float(np.nanmedian(-heading["turn_sign"]*np.degrees((heading["course"]-angle)[heading["core"]]))))
    return dict(heading=angles,beta=beta,vector_radius_mm=circle["radius_mm"],circle_rms_mm=circle["rms_mm"],
                circle_condition=circle["condition"],lag_median_deg=float(np.nanmedian(lag)),
                lag_p95_deg=float(np.nanpercentile(lag,95)),circle_center_right_mm=float(circle["center"][0]),
                circle_center_forward_mm=float(circle["center"][1]),
                block_split_lag_min_deg=min(split_lags),block_split_lag_max_deg=max(split_lags))


def inventory(archive):
    rows = []
    for group, paths in (("pixel_archive", archive.glob("*/motion.settings.json")),
                         ("legacy_top_level", (ROOT/"tools/logs").glob("*.settings.json"))):
        for path in sorted(paths):
            try:
                d = read_json(path)
                p = d.get("parameters", {})
                if "velo" not in p:
                    continue
                rows.append(dict(path=str(path), group=group, mouse=d.get("mouse", "unknown"),
                    motion=d.get("motion_name", "unknown"), speed_m_s=p["velo"], suction=d.get("suction", {}).get("enabled"),
                    settings_sha256=digest(path), eligibility="same_recent_A_500_domain" if group=="pixel_archive" else "hardware_and_video_pairing_unverified"))
            except (ValueError, OSError) as error:
                rows.append(dict(path=str(path), group=group, eligibility="unreadable", error=str(error)))
    return rows


def load_entry(entry):
    for path, expected in entry["input_sha256"].items():
        if digest(path) != expected:
            raise ValueError(f"Input changed since previous model: {path}")
    inputs = list(entry["input_sha256"])
    csv = Path(next(p for p in inputs if p.endswith("motion.csv")))
    trajectory = Path(next(p for p in inputs if p.endswith("normalized_trajectory.csv")))
    report = read_json(trajectory.parent/"report.json")
    pose = pd.read_csv(trajectory)
    raw = pd.read_csv(csv)
    required = ["time_ms", "sequence", "ego.velo", "ideal.velo", "ego.rad_velo", "ideal.rad_velo", "ego.horizon_accel.get"]
    if (not np.isfinite(raw[required].to_numpy()).all() or not raw.time_ms.diff().iloc[1:].between(1, 10).all()
            or not raw.sequence.diff().iloc[1:].eq(1).all()):
        raise ValueError(f"Invalid telemetry: {csv}")
    settings = entry["settings"]
    start = settings["log"]["motion_start_time_ms"]
    raw["elapsed_s"] = (raw.time_ms-start)/1000
    raw.attrs["motion_end_s"] = (settings["log"]["motion_end_time_ms"]-start)/1000
    if len(pose) < 100 or report["frame_stride"] != 1:
        raise ValueError("Detailed video data required")
    return raw, pose, trajectory, report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=ROOT/"tools/turn_analysis/20261112_A_speed_prediction_v2/model.json")
    parser.add_argument("--archive", type=Path, default=Path("D:/TYPE8_Camera/trials"))
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("出力先は未作成のディレクトリを指定してください")
    manifest = read_json(args.manifest)
    args.output.mkdir(parents=True)
    inv = inventory(args.archive)
    pd.DataFrame(inv).to_csv(args.output/"data_inventory.csv", index=False)
    metrics, variations, sources = [], [], []
    fig, axes = plt.subplots(4, 4, figsize=(17, 16), constrained_layout=True)
    ayfig, ayaxes = plt.subplots(4, 4, figsize=(17, 16), constrained_layout=True)
    optfig, optaxes = plt.subplots(4,4,figsize=(17,16),constrained_layout=True)
    for (motion, entries), ax, ayax, optax in zip(manifest["inputs"].items(), axes.flat, ayaxes.flat,optaxes.flat):
        for entry in entries:
            raw, pose, pose_path, report = load_entry(entry)
            time = pose.time_from_motion_start_s.to_numpy()
            xy = pose[["right_mm", "forward_mm"]].to_numpy()
            detected = pose.detected.astype(bool).to_numpy()
            all_derivatives = {}
            baseline = None
            for half in (.015, .025, .035):
                derivative = local_polynomial(time, xy, detected, half)
                all_derivatives[half] = derivative
                for shift in (-.005, 0., .005):
                    h = heading_from_velocity(time, derivative, raw, shift)
                    tau, fitted, residual = fit_tau(time, h, raw)
                    lag = -h["turn_sign"] * np.degrees(h["beta"][h["core"]])
                    variations.append(dict(motion=motion, trial=entry["id"], half_window_ms=half*1000,
                        gyro_time_shift_ms=shift*1000, inferred_lag_median_deg=float(np.median(lag)),
                        inferred_lag_p95_deg=float(np.percentile(lag, 95)), tau_ms=tau*1000,
                        model_residual_deg=residual, gyro_gain=h["gyro_gain"]))
                    if half == .025 and shift == 0:
                        baseline = (h, fitted, tau, residual)
            h, fitted, tau, residual = baseline
            imu = imu_comparison(time, all_derivatives[.025], h, raw)
            optical = optical_heading(pose,pose_path,h)
            synchronized=fit_heading_time(time,optical,h,raw)
            optical_imu=imu_comparison(time,all_derivatives[.025],dict(h,psi_cal=optical["heading"],gyro_gain=synchronized["gain"]),raw,synchronized["shift_s"])
            optical_h=dict(h,beta=optical["beta"],gyro_gain=synchronized["gain"])
            raw.attrs["heading_shift_s"]=synchronized["shift_s"]
            optical_tau,optical_fitted,optical_residual=fit_tau(time,optical_h,raw)
            point_comparison=compare_point_offset(time,optical_h,raw)
            optical_window_lags=[]
            for half,derivative in all_derivatives.items():
                alternate=heading_from_velocity(time,derivative,raw)
                optical_window_lags.append(float(np.nanmedian(-h["turn_sign"]*np.degrees((alternate["course"]-optical["heading"])[alternate["core"]]))))
            # 横力の線形近似を診断。時間合わせは軌跡ではなくマーカー姿勢から求めたもの。
            rt=raw.elapsed_s.to_numpy()
            ay_aligned=np.interp(time+synchronized["shift_s"],rt,-raw["ego.horizon_accel.get"]-imu["imu_stationary_bias_m_s2"])
            optical_finite=np.isfinite(optical["beta"])
            periods=raw["irq.period_us"].to_numpy()
            period_s=float(np.median(periods[np.isfinite(periods) & (periods>500) & (periods<2000)])/1e6)
            filtered_beta=np.mean([np.interp(time-j*period_s,time[optical_finite],optical["beta"][optical_finite]) for j in range(20)],axis=0)
            force_mask=h["core"] & (abs(filtered_beta)>np.deg2rad(1))
            force_k=float(-filtered_beta[force_mask]@ay_aligned[force_mask]/(filtered_beta[force_mask]@filtered_beta[force_mask]))
            core = h["core"]
            entry_variations = [v for v in variations if v["motion"] == motion and v["trial"] == entry["id"]]
            medians = [v["inferred_lag_median_deg"] for v in entry_variations]
            lag = -h["turn_sign"] * np.degrees(h["beta"][core])
            row = dict(motion=motion, trial=entry["id"], tau_ms=tau*1000, beta_model_residual_deg=residual,
                lag_median_deg=float(np.median(lag)), lag_p95_deg=float(np.percentile(lag,95)),
                raw_gyro_lag_median_deg=float(np.median(-h["turn_sign"]*np.degrees(h["beta_raw"][core]))),
                sensitivity_lag_min_deg=float(min(medians)), sensitivity_lag_max_deg=float(max(medians)),
                gyro_gain=h["gyro_gain"], gyro_bias_rad_s=h["gyro_bias_rad_s"],
                raw_gyro_turn_deg=h["raw_angle_deg"], video_course_turn_deg=h["path_angle_deg"],
                previous_model_tau_ms=manifest["model"][motion]["spatial"][2]*1000,
                optical_tau_ms=optical_tau*1000,optical_beta_model_residual_deg=optical_residual,
                optical_heading_sync_shift_ms=synchronized["shift_s"]*1000,
                optical_heading_sync_gain=synchronized["gain"],optical_gyro_heading_rmse_deg=synchronized["rmse_deg"],
                optical_force_k_m_s2=force_k,
                optical_imu_video_rmse_m_s2=optical_imu["imu_video_rmse_m_s2"],
                optical_imu_video_correlation=optical_imu["imu_video_correlation"],
                optical_imu_video_scale=optical_imu["imu_video_scale"],
                optical_window_lag_min_deg=min(optical_window_lags),optical_window_lag_max_deg=max(optical_window_lags),
                **{k:v for k,v in point_comparison.items() if np.isscalar(v)},
                **{"optical_"+k:v for k,v in optical.items() if np.isscalar(v)},
                **{k:v for k,v in imu.items() if np.isscalar(v) or v is None})
            metrics.append(row)
            output = args.output/motion/entry["id"]
            output.mkdir(parents=True)
            write_json(output/"offset_comparison.json",point_comparison)
            pd.DataFrame(dict(time_s=time, detected=detected, course_rad=h["course"],
                gyro_heading_raw_rad=h["psi_raw"], gyro_heading_straight_calibrated_rad=h["psi_cal"],
                beta_raw_rad=h["beta_raw"], beta_straight_calibrated_rad=h["beta"], beta_fitted_rad=fitted,
                optical_heading_rad=optical["heading"],optical_beta_rad=optical["beta"],
                optical_fitted_beta_rad=optical_fitted,gyro_heading_optical_synchronized_rad=synchronized["heading"],
                imu_ay_optical_synchronized_m_s2=ay_aligned,
                video_ay_optical_synchronized_m_s2=optical_imu["video_filtered"],
                velocity_video_m_s=h["speed"], core=core, imu_ay_right_m_s2=imu["observed"],
                video_ay_filtered_m_s2=imu["video_filtered"], centripetal_m_s2=imu["centripetal"])).to_csv(output/"observations.csv",index=False)
            view = h["moving"] & (time > h["turn_first_s"]-.06) & (time < h["turn_last_s"]+.08)
            phase = (time-h["turn_first_s"])/(h["turn_last_s"]-h["turn_first_s"])
            ax.plot(phase[view], -h["turn_sign"]*np.degrees(h["beta"][view]), lw=1, alpha=.75)
            optax.plot(phase[view],-h["turn_sign"]*np.degrees(optical["beta"][view]),lw=1,alpha=.75)
            ayax.plot(phase[view], h["turn_sign"]*optical_imu["observed"][view], color="#c56b28", lw=1, alpha=.5)
            ayax.plot(phase[view], h["turn_sign"]*optical_imu["video_filtered"][view], color="#346aa1", lw=1, alpha=.5)
            paths = [pose_path, pose_path.parent/"report.json", pose_path.parent/"summary.json",
                     pose_path.parent/"apparent_floor_marker_pose.csv",pose_path.parent/"body_geometry_used.json",Path(__file__)]
            sources.append(dict(motion=motion, trial=entry["trial_json"], video=entry["video"],
                inherited_input_sha256=entry["input_sha256"], additional_sha256={str(p):digest(p) for p in paths},
                timing_basis=report["timing"], inferred_force_coefficient_is_provisional=True))
        rows = [r for r in metrics if r["motion"] == motion]
        print(motion, "lag median", round(float(np.median([r["lag_median_deg"] for r in rows])),3),
              "deg; gyro scale",round(float(np.mean([r["gyro_gain"] for r in rows])),5),flush=True)
        ax.set_title(motion)
        ax.axhline(0,color="black",lw=.6)
        ax.set(xlabel="turn phase",ylabel="inferred lag [deg]")
        ayax.set_title(motion)
        ayax.set(xlabel="turn phase",ylabel="lateral acceleration [m/s2]")
        optax.set_title(motion)
        optax.set(xlabel="turn phase",ylabel="optical inferred lag [deg]")
        optax.axhline(0,color="black",lw=.6)
        for a in (ax,ayax,optax): a.grid(alpha=.25)
    for a in list(axes.flat)[14:]+list(ayaxes.flat)[14:]+list(optaxes.flat)[14:]: a.set_visible(False)
    fig.suptitle("Mouse A / 500: course minus straight-calibrated gyro heading\nThree trials per motion; inference depends on synchronization and straight zero-slip assumption")
    ayfig.suptitle("Lateral acceleration: IMU (orange) vs video kinematics (blue)\nVideo signal includes 20-tap IMU averaging delay; sign normalized to turn direction")
    optfig.suptitle("Video-only course minus marker heading\nUnknown constant parallax offset inferred from rigid marker-vector circle; short arcs are poorly conditioned")
    fig.savefig(args.output/"inferred_slip.png",dpi=150)
    ayfig.savefig(args.output/"lateral_acceleration.png",dpi=150)
    optfig.savefig(args.output/"optical_slip.png",dpi=150)
    plt.close(fig); plt.close(ayfig); plt.close(optfig)
    frame=pd.DataFrame(metrics)
    frame.to_csv(args.output/"trial_metrics.csv",index=False)
    pd.DataFrame(variations).to_csv(args.output/"sensitivity.csv",index=False)
    summary=frame.groupby("motion").agg(count=("trial","count"), lag_median_deg=("lag_median_deg","median"),
        lag_min_deg=("lag_median_deg","min"), lag_max_deg=("lag_median_deg","max"),
        sensitivity_min_deg=("sensitivity_lag_min_deg","min"), sensitivity_max_deg=("sensitivity_lag_max_deg","max"),
        tau_median_ms=("tau_ms","median"), previous_model_tau_ms=("previous_model_tau_ms","first"),
        imu_video_correlation_mean=("imu_video_correlation","mean"), imu_video_rmse_mean_m_s2=("imu_video_rmse_m_s2","mean"),
        optical_lag_median_deg=("optical_lag_median_deg","median"),optical_lag_min_deg=("optical_lag_median_deg","min"),
        optical_lag_max_deg=("optical_lag_median_deg","max"),optical_circle_condition_max=("optical_circle_condition","max"),
        optical_circle_rms_mean_mm=("optical_circle_rms_mm","mean"),
        optical_tau_median_ms=("optical_tau_ms","median"),optical_force_k_median_m_s2=("optical_force_k_m_s2","median"),
        optical_heading_sync_shift_mean_ms=("optical_heading_sync_shift_ms","mean"),optical_heading_rmse_mean_deg=("optical_gyro_heading_rmse_deg","mean"),
        optical_imu_video_correlation_mean=("optical_imu_video_correlation","mean"),optical_imu_video_rmse_mean_m_s2=("optical_imu_video_rmse_m_s2","mean"),
        point_only_offset_median_mm=("point_only_offset_mm","median"),point_only_residual_mean_deg=("point_only_residual_deg","mean"),
        optical_beta_model_residual_mean_deg=("optical_beta_model_residual_deg","mean"),combined_condition_max=("combined_condition","max"),
        gyro_gain_mean=("gyro_gain","mean"))
    summary.to_csv(args.output/"motion_summary.csv")
    write_json(args.output/"report.json",dict(status="PROVISIONAL_OBSERVATION_NOT_PARAMETER_ADOPTION",mouse="A",speed_mm_s=500,
        sources=sources,manifest_sha256=digest(args.manifest),source_sha256=digest(__file__),
        inventory_counts={str(k):v for k,v in Counter((r.get("group"),r.get("mouse"),r.get("speed_m_s"),r.get("suction")) for r in inv).items()},
        assumptions=["Entry and exit straight sideslip are zero; gyro scale is calibrated using those video course directions.",
            "Video trajectory-fitted synchronization is reused; +/-5 ms sensitivity is a scenario, not a proven timing bound.",
            "Derivative windows +/-15,25,35 ms; local cubic polynomial, no smoothing over gaps >20 ms.",
            "Relative marker midpoint approximates axle center. Marker placement, camera height and missing absolute heading remain systematic errors.",
            "Lateral IMU sign inverted to right-positive; 20-sample averaging delay modeled. IMU lever arm, tilt and exact captured ELF are unverified.",
            "Video-only heading uses a fitted constant marker-vector parallax offset and free marker distance; camera nadir is inferred, not independently measured.",
            "Marker midpoint offset and axle sideslip are compared as competing explanations. Condition numbers and fit residuals are diagnostics, not confidence intervals.",
            "All fitted tau values are effective parameters, not independent physical constants. No new-speed validation or Params modification."]))
    (args.output/"analysis_source.py").write_bytes(Path(__file__).read_bytes())


if __name__ == "__main__":
    main()
