"""速度・吸引条件別に動画姿勢とIMUから滑りを観測する（オフライン）。

IMUは機体中心、前後マーカー中点も中心付近とするユーザー情報に基づく条件付き評価。
画像の機首方向と移動方向を分離し、既存のモデル滑り角を教師にしない。
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
import pandas as pd
from scipy.integrate import cumulative_trapezoid
from scipy.optimize import least_squares

from turn_slip_observation import fit_marker_circle, local_polynomial
from turn_pose_ekf import Config as NoiseConfig, rotation, J


@dataclass
class Observation:
    frame: pd.DataFrame
    diagnostics: dict


def slip_motion(state, velocity, gyro, dt, k, gain):
    """状態[x,y,psi,beta,bg,ba]。小角度の線形タイヤモデルを厳密離散化。"""
    if not np.isfinite([*state, velocity, gyro, dt, k, gain]).all() or not 0 < dt < .030 or k <= 0 or gain <= 0:
        raise ValueError("Invalid slip prediction")
    x = np.asarray(state, float)
    u = max(.1, abs(velocity))
    decay = math.exp(-k/u*dt)
    omega = gain*(gyro-x[4])
    coupling = velocity/k*(1-decay)
    beta = decay*x[3]-coupling*omega
    mid_beta = .5*(x[3]+beta)
    mid_yaw = x[2]+.5*omega*dt
    r = rotation(mid_yaw)
    body = np.array([velocity, velocity*math.tan(mid_beta)])
    db = np.array([0., velocity/math.cos(mid_beta)**2])
    predicted = x.copy()
    predicted[:2] += r @ body*dt
    predicted[2] += omega*dt
    predicted[3] = beta
    f = np.eye(6)
    f[:2, 2] = r @ J @ body*dt
    f[:2, 3] = r @ db*.5*(1+decay)*dt
    f[:2, 4] = (-r @ J @ body*.5*gain*dt+r @ db*.5*coupling*gain)*dt
    f[2, 4] = -gain*dt
    f[3, 3], f[3, 4] = decay, coupling*gain
    return predicted, f


class ConditionedTurnEkf:
    """吸引ON/OFFで滑り係数・観測雑音を切替える解析用6状態EKF。

    横加速度は予測入力にはせず観測として1回だけ使う。
    壁の光線観測は8状態版と別で、ここへ未校正IRを入力しない。
    """
    def __init__(self, suction_enabled, profiles, gyro_bias=0., accel_bias=0.):
        self.noise = NoiseConfig.for_suction(suction_enabled)
        self.profile_name = "suction_on" if suction_enabled else "suction_off"
        self.parameters = profiles[self.profile_name]
        self.k = float(self.parameters["k_m_s2"])
        self.gain = float(self.parameters["gyro_gain"])
        self.ay_std = float(self.parameters["ay_std_m_s2"])
        if min(self.k, self.gain, self.ay_std) <= 0:
            raise ValueError("Invalid condition calibration")
        self.state = np.array([0., 0., 0., 0., gyro_bias, accel_bias])
        self.covariance = np.diag([.001**2, .001**2, math.radians(2)**2, math.radians(3)**2,
                                   self.noise.initial_gyro_bias_std**2, self.noise.initial_accel_bias_std**2])

    def predict(self, velocity, gyro, dt):
        self.state, f = slip_motion(self.state, velocity, gyro, dt, self.k, self.gain)
        g = -f[:, 4].copy()
        g[4] = 0.
        q = np.diag([(.03*abs(velocity)*dt)**2, (.03*abs(velocity)*dt)**2,
                     math.radians(.3)**2*dt, math.radians(8)**2*dt,
                     self.noise.gyro_bias_walk**2*dt, self.noise.accel_bias_walk**2*dt])
        q += np.outer(g,g)*self.noise.gyro_std**2
        self.covariance = f @ self.covariance @ f.T+q

    def observe_acceleration(self, acceleration):
        h = np.array([0.,0.,0.,-self.k,0.,1.])
        residual = acceleration-float(h @ self.state)
        variance = self.ay_std**2+(.25*self.k*self.state[3])**2
        innovation = float(h @ self.covariance @ h+variance)
        if not math.isfinite(acceleration) or residual**2/innovation > 25:
            return False
        gain = self.covariance @ h/innovation
        self.state += gain*residual
        a = np.eye(6)-np.outer(gain,h)
        self.covariance = a @ self.covariance @ a.T+np.outer(gain,gain)*variance
        self.covariance = .5*(self.covariance+self.covariance.T)
        return True

    def estimate(self):
        return dict(x_mm=self.state[0]*1000, y_mm=self.state[1]*1000, yaw_deg=math.degrees(self.state[2]),
                    beta_deg=math.degrees(self.state[3]), sigma_beta_deg=math.degrees(math.sqrt(self.covariance[3,3])))


def replay_conditioned(raw, suction_enabled, profiles, start_ms=None, end_ms=None):
    """20点平均エンコーダと横加速度の重心時刻をそろえて事後推定する。"""
    columns = ["time_ms", "sequence", "ideal.velo", "ideal.rad_velo", "ego.velo", "ego.accel", "ego.rad_velo", "ego.horizon_accel.get"]
    if not np.isfinite(raw[columns]).all().all() or np.any(np.diff(raw.time_ms) <= 0) or np.any(np.diff(raw.sequence) != 1):
        raise ValueError("Invalid runtime telemetry")
    times = raw.time_ms.to_numpy(float)/1000
    if np.max(np.diff(times)) >= .030:
        raise ValueError("Runtime timestamp gap")
    periods = raw["irq.period_us"].to_numpy(float)*1e-6
    known = np.isfinite(periods) & (periods > .0005) & (periods < .002)
    if known.sum() < 30:
        raise ValueError("Runtime control period unavailable")
    tick = float(np.median(periods[known]))
    wheel = (raw["ego.velo"].to_numpy()-.010*raw["ego.accel"].to_numpy())/(tick*1000)
    active = abs(raw["ideal.velo"].to_numpy()) > .01
    first, last = np.flatnonzero(active)[[0,-1]]
    if start_ms is not None:
        first = max(first, int(np.searchsorted(times,start_ms/1000)))
    if end_ms is not None:
        last = min(last, int(np.searchsorted(times,end_ms/1000)))
    quiet = (np.arange(len(times)) < first) & (times < times[first]-.04) & (abs(wheel) < .02)
    quiet &= (abs(raw["ego.horizon_accel.get"].to_numpy()) < 2.)
    if quiet.sum() < 10:
        raise ValueError("Runtime stationary bias unavailable")
    bg = float(np.median(raw["ego.rad_velo"].to_numpy()[quiet]))
    ba = float(np.median(raw["ego.horizon_accel.get"].to_numpy()[quiet]))
    f = ConditionedTurnEkf(suction_enabled, profiles, bg, ba)
    velocity = np.interp(times+9.5*tick, times, wheel)
    ay = np.interp(times+9.5*tick, times, raw["ego.horizon_accel.get"])
    gyro = raw["ego.rad_velo"].to_numpy(float)
    rows = []
    for i in range(first,last+1):
        if times[i]+9.5*tick > times[-1]:
            raise ValueError("Runtime post-roll unavailable")
        if i > first:
            f.predict(float(.5*(velocity[i]+velocity[i-1])),float(.5*(gyro[i]+gyro[i-1])),times[i]-times[i-1])
        accepted = f.observe_acceleration(float(ay[i])) if abs(velocity[i]) > .15 else False
        rows.append(dict(time_ms=times[i]*1000,time_s=times[i]-times[first],
                         ideal_omega_rad_s=raw["ideal.rad_velo"].iloc[i],velocity_m_s=velocity[i],
                         accel_accepted=accepted, **f.estimate()))
    return pd.DataFrame(rows), dict(profile=f.profile_name, suction_enabled=suction_enabled,
                                    parameters=f.parameters, gyro_bias_rad_s=bg,
                                    accel_bias_m_s2=ba, averaging_delay_ms=tick*9500,
                                    minimum_covariance_eigenvalue=float(np.linalg.eigvalsh(f.covariance).min()))


def force_fit(beta_rad, acceleration, mask):
    """ay=-K beta。零点は静止区間で決め、旋回波形から任意のbiasを追加しない。"""
    b, a = np.asarray(beta_rad), np.asarray(acceleration)
    selected = np.asarray(mask, bool) & np.isfinite(b) & np.isfinite(a)
    if selected.sum() < 8 or np.sqrt(np.mean(b[selected]**2)) < math.radians(.5):
        raise ValueError("Insufficient slip/force excitation")
    def residual(p):
        return -p[0]*b[selected]-a[selected]
    result = least_squares(residual, [100.], bounds=([.01], [3000.]),
                           x_scale=[100.], loss="soft_l1", f_scale=1.)
    error = residual(result.x)
    return dict(k_m_s2=float(result.x[0]), residual_bias_m_s2=0.,
                rmse_m_s2=float(np.sqrt(np.mean(error**2))), samples=int(selected.sum()),
                beta_rms_deg=float(np.degrees(np.sqrt(np.mean(b[selected]**2)))),
                at_bound=bool(result.active_mask.any()))


def observe(raw, settings, pose, geometry, report, half_window_s=.020):
    """登録済み全フレームのマーカーと、MCU時刻付き実測ログを照合する。"""
    required = ["time_ms", "sequence", "ego.velo", "ego.accel", "ego.rad_velo", "ideal.velo",
                "ideal.rad_velo", "ego.horizon_accel.get"]
    if settings.get("mouse") != "A" or not np.isfinite(raw[required]).all().all():
        raise ValueError("Mouse A and finite motion telemetry required")
    if np.any(np.diff(raw.time_ms) <= 0) or np.max(np.diff(raw.time_ms)) > 10 or np.any(np.diff(raw.sequence) != 1):
        raise ValueError("Discontinuous motion telemetry")
    calibration = report["quality"]["marker_calibration"]
    if report["frame_stride"] != 1 or calibration.get("per_frame_registration_required") is not True:
        raise ValueError("Per-frame registered full-video observations required")
    start = settings["log"]["motion_start_time_ms"]
    rt = (raw.time_ms.to_numpy(float)-start)/1000
    period = raw["irq.period_us"].to_numpy(float)/1e6
    known = np.isfinite(period) & (period > .0005) & (period < .002)
    if known.sum() < 30:
        raise ValueError("Measured control period is unavailable")
    tick = float(np.median(period[known]))
    # This cancels the acceleration term in the source expression, recovering
    # the firmware's 20-tick wheel mean, NOT twenty missing snapshot pulses.
    wheel = (raw["ego.velo"].to_numpy()-.010*raw["ego.accel"].to_numpy())/(tick*1000)
    # Suction causes sizeable zero-mean gyro vibration even with stopped wheels.
    # Do not retain only low-rate samples, which biases that calibration.
    quiet = (rt < -.04) & (abs(wheel) < .02) & (abs(raw["ideal.velo"].to_numpy()) < .01)
    quiet &= (abs(raw["ego.accel"].to_numpy()) < 2.) & (abs(raw["ego.horizon_accel.get"].to_numpy()) < 2.)
    if quiet.sum() < 10:
        raise ValueError("Insufficient pre-motion stationary bias data")
    bias = [float(np.median(raw[c].to_numpy()[quiet])) for c in
            ("ego.rad_velo", "ego.accel", "ego.horizon_accel.get")]
    w = raw["ego.rad_velo"].to_numpy()-bias[0]
    angle = cumulative_trapezoid(w, rt, initial=0.)
    command = raw["ideal.rad_velo"].to_numpy(float)
    turn = abs(command) > .02*max(abs(command))
    if turn.sum() < 6:
        raise ValueError("Insufficient turn samples")
    begin, end = rt[turn][[0, -1]]
    vt = pose.time_video_s.to_numpy(float)
    video_start = report["required_motion_window_quality"]["video_motion_start_s"]
    t = vt-video_start
    valid = pose.marker_detected_raw.fillna(False).to_numpy(bool) & pose.calibration_valid.fillna(False).to_numpy(bool)
    front = pose[["blue_x_mm", "blue_y_mm"]].to_numpy(float)
    rear = pose[["yellow_x_mm", "yellow_y_mm"]].to_numpy(float)
    af = 1-geometry["front_height_mm"]/geometry["camera_height_mm"]
    ar = 1-geometry["rear_height_mm"]/geometry["camera_height_mm"]
    center = (af*front+ar*rear)/2
    vectors = af*front-ar*rear
    valid &= np.isfinite(center).all(axis=1) & np.isfinite(vectors).all(axis=1)
    usable = valid & (t > begin-.02) & (t < end+.02)
    circle = fit_marker_circle(vectors[usable])
    if circle["condition"] > 50 or circle["rms_mm"] > 1.:
        raise ValueError("Marker yaw geometry is weak or inconsistent")
    body_vector = vectors-circle["center"]
    theta = np.full(len(t), np.nan)
    theta[valid] = np.unwrap(np.arctan2(body_vector[valid, 1], body_vector[valid, 0]))
    derivative = local_polynomial(t, center, valid, half_window_s)
    velocity, acceleration = derivative[:, 1]/1000, derivative[:, 2]/1000
    speed = np.linalg.norm(velocity, axis=1)
    course = np.full(len(t), np.nan)
    moving = valid & np.isfinite(speed) & (speed > settings["parameters"]["velo"]*.4)
    course[moving] = np.unwrap(np.arctan2(velocity[moving, 1], velocity[moving, 0]))
    entry = moving & (t > max(.02, begin-.15)) & (t < begin-.025)
    if entry.sum() < 5:
        raise ValueError("Insufficient measured straight entry")
    theta += float(np.nanmedian(course[entry]-theta[entry]))
    yaw_change = float(np.interp(end, t[valid], theta[valid])-np.interp(begin, t[valid], theta[valid]))
    sign = float(np.sign(yaw_change/(np.interp(end, rt, angle)-np.interp(begin, rt, angle))))
    selected = valid & (t > begin-.05) & (t < end+.06)
    def residual(p):
        return sign*p[0]*np.interp(t[selected]+p[1], rt, angle)+p[2]-theta[selected]
    initial_offset = float(np.nanmedian(theta[selected]-sign*np.interp(t[selected], rt, angle)))
    fit = least_squares(residual, [1., 0., initial_offset], bounds=([.9, -.08, -50.], [1.1, .08, 50.]),
                        x_scale=[1., .01, 1.], loss="soft_l1", f_scale=.005)
    if fit.active_mask.any():
        raise ValueError("Yaw/time fit reached its calibration bounds")
    gain, shift = fit.x[:2]
    query = t+shift
    gyro = sign*gain*np.interp(query, rt, w)
    u = np.interp(query+9.5*tick, rt, wheel)
    ay = sign*(np.interp(query, rt, raw["ego.horizon_accel.get"])-bias[2])
    ax = np.interp(query, rt, raw["ego.accel"])-bias[1]
    beta = (course-theta+math.pi)%(2*math.pi)-math.pi
    # Compare averaged beta to the actual averaged accelerometer, maintaining
    # the known FIR phase instead of granting the IMU a future slip estimate.
    finite = np.isfinite(beta)
    averaged_beta = np.mean([np.interp(t-j*tick, t[finite], beta[finite]) for j in range(20)], axis=0)
    cmd = np.interp(query, rt, command)
    core = moving & (abs(cmd) > .5*max(abs(command)))
    fit_mask = moving & (abs(cmd) > .20*max(abs(command))) & (t > begin-.01) & (t < end+.025)
    c, s = np.cos(theta), np.sin(theta)
    vf, vl = c*velocity[:, 0]+s*velocity[:, 1], -s*velocity[:, 0]+c*velocity[:, 1]
    af_video = c*acceleration[:, 0]+s*acceleration[:, 1]
    al_video = -s*acceleration[:, 0]+c*acceleration[:, 1]
    frame = pd.DataFrame(dict(time_s=t, log_time_s=query, detected=valid, core=core, fit=fit_mask,
                              forward_marker_m_s=vf, lateral_marker_m_s=vl,
                              beta_rad=beta, lag_deg=-np.sign(yaw_change)*np.degrees(beta),
                              averaged_beta_rad=averaged_beta, body_yaw_rad=theta, course_rad=course,
                              gyro_rad_s=gyro, wheel_forward_m_s=u, imu_forward_m_s2=ax, imu_left_m_s2=ay,
                              video_forward_m_s2=af_video, video_left_m_s2=al_video,
                              center_x_mm=center[:, 0], center_y_mm=center[:, 1]))
    force = force_fit(averaged_beta, ay, fit_mask)
    diagnostics = dict(gyro_gain=float(gain), time_shift_ms=float(shift*1000),
                       yaw_fit_rmse_deg=float(np.sqrt(np.mean(np.degrees(residual(fit.x))**2))),
                       circle_condition=circle["condition"], circle_rms_mm=circle["rms_mm"],
                       marker_span_mm=circle["radius_mm"], stationary_samples=int(quiet.sum()),
                       stationary_gyro_mad_rad_s=float(1.4826*np.median(abs(w[quiet]))),
                       irq_missing_samples=int((~known).sum()), measured_period_us=tick*1e6,
                       core_samples=int(core.sum()), lag_core_median_deg=float(np.nanmedian(frame.loc[core, "lag_deg"])),
                       beta_core_median_deg=float(np.nanmedian(np.degrees(beta[core]))),
                       derivative_half_window_ms=half_window_s*1000, force=force,
                       imu_at_body_center_assumed=True, marker_midpoint_at_center_assumed=True,
                       frame_handedness_vs_left_positive_gyro=sign,
                       turn_first_s=float(begin), turn_last_s=float(end))
    return Observation(frame, diagnostics)
