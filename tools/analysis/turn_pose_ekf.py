"""ターン中の位置・ヨー・横滑りを推定する保存ログ用EKF。

SI単位、入口前方をx、入口左をy、左旋回を正とする。
状態: x,y,yaw,vx,vy,bg,bax,bay。横滑りゼロや既存の滑りモデルは観測にしない。
壁は既知の有限線分と校正済み光線を使う場合だけ更新する。
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import pandas as pd


RIGHT = "Encoder_GetProperty_Right().sp_pulse"
LEFT = "Encoder_GetProperty_Left().sp_pulse"
PROFILE_FILE = Path(__file__).with_name("turn_pose_profiles_A.json")


@dataclass(frozen=True)
class Config:
    mm_per_tick: float = 15.02 * math.pi / 172
    gyro_std: float = .04
    accel_std: float = 1.
    wheel_slip_std: float = .04
    velocity_walk: float = .08
    gyro_bias_walk: float = .002
    accel_bias_walk: float = .05
    initial_heading_std: float = math.radians(2)
    initial_accel_bias_std: float = .3
    initial_gyro_bias_std: float = .02
    wall_interval_s: float = .020
    wall_residual_limit_m: float = .030
    nis_limit: float = 9.

    def __post_init__(self):
        if any(not math.isfinite(v) or v <= 0 for v in asdict(self).values()):
            raise ValueError("Configuration values must be finite and positive")

    @classmethod
    def for_suction(cls, enabled):
        """吸引振動を区別する。速度から吸引状態を推測しない。機体A用。"""
        if type(enabled) is not bool:
            raise ValueError("Explicit suction.enabled boolean is required")
        profiles = json.loads(PROFILE_FILE.read_text(encoding="utf-8"))
        if profiles.get("mouse") != "A":
            raise ValueError("Unexpected mouse calibration")
        profile = profiles["suction_on" if enabled else "suction_off"]
        return cls(**profile["noise"])


def rotation(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s], [s, c]])


J = np.array([[0., -1.], [1., 0.]])


def motion(state, gyro, acceleration, dt):
    """中央姿勢で加速度を回転し、位置を2次積分。F/Gは同じ離散式の微分。"""
    x = np.asarray(state, dtype=float)
    a = np.asarray(acceleration, dtype=float) - x[6:8]
    if x.shape != (8,) or a.shape != (2,) or not np.isfinite([*x, gyro, *a, dt]).all() or not 0 < dt < .030:
        raise ValueError("Invalid state, IMU sample or timestamp gap")
    r = rotation(x[2] + .5 * (gyro - x[5]) * dt)
    world_a = r @ a
    yaw_a = J @ world_a
    out = x.copy()
    out[:2] += x[3:5] * dt + .5 * world_a * dt**2
    out[3:5] += world_a * dt
    out[2] += (gyro - x[5]) * dt
    f = np.eye(8)
    f[:2, 3:5] = np.eye(2) * dt
    f[:2, 2], f[3:5, 2] = .5 * yaw_a * dt**2, yaw_a * dt
    f[:2, 5], f[3:5, 5] = -.25 * yaw_a * dt**3, -.5 * yaw_a * dt**2
    f[2, 5] = -dt
    f[:2, 6:8], f[3:5, 6:8] = -.5 * r * dt**2, -r * dt
    g = np.zeros((8, 3))
    g[:2, :2], g[3:5, :2] = .5 * r * dt**2, r * dt
    g[:2, 2], g[3:5, 2], g[2, 2] = .25 * yaw_a * dt**3, .5 * yaw_a * dt**2, dt
    return out, f, g


def forward_measurement(state):
    c, s = math.cos(state[2]), math.sin(state[2])
    h = np.zeros(8)
    h[2] = -s * state[3] + c * state[4]
    h[3:5] = c, s
    return c * state[3] + s * state[4], h


def ray_prediction(state, sensor, walls):
    """最初の壁への光線長とH。端点・斜入射・対応が曖昧な候補は使わない。

    sensor: offset_m[前,左], angle_rad, min_m, max_m, sigma_m。
    walls: [{a_m:[x,y], b_m:[x,y]}]。全座標は同じ入口固定座標。
    """
    r = rotation(state[2])
    offset = np.asarray(sensor["offset_m"], float)
    direction = np.array([math.cos(sensor["angle_rad"]), math.sin(sensor["angle_rad"])])
    origin, beam = state[:2] + r @ offset, r @ direction
    hits = []
    for wall in walls:
        a, b = np.asarray(wall["a_m"], float), np.asarray(wall["b_m"], float)
        length = float(np.linalg.norm(b-a))
        if length <= .004:
            raise ValueError("Wall segment is too short")
        tangent = (b-a)/length
        normal = J @ tangent
        den = float(normal @ beam)
        if abs(den) < 1e-9:
            continue
        distance = float(normal @ (a-origin)/den)
        along = float((origin+distance*beam-a) @ tangent)
        if distance > 0 and 0 <= along <= length:
            # Keep untrusted hits too: never see through a near or grazing wall.
            trusted = (sensor["min_m"] < distance < sensor["max_m"] and abs(den) >= .25
                       and .002 < along < length-.002)
            h = np.zeros(8)
            h[:2] = -normal/den
            h[2] = float((-normal @ r @ J @ offset-distance*(normal @ r @ J @ direction))/den)
            hits.append((distance, h, trusted))
    hits.sort(key=lambda hit: hit[0])
    if not hits or not hits[0][2] or (len(hits) > 1 and hits[1][0]-hits[0][0] < .005):
        return None
    return hits[0][:2]


class TurnPoseEkf:
    def __init__(self, config=None, bias=(0., 0., 0.)):
        self.config = config or Config()
        self.state = np.zeros(8)
        self.state[5:8] = bias
        self.covariance = np.diag([.001**2, .001**2, self.config.initial_heading_std**2,
                                  .03**2, .03**2, self.config.initial_gyro_bias_std**2,
                                  self.config.initial_accel_bias_std**2, self.config.initial_accel_bias_std**2])
        self.wall_times = {}
        self.wall_updates = 0

    def predict(self, gyro, acceleration, dt):
        self.state, f, g = motion(self.state, gyro, acceleration, dt)
        cfg = self.config
        q = g @ np.diag([cfg.accel_std**2, cfg.accel_std**2, cfg.gyro_std**2]) @ g.T
        q[3:5, 3:5] += np.eye(2) * cfg.velocity_walk**2 * dt
        q[5, 5] += cfg.gyro_bias_walk**2 * dt
        q[6:8, 6:8] += np.eye(2) * cfg.accel_bias_walk**2 * dt
        self.covariance = f @ self.covariance @ f.T + q
        self.covariance = .5 * (self.covariance+self.covariance.T)

    def observe(self, measured, predicted, h, variance, gate=False):
        if not np.isfinite([measured, predicted, variance, *h]).all() or variance <= 0:
            raise ValueError("Invalid observation")
        residual = measured-predicted
        innovation = float(h @ self.covariance @ h+variance)
        nis = residual**2/innovation
        if gate and (abs(residual) > self.config.wall_residual_limit_m or nis > self.config.nis_limit):
            return False, nis
        gain = self.covariance @ h/innovation
        self.state += gain*residual
        a = np.eye(8)-np.outer(gain, h)
        self.covariance = a @ self.covariance @ a.T+np.outer(gain, gain)*variance
        self.covariance = .5*(self.covariance+self.covariance.T)
        return True, nis

    def observe_forward(self, velocity, variance):
        predicted, h = forward_measurement(self.state)
        return self.observe(velocity, predicted, h, variance)

    def observe_wall(self, value_m, sensor, walls, time_s, valid=True):
        key = sensor["name"]
        if (not valid or not math.isfinite(value_m) or not sensor["min_m"] < value_m < sensor["max_m"]
                or time_s-self.wall_times.get(key, -math.inf) < self.config.wall_interval_s):
            return False
        prediction = ray_prediction(self.state, sensor, walls)
        if prediction is None:
            return False
        accepted, _ = self.observe(value_m, *prediction, sensor["sigma_m"]**2, gate=True)
        if accepted:
            self.wall_updates += 1
            self.wall_times[key] = time_s
        return accepted

    def estimate(self):
        x = self.state
        body = rotation(-x[2]) @ x[3:5]
        speed2 = float(body @ body)
        beta = math.atan2(body[1], body[0]) if body[0] > .1 else math.nan
        h = np.zeros(8)
        if math.isfinite(beta):
            h[2], h[3], h[4] = -1., -x[4]/speed2, x[3]/speed2
        return dict(x_mm=x[0]*1000, y_mm=x[1]*1000, yaw_deg=math.degrees(x[2]),
                    forward_m_s=body[0], lateral_m_s=body[1], beta_deg=math.degrees(beta),
                    sigma_x_mm=1000*math.sqrt(max(0., self.covariance[0, 0])),
                    sigma_y_mm=1000*math.sqrt(max(0., self.covariance[1, 1])),
                    sigma_yaw_deg=math.degrees(math.sqrt(max(0., self.covariance[2, 2]))),
                    sigma_beta_deg=math.degrees(math.sqrt(max(0., h @ self.covariance @ h))) if math.isfinite(beta) else math.nan,
                    gyro_bias_rad_s=x[5], accel_bias_forward_m_s2=x[6], accel_bias_left_m_s2=x[7],
                    wall_updates=self.wall_updates)


def replay(log, settings, accel_delay_s=None, imu_forward_m=0., imu_left_m=0., geometry=None, config=None):
    """MCU時刻付き周期ログの事後再生。加速度遅れ補正には最大約10ms先を使う。

    スナップショット車輪パルスの速度換算だけにirq.period_usを使用。
    積分はMCU時刻差。間引き区間のパルスや平均前の加速度は復元しない。
    """
    if settings.get("mouse") != "A":
        raise ValueError("Only mouse A sensor signs have been reviewed")
    if geometry is not None and (geometry.get("mouse") != "A" or geometry.get("calibrated") is not True
                                 or geometry.get("range_model") != "calibrated_beam_length"):
        raise ValueError("Calibrated mouse A beam-length observation model required")
    suction = settings.get("suction", {}).get("enabled")
    selected = Config.for_suction(suction)
    cfg = config or selected
    required = ["time_ms", "sequence", "ideal.velo", "ideal.rad_velo", RIGHT, LEFT,
                "ego.rad_velo", "ego.accel", "ego.horizon_accel.get", "irq.period_us"]
    data = log.copy()
    if not set(required).issubset(data):
        raise ValueError("MCU-timestamped telemetry with wheel/IMU/IRQ columns is required")
    # The first row in older logs can lack IRQ diagnostics; do not invent it.
    if pd.isna(data["irq.period_us"].iloc[0]):
        data = data.iloc[1:].copy()
    if not np.isfinite(data[required]).all().all():
        bad = ~np.isfinite(data[required])
        columns = [name for name in required if bad[name].any()]
        indices = data.index[bad.any(axis=1)].tolist()
        raise ValueError(f"Missing or nonfinite required sample: {columns}, rows={indices[:8]}")
    times = data.time_ms.to_numpy(float)/1000
    if np.any(np.diff(times) <= 0) or np.any(np.diff(times) >= .030) or np.any(np.diff(data.sequence) != 1):
        raise ValueError("Discontinuous sequence or MCU time")
    periods = data["irq.period_us"].to_numpy(float)*1e-6
    if np.any((periods < .0005) | (periods > .002)):
        raise ValueError("Invalid encoder snapshot period")
    delay = 9.5*float(np.median(periods)) if accel_delay_s is None else float(accel_delay_s)
    if not 0 <= delay <= .025:
        raise ValueError("Invalid acceleration averaging delay")
    scale = cfg.mm_per_tick/1000/periods
    velocity = .5*(data[RIGHT].to_numpy()-data[LEFT].to_numpy())*scale
    moving = (abs(data["ideal.velo"].to_numpy()) > .01) | (abs(data["ideal.rad_velo"].to_numpy()) > .05)
    if not moving.any():
        raise ValueError("No commanded motion interval")
    if (moving & (data["ego.rad_velo"].to_numpy() == 0.) & (data["ego.accel"].to_numpy() == 0.)
            & (data["ego.horizon_accel.get"].to_numpy() == 0.)).any():
        raise ValueError("Suspicious simultaneous zero IMU sample during motion")
    first, last = np.flatnonzero(moving)[[0, -1]]
    quiet = (np.arange(len(data)) < first) & (times < times[first]-.04)
    quiet &= (abs(data[RIGHT].to_numpy()) < .1) & (abs(data[LEFT].to_numpy()) < .1)
    if not suction:
        quiet &= abs(data["ego.rad_velo"].to_numpy()) < .1
    quiet &= (abs(data["ego.accel"].to_numpy()) < 2.) & (abs(data["ego.horizon_accel.get"].to_numpy()) < 2.)
    quiet &= ~((data["ego.rad_velo"].to_numpy() == 0.) & (data["ego.accel"].to_numpy() == 0.)
               & (data["ego.horizon_accel.get"].to_numpy() == 0.))
    if quiet.sum() < 30:
        raise ValueError("At least 30 pre-motion stationary samples are required")
    bias = [float(np.median(data[col].to_numpy()[quiet])) for col in
            ("ego.rad_velo", "ego.accel", "ego.horizon_accel.get")]
    imu = np.column_stack([np.interp(times+delay, times, data[col]) for col in ("ego.accel", "ego.horizon_accel.get")])
    gyro = data["ego.rad_velo"].to_numpy(float)
    alpha = np.gradient(gyro-bias[0], times)
    omega = gyro-bias[0]
    # a_IMU = a_axle + alpha J r - omega^2 r; geometry is a scenario, not fitted.
    imu[:, 0] += alpha*imu_left_m+omega**2*imu_forward_m
    imu[:, 1] += -alpha*imu_forward_m+omega**2*imu_left_m
    estimator = TurnPoseEkf(cfg, bias)
    estimator.state[3] = velocity[first]
    rows = []
    for i in range(first, last+1):
        if times[i]+delay > times[-1]:
            raise ValueError("Missing post-roll for acceleration delay compensation")
        if i > first:
            dt = times[i]-times[i-1]
            estimator.predict(.5*(gyro[i]+gyro[i-1]), .5*(imu[i]+imu[i-1]), dt)
        estimator.observe_forward(velocity[i], scale[i]**2/12+cfg.wheel_slip_std**2)
        row = data.iloc[i]
        if geometry is not None:
            for sensor in geometry["sensors"]:
                valid = int(row[sensor["valid_column"]]) == sensor["valid_value"]
                estimator.observe_wall(float(row[sensor["column"]])/1000, sensor, geometry["walls"], times[i], valid)
        rows.append(dict(time_s=times[i]-times[first], time_ms=times[i]*1000,
                         ideal_omega_rad_s=row["ideal.rad_velo"], **estimator.estimate()))
    metadata = dict(mouse="A", stationary_samples=int(quiet.sum()), initial_bias=bias,
                    suction_enabled=suction, noise_profile=("explicit_override" if config else "suction_on" if suction else "suction_off"),
                    accel_delay_ms=delay*1000, imu_forward_mm=imu_forward_m*1000, imu_left_mm=imu_left_m*1000,
                    rows=len(rows), median_delivery_ms=float(np.median(np.diff(times))*1000),
                    median_irq_us=float(np.median(periods)*1e6), wall_updates=estimator.wall_updates,
                    config=asdict(cfg), offline_future_samples_used=True,
                    minimum_covariance_eigenvalue=float(np.linalg.eigvalsh(estimator.covariance).min()))
    metadata["initial_bias_mad_std"] = [float(1.4826*np.median(abs(data[col].to_numpy()[quiet]-value)))
                                      for col, value in zip(("ego.rad_velo", "ego.accel", "ego.horizon_accel.get"), bias)]
    metadata["profile_sha256"] = hashlib.sha256(PROFILE_FILE.read_bytes()).hexdigest()
    return pd.DataFrame(rows), metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", required=True, type=Path)
    parser.add_argument("--settings", required=True, type=Path)
    parser.add_argument("--geometry", type=Path)
    parser.add_argument("--slip-model", action="store_true", help="吸引状態別の校正済み滑りモデルを使う6状態版")
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--accel-delay-ms", type=float)
    parser.add_argument("--imu-forward-mm", type=float, default=0.)
    parser.add_argument("--imu-left-mm", type=float, default=0.)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("出力先には新しいディレクトリを指定してください")
    settings = json.loads(args.settings.read_text(encoding="utf-8"))
    geometry = json.loads(args.geometry.read_text(encoding="utf-8")) if args.geometry else None
    additional_sources = [PROFILE_FILE]
    if args.slip_model:
        if geometry or args.accel_delay_ms is not None or args.imu_forward_mm or args.imu_left_mm:
            parser.error("--slip-modelは中心IMU・既定の平均遅れを使い、未校正の壁観測とは併用しません")
        if settings.get("mouse") != "A":
            parser.error("滑りモデルの校正は機体A用です")
        from turn_slip_conditions import replay_conditioned
        profiles = json.loads(PROFILE_FILE.read_text(encoding="utf-8"))
        frame, report = replay_conditioned(pd.read_csv(args.log), settings.get("suction", {}).get("enabled"), profiles)
        report.update(mouse="A", model="suction_conditioned_slip_ekf", offline_future_samples_used=True)
        additional_sources.append(Path(__file__).with_name("turn_slip_conditions.py"))
    else:
        frame, report = replay(pd.read_csv(args.log), settings,
                               None if args.accel_delay_ms is None else args.accel_delay_ms/1000,
                               args.imu_forward_mm/1000, args.imu_left_mm/1000, geometry)
    report["status"] = "PROVISIONAL_OFFLINE_ESTIMATE_NOT_PARAMETER_ADOPTION"
    report["sources_sha256"] = {str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest()
                                for p in [args.log, args.settings, Path(__file__)] + additional_sources + ([args.geometry] if args.geometry else [])}
    args.output.mkdir(parents=True)
    frame.to_csv(args.output/"states.csv", index=False)
    (args.output/"report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
