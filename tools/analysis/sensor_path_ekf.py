"""Causal wheel/IMU/range EKF with uncertain velocity and relative passage walls.

States (SI): east, north, left-positive yaw from north, forward velocity,
yaw rate, gyro bias, longitudinal accelerometer bias, passage centre offset.
Velocity observations can be wheel snapshots or logged ego.velo. Logged pose
and commands never enter SensorPathEkf. Commands are used by the offline CLI
only to select the same motion interval for comparison.
"""
from __future__ import annotations

import argparse
from collections import deque
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import pandas as pd

from encoder_wall_imu_ekf import LEFT, RIGHT
from streaming_wall_ekf import LocalConfig, WallRangeGate, lateral_observation


MOTION_COLUMNS = ("sequence", "time_ms", "irq.period_us", RIGHT, LEFT, "ego.rad_velo", "ego.accel")
ESTIMATED_MOTION_COLUMNS = ("sequence", "time_ms", "ego.velo", "ego.rad_velo", "ego.accel")
VELOCITY_SOURCES = ("encoders", "estimated_velocity")
WALL_COLUMNS = ("sen_l.distance", "sen_r.distance", "sen_fl.distance", "sen_fr.distance")
NAMES = ("logged_velocity", "wheel_imu", "sensor_ekf", "sensor_wall_ekf")
LABELS = ("記録速度＋IMUの積分（比較用）", "車輪パルス＋IMUの積分", "車輪＋IMUのEKF", "車輪＋IMU＋壁のEKF")
COLORS = ("#888888", "#2366b4", "#bd5b16", "#298452")


@dataclass(frozen=True)
class Config:
    mm_per_tick: float = 15.02*math.pi/172
    tread_m: float = .028
    wheel_slip_std_m_s: float = .03
    gyro_std_rad_s: float = .03
    acceleration_std_m_s2: float = 2.
    angular_acceleration_std_rad_s2: float = 80.
    velocity_walk_m_s_sqrt_s: float = .15
    position_walk_m_sqrt_s: float = .002
    gyro_bias_walk_rad_s_sqrt_s: float = .001
    accel_bias_walk_m_s2_sqrt_s: float = .01
    wall_std_m: float = .002
    wall_update_interval_s: float = .010
    estimated_velocity_std_m_s: float = .08
    estimated_accel_gain_s: float = .010

    def __post_init__(self):
        if any(not math.isfinite(v) or v <= 0 for v in asdict(self).values()):
            raise ValueError("Configuration must be finite and positive")


def wheel_measurements(row, config):
    # The latest measured interrupt period is a snapshot approximation, not
    # proof of the period of every omitted control tick. No missing count is filled.
    duration = float(row["irq.period_us"])*1e-6
    if not .0005 < duration < .002:
        raise ValueError("Invalid recorded encoder snapshot duration")
    scale = config.mm_per_tick/1000/duration
    return float(row[RIGHT])*scale, -float(row[LEFT])*scale, scale**2/6+config.wheel_slip_std_m_s**2


def motion(state, acceleration, dt):
    """Midpoint pose and longitudinal acceleration prediction, with Jacobian."""
    if not np.isfinite([*state, acceleration, dt]).all() or not 0 < dt < .1:
        raise ValueError("Invalid state, acceleration or timestamp interval")
    result = np.array(state, dtype=float, copy=True)
    a = acceleration-state[6]
    distance = state[3]*dt+.5*a*dt**2
    angle = state[2]+.5*state[4]*dt
    s, c = math.sin(angle), math.cos(angle)
    result[0] -= distance*s
    result[1] += distance*c
    result[2] += state[4]*dt
    result[3] += a*dt
    f = np.eye(8)
    f[0, 2], f[1, 2] = -distance*c, -distance*s
    f[0, 3], f[1, 3] = -dt*s, dt*c
    f[0, 4], f[1, 4] = -.5*distance*c*dt, -.5*distance*s*dt
    f[0, 6], f[1, 6] = .5*dt**2*s, -.5*dt**2*c
    f[2, 4], f[3, 6] = dt, -dt
    return result, f


class SensorPathEkf:
    """Fixed-size sequential estimator; no map, pose reset, loop closure or route."""
    def __init__(self, use_walls=True, config=None, holdout_fold=None, velocity_source="encoders"):
        if holdout_fold not in (None, 0, 1, 2):
            raise ValueError("Invalid holdout fold")
        if velocity_source not in VELOCITY_SOURCES:
            raise ValueError("Invalid velocity observation source")
        self.config = config or Config()
        self.velocity_source = velocity_source
        self.use_walls = use_walls
        self.holdout_fold = holdout_fold
        self.state = np.zeros(8)
        self.covariance = np.diag([0., 0., math.radians(1)**2, .2**2, 1., .2**2, 1., .02**2])
        self.previous = None
        self.gate = WallRangeGate(LocalConfig())
        self.axis = None
        self.wall_anchored = False
        self.last_wall_time = -math.inf
        self.wall_updates = 0
        self.wall_anchors = 0
        self.travel = 0.

    def initialize_bias(self, gyro_bias, accel_bias, ready):
        if self.previous is not None:
            raise ValueError("Calibration must precede motion")
        self.state[5:7] = gyro_bias, accel_bias
        self.covariance[5, 5] = (.02 if ready else .2)**2
        self.covariance[6, 6] = (.2 if ready else 1.)**2

    def predict(self, acceleration, dt):
        old = self.state.copy()
        self.state, f = motion(old, acceleration, dt)
        angle = old[2]+.5*old[4]*dt
        g = np.zeros((8, 2))
        g[0, 0], g[1, 0] = -.5*dt**2*math.sin(angle), .5*dt**2*math.cos(angle)
        g[3, 0], g[2, 1], g[4, 1] = dt, .5*dt**2, dt
        q = g@np.diag([self.config.acceleration_std_m_s2**2,
                       self.config.angular_acceleration_std_rad_s2**2])@g.T
        for index, std in ((0, self.config.position_walk_m_sqrt_s), (1, self.config.position_walk_m_sqrt_s),
                           (3, self.config.velocity_walk_m_s_sqrt_s), (5, self.config.gyro_bias_walk_rad_s_sqrt_s),
                           (6, self.config.accel_bias_walk_m_s2_sqrt_s)):
            q[index, index] += std**2*dt
        self.covariance = f@self.covariance@f.T+q
        return g[:, 0]

    def observe(self, value, h, variance, reject=False, error_noise_cross=None):
        residual = float(value-h@self.state)
        innovation = float(h@self.covariance@h+variance)
        cross = None if error_noise_cross is None else np.asarray(error_noise_cross, dtype=float)
        if cross is not None:
            if cross.shape != (8,) or not np.isfinite(cross).all():
                raise ValueError("Invalid prior-error/measurement-noise cross covariance")
            innovation += 2*float(h@cross)
        if not math.isfinite(innovation) or innovation <= 0:
            raise ValueError("Invalid innovation covariance")
        if reject and (abs(residual) > .020 or residual**2/innovation > 9.):
            return False
        gain = (self.covariance@h+(0. if cross is None else cross))/innovation
        self.state += gain*residual
        a = np.eye(8)-np.outer(gain, h)
        self.covariance = a@self.covariance@a.T+np.outer(gain, gain)*variance
        if cross is not None:
            correction = np.outer(a@cross, gain)
            self.covariance -= correction+correction.T
        self.covariance = .5*(self.covariance+self.covariance.T)
        return True

    def anchor_wall(self, lateral, variance):
        # Introduce an unknown centre line correlated with the current pose.
        # The first range sets this nuisance offset, without changing pose or
        # reusing that same observation as an independent second update.
        normal = np.array([-math.cos(self.axis), -math.sin(self.axis)])
        h = np.r_[normal, np.zeros(6)]
        cross = self.covariance@h
        offset_variance = float(h@cross+variance)
        self.state[7] = h@self.state-lateral
        self.covariance[:, 7] = cross
        self.covariance[7, :] = cross
        self.covariance[7, 7] = offset_variance
        self.wall_anchored = True
        self.wall_anchors += 1

    def update(self, sample, wall_heldout=None):
        required = MOTION_COLUMNS if self.velocity_source == "encoders" else ESTIMATED_MOTION_COLUMNS
        row = {key: float(sample[key]) for key in required}
        if not np.isfinite(list(row.values())).all():
            raise ValueError("Nonfinite required sensor value")
        if self.velocity_source == "encoders":
            right, left, variance = wheel_measurements(row, self.config)
            observed_velocity = .5*(right+left)
        else:
            observed_velocity = row["ego.velo"]
        previous = self.previous
        dt = 0.
        acceleration_response = np.zeros(8)
        if previous is not None:
            dt = (row["time_ms"]-previous["time_ms"])/1000
            if row["sequence"] != previous["sequence"]+1 or not 0 < dt < .1:
                raise ValueError("Discontinuous sequence/time; split captures first")
            acceleration_response = self.predict(.5*(row["ego.accel"]+previous["ego.accel"]), dt)
        if self.velocity_source == "encoders":
            for value, direction in ((right, 1.), (left, -1.)):
                h = np.zeros(8)
                h[3], h[4] = 1., direction*self.config.tread_m/2
                self.observe(value, h, variance)
        else:
            # Firmware v_est = mean(wheel_v) + tau * mean(accel), tau=10ms.
            # Its acceleration bias enters the speed observation. The current
            # acceleration also enters half of the trapezoidal prediction:
            # Cov(true-minus-predicted error, speed noise) = -G*(1/2)*tau*Ra.
            # Temporal FIR correlations/latency remain an explicit approximation.
            tau = self.config.estimated_accel_gain_s
            accel_variance = self.config.acceleration_std_m_s2**2
            cross = -.5*acceleration_response*tau*accel_variance
            h = np.zeros(8)
            h[3], h[6] = 1., tau
            variance = self.config.estimated_velocity_std_m_s**2+tau**2*accel_variance
            self.observe(observed_velocity, h, variance, error_noise_cross=cross)
        h = np.zeros(8)
        h[4:6] = 1.
        self.observe(row["ego.rad_velo"], h, self.config.gyro_std_rad_s**2)
        # Fold assignment uses motion inputs, never wall values. Replay can
        # supply a common external mask to compare input sources on the same blocks.
        self.travel += abs(observed_velocity)*dt
        heldout = self.holdout_fold is not None and int(self.travel/.036) % 3 == self.holdout_fold
        if wall_heldout is not None:
            heldout = bool(wall_heldout)
        active = abs(self.state[3]) > .2 and abs(row["ego.rad_velo"]-self.state[5]) < 1.2
        updated, mask = False, 0
        if not active:
            self.gate.reset()
            self.axis, self.wall_anchored = None, False
        elif self.use_walls and not heldout:
            if self.axis is None:
                candidate = round(self.state[2]/(math.pi/2))*(math.pi/2)
                if abs(self.state[2]-candidate) < math.radians(25):
                    self.axis = candidate
            if self.axis is not None:
                wall_row = {key: float(sample[key]) for key in WALL_COLUMNS}
                if not np.isfinite(list(wall_row.values())).all():
                    raise ValueError("Nonfinite required wall value")
                wall_row["ego.rad_velo"] = row["ego.rad_velo"]-self.state[5]
                mask = self.gate.update(wall_row, abs(observed_velocity)*dt)
                measurement = lateral_observation(wall_row, mask, self.gate.config)
                if measurement and (row["time_ms"]/1000-self.last_wall_time >= self.config.wall_update_interval_s):
                    lateral, base_variance = measurement
                    wall_variance = base_variance+self.config.wall_std_m**2
                    if not self.wall_anchored:
                        self.anchor_wall(lateral, wall_variance)
                    else:
                        h = np.zeros(8)
                        h[:2] = -math.cos(self.axis), -math.sin(self.axis)
                        h[7] = -1.
                        updated = self.observe(lateral, h, wall_variance, reject=True)
                        self.wall_updates += int(updated)
                    self.last_wall_time = row["time_ms"]/1000
        else:
            self.gate.reset()
        self.previous = row
        return {"sequence": int(row["sequence"]), "time_ms": row["time_ms"],
                "east_mm": self.state[0]*1000, "north_mm": self.state[1]*1000,
                "heading_rad": self.state[2], "velocity_m_s": self.state[3], "omega_rad_s": self.state[4],
                "gyro_bias_rad_s": self.state[5], "accel_bias_m_s2": self.state[6],
                "std_position_mm": math.sqrt(max(0., np.trace(self.covariance[:2, :2])))*1000,
                "wall_updated": updated, "wall_mask": mask, "heldout": heldout}


def calibrate(rows, config, velocity_source="encoders"):
    quiet = deque(maxlen=500)
    skipped, frozen = 0, 0
    for row in rows:
        duration = float(row.get("irq.period_us", float("nan")))
        if velocity_source == "encoders" and (not math.isfinite(duration) or not 500 < duration < 2000):
            skipped += 1
            quiet.clear()
            continue
        if float(row["ego.rad_velo"]) == 0. and float(row["ego.accel"]) == 0.:
            frozen += 1
            continue
        if velocity_source == "encoders":
            right, left, _ = wheel_measurements(row, config)
            quiet_velocity = max(abs(right), abs(left))
        else:
            quiet_velocity = abs(float(row["ego.velo"]))
        if quiet_velocity < .02 and abs(float(row["ego.rad_velo"])) < .035:
            quiet.append((float(row["ego.rad_velo"]), float(row["ego.accel"])))
    values = np.asarray(quiet)
    # Error-state telemetry can consist of frozen exact zero ego values. This
    # cannot establish sensor bias even if hundreds of such rows were received.
    # A healthy still gyro can remain on the same binary16 quantization level.
    # Reject the simultaneous zero placeholder above, not low sample diversity.
    ready = len(quiet) >= 30
    gyro, acceleration = np.median(values, axis=0) if ready else (0., 0.)
    return {"gyro_bias_rad_s": float(gyro), "accel_bias_m_s2": float(acceleration),
            "ready": bool(ready), "quiet_samples": len(quiet), "skipped_pre_motion_invalid_duration": skipped,
            "skipped_frozen_zero_sensor_pairs": frozen}


def integrate(velocity, gyro, timestamps):
    dt = np.r_[0., np.diff(timestamps)/1000]
    mean_v = .5*(velocity+np.r_[velocity[0], velocity[:-1]])
    mean_w = .5*(gyro+np.r_[gyro[0], gyro[:-1]])
    delta = mean_w*dt
    yaw = np.cumsum(delta)
    return np.c_[np.cumsum(-mean_v*dt*np.sin(yaw-.5*delta)),
                 np.cumsum(mean_v*dt*np.cos(yaw-.5*delta))]*1000


def replay(frame, config=None, holdout_fold=None, velocity_source="encoders", calibration_override=None, wall_holdout=None):
    config = config or Config()
    # Commands select the intended motion span; their values and logged pose
    # are not passed to either estimator. ego.velo is optional speed observation.
    moving = np.flatnonzero((frame["ideal.velo"].abs() > .01)|(frame["ideal.rad_velo"].abs() > .05))
    if not len(moving):
        raise ValueError("No motion interval")
    first, last = int(moving[0]), int(moving[-1])
    rows = frame.to_dict("records")
    calibration = (dict(calibration_override) if calibration_override is not None
                   else calibrate(rows[:first], config, velocity_source))
    if wall_holdout is not None and len(wall_holdout) != last-first+1:
        raise ValueError("Holdout mask must match the common motion interval")
    estimators = {name: SensorPathEkf(use_walls=name == "sensor_wall_ekf", config=config, holdout_fold=holdout_fold,
                                     velocity_source=velocity_source)
                  for name in NAMES[2:]}
    states = []
    for estimator in estimators.values():
        estimator.initialize_bias(calibration["gyro_bias_rad_s"], calibration["accel_bias_m_s2"], calibration["ready"])
    for index, sample in enumerate(rows[first:last+1]):
        values = {}
        for name, estimator in estimators.items():
            result = estimator.update(sample, wall_heldout=None if wall_holdout is None else wall_holdout[index])
            if not np.isfinite(estimator.state).all() or np.linalg.eigvalsh(estimator.covariance).min() < -1e-10:
                raise RuntimeError("Invalid state/covariance")
            values.update({name+"."+key: value for key, value in result.items() if key not in ("sequence", "time_ms")})
        states.append({"sequence": int(sample["sequence"]), "time_ms": float(sample["time_ms"]), **values})
    output = pd.DataFrame(states)
    selected = frame.iloc[first:last+1]
    wheels = np.array([wheel_measurements(row, config)[:2] for row in rows[first:last+1]])
    gyro = selected["ego.rad_velo"].to_numpy(float)-calibration["gyro_bias_rad_s"]
    for name, velocity in (("logged_velocity", selected["ego.velo"].to_numpy(float)), ("wheel_imu", wheels.mean(axis=1))):
        xy = integrate(velocity, gyro, output.time_ms.to_numpy(float))
        output[name+".east_mm"], output[name+".north_mm"] = xy.T
    paths = {name: output[[name+".east_mm", name+".north_mm"]].to_numpy(float) for name in NAMES}
    metrics = []
    for name, xy in paths.items():
        delta = np.linalg.norm(xy-paths["wheel_imu"], axis=1)
        metrics.append({"method": name, "endpoint_east_mm": float(xy[-1, 0]), "endpoint_north_mm": float(xy[-1, 1]),
                        "endpoint_distance_from_origin_mm_not_accuracy": float(np.linalg.norm(xy[-1])),
                        "path_length_mm": float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum()),
                        "p95_separation_from_wheel_imu_mm": float(np.percentile(delta, 95)),
                        "max_separation_from_wheel_imu_mm": float(delta.max())})
    report = {"motion_first_row": first, "motion_last_row_inclusive": last, "motion_rows": len(output),
              "velocity_source": velocity_source, "shared_calibration": calibration_override is not None,
              "calibration": calibration, "config": asdict(config), "metrics": metrics,
              "wall_updates": estimators["sensor_wall_ekf"].wall_updates,
              "wall_anchors": estimators["sensor_wall_ekf"].wall_anchors,
              "wall_vs_no_wall_p95_separation_mm": float(np.percentile(np.linalg.norm(paths["sensor_wall_ekf"]-paths["sensor_ekf"], axis=1), 95)),
              "maximum_abs_gyro_bias_deg_s": float(np.degrees(output["sensor_wall_ekf.gyro_bias_rad_s"].abs().max())),
              "maximum_abs_accel_bias_m_s2": float(output["sensor_wall_ekf.accel_bias_m_s2"].abs().max())}
    return output, report


def plot(output, frame, title, velocity_source="encoders"):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    fonts = {font.name for font in font_manager.fontManager.ttflist}
    plt.rcParams["font.family"] = next((name for name in ("Yu Gothic", "Meiryo", "MS Gothic") if name in fonts), "DejaVu Sans")
    plt.rcParams["axes.unicode_minus"] = False
    paths = [frame[[name+".east_mm", name+".north_mm"]].to_numpy(float) for name in NAMES]
    points = np.vstack(paths)
    low, high = points.min(axis=0)-45, points.max(axis=0)+45
    fig, axes = plt.subplots(1, 2, figsize=(13, 7), constrained_layout=True)
    for axis in axes:
        axis.scatter(0, 0, c="black", marker="o", s=30, zorder=5)
        axis.set_aspect("equal", adjustable="box")
        axis.grid(alpha=.2)
        axis.set(xlabel="東 [mm]", ylabel="北 [mm]", xlim=(low[0], high[0]), ylim=(low[1], high[1]))
    labels = LABELS if velocity_source == "encoders" else (*LABELS[:2], "推定速度＋IMUのEKF", "推定速度＋IMU＋壁のEKF")
    for name, label, color, xy in zip(NAMES, labels, COLORS, paths):
        axes[0].plot(*xy.T, lw=1.2, color=color, label=label)
        axes[0].scatter(*xy[-1], marker="x", color=color, s=30)
    trusted = frame["sensor_wall_ekf.wall_updated"].to_numpy(bool)
    axes[1].plot(*paths[-1].T, color=COLORS[-1], lw=1.1, label="センサ＋壁EKFの経路")
    axes[1].scatter(*paths[-1][trusted].T, s=7, color="#6d28a8", label="壁更新を採用した点", zorder=3)
    axes[0].legend(fontsize=8, loc="lower center", bbox_to_anchor=(.5, -.30))
    axes[1].legend(fontsize=8, loc="lower center", bbox_to_anchor=(.5, -.22))
    fig.suptitle(title+"\n共通のMCU時間軸・共通始点。終点・角度・縮尺の合わせなし。", fontsize=12)
    fig.savefig(output/"paths_overlay.png", dpi=170, bbox_inches="tight")
    plt.close(fig)


def plot_signals(output, source, states, title):
    import matplotlib.pyplot as plt
    selected = source.set_index("sequence").loc[states.sequence]
    time = (states.time_ms-states.time_ms.iloc[0])/1000
    wheels = np.array([wheel_measurements(row, Config())[:2] for row in selected.to_dict("records")])
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True, constrained_layout=True)
    axes[0].plot(time, wheels.mean(axis=1), color="#c9c9c9", lw=.6, label="間引きパルスからの速度観測")
    axes[0].plot(time, selected["ego.velo"].to_numpy(), color="#888888", lw=.8, label="記録速度（比較用）")
    axes[0].plot(time, states["sensor_wall_ekf.velocity_m_s"], color="#298452", lw=.9, label="EKF推定速度")
    axes[0].set_ylabel("速度 [m/s]")
    axes[1].plot(time, selected["ego.rad_velo"].to_numpy(), color="#888888", lw=.8, label="記録ジャイロ")
    axes[1].plot(time, states["sensor_wall_ekf.omega_rad_s"], color="#298452", lw=.9, label="EKF角速度")
    axes[1].set_ylabel("角速度 [rad/s]")
    axes[2].plot(time, selected["sen_l.distance"].to_numpy(), lw=.7, label="左壁距離")
    axes[2].plot(time, selected["sen_r.distance"].to_numpy(), lw=.7, label="右壁距離")
    mask = states["sensor_wall_ekf.wall_updated"].to_numpy(bool)
    axes[2].scatter(time[mask], np.full(mask.sum(), 25.), s=7, color="#6d28a8", label="壁更新")
    axes[2].set_ylabel("壁距離 [mm]")
    axes[3].plot(time, np.degrees(states["sensor_wall_ekf.gyro_bias_rad_s"]), color="#bd5b16", label="推定ジャイロバイアス")
    axes[3].set(xlabel="MCU経過時間 [s]", ylabel="バイアス [deg/s]")
    for axis in axes:
        axis.grid(alpha=.2)
        axis.legend(fontsize=8, loc="upper right")
    fig.suptitle(title+" · 観測値と推定値（外部の正解値なし）", fontsize=12)
    fig.savefig(output/"sensor_signals.png", dpi=160, bbox_inches="tight")
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", required=True, type=Path)
    parser.add_argument("--settings", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--title", default="機体A · 速度も状態に含めた逐次センサEKF")
    parser.add_argument("--velocity-source", choices=VELOCITY_SOURCES, default="encoders")
    args = parser.parse_args(argv)
    metadata = json.loads(args.settings.read_text(encoding="utf-8"))
    if metadata.get("firmware", {}).get("mouse") != "A":
        raise ValueError("Only verified Mouse A calibration/signs are supported")
    inputs = {str(path.resolve()): hashlib.sha256(path.read_bytes()).hexdigest() for path in (args.log, args.settings)}
    frame = pd.read_csv(args.log)
    if not (frame.sequence.diff().dropna() == 1).all() or not frame.time_ms.diff().dropna().between(1, 99).all():
        raise ValueError("Split disconnected captures before replay")
    # Interrupt timing is explicitly invalid immediately after its reset. Such
    # pre-motion rows are skipped for calibration, never imputed. Motion rows
    # still require finite timing in SensorPathEkf.update.
    needed = (*(key for key in MOTION_COLUMNS if key != "irq.period_us"), *WALL_COLUMNS, "ego.velo", "ideal.velo", "ideal.rad_velo")
    if not np.isfinite(frame[list(needed)].to_numpy(float)).all():
        raise ValueError("Missing/nonfinite required columns")
    states, report = replay(frame, velocity_source=args.velocity_source)
    args.output.mkdir(parents=True, exist_ok=True)
    states.to_csv(args.output/"paths_and_states.csv", index=False)
    pd.DataFrame(report["metrics"]).to_csv(args.output/"path_metrics.csv", index=False)
    report.update({"input_sha256": inputs, "machine": "A", "settings": metadata,
        "state_dimension": 8, "states": ["east_m", "north_m", "yaw_rad", "velocity_m_s", "omega_rad_s", "gyro_bias_rad_s", "accel_bias_m_s2", "passage_offset_m"],
        "time_basis": "MCU timestamp differences; wheel snapshot duration uses recorded irq.period_us",
        "endpoint_fitted": False, "scale_fitted": False, "ground_truth_available": False,
        "limitations": ["Wheel counts are decimated one-interrupt snapshots; omitted ticks are not recoverable.",
            "MCU and control-integral clocks disagree; no endpoint-based clock correction is applied.",
            "IMU columns are calibrated gyro and 20-tick averaged longitudinal acceleration, not raw ADC.",
            "Latest interrupt period approximates the wheel snapshot duration; unrecorded tick periods are unknown.",
            "No independent bias/slip/scale ground truth; bias states may absorb model and timing errors.",
            "Wall observations assume near-cardinal straight passages, nominal 45mm side reference, and small yaw.",
            "Passage offset is newly initialized with pose correlation; no known absolute wall/map/loop identity is used.",
            "Noise assumptions are fixed trial values; filtered acceleration has temporal correlation/delay not explicitly modelled.",
            "Comparison distances are between estimates, not absolute position accuracy. Shortest runs did not return home.",
            "Commands only select motion span; ego.velo is an EKF speed observation only in estimated_velocity mode.",
            "Estimated speed contains wheel averages plus 10ms times the same IMU acceleration; the instantaneous shared-noise cross covariance and acceleration bias are approximated explicitly, but FIR time correlations are not identified.",
            "Estimated-velocity mode removes both direct wheel updates, including wheel-difference yaw information; gyro and walls must constrain yaw/bias."]})
    root = Path(__file__).resolve().parents[2]
    implementation = [Path(__file__).resolve(), root/"tools/analysis/encoder_wall_imu_ekf.py", root/"tools/analysis/streaming_wall_ekf.py",
                      root/"Params/mouse_config.h", root/"Peripheral/Src/encoder.c", root/"Module/Src/interrupt.cpp", root/"Module/Src/interrupt_timing.cpp"]
    report["implementation_sha256"] = {str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest() for path in implementation}
    assert all(hashlib.sha256(Path(path).read_bytes()).hexdigest() == digest for path, digest in inputs.items())
    (args.output/"report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    plot(args.output, states, args.title, args.velocity_source)
    plot_signals(args.output, frame, states, args.title)
    print(json.dumps({key: report[key] for key in ("motion_rows", "calibration", "wall_updates", "wall_anchors", "metrics")}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
