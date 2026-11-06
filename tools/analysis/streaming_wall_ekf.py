"""Causal, sample-at-a-time wall EKFs; saved-CSV replay or growing-CSV follow.

Global state: east, north, yaw, gyro bias (SI units).
Local state: corridor-relative left-positive lateral displacement and yaw error.
The local model applies to straight passages, not a frame fixed to the robot's
instantaneous origin. It is suspended/reinitialized across turns and stops.
No future interpolation, route input, endpoint fit, or full-log calibration.
"""
from __future__ import annotations

import argparse
from collections import deque
import csv
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np

from wall_heading_ekf import Config as GlobalConfig
from wall_heading_ekf import WallHeadingEkf, wall_heading_measurement, wrap_angle


REQUIRED = ("sequence", "time_ms", "period_ms", "ideal.velo", "ego.velo", "ideal.rad_velo",
            "ego.rad_velo", "ideal.radian", "ego.radian", "ego.turn_x", "ego.turn_y",
            "sen_l.distance", "sen_r.distance", "sen_fl.distance", "sen_fr.distance")


@dataclass(frozen=True)
class LocalConfig:
    reference_mm: float = 45.
    side_noise_mm: float = 1.
    calibration_noise_mm: float = .75
    lateral_walk_mm_sqrt_s: float = 2.
    gyro_walk_deg_sqrt_s: float = .3
    initial_lateral_std_mm: float = 12.
    initial_heading_std_deg: float = 8.
    confirm_distance_mm: float = 6.
    confirm_samples: int = 3
    max_range_step_mm: float = 1.5
    innovation_limit_mm: float = 20.
    nis_limit: float = 9.

    def __post_init__(self):
        if any(not math.isfinite(v) or v <= 0 for v in asdict(self).values()):
            raise ValueError("Local parameters must be finite and positive")
        if not isinstance(self.confirm_samples, int):
            raise ValueError("confirm_samples must be an integer")


def local_motion(state, travel_m, delta_yaw_rad):
    """Midpoint integration of e'=v*sin(psi), psi'=omega in a straight passage."""
    state = np.asarray(state, dtype=float)
    mid = state[1] + .5 * delta_yaw_rad
    predicted = state + [travel_m * math.sin(mid), delta_yaw_rad]
    jacobian = np.array([[1., travel_m * math.cos(mid)], [0., 1.]])
    return predicted, jacobian


class LateralHeadingEkf:
    """Exactly two estimated states [left-positive e_m, left-positive psi_rad]."""

    def __init__(self, config=None):
        self.config = config or LocalConfig()
        self.reset()

    def reset(self):
        self.state = np.zeros(2)
        self.covariance = np.diag([(self.config.initial_lateral_std_mm / 1000) ** 2,
                                   math.radians(self.config.initial_heading_std_deg) ** 2])
        self.updates = 0

    def predict(self, travel_m, delta_yaw_rad, dt):
        if not np.isfinite([travel_m, delta_yaw_rad, dt]).all() or dt <= 0 or dt >= .1:
            raise ValueError("Invalid local motion increment or time units")
        self.state, f = local_motion(self.state, travel_m, delta_yaw_rad)
        q = np.diag([(self.config.lateral_walk_mm_sqrt_s / 1000) ** 2 * dt,
                     math.radians(self.config.gyro_walk_deg_sqrt_s) ** 2 * dt])
        self.covariance = f @ self.covariance @ f.T + q
        self.covariance = .5 * (self.covariance + self.covariance.T)

    def observe_lateral(self, value_m, variance_m2):
        if not math.isfinite(value_m) or not math.isfinite(variance_m2) or variance_m2 <= 0:
            raise ValueError("Invalid local observation")
        residual = value_m - self.state[0]
        innovation_variance = self.covariance[0, 0] + variance_m2
        if abs(residual) > self.config.innovation_limit_mm / 1000 or residual ** 2 / innovation_variance > self.config.nis_limit:
            return False
        k = self.covariance[:, 0] / innovation_variance
        self.state += k * residual
        a = np.eye(2)
        a[:, 0] -= k
        self.covariance = a @ self.covariance @ a.T + np.outer(k, k) * variance_m2
        self.covariance = .5 * (self.covariance + self.covariance.T)
        self.updates += 1
        return True


class LateralOnlyKalman:
    """One-state random-walk comparator: wall smoothing without heading estimation."""

    def __init__(self, config):
        self.config = config
        self.reset()

    def reset(self):
        self.value = 0.
        self.variance = (self.config.initial_lateral_std_mm / 1000) ** 2

    def predict(self, dt):
        self.variance += (self.config.lateral_walk_mm_sqrt_s / 1000) ** 2 * dt

    def observe(self, value, variance):
        residual = value - self.value
        if abs(residual) > self.config.innovation_limit_mm / 1000 or residual ** 2 / (self.variance + variance) > self.config.nis_limit:
            return False
        gain = self.variance / (self.variance + variance)
        self.value += gain * residual
        self.variance = (1-gain) ** 2 * self.variance + gain ** 2 * variance
        return True


class CausalOdometry:
    """Unwrap logged local integrals using only the current and earlier rows.

    A 21-element trailing median replaces the former centered interpolation.
    Before a valid local interval exists, only the current timestamp is used.
    This does not establish that timestamp and control clocks are equivalent.
    """

    def __init__(self):
        self.previous = None
        self.started = False
        self.durations = deque(maxlen=21)
        self.yaw = 0.
        self.xy = np.zeros(2)

    def update(self, row):
        if not all(math.isfinite(float(row[key])) for key in REQUIRED):
            raise ValueError("Nonfinite/missing required sample field")
        if self.previous is not None:
            if row["sequence"] != self.previous["sequence"] + 1:
                raise ValueError("Discontinuous sequence; start a new estimator after a gap")
            timestamp_dt = (row["time_ms"] - self.previous["time_ms"]) / 1000
            if not 0 < timestamp_dt < .1:
                raise ValueError("Invalid timestamp interval; use a single uninterrupted stream")
        else:
            timestamp_dt = row["period_ms"] / 1000
            if not 0 < timestamp_dt < .1:
                raise ValueError("Invalid sample period units")
        moving_command = abs(row["ideal.velo"]) > .01 or abs(row["ideal.rad_velo"]) > .05
        just_started = moving_command and not self.started
        if just_started:
            self.started = True
        previous = self.previous
        self.previous = {key: float(row[key]) for key in REQUIRED}
        result = {"started": self.started, "just_started": just_started, "dt": timestamp_dt,
                  "dt_source": "timestamp_fallback", "step_m": np.zeros(2), "travel_m": 0.,
                  "delta_yaw": 0., "gyro_delta": 0., "raw_yaw": self.yaw, "raw_xy_m": self.xy.copy(), "reset": False}
        if not self.started or just_started or previous is None:
            return result
        old_xy = np.array([previous["ego.turn_x"], previous["ego.turn_y"]])
        xy = np.array([row["ego.turn_x"], row["ego.turn_y"]])
        delta_xy = xy - old_xy
        norm, old_norm, step_norm = np.linalg.norm(xy), np.linalg.norm(old_xy), np.linalg.norm(delta_xy)
        coordinate_reset = step_norm > 8 or (norm < .5 and old_norm > 1) or (norm < 3 and old_norm > 8)
        velocity = .5 * (row["ego.velo"] + previous["ego.velo"])
        if abs(velocity) > .1 and not coordinate_reset:
            inferred = step_norm / abs(velocity) / 1000
            if .003 < inferred < .010:
                self.durations.append(float(inferred))
        dt = float(np.median(self.durations)) if self.durations else timestamp_dt
        expected_yaw = .5 * (row["ego.rad_velo"] + previous["ego.rad_velo"]) * dt
        local_yaw = row["ego.radian"]
        delta_yaw = local_yaw - previous["ego.radian"]
        angle_reset = (abs(delta_yaw) > .3 or
                       (abs(delta_yaw - expected_yaw) > .02 and abs(local_yaw) < .025) or
                       (coordinate_reset and abs(local_yaw) < .025 and abs(delta_yaw-expected_yaw) > .002))
        if angle_reset:
            delta_yaw = expected_yaw
        old_yaw = self.yaw
        self.yaw += delta_yaw
        offset = .5 * ((old_yaw-previous["ego.radian"]) + (self.yaw-local_yaw))
        east = -(delta_xy[0]*math.cos(offset)+delta_xy[1]*math.sin(offset))/1000
        north = (-delta_xy[0]*math.sin(offset)+delta_xy[1]*math.cos(offset))/1000
        reset = bool(coordinate_reset or angle_reset)
        if reset:
            east = -velocity*dt*math.sin((old_yaw+self.yaw)/2)
            north = velocity*dt*math.cos((old_yaw+self.yaw)/2)
        step = np.array([east, north])
        self.xy += step
        result.update(dt=dt, dt_source="trailing_local_integrals" if self.durations else "timestamp_fallback",
                      step_m=step, travel_m=float(np.linalg.norm(step))*np.sign(velocity),
                      delta_yaw=delta_yaw, gyro_delta=expected_yaw, raw_yaw=self.yaw, raw_xy_m=self.xy.copy(), reset=reset)
        return result


def straight_candidate(row):
    # No logged motion ID: nonzero ideal.radian also excludes slow turn tails.
    return row["ideal.velo"] > .2 and abs(row["ideal.rad_velo"]) <= 1.21 and abs(row["ideal.radian"]) < .05


class WallRangeGate:
    def __init__(self, config):
        self.config = config
        self.reset()

    def reset(self):
        self.previous = [None, None]
        self.distance = [0., 0.]
        self.count = [0, 0]

    def update(self, row, travel_m):
        mask = 0
        front_clear = min(row["sen_fl.distance"], row["sen_fr.distance"]) > 112
        for index, side in enumerate(("l", "r")):
            value = row["sen_"+side+".distance"]
            previous = self.previous[index]
            valid = front_clear and 30 < value < 60 and abs(row["ego.rad_velo"]) < 1.5
            if not valid:
                self.previous[index], self.distance[index], self.count[index] = None, 0., 0
                continue
            if previous is None or abs(value-previous) > self.config.max_range_step_mm:
                self.distance[index], self.count[index] = 0., 1
            else:
                self.distance[index] += max(0., travel_m)*1000
                self.count[index] += 1
            self.previous[index] = value
            if self.distance[index] >= self.config.confirm_distance_mm and self.count[index] >= self.config.confirm_samples:
                mask |= 1 << index
        return mask


def lateral_observation(row, mask, config):
    if mask == 3:
        # Common-mode distance calibration error cancels in the difference.
        value = (row["sen_r.distance"]-row["sen_l.distance"])/2
        variance = .5*config.side_noise_mm**2 + config.calibration_noise_mm**2
    elif mask == 1:
        value = config.reference_mm-row["sen_l.distance"]
        variance = config.side_noise_mm**2+config.calibration_noise_mm**2
    elif mask == 2:
        value = row["sen_r.distance"]-config.reference_mm
        variance = config.side_noise_mm**2+config.calibration_noise_mm**2
    else:
        return None
    return value/1000, variance/1e6


class SpatialBlocks:
    """Disjoint 18mm blocks; fold membership is known before any wall update.

    Partitioning uses motion commands and wheel displacement, not wall values.
    Held-out blocks are excluded in their entirety, including local range updates.
    """

    def __init__(self, fold=None):
        if fold not in (None, 0, 1, 2):
            raise ValueError("fold must be None, 0, 1 or 2")
        self.fold = fold
        self.number = -1
        self.rows = []
        self.distance = 0.

    def begin(self, active):
        if not active:
            self.rows = []
            self.distance = 0.
            return False, False
        new = not self.rows
        if new:
            self.number += 1
        return self.fold is not None and self.number % 3 == self.fold, new

    def append(self, entry):
        if self.rows:
            self.distance += max(0., entry["travel_m"])
        entry = {**entry, "spatial_m": self.distance}
        self.rows.append(entry)
        if len(self.rows) >= 128:
            self.rows, self.distance = [], 0.
            return None
        if self.distance < .018 or len(self.rows) < 6:
            return None
        completed = self.rows
        self.rows, self.distance = [], 0.
        return completed


def block_observation(block, global_config):
    travel = np.array([row["spatial_m"] for row in block])
    slopes, variances, sides = [], [], []
    if any(min(row["sen_fl.distance"], row["sen_fr.distance"]) <= 112 or abs(row["ego.rad_velo"]) >= 1.5 for row in block):
        return None
    for side in ("l", "r"):
        values = np.array([row["sen_"+side+".distance"] for row in block])
        if not ((values > 30) & (values < 60)).all() or max(abs(np.diff(values))) > 1.5:
            continue
        slope, residual, denominator = wall_heading_measurement(
            travel, values, "left" if side == "l" else "right")
        if abs(slope) > math.sin(math.radians(15)) or residual > .00075:
            continue
        slopes.append(slope)
        sides.append(side)
        variances.append(max(residual, global_config.sensor_noise_floor_mm/1000)**2/denominator)
    if not sides or (len(sides) == 2 and abs(slopes[0]-slopes[1]) > .12):
        return None
    yaw = np.array([row["raw_yaw"] for row in block])
    axis = round(float(yaw.mean())/(math.pi/2))*(math.pi/2)
    weights = 1/np.array(variances)
    slope = float(np.average(slopes, weights=weights))
    mean_heading = math.asin(slope)
    axis_valid = abs(wrap_angle(float(yaw.mean())-axis)) <= math.radians(25)
    return {"sides": sides, "axis": axis, "axis_valid": axis_valid,
            "mean_local_heading": mean_heading,
            "heading": axis+mean_heading+yaw[-1]-float(yaw.mean()),
            "variance": 1/weights.sum()/(1-slope**2)+math.radians(global_config.heading_noise_floor_deg)**2}


class StreamingWallEstimator:
    """Call update(sample) once per received row; memory is bounded independently of run length."""

    def __init__(self, holdout_fold=None, local_config=None, global_config=None):
        self.local_config = local_config or LocalConfig()
        self.global_config = global_config or GlobalConfig()
        self.odometry = CausalOdometry()
        self.local = LateralHeadingEkf(self.local_config)
        self.lateral_only = LateralOnlyKalman(self.local_config)
        self.gate = WallRangeGate(self.local_config)
        self.blocks = SpatialBlocks(holdout_fold)
        self.quiet = deque(maxlen=500)
        self.bias = 0.
        self.bias_ready = False
        self.global_ekf = None
        self.bias_path = None
        self.local_active = False
        self.segment = -1
        self.previous_heldout = False
        self.last_validation = None
        self.updates_global = 0

    def update(self, sample):
        row = {key: float(sample[key]) for key in REQUIRED}
        odo = self.odometry.update(row)
        self.last_validation = None
        if not odo["started"]:
            if abs(row["ego.velo"]) < .01 and abs(row["ego.rad_velo"]) < .035 and abs(row["ideal.rad_velo"]) < .001:
                self.quiet.append(row["ego.rad_velo"])
            return {"sequence": int(row["sequence"]), "time_ms": row["time_ms"], "started": False}
        if odo["just_started"]:
            self.bias_ready = len(self.quiet) >= 30
            self.bias = float(np.median(self.quiet)) if self.bias_ready else 0.
            self.global_ekf = WallHeadingEkf(self.bias, self.global_config)
            self.bias_path = WallHeadingEkf(self.bias, self.global_config)
        if not odo["just_started"]:
            before = odo["raw_yaw"]-odo["delta_yaw"]
            for filt in (self.global_ekf, self.bias_path):
                filt.predict(odo["step_m"], before, odo["delta_yaw"], odo["dt"], odo["reset"])
        active = straight_candidate(row)
        if active and not self.local_active:
            self.local.reset()
            self.lateral_only.reset()
            self.gate.reset()
            self.segment += 1
        if not active:
            self.gate.reset()
        elif self.local_active:
            self.local.predict(odo["travel_m"], odo["gyro_delta"]-self.bias*odo["dt"], odo["dt"])
            self.lateral_only.predict(odo["dt"])
        self.local_active = active
        heldout, new_block = self.blocks.begin(active)
        if new_block and (heldout or self.previous_heldout):
            self.gate.reset()
        self.previous_heldout = heldout
        prior_local = self.local.state.copy()
        prior_global = self.global_ekf.state.copy()
        prior_lateral = self.lateral_only.value
        completed = None
        if active:
            completed = self.blocks.append({**row, **odo, "local": prior_local, "global": prior_global,
                                            "lateral_only": prior_lateral, "bias_path": self.bias_path.state.copy(), "heldout": heldout})
        mask, accepted = 0, False
        if active and not heldout:
            mask = self.gate.update(row, odo["travel_m"])
            observation = lateral_observation(row, mask, self.local_config)
            if observation is not None:
                accepted = self.local.observe_lateral(*observation)
                self.lateral_only.observe(*observation)
        if completed:
            observation = block_observation(completed, self.global_config)
            if observation:
                if heldout:
                    self.last_validation = score_block(completed, observation, self.local_config)
                elif observation["axis_valid"]:
                    self.updates_global += int(self.global_ekf.observe_heading(
                        observation["heading"], observation["variance"])["accepted"])
        result = {"sequence": int(row["sequence"]), "time_ms": row["time_ms"], "started": True,
                  "dt_ms": odo["dt"]*1000, "dt_source": odo["dt_source"], "reset_bridge": odo["reset"],
                  "raw.east_mm": odo["raw_xy_m"][0]*1000, "raw.north_mm": odo["raw_xy_m"][1]*1000,
                  "raw.yaw_rad": odo["raw_yaw"], "initial_bias_deg_s": math.degrees(self.bias), "bias_ready": self.bias_ready,
                  "global.east_mm": self.global_ekf.state[0]*1000, "global.north_mm": self.global_ekf.state[1]*1000,
                  "global.yaw_rad": self.global_ekf.state[2], "global.bias_deg_s": math.degrees(self.global_ekf.state[3]),
                  "bias_only.east_mm": self.bias_path.state[0]*1000, "bias_only.north_mm": self.bias_path.state[1]*1000,
                  "segment": self.segment, "local_active": active, "wall_mask": mask, "local_updated": accepted,
                  "local_updates_in_segment": self.local.updates, "local_observed": active and self.local.updates > 0, "heldout": heldout,
                  "local.e_mm": self.local.state[0]*1000 if active else None,
                  "local.psi_deg": math.degrees(self.local.state[1]) if active else None,
                  "local.prior_e_mm": prior_local[0]*1000 if active else None,
                  "local.std_e_mm": math.sqrt(max(0, self.local.covariance[0, 0]))*1000 if active else None,
                  "local.std_psi_deg": math.degrees(math.sqrt(max(0, self.local.covariance[1, 1]))) if active else None,
                  "lateral_only.e_mm": self.lateral_only.value*1000 if active else None,
                  "left_mm": row["sen_l.distance"], "right_mm": row["sen_r.distance"]}
        return result


def score_block(block, observation, config):
    first, last = block[0], block[-1]
    axis = observation["axis"]
    left_axis = np.array([-math.cos(axis), -math.sin(axis)])
    predicted = {"raw": float((last["raw_xy_m"]-first["raw_xy_m"])@left_axis),
                 "bias_only": float((last["bias_path"][:2]-first["bias_path"][:2])@left_axis),
                 "global_ekf": float((last["global"][:2]-first["global"][:2])@left_axis),
                 "lateral_only": last["lateral_only"]-first["lateral_only"],
                 "local_ekf": last["local"][0]-first["local"][0]}
    errors = {}
    for name, change in predicted.items():
        if name in ("raw", "bias_only", "global_ekf") and not observation["axis_valid"]:
            continue
        errors[name] = [(sign*change*1000-(last["sen_"+side+".distance"]-first["sen_"+side+".distance"]))
                        for side, sign in (("l", -1), ("r", 1)) if side in observation["sides"]]
    mean_local = float(np.mean([row["local"][1] for row in block]))
    local_angle_error = math.degrees(wrap_angle(mean_local-observation["mean_local_heading"]))
    mask = sum(1 if side == "l" else 2 for side in observation["sides"])
    measured_e, _ = lateral_observation(last, mask, config)
    return {"first_sequence": int(first["sequence"]), "last_sequence": int(last["sequence"]),
            "sides": "+".join(observation["sides"]), "global_axis_valid": bool(observation["axis_valid"]),
            "range_change_errors_mm": errors, "local_heading_error_deg": local_angle_error,
            "local_lateral_error_mm": (last["local"][0]-measured_e)*1000,
            "lateral_only_error_mm": (last["lateral_only"]-measured_e)*1000}


class CsvSamples:
    """Read complete CSV records incrementally, optionally waiting for appended rows."""

    def __init__(self, path, follow=False, max_rows=None, required_columns=None):
        self.path, self.follow, self.max_rows = Path(path), follow, max_rows
        self.required = tuple(required_columns) if required_columns is not None else REQUIRED
        self.rows = 0
        self.digest = hashlib.sha256()

    def __iter__(self):
        with self.path.open(encoding="utf-8", newline="") as file:
            buffer, header = "", None
            while self.max_rows is None or self.rows < self.max_rows:
                part = file.readline()
                if not part:
                    if self.follow:
                        time.sleep(.05)
                        continue
                    if buffer:
                        raise ValueError("Incomplete final CSV line; use --follow for a growing file")
                    break
                buffer += part
                if not buffer.endswith("\n"):
                    continue
                self.digest.update(buffer.encode("utf-8"))
                fields = next(csv.reader([buffer]))
                buffer = ""
                if header is None:
                    header = fields
                    if not set(self.required).issubset(header):
                        raise ValueError("Missing required BLE CSV columns")
                    continue
                if len(fields) != len(header):
                    raise ValueError("Malformed CSV row")
                row = dict(zip(header, fields))
                self.rows += 1
                yield {key: float(row[key]) for key in self.required}


def aggregate(records):
    variants = ("raw", "bias_only", "global_ekf", "lateral_only", "local_ekf")
    result = {}
    for name in variants:
        errors = [error for row in records for error in row["range_change_errors_mm"].get(name, [])]
        result[name] = {"range_change_rmse_mm": float(np.sqrt(np.mean(np.square(errors)))) if errors else None,
                        "side_differences": len(errors)}
    for name, key in (("local_ekf", "local_lateral_error_mm"), ("lateral_only", "lateral_only_error_mm")):
        result[name]["lateral_rmse_mm"] = float(
            np.sqrt(np.mean([row[key]**2 for row in records]))) if records else None
    result["local_ekf"]["heading_rmse_deg"] = float(
        np.sqrt(np.mean([row["local_heading_error_deg"]**2 for row in records]))) if records else None
    return result


def plot_results(output, rows):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    import pandas as pd
    data = pd.DataFrame([row for row in rows if row["started"]])
    if data.empty:
        return
    fonts = {font.name for font in font_manager.fontManager.ttflist}
    plt.rcParams["font.family"] = next(
        (name for name in ("Yu Gothic", "Meiryo", "MS Gothic") if name in fonts), "DejaVu Sans")
    plt.rcParams["axes.unicode_minus"] = False
    t = (data.time_ms-data.time_ms.iloc[0])/1000
    fig, axes = plt.subplots(1, 2, figsize=(13, 7), constrained_layout=True)
    for name, label, color in [("raw", "因果的な局所座標復元", "#888888"), ("bias_only", "走行前の零点補正のみ", "#2366b4"), ("global", "逐次4状態EKF", "#bd5b16")]:
        axes[0].plot(data[name+".east_mm"]/90, data[name+".north_mm"]/90, label=label, color=color, lw=1.5)
    axes[0].set(xlabel="東 [区画 / 90 mm]", ylabel="北 [区画 / 90 mm]", title="1行ずつ更新した全体位置推定")
    axes[0].set_aspect("equal")
    axes[0].grid(alpha=.25)
    axes[0].legend(fontsize=8)
    axes[1].plot(t, data["global.bias_deg_s"], color="#bd5b16", label="4状態EKFのバイアス")
    axes[1].axhline(data.initial_bias_deg_s.iloc[0], color="#2366b4", ls="--", label="走行前の静止時中央値")
    axes[1].set(xlabel="MCUログ経過時間 [s]", ylabel="ジャイロバイアス [°/s]", title="校正値の取り込みと逐次推定")
    axes[1].grid(alpha=.25)
    axes[1].legend(fontsize=8)
    fig.suptitle("未来の補間を使わない逐次EKF · 元のオフライン復元とは前処理が異なる\n外部位置の正解値は未取得。保存ログの因果的な再生であり、実機への組み込みは未実施。", fontsize=12)
    fig.savefig(output/"streaming_global.png", dpi=165, bbox_inches="tight")
    plt.close(fig)
    fig, axes = plt.subplots(2, 1, figsize=(13, 8), sharex=True, constrained_layout=True)
    active = data.local_active.astype(bool)
    both = active & data.left_mm.between(30, 60) & data.right_mm.between(30, 60)
    axes[0].scatter(t[both], ((data.right_mm-data.left_mm)/2)[both],
                    s=3, color="#999999", alpha=.45, label="左右距離差の参考値")
    axes[0].plot(t, data["lateral_only.e_mm"], color="#2366b4", lw=1, label="横ずれだけの1状態KF")
    axes[0].plot(t, data["local.e_mm"], color="#bd5b16", lw=1.2, label="横ずれ＋角度の2状態EKF")
    axes[0].set(ylabel="横ずれ [mm・左正]", title="現在の通路中心に対する偏差（直進候補のみ）")
    axes[1].plot(t, data["local.psi_deg"], color="#bd5b16", lw=1.2, label="通路方向に対する角度")
    axes[1].set(xlabel="MCUログ経過時間 [s]", ylabel="姿勢偏差 [°・左正]", title="旋回・停止を挟むと再初期化。区画の局所座標リセットだけでは初期化しない。")
    for axis in axes:
        axis.axhline(0, color="#555555", lw=.7)
        axis.grid(alpha=.25)
        axis.legend(fontsize=9)
    fig.suptitle("直進通路内の2状態EKF · x=[横ずれ,姿勢偏差]\n壁と平行な通路・距離校正を仮定。前後位置、旋回中の横位置、実際の中心走行の改善は評価しない。", fontsize=12)
    fig.savefig(output/"streaming_local.png", dpi=165, bbox_inches="tight")
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--settings", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--follow", action="store_true",
                        help="Wait for complete appended CSV rows; Ctrl+C saves the processed prefix")
    parser.add_argument("--max-rows", type=int)
    args = parser.parse_args(argv)
    if args.max_rows is not None and args.max_rows <= 0:
        parser.error("--max-rows must be positive")
    args.output.mkdir(parents=True, exist_ok=True)
    estimator = StreamingWallEstimator()
    reader = CsvSamples(args.log, args.follow, args.max_rows)
    outputs = []
    stopped = False
    with (args.output/"streaming_states.csv").open("w", newline="", encoding="utf-8") as file:
        writer = None
        try:
            for sample in reader:
                result = estimator.update(sample)
                outputs.append(result)
                if result["started"]:
                    if writer is None:
                        writer = csv.DictWriter(file, fieldnames=list(result))
                        writer.writeheader()
                    writer.writerow(result)
        except KeyboardInterrupt:
            stopped = True
    validations = []
    for fold in (range(3) if reader.rows else ()):
        trial = StreamingWallEstimator(holdout_fold=fold)
        validation_reader = CsvSamples(args.log, max_rows=reader.rows)
        for sample in validation_reader:
            trial.update(sample)
            if trial.last_validation:
                validations.append({"fold": fold, **trial.last_validation})
        if validation_reader.digest.hexdigest() != reader.digest.hexdigest():
            raise RuntimeError("Processed CSV prefix changed during replay; keep the input prefix immutable")
    metadata = json.loads(args.settings.read_text(encoding="utf-8")) if args.settings else {}
    rows = [r for r in outputs if r["started"]]
    covariances = [estimator.local.covariance]
    if estimator.global_ekf is not None:
        covariances.append(estimator.global_ekf.covariance)
    assert all(np.isfinite(p).all() and np.linalg.eigvalsh(p).min() >= -1e-10 for p in covariances)
    report = {"input": str(args.log.resolve()), "processed_rows": reader.rows, "motion_rows": len(rows),
              "processed_csv_prefix_sha256": reader.digest.hexdigest(), "follow": args.follow, "interrupted": stopped,
              "machine": metadata.get("firmware", {}).get("mouse", "not recorded"), "settings": metadata,
              "firmware_build_id": "not recorded", "quiet_samples": len(estimator.quiet),
              "initial_bias_deg_s": math.degrees(estimator.bias), "initial_bias_ready": estimator.bias_ready,
              "local_state_dimension": 2, "global_state_dimension": 4,
              "local_config": asdict(estimator.local_config), "global_config": asdict(estimator.global_config),
              "global_updates": estimator.updates_global, "local_updates": sum(r["local_updated"] for r in rows),
              "local_segments": estimator.segment+1, "validation_blocks": len(validations),
              "three_fold_disjoint_block_scores": aggregate(validations),
              "validation_definition": "18mm disjoint blocks assigned before seeing range values; every sample in held-out blocks excluded from both estimators' wall updates; no shared boundary samples",
              "causality": "update(sample) uses no future samples; trailing21 median, past500 quiet samples, max128 sample spatial block, two per-side gates",
              "memory_bound_samples": {"odometry_dt": 21, "quiet_calibration": 500, "wall_block": 128},
              "limitations": ["Local coordinates mean passage-relative lateral offset and heading, not the origin of the instantaneous robot frame.",
                              "Straight-passage model; local state is reset on detected turn/stop, not at each logged local-coordinate reset. No motion IDs were captured.",
                              "Converted side ranges are approximated as transverse distances: L=45mm-e, R=45mm+e; mounting geometry/calibration and lateral slip are unverified.",
                              "Heading is inferred through motion and lateral observations; it is weakly observable without travel or stable walls.",
                              "No external pose ground truth. Held-out wall consistency does not establish physical trajectory or centre-tracking improvement.",
                              "Causal reset bridging/timing still approximate missing firmware timing and reset flags. New results are not directly comparable to former offline-preprocessed scores.",
                              "Live-follow reads an existing CSV writer; no BLE connection or firmware deployment is performed.",
                              "Runtime estimator has bounded memory; CLI retains output history only for plotting and validation replay."],
              "implementation_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              "global_implementation_sha256": hashlib.sha256(Path(__file__).with_name("wall_heading_ekf.py").read_bytes()).hexdigest(),
              "reference": "https://atsushisakai.github.io/PythonRobotics/modules/2_localization/extended_kalman_filter_localization_files/extended_kalman_filter_localization.html"}
    (args.output/"report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    (args.output/"validation_blocks.json").write_text(json.dumps(validations,
                                                                 ensure_ascii=False, indent=2), encoding="utf-8")
    plot_results(args.output, outputs)
    print(json.dumps({key: report[key] for key in ("machine", "processed_rows", "initial_bias_deg_s",
                                                   "validation_blocks", "three_fold_disjoint_block_scores")}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
