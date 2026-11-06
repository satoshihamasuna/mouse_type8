"""Sample-at-a-time two-state EKF driven by both wheel encoders, IMU and walls.

State: passage-relative left-positive lateral offset e_m and heading psi_rad.
Wheel and IMU yaw rates are fused as uncertain prediction inputs; gyro bias is
calibrated before motion, outside the two estimated states. No logged ego pose,
ego velocity, global EKF, future interpolation or route is required.
"""
from __future__ import annotations

import argparse
import csv
from collections import deque
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import pandas as pd

from streaming_wall_ekf import (CsvSamples, LocalConfig, LateralHeadingEkf,
                                SpatialBlocks, WallRangeGate, lateral_observation, straight_candidate)
from wall_heading_ekf import wall_heading_measurement, wrap_angle
from compare_ekf_paths import LocalPosePath


RIGHT = "Encoder_GetProperty_Right().sp_pulse"
LEFT = "Encoder_GetProperty_Left().sp_pulse"
REQUIRED = ("sequence", "time_ms", "period_ms", RIGHT, LEFT, "ego.rad_velo", "ideal.velo",
            "ideal.rad_velo", "ideal.radian", "sen_l.distance", "sen_r.distance", "sen_fl.distance", "sen_fr.distance")
MODES = ("gyro_only", "fused", "encoder_only")


@dataclass(frozen=True)
class SensorConfig:
    mm_per_tick: float = 15.02*math.pi/172
    tread_m: float = .028
    encoder_tick_s: float = .001
    trailing_samples: int = 5
    gyro_rate_std_min_rad_s: float = .03
    wheel_slip_std_m_s: float = .03
    disagreement_sigma: float = 3.

    def __post_init__(self):
        if any(not math.isfinite(value) or value <= 0 for value in asdict(self).values()):
            raise ValueError("Invalid sensor configuration")
        if not isinstance(self.trailing_samples, int):
            raise ValueError("trailing_samples must be an integer")


def decode_wheel_rates(right_ticks, left_ticks, config):
    """CSV values are one control tick, not the 5/6ms delivery interval.

    Firmware right count is positive forward and left count is negative forward.
    This adapter cannot reconstruct pulses absent from the decimated CSV.
    """
    scale = config.mm_per_tick/1000/config.encoder_tick_s
    return right_ticks*scale, -left_ticks*scale


def fused_controls(v_right, v_left, gyro, wheel_variances, gyro_variance, tread_m,
                   mode="fused", disagreement_sigma=3.):
    if mode not in MODES or not np.isfinite([v_right, v_left, gyro, *wheel_variances, gyro_variance, tread_m]).all():
        raise ValueError("Invalid sensor inputs or fusion mode")
    if min(*wheel_variances, gyro_variance, tread_m) <= 0:
        raise ValueError("Nonpositive sensor variance or tread")
    vr, vl = wheel_variances
    velocity = .5*(v_right+v_left)
    encoder_yaw = (v_right-v_left)/tread_m
    velocity_var = .25*(vr+vl)
    encoder_yaw_var = (vr+vl)/tread_m**2
    # Conservative disagreement rule treats wheel slip as more likely than a
    # sudden IMU error; this assumption is explicit, not extra EKF states.
    disagreement = abs(encoder_yaw-gyro)/math.sqrt(encoder_yaw_var+gyro_variance)
    inflation = max(1., (disagreement/disagreement_sigma)**2) if mode == "fused" else 1.
    encoder_yaw_var *= inflation
    encoder_weight = gyro_variance/(encoder_yaw_var+gyro_variance)
    if mode == "gyro_only":
        encoder_weight = 0.
    elif mode == "encoder_only":
        encoder_weight = 1.
    omega = encoder_weight*encoder_yaw+(1-encoder_weight)*gyro
    omega_var = encoder_weight**2*encoder_yaw_var+(1-encoder_weight)**2*gyro_variance
    velocity_yaw_cov = encoder_weight*(vr-vl)/(2*tread_m)
    covariance = np.array([[velocity_var, velocity_yaw_cov], [velocity_yaw_cov, omega_var]])
    return {"velocity": velocity, "omega": omega, "encoder_yaw": encoder_yaw,
            "encoder_weight": encoder_weight, "encoder_variance_inflation": inflation,
            "control_covariance": covariance}


def sensor_motion(state, velocity, omega, dt):
    mid = state[1]+.5*omega*dt
    state_next = np.asarray(state, dtype=float)+[velocity*dt*math.sin(mid), omega*dt]
    f = np.array([[1., velocity*dt*math.cos(mid)], [0., 1.]])
    g = np.array([[dt*math.sin(mid), .5*velocity*dt**2*math.cos(mid)], [0., dt]])
    return state_next, f, g


class EncoderWallImuEkf(LateralHeadingEkf):
    """Exactly two states; input covariance includes wheel/IMU rate uncertainty."""
    def predict_sensors(self, velocity, omega, dt, control_covariance):
        covariance = np.asarray(control_covariance, dtype=float)
        if covariance.shape != (2, 2) or not np.isfinite(covariance).all() or np.linalg.eigvalsh(covariance).min() < -1e-12:
            raise ValueError("Invalid control covariance")
        if not np.isfinite([velocity, omega, dt]).all() or not 0 < dt < .1:
            raise ValueError("Invalid velocity, yaw rate or time units")
        self.state, f, g = sensor_motion(self.state, velocity, omega, dt)
        q = np.diag([(self.config.lateral_walk_mm_sqrt_s/1000)**2*dt,
                     math.radians(self.config.gyro_walk_deg_sqrt_s)**2*dt])
        self.covariance = f@self.covariance@f.T+g@covariance@g.T+q
        self.covariance = .5*(self.covariance+self.covariance.T)


def local_block_observation(block):
    if any(min(row["sen_fl.distance"], row["sen_fr.distance"]) <= 112 or abs(row["ego.rad_velo"]) >= 1.5 for row in block):
        return None
    travel = np.array([row["spatial_m"] for row in block])
    slopes, sides = [], []
    for side in ("l", "r"):
        values = np.array([row["sen_"+side+".distance"] for row in block])
        if not ((values > 30)&(values < 60)).all() or max(abs(np.diff(values))) > 1.5:
            continue
        slope, residual, _ = wall_heading_measurement(travel, values, "left" if side == "l" else "right")
        if abs(slope) <= math.sin(math.radians(15)) and residual <= .00075:
            slopes.append(slope); sides.append(side)
    if not sides or (len(slopes) == 2 and abs(slopes[0]-slopes[1]) > .12):
        return None
    return sides, math.asin(float(np.mean(slopes)))


class StreamingEncoderFusion:
    """Direct measurement adapter; bounded histories, no dependency on logged pose."""
    def __init__(self, mode="fused", holdout_fold=None, sensor_config=None, local_config=None):
        if mode not in MODES:
            raise ValueError("Unknown fusion mode")
        self.mode = mode
        self.sensors = sensor_config or SensorConfig()
        self.local_config = local_config or LocalConfig()
        self.ekf = EncoderWallImuEkf(self.local_config)
        self.wheels = deque(maxlen=self.sensors.trailing_samples)
        self.quiet = deque(maxlen=500)
        self.blocks = SpatialBlocks(holdout_fold)
        self.gate = WallRangeGate(self.local_config)
        self.previous = None
        self.started = False
        self.bias = 0.
        self.bias_ready = False
        self.gyro_variance = self.sensors.gyro_rate_std_min_rad_s**2
        self.segment = -1
        self.active = False
        self.previous_heldout = False
        self.base_xy = np.zeros(2)
        self.base_yaw = 0.
        self.path = LocalPosePath()
        self.last_validation = None
        self.minimum_eigenvalue = math.inf

    def update(self, sample):
        row = {key: float(sample[key]) for key in REQUIRED}
        if not np.isfinite(list(row.values())).all():
            raise ValueError("Nonfinite required sensor value")
        dt = row["period_ms"]/1000 if self.previous is None else (row["time_ms"]-self.previous["time_ms"])/1000
        if not 0 < dt < .1 or (self.previous is not None and row["sequence"] != self.previous["sequence"]+1):
            raise ValueError("Invalid timestamp interval or discontinuous sequence")
        right, left = decode_wheel_rates(row[RIGHT], row[LEFT], self.sensors)
        self.wheels.append((right, left))
        filtered_right, filtered_left = np.mean(self.wheels, axis=0)
        previous = self.previous
        self.previous = row.copy()
        self.last_validation = None
        moving = abs(row["ideal.velo"]) > .01 or abs(row["ideal.rad_velo"]) > .05
        just_started = not self.started and moving
        if not self.started and not moving:
            if max(abs(right), abs(left)) < .02 and abs(row["ego.rad_velo"]) < .035:
                self.quiet.append(row["ego.rad_velo"])
            return {"sequence": int(row["sequence"]), "time_ms": row["time_ms"], "started": False}
        if just_started:
            self.started = True
            self.bias_ready = len(self.quiet) >= 30
            if self.bias_ready:
                self.bias = float(np.median(self.quiet))
                spread = 1.4826*float(np.median(abs(np.array(self.quiet)-self.bias)))
                self.gyro_variance = max(spread, self.sensors.gyro_rate_std_min_rad_s)**2
        gyro = row["ego.rad_velo"]-self.bias
        if previous is not None and not just_started:
            gyro = .5*(row["ego.rad_velo"]+previous["ego.rad_velo"])-self.bias
        # A tick count is a difference of quantized endpoint angles: 1/6 tick^2.
        # Keep a conservative raw variance even for overlapping trailing means.
        wheel_variance = (self.sensors.mm_per_tick/1000/self.sensors.encoder_tick_s)**2/6+self.sensors.wheel_slip_std_m_s**2
        control = fused_controls(float(filtered_right), float(filtered_left), gyro, (wheel_variance, wheel_variance),
                                 self.gyro_variance, self.sensors.tread_m, self.mode, self.sensors.disagreement_sigma)
        velocity, omega = control["velocity"], control["omega"]
        ds = 0. if just_started else velocity*dt
        if not just_started:
            mid = self.base_yaw+.5*omega*dt
            self.base_xy += [-ds*math.sin(mid), ds*math.cos(mid)]
            self.base_yaw += omega*dt
        active = straight_candidate(row)
        if active and not self.active:
            self.ekf.reset(); self.gate.reset(); self.segment += 1
        elif active:
            self.ekf.predict_sensors(velocity, omega, dt, control["control_covariance"])
        if not active:
            self.gate.reset()
        self.active = active
        heldout, new_block = self.blocks.begin(active)
        if new_block and (heldout or self.previous_heldout):
            self.gate.reset()
        self.previous_heldout = heldout
        prior = self.ekf.state.copy()
        completed = self.blocks.append({**row, "travel_m": max(0., ds), "local": prior}) if active else None
        mask, updated = 0, False
        if active and not heldout:
            mask = self.gate.update(row, ds)
            measurement = lateral_observation(row, mask, self.local_config)
            if measurement:
                updated = self.ekf.observe_lateral(*measurement)
        if completed:
            observation = local_block_observation(completed)
            if observation and heldout:
                sides, angle = observation
                first, last = completed[0], completed[-1]
                change = last["local"][0]-first["local"][0]
                errors = [sign*change*1000-(last["sen_"+side+".distance"]-first["sen_"+side+".distance"])
                          for side, sign in (("l", -1), ("r", 1)) if side in sides]
                side_mask = sum(1 if side == "l" else 2 for side in sides)
                measured_e, _ = lateral_observation(last, side_mask, self.local_config)
                self.last_validation = {"first_sequence": int(first["sequence"]), "last_sequence": int(last["sequence"]),
                    "range_errors_mm": errors, "lateral_error_mm": (last["local"][0]-measured_e)*1000,
                    "heading_error_deg": math.degrees(wrap_angle(float(np.mean([r["local"][1] for r in completed]))-angle))}
        minimum = float(np.linalg.eigvalsh(self.ekf.covariance).min())
        self.minimum_eigenvalue = min(self.minimum_eigenvalue, minimum)
        if minimum < -1e-10 or not np.isfinite(self.ekf.state).all():
            raise RuntimeError("Invalid EKF state/covariance")
        path_input = {"sequence": int(row["sequence"]), "time_ms": row["time_ms"], "dt_ms": dt*1000,
            "raw.east_mm": self.base_xy[0]*1000, "raw.north_mm": self.base_xy[1]*1000, "raw.yaw_rad": self.base_yaw,
            "initial_bias_deg_s": 0.,  # Base odometry has ALREADY removed the gyro bias.
            "segment": self.segment, "local_active": active, "local_observed": active and self.ekf.updates > 0,
            "local.e_mm": self.ekf.state[0]*1000 if active else None,
            "local.psi_deg": math.degrees(self.ekf.state[1]) if active else None,
            "local.std_psi_deg": math.degrees(math.sqrt(max(0., self.ekf.covariance[1, 1]))) if active else None}
        path_result = self.path.update(path_input)
        return {**path_input, "started": True, "mode": self.mode, "gyro_bias_deg_s": math.degrees(self.bias),
                "bias_ready": self.bias_ready, "right_velocity_m_s": float(filtered_right),
                "left_velocity_m_s": float(filtered_left), "velocity_m_s": velocity,
                "encoder_yaw_rad_s": control["encoder_yaw"], "gyro_yaw_rad_s": gyro, "fused_yaw_rad_s": omega,
                "encoder_weight": control["encoder_weight"], "encoder_variance_inflation": control["encoder_variance_inflation"],
                "wall_mask": mask, "local_updated": updated, "heldout": heldout,
                "path.east_mm": path_result["east_mm"], "path.north_mm": path_result["north_mm"],
                "local.std_e_mm": math.sqrt(max(0., self.ekf.covariance[0, 0]))*1000 if active else None}


def score(records):
    errors = [value for row in records for value in row["range_errors_mm"]]
    return {"blocks": len(records), "side_differences": len(errors),
            "range_change_rmse_mm": float(np.sqrt(np.mean(np.square(errors)))) if errors else None,
            "lateral_rmse_mm": float(np.sqrt(np.mean([r["lateral_error_mm"]**2 for r in records]))) if records else None,
            "heading_rmse_deg": float(np.sqrt(np.mean([r["heading_error_deg"]**2 for r in records]))) if records else None}


def plot(output, frame, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    fonts = {font.name for font in font_manager.fontManager.ttflist}
    plt.rcParams["font.family"] = next((name for name in ("Yu Gothic", "Meiryo", "MS Gothic") if name in fonts), "DejaVu Sans")
    plt.rcParams["axes.unicode_minus"] = False
    labels = {"gyro_only": "左右ENC速度＋IMU旋回＋壁", "fused": "左右ENC速度・旋回＋IMU＋壁", "encoder_only": "旋回予測はENCのみ（壁採用条件は共通）"}
    colors = {"gyro_only": "#2366b4", "fused": "#298452", "encoder_only": "#888888"}
    t = (frame.time_ms-frame.time_ms.iloc[0])/1000
    fig, axes = plt.subplots(4, 1, figsize=(13, 11), sharex=True, constrained_layout=True)
    for mode in ("gyro_only", "fused"):
        axes[0].plot(t, frame[mode+".e_mm"], label=labels[mode], color=colors[mode], ls="--" if mode == "gyro_only" else "-")
        axes[1].plot(t, frame[mode+".psi_deg"], label=labels[mode], color=colors[mode], ls="--" if mode == "gyro_only" else "-")
    axes[0].set(ylabel="横偏差 [mm・左正]", title="同じ左右ENC前進速度・同じ壁観測で、旋回入力を比較")
    axes[1].set_ylabel("姿勢偏差 [°・左正]")
    axes[2].plot(t, frame.encoder_yaw_rad_s, color="#888888", lw=.7, alpha=.5, label="左右ENC差による角速度")
    axes[2].plot(t, frame.gyro_yaw_rad_s, color="#bd5b16", lw=.8, label="IMU角速度（零点補正済み）")
    axes[2].set_ylabel("角速度 [rad/s]")
    axes[3].plot(t, frame.encoder_weight*100, color="#298452", label="ENC角速度の統合比率")
    axes[3].set(xlabel="MCUログ経過時間 [s]", ylabel="ENCの比率 [%]")
    for axis in axes:
        axis.grid(alpha=.25); axis.legend(fontsize=8, loc="upper right")
    fig.suptitle(title+"\n全て逐次2状態EKF。1msのENCスナップショットを5ms間隔で配信したログのため、欠けたパルスは復元できない。", fontsize=12)
    fig.savefig(output/"sensor_fusion_states.png", dpi=160, bbox_inches="tight"); plt.close(fig)
    fig, axes = plt.subplots(1, 2, figsize=(13, 7), constrained_layout=True)
    for axis, modes in zip(axes, [("gyro_only", "fused"), ("encoder_only",)]):
        for mode in modes:
            axis.plot(frame[mode+".east_mm"], frame[mode+".north_mm"], color=colors[mode], label=labels[mode],
                      ls="--" if mode == "gyro_only" else "-", lw=1.5)
            axis.scatter(frame[mode+".east_mm"].iloc[-1], frame[mode+".north_mm"].iloc[-1], marker="x", color=colors[mode])
        axis.scatter(0, 0, facecolor="white", edgecolor="#222222", s=45)
        axis.set(xlabel="東 [mm]", ylabel="北 [mm]"); axis.set_aspect("equal"); axis.grid(alpha=.25); axis.legend(fontsize=8)
    axes[0].set_title("IMUを使う2条件 · ほぼ重なる場合はENC旋回の寄与が小さい")
    axes[1].set_title("比較用：旋回予測がENCのみの条件（縮尺は別）")
    fig.suptitle(title+" · 外側の積分で描いた推定経路\n前進・旋回はENC/IMU、直進のe・ψ補正を接続。実位置精度の評価ではない。", fontsize=12)
    fig.savefig(output/"sensor_fusion_paths.png", dpi=160, bbox_inches="tight"); plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--settings", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--follow", action="store_true")
    parser.add_argument("--max-rows", type=int)
    args = parser.parse_args(argv)
    metadata = json.loads(args.settings.read_text(encoding="utf-8"))
    if metadata.get("firmware", {}).get("mouse") != "A":
        raise ValueError("This replay uses verified Mouse A encoder/sign settings only")
    if args.max_rows is not None and args.max_rows <= 0:
        parser.error("--max-rows must be positive")
    args.output.mkdir(parents=True, exist_ok=True)
    estimators = {mode: StreamingEncoderFusion(mode) for mode in MODES}
    reader = CsvSamples(args.log, args.follow, args.max_rows, required_columns=REQUIRED)
    outputs = []
    with (args.output/"fusion_states.csv").open("w", newline="", encoding="utf-8") as file:
        writer = None
        try:
            for sample in reader:
                results = {mode: estimator.update(sample) for mode, estimator in estimators.items()}
                if not results["fused"]["started"]:
                    continue
                fused = results["fused"]
                output = {key: fused[key] for key in ("sequence", "time_ms", "velocity_m_s", "right_velocity_m_s", "left_velocity_m_s",
                           "encoder_yaw_rad_s", "gyro_yaw_rad_s", "fused_yaw_rad_s", "encoder_weight", "gyro_bias_deg_s")}
                for mode, result in results.items():
                    output.update({mode+".e_mm": result["local.e_mm"], mode+".psi_deg": result["local.psi_deg"],
                        mode+".east_mm": result["path.east_mm"], mode+".north_mm": result["path.north_mm"],
                        mode+".observed": result["local_observed"], mode+".updated": result["local_updated"],
                        mode+".std_e_mm": result["local.std_e_mm"], mode+".std_psi_deg": result["local.std_psi_deg"]})
                if writer is None:
                    writer = csv.DictWriter(file, fieldnames=list(output)); writer.writeheader()
                writer.writerow(output)
                outputs.append(output)
        except KeyboardInterrupt:
            pass
    validation = {mode: [] for mode in MODES}
    for fold in (range(3) if reader.rows else ()):
        trial = {mode: StreamingEncoderFusion(mode, fold) for mode in MODES}
        prefix = CsvSamples(args.log, max_rows=reader.rows, required_columns=REQUIRED)
        for sample in prefix:
            for mode, estimator in trial.items():
                estimator.update(sample)
                if estimator.last_validation:
                    validation[mode].append({"fold": fold, **estimator.last_validation})
        if prefix.digest.hexdigest() != reader.digest.hexdigest():
            raise RuntimeError("Processed input prefix changed during validation")
    keys = lambda values: [(r["fold"], r["first_sequence"], r["last_sequence"]) for r in values]
    assert keys(validation["fused"]) == keys(validation["gyro_only"]) == keys(validation["encoder_only"])
    frame = pd.DataFrame(outputs)
    root = Path(__file__).resolve().parents[2]
    report = {"input": str(args.log.resolve()), "machine": "A", "settings": metadata,
        "processed_rows": reader.rows, "motion_rows": len(outputs), "processed_csv_prefix_sha256": reader.digest.hexdigest(),
        "state_dimension": 2, "states": ["passage_lateral_offset_m", "passage_heading_rad"],
        "sensor_config": asdict(estimators["fused"].sensors), "local_config": asdict(estimators["fused"].local_config),
        "gyro_bias_deg_s": math.degrees(estimators["fused"].bias), "bias_ready": estimators["fused"].bias_ready,
        "encoder_weight_mean": float(frame.encoder_weight.mean()) if len(frame) else None,
        "scores": {mode: score(records) for mode, records in validation.items()},
        "validation_interval_keys_identical": True, "encoder_snapshot_duration_ms": 1.,
        "time_basis": "MCU timestamp differences; no reconstruction from ego.length, turn_x/y or local angles",
        "limitations": ["Decimated one-control-tick encoder pulses are velocity snapshots, not cumulative travel between CSV rows; missing pulses cannot be recovered.",
            "Logging/control clocks may differ; all ablations use the same timestamp clock and sampled-wheel adapter. Do not directly compare former reconstructed-odometry endpoint numbers.",
            "Input-rate fusion assumes independent wheel/IMU noise; input covariance is propagated into the 2x2 EKF covariance. Sliding means retain a conservative raw-noise floor.",
            "Gyro bias is calibrated before motion, not an additional EKF state. Slip and bias changes are not independently estimated.",
            "Mode names refer to yaw prediction inputs. All modes retain the same IMU-based wall-quality gate to keep evaluation intervals identical.",
            "Local state is suspended through turns/stops; global path is external integration with the previously documented cardinal-axis assumptions.",
            "Wall geometry/calibration and true position are unverified. Held-out wall prediction error is not physical centre-tracking accuracy.",
            "Host replay/follow only; no firmware changes or live BLE connection."],
        "implementation_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "source_provenance": {str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest() for path in
            (root/"Params/mouse_config.h", root/"Peripheral/Src/encoder.c", root/"Module/Src/interrupt.cpp", root/"Module/Src/log_data.cpp")}}
    (args.output/"report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    (args.output/"validation_blocks.json").write_text(json.dumps(validation, ensure_ascii=False, indent=2), encoding="utf-8")
    if len(frame):
        plot(args.output, frame, "機体A · 左右エンコーダ＋壁＋IMUの逐次2状態EKF")
    print(json.dumps({key: report[key] for key in ("processed_rows", "gyro_bias_deg_s", "encoder_weight_mean", "scores")}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
