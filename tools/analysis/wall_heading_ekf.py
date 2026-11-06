"""Offline wall-heading EKF experiment for recorded BLE exploration data.

State is [east_m, north_m, left_positive_yaw_rad, residual_gyro_bias_rad_s].
Wall observations constrain heading, not absolute position. No route or return
coordinate is passed into prediction, observation extraction, or replay.
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


def wrap_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


@dataclass(frozen=True)
class Config:
    block_distance_m: float = .018
    sensor_noise_floor_mm: float = .25
    heading_noise_floor_deg: float = 1.5
    gyro_random_walk_deg: float = .3
    bias_random_walk_deg_s: float = .02
    reset_heading_noise_deg: float = .3
    initial_heading_std_deg: float = 2.
    initial_bias_std_deg_s: float = .15
    innovation_limit_deg: float = 12.
    nis_limit: float = 9.
    holdout_stride: int = 3

    def __post_init__(self):
        for key, value in asdict(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError("Invalid EKF configuration: " + key)
        if not isinstance(self.holdout_stride, int) or self.holdout_stride < 2:
            raise ValueError("holdout_stride must be an integer >= 2")


def motion(state, raw_step_m, raw_heading_before, raw_heading_delta, dt):
    """Rotate the observed displacement by estimated yaw error; retain its size."""
    state = np.asarray(state, dtype=float)
    angle = state[2] - raw_heading_before - .5 * state[3] * dt
    c, s = math.cos(angle), math.sin(angle)
    de, dn = raw_step_m
    east = c * de - s * dn
    north = s * de + c * dn
    predicted = state.copy()
    predicted[:2] += (east, north)
    predicted[2] += raw_heading_delta - state[3] * dt
    jacobian = np.eye(4)
    jacobian[0, 2], jacobian[1, 2] = -north, east
    jacobian[0, 3], jacobian[1, 3] = .5 * dt * north, -.5 * dt * east
    jacobian[2, 3] = -dt
    return predicted, jacobian


class WallHeadingEkf:
    def __init__(self, bias_rad_s=0., config=None):
        self.config = config or Config()
        self.state = np.array([0., 0., 0., bias_rad_s])
        self.covariance = np.diag([.002 ** 2, .002 ** 2,
            math.radians(self.config.initial_heading_std_deg) ** 2,
            math.radians(self.config.initial_bias_std_deg_s) ** 2])

    def predict(self, step_m, heading_before, heading_delta, dt, reset=False):
        if not np.isfinite([*step_m, heading_before, heading_delta, dt]).all() or dt <= 0:
            raise ValueError("Invalid odometry increment or control interval")
        self.state, f = motion(self.state, step_m, heading_before, heading_delta, dt)
        sigma_s = .02 * float(np.linalg.norm(step_m)) + .00002
        sigma_yaw2 = math.radians(self.config.gyro_random_walk_deg) ** 2 * dt
        if reset:
            sigma_yaw2 += math.radians(self.config.reset_heading_noise_deg) ** 2
        q = np.diag([sigma_s ** 2, sigma_s ** 2, sigma_yaw2,
                     math.radians(self.config.bias_random_walk_deg_s) ** 2 * dt])
        self.covariance = f @ self.covariance @ f.T + q
        self.covariance = .5 * (self.covariance + self.covariance.T)

    def observe_heading(self, angle_rad, variance_rad2):
        if not math.isfinite(angle_rad) or not math.isfinite(variance_rad2) or variance_rad2 <= 0:
            raise ValueError("Invalid heading observation")
        innovation = wrap_angle(angle_rad - self.state[2])
        innovation_variance = self.covariance[2, 2] + variance_rad2
        nis = innovation ** 2 / innovation_variance
        accepted = bool(abs(innovation) <= math.radians(self.config.innovation_limit_deg) and nis <= self.config.nis_limit)
        if accepted:
            gain = self.covariance[:, 2] / innovation_variance
            self.state += gain * innovation
            a = np.eye(4)
            a[:, 2] -= gain
            # Joseph form retains numerical symmetry and positive covariance.
            self.covariance = a @ self.covariance @ a.T + np.outer(gain, gain) * variance_rad2
            self.covariance = .5 * (self.covariance + self.covariance.T)
        return {"innovation_rad": innovation, "nis": nis, "accepted": accepted}


def wall_heading_measurement(travel_m, distance_mm, side):
    """Range slope for an axis-aligned parallel wall: L'=-sin(yaw), R'=sin(yaw)."""
    travel_m = np.asarray(travel_m, dtype=float)
    distances = np.asarray(distance_mm, dtype=float) / 1000
    if side not in ("left", "right") or travel_m.shape != distances.shape or travel_m.ndim != 1:
        raise ValueError("Invalid side or wall observation shape")
    if not np.isfinite(travel_m).all() or not np.isfinite(distances).all():
        raise ValueError("Nonfinite wall observation")
    x = travel_m - travel_m.mean()
    denominator = float(x @ x)
    if len(x) < 5 or denominator <= 0:
        raise ValueError("Insufficient spatial span for a wall observation")
    slope = float(x @ (distances - distances.mean()) / denominator)
    fitted = distances.mean() + slope * x
    residual_m = float(np.sqrt(np.mean((distances - fitted) ** 2)))
    signed_slope = -slope if side == "left" else slope
    return signed_slope, residual_m, denominator


def build_observations(data, odometry, config=None):
    """Use non-overlapping, past-only distance blocks; freeze associations from raw yaw."""
    cfg = config or Config()
    xy = odometry[["east_mm", "north_mm"]].to_numpy(float) / 1000
    ds = np.r_[0., np.linalg.norm(np.diff(xy, axis=0), axis=1)]
    travel = np.cumsum(ds)
    yaw = odometry.heading_rad.to_numpy(float)
    left = data["sen_l.distance"].to_numpy(float)
    right = data["sen_r.distance"].to_numpy(float)
    fronts = data[["sen_fl.distance", "sen_fr.distance"]].to_numpy(float)
    eligible = ((data["ideal.velo"].to_numpy() > .2) & (data["ego.velo"].to_numpy() > .12)
                & (abs(data["ideal.rad_velo"].to_numpy()) <= 1.21)
                & (abs(data["ego.rad_velo"].to_numpy()) < 1.5)
                & (fronts.min(axis=1) > 112))
    mask = np.where((left > 30) & (left < 60), 1, 0) + np.where((right > 30) & (right < 60), 2, 0)
    observations, start, previous_mask = [], None, 0
    rejected = {"range_slope": 0, "wall_curvature": 0, "left_right_disagreement": 0, "axis_association": 0}
    for index in range(len(data)):
        current_mask = int(mask[index]) if eligible[index] else 0
        if not current_mask:
            start, previous_mask = None, 0
            continue
        if start is None or current_mask != previous_mask:
            start, previous_mask = index, current_mask
            continue
        if travel[index] - travel[start] < cfg.block_distance_m or index - start < 5:
            continue
        indices = np.arange(start, index + 1)
        slopes, rms, denominators = [], [], []
        sides = []
        for bit, name, values in ((1, "left", left), (2, "right", right)):
            if current_mask & bit:
                slope, residual, denominator = wall_heading_measurement(travel[indices], values[indices], name)
                slopes.append(slope); rms.append(residual); denominators.append(denominator); sides.append(name)
        reason = None
        if max(abs(np.asarray(slopes))) > math.sin(math.radians(15)):
            reason = "range_slope"
        elif max(rms) > .00075:
            reason = "wall_curvature"
        elif len(slopes) == 2 and abs(slopes[0] - slopes[1]) > .12:
            reason = "left_right_disagreement"
        axis = round(float(yaw[indices].mean()) / (math.pi / 2)) * (math.pi / 2)
        if abs(wrap_angle(float(yaw[indices].mean()) - axis)) > math.radians(25):
            reason = "axis_association"
        if reason:
            rejected[reason] += 1
        else:
            variances = [(max(residual, cfg.sensor_noise_floor_mm / 1000) ** 2) / denom
                         for residual, denom in zip(rms, denominators)]
            weights = 1 / np.asarray(variances)
            slope = float(np.average(slopes, weights=weights))
            measured_mean_yaw = axis + math.asin(slope)
            # A slope observes the block's average heading. Transport it to the
            # endpoint with measured yaw change, without an endpoint/map constraint.
            measured_end_yaw = measured_mean_yaw + yaw[index] - float(yaw[indices].mean())
            variance = (1 / float(weights.sum())) / (1 - slope ** 2)
            variance += math.radians(cfg.heading_noise_floor_deg) ** 2
            observations.append({"index": index, "start_index": start, "sequence": int(data.sequence.iloc[index]),
                "sides": "+".join(sides), "side_mask": current_mask, "axis_rad": axis,
                "heading_rad": measured_end_yaw, "variance_rad2": variance,
                "holdout": (len(observations) + 1) % cfg.holdout_stride == 0,
                "spatial_span_mm": float((travel[index] - travel[start]) * 1000),
                "max_fit_residual_mm": float(max(rms) * 1000)})
        # Adjacent blocks share only a boundary sample, not their interior.
        start = index
    return observations, rejected


def estimate_stationary_bias(pre_motion):
    quiet = ((pre_motion["ego.velo"].abs() < .01) & (pre_motion["ego.rad_velo"].abs() < .035)
             & (pre_motion["ideal.velo"].abs() < .001) & (pre_motion["ideal.rad_velo"].abs() < .001))
    values = pre_motion.loc[quiet, "ego.rad_velo"].to_numpy(float)
    if len(values) < 30:
        raise ValueError("Need at least 30 quiet pre-motion gyro samples")
    return float(np.median(values)), len(values)


def replay(odometry, observations, bias, config=None, use_walls=True):
    cfg = config or Config()
    ekf = WallHeadingEkf(bias, cfg)
    raw_xy = odometry[["east_mm", "north_mm"]].to_numpy(float) / 1000
    raw_yaw = odometry.heading_rad.to_numpy(float)
    dt = odometry.estimated_control_interval_ms.to_numpy(float) / 1000
    resets = odometry.interpolated_interval.to_numpy(bool)
    by_index = {item["index"]: item for item in observations}
    states, predicted_states, records = [], [], []
    minimum_eigenvalue = math.inf
    for index in range(len(odometry)):
        if index:
            ekf.predict(raw_xy[index] - raw_xy[index - 1], raw_yaw[index - 1],
                        raw_yaw[index] - raw_yaw[index - 1], dt[index], bool(resets[index]))
        predicted_states.append(ekf.state.copy())
        if index in by_index:
            observation = by_index[index]
            innovation = wrap_angle(observation["heading_rad"] - ekf.state[2])
            record = {**observation, "prior_heading_rad": float(ekf.state[2]), "innovation_rad": innovation, "updated": False}
            if use_walls and not observation["holdout"]:
                result = ekf.observe_heading(observation["heading_rad"], observation["variance_rad2"])
                record.update(result)
                record["updated"] = result["accepted"]
            records.append(record)
        states.append(ekf.state.copy())
        minimum_eigenvalue = min(minimum_eigenvalue, float(np.linalg.eigvalsh(ekf.covariance).min()))
    return np.array(states), np.array(predicted_states), records, minimum_eigenvalue


def heldout_scores(data, odometry, states, predictions, observations):
    raw_xy = odometry[["east_mm", "north_mm"]].to_numpy(float) / 1000
    ds = np.r_[0., np.linalg.norm(np.diff(raw_xy, axis=0), axis=1)]
    heading_errors, range_errors = [], []
    for observation in observations:
        if not observation["holdout"]:
            continue
        a, b = observation["start_index"], observation["index"]
        axis = observation["axis_rad"]
        heading_errors.append(wrap_angle(predictions[b, 2] - observation["heading_rad"]))
        midpoint = .5 * (states[a:b, 2] + predictions[a + 1:b + 1, 2])
        predicted_change_m = float(np.sum(ds[a + 1:b + 1] * np.sin(midpoint - axis)))
        for side, bit, sign in (("l", 1, -1), ("r", 2, 1)):
            if observation["side_mask"] & bit:
                values = data["sen_" + side + ".distance"].to_numpy(float)
                actual_change_mm = values[b] - values[a]
                range_errors.append(sign * predicted_change_m * 1000 - actual_change_mm)
    return {"heldout_blocks": len(heading_errors), "heldout_side_differences": len(range_errors),
            "heading_rmse_deg": float(np.degrees(np.sqrt(np.mean(np.square(heading_errors))))) if heading_errors else None,
            "side_distance_change_rmse_mm": float(np.sqrt(np.mean(np.square(range_errors)))) if range_errors else None,
            "return_offset_mm_not_ground_truth": float(np.linalg.norm(states[-1, :2]) * 1000),
            "return_xy_mm": (states[-1, :2] * 1000).tolist()}


def load_inputs(log_path, odometry_path):
    full = pd.read_csv(log_path)
    odometry = pd.read_csv(odometry_path)
    needed = ["sequence", "time_ms", "ideal.velo", "ego.velo", "ideal.rad_velo", "ego.rad_velo",
              "sen_l.distance", "sen_r.distance", "sen_fl.distance", "sen_fr.distance"]
    if not set(needed).issubset(full.columns):
        raise ValueError("Missing BLE sensor/odometry columns")
    needed_odom = ["sequence", "mcu_time_ms", "east_mm", "north_mm", "heading_rad", "estimated_control_interval_ms", "interpolated_interval"]
    if not set(needed_odom).issubset(odometry.columns) or len(odometry) < 2:
        raise ValueError("Missing reconstructed odometry columns")
    data = full[full.sequence.isin(odometry.sequence)].reset_index(drop=True)
    if not data.sequence.equals(odometry.sequence):
        raise ValueError("Raw log and odometry sequences do not match exactly")
    numeric_odom = [column for column in needed_odom if column != "interpolated_interval"]
    if not np.isfinite(data[needed].to_numpy(float)).all() or not np.isfinite(odometry[numeric_odom].to_numpy(float)).all():
        raise ValueError("Nonfinite required field")
    if not (data.sequence.diff().dropna() == 1).all() or not (data.time_ms.diff().dropna() > 0).all():
        raise ValueError("Discontinuous sequence or MCU time")
    if not ((odometry.estimated_control_interval_ms > 0) & (odometry.estimated_control_interval_ms < 20)).all():
        raise ValueError("Invalid reconstructed control interval")
    if not odometry.interpolated_interval.isin([True, False]).all():
        raise ValueError("Reset interval flags must be boolean")
    if not np.array_equal(data.time_ms.to_numpy(), odometry.mcu_time_ms.to_numpy()):
        raise ValueError("Raw log and odometry MCU times do not match")
    first_index = int(full.index[full.sequence == odometry.sequence.iloc[0]][0])
    earlier_gaps = np.flatnonzero(np.diff(full.time_ms.iloc[:first_index + 1]) > 1000)
    stream_start = int(earlier_gaps[-1] + 1) if len(earlier_gaps) else 0
    pre_motion = full.iloc[stream_start:first_index]
    return data, odometry, pre_motion


def plot_results(output, data, odometry, variants, observations, route_path=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    fonts = {font.name for font in font_manager.fontManager.ttflist}
    plt.rcParams["font.family"] = next((font for font in ("Yu Gothic", "Meiryo", "MS Gothic") if font in fonts), "DejaVu Sans")
    plt.rcParams["axes.unicode_minus"] = False
    fig, axes = plt.subplots(1, 2, figsize=(13, 7), constrained_layout=True)
    labels = {"raw": "元の復元軌跡", "bias_only": "静止時バイアス補正のみ", "wall_ekf": "壁観測 + EKF（1/3は評価用）"}
    colors = {"raw": "#888888", "bias_only": "#2366b4", "wall_ekf": "#bd5b16"}
    route = json.loads(Path(route_path).read_text(encoding="utf-8")) if route_path else None
    if route:
        for phase in ("outward", "return"):
            points = np.asarray(route[phase])
            axes[0].plot(points[:, 0], points[:, 1], "--", color="#333333", lw=1, alpha=.55)
    for name, states in variants.items():
        xy = states[:, :2] * 1000 / 90
        axes[0].plot(xy[:, 0], xy[:, 1], color=colors[name], lw=1.5, label=labels[name])
        axes[0].scatter(*xy[-1], marker="x", color=colors[name], s=45)
    axes[0].scatter(0, 0, facecolor="white", edgecolor="#222222", s=50, zorder=5)
    axes[0].set(xlabel="X [区画 / 90 mm]", ylabel="Y [区画 / 90 mm]", title="軌跡の比較 · route / 終点はフィルタに未入力")
    axes[0].set_aspect("equal"); axes[0].grid(alpha=.25); axes[0].legend(fontsize=8)
    heldout = [item for item in observations if item["holdout"]]
    t = (data.time_ms - data.time_ms.iloc[0]).to_numpy() / 1000
    for name, states in variants.items():
        errors = [math.degrees(wrap_angle(states[item["index"], 2] - item["heading_rad"])) for item in heldout]
        axes[1].scatter([t[item["index"]] for item in heldout], errors, s=22,
                        color=colors[name], label=labels[name])
    axes[1].axhline(0, color="#555555", lw=.8)
    axes[1].set(xlabel="ログ内経過時間 [s・MCU時刻]", ylabel="予測姿勢 − 未使用の壁観測 [°]", title="補正に使っていない壁観測で比較")
    axes[1].grid(alpha=.25); axes[1].legend(fontsize=8)
    fig.suptitle("壁の距離変化を使うオフライン拡張カルマンフィルタ\n位置の正解値は未取得。軌跡は測定後の推定であり、実際の走行動作は変わらない。", fontsize=13)
    fig.savefig(output / "ekf_comparison.png", dpi=170, bbox_inches="tight")
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--odometry", type=Path, required=True, help="Explicit continuous-odometry CSV; no automatic latest selection")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--route", type=Path, help="Display only; never used by the EKF or its wall observations")
    parser.add_argument("--settings", type=Path, help="Capture metadata for provenance only")
    args = parser.parse_args(argv)
    cfg = Config()
    data, odometry, pre_motion = load_inputs(args.log, args.odometry)
    bias, quiet_count = estimate_stationary_bias(pre_motion)
    observations, rejected = build_observations(data, odometry, cfg)
    if sum(item["holdout"] for item in observations) < 3:
        raise ValueError("Not enough held-out wall observations for evaluation")
    raw_states, _, _, _ = replay(odometry, observations, 0., cfg, use_walls=False)
    raw_xy = odometry[["east_mm", "north_mm"]].to_numpy(float) / 1000
    if not np.allclose(raw_states[:, :2], raw_xy - raw_xy[0], atol=1e-10):
        raise RuntimeError("Zero-update replay does not reproduce raw odometry")
    variants, scores, outputs = {"raw": raw_states}, {}, {}
    for name, initial_bias, use_walls in (("raw", 0., False), ("bias_only", bias, False), ("wall_ekf", bias, True)):
        states, predictions, records, minimum_eigenvalue = replay(odometry, observations, initial_bias, cfg, use_walls)
        if not np.isfinite(states).all() or minimum_eigenvalue < -1e-10:
            raise RuntimeError("Nonfinite state or non-positive covariance")
        variants[name] = states
        scores[name] = {**heldout_scores(data, odometry, states, predictions, observations),
                        "wall_updates": int(sum(record["updated"] for record in records)),
                        "min_covariance_eigenvalue": minimum_eigenvalue,
                        "last_bias_deg_s": math.degrees(states[-1, 3])}
        outputs[name] = records
    # Freeze parameters, then rotate which third of the spatial blocks is held
    # out. This checks that the result does not depend on one convenient split.
    folds = []
    for fold in range(cfg.holdout_stride):
        fold_observations = [{**item, "holdout": (number + 1) % cfg.holdout_stride == fold}
                             for number, item in enumerate(observations)]
        fold_scores = {}
        for name, initial_bias, use_walls in (("raw", 0., False), ("bias_only", bias, False), ("wall_ekf", bias, True)):
            states, predictions, records, _ = replay(odometry, fold_observations, initial_bias, cfg, use_walls)
            fold_scores[name] = heldout_scores(data, odometry, states, predictions, fold_observations)
        folds.append({"fold": fold, "scores": fold_scores})
    cross_validation = {}
    for name in variants:
        cross_validation[name] = {}
        for metric, count_key in (("heading_rmse_deg", "heldout_blocks"),
                                  ("side_distance_change_rmse_mm", "heldout_side_differences")):
            count = sum(item["scores"][name][count_key] for item in folds)
            mse = sum(item["scores"][name][metric] ** 2 * item["scores"][name][count_key] for item in folds) / count
            cross_validation[name][metric] = math.sqrt(mse)
            cross_validation[name][count_key] = count
    args.output.mkdir(parents=True, exist_ok=True)
    export = odometry[["sequence", "mcu_time_ms"]].copy()
    for name, states in variants.items():
        for column, values in zip(("east_mm", "north_mm", "heading_rad", "bias_rad_s"), states.T):
            export[name + "." + column] = values * 1000 if column.endswith("_mm") else values
    export.to_csv(args.output / "trajectories.csv", index=False)
    pd.DataFrame(outputs["wall_ekf"]).to_csv(args.output / "wall_observations.csv", index=False)
    metadata = json.loads(args.settings.read_text(encoding="utf-8")) if args.settings else {}
    input_paths = [args.log, args.odometry] + ([args.settings] if args.settings else []) + ([args.route] if args.route else [])
    report = {"inputs": {"log": str(args.log.resolve()), "odometry": str(args.odometry.resolve()),
                         "settings": str(args.settings.resolve()) if args.settings else None,
                         "route_display_only": str(args.route.resolve()) if args.route else None},
              "machine": metadata.get("firmware", {}).get("mouse", "not recorded"),
              "run_options": metadata.get("options", {}), "firmware_build_id": "not recorded in capture",
              "config": asdict(cfg), "rows": len(data), "quiet_calibration_samples": quiet_count,
              "initial_bias_deg_s": math.degrees(bias), "observations": len(observations),
              "observation_rejections": rejected, "scores": scores,
              "three_fold_cross_validation": cross_validation, "folds": folds,
              "route_used_for_filtering": False, "return_position_used_for_filtering": False,
              "config_selected_using_return_or_route": False, "ground_truth_position_available": False,
              "limitations": ["Axis-aligned parallel-wall assumption; associations are frozen from raw odometry, not an independent wall map.",
                              "Held-out wall observations assess model/sensor consistency, not externally measured trajectory accuracy; adjacent blocks share one boundary sample.",
                              "Reconstructed odometry already contains reset-detection and timing uncertainty; bias may absorb those errors.",
                              "The preprocessing that created the input odometry is offline; this is not a firmware-ready streaming implementation.",
                              "A posteriori estimator changes do not change the recorded physical motion.",
                              "No forward-wall or absolute position measurements; lateral slip and distance-scale error remain."],
              "sha256": {str(path.resolve()): hashlib.sha256(path.read_bytes()).hexdigest() for path in input_paths},
              "implementation_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              "reference": "https://atsushisakai.github.io/PythonRobotics/modules/2_localization/extended_kalman_filter_localization_files/extended_kalman_filter_localization.html"}
    (args.output / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    plot_results(args.output, data, odometry, variants, observations, args.route)
    print(json.dumps({"observations": len(observations), "quiet_bias_deg_s": math.degrees(bias),
                      "scores": scores, "three_fold_cross_validation": cross_validation}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
