import argparse
import json
import re
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[2]
LOG_DIR = ROOT / "tools" / "logs"
LOG_PERIOD_MS = 1.0
JERK_FF_LIMIT_V = 1.25

COMPARISONS = (
    ("ideal.velo", "ego.velo", "Velocity", "Velocity"),
    ("ideal.rad_velo", "ego.rad_velo", "Angular velocity", "Angular velocity"),
    ("ideal.length", "ego.length", "Length", "Length"),
    ("ideal.radian", "ego.radian", "Radian", "Radian"),
    ("ideal.accel", "ego.accel", "Acceleration", "Acceleration"),
    ("ideal.x_point", "ego.x_point", "X point", "X point"),
)


def latest_csv(log_dir=LOG_DIR):
    files = list(Path(log_dir).glob("*.csv"))
    if not files:
        raise FileNotFoundError(f"CSVがありません: {log_dir}")
    return max(files, key=lambda path: path.stat().st_mtime)


def load_log(path):
    frame = pd.read_csv(path)
    frame.columns = [str(column).strip() for column in frame.columns]
    frame = frame.apply(pd.to_numeric, errors="coerce")
    frame = frame.dropna(how="all").reset_index(drop=True)
    if frame.empty:
        raise ValueError("CSVにログデータがありません。")
    return frame


def load_settings(csv_path, settings_path=None):
    path = settings_path or csv_path.with_suffix(".settings.json")
    if not path.exists():
        return None, None
    return json.loads(path.read_text(encoding="utf-8")), path


def infer_jerk_gain_from_source(settings):
    """Read the eighth t_ff_gain value for long-turn 90 from Param_A."""
    if not settings:
        return None, None
    feedforward = settings.get("feedforward", {})
    if "ff_om_jerk" in feedforward:
        return float(feedforward["ff_om_jerk"]), "settings.json"

    motion_name = str(settings.get("motion_name", ""))
    speed = settings.get("preset_speed_mm_s")
    match = re.fullmatch(r"long_([rl])90", motion_name)
    if not match or speed is None:
        return None, None
    direction = match.group(1).upper()
    speed = int(speed)
    header = ROOT / "Params" / "Param_A" / f"turn_{speed}.h"
    if not header.exists():
        return None, None
    source = header.read_text(encoding="utf-8", errors="replace")
    gain_names = (
        f"ff_gain_long_turn_90_{direction}_{speed}",
        f"ff_gain_long_turn_{direction}_{speed}",
    )
    for gain_name in gain_names:
        gain_match = re.search(
            rf"\b{re.escape(gain_name)}\s*=\s*\{{(?P<values>.*?)\}}\s*;",
            source,
            re.DOTALL,
        )
        if not gain_match:
            continue
        values = re.findall(
            r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?f?",
            gain_match.group("values"),
        )
        if len(values) >= 8:
            return float(values[7].rstrip("f")), f"Param_A/{header.name}"
        return 0.0, f"Param_A/{header.name}"
    return None, None


def forward_difference(values, period_ms):
    values = np.asarray(values, dtype=float)
    result = np.zeros_like(values)
    if len(values) > 1:
        result[:-1] = np.diff(values) * 1000.0 / period_ms
    return result


def reconstruct_angular_feedforward(frame, settings, period_ms, jerk_gain=None):
    if settings is None or "ideal.rad_velo" not in frame:
        return None
    ff = settings.get("feedforward", {})
    required = ("ff_om_velo", "ff_om_accel", "ff_om_decel", "ff_om_bias")
    if not all(name in ff for name in required):
        return None

    omega = frame["ideal.rad_velo"].to_numpy(dtype=float)
    alpha = forward_difference(omega, period_ms)
    jerk = forward_difference(alpha, period_ms)
    direction = np.where(np.abs(omega) > 0.001, np.sign(omega), np.sign(alpha))
    velocity_ff = float(ff["ff_om_velo"]) * omega
    accel_gain = np.where(
        omega * alpha < 0.0,
        float(ff["ff_om_decel"]),
        float(ff["ff_om_accel"]),
    )
    accel_ff = accel_gain * alpha
    bias_ff = float(ff["ff_om_bias"]) * direction
    base_ff = velocity_ff + accel_ff + bias_ff
    profile_jerk_ff = None
    if jerk_gain is not None:
        profile_jerk_ff = np.clip(jerk_gain * jerk, -JERK_FF_LIMIT_V, JERK_FF_LIMIT_V)
    residual_jerk_ff = None
    if "om_feedforward" in frame:
        residual_jerk_ff = frame["om_feedforward"].to_numpy(dtype=float) - base_ff
    return {
        "alpha": alpha,
        "jerk": jerk,
        "velocity_ff": velocity_ff,
        "accel_ff": accel_ff,
        "profile_jerk_ff": profile_jerk_ff,
        "residual_jerk_ff": residual_jerk_ff,
    }


def infer_jerk_gain_from_log(frame, settings, period_ms):
    """Estimate captured Kj after subtracting settings-based legacy OM-FF.

    This assumes the acceleration taper is disabled; otherwise its voltage is
    also present in the residual and the estimate is intentionally not pure Kj.
    """
    reconstructed = reconstruct_angular_feedforward(frame, settings, period_ms)
    if reconstructed is None or reconstructed["residual_jerk_ff"] is None:
        return None, None
    omega = frame["ideal.rad_velo"].to_numpy(dtype=float)
    jerk = reconstructed["jerk"]
    residual = reconstructed["residual_jerk_ff"]
    mask = (
        (np.abs(omega) > 1.0)
        & np.isfinite(jerk)
        & np.isfinite(residual)
        & (np.abs(jerk) > 1000.0)
    )
    if np.count_nonzero(mask) < 20:
        return None, None
    candidates = np.linspace(0.0, 5.0e-6, 5001)
    losses = np.empty_like(candidates)
    for index, candidate in enumerate(candidates):
        predicted = np.clip(candidate * jerk[mask], -JERK_FF_LIMIT_V, JERK_FF_LIMIT_V)
        error = residual[mask] - predicted
        # Limit isolated transition/half-float artifacts in the fit.
        losses[index] = np.mean(np.minimum(np.square(error), 0.01))
    best = float(candidates[int(np.argmin(losses))])
    return best, "inferred from OM-FF residual"


def time_axis(frame, period_ms):
    if "cnt" in frame and frame["cnt"].notna().all():
        return frame["cnt"].to_numpy(dtype=float) * period_ms
    return np.arange(len(frame), dtype=float) * period_ms


def rms_error(target, measured):
    valid = np.isfinite(target) & np.isfinite(measured)
    if not np.any(valid):
        return float("nan")
    return float(np.sqrt(np.mean(np.square(measured[valid] - target[valid]))))


def integrate_pose(velocity, angular_velocity, period_ms, initial_heading=0.0):
    """Integrate pose using the firmware coordinate convention: x=right, y=forward."""
    velocity = np.nan_to_num(np.asarray(velocity, dtype=float))
    angular_velocity = np.nan_to_num(np.asarray(angular_velocity, dtype=float))
    count = min(len(velocity), len(angular_velocity))
    x = np.zeros(count, dtype=float)
    y = np.zeros(count, dtype=float)
    heading = np.zeros(count, dtype=float)
    if count == 0:
        return x, y, heading
    heading[0] = initial_heading
    dt_s = period_ms / 1000.0
    for index in range(1, count):
        omega_mean = (angular_velocity[index - 1] + angular_velocity[index]) * 0.5
        heading_mid = heading[index - 1] + omega_mean * dt_s * 0.5
        heading[index] = heading[index - 1] + omega_mean * dt_s
        velocity_mean = (velocity[index - 1] + velocity[index]) * 0.5
        distance_mm = velocity_mean * period_ms
        x[index] = x[index - 1] + distance_mm * np.sin(heading_mid)
        y[index] = y[index - 1] + distance_mm * np.cos(heading_mid)
    return x, y, heading


def plot_comparisons(frame, source_name, period_ms=LOG_PERIOD_MS, settings=None,
                     jerk_gain=None, jerk_gain_source=None):
    time_ms = time_axis(frame, period_ms)
    figure, axes = plt.subplots(6, 2, figsize=(16, 20), constrained_layout=True)
    axes = axes.ravel()

    for axis, (target_name, measured_name, title, ylabel) in zip(axes, COMPARISONS):
        if target_name not in frame or measured_name not in frame:
            axis.set_visible(False)
            continue
        target = frame[target_name].to_numpy(dtype=float)
        measured = frame[measured_name].to_numpy(dtype=float)
        error = measured - target
        rmse = rms_error(target, measured)
        axis.plot(time_ms, target, label=f"target: {target_name}", linewidth=1.8)
        axis.plot(time_ms, measured, label=f"measured: {measured_name}", linewidth=1.2)
        axis.plot(time_ms, error, label="error", linewidth=0.9, alpha=0.65)
        axis.set_title(f"{title}  RMSE={rmse:.5g}")
        axis.set_xlabel("Time [ms]")
        axis.set_ylabel(ylabel)
        axis.grid(True, alpha=0.3)
        axis.legend(fontsize=8)

    target_pose = measured_pose = None
    if all(column in frame for column in ("ideal.velo", "ideal.rad_velo", "ego.velo", "ego.rad_velo")):
        target_heading = float(frame["ideal.radian"].iloc[0]) if "ideal.radian" in frame else 0.0
        measured_heading = float(frame["ego.radian"].iloc[0]) if "ego.radian" in frame else 0.0
        target_pose = integrate_pose(frame["ideal.velo"], frame["ideal.rad_velo"], period_ms, target_heading)
        measured_pose = integrate_pose(frame["ego.velo"], frame["ego.rad_velo"], period_ms, measured_heading)

    trajectory_axis = axes[6]
    trajectory_columns = ("ideal.turn_x", "ideal.turn_y", "ego.turn_x", "ego.turn_y")
    if target_pose is not None:
        trajectory_axis.plot(target_pose[0], target_pose[1], label="target (integrated v/omega)", linewidth=2.0)
        trajectory_axis.plot(measured_pose[0], measured_pose[1], label="measured (integrated v/omega)", linewidth=1.5)
        if all(column in frame for column in trajectory_columns):
            trajectory_axis.plot(frame["ideal.turn_x"], frame["ideal.turn_y"], "--", label="target (logged position)", alpha=0.6)
            trajectory_axis.plot(frame["ego.turn_x"], frame["ego.turn_y"], "--", label="measured (logged position)", alpha=0.6)
        trajectory_axis.scatter(0.0, 0.0, marker="o", label="integration start")
        trajectory_axis.set_title("Position trajectory from velocity / angular velocity")
        trajectory_axis.set_xlabel("turn_x")
        trajectory_axis.set_ylabel("turn_y")
        trajectory_axis.axis("equal")
        trajectory_axis.grid(True, alpha=0.3)
        trajectory_axis.legend(fontsize=8)
    else:
        trajectory_axis.set_visible(False)

    position_axis = axes[7]
    if target_pose is not None:
        position_axis.plot(time_ms, target_pose[0], label="target x (integrated)", linewidth=1.7)
        position_axis.plot(time_ms, measured_pose[0], label="measured x (integrated)", linewidth=1.1)
        position_axis.plot(time_ms, target_pose[1], label="target y (integrated)", linewidth=1.7)
        position_axis.plot(time_ms, measured_pose[1], label="measured y (integrated)", linewidth=1.1)
        position_axis.set_title("Integrated x / y time series")
        position_axis.set_xlabel("Time [ms]")
        position_axis.set_ylabel("Position")
        position_axis.grid(True, alpha=0.3)
        position_axis.legend(fontsize=8)
    else:
        position_axis.set_visible(False)

    motor_axis = axes[8]
    motor_columns = [name for name in ("V_r", "V_l") if name in frame]
    if motor_columns:
        for name in motor_columns:
            motor_axis.plot(time_ms, frame[name], label=name, linewidth=1.1)
        if "Battery" in frame:
            battery = frame["Battery"].to_numpy(dtype=float)
            motor_axis.plot(time_ms, battery, "--", label="+Battery", alpha=0.65)
            motor_axis.plot(time_ms, -battery, "--", label="-Battery", alpha=0.65)
        motor_axis.set_title("Motor command voltage")
        motor_axis.set_xlabel("Time [ms]")
        motor_axis.set_ylabel("Voltage [V]")
        motor_axis.grid(True, alpha=0.3)
        motor_axis.legend(fontsize=8)
    else:
        motor_axis.set_visible(False)

    control_axis = axes[9]
    plotted_control = False
    for name in ("sp_feedforward", "sp_feedback", "om_feedforward", "om_feedback"):
        if name in frame:
            control_axis.plot(time_ms, frame[name], label=name, linewidth=1.0)
            plotted_control = True
    if plotted_control:
        control_axis.set_title("Control voltage components")
        control_axis.set_xlabel("Time [ms]")
        control_axis.set_ylabel("Voltage [V]")
        control_axis.grid(True, alpha=0.3)
        control_axis.legend(fontsize=8)
    else:
        control_axis.set_visible(False)

    reconstructed = reconstruct_angular_feedforward(frame, settings, period_ms, jerk_gain)
    kinematics_axis = axes[10]
    feedforward_axis = axes[11]
    if reconstructed is not None:
        kinematics_axis.plot(
            time_ms, reconstructed["alpha"], label="ideal angular acceleration", linewidth=1.1,
        )
        kinematics_axis.set_title("Derived ideal angular acceleration / jerk")
        kinematics_axis.set_xlabel("Time [ms]")
        kinematics_axis.set_ylabel("Angular acceleration [rad/s^2]")
        kinematics_axis.grid(True, alpha=0.3)
        jerk_axis = kinematics_axis.twinx()
        jerk_axis.plot(
            time_ms, reconstructed["jerk"], color="tab:red",
            label="ideal angular jerk", alpha=0.65,
        )
        jerk_axis.set_ylabel("Angular jerk [rad/s^3]")
        lines = kinematics_axis.lines + jerk_axis.lines
        kinematics_axis.legend(lines, [line.get_label() for line in lines], fontsize=8)

        feedforward_axis.plot(
            time_ms, reconstructed["velocity_ff"], label="omega FF from settings", linewidth=1.0,
        )
        feedforward_axis.plot(
            time_ms, reconstructed["accel_ff"], label="accel/decel FF from settings", linewidth=1.0,
        )
        if reconstructed["profile_jerk_ff"] is not None:
            feedforward_axis.plot(
                time_ms, reconstructed["profile_jerk_ff"],
                label="jerk FF from profile", linewidth=1.2,
            )
        if reconstructed["residual_jerk_ff"] is not None:
            feedforward_axis.plot(
                time_ms, reconstructed["residual_jerk_ff"], "--",
                label="extra FF inferred from log", linewidth=1.0,
            )
        if "om_feedforward" in frame:
            feedforward_axis.plot(
                time_ms, frame["om_feedforward"], label="logged om_feedforward", alpha=0.65,
            )
        gain_text = "unknown" if jerk_gain is None else f"{jerk_gain:.4g}"
        source_text = "" if not jerk_gain_source else f" ({jerk_gain_source})"
        feedforward_axis.set_title(f"Angular feedforward reconstruction  Kj={gain_text}{source_text}")
        feedforward_axis.set_xlabel("Time [ms]")
        feedforward_axis.set_ylabel("Voltage [V]")
        feedforward_axis.grid(True, alpha=0.3)
        feedforward_axis.legend(fontsize=8)
    else:
        kinematics_axis.set_visible(False)
        feedforward_axis.set_visible(False)

    figure.suptitle(f"Micromouse log comparison\n{source_name}", fontsize=14)
    return figure


def main():
    parser = argparse.ArgumentParser(description="myshellログCSVの目標値・測定値・位置軌跡を比較します。")
    parser.add_argument("csv", nargs="?", type=Path, help="解析するCSV。省略時はtools/logs内の最新CSV。")
    parser.add_argument("--period-ms", type=float, default=LOG_PERIOD_MS, help="1ログサンプルの周期 [ms]。")
    parser.add_argument("--save", type=Path, help="グラフを保存するPNG/PDFパス。")
    parser.add_argument("--no-show", action="store_true", help="グラフウィンドウを表示しません。")
    parser.add_argument("--settings", type=Path, help="Settings JSON paired with the CSV.")
    parser.add_argument(
        "--jerk-gain", type=float,
        help="OM jerk FF gain. Settings/source inference is used when omitted.",
    )
    args = parser.parse_args()

    csv_path = args.csv or latest_csv()
    frame = load_log(csv_path)
    settings, settings_path = load_settings(csv_path, args.settings)
    jerk_gain = args.jerk_gain
    jerk_gain_source = "command line" if jerk_gain is not None else None
    if jerk_gain is None:
        configured_jerk = (settings or {}).get("feedforward", {}).get("ff_om_jerk")
        if configured_jerk is not None:
            jerk_gain = float(configured_jerk)
            jerk_gain_source = "settings.json"
        else:
            jerk_gain, jerk_gain_source = infer_jerk_gain_from_log(
                frame, settings, args.period_ms,
            )
        if jerk_gain is None:
            jerk_gain, jerk_gain_source = infer_jerk_gain_from_source(settings)
    figure = plot_comparisons(
        frame, csv_path.name, args.period_ms, settings, jerk_gain, jerk_gain_source,
    )

    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(args.save, dpi=160)
        print(f"Saved: {args.save}")
    print(f"Analyzed: {csv_path} ({len(frame)} rows)")
    if settings_path:
        print(f"Settings: {settings_path}")
    if jerk_gain is not None:
        print(f"Jerk gain: {jerk_gain:.7g} ({jerk_gain_source})")
        if jerk_gain_source == "inferred from OM-FF residual":
            print("Note: inferred Kj assumes the acceleration taper was disabled.")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
