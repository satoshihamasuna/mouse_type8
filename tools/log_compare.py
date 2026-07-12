import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


LOG_DIR = Path(__file__).resolve().parent / "logs"
LOG_PERIOD_MS = 2.0

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


def plot_comparisons(frame, source_name, period_ms=LOG_PERIOD_MS):
    time_ms = time_axis(frame, period_ms)
    figure, axes = plt.subplots(4, 2, figsize=(15, 13), constrained_layout=True)
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

    figure.suptitle(f"Micromouse log comparison\n{source_name}", fontsize=14)
    return figure


def main():
    parser = argparse.ArgumentParser(description="myshellログCSVの目標値・測定値・位置軌跡を比較します。")
    parser.add_argument("csv", nargs="?", type=Path, help="解析するCSV。省略時はtools/logs内の最新CSV。")
    parser.add_argument("--period-ms", type=float, default=LOG_PERIOD_MS, help="1ログサンプルの周期 [ms]。")
    parser.add_argument("--save", type=Path, help="グラフを保存するPNG/PDFパス。")
    parser.add_argument("--no-show", action="store_true", help="グラフウィンドウを表示しません。")
    args = parser.parse_args()

    csv_path = args.csv or latest_csv()
    frame = load_log(csv_path)
    figure = plot_comparisons(frame, csv_path.name, args.period_ms)

    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(args.save, dpi=160)
        print(f"Saved: {args.save}")
    print(f"Analyzed: {csv_path} ({len(frame)} rows)")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
