"""Compare encoder-only and logged velo/omega odometry for turn logs."""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[2]
TOOLS = ROOT / "tools"
sys.path.insert(0, str(Path(__file__).resolve().parent))

from turn_ff_report import select_latest_group, turn_segments  # noqa: E402


DT_S = 0.001
TIRE_DIAMETER_MM = 15.02
ENCODER_RESOLUTION = 172
MM_PER_PULSE = TIRE_DIAMETER_MM * math.pi / ENCODER_RESOLUTION
NOMINAL_TREAD_MM = 28.0

MOTIONS_1600 = (
    "long_r90", "long_l90", "long_r180", "long_l180",
    "in_r45", "in_l45", "in_r135", "in_l135", "r_v90", "l_v90",
)

FAMILY = {
    "long_r90": "long90", "long_l90": "long90",
    "long_r180": "long180", "long_l180": "long180",
    "in_r45": "in45", "in_l45": "in45",
    "in_r135": "in135", "in_l135": "in135",
    "r_v90": "v90", "l_v90": "v90",
}


@dataclass
class Trace:
    motion: str
    source: str
    direction: float
    wheel_x: np.ndarray
    wheel_y: np.ndarray
    wheel_theta: np.ndarray
    vo_x: np.ndarray
    vo_y: np.ndarray
    vo_theta: np.ndarray
    wheel_distance_mm: float
    vo_distance_mm: float
    encoder_difference_mm: float


def integrate_odometry(ds_mm: np.ndarray, dtheta_rad: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Integrate increments with midpoint heading in firmware x/y convention."""
    theta_before = np.r_[0.0, np.cumsum(dtheta_rad[:-1])]
    theta_mid = theta_before + dtheta_rad / 2.0
    x = np.r_[0.0, np.cumsum(ds_mm * np.sin(theta_mid))]
    y = np.r_[0.0, np.cumsum(ds_mm * np.cos(theta_mid))]
    theta = np.r_[0.0, np.cumsum(dtheta_rad)]
    return x, y, theta


def expected_segment_ms(settings: dict) -> int:
    params = settings["parameters"]
    omega_max = abs(float(params["velo"]) / (float(params["r_min"]) / 1000.0))
    duration_ms = abs(math.radians(float(params["degree"]))) / (0.7043 * omega_max) * 1000.0
    return math.ceil(duration_ms) + 2


def extract_traces(speed: int, motions: tuple[str, ...] = MOTIONS_1600) -> list[Trace]:
    traces: list[Trace] = []
    for motion in motions:
        for csv_path, settings in select_latest_group(speed, motion):
            frame = pd.read_csv(csv_path)
            direction = 1.0 if float(settings["parameters"]["degree"]) > 0.0 else -1.0
            ideal_omega = direction * frame["ideal.rad_velo"].to_numpy(float)
            expected = expected_segment_ms(settings)
            tolerance = max(3, math.ceil(expected * 0.06))
            for start, end in turn_segments(ideal_omega):
                if abs((end - start + 1) - expected) > tolerance:
                    continue
                segment = frame.iloc[start:end + 1]
                right_mm = (
                    segment["Encoder_GetProperty_Right().sp_pulse"].to_numpy(float)
                    * MM_PER_PULSE
                )
                # A forward-moving left wheel has a negative logged pulse.
                left_mm = (
                    -segment["Encoder_GetProperty_Left().sp_pulse"].to_numpy(float)
                    * MM_PER_PULSE
                )
                wheel_ds = (right_mm + left_mm) / 2.0
                wheel_dtheta = (right_mm - left_mm) / NOMINAL_TREAD_MM
                vo_ds = segment["ego.velo"].to_numpy(float)
                vo_dtheta = segment["ego.rad_velo"].to_numpy(float) * DT_S
                wx, wy, wt = integrate_odometry(wheel_ds, wheel_dtheta)
                vx, vy, vt = integrate_odometry(vo_ds, vo_dtheta)
                # Normalize right and left turns into the same positive lateral direction.
                traces.append(Trace(
                    motion=motion,
                    source=csv_path.name,
                    direction=direction,
                    wheel_x=direction * wx,
                    wheel_y=wy,
                    wheel_theta=direction * wt,
                    vo_x=direction * vx,
                    vo_y=vy,
                    vo_theta=direction * vt,
                    wheel_distance_mm=float(np.sum(wheel_ds)),
                    vo_distance_mm=float(np.sum(vo_ds)),
                    encoder_difference_mm=float(abs(np.sum(right_mm - left_mm))),
                ))
    return traces


def summarize(traces: list[Trace]) -> tuple[pd.DataFrame, float]:
    effective_tread = sum(t.encoder_difference_mm for t in traces) / sum(abs(t.vo_theta[-1]) for t in traces)
    rows = []
    for motion in MOTIONS_1600:
        selected = [t for t in traces if t.motion == motion]
        if not selected:
            continue
        calibrated_separation = []
        for trace in selected:
            ds = np.hypot(np.diff(trace.wheel_x), np.diff(trace.wheel_y))
            dtheta = np.diff(trace.wheel_theta) * NOMINAL_TREAD_MM / effective_tread
            cx, cy, _ = integrate_odometry(ds, dtheta)
            calibrated_separation.append(
                math.hypot(cx[-1] - trace.vo_x[-1], cy[-1] - trace.vo_y[-1])
            )
        values = np.asarray([
            (
                t.wheel_x[-1], t.wheel_y[-1], math.degrees(t.wheel_theta[-1]),
                t.vo_x[-1], t.vo_y[-1], math.degrees(t.vo_theta[-1]),
                t.wheel_distance_mm / t.vo_distance_mm,
                t.encoder_difference_mm / abs(t.vo_theta[-1]),
                math.hypot(t.wheel_x[-1] - t.vo_x[-1], t.wheel_y[-1] - t.vo_y[-1]),
            )
            for t in selected
        ])
        mean = values.mean(axis=0)
        std = values.std(axis=0)
        rows.append({
            "motion": motion,
            "turns": len(selected),
            "wheel_x_mm": mean[0], "wheel_y_mm": mean[1],
            "wheel_angle_deg": mean[2],
            "vo_x_mm": mean[3], "vo_y_mm": mean[4],
            "vo_angle_deg": mean[5],
            "angle_difference_deg": mean[2] - mean[5],
            "endpoint_separation_mm": mean[8],
            "endpoint_separation_std_mm": std[8],
            "calibrated_endpoint_separation_mm": float(np.mean(calibrated_separation)),
            "encoder_to_vo_distance_ratio": mean[6],
            "effective_tread_mm": mean[7],
        })
    return pd.DataFrame(rows), effective_tread


def _resample(values: np.ndarray, count: int = 151) -> np.ndarray:
    return np.interp(np.linspace(0.0, 1.0, count), np.linspace(0.0, 1.0, len(values)), values)


def plot_trajectories(traces: list[Trace], effective_tread_mm: float, output: Path) -> None:
    families = ("long90", "long180", "in45", "in135", "v90")
    figure, axes = plt.subplots(2, 3, figsize=(13, 8.5))
    for axis, family in zip(axes.flat, families):
        selected = [t for t in traces if FAMILY[t.motion] == family]
        wheel_x = np.mean([_resample(t.wheel_x) for t in selected], axis=0)
        wheel_y = np.mean([_resample(t.wheel_y) for t in selected], axis=0)
        vo_x = np.mean([_resample(t.vo_x) for t in selected], axis=0)
        vo_y = np.mean([_resample(t.vo_y) for t in selected], axis=0)
        # Reintegrate encoder increments by scaling only the angular increments
        # from nominal to the observed effective tread.
        calibrated_x, calibrated_y = [], []
        for trace in selected:
            ds = np.diff(np.hypot(0.0, trace.wheel_y))
            # Recover signed distance from path increments rather than radial distance.
            dx, dy = np.diff(trace.wheel_x), np.diff(trace.wheel_y)
            ds = np.hypot(dx, dy)
            dtheta = np.diff(trace.wheel_theta) * NOMINAL_TREAD_MM / effective_tread_mm
            cx, cy, _ = integrate_odometry(ds, dtheta)
            calibrated_x.append(_resample(cx))
            calibrated_y.append(_resample(cy))
        cx = np.mean(calibrated_x, axis=0)
        cy = np.mean(calibrated_y, axis=0)
        axis.plot(wheel_x, wheel_y, label="encoder only (28 mm tread)", linewidth=2)
        axis.plot(vo_x, vo_y, label="velo + gyro omega", linewidth=2)
        axis.plot(cx, cy, label=f"encoder only ({effective_tread_mm:.1f} mm tread)", linewidth=1.5, linestyle="--")
        axis.scatter([wheel_x[-1], vo_x[-1], cx[-1]], [wheel_y[-1], vo_y[-1], cy[-1]], s=20)
        axis.set_title(f"{family} ({len(selected)} turns)")
        axis.set_xlabel("normalized lateral x [mm]")
        axis.set_ylabel("forward y [mm]")
        axis.grid(True, alpha=0.25)
        axis.set_aspect("equal", adjustable="datalim")
    axes.flat[-1].axis("off")
    handles, labels = axes.flat[0].get_legend_handles_labels()
    figure.legend(handles, labels, loc="lower right", bbox_to_anchor=(0.98, 0.08))
    figure.suptitle("1600 mm/s turn odometry comparison (R/L direction normalized)")
    figure.tight_layout(rect=(0, 0.05, 1, 0.96))
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=170)
    plt.close(figure)


def write_report(summary: pd.DataFrame, effective_tread_mm: float, output: Path) -> None:
    total_turns = int(summary["turns"].sum())
    distance_ratio = float(np.average(summary["encoder_to_vo_distance_ratio"], weights=summary["turns"]))
    angle_error = float(np.average(summary["angle_difference_deg"], weights=summary["turns"]))
    endpoint_error = float(np.average(summary["endpoint_separation_mm"], weights=summary["turns"]))
    calibrated_error = float(np.average(
        summary["calibrated_endpoint_separation_mm"], weights=summary["turns"]
    ))
    lines = [
        "# 1600 mm/s turn odometry comparison", "",
        f"- turns: {total_turns}",
        f"- nominal tread: {NOMINAL_TREAD_MM:.1f} mm",
        f"- encoder/gyro observed effective tread: {effective_tread_mm:.3f} mm",
        f"- encoder distance / velo distance: {distance_ratio:.4f}",
        f"- mean wheel-minus-velo/omega angle: {angle_error:+.3f} deg",
        f"- mean endpoint separation: {endpoint_error:.3f} mm", "",
        f"- mean endpoint separation with observed tread: {calibrated_error:.3f} mm", "",
        "| motion | n | wheel angle [deg] | velo/omega angle [deg] | angle diff [deg] | endpoint diff [mm] | distance ratio | effective tread [mm] |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in summary.itertuples():
        lines.append(
            f"| {row.motion} | {row.turns} | {row.wheel_angle_deg:.3f} | "
            f"{row.vo_angle_deg:.3f} | {row.angle_difference_deg:+.3f} | "
            f"{row.endpoint_separation_mm:.3f} | {row.encoder_to_vo_distance_ratio:.4f} | "
            f"{row.effective_tread_mm:.3f} |"
        )
    lines.extend(("", "![Trajectories](trajectories.png)", ""))
    output.write_text("\n".join(lines), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--speed", type=int, default=1600)
    parser.add_argument(
        "--output", type=Path,
        default=ROOT / "tools" / "turn_analysis" / "1600" / "odometry_compare",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    traces = extract_traces(args.speed)
    if not traces:
        raise SystemExit(f"no {args.speed} mm/s turn logs found")
    summary, effective_tread = summarize(traces)
    args.output.mkdir(parents=True, exist_ok=True)
    summary.to_csv(args.output / "summary.csv", index=False)
    plot_trajectories(traces, effective_tread, args.output / "trajectories.png")
    write_report(summary, effective_tread, args.output / "report.md")
    print(json.dumps({
        "turns": int(summary["turns"].sum()),
        "motions": int(len(summary)),
        "effective_tread_mm": effective_tread,
        "output": str(args.output),
    }, indent=2))


if __name__ == "__main__":
    main()
