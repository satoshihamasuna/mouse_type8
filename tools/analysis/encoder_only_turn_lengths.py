"""Solve 1600 mm/s Lstart/Lend from encoder pulses only.

The ideal yaw-rate column is used only to locate the commanded turn interval.
Trajectory distance and angle use the right/left encoder pulses and the
specified effective tread; ego.velo, gyro omega, beta, and acceleration are
not used.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parent))

from odometry_compare import MM_PER_PULSE, expected_segment_ms, integrate_odometry  # noqa: E402
from turn_ff_report import select_latest_group, turn_segments  # noqa: E402


TURN_CASES = (
    ("long90", "R", "long_r90", 90.0, 90.0, 0.0),
    ("long90", "L", "long_l90", 90.0, 90.0, 0.0),
    ("long180", "R", "long_r180", 90.0, 0.0, 0.0),
    ("long180", "L", "long_l180", 90.0, 0.0, 0.0),
    ("in45", "R", "in_r45", 45.0, 90.0, 0.0),
    ("in45", "L", "in_l45", 45.0, 90.0, 0.0),
    ("out45", "R", "out_r45", 90.0, 45.0, 45.0),
    ("out45", "L", "out_l45", 90.0, 45.0, 45.0),
    ("in135", "R", "in_r135", 90.0, 45.0, 0.0),
    ("in135", "L", "in_l135", 90.0, 45.0, 0.0),
    ("out135", "R", "out_r135", 90.0, -45.0, 45.0),
    ("out135", "L", "out_l135", 90.0, -45.0, 45.0),
    ("v90", "R", "r_v90", 90.0, 0.0, 45.0),
    ("v90", "L", "l_v90", 90.0, 0.0, 45.0),
)


def rotate_from_body(lateral: float, forward: float, heading: float) -> tuple[float, float]:
    return (
        lateral * math.cos(heading) + forward * math.sin(heading),
        -lateral * math.sin(heading) + forward * math.cos(heading),
    )


def calculate(tread_mm: float) -> pd.DataFrame:
    rows: list[dict] = []
    for motion, side, log_motion, target_lateral, target_forward, start_deg in TURN_CASES:
        for csv_path, settings in select_latest_group(1600, log_motion):
            frame = pd.read_csv(csv_path)
            direction = 1.0 if float(settings["parameters"]["degree"]) > 0.0 else -1.0
            ideal_omega = direction * frame["ideal.rad_velo"].to_numpy(float)
            expected = expected_segment_ms(settings)
            tolerance = max(3, math.ceil(expected * 0.06))
            for turn_index, (start, end) in enumerate(turn_segments(ideal_omega), start=1):
                if abs((end - start + 1) - expected) > tolerance:
                    continue
                segment = frame.iloc[start:end + 1]
                right_mm = (
                    segment["Encoder_GetProperty_Right().sp_pulse"].to_numpy(float)
                    * MM_PER_PULSE
                )
                left_mm = (
                    -segment["Encoder_GetProperty_Left().sp_pulse"].to_numpy(float)
                    * MM_PER_PULSE
                )
                ds_mm = (right_mm + left_mm) / 2.0
                dtheta = (right_mm - left_mm) / tread_mm
                local_x, local_y, local_theta = integrate_odometry(ds_mm, dtheta)
                body_lateral = direction * float(local_x[-1])
                body_forward = float(local_y[-1])
                turn_angle = direction * float(local_theta[-1])
                start_heading = math.radians(start_deg)
                body_lateral, body_forward = rotate_from_body(
                    body_lateral, body_forward, start_heading
                )
                end_heading = start_heading + turn_angle
                matrix = np.array([
                    [math.sin(start_heading), math.sin(end_heading)],
                    [math.cos(start_heading), math.cos(end_heading)],
                ])
                rhs = np.array([
                    target_lateral - body_lateral,
                    target_forward - body_forward,
                ])
                determinant = float(np.linalg.det(matrix))
                lengths = np.linalg.solve(matrix, rhs)
                feasible = bool(
                    abs(determinant) >= 0.10
                    and lengths[0] >= 0.0
                    and lengths[1] >= 0.0
                )
                rows.append({
                    "motion": motion,
                    "side": side,
                    "source": csv_path.name,
                    "turn_index": turn_index,
                    "tread_mm": tread_mm,
                    "turn_angle_deg": math.degrees(turn_angle),
                    "body_lateral_mm": body_lateral,
                    "body_forward_mm": body_forward,
                    "lstart_mm": float(lengths[0]),
                    "lend_mm": float(lengths[1]),
                    "matrix_determinant": determinant,
                    "feasible_nonnegative": feasible,
                })
    return pd.DataFrame(rows)


def summarize(frame: pd.DataFrame) -> tuple[pd.DataFrame, pd.DataFrame]:
    by_side = frame.groupby(["motion", "side"], sort=False).agg(
        turns=("turn_index", "count"),
        angle_deg=("turn_angle_deg", "mean"),
        body_lateral_mm=("body_lateral_mm", "mean"),
        body_forward_mm=("body_forward_mm", "mean"),
        lstart_mm=("lstart_mm", "mean"),
        lend_mm=("lend_mm", "mean"),
        lstart_std_mm=("lstart_mm", "std"),
        lend_std_mm=("lend_mm", "std"),
        feasible=("feasible_nonnegative", "all"),
    ).reset_index()
    by_side[["lstart_std_mm", "lend_std_mm"]] = by_side[[
        "lstart_std_mm", "lend_std_mm"
    ]].fillna(0.0)

    adopted_rows = []
    for motion, group in by_side.groupby("motion", sort=False):
        feasible = bool(group["feasible"].all())
        adopted_rows.append({
            "motion": motion,
            "tread_mm": float(frame["tread_mm"].iloc[0]),
            "lstart_mm": float(group["lstart_mm"].mean()) if feasible else math.nan,
            "lend_mm": float(group["lend_mm"].mean()) if feasible else math.nan,
            "right_lstart_mm": float(group.loc[group["side"] == "R", "lstart_mm"].iloc[0]),
            "left_lstart_mm": float(group.loc[group["side"] == "L", "lstart_mm"].iloc[0]),
            "right_lend_mm": float(group.loc[group["side"] == "R", "lend_mm"].iloc[0]),
            "left_lend_mm": float(group.loc[group["side"] == "L", "lend_mm"].iloc[0]),
            "feasible_nonnegative": feasible,
        })
    return by_side, pd.DataFrame(adopted_rows)


def write_report(by_side: pd.DataFrame, adopted: pd.DataFrame, output: Path) -> None:
    lines = [
        "# Encoder-only 1600 mm/s turn lengths", "",
        f"- effective tread: {adopted['tread_mm'].iloc[0]:.1f} mm",
        "- odometry inputs: right/left encoder pulses only",
        "- `ideal.rad_velo` is used only to locate the commanded turn interval",
        "- `ego.velo`, gyro omega, beta, and acceleration are not integrated", "",
        "| motion | side | turns | encoder angle [deg] | body end (lat, fwd) [mm] | Lstart [mm] | Lend [mm] | feasible |",
        "| --- | :---: | ---: | ---: | ---: | ---: | ---: | :---: |",
    ]
    for row in by_side.itertuples():
        lines.append(
            f"| {row.motion} | {row.side} | {row.turns} | {row.angle_deg:.3f} | "
            f"({row.body_lateral_mm:.2f}, {row.body_forward_mm:.2f}) | "
            f"{row.lstart_mm:.2f} | {row.lend_mm:.2f} | "
            f"{'yes' if row.feasible else 'no'} |"
        )
    lines.extend(("", "## R/L mean adopted values", "", "| motion | Lstart [mm] | Lend [mm] |", "| --- | ---: | ---: |"))
    for row in adopted.itertuples():
        if row.feasible_nonnegative:
            lines.append(f"| {row.motion} | {row.lstart_mm:.2f} | {row.lend_mm:.2f} |")
        else:
            lines.append(f"| {row.motion} | infeasible | infeasible |")
    lines.extend((
        "", "long180 is infeasible with non-negative straight lengths: the encoder-only turn body already ends at about 96 mm lateral, beyond the 90 mm target, while its exit heading is slightly below 180 degrees.", "",
    ))
    output.write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tread-mm", type=float, default=35.1)
    parser.add_argument(
        "--output", type=Path,
        default=ROOT / "tools" / "turn_analysis" / "1600" / "encoder_only_tread35_1",
    )
    args = parser.parse_args()
    frame = calculate(args.tread_mm)
    by_side, adopted = summarize(frame)
    args.output.mkdir(parents=True, exist_ok=True)
    frame.to_csv(args.output / "turns.csv", index=False)
    by_side.to_csv(args.output / "by_side.csv", index=False)
    adopted.to_csv(args.output / "adopted_lengths.csv", index=False)
    write_report(by_side, adopted, args.output / "README.md")
    print(adopted.to_string(index=False))


if __name__ == "__main__":
    main()
