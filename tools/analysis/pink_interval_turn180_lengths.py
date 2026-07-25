"""Calculate long180 Lstart/Lend from the saved LED-on (pink) video interval."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
import pandas as pd


def calculate(metrics_path: Path, quality: pd.Series) -> dict:
    metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
    pose = pd.read_csv(metrics_path.with_name("video_marker_pose.csv"))
    active = np.flatnonzero(pose["turn_led_active"].to_numpy(bool))
    if not len(active):
        raise RuntimeError(f"LED interval is missing: {metrics_path}")
    lo, hi = int(active[0]), int(active[-1])
    xy = pose[["center_x_mm", "center_y_mm"]].to_numpy(float)
    heading = np.unwrap(pose["heading_rad"].to_numpy(float))
    # Use the straight approach frames for a stable initial direction.
    h0 = float(np.median(heading[:20]))
    forward = np.array([math.cos(h0), math.sin(h0)])
    left = np.array([-forward[1], forward[0]])
    delta = xy[lo:hi + 1] - xy[lo]
    points = np.column_stack((delta @ forward, delta @ left))
    displacement = points[-1]
    apex = float(np.max(points[:, 0]))
    lstart = 95.0 - apex

    final_heading = math.radians(float(metrics["log_turn_angle_deg"]))
    final = np.array([math.cos(final_heading), math.sin(final_heading)])
    target = np.array([0.0, -90.0])  # initial-forward, initial-left
    remainder = target - (np.array([lstart, 0.0]) + displacement)
    lend = float(remainder @ final)
    residual = float(np.linalg.norm(remainder - lend * final))
    return {
        "speed_mm_s": int(quality["speed_mm_s"]),
        "run": metrics_path.parents[1].name,
        "led_start_frame": lo,
        "led_end_frame": hi,
        "pink_lstart_mm": lstart,
        "pink_lend_mm": lend,
        "pink_apex_forward_mm": apex,
        "pink_target_residual_mm": residual,
        "trajectory_fit_rmse_mm": float(quality["trajectory_fit_rmse_mm"]),
        "marker_detection_pct": float(quality["marker_detection_pct"]),
        "accepted": bool(quality["accepted"]),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path("tools/turn_analysis"))
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("tools/turn_analysis/video_20260726_long180_pink"),
    )
    args = parser.parse_args()
    rows = []
    for speed in (1400, 1600, 1800, 2000):
        run_root = args.root / str(speed) / "video_20260726_lzero"
        quality = pd.read_csv(run_root / "run_results.csv")
        quality["speed_mm_s"] = speed
        quality = quality.set_index("run")
        for metrics_path in sorted(run_root.rglob("metrics.json")):
            run = metrics_path.parents[1].name
            rows.append(calculate(metrics_path, quality.loc[run]))
    runs = pd.DataFrame(rows).sort_values(["speed_mm_s", "run"])
    accepted = runs[runs["accepted"]]
    summary = accepted.groupby("speed_mm_s", as_index=False).agg(
        total_accepted=("run", "size"),
        adopted_lstart_mm=("pink_lstart_mm", "median"),
        adopted_lend_mm=("pink_lend_mm", "median"),
        lstart_min_mm=("pink_lstart_mm", "min"),
        lstart_max_mm=("pink_lstart_mm", "max"),
        lend_min_mm=("pink_lend_mm", "min"),
        lend_max_mm=("pink_lend_mm", "max"),
        target_residual_median_mm=("pink_target_residual_mm", "median"),
    )
    args.output.mkdir(parents=True, exist_ok=True)
    runs.to_csv(args.output / "run_results.csv", index=False)
    summary.to_csv(args.output / "adopted_summary.csv", index=False)
    print(summary.to_string(index=False))


if __name__ == "__main__":
    main()
