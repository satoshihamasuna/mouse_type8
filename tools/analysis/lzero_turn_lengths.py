"""Determine 1600 mm/s Lstart/Lend from Lstart=Lend=0 videos and logs.

The Pixel videos are slow-motion recordings.  Coloured body markers provide
the physical chassis trajectory, while the corresponding controller log
provides the angular-profile boundaries used to cut that trajectory.
"""
from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[2]
VIDEO_ROOT = ROOT.parent / "video3"
OUTPUT = ROOT / "tools" / "turn_analysis" / "1600" / "video_20260724_lzero"
DEFAULT_MANIFEST = OUTPUT / "manifest.csv"
MANIFEST_COLUMNS = (
    "video",
    "log_stem",
    "motion",
    "target_lateral_mm",
    "target_forward_mm",
)

START_HEADING_DEG = {
    "long_r90": 0.0,
    "long_r180": 0.0,
    "in_r45": 0.0,
    "in_r135": 0.0,
    "out_r45": -45.0,
    "out_r135": -45.0,
    "r_v90": -45.0,
}


def read_manifest(path: Path) -> list[tuple[str, str, str, float, float]]:
    """Read and validate video/log/target mappings."""
    frame = pd.read_csv(path)
    missing = [column for column in MANIFEST_COLUMNS if column not in frame.columns]
    if missing:
        raise ValueError(f"manifest is missing columns: {', '.join(missing)}")
    if frame.empty:
        raise ValueError("manifest has no runs")
    if frame["log_stem"].duplicated().any():
        duplicates = frame.loc[frame["log_stem"].duplicated(), "log_stem"].tolist()
        raise ValueError(f"duplicate log_stem values: {duplicates}")
    unsupported = sorted(set(frame["motion"]) - set(START_HEADING_DEG))
    if unsupported:
        raise ValueError(f"unsupported motions: {unsupported}")

    runs = []
    for row in frame.itertuples(index=False):
        video = str(row.video)
        log_stem = str(row.log_stem)
        video_path = VIDEO_ROOT / video
        log_path = ROOT / "tools" / "logs" / f"{log_stem}.csv"
        settings_path = ROOT / "tools" / "logs" / f"{log_stem}.settings.json"
        if not video_path.is_file():
            raise FileNotFoundError(f"video not found: {video_path}")
        if not log_path.is_file():
            raise FileNotFoundError(f"log not found: {log_path}")
        if settings_path.is_file():
            settings = json.loads(settings_path.read_text(encoding="utf-8"))
            parameters = settings.get("parameters", {})
            if float(parameters.get("lstart", math.nan)) != 0.0:
                raise ValueError(f"Lstart is not zero: {settings_path}")
            if float(parameters.get("lend", math.nan)) != 0.0:
                raise ValueError(f"Lend is not zero: {settings_path}")
        runs.append(
            (
                video,
                log_stem,
                str(row.motion),
                float(row.target_lateral_mm),
                float(row.target_forward_mm),
            )
        )
    return runs


def target_in_start_frame(
    motion: str, lateral_mm: float, forward_mm: float
) -> tuple[float, float]:
    """Convert the user's board coordinates to initial forward/left axes."""
    theta = math.radians(START_HEADING_DEG[motion])
    target_forward = lateral_mm * math.sin(theta) + forward_mm * math.cos(theta)
    target_left = lateral_mm * math.cos(theta) - forward_mm * math.sin(theta)
    return target_left, target_forward


def run_tools(item: tuple[str, str, str, float, float], force: bool) -> Path:
    video_name, log_stem, motion, _, _ = item
    run_dir = OUTPUT / log_stem[:15] / motion
    metrics = run_dir / "metrics.json"
    if metrics.exists() and not force:
        return metrics
    run_dir.mkdir(parents=True, exist_ok=True)
    clip = run_dir / "trim.mp4"
    if not clip.exists():
        subprocess.run(
            [
                sys.executable, str(ROOT / "tools/analysis/trim_motion_video.py"),
                "--video", str(VIDEO_ROOT / video_name),
                "--output", str(clip),
                "--metadata", str(run_dir / "trim_metadata.json"),
                "--front-color", "red",
            ],
            check=True,
        )
    subprocess.run(
        [
            sys.executable, str(ROOT / "tools/analysis/video_marker_pose.py"),
            "--video", str(clip),
            "--log", str(ROOT / "tools/logs" / f"{log_stem}.csv"),
            "--output", str(run_dir),
            "--front-color", "red",
            "--calibration-video", str(VIDEO_ROOT / video_name),
        ],
        check=True,
    )
    clip.unlink()
    return metrics


def interp_xy(pose: pd.DataFrame, time_s: float) -> np.ndarray:
    time = pose["time_video_s"].to_numpy(float)
    return np.array(
        [np.interp(time_s, time, pose[column].to_numpy(float))
         for column in ("center_x_mm", "center_y_mm")]
    )


def solve_lengths(
    displacement: np.ndarray,
    body_points: np.ndarray,
    final_heading: float,
    target_lateral: float,
    target_forward: float,
    long180: bool,
) -> tuple[float, float, float, float]:
    """Solve in local (forward, left) coordinates."""
    target = np.array([target_forward, target_lateral])
    final = np.array([math.cos(final_heading), math.sin(final_heading)])
    if long180:
        apex = float(np.max(body_points[:, 0]))
        lstart = 95.0 - apex
        remainder = target - (np.array([lstart, 0.0]) + displacement)
        lend = float(remainder @ final)
        residual = float(np.linalg.norm(remainder - lend * final))
        return lstart, lend, residual, apex
    basis = np.column_stack((np.array([1.0, 0.0]), final))
    lengths = np.linalg.solve(basis, target - displacement)
    return float(lengths[0]), float(lengths[1]), 0.0, float("nan")


def video_result(metrics_path: Path, target_x: float, target_y: float) -> dict:
    metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
    pose = pd.read_csv(metrics_path.with_name("video_marker_pose.csv"))
    start_s = float(metrics.get(
        "profile_video_start_s",
        float(metrics["log_time_zero_at_video_s"])
        + .001 * float(metrics["log_omega_start_frame"])
        * float(metrics["slow_motion_factor"]),
    ))
    end_s = float(metrics.get(
        "profile_video_end_s",
        float(metrics["log_time_zero_at_video_s"])
        + .001 * float(metrics["log_omega_end_frame"])
        * float(metrics["slow_motion_factor"]),
    ))
    xy_start = interp_xy(pose, start_s)
    xy_end = interp_xy(pose, end_s)

    heading = np.unwrap(pose["heading_rad"].to_numpy(float))
    sample_times = pose["time_video_s"].to_numpy(float)
    start_index = int(np.argmin(np.abs(sample_times - start_s)))
    end_index = int(np.argmin(np.abs(sample_times - end_s)))
    h0 = float(np.median(
        heading[max(0, start_index - 3):min(len(heading), start_index + 4)]
    ))
    marker_h1 = float(np.median(
        heading[max(0, end_index - 3):min(len(heading), end_index + 4)]
    ))
    initial = np.array([math.cos(h0), math.sin(h0)])
    left = np.array([-initial[1], initial[0]])
    displacement_world = xy_end - xy_start
    displacement = np.array(
        [displacement_world @ initial, displacement_world @ left]
    )
    keep = (sample_times >= start_s) & (sample_times <= end_s)
    points_world = pose.loc[keep, ["center_x_mm", "center_y_mm"]].to_numpy(float)
    points_delta = points_world - xy_start
    points = np.column_stack((points_delta @ initial, points_delta @ left))
    # The marker positions supply the ground trajectory; the log supplies the
    # controlled exit heading, which is much less noisy than a blurred marker
    # at the angular-profile boundary.
    final_heading = math.radians(float(metrics["log_turn_angle_deg"]))
    local_x, local_y = target_in_start_frame(
        str(metrics["motion_name"]), target_x, target_y
    )
    lstart, lend, residual, apex = solve_lengths(
        displacement, points, final_heading, local_x, local_y,
        metrics["motion_name"] == "long_r180",
    )
    return {
        "video_lstart_mm": lstart,
        "video_lend_mm": lend,
        "video_body_forward_mm": displacement[0],
        "video_body_lateral_mm": displacement[1],
        "video_marker_turn_angle_deg": math.degrees(marker_h1 - h0),
        "video_turn_angle_deg": math.degrees(final_heading),
        "video_target_residual_mm": residual,
        "video_body_apex_forward_mm": apex,
    }


def log_result(log_path: Path, motion: str, target_x: float, target_y: float) -> dict:
    frame = pd.read_csv(log_path)
    omega = frame["ideal.rad_velo"].to_numpy(float)
    active = np.flatnonzero(np.abs(omega) > .03)
    cuts = np.flatnonzero(np.diff(active) > 1) + 1
    group = max(np.split(active, cuts), key=lambda g: np.max(np.abs(omega[g])))
    start, end = int(group[0]), int(group[-1])
    lateral = frame["ego.turn_x"].to_numpy(float)
    forward = frame["ego.turn_y"].to_numpy(float)
    points = np.column_stack((
        forward[start:end + 1] - forward[start],
        lateral[start:end + 1] - lateral[start],
    ))
    displacement = points[-1]
    angle = float(frame["ego.radian"].iloc[end] - frame["ego.radian"].iloc[start])
    local_x, local_y = target_in_start_frame(motion, target_x, target_y)
    lstart, lend, residual, apex = solve_lengths(
        displacement, points, angle, local_x, local_y, motion == "long_r180"
    )
    return {
        "log_lstart_mm": lstart,
        "log_lend_mm": lend,
        "log_body_forward_mm": displacement[0],
        "log_body_lateral_mm": displacement[1],
        "log_turn_angle_deg": math.degrees(angle),
        "log_target_residual_mm": residual,
        "log_body_apex_forward_mm": apex,
    }


def write_report(runs: pd.DataFrame, summary: pd.DataFrame) -> None:
    lines = [
        "# Lstart=Lend=0 実測からの1600 mm/sターン長",
        "",
        "赤・黄マーカーの動画軌跡をログの角速度プロファイル開始・終了時刻で切り、"
        "指定された終点へ初期直進と後直進を加える連立方程式から求めた。"
        "long180だけは旋回軌跡の頂点を95 mmに置いてLstartを決め、終点からLendを決めた。",
        "",
        "| motion | 採用数 | 採用 Lstart [mm] | 採用 Lend [mm] | 動画の範囲 Lstart | 動画の範囲 Lend | ログ中央値 Lstart/Lend |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in summary.itertuples():
        lines.append(
            f"| {row.motion} | {row.accepted_runs}/{row.total_runs} | "
            f"{row.adopted_lstart_mm:.2f} | "
            f"{row.adopted_lend_mm:.2f} | {row.video_lstart_min_mm:.2f}–"
            f"{row.video_lstart_max_mm:.2f} | {row.video_lend_min_mm:.2f}–"
            f"{row.video_lend_max_mm:.2f} | {row.log_lstart_median_mm:.2f} / "
            f"{row.log_lend_median_mm:.2f} |"
        )
    lines += [
        "",
        "採用値は有効動画の中央値。検出率95%以上、スロー倍率7.5–9.0、"
        "動画―ログ軌跡RMSE 8 mm以下を有効とした。右旋回だけを撮影しているため、左右は同じ"
        "Lstart/Lendを使い、r_minとdegreeの符号だけを反転する前提とした。",
        "",
        "全ランの同期誤差・軌跡変位・旋回角は `run_results.csv` に保存した。",
    ]
    (OUTPUT / "README.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    global OUTPUT, VIDEO_ROOT
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--video-root", type=Path, default=VIDEO_ROOT)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--force-motion", action="append", default=[])
    parser.add_argument("--workers", type=int, default=3)
    args = parser.parse_args()
    OUTPUT = args.output.resolve()
    VIDEO_ROOT = args.video_root.resolve()
    manifest_path = args.manifest.resolve()
    runs_manifest = read_manifest(manifest_path)
    OUTPUT.mkdir(parents=True, exist_ok=True)
    if manifest_path.parent != OUTPUT:
        (OUTPUT / "manifest.csv").write_text(
            manifest_path.read_text(encoding="utf-8"), encoding="utf-8"
        )

    metric_paths: dict[str, Path] = {}
    with ThreadPoolExecutor(max_workers=max(1, args.workers)) as executor:
        futures = {
            executor.submit(
                run_tools, item, args.force or item[2] in args.force_motion
            ): item
            for item in runs_manifest
        }
        for future in as_completed(futures):
            item = futures[future]
            metric_paths[item[1]] = future.result()
            print(f"tracked {item[1]}", flush=True)

    rows = []
    for _, log_stem, motion, target_x, target_y in runs_manifest:
        metrics_path = metric_paths[log_stem]
        metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
        row = {
            "run": log_stem[:15],
            "motion": motion,
            "target_lateral_mm": target_x,
            "target_forward_mm": target_y,
            "marker_detection_pct": 100.0 * float(metrics["marker_detection_rate"]),
            "trajectory_fit_rmse_mm": float(metrics["trajectory_fit_rmse_mm"]),
            "slow_motion_factor": float(metrics["slow_motion_factor"]),
        }
        row.update(video_result(metrics_path, target_x, target_y))
        row.update(log_result(ROOT / "tools/logs" / f"{log_stem}.csv",
                              motion, target_x, target_y))
        rows.append(row)
    runs = pd.DataFrame(rows)
    runs["accepted"] = (
        (runs["marker_detection_pct"] >= 95.0)
        & runs["slow_motion_factor"].between(7.5, 9.0)
        & (runs["trajectory_fit_rmse_mm"] <= 8.0)
    )
    runs.to_csv(OUTPUT / "run_results.csv", index=False)

    summaries = []
    for motion, group in runs.groupby("motion", sort=False):
        accepted = group[group["accepted"]]
        if len(accepted) < 2:
            raise RuntimeError(
                f"{motion}: only {len(accepted)}/{len(group)} runs passed quality checks"
            )
        summaries.append({
            "motion": motion,
            "total_runs": len(group),
            "accepted_runs": len(accepted),
            "adopted_lstart_mm": float(accepted["video_lstart_mm"].median()),
            "adopted_lend_mm": float(accepted["video_lend_mm"].median()),
            "video_lstart_min_mm": float(accepted["video_lstart_mm"].min()),
            "video_lstart_max_mm": float(accepted["video_lstart_mm"].max()),
            "video_lend_min_mm": float(accepted["video_lend_mm"].min()),
            "video_lend_max_mm": float(accepted["video_lend_mm"].max()),
            "log_lstart_median_mm": float(group["log_lstart_mm"].median()),
            "log_lend_median_mm": float(group["log_lend_mm"].median()),
            "trajectory_fit_rmse_median_mm": float(
                group["trajectory_fit_rmse_mm"].median()
            ),
            "marker_detection_min_pct": float(accepted["marker_detection_pct"].min()),
        })
    summary = pd.DataFrame(summaries)
    summary.to_csv(OUTPUT / "adopted_summary.csv", index=False)
    write_report(runs, summary)
    print(summary.to_string(index=False))


if __name__ == "__main__":
    main()
