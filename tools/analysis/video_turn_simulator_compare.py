"""LEGACY: compare the blue-LED turn segment with tools/turn_simulator.py.

The indicator in myshell encloses the complete Init_Motion_* call: Lstart,
the angular profile, and Lend.  This script puts the detected LED-on video
trajectory and the simulator trajectory in the same start-pose coordinate
system.  Endpoint error is projected onto the simulator exit tangent:

    positive  -> the video turn segment is short by this many millimetres
    negative  -> the video turn segment travels too far

New Lstart/Lend tuning should use lzero_turn_lengths.py.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[2]
TOOLS = ROOT / "tools"
sys.path.insert(0, str(TOOLS))
sys.path.insert(0, str(TOOLS / "analysis"))

from turn_simulator import PRESETS, simulate_turn  # noqa: E402
from turn1600_variable_speed_lengths import _full_trajectory  # noqa: E402


MOTION_MAP = {
    "long_r90": "long_turn90",
    "long_r180": "long_turn180",
    "in_r45": "turn_in45",
    "in_r135": "turn_in135",
    "out_r45": "turn_out45",
    "out_r135": "turn_out135",
    "r_v90": "turn_v90",
}


def first_nonzero_tangent(points: np.ndarray, reverse: bool = False) -> np.ndarray:
    delta = np.diff(points, axis=0)
    indices = range(len(delta) - 1, -1, -1) if reverse else range(len(delta))
    for index in indices:
        norm = float(np.linalg.norm(delta[index]))
        if norm > 1.0e-9:
            return delta[index] / norm
    raise ValueError("trajectory has no non-zero tangent")


def simulator_trajectory(settings: dict, motion: str) -> np.ndarray:
    preset = PRESETS["turn1600"]
    sim_motion = MOTION_MAP[motion]
    result = simulate_turn(
        sim_motion,
        float(settings["parameters"]["velo"]),
        preset.motor,
        abs(float(settings["parameters"]["r_min"])),
        float(settings["pid"]["sp_kp"]),
        float(settings["pid"]["sp_ki"]),
        preset.alpha,
        preset.dynamics,
    )
    lateral, forward, _, _ = _full_trajectory(
        result,
        "R",
        float(settings["parameters"]["lstart"]),
        float(settings["parameters"]["lend"]),
        ideal=False,
    )
    return np.column_stack((lateral, forward))


def normalize_simulator(points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    origin = points[0]
    initial = first_nonzero_tangent(points)
    left = np.array((-initial[1], initial[0]))
    delta = points - origin
    local = np.column_stack((delta @ initial, delta @ left))
    final_global = first_nonzero_tangent(points, reverse=True)
    final_local = np.array((final_global @ initial, final_global @ left))
    return local, final_local / np.linalg.norm(final_local)


def normalize_video(pose: pd.DataFrame, start: int, end: int) -> np.ndarray:
    segment = pose.loc[start:end].copy()
    for column in ("center_x_mm", "center_y_mm", "heading_rad"):
        segment[column] = pd.to_numeric(segment[column], errors="coerce").interpolate(
            limit_direction="both"
        )
    segment = segment.dropna(subset=["center_x_mm", "center_y_mm", "heading_rad"])
    if len(segment) < 3:
        raise ValueError("LED-on segment has fewer than three tracked positions")

    boundary = segment.iloc[: min(3, len(segment))]
    origin = boundary[["center_x_mm", "center_y_mm"]].mean().to_numpy(float)
    heading = np.arctan2(
        np.sin(boundary["heading_rad"]).mean(),
        np.cos(boundary["heading_rad"]).mean(),
    )
    initial = np.array((math.cos(heading), math.sin(heading)))
    left = np.array((-initial[1], initial[0]))
    delta = segment[["center_x_mm", "center_y_mm"]].to_numpy(float) - origin
    return np.column_stack((delta @ initial, delta @ left))


def infer_motion(metrics: dict) -> str:
    motion = str(metrics.get("motion_name") or "")
    if motion:
        return motion
    log = str(metrics["log"])
    for candidate in MOTION_MAP:
        if candidate in log:
            return candidate
    raise ValueError(f"cannot infer motion from {log}")


def analyse(metrics_path: Path, output: Path) -> dict:
    metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
    motion = infer_motion(metrics)
    if motion not in MOTION_MAP:
        raise ValueError(f"unsupported motion {motion}")

    pose_path = metrics_path.with_name("video_marker_pose.csv")
    pose = pd.read_csv(pose_path)
    active = pose["turn_led_active"].astype(str).str.lower().eq("true")
    active_indices = np.flatnonzero(active.to_numpy())
    if len(active_indices) < 3:
        raise ValueError("no usable LED-on interval")
    led_start, led_end = int(active_indices[0]), int(active_indices[-1])

    setting_name = str(metrics["log"]).replace(".csv", ".settings.json")
    setting_path = ROOT / "tools" / "logs" / setting_name
    settings = json.loads(setting_path.read_text(encoding="utf-8"))

    video = normalize_video(pose, led_start, led_end)
    simulator, exit_tangent = normalize_simulator(
        simulator_trajectory(settings, motion)
    )
    endpoint_delta = simulator[-1] - video[-1]
    exit_left = np.array((-exit_tangent[1], exit_tangent[0]))
    signed_shortfall = float(endpoint_delta @ exit_tangent)
    cross_error = float(endpoint_delta @ exit_left)

    run = str(metrics["log"])[:15]
    figure, axis = plt.subplots(figsize=(6.8, 6.0), constrained_layout=True)
    axis.plot(simulator[:, 0], simulator[:, 1], color="#e64b35", linewidth=2.2,
              label="turn simulator (JSON Lstart/Lend)")
    axis.plot(video[:, 0], video[:, 1], color="#ca22cf", linewidth=3.0,
              label="video: blue LED on")
    axis.scatter(*simulator[-1], marker="x", color="#e64b35", s=75, zorder=4)
    axis.scatter(*video[-1], marker="o", color="#ca22cf", s=45, zorder=4)
    axis.annotate(
        f"exit shortfall = {signed_shortfall:+.1f} mm\n"
        f"cross error = {cross_error:+.1f} mm",
        xy=video[-1],
        xytext=(12, 12),
        textcoords="offset points",
        bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.88},
    )
    axis.set_title(f"{run} {motion}: LED-on turn vs simulator")
    axis.set_xlabel("initial forward [mm]")
    axis.set_ylabel("initial left [mm]")
    axis.grid(True, alpha=0.3)
    axis.axis("equal")
    axis.legend(loc="best")
    figure.savefig(output / f"{run}_{motion}.png", dpi=180)
    plt.close(figure)

    parameters = settings["parameters"]
    return {
        "run": run,
        "motion": motion,
        "settings_json": setting_name,
        "front_marker_color": metrics.get("front_marker_color", "blue"),
        "marker_detection_pct": 100.0 * float(metrics["marker_detection_rate"]),
        "led_start_frame": led_start,
        "led_end_frame": led_end,
        "r_min_setting_mm": float(parameters["r_min"]),
        "lstart_setting_mm": float(parameters["lstart"]),
        "lend_setting_mm": float(parameters["lend"]),
        "simulator_endpoint_forward_mm": float(simulator[-1, 0]),
        "simulator_endpoint_left_mm": float(simulator[-1, 1]),
        "video_led_endpoint_forward_mm": float(video[-1, 0]),
        "video_led_endpoint_left_mm": float(video[-1, 1]),
        "signed_rear_shortfall_mm": signed_shortfall,
        "rear_shortfall_mm": max(0.0, signed_shortfall),
        "rear_overshoot_mm": max(0.0, -signed_shortfall),
        "cross_track_error_mm": cross_error,
        "boundary_resolution_mm": float(metrics["rear_boundary_resolution_mm"]),
    }


def select_adopted(frame: pd.DataFrame) -> pd.DataFrame:
    adopted = []
    for motion, group in frame.groupby("motion", sort=False):
        if motion == "in_r135":
            red = group[group["front_marker_color"] == "red"]
            chosen = red.iloc[-1] if len(red) else group.iloc[-1]
            basis = "latest red-marker run" if len(red) else "latest run"
        elif motion == "long_r180" and len(group) > 1:
            chosen = group.iloc[
                np.argsort(group["signed_rear_shortfall_mm"].to_numpy())[
                    len(group) // 2
                ]
            ]
            basis = f"median of {len(group)} runs"
        else:
            chosen = group.iloc[-1]
            basis = "one run"
        adopted.append({
            "motion": motion,
            "settings_json": chosen["settings_json"],
            "lstart_setting_mm": chosen["lstart_setting_mm"],
            "lend_setting_mm": chosen["lend_setting_mm"],
            "signed_rear_shortfall_mm": chosen["signed_rear_shortfall_mm"],
            "rear_shortfall_mm": chosen["rear_shortfall_mm"],
            "rear_overshoot_mm": chosen["rear_overshoot_mm"],
            "cross_track_error_mm": chosen["cross_track_error_mm"],
            "boundary_resolution_mm": chosen["boundary_resolution_mm"],
            "basis": basis,
        })
    return pd.DataFrame(adopted)


def write_report(adopted: pd.DataFrame, output: Path) -> None:
    lines = [
        "# LED点灯ターンとturnシミュレータの比較",
        "",
        "青色LEDが点灯している `Init_Motion_*` 全区間（Lstart、角速度プロファイル、",
        "Lend）を動画から抽出し、対応する設定JSONで `tools/turn_simulator.py` を",
        "実行した軌道と開始位置・開始方位を揃えて比較した。",
        "",
        "| motion | JSON Lstart [mm] | JSON Lend [mm] | 後距離不足 [mm] | 過走 [mm] | 横誤差 [mm] |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for row in adopted.itertuples():
        lines.append(
            f"| {row.motion} | {row.lstart_setting_mm:.2f} | "
            f"{row.lend_setting_mm:.2f} | {row.rear_shortfall_mm:.2f} | "
            f"{row.rear_overshoot_mm:.2f} | {row.cross_track_error_mm:+.2f} |"
        )
    lines += [
        "",
        "後距離不足は、シミュレータ終点と動画LED消灯点との差をシミュレータの",
        "退出接線方向へ射影した値。正なら動画側が手前、負の成分は過走として分けた。",
        "境界は動画1フレームの影響を受けるため、約3 mmの分解能がある。",
        "",
        "全走行の数値は [run_results.csv](run_results.csv)、採用値は",
        "[adopted_summary.csv](adopted_summary.csv) に保存した。",
    ]
    (output / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--root",
        type=Path,
        default=ROOT / "tools" / "turn_analysis" / "1600",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "tools" / "turn_analysis" / "1600"
        / "video_turn_simulator_comparison",
    )
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    rows = []
    for metrics_path in sorted(args.root.rglob("metrics.json")):
        if metrics_path.parent == args.output:
            continue
        if not metrics_path.with_name("video_marker_pose.csv").exists():
            continue
        metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
        if "led_start_frame" not in metrics:
            continue
        rows.append(analyse(metrics_path, args.output))

    frame = pd.DataFrame(rows).sort_values(["run", "motion"])
    frame.to_csv(args.output / "run_results.csv", index=False)
    adopted = select_adopted(frame)
    adopted.to_csv(args.output / "adopted_summary.csv", index=False)
    write_report(adopted, args.output)
    print(adopted.to_string(index=False))


if __name__ == "__main__":
    main()
