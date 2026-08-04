"""Add angular-profile start/end points to saved trajectory overlays.

This works from the files retained below tools/turn_analysis; the source MP4
and run log are not required.  Existing trajectory_overlay.jpg files are never
overwritten.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import pandas as pd

from video_marker_pose import ARUCO_INNER_CORNERS, WORLD_CORNERS


OUTPUT_NAME = "trajectory_overlay_profile_points.jpg"
PROFILE_COLOUR = (255, 255, 0)  # cyan in BGR
START_COLOUR = (255, 200, 0)
END_COLOUR = (0, 165, 255)


def marker_points(image: np.ndarray) -> dict[int, np.ndarray]:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    corners, ids, _ = cv2.aruco.ArucoDetector(dictionary).detectMarkers(image)
    if ids is None:
        return {}
    found = {
        int(marker_id): quad.reshape(4, 2)
        for marker_id, quad in zip(ids.ravel(), corners)
    }
    return {
        marker_id: found[marker_id][corner]
        for marker_id, corner in ARUCO_INNER_CORNERS.items()
        if marker_id in found
    }


def complete_quad(points: dict[int, np.ndarray]) -> np.ndarray | None:
    """Return the four field corners; infer one missing corner as a parallelogram."""
    order = list(ARUCO_INNER_CORNERS)
    indexed = {order.index(marker_id): point for marker_id, point in points.items()}
    if len(indexed) == 4:
        return np.float32([indexed[i] for i in range(4)])
    if len(indexed) != 3:
        return None
    # Layout: 0--1
    #         |  |
    #         2--3
    formulas = {
        0: (1, 2, 3),
        1: (0, 3, 2),
        2: (0, 3, 1),
        3: (1, 2, 0),
    }
    missing = next(i for i in range(4) if i not in indexed)
    a, b, subtract = formulas[missing]
    indexed[missing] = indexed[a] + indexed[b] - indexed[subtract]
    return np.float32([indexed[i] for i in range(4)])


def profile_times(metrics: dict) -> tuple[float, float]:
    if "profile_video_start_s" in metrics and "profile_video_end_s" in metrics:
        return (
            float(metrics["profile_video_start_s"]),
            float(metrics["profile_video_end_s"]),
        )
    zero = float(metrics["log_time_zero_at_video_s"])
    factor = float(metrics["slow_motion_factor"])
    return (
        zero + 0.001 * float(metrics["log_omega_start_frame"]) * factor,
        zero + 0.001 * float(metrics["log_omega_end_frame"]) * factor,
    )


def interpolate_world(pose: pd.DataFrame, time_s: float) -> np.ndarray:
    times = pose["time_video_s"].to_numpy(float)
    return np.array(
        [
            np.interp(time_s, times, pose["center_x_mm"].to_numpy(float)),
            np.interp(time_s, times, pose["center_y_mm"].to_numpy(float)),
        ],
        dtype=np.float32,
    )


def nearest_coloured_path(image: np.ndarray, point: np.ndarray) -> np.ndarray:
    """Snap small calibration errors to the centre of the drawn video path."""
    b, g, r = cv2.split(image)
    magenta = (b > 150) & (r > 150) & (g < 120)
    green = (g > 140) & (b < 130) & (r < 130)
    mask = magenta | green
    ys, xs = np.nonzero(mask)
    if not len(xs):
        return point
    delta2 = (xs - point[0]) ** 2 + (ys - point[1]) ** 2
    nearest = int(np.argmin(delta2))
    if delta2[nearest] > 80.0**2:
        return point
    # Average nearby coloured pixels to land near the thick line's centre.
    close = (xs - xs[nearest]) ** 2 + (ys - ys[nearest]) ** 2 <= 7.0**2
    return np.array([np.mean(xs[close]), np.mean(ys[close])], dtype=np.float32)


def draw_label(
    image: np.ndarray, point: np.ndarray, text: str, colour: tuple[int, int, int]
) -> None:
    p = tuple(np.round(point).astype(int))
    cv2.circle(image, p, 15, (20, 20, 20), 7, cv2.LINE_AA)
    cv2.circle(image, p, 15, colour, 4, cv2.LINE_AA)
    origin = (p[0] + 20, p[1] - 15)
    cv2.putText(
        image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.65,
        (20, 20, 20), 5, cv2.LINE_AA,
    )
    cv2.putText(
        image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.65,
        colour, 2, cv2.LINE_AA,
    )


def annotate(run_dir: Path, fallback_quad: np.ndarray | None) -> tuple[bool, np.ndarray | None]:
    overlay_path = run_dir / "trajectory_overlay.jpg"
    metrics_path = run_dir / "metrics.json"
    pose_path = run_dir / "video_marker_pose.csv"
    if not (overlay_path.exists() and metrics_path.exists() and pose_path.exists()):
        return False, fallback_quad

    image = cv2.imread(str(overlay_path))
    points = marker_points(image)
    quad = complete_quad(points)
    used_fallback = quad is None
    if quad is None:
        quad = fallback_quad
    if quad is None:
        return False, fallback_quad

    metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
    pose = pd.read_csv(pose_path)
    start_s, end_s = profile_times(metrics)
    start_world = interpolate_world(pose, start_s)
    end_world = interpolate_world(pose, end_s)
    homography = cv2.getPerspectiveTransform(WORLD_CORNERS, quad)
    profile = pose[
        pose["time_video_s"].between(start_s, end_s)
    ][["center_x_mm", "center_y_mm"]].to_numpy(np.float32)
    profile = np.vstack([start_world, profile, end_world])
    profile_px = cv2.perspectiveTransform(
        profile.reshape(-1, 1, 2), homography
    ).reshape(-1, 2)
    start_px = nearest_coloured_path(image, profile_px[0])
    end_px = nearest_coloured_path(image, profile_px[-1])

    cv2.polylines(
        image, [np.round(profile_px).astype(int)], False,
        PROFILE_COLOUR, 4, cv2.LINE_AA,
    )
    draw_label(image, start_px, "OMEGA START", START_COLOUR)
    draw_label(image, end_px, "OMEGA END", END_COLOUR)
    note = (
        f"ANGULAR PROFILE {start_s:.3f} - {end_s:.3f} s"
        + ("  (CALIBRATION FROM SAME SET)" if used_fallback else "")
    )
    cv2.rectangle(image, (22, 196), (690, 238), (20, 20, 20), -1)
    cv2.putText(
        image, note, (42, 225), cv2.FONT_HERSHEY_SIMPLEX, 0.62,
        (255, 255, 255), 2, cv2.LINE_AA,
    )
    cv2.imwrite(str(run_dir / OUTPUT_NAME), image, [cv2.IMWRITE_JPEG_QUALITY, 94])
    return True, quad if len(points) >= 3 else fallback_quad


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path("tools/turn_analysis"))
    args = parser.parse_args()
    overlays = sorted(args.root.rglob("trajectory_overlay.jpg"))
    # Process each acquisition set together so an unobscured calibration can
    # serve the rare image where two ArUcos are hidden or unreadable.
    groups: dict[Path, list[Path]] = {}
    for overlay in overlays:
        groups.setdefault(overlay.parents[2], []).append(overlay.parent)
    written = failed = 0
    for run_dirs in groups.values():
        fallback = None
        # Prefer a directly detected calibration from this acquisition set.
        for run_dir in run_dirs:
            image = cv2.imread(str(run_dir / "trajectory_overlay.jpg"))
            candidate = complete_quad(marker_points(image))
            if candidate is not None:
                fallback = candidate
                break
        for run_dir in run_dirs:
            ok, candidate = annotate(run_dir, fallback)
            if candidate is not None:
                fallback = candidate
            written += int(ok)
            failed += int(not ok)
    print(f"written={written} failed={failed} output={OUTPUT_NAME}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
