"""Render a top-down maze crop with a cumulative marker-derived body trail."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import pandas as pd

from video_marker_pose import detect_aruco_quad


GRID_MM = 360.0


def world_to_pixel(points: np.ndarray, size: int) -> np.ndarray:
    result = np.asarray(points, dtype=float).copy()
    result[:, 0] *= (size - 1) / GRID_MM
    result[:, 1] = (GRID_MM - result[:, 1]) * (size - 1) / GRID_MM
    return np.round(result).astype(int)


def load_trimmed_frames(video: Path, start: int, end: int) -> tuple[list[np.ndarray], float]:
    cap = cv2.VideoCapture(str(video))
    if not cap.isOpened():
        raise RuntimeError(f"cannot open video: {video}")
    fps = float(cap.get(cv2.CAP_PROP_FPS))
    cap.set(cv2.CAP_PROP_POS_FRAMES, start)
    frames = []
    for _ in range(start, end + 1):
        ok, frame = cap.read()
        if not ok:
            break
        frames.append(frame)
    cap.release()
    expected = end - start + 1
    if len(frames) != expected:
        raise RuntimeError(f"expected {expected} frames, read {len(frames)}")
    return frames, fps


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--video", type=Path, required=True)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--size", type=int, default=900)
    args = parser.parse_args()

    metadata = json.loads(
        (args.run_dir / "trim_metadata.json").read_text(encoding="utf-8")
    )
    pose = pd.read_csv(args.run_dir / "video_marker_pose.csv")
    frames, fps = load_trimmed_frames(
        args.video,
        int(metadata["trim_start_frame"]),
        int(metadata["trim_end_frame"]),
    )
    if len(frames) != len(pose):
        raise RuntimeError(f"frame/pose mismatch: {len(frames)} != {len(pose)}")

    quads = [detect_aruco_quad(frame) for frame in frames]
    valid = np.asarray([quad for quad in quads if quad is not None])
    if not len(valid):
        raise RuntimeError("four-ArUco maze calibration was not detected")
    source_quad = np.median(valid, axis=0).astype(np.float32)
    n = args.size - 1
    # Source order corresponds to world (0,360), (360,360), (0,0), (360,0).
    destination_quad = np.float32([[0, 0], [n, 0], [0, n], [n, n]])
    warp = cv2.getPerspectiveTransform(source_quad, destination_quad)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        str(args.output), cv2.VideoWriter_fourcc(*"mp4v"), fps,
        (args.size, args.size),
    )
    if not writer.isOpened():
        raise RuntimeError(f"cannot create output: {args.output}")

    centers = pose[["center_x_mm", "center_y_mm"]].to_numpy(float)
    fronts = pose[["blue_x_mm", "blue_y_mm"]].to_numpy(float)
    rears = pose[["yellow_x_mm", "yellow_y_mm"]].to_numpy(float)
    center_px = world_to_pixel(centers, args.size)
    front_px = world_to_pixel(fronts, args.size)
    rear_px = world_to_pixel(rears, args.size)
    led_active = pose["turn_led_active"].to_numpy(bool)

    for index, frame in enumerate(frames):
        canvas = cv2.warpPerspective(
            frame, warp, (args.size, args.size),
            flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT,
        )
        if index:
            cv2.polylines(
                canvas, [center_px[:index + 1]], False,
                (220, 40, 220), 7, cv2.LINE_AA,
            )
        centre = tuple(center_px[index])
        front = tuple(front_px[index])
        rear = tuple(rear_px[index])
        cv2.line(canvas, rear, front, (255, 255, 255), 3, cv2.LINE_AA)
        cv2.circle(canvas, front, 9, (20, 20, 230), -1, cv2.LINE_AA)
        cv2.circle(canvas, rear, 9, (0, 220, 255), -1, cv2.LINE_AA)
        cv2.circle(canvas, centre, 12, (20, 20, 20), 5, cv2.LINE_AA)
        cv2.circle(canvas, centre, 12, (255, 255, 0), 4, cv2.LINE_AA)

        cv2.rectangle(canvas, (14, 14), (500, 114), (18, 18, 18), -1)
        cv2.putText(
            canvas, "RED/YELLOW MIDPOINT = BODY CENTER", (30, 48),
            cv2.FONT_HERSHEY_SIMPLEX, .66, (255, 255, 255), 2, cv2.LINE_AA,
        )
        cv2.putText(
            canvas, "MAGENTA = CUMULATIVE CENTER TRAIL", (30, 78),
            cv2.FONT_HERSHEY_SIMPLEX, .62, (220, 80, 220), 2, cv2.LINE_AA,
        )
        state = "TURN LED ON" if led_active[index] else "TURN LED OFF"
        cv2.putText(
            canvas, f"{index / fps:5.2f} s   {state}", (30, 104),
            cv2.FONT_HERSHEY_SIMPLEX, .58,
            (255, 255, 255), 2, cv2.LINE_AA,
        )
        writer.write(canvas)
    writer.release()
    print(
        f"wrote {args.output} frames={len(frames)} fps={fps:.6f} "
        f"aruco_complete={len(valid)}/{len(frames)}"
    )


if __name__ == "__main__":
    main()
