"""Trim a marker-equipped micromouse video to motion with a time margin."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import pandas as pd

from video_marker_pose import (
    WORLD_CORNERS,
    colour_components,
    detect_aruco_quad,
    select_marker_pair,
    transform_points,
)


def scan_video(
    path: Path, sample_step: int = 3, front_colour: str = "blue"
) -> tuple[np.ndarray, np.ndarray, float]:
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise RuntimeError(f"cannot open video: {path}")
    fps = float(cap.get(cv2.CAP_PROP_FPS))
    points: list[np.ndarray] = []
    led_counts: list[int] = []
    homography = None
    inverse = None
    previous = None
    frame_index = 0
    while True:
        ok = cap.grab()
        if not ok:
            break
        if frame_index % sample_step:
            points.append(np.array([np.nan, np.nan]))
            led_counts.append(0)
            frame_index += 1
            continue
        ok, frame = cap.retrieve()
        if not ok:
            break
        if homography is None:
            quad = detect_aruco_quad(frame)
            if quad is not None:
                homography = cv2.getPerspectiveTransform(quad, WORLD_CORNERS)
                inverse = cv2.getPerspectiveTransform(WORLD_CORNERS, quad)
        pair = None
        if homography is not None:
            pair = select_marker_pair(
                colour_components(frame, front_colour),
                colour_components(frame, "yellow"),
                homography,
                previous,
            )
        if pair is None:
            points.append(np.array([np.nan, np.nan]))
            led_counts.append(0)
            frame_index += 1
            continue
        blue, yellow = pair
        centre = (blue + yellow) / 2.0
        previous = centre
        points.append(centre)

        centre_px, blue_px = transform_points(np.vstack([centre, blue]), inverse)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hue, sat, val = cv2.split(hsv)
        blue_light = (
            (hue >= 82) & (hue <= 142) & (sat >= 90) & (val >= 120)
        ).astype(np.uint8)
        roi = np.zeros(blue_light.shape, np.uint8)
        marker_span_px = float(np.linalg.norm(centre_px - blue_px)) * 2.0
        cv2.circle(
            roi, tuple(np.round(centre_px).astype(int)),
            max(20, int(round(marker_span_px * .70))), 1, -1,
        )
        cv2.circle(
            roi, tuple(np.round(blue_px).astype(int)),
            max(7, int(round(marker_span_px * .20))), 0, -1,
        )
        led_counts.append(int(np.count_nonzero(blue_light & roi)))
        frame_index += 1
    cap.release()
    points_array = np.asarray(points, float)
    for axis in range(2):
        points_array[:, axis] = (
            pd.Series(points_array[:, axis])
            .interpolate(limit=10, limit_direction="both")
            .rolling(5, center=True, min_periods=1)
            .median()
        )
    return points_array, np.asarray(led_counts), fps


def motion_window(
    points: np.ndarray, led_counts: np.ndarray, fps: float, margin_s: float
) -> tuple[int, int, dict[str, float | int]]:
    lag = max(2, int(round(.10 * fps)))
    displacement = np.linalg.norm(
        np.roll(points, -lag, axis=0) - np.roll(points, lag, axis=0), axis=1
    )
    speed = displacement / (2.0 * lag / fps)
    speed[~np.isfinite(speed)] = 0.0
    speed[:lag] = 0.0
    speed[-lag:] = 0.0
    speed = (
        pd.Series(speed).rolling(max(3, int(round(.20 * fps))), center=True, min_periods=1)
        .median().to_numpy()
    )
    moving = speed > 12.0
    # Bridge short slow portions inside one physical run.
    close_width = max(3, int(round(.35 * fps)))
    moving = (
        pd.Series(moving.astype(float))
        .rolling(close_width, center=True, min_periods=1).max()
        .rolling(close_width, center=True, min_periods=1).min()
        .fillna(0).to_numpy() > .5
    )
    indices = np.flatnonzero(moving)
    if not len(indices):
        raise RuntimeError("mouse motion was not detected")
    cuts = np.flatnonzero(np.diff(indices) > 1) + 1
    groups = np.split(indices, cuts)
    led_threshold = max(100.0, .08 * float(np.percentile(led_counts, 95)))
    led_active = led_counts > led_threshold

    def group_score(group: np.ndarray) -> tuple[float, float, float]:
        led_overlap = int(np.count_nonzero(led_active[group]))
        extent = float(
            np.ptp(points[group, 0]) + np.ptp(points[group, 1])
        )
        # The actual 1600 mm/s run crosses most of the calibrated field.
        # Setup motion can keep the blue LED on and can have a larger one-frame
        # speed peak, but its total displacement is much smaller.
        return extent, led_overlap, float(speed[group].max())

    run = max(groups, key=group_score)
    motion_start, motion_end = int(run[0]), int(run[-1])
    margin_frames = int(round(margin_s * fps))
    trim_start = max(0, motion_start - margin_frames)
    trim_end = min(len(points) - 1, motion_end + margin_frames)
    details = {
        "motion_start_frame": motion_start,
        "motion_end_frame": motion_end,
        "motion_start_s": motion_start / fps,
        "motion_end_s": motion_end / fps,
        "trim_start_frame": trim_start,
        "trim_end_frame": trim_end,
        "trim_start_s": trim_start / fps,
        "trim_end_s": trim_end / fps,
        "led_detection_threshold_pixels": led_threshold,
        "motion_peak_playback_mm_s": float(speed[run].max()),
    }
    return trim_start, trim_end, details


def write_clip(source: Path, destination: Path, start: int, end: int) -> int:
    cap = cv2.VideoCapture(str(source))
    fps = float(cap.get(cv2.CAP_PROP_FPS))
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    destination.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        str(destination), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height)
    )
    if not writer.isOpened():
        cap.release()
        raise RuntimeError(f"cannot create clip: {destination}")
    cap.set(cv2.CAP_PROP_POS_FRAMES, start)
    written = 0
    for _ in range(start, end + 1):
        ok, frame = cap.read()
        if not ok:
            break
        writer.write(frame)
        written += 1
    writer.release()
    cap.release()
    if written != end - start + 1:
        raise RuntimeError(f"clip was truncated: expected {end-start+1}, wrote {written}")
    return written


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--video", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--margin-s", type=float, default=1.0)
    parser.add_argument("--sample-step", type=int, default=3)
    parser.add_argument(
        "--front-color", choices=("blue", "red", "magenta"), default="blue"
    )
    parser.add_argument("--metadata", type=Path)
    args = parser.parse_args()
    points, led_counts, fps = scan_video(
        args.video, max(1, args.sample_step), args.front_color
    )
    start, end, details = motion_window(points, led_counts, fps, args.margin_s)
    written = write_clip(args.video, args.output, start, end)
    details.update(
        {
            "source_video": args.video.name,
            "trimmed_video": args.output.name,
            "fps": fps,
            "trimmed_frames": written,
            "trimmed_duration_s": written / fps,
            "margin_s": args.margin_s,
            "scan_sample_step": max(1, args.sample_step),
            "front_marker_color": args.front_color,
        }
    )
    metadata = args.metadata or args.output.with_suffix(".trim.json")
    metadata.write_text(json.dumps(details, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(details, ensure_ascii=False))


if __name__ == "__main__":
    main()
