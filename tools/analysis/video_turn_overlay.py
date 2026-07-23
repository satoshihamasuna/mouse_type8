"""LEGACY: match 2026-07-23 turn videos to logs and overlay trajectories.

The camera is fixed above a dark calibration mat.  The robot is extracted with
background subtraction, while the log path is reconstructed from measured
forward/angular velocity.  A similarity transform aligns the log path to the
video track; this makes endpoint and shape disagreement directly visible.

New Lstart/Lend tuning should use lzero_turn_lengths.py.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import cv2
import matplotlib
import numpy as np
import pandas as pd

matplotlib.use("Agg")
import matplotlib.pyplot as plt


PAIRS = [
    ("long_r90", "PXL_20260723_011408272_longturn90.mp4", "20260723_101617_myshell_debug_log_long_r90.csv"),
    ("long_r180", "PXL_20260723_011712359__longturn180.mp4", "20260723_101802_myshell_debug_log_long_r180.csv"),
    ("in_r45", "PXL_20260723_011852390_turnin45.mp4", "20260723_102035_myshell_debug_log_in_r45.csv"),
    ("in_r135", "PXL_20260723_012136776__turnin135.mp4", "20260723_102154_myshell_debug_log_in_r135.csv"),
    ("r_v90", "PXL_20260723_012323009_turnv90.mp4", "20260723_102414_myshell_debug_log_r_v90.csv"),
    ("out_r45", "PXL_20260723_015415798_turnout45.mp4", "20260723_105431_myshell_debug_log_out_r45.csv"),
    ("out_r135", "PXL_20260723_015553139_turnout135.mp4", "20260723_105610_myshell_debug_log_out_r135.csv"),
]

# Turn-onset frame and robot centre measured from the calibration-grid video.
# Coordinates refer to the 304 x 540 analysis frame. Heading is the observed
# entry direction in image coordinates.
VIDEO_ALIGNMENT = {
    "long_r90":  (34.37, (150, 150), (0.0, 1.0)),
    "long_r180": (45.63, (185, 235), (0.0, 1.0)),
    "in_r45":    (22.95, (383, 320), (0.0, -1.0)),
    "in_r135":   (33.68, (400, 320), (0.0, -1.0)),
    "r_v90":     (30.79, (125, 195), (-0.7071, 0.7071)),
    "out_r45":   (32.10, (140, 215), (0.7071, 0.7071)),
    "out_r135":  (24.30, (460, 80), (0.0, 1.0)),
}

VIDEO_RUN_WINDOWS = {
    "long_r90": (31.0, 37.0),
    "long_r180": (44.4, 47.2),
    "in_r45": (22.4, 23.9),
    "in_r135": (33.4, 35.8),
    "r_v90": (30.3, 32.8),
    "out_r45": (31.5, 34.2),
    "out_r135": (23.5, 27.6),
}


def read_video(path: Path, target_h: int = 540, sample_hz: float = 20.0):
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise RuntimeError(f"cannot open {path}")
    fps = cap.get(cv2.CAP_PROP_FPS)
    count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    duration = count / fps
    step = max(1, int(round(fps / sample_hz)))
    frames, times = [], []
    index = 0
    while True:
        ok = cap.grab()
        if not ok:
            break
        if index % step == 0:
            ok, frame = cap.retrieve()
            if not ok:
                break
            scale = target_h / frame.shape[0]
            frame = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            frames.append(frame)
            times.append(index / fps)
        index += 1
    cap.release()
    return np.asarray(frames), np.asarray(times), fps, duration


def track_robot(frames: np.ndarray):
    # Locate the multicolour PCB in a compact window. Requiring both the blue
    # LED and yellow/orange PCB rejects the red course tape and white grid.
    picks = np.linspace(0, len(frames) - 1, min(80, len(frames))).astype(int)
    background = np.median(frames[picks], axis=0).astype(np.uint8)
    h, w = background.shape[:2]
    points = []
    for frame in frames:
        diff = np.max(cv2.absdiff(frame, background), axis=2)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hue = hsv[:, :, 0]
        sat = hsv[:, :, 1]
        val = hsv[:, :, 2]
        blue = (((hue >= 82) & (hue <= 138) & (sat > 80) & (val > 80))).astype(np.float32)
        yellow = (((hue >= 5) & (hue <= 42) & (sat > 90) & (val > 70))).astype(np.float32)
        changed = (diff > 32)
        colourful = ((val > 45) & changed).astype(np.float32)
        blue *= changed
        yellow *= changed
        k = (35, 35)
        score = (4.0 * cv2.boxFilter(blue, -1, k, normalize=False) +
                 2.0 * cv2.boxFilter(yellow, -1, k, normalize=False) +
                 cv2.boxFilter(colourful, -1, k, normalize=False))
        score[:int(.14*h)] = 0
        score[int(.84*h):] = 0
        score[:, :int(.245*w)] = 0
        score[:, int(.775*w):] = 0
        _, peak, _, location = cv2.minMaxLoc(score)
        points.append(location if peak > 25 else (np.nan, np.nan))
    points = np.asarray(points, float)
    for axis in range(2):
        series = pd.Series(points[:, axis]).interpolate(limit=8, limit_direction="both")
        points[:, axis] = series.rolling(5, center=True, min_periods=1).median()
    return background, points


def track_blue_led(frames: np.ndarray):
    """Track the midpoint of the two blue LEDs mounted on the robot."""
    h, w = frames[0].shape[:2]
    pair_sets = []
    for i, frame in enumerate(frames):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hue, sat, val = cv2.split(hsv)
        mask = ((hue >= 88) & (hue <= 138) & (sat >= 115) & (val >= 155)).astype(np.uint8)
        # Only the calibration mat can contain the running robot.
        mat_top = .20 if w < h else .02
        mask[:int(mat_top*h)] = 0
        mask[int(.86*h):] = 0
        mask[:, :int(.20*w)] = 0
        mask[:, int(.82*w):] = 0
        pairs = []
        grouped = cv2.dilate(mask, np.ones((11, 11), np.uint8))
        n, labels, stats, _ = cv2.connectedComponentsWithStats(grouped)
        for label in range(1, n):
            if not 20 <= stats[label, cv2.CC_STAT_AREA] <= 2500:
                continue
            region = labels == label
            ys, xs = np.where((mask > 0) & region)
            if len(xs) < 4:
                continue
            samples = np.column_stack([xs, ys]).astype(np.float32)
            _, cluster_labels, centers = cv2.kmeans(
                samples, 2, None,
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, .05),
                5, cv2.KMEANS_PP_CENTERS,
            )
            cluster_labels = cluster_labels.ravel()
            counts = np.bincount(cluster_labels, minlength=2)
            if counts.min() < 1 or counts.min() / counts.max() < .10:
                continue
            ca, cb = centers.astype(float)
            separation = float(np.linalg.norm(ca - cb))
            if not 3.0 <= separation <= 55.0:
                continue
            weights = val[ys, xs].astype(float) - 140.0
            sa = float(weights[cluster_labels == 0].sum())
            sb = float(weights[cluster_labels == 1].sum())
            balance = min(sa, sb) / max(sa, sb)
            midpoint = (ca + cb) / 2.0
            score = (sa + sb) * (0.5 + 0.5 * balance)
            pairs.append((score, midpoint, ca, cb, separation))
        pair_sets.append(pairs)
    # Reject pairs formed by stationary blue-ish floor scratches.
    occupancy = np.zeros((h, w), np.uint16)
    for pairs in pair_sets:
        marked = np.zeros((h, w), np.uint8)
        for _, midpoint, _, _, _ in pairs:
            cv2.circle(marked, tuple(np.round(midpoint).astype(int)), 6, 1, -1)
        occupancy += marked
    tracks = np.full((len(frames), 2), np.nan, float)
    strengths = np.zeros(len(frames), float)
    separations = np.full(len(frames), np.nan, float)
    static_limit = max(8, int(len(frames) * .06))
    for i, pairs in enumerate(pair_sets):
        moving = [pair for pair in pairs
                  if occupancy[int(round(pair[1][1])), int(round(pair[1][0]))] < static_limit]
        if moving:
            score, midpoint, _, _, separation = max(moving, key=lambda item: item[0])
            strengths[i], tracks[i], separations[i] = score, midpoint, separation
    return tracks, strengths, separations


def track_robot_body_center(frames: np.ndarray, times: np.ndarray, window: tuple[float, float]):
    """Track the centre of the large yellow/orange robot-top region.

    This reproduces the robust body-centre proxy used by the earlier
    video_robot_track.csv analysis, rather than following LED highlights.
    """
    h, w = frames[0].shape[:2]
    points = np.full((len(frames), 2), np.nan, float)
    scores = np.zeros(len(frames), float)
    for i, (frame, time_s) in enumerate(zip(frames, times)):
        if not window[0] <= time_s <= window[1]:
            continue
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hue, sat, val = cv2.split(hsv)
        mask = ((hue >= 5) & (hue <= 42) & (sat >= 85) & (val >= 75)).astype(np.uint8) * 255
        mat_top = .20 if w < h else .02
        mask[:int(mat_top*h)] = 0
        mask[int(.86*h):] = 0
        mat_left = .05 if w < h else .18
        mat_right = .98 if w < h else .84
        mask[:, :int(mat_left*w)] = 0
        mask[:, int(mat_right*w):] = 0
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
        n, labels, stats, centers = cv2.connectedComponentsWithStats(mask)
        best = None
        for label in range(1, n):
            x, y, ww, hh, area = stats[label]
            if not 12 <= area <= 3500:
                continue
            aspect = max(ww, hh) / max(1, min(ww, hh))
            if aspect > 4.0:
                continue
            # Area dominates; brightness breaks ties between PCB and scratches.
            strength = float(area * np.mean(val[labels == label]) / 255.0)
            if best is None or strength > best[0]:
                best = (strength, centers[label])
        if best is not None:
            scores[i], points[i] = best
    present = scores > 0
    if present.any():
        threshold = max(25.0, float(np.median(scores[present]) * .45))
        points[scores < threshold] = np.nan
    # Remove recurring fixed yellow objects (shelf/tape/reflections). A moving
    # robot visits each spatial bin briefly, while false objects recur.
    occupancy = np.zeros((h, w), np.uint16)
    for point in points[np.isfinite(points[:, 0])]:
        cv2.circle(occupancy, tuple(np.round(point).astype(int)), 8, 1, -1)
    window_frames = max(1, int(np.count_nonzero((times >= window[0]) & (times <= window[1]))))
    static_limit = max(5, int(window_frames * .18))
    for i, point in enumerate(points):
        if np.isfinite(point[0]) and occupancy[int(round(point[1])), int(round(point[0]))] > static_limit:
            points[i] = np.nan
    return points, scores


def extend_body_track_optical_flow(
    frames: np.ndarray, times: np.ndarray, seed_points: np.ndarray, window: tuple[float, float]
):
    """Extend colour-based body centres through blurred turns with optical flow."""
    inside = np.flatnonzero((times >= window[0]) & (times <= window[1]))
    seeded = inside[np.isfinite(seed_points[inside, 0])]
    if not len(seeded):
        return seed_points
    start = int(seeded[0])
    result = np.full_like(seed_points, np.nan)
    result[start] = seed_points[start]
    old_gray = cv2.cvtColor(frames[start], cv2.COLOR_BGR2GRAY)
    centre = seed_points[start].copy()

    def features(gray, frame, c):
        mask = np.zeros_like(gray)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        colourful = ((hsv[:, :, 1] > 55) & (hsv[:, :, 2] > 55)).astype(np.uint8) * 255
        cv2.circle(mask, tuple(np.round(c).astype(int)), 32, 255, -1)
        mask = cv2.bitwise_and(mask, colourful)
        pts = cv2.goodFeaturesToTrack(gray, 60, .01, 3, mask=mask, blockSize=5)
        return pts

    old_pts = features(old_gray, frames[start], centre)
    for i in range(start + 1, inside[-1] + 1):
        new_gray = cv2.cvtColor(frames[i], cv2.COLOR_BGR2GRAY)
        if old_pts is None or len(old_pts) < 3:
            old_pts = features(old_gray, frames[i - 1], centre)
        if old_pts is not None and len(old_pts):
            new_pts, status, _ = cv2.calcOpticalFlowPyrLK(
                old_gray, new_gray, old_pts, None,
                winSize=(61, 61), maxLevel=4,
                criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 40, .01),
            )
            good_old = old_pts[status.ravel() == 1].reshape(-1, 2)
            good_new = new_pts[status.ravel() == 1].reshape(-1, 2)
            if len(good_new) >= 3:
                flow = good_new - good_old
                displacement = np.median(flow, axis=0)
                if np.linalg.norm(displacement) < 90:
                    centre += displacement
                    result[i] = centre
                old_pts = good_new.reshape(-1, 1, 2)
            else:
                old_pts = None
        # A nearby colour centre corrects slow optical-flow drift.
        if np.isfinite(seed_points[i, 0]) and np.linalg.norm(seed_points[i] - centre) < 45:
            centre = .75 * centre + .25 * seed_points[i]
            result[i] = centre
        old_gray = new_gray
        if i % 4 == 0:
            old_pts = features(old_gray, frames[i], centre)
    return result


def longest_motion_track(points: np.ndarray, times: np.ndarray):
    """Interpolate short LED dropouts and retain the principal moving run."""
    valid = np.isfinite(points[:, 0])
    if valid.sum() < 3:
        raise RuntimeError("too few blue LED detections")
    # Split detections at gaps longer than 0.7 s and score by spatial extent.
    idx = np.flatnonzero(valid)
    cuts = np.flatnonzero(np.diff(times[idx]) > .7) + 1
    groups = np.split(idx, cuts)
    group = max(groups, key=lambda g: np.ptp(points[g, 0]) + np.ptp(points[g, 1]))
    lo, hi = group[0], group[-1] + 1
    segment = points[lo:hi].copy()
    x = np.arange(len(segment))
    ok = np.isfinite(segment[:, 0])
    for axis in range(2):
        segment[:, axis] = np.interp(x, x[ok], segment[ok, axis])
        segment[:, axis] = pd.Series(segment[:, axis]).rolling(5, center=True, min_periods=1).median()
    # Trim stationary tails based on displacement from a 0.2-s smoothed trace.
    speed = np.linalg.norm(np.gradient(segment, axis=0), axis=1)
    moving = pd.Series(speed).rolling(7, center=True, min_periods=1).median().to_numpy() > .35
    moving_idx = np.flatnonzero(moving)
    if len(moving_idx):
        a, b = max(0, moving_idx[0] - 3), min(len(segment), moving_idx[-1] + 4)
        segment, lo, hi = segment[a:b], lo + a, lo + b
    return segment, times[lo:hi], lo, hi


def gate_body_track_by_blue(
    frames: np.ndarray, body_path: np.ndarray, times: np.ndarray, absolute_lo: int
):
    """Keep only the body-centre trajectory while blue illumination is visible."""
    active = np.zeros(len(body_path), bool)
    for j, centre in enumerate(body_path):
        frame = frames[absolute_lo + j]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hue, sat, val = cv2.split(hsv)
        blue = ((hue >= 86) & (hue <= 142) & (sat >= 90) & (val >= 120)).astype(np.uint8)
        local = np.zeros_like(blue)
        cv2.circle(local, tuple(np.round(centre).astype(int)), 42, 1, -1)
        active[j] = int(np.count_nonzero(blue & local)) >= 3
    indices = np.flatnonzero(active)
    if len(indices) < 2:
        raise RuntimeError("blue illumination was not detected around the robot centre")
    # Join short LED-detection dropouts (motion blur), but split true off periods.
    cuts = np.flatnonzero(np.diff(times[indices]) > .30) + 1
    groups = np.split(indices, cuts)
    group = max(groups, key=lambda g: np.ptp(body_path[g, 0]) + np.ptp(body_path[g, 1]))
    lo, hi = int(group[0]), int(group[-1] + 1)
    return body_path[lo:hi], times[lo:hi], absolute_lo + lo, absolute_lo + hi, active[lo:hi]


def active_video_segment(points: np.ndarray, times: np.ndarray):
    speed = np.linalg.norm(np.gradient(points, axis=0), axis=1)
    speed = pd.Series(speed).rolling(11, center=True, min_periods=1).median().to_numpy()
    moving = speed > max(0.45, np.percentile(speed, 55))
    indices = np.flatnonzero(moving)
    # Preserve the complete connected run, with a small margin.
    lo, hi = max(0, indices[0] - 8), min(len(points), indices[-1] + 9)
    return points[lo:hi], times[lo:hi], lo, hi


def log_path(path: Path):
    data = pd.read_csv(path)
    t = data["cnt"].to_numpy(float) * 0.001
    v = data["ego.velo"].to_numpy(float)
    omega = data["ego.rad_velo"].to_numpy(float)
    ideal_omega = data["ideal.rad_velo"].to_numpy(float)
    active = np.flatnonzero(np.abs(ideal_omega) > 0.03)
    lo, hi = max(0, active[0] - 1), min(len(data), active[-1] + 2)
    theta = np.zeros(len(data))
    x = np.zeros(len(data))
    y = np.zeros(len(data))
    for i in range(1, len(data)):
        dt = t[i] - t[i - 1]
        theta[i] = theta[i - 1] + omega[i] * dt
        x[i] = x[i - 1] + v[i] * math.sin(theta[i]) * dt
        y[i] = y[i - 1] + v[i] * math.cos(theta[i]) * dt
    path_m = np.column_stack([x, y])
    return data, t, path_m[lo:hi] - path_m[lo], lo, hi


def resample_curve(points: np.ndarray, n: int = 250):
    d = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    keep = np.r_[True, np.diff(d) > 1e-9]
    d, points = d[keep], points[keep]
    q = np.linspace(0, d[-1], n)
    return np.column_stack([np.interp(q, d, points[:, j]) for j in range(2)])


def align_similarity(source: np.ndarray, target: np.ndarray):
    a, b = resample_curve(source), resample_curve(target)
    ac, bc = a - a.mean(0), b - b.mean(0)
    u, _, vt = np.linalg.svd(ac.T @ bc)
    r = u @ vt
    if np.linalg.det(r) < 0:
        u[:, -1] *= -1
        r = u @ vt
    scale = np.sum((ac @ r) * bc) / np.sum(ac * ac)
    translation = b.mean(0) - scale * a.mean(0) @ r
    fitted = scale * source @ r + translation
    fit_sample = scale * a @ r + translation
    rmse = float(np.sqrt(np.mean(np.sum((fit_sample - b) ** 2, axis=1))))
    return fitted, scale, rmse


def align_log_to_partial_video(log_m: np.ndarray, video_px: np.ndarray, px_per_m: float = 708.0):
    """Rigidly align the best log sub-curve to a detected LED partial curve."""
    target = resample_curve(video_px, 80)
    target_len_m = np.sum(np.linalg.norm(np.diff(target, axis=0), axis=1)) / px_per_m
    d = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(log_m, axis=0), axis=1))]
    best = None
    for start in np.linspace(0, max(0, d[-1] - target_len_m), 160):
        end = start + target_len_m
        q = np.linspace(start, end, len(target))
        source = np.column_stack([np.interp(q, d, log_m[:, axis]) for axis in range(2)]) * px_per_m
        ac, bc = source - source.mean(0), target - target.mean(0)
        u, _, vt = np.linalg.svd(ac.T @ bc)
        rotation = u @ vt
        if np.linalg.det(rotation) < 0:
            u[:, -1] *= -1
            rotation = u @ vt
        translation = target.mean(0) - source.mean(0) @ rotation
        fitted = source @ rotation + translation
        rmse = float(np.sqrt(np.mean(np.sum((fitted - target) ** 2, axis=1))))
        if best is None or rmse < best[0]:
            best = (rmse, rotation, translation, start, end)
    rmse, rotation, translation, start, end = best
    full = log_m * px_per_m @ rotation + translation
    return full, rmse, start, end


def draw_overlay(background, video_points, log_points_px, title, out_path):
    canvas = background.copy()
    vp = np.round(video_points).astype(int)
    lp = np.round(log_points_px).astype(int)
    cv2.polylines(canvas, [vp], False, (40, 220, 40), 7, cv2.LINE_AA)
    cv2.polylines(canvas, [lp], False, (30, 80, 245), 5, cv2.LINE_AA)
    for pts, colour in ((vp, (40, 220, 40)), (lp, (30, 80, 245))):
        cv2.circle(canvas, tuple(pts[0]), 10, colour, -1, cv2.LINE_AA)
        cv2.drawMarker(canvas, tuple(pts[-1]), colour, cv2.MARKER_TILTED_CROSS, 22, 4)
    cv2.rectangle(canvas, (12, 12), (515, 92), (0, 0, 0), -1)
    cv2.putText(canvas, title, (24, 42), cv2.FONT_HERSHEY_SIMPLEX, .8, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(canvas, "VIDEO TRACK", (24, 76), cv2.FONT_HERSHEY_SIMPLEX, .65, (40, 220, 40), 2)
    cv2.putText(canvas, "LOG v/omega", (245, 76), cv2.FONT_HERSHEY_SIMPLEX, .65, (30, 80, 245), 2)
    cv2.imwrite(str(out_path), canvas)


def representative_frame(path: Path, target_h: int = 540):
    cap = cv2.VideoCapture(str(path))
    fps = cap.get(cv2.CAP_PROP_FPS)
    count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    duration = count / fps
    cap.set(cv2.CAP_PROP_POS_FRAMES, max(0, count // 2))
    ok, frame = cap.read()
    cap.release()
    if not ok:
        raise RuntimeError(f"cannot read {path}")
    scale = target_h / frame.shape[0]
    return cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA), fps, duration


def frame_at_time(path: Path, seconds: float, target_h: int = 540):
    cap = cv2.VideoCapture(str(path))
    fps = cap.get(cv2.CAP_PROP_FPS)
    count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    duration = count / fps
    cap.set(cv2.CAP_PROP_POS_MSEC, seconds * 1000)
    ok, frame = cap.read()
    cap.release()
    if not ok:
        raise RuntimeError(f"cannot read {path} at {seconds}s")
    scale = target_h / frame.shape[0]
    return cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA), fps, duration


def draw_log_overlay(background, path_m, title, origin, heading, out_path):
    # Visible mat: 8 x 90-mm cells and about 510 px at this resolution.
    px_per_m = 708.0
    heading = np.asarray(heading, float)
    heading /= np.linalg.norm(heading)
    lateral_basis = np.array([heading[1], -heading[0]])
    px = (np.asarray(origin, float) + path_m[:, 1, None] * heading * px_per_m +
          path_m[:, 0, None] * lateral_basis * px_per_m)
    curve = np.round(px).astype(int)
    canvas = background.copy()
    cv2.polylines(canvas, [curve], False, (0, 0, 0), 11, cv2.LINE_AA)
    cv2.polylines(canvas, [curve], False, (20, 70, 250), 6, cv2.LINE_AA)
    cv2.circle(canvas, tuple(curve[0]), 11, (40, 220, 40), -1, cv2.LINE_AA)
    cv2.drawMarker(canvas, tuple(curve[-1]), (20, 70, 250), cv2.MARKER_TILTED_CROSS, 24, 5)
    cv2.rectangle(canvas, (12, 12), (590, 92), (0, 0, 0), -1)
    cv2.putText(canvas, title, (24, 42), cv2.FONT_HERSHEY_SIMPLEX, .8, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(canvas, "LOG-MEASURED v/omega PATH", (24, 76), cv2.FONT_HERSHEY_SIMPLEX, .65, (20, 70, 250), 2)
    cv2.imwrite(str(out_path), canvas)


def draw_video_log_overlay(background, video_px, log_px, title, out_path):
    canvas = background.copy()
    video_curve = np.round(video_px).astype(int)
    log_curve = np.round(log_px).astype(int)
    cv2.polylines(canvas, [log_curve], False, (0, 0, 0), 9, cv2.LINE_AA)
    cv2.polylines(canvas, [log_curve], False, (30, 70, 245), 4, cv2.LINE_AA)
    cv2.polylines(canvas, [video_curve], False, (0, 0, 0), 10, cv2.LINE_AA)
    cv2.polylines(canvas, [video_curve], False, (0, 220, 255), 5, cv2.LINE_AA)
    cv2.circle(canvas, tuple(log_curve[0]), 8, (40, 220, 40), -1, cv2.LINE_AA)
    cv2.drawMarker(canvas, tuple(log_curve[-1]), (30, 70, 245), cv2.MARKER_TILTED_CROSS, 20, 4)
    cv2.rectangle(canvas, (8, 8), (min(canvas.shape[1] - 8, 650), 91), (0, 0, 0), -1)
    cv2.putText(canvas, title, (18, 38), cv2.FONT_HERSHEY_SIMPLEX, .75, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(canvas, "YELLOW: BODY CENTER WHILE BLUE ON   RED: LOG", (18, 73),
                cv2.FONT_HERSHEY_SIMPLEX, .57, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.imwrite(str(out_path), canvas)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--video-dir", type=Path, required=True)
    parser.add_argument("--log-dir", type=Path, default=Path("tools/logs"))
    parser.add_argument("--output", type=Path, default=Path("tools/turn_analysis/1600/video_20260723_overlay"))
    parser.add_argument("--motion", action="append", help="process only named motion(s)")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    metrics = []
    all_plots = []
    selected = [p for p in PAIRS if not args.motion or p[0] in args.motion]
    for motion, video_name, log_name in selected:
        frames, frame_times, fps, duration = read_video(args.video_dir / video_name, sample_hz=15)
        window_start, window_end = VIDEO_RUN_WINDOWS[motion]
        body_points, body_strength = track_robot_body_center(
            frames, frame_times, (window_start, window_end))
        full_body_path, full_body_times, body_lo, body_hi = longest_motion_track(body_points, frame_times)
        video_path, video_times, vlo, vhi, blue_active = gate_body_track_by_blue(
            frames, full_body_path, full_body_times, body_lo)
        background = frames[(vlo + vhi) // 2].copy()
        data, log_times, path_m, llo, lhi = log_path(args.log_dir / log_name)
        aligned_log, fit_rmse_px, matched_start_m, matched_end_m = align_log_to_partial_video(path_m, video_path)
        dt = np.diff(log_times[llo:lhi], prepend=log_times[llo])
        angle_deg = math.degrees(float(np.sum(data["ego.rad_velo"].iloc[llo:lhi].to_numpy() * dt)))
        target_deg = math.degrees(float(np.sum(data["ideal.rad_velo"].iloc[llo:lhi].to_numpy() * dt)))
        speed = data["ego.velo"].iloc[llo:lhi]
        displacement = path_m[-1] * 1000
        path_length = float(np.sum(np.linalg.norm(np.diff(path_m, axis=0), axis=1)) * 1000)
        metrics.append({
            "motion": motion, "video": video_name, "log": log_name,
            "video_body_start_s": float(video_times[0]),
            "video_body_end_s": float(video_times[-1]),
            "video_body_center_samples": int(len(video_path)),
            "trajectory_gate": "blue illumination near body centre",
            "video_log_fit_rmse_px": fit_rmse_px,
            "video_log_fit_rmse_mm": fit_rmse_px / 708.0 * 1000.0,
            "matched_log_arc_start_mm": matched_start_m * 1000.0,
            "matched_log_arc_end_mm": matched_end_m * 1000.0,
            "log_start_ms": int(llo), "log_end_ms": int(lhi - 1),
            "turn_angle_deg": angle_deg, "angle_error_deg": angle_deg - target_deg,
            "mean_speed_m_s": float(speed.mean()), "min_speed_m_s": float(speed.min()),
            "endpoint_lateral_mm": float(displacement[0]),
            "endpoint_forward_mm": float(displacement[1]),
            "turn_path_length_mm": path_length,
            "video_duration_s": duration, "fps": fps,
        })
        overlay = args.output / f"{motion}_trajectory_overlay.jpg"
        draw_video_log_overlay(background, video_path, aligned_log, motion, overlay)
        all_plots.append((motion, cv2.cvtColor(cv2.imread(str(overlay)), cv2.COLOR_BGR2RGB)))
        with (args.output / f"{motion}_metrics.json").open("w", encoding="utf-8") as f:
            json.dump(metrics[-1], f, ensure_ascii=False, indent=2)
    metric_files = list(args.output.glob("*_metrics.json"))
    metrics = [json.loads(p.read_text(encoding="utf-8")) for p in metric_files]
    metrics.sort(key=lambda row: [p[0] for p in PAIRS].index(row["motion"]))
    pd.DataFrame(metrics).to_csv(args.output / "turn_metrics.csv", index=False)
    all_plots = []
    for row in metrics:
        image_path = args.output / f"{row['motion']}_trajectory_overlay.jpg"
        if image_path.exists():
            all_plots.append((row["motion"], cv2.cvtColor(cv2.imread(str(image_path)), cv2.COLOR_BGR2RGB)))
    fig, axes = plt.subplots(4, 2, figsize=(14, 15))
    for ax, (motion, image) in zip(axes.flat, all_plots):
        ax.imshow(image)
        ax.set_title(motion)
        ax.axis("off")
    axes.flat[-1].axis("off")
    fig.tight_layout()
    fig.savefig(args.output / "all_turn_overlays.jpg", dpi=150)
    plt.close(fig)
    with (args.output / "manifest.json").open("w", encoding="utf-8") as f:
        json.dump({"pairs": metrics}, f, ensure_ascii=False, indent=2)


if __name__ == "__main__":
    main()
