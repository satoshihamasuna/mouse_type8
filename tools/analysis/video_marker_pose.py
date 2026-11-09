"""Measure a micromouse pose from coloured body markers and corner ArUcos.

The calibration field is an 8 x 8 grid of 45 mm half-cells (360 mm square).
ArUco IDs 5, 7, 6, and 4 are at the top-left, top-right, bottom-left, and
bottom-right corners.
The blue circle is the front marker and the yellow circle is the rear marker.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import cv2
import matplotlib
import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pixel_camera_reference import detect_board_reference, detect_registered_vertices
from pixel_camera_geometry import (validate_geometry, correct_body_markers, pose_from_corrected_markers,
                                   apparent_floor, geometry_for_measurement)

matplotlib.use("Agg")
import matplotlib.pyplot as plt


GRID_MM = 360.0
ARUCO_CORNER_IDS = (5, 7, 6, 4)
WORLD_CORNERS = np.float32(
    [[0.0, GRID_MM], [GRID_MM, GRID_MM], [0.0, 0.0], [GRID_MM, 0.0]]
)


def colour_components(frame: np.ndarray, colour: str, detection: dict | None = None) -> list[tuple[np.ndarray, int]]:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hue, sat, val = cv2.split(hsv)
    if colour == "blue":
        # The matte blue circle becomes weakly saturated in portrait videos
        # after MP4 trimming/recompression, while its hue remains stable.
        mask, saturation_min, value_min = (hue >= 93) & (hue <= 115), 55, 100
    elif colour == "red":
        mask, saturation_min, value_min = (hue <= 10) | (hue >= 170), 100, 95
    elif colour == "magenta":
        mask, saturation_min, value_min = (hue >= 140) & (hue <= 169), 90, 90
    else:
        mask, saturation_min, value_min = (hue >= 25) & (hue <= 36), 130, 145
    if detection:
        limits = detection.get(colour, {})
        if set(limits)-{'minimum_saturation', 'minimum_value'}:
            raise ValueError('unknown colour detection setting')
        saturation_min = limits.get('minimum_saturation', saturation_min)
        value_min = limits.get('minimum_value', value_min)
        if not all(np.isfinite(v) and 0 <= v <= 255 for v in (saturation_min, value_min)):
            raise ValueError('colour thresholds must be finite values between zero and 255')
    mask &= (sat >= saturation_min) & (val >= value_min)
    mask = cv2.morphologyEx(
        mask.astype(np.uint8) * 255, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)
    )
    n, _, stats, centres = cv2.connectedComponentsWithStats(mask)
    h, w = frame.shape[:2]
    result = []
    for label in range(1, n):
        area = int(stats[label, cv2.CC_STAT_AREA])
        x, y = centres[label]
        if 15 <= area <= 3000 and .04 * w < x < .96 * w and .02 * h < y < .99 * h:
            result.append((centres[label].astype(float), area))
    return result


def inner_aruco_points(found: dict[int, np.ndarray]) -> dict[int, np.ndarray]:
    found = {marker_id: found[marker_id] for marker_id in ARUCO_CORNER_IDS if marker_id in found}
    if not found:
        return {}
    # Printed markers may be rotated independently on the board. Their decoded
    # vertex indices rotate too, so choose the vertex nearest the board centre.
    centre = np.mean([quad.mean(axis=0) for quad in found.values()], axis=0)
    return {marker_id: quad[np.argmin(np.sum((quad - centre) ** 2, axis=1))]
            for marker_id, quad in found.items()}


def marker_registration(frame: np.ndarray, marker_reference: dict) -> dict:
    markers = detect_registered_vertices(frame, marker_reference)
    result = detect_board_reference(frame, markers, marker_reference)
    result["detected_marker_ids"] = [key for item in markers.values() for key in item.get("decoded_ids", item["ids"])]
    return result


def registration_quad(result: dict) -> np.ndarray | None:
    if not result.get("available"):
        return None
    names = {int(marker_id): name for name, marker_id in result["board_corner_ids"].items()}
    if not all(marker_id in names for marker_id in ARUCO_CORNER_IDS):
        return None
    return np.float32([result["points_px"][names[marker_id]] for marker_id in ARUCO_CORNER_IDS])


def detect_aruco_quad(frame: np.ndarray, marker_reference: dict | None = None) -> np.ndarray | None:
    if marker_reference is not None:
        return registration_quad(marker_registration(frame, marker_reference))
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    corners, ids, _ = cv2.aruco.ArucoDetector(dictionary).detectMarkers(frame)
    if ids is None:
        return None
    found = {int(marker_id): quad.reshape(4, 2) for marker_id, quad in zip(ids.ravel(), corners)}
    if not all(marker_id in found for marker_id in ARUCO_CORNER_IDS):
        return None
    points = inner_aruco_points(found)
    return np.float32([points[marker_id] for marker_id in ARUCO_CORNER_IDS])


def transform_points(points: np.ndarray, homography: np.ndarray) -> np.ndarray:
    return cv2.perspectiveTransform(np.float32(points).reshape(-1, 1, 2), homography).reshape(-1, 2)


def board_to_pose_transform(profile):
    corners = dict(TL=[0, 0], TR=[GRID_MM, 0], BR=[GRID_MM, GRID_MM], BL=[0, GRID_MM])
    names = {int(key): name for name, key in profile['board_corner_ids'].items()}
    source = np.float32([corners[names[key]] for key in ARUCO_CORNER_IDS])
    return cv2.getPerspectiveTransform(source, WORLD_CORNERS)


def detect_led_interval(
    frames: list[np.ndarray], pose: pd.DataFrame, calibration_quad: np.ndarray
) -> tuple[np.ndarray, np.ndarray, float]:
    """Detect blue turn LEDs while excluding the permanent blue circle."""
    inverse = cv2.getPerspectiveTransform(WORLD_CORNERS, calibration_quad)
    if 'front_floor_x_mm' in pose:
        centre_points = pose[['midpoint_floor_x_mm', 'midpoint_floor_y_mm']].to_numpy()
        front_points = pose[['front_floor_x_mm', 'front_floor_y_mm']].to_numpy()
    else:
        centre_points = pose[["center_x_mm", "center_y_mm"]].to_numpy()
        front_points = pose[["blue_x_mm", "blue_y_mm"]].to_numpy()
    world_points = np.column_stack(
        [
            centre_points, front_points,
        ]
    ).reshape(-1, 2, 2)
    counts = np.zeros(len(pose), dtype=int)
    for i, (frame, points_mm) in enumerate(zip(frames, world_points)):
        if 'front_image_x_px' in pose:
            row = pose.iloc[i]
            centre_px = np.array([row.midpoint_image_x_px, row.midpoint_image_y_px])
            blue_px = np.array([row.front_image_x_px, row.front_image_y_px])
        else:
            centre_px, blue_px = transform_points(points_mm, inverse)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hue, sat, val = cv2.split(hsv)
        blue_light = (
            (hue >= 82) & (hue <= 142) & (sat >= 90) & (val >= 120)
        ).astype(np.uint8)
        roi = np.zeros(blue_light.shape, np.uint8)
        # Radius follows the apparent marker scale, making the test robust to
        # modest changes in camera height and perspective.
        marker_span_px = float(np.linalg.norm(centre_px - blue_px)) * 2.0
        cv2.circle(
            roi, tuple(np.round(centre_px).astype(int)),
            max(20, int(round(marker_span_px * .70))), 1, -1,
        )
        cv2.circle(
            roi, tuple(np.round(blue_px).astype(int)),
            max(7, int(round(marker_span_px * .20))), 0, -1,
        )
        counts[i] = int(np.count_nonzero(blue_light & roi))
    threshold = max(100.0, .08 * float(np.percentile(counts, 95)))
    raw_active = counts > threshold
    indices = np.flatnonzero(raw_active)
    active = np.zeros(len(pose), dtype=bool)
    if len(indices):
        cuts = np.flatnonzero(np.diff(indices) > 1) + 1
        groups = np.split(indices, cuts)
        group = max(groups, key=lambda g: (len(g), counts[g].sum()))
        active[group[0]:group[-1] + 1] = True
    return counts, active, threshold


def select_marker_pair(
    blues: list[tuple[np.ndarray, int]],
    yellows: list[tuple[np.ndarray, int]],
    homography: np.ndarray,
    previous: np.ndarray | None,
    body_geometry: dict | None = None,
    board_to_pose: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray] | None:
    candidates = []
    for blue, blue_area in blues:
        for yellow, yellow_area in yellows:
            blue_mm, yellow_mm = transform_points(np.vstack([blue, yellow]), homography)
            if body_geometry is not None:
                board = transform_points([blue_mm, yellow_mm], np.linalg.inv(board_to_pose))
                corrected = correct_body_markers(board[0], board[1], body_geometry)
                blue_mm, yellow_mm = transform_points(
                    [corrected['front_xy_mm'], corrected['rear_xy_mm']], board_to_pose)
            separation_mm = float(np.linalg.norm(blue_mm - yellow_mm))
            if not 32.0 <= separation_mm <= 72.0:
                continue
            centre = (blue_mm + yellow_mm) / 2.0
            if not (-20.0 <= centre[0] <= GRID_MM + 20.0
                    and -20.0 <= centre[1] <= GRID_MM + 20.0):
                continue
            # The true circles are the largest similarly sized colour regions.
            balance = min(blue_area, yellow_area) / max(blue_area, yellow_area)
            score = math.log1p(blue_area + yellow_area) + 1.5 * balance
            nominal_span = body_geometry.get('nominal_marker_span_mm', 55.0) if body_geometry is not None else 55.0
            score -= .020 * (separation_mm - nominal_span) ** 2
            if previous is not None:
                step = float(np.linalg.norm(centre - previous))
                score -= .08 * max(0.0, step - 35.0)
            candidates.append((score, blue_mm, yellow_mm))
    if not candidates:
        return None
    _, blue_mm, yellow_mm = max(candidates, key=lambda item: item[0])
    return blue_mm, yellow_mm


def calibration_quad_from_video(path: Path, sample_step: int = 30, marker_reference: dict | None = None) -> np.ndarray | None:
    """Find a stable board quad, optionally registered by any two or more markers."""
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        return None
    if marker_reference is not None:
        cap.set(cv2.CAP_PROP_ORIENTATION_AUTO, 0)
    quads = []
    index = 0
    while True:
        ok = cap.grab()
        if not ok:
            break
        if index % sample_step == 0:
            ok, frame = cap.retrieve()
            if ok:
                quad = detect_aruco_quad(frame, marker_reference)
                if quad is not None:
                    quads.append(quad)
        index += 1
    cap.release()
    if not quads:
        return None
    return np.median(np.asarray(quads), axis=0).astype(np.float32)


def iter_video_frames(path, raw_orientation=False, video_window=(0, None), frame_stride=1):
    """画像を蓄積せず、実PTSと元のフレーム番号で同じ区間を読み直す。"""
    if not isinstance(frame_stride, int) or not 1 <= frame_stride <= 4:
        raise ValueError('frame_stride must be an integer from1 to4')
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise RuntimeError(f'cannot open video: {path}')
    if raw_orientation:
        cap.set(cv2.CAP_PROP_ORIENTATION_AUTO, 0)
    start, end = video_window
    index = 0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            pts = float(cap.get(cv2.CAP_PROP_POS_MSEC))/1000
            if end is not None and pts > end:
                break
            if pts >= start and index % frame_stride == 0:
                yield index, frame, pts
            index += 1
    finally:
        cap.release()


def read_video_pose(
    path: Path, front_colour: str = "blue", calibration_video: Path | None = None,
    marker_reference: dict | None = None,
    body_geometry: dict | None = None,
    video_window: tuple[float, float | None] = (0, None),
    marker_detection: dict | None = None,
    progress=None,
    frame_stride=1,
) -> tuple[pd.DataFrame, np.ndarray, float, float]:
    if not isinstance(frame_stride, int) or not 1 <= frame_stride <= 4:
        raise ValueError('frame_stride must be an integer from1 to4')
    geometry_transform = None
    if body_geometry is not None:
        validate_geometry(body_geometry)
        if body_geometry.get('front_colour', front_colour) != front_colour:
            raise ValueError('front marker colour does not match the body geometry configuration')
        if body_geometry.get('rear_colour', 'yellow') != 'yellow':
            raise ValueError('rear marker colour must be yellow')
        if marker_reference is None:
            raise ValueError('body geometry requires a marker reference with explicit board axes')
        expected_id = body_geometry.get('reference_id')
        if expected_id and expected_id != marker_reference['reference_id']:
            raise ValueError('body geometry belongs to a different board reference')
        geometry_transform = board_to_pose_transform(marker_reference)
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise RuntimeError(f"cannot open video: {path}")
    if marker_reference is not None:
        cap.set(cv2.CAP_PROP_ORIENTATION_AUTO, 0)
    fps = float(cap.get(cv2.CAP_PROP_FPS))
    quads, registrations, timestamps = [], [], []
    start_s, end_s = video_window
    if not np.isfinite(start_s) or start_s < 0 or (end_s is not None and (not np.isfinite(end_s) or end_s <= start_s)):
        raise ValueError('invalid video time window')
    dynamic_grid = bool(marker_reference and marker_reference.get('grid_refinement', {}).get('enabled'))
    index = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        pts = float(cap.get(cv2.CAP_PROP_POS_MSEC))/1000
        if pts < start_s:
            index += 1
            continue
        if end_s is not None and pts > end_s:
            break
        if index % frame_stride:
            index += 1
            continue
        timestamps.append(pts)
        if marker_reference is None:
            quads.append(detect_aruco_quad(frame))
        else:
            result = marker_registration(frame, marker_reference)
            registrations.append(result)
            quads.append(registration_quad(result))
        index += 1
        if progress is not None and len(quads) % 120 == 0:
            progress('calibration', len(quads), pts)
    cap.release()
    frame_count = len(quads)
    if frame_count < 3:
        raise ValueError('at least three video frames are required')
    if not np.isfinite(timestamps).all() or np.any(np.diff(timestamps) <= 0):
        raise ValueError('video presentation timestamps must be finite and increasing')
    if marker_reference is None:
        complete_rate = sum(quad is not None for quad in quads)/frame_count
    else:
        complete_rate = sum(set(ARUCO_CORNER_IDS) <= set(result["detected_marker_ids"]) for result in registrations)/frame_count
    valid_quads = np.asarray([quad for quad in quads if quad is not None])
    if len(valid_quads):
        fixed_quad = np.median(valid_quads, axis=0).astype(np.float32)
    elif calibration_video is not None:
        fixed_quad = calibration_quad_from_video(calibration_video, marker_reference=marker_reference)
        if fixed_quad is None:
            raise RuntimeError("no valid marker calibration; reference mode requires at least two registered markers")
    else:
        raise RuntimeError("no valid marker calibration; four ArUcos or a reference with two matched markers are required")
    homography = cv2.getPerspectiveTransform(fixed_quad, WORLD_CORNERS)

    records = []
    previous = None
    for offset, (index, frame, pts) in enumerate(iter_video_frames(path, marker_reference is not None, video_window, frame_stride)):
        local_homography = homography
        if dynamic_grid:
            if quads[offset] is None:
                records.append(dict(frame=index, time_video_s=pts, marker_detected_raw=False, calibration_valid=False))
                continue
            local_homography = cv2.getPerspectiveTransform(quads[offset], WORLD_CORNERS)
        pair = select_marker_pair(
            colour_components(frame, front_colour, marker_detection),
            colour_components(frame, "yellow", marker_detection),
            local_homography,
            previous,
            body_geometry, geometry_transform,
        )
        if pair is None:
            records.append(
                {"frame": index, "time_video_s": pts, "marker_detected_raw": False, "calibration_valid": True}
            )
            continue
        blue, yellow = pair
        centre = (blue + yellow) / 2.0
        additional = {}
        floor = np.vstack([blue, (blue+yellow)/2])
        if body_geometry is not None:
            board_points = transform_points([blue, yellow], np.linalg.inv(geometry_transform))
            geometry_pose = pose_from_corrected_markers(*board_points, body_geometry)
            if geometry_pose['axle_xy_mm'] is not None:
                centre = transform_points([geometry_pose['axle_xy_mm']], geometry_transform)[0]
            floor_front = apparent_floor(board_points[0], body_geometry['front_height_mm'],
                                         body_geometry['camera_center_board_mm'])
            floor_rear = apparent_floor(board_points[1], body_geometry['rear_height_mm'],
                                        body_geometry['camera_center_board_mm'])
            floor = transform_points([floor_front, (floor_front+floor_rear)/2], geometry_transform)
        pixels = transform_points(floor, np.linalg.inv(local_homography))
        additional = dict(front_floor_x_mm=floor[0, 0], front_floor_y_mm=floor[0, 1],
                          midpoint_floor_x_mm=floor[1, 0], midpoint_floor_y_mm=floor[1, 1],
                          front_image_x_px=pixels[0, 0], front_image_y_px=pixels[0, 1],
                          midpoint_image_x_px=pixels[1, 0], midpoint_image_y_px=pixels[1, 1])
        heading = math.atan2(blue[1] - yellow[1], blue[0] - yellow[0])
        records.append(
            {
                "frame": index,
                "time_video_s": pts,
                "marker_detected_raw": True,
                "calibration_valid": True,
                "blue_x_mm": blue[0],
                "blue_y_mm": blue[1],
                "yellow_x_mm": yellow[0],
                "yellow_y_mm": yellow[1],
                "center_x_mm": centre[0],
                "center_y_mm": centre[1],
                "heading_rad": heading,
                "marker_separation_mm": np.linalg.norm(blue - yellow),
                **additional,
            }
        )
        previous = centre
    pose = pd.DataFrame(records)
    if not pose.marker_detected_raw.any():
        raise ValueError('no valid body marker pair; colour/height/calibration quality must be checked')
    for column in ('front_floor_x_mm', 'front_floor_y_mm', 'midpoint_floor_x_mm', 'midpoint_floor_y_mm',
                   'front_image_x_px', 'front_image_y_px', 'midpoint_image_x_px', 'midpoint_image_y_px'):
        if column in pose:
            pose[column] = pose[column].interpolate(limit_direction='both')
    for column in [
        "blue_x_mm", "blue_y_mm", "yellow_x_mm", "yellow_y_mm",
        "center_x_mm", "center_y_mm", "marker_separation_mm",
    ]:
        pose[column] = pose[column].interpolate(limit_direction="both")
    for column in ["center_x_mm", "center_y_mm", "blue_x_mm", "blue_y_mm",
                   "yellow_x_mm", "yellow_y_mm", "marker_separation_mm"]:
        pose[column] = pose[column].rolling(max(1, 5//frame_stride), center=True, min_periods=1).median()
    raw_heading = pose["heading_rad"].to_numpy(float)
    heading_cos = pd.Series(np.cos(raw_heading)).interpolate(limit_direction="both")
    heading_sin = pd.Series(np.sin(raw_heading)).interpolate(limit_direction="both")
    unwrapped = np.unwrap(np.arctan2(heading_sin, heading_cos))
    pose["heading_rad"] = pd.Series(unwrapped).rolling(max(1, 5//frame_stride), center=True, min_periods=1).median()
    dx = np.gradient(pose["center_x_mm"], pose.time_video_s.to_numpy())
    dy = np.gradient(pose["center_y_mm"], pose.time_video_s.to_numpy())
    pose["speed_playback_mm_s"] = (
        pd.Series(np.hypot(dx, dy)).rolling(max(1, 9//frame_stride), center=True, min_periods=1).median()
    )
    led_counts, led_active, led_threshold = detect_led_interval(
        (frame for _, frame, _ in iter_video_frames(path, marker_reference is not None, video_window, frame_stride)), pose, fixed_quad)
    pose["turn_led_blue_pixels"] = led_counts
    pose["turn_led_active"] = led_active
    pose.attrs["turn_led_threshold"] = led_threshold
    pose.attrs["marker_calibration"] = dict(
        method="registered_markers_plus_distributed_grid" if dynamic_grid else
               ("registered_marker_vertices" if marker_reference is not None else "four_decoded_arucos"),
        reference_id=marker_reference.get("reference_id") if marker_reference is not None else None,
        calibration_frame_rate=len(valid_quads)/frame_count,
        four_aruco_detection_rate=complete_rate,
        used_marker_ids=sorted({marker_id for result in registrations if result.get("available") for marker_id in result["used_marker_ids"]}),
        metric_accuracy_verified=False,
        video_time_basis='decoded_presentation_timestamps',
    )
    pose.attrs['body_geometry'] = dict(applied=body_geometry is not None,
        center_kind=('axle' if body_geometry.get('axle_offset_from_marker_midpoint_mm') is not None else 'marker_midpoint')
                    if body_geometry else 'marker_midpoint', configuration=body_geometry)
    return pose, fixed_quad, fps, complete_rate


def log_elapsed_seconds(raw: pd.DataFrame) -> np.ndarray:
    """Use recorded milliseconds, relative to the first row, including tick wrap."""
    key = "time_ms" if "time_ms" in raw else "cnt"
    ticks = raw[key].to_numpy(float)
    if not len(ticks) or not np.isfinite(ticks).all():
        raise ValueError("log timestamps must be finite and nonempty")
    steps = np.diff(ticks)
    if key == "time_ms":
        steps = np.mod(steps, 2 ** 32)
        if np.any(steps >= 2 ** 31):
            raise ValueError("log timestamps moved backwards")
    if np.any(steps <= 0):
        raise ValueError("log timestamps must be strictly increasing")
    return np.concatenate(([0.0], np.cumsum(steps))) * .001


def profile_log_time_s(metrics: dict, boundary: str) -> float:
    """New metrics record seconds; legacy metrics used 1 ms row indexes."""
    key = f"log_omega_{boundary}_time_s"
    if key in metrics:
        return float(metrics[key])
    return .001 * float(metrics[f"log_omega_{boundary}_frame"])


def integrate_log(path: Path) -> pd.DataFrame:
    raw = pd.read_csv(path)
    time_s = log_elapsed_seconds(raw)
    velocity_mm_s = raw["ego.velo"].to_numpy(float) * 1000.0
    omega = raw["ego.rad_velo"].to_numpy(float)
    x = np.zeros(len(raw))
    y = np.zeros(len(raw))
    heading = np.zeros(len(raw))
    for i in range(1, len(raw)):
        dt = time_s[i] - time_s[i - 1]
        heading[i] = heading[i - 1] + .5 * (omega[i - 1] + omega[i]) * dt
        theta_mid = .5 * (heading[i - 1] + heading[i])
        speed_mid = .5 * (velocity_mm_s[i - 1] + velocity_mm_s[i])
        x[i] = x[i - 1] - speed_mid * math.sin(theta_mid) * dt
        y[i] = y[i - 1] + speed_mid * math.cos(theta_mid) * dt
    return pd.DataFrame(
        {
            "time_log_s": time_s,
            "x_local_mm": x,
            "y_local_mm": y,
            "heading_local_rad": heading,
            "speed_log_mm_s": velocity_mm_s,
            "omega_log_rad_s": omega,
            "ideal_omega_rad_s": raw["ideal.rad_velo"].to_numpy(float),
        }
    )


def measure_log_length_phases(path: Path) -> dict[str, float | int]:
    """Measure Lstart/Lend-like distances from the log's length resets."""
    raw = pd.read_csv(path)
    time_s = log_elapsed_seconds(raw)
    ideal_length = raw["ideal.length"].to_numpy(float)
    ego_length = raw["ego.length"].to_numpy(float)
    ideal_omega = raw["ideal.rad_velo"].to_numpy(float)
    active_indices = np.flatnonzero(np.abs(ideal_omega) > .03)
    cuts = np.flatnonzero(np.diff(active_indices) > 1) + 1
    groups = np.split(active_indices, cuts)
    # The commanded turn is the group with the largest angular-rate peak;
    # small opposite-sign post-turn ringing forms a separate group.
    turn_group = max(groups, key=lambda g: np.max(np.abs(ideal_omega[g])))
    omega_start, omega_end = int(turn_group[0]), int(turn_group[-1])
    resets = np.flatnonzero(np.diff(ideal_length) < -5.0) + 1
    reset_before = int(resets[resets < omega_start][-1])
    reset_after = int(resets[resets > omega_end][0])
    reset_next = int(resets[resets > reset_after][0])
    return {
        "log_front_reset_frame": reset_before,
        "log_omega_start_frame": omega_start,
        "log_omega_end_frame": omega_end,
        "log_omega_start_time_s": float(time_s[omega_start]),
        "log_omega_end_time_s": float(time_s[omega_end]),
        "log_rear_reset_frame": reset_after,
        "log_rear_next_reset_frame": reset_next,
        "log_measured_front_distance_mm": float(
            np.max(ego_length[reset_before:omega_start])
        ),
        "log_ideal_front_distance_mm": float(
            np.max(ideal_length[reset_before:omega_start])
        ),
        "log_measured_rear_distance_mm": float(
            np.max(ego_length[reset_after:reset_next])
        ),
        "log_ideal_rear_distance_mm": float(
            np.max(ideal_length[reset_after:reset_next])
        ),
    }


def rigid_fit(source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
    source_c = source - source.mean(axis=0)
    target_c = target - target.mean(axis=0)
    u, _, vt = np.linalg.svd(source_c.T @ target_c)
    rotation = u @ vt
    if np.linalg.det(rotation) < 0:
        u[:, -1] *= -1
        rotation = u @ vt
    translation = target.mean(axis=0) - source.mean(axis=0) @ rotation
    fitted = source @ rotation + translation
    rmse = float(np.sqrt(np.mean(np.sum((fitted - target) ** 2, axis=1))))
    return rotation, translation, rmse


def central_turn_circle(
    x: np.ndarray, y: np.ndarray, heading: np.ndarray
) -> tuple[np.ndarray, float, float]:
    """Fit a circle to the central 70% of the observed heading change."""
    start = float(np.median(heading[:20]))
    end = float(np.median(heading[-20:]))
    low, high = sorted([start + .15 * (end - start), start + .85 * (end - start)])
    keep = (heading >= low) & (heading <= high)
    x_fit, y_fit = x[keep], y[keep]
    design = np.column_stack([2.0 * x_fit, 2.0 * y_fit, np.ones(len(x_fit))])
    cx, cy, constant = np.linalg.lstsq(design, x_fit ** 2 + y_fit ** 2, rcond=None)[0]
    radius = float(np.sqrt(constant + cx ** 2 + cy ** 2))
    residual = np.hypot(x_fit - cx, y_fit - cy) - radius
    return np.array([cx, cy]), radius, float(np.std(residual))


def estimate_led_front_rear_distances(pose: pd.DataFrame) -> dict[str, float | int]:
    """Estimate straight distances inside the LED-on interval around the arc."""
    led_indices = np.flatnonzero(pose["turn_led_active"].to_numpy(bool))
    if not len(led_indices):
        raise RuntimeError("turn LED interval was not detected")
    led_lo, led_hi = int(led_indices[0]), int(led_indices[-1])
    x = pose.center_x_mm.to_numpy()
    y = pose.center_y_mm.to_numpy()
    heading = pose.heading_rad.to_numpy()
    circle_centre, radius, circle_residual = central_turn_circle(x, y, heading)
    start_heading = float(np.median(heading[:20]))
    end_heading = float(np.median(heading[-20:]))
    direction = float(np.sign(end_heading - start_heading))
    start_forward = np.array([math.cos(start_heading), math.sin(start_heading)])
    end_forward = np.array([math.cos(end_heading), math.sin(end_heading)])
    start_right = np.array([start_forward[1], -start_forward[0]])
    end_right = np.array([end_forward[1], -end_forward[0]])
    entry_tangent = circle_centre + direction * radius * start_right
    exit_tangent = circle_centre + direction * radius * end_right
    led_start = np.array([x[led_lo], y[led_lo]])
    led_end = np.array([x[led_hi], y[led_hi]])
    front_distance = float(np.dot(entry_tangent - led_start, start_forward))
    rear_distance = float(np.dot(led_end - exit_tangent, end_forward))
    step = np.hypot(np.diff(x), np.diff(y))
    front_resolution = float(np.median(step[max(0, led_lo - 2):led_lo + 2]) / 2.0)
    rear_resolution = float(np.median(step[max(0, led_hi - 2):led_hi + 2]) / 2.0)
    return {
        "led_start_frame": led_lo,
        "led_end_frame": led_hi,
        "led_start_video_s": float(pose.time_video_s.iloc[led_lo]),
        "led_end_video_s": float(pose.time_video_s.iloc[led_hi]),
        "led_duration_video_s": float(
            pose.time_video_s.iloc[led_hi] - pose.time_video_s.iloc[led_lo]
        ),
        "front_distance_estimate_mm": front_distance,
        "rear_distance_estimate_mm": rear_distance,
        "front_boundary_resolution_mm": front_resolution,
        "rear_boundary_resolution_mm": rear_resolution,
        "turn_circle_fit_residual_std_mm": circle_residual,
        "entry_tangent_x_mm": float(entry_tangent[0]),
        "entry_tangent_y_mm": float(entry_tangent[1]),
        "exit_tangent_x_mm": float(exit_tangent[0]),
        "exit_tangent_y_mm": float(exit_tangent[1]),
    }


def estimate_profile_matched_lengths(
    pose: pd.DataFrame,
    log_lengths: dict[str, float | int],
    slow_factor: float,
    video_time_zero: float,
    fps: float,
    target_leg_mm: float = 90.0,
) -> dict[str, float]:
    """Solve Lstart/Lend from the slip-inclusive image displacement.

    The body segment is cut at the main ideal-omega profile boundaries mapped
    onto video time. The marker trajectory is the physical chassis motion, so
    lateral slip and the non-ideal final heading are inherently included.
    """
    time_s = pose.time_video_s.to_numpy()
    x = pose.center_x_mm.to_numpy()
    y = pose.center_y_mm.to_numpy()
    heading = np.unwrap(pose.heading_rad.to_numpy())
    video_start = video_time_zero + profile_log_time_s(log_lengths, "start") * slow_factor
    video_end = video_time_zero + profile_log_time_s(log_lengths, "end") * slow_factor
    start_heading = float(np.median(heading[:20]))
    end_heading = float(np.median(heading[-20:]))
    start_forward = np.array([math.cos(start_heading), math.sin(start_heading)])
    end_forward = np.array([math.cos(end_heading), math.sin(end_heading)])
    basis = np.column_stack([start_forward, end_forward])
    target = target_leg_mm * start_forward + target_leg_mm * end_forward

    def solve(start_s: float, end_s: float) -> np.ndarray:
        start = np.array([np.interp(start_s, time_s, x), np.interp(start_s, time_s, y)])
        end = np.array([np.interp(end_s, time_s, x), np.interp(end_s, time_s, y)])
        return np.linalg.solve(basis, target - (end - start))

    estimate = solve(video_start, video_end)
    samples = np.asarray(
        [
            solve(video_start + start_shift / fps, video_end + end_shift / fps)
            for start_shift in (-.5, 0.0, .5)
            for end_shift in (-.5, 0.0, .5)
        ]
    )
    return {
        "profile_video_start_s": video_start,
        "profile_video_end_s": video_end,
        "slip_inclusive_image_lstart_mm": float(estimate[0]),
        "slip_inclusive_image_lend_mm": float(estimate[1]),
        "slip_inclusive_image_lstart_half_frame_min_mm": float(samples[:, 0].min()),
        "slip_inclusive_image_lstart_half_frame_max_mm": float(samples[:, 0].max()),
        "slip_inclusive_image_lend_half_frame_min_mm": float(samples[:, 1].min()),
        "slip_inclusive_image_lend_half_frame_max_mm": float(samples[:, 1].max()),
    }


def fit_time_and_pose(
    video: pd.DataFrame, log: pd.DataFrame, fixed_slow_factor: float | None = None,
    required_log_interval: tuple[float, float] | None = None,
):
    video_t = video["time_video_s"].to_numpy()
    video_xy = video[["center_x_mm", "center_y_mm"]].to_numpy()
    raw_valid = video['marker_detected_raw'].to_numpy(bool) if 'marker_detected_raw' in video else np.ones(len(video), bool)
    log_t = log["time_log_s"].to_numpy()
    log_xy = log[["x_local_mm", "y_local_mm"]].to_numpy()
    best = None
    offset_max = min(3.2, max(.5, float(video_t[-1] - .2)))
    slow_factors = (
        [fixed_slow_factor]
        if fixed_slow_factor is not None
        else np.linspace(6.5, 9.5, 121)
    )
    for slow_factor in slow_factors:
        # A one-second trim margin becomes slightly shorter than one second
        # after Pixel slow-motion playback is mapped back to log time.  Keep a
        # wider negative range so the optimum does not stick to the old -0.5 s
        # boundary when Lstart/Lend are both zero.
        offsets = np.linspace(-1.5, offset_max, 226)
        if fixed_slow_factor is not None:
            offsets = np.linspace(video_t[0]-slow_factor*log_t[-1], video_t[-1]-slow_factor*log_t[0], 501)
        if required_log_interval is not None:
            lo, hi = required_log_interval
            if not np.isfinite([lo, hi]).all() or hi <= lo:
                raise ValueError('invalid required log interval')
            offset_lo = video_t[0]-slow_factor*lo
            offset_hi = video_t[-1]-slow_factor*hi
            if offset_hi < offset_lo:
                continue
            offsets = np.linspace(offset_lo, offset_hi, 501)
        for offset_s in offsets:
            mapped_t = (video_t - offset_s) / slow_factor
            keep = (mapped_t >= log_t[0]) & (mapped_t <= log_t[-1]) & raw_valid
            if keep.sum() < 20:
                continue
            source = np.column_stack(
                [np.interp(mapped_t[keep], log_t, log_xy[:, axis]) for axis in range(2)]
            )
            rotation, translation, rmse = rigid_fit(source, video_xy[keep])
            if best is None or rmse < best[0]:
                best = (rmse, slow_factor, offset_s, rotation, translation, keep, mapped_t)
    if best is None:
        raise RuntimeError("video/log time alignment failed")
    rmse, factor, offset, rotation, translation, keep, mapped_t = best
    source_all = np.column_stack(
        [np.interp(mapped_t[keep], log_t, log_xy[:, axis]) for axis in range(2)]
    )
    fitted = source_all @ rotation + translation
    comparison = video.loc[
        keep,
        [
            "frame", "time_video_s", "center_x_mm", "center_y_mm",
            "heading_rad", "speed_playback_mm_s", "turn_led_blue_pixels",
            "turn_led_active",
        ],
    ].copy()
    comparison["time_log_s"] = mapped_t[keep]
    comparison["log_x_mm"] = fitted[:, 0]
    comparison["log_y_mm"] = fitted[:, 1]
    comparison["position_error_mm"] = np.linalg.norm(fitted - video_xy[keep], axis=1)
    comparison["video_speed_true_mm_s"] = comparison["speed_playback_mm_s"] * factor
    comparison["log_speed_mm_s"] = np.interp(
        comparison["time_log_s"], log_t, log["speed_log_mm_s"]
    )
    return comparison, factor, offset, rotation, translation, rmse


def draw_outputs(
    video_path: Path,
    pose: pd.DataFrame,
    comparison: pd.DataFrame,
    quad: np.ndarray,
    output: Path,
    raw_orientation: bool = False,
):
    led_geometry = estimate_led_front_rear_distances(pose)
    cap = cv2.VideoCapture(str(video_path))
    if raw_orientation:
        cap.set(cv2.CAP_PROP_ORIENTATION_AUTO, 0)
    cap.set(cv2.CAP_PROP_POS_FRAMES, int(pose.frame.iloc[len(pose)//2]))
    ok, frame = cap.read()
    cap.release()
    if not ok:
        raise RuntimeError("cannot read representative video frame")
    homography_inv = cv2.getPerspectiveTransform(WORLD_CORNERS, quad)
    video_px = transform_points(
        comparison[["center_x_mm", "center_y_mm"]].to_numpy(), homography_inv
    ).round().astype(int)
    log_px = transform_points(comparison[["log_x_mm", "log_y_mm"]].to_numpy(), homography_inv)
    log_px = log_px.round().astype(int)
    cv2.polylines(frame, [log_px], False, (30, 70, 245), 5, cv2.LINE_AA)
    led_active = comparison["turn_led_active"].to_numpy(bool)
    # Draw adjacent video segments separately, retaining the shared boundary
    # point so the trajectory remains visually continuous.
    cv2.polylines(frame, [video_px], False, (40, 220, 40), 8, cv2.LINE_AA)
    led_indices = np.flatnonzero(led_active)
    if len(led_indices):
        led_lo = max(0, int(led_indices[0] - 1))
        led_hi = min(len(video_px), int(led_indices[-1] + 2))
        cv2.polylines(
            frame, [video_px[led_lo:led_hi]], False, (220, 40, 220), 10, cv2.LINE_AA
        )
    tangent_px = transform_points(
        np.array(
            [
                [led_geometry["entry_tangent_x_mm"], led_geometry["entry_tangent_y_mm"]],
                [led_geometry["exit_tangent_x_mm"], led_geometry["exit_tangent_y_mm"]],
            ]
        ),
        homography_inv,
    ).round().astype(int)
    for point in tangent_px:
        cv2.drawMarker(
            frame, tuple(point), (255, 255, 255),
            cv2.MARKER_TILTED_CROSS, 24, 4, cv2.LINE_AA,
        )
    # Draw a real legend with colour samples so it remains unambiguous when
    # copied into a report or viewed independently.
    cv2.rectangle(frame, (22, 18), (555, 188), (20, 20, 20), -1)
    cv2.rectangle(frame, (22, 18), (555, 188), (230, 230, 230), 2)
    cv2.line(frame, (45, 47), (115, 47), (40, 220, 40), 8, cv2.LINE_AA)
    cv2.putText(frame, "VIDEO: LED OFF", (135, 55),
                cv2.FONT_HERSHEY_SIMPLEX, .65, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.line(frame, (45, 84), (115, 84), (220, 40, 220), 9, cv2.LINE_AA)
    cv2.putText(frame, "VIDEO: TURN LED ON", (135, 92),
                cv2.FONT_HERSHEY_SIMPLEX, .65, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.line(frame, (45, 121), (115, 121), (30, 70, 245), 6, cv2.LINE_AA)
    cv2.putText(frame, "LOG: INTEGRATED v / omega", (135, 129),
                cv2.FONT_HERSHEY_SIMPLEX, .65, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(
        frame,
        (
            f"VIDEO GEOM. FRONT {led_geometry['front_distance_estimate_mm']:.1f} mm"
            f"   REAR {led_geometry['rear_distance_estimate_mm']:.1f} mm"
        ),
        (42, 169), cv2.FONT_HERSHEY_SIMPLEX, .66,
        (255, 255, 255), 2, cv2.LINE_AA,
    )
    cv2.imwrite(str(output / "trajectory_overlay.jpg"), frame)

    fig, axes = plt.subplots(2, 2, figsize=(12, 9))
    axes[0, 0].plot(pose.center_x_mm, pose.center_y_mm, color="green", label="video: LED off")
    led_pose = pose[pose.turn_led_active]
    axes[0, 0].plot(
        led_pose.center_x_mm, led_pose.center_y_mm,
        color="magenta", linewidth=3, label="video: turn LED on",
    )
    axes[0, 0].plot(comparison.log_x_mm, comparison.log_y_mm, color="red", label="log")
    axes[0, 0].set_aspect("equal")
    axes[0, 0].set_xlabel("field x [mm]")
    axes[0, 0].set_ylabel("field y [mm]")
    axes[0, 0].legend()
    axes[0, 0].grid()
    axes[0, 1].plot(comparison.time_log_s, comparison.video_speed_true_mm_s, label="video")
    axes[0, 1].plot(comparison.time_log_s, comparison.log_speed_mm_s, label="log")
    axes[0, 1].set_xlabel("aligned log time [s]")
    axes[0, 1].set_ylabel("speed [mm/s]")
    axes[0, 1].legend()
    axes[0, 1].grid()
    axes[1, 0].plot(comparison.time_log_s, comparison.position_error_mm)
    axes[1, 0].set_xlabel("aligned log time [s]")
    axes[1, 0].set_ylabel("position error [mm]")
    axes[1, 0].grid()
    axes[1, 1].plot(pose.time_video_s, np.degrees(pose.heading_rad))
    axes[1, 1].set_xlabel("video playback time [s]")
    axes[1, 1].set_ylabel("marker heading [deg]")
    axes[1, 1].grid()
    fig.tight_layout()
    fig.savefig(output / "marker_log_comparison.png", dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--video", type=Path, required=True)
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--front-color", choices=("blue", "red", "magenta"), default="blue"
    )
    parser.add_argument(
        "--calibration-video",
        type=Path,
        help="fallback source video containing all four ArUcos",
    )
    parser.add_argument("--marker-reference", type=Path,
                        help="registered marker reference JSON; use any two or more matching ArUcos")
    parser.add_argument('--body-geometry', type=Path,
                        help='marker heights, camera centre and optional axle offset JSON; missing camera geometry is rejected')
    parser.add_argument('--camera-height-mm', type=float,
                        help='camera optical centre height above board, in mm, for this measurement')
    parser.add_argument('--camera-foot-mm', type=float, nargs=2, metavar=('X', 'Y'),
                        help='point vertically below camera, in board mm; not necessarily image centre')
    parser.add_argument('--video-start', type=float, default=0, help='start PTS in seconds')
    parser.add_argument('--video-end', type=float, help='end PTS in seconds; decode only this analysis window')
    parser.add_argument(
        "--slow-factor",
        type=float,
        help="fixed slow-motion playback factor; otherwise fit within 6.5–9.5",
    )
    args = parser.parse_args()
    if args.body_geometry is None and (args.camera_height_mm is not None or args.camera_foot_mm is not None):
        parser.error('--camera-height-mm and --camera-foot-mm require --body-geometry')
    args.output.mkdir(parents=True, exist_ok=True)
    marker_reference = json.loads(args.marker_reference.read_text(encoding="utf-8")) if args.marker_reference else None
    body_geometry = json.loads(args.body_geometry.read_text(encoding='utf-8')) if args.body_geometry else None
    if body_geometry is not None:
        body_geometry = geometry_for_measurement(body_geometry, args.camera_height_mm, args.camera_foot_mm)
        body_geometry['measurement_video_file'] = str(args.video.resolve())
        body_geometry['source_geometry_file'] = str(args.body_geometry.resolve())
        (args.output/'body_geometry_used.json').write_text(json.dumps(body_geometry, ensure_ascii=False, indent=2), encoding='utf-8')
    if marker_reference is not None:
        (args.output / "marker_reference_used.json").write_text(json.dumps(marker_reference, ensure_ascii=False, indent=2), encoding="utf-8")

    pose, quad, fps, aruco_rate = read_video_pose(
        args.video, args.front_color, args.calibration_video, marker_reference, body_geometry,
        (args.video_start, args.video_end),
        marker_detection=body_geometry.get('marker_detection') if body_geometry else None
    )
    log = integrate_log(args.log)
    log_lengths = measure_log_length_phases(args.log)
    comparison, factor, offset, rotation, translation, fit_rmse = fit_time_and_pose(
        pose, log, args.slow_factor
    )
    pose.to_csv(args.output / "video_marker_pose.csv", index=False)
    comparison.to_csv(args.output / "video_log_comparison.csv", index=False)
    draw_outputs(args.video, pose, comparison, quad, args.output, raw_orientation=marker_reference is not None)

    omega_active = np.flatnonzero(np.abs(log["ideal_omega_rad_s"].to_numpy()) > .03)
    turn_lo, turn_hi = omega_active[0], omega_active[-1]
    turn_mask = (
        (comparison.time_log_s >= log.time_log_s.iloc[turn_lo])
        & (comparison.time_log_s <= log.time_log_s.iloc[turn_hi])
    )
    moving_mask = comparison.log_speed_mm_s.abs() > 100.0
    speed_error = (
        comparison.loc[moving_mask, "video_speed_true_mm_s"]
        - comparison.loc[moving_mask, "log_speed_mm_s"]
    )
    _, video_radius, _ = central_turn_circle(
        pose.center_x_mm.to_numpy(),
        pose.center_y_mm.to_numpy(),
        pose.heading_rad.to_numpy(),
    )
    _, log_radius, _ = central_turn_circle(
        log.x_local_mm.to_numpy(),
        log.y_local_mm.to_numpy(),
        log.heading_local_rad.to_numpy(),
    )
    led_geometry = estimate_led_front_rear_distances(pose)
    settings_path = args.log.with_name(args.log.stem + ".settings.json")
    settings = (
        json.loads(settings_path.read_text(encoding="utf-8"))
        if settings_path.exists() else {}
    )
    motion_name = str(settings.get("motion_name", "unknown"))
    if motion_name in {"long_r90", "long_l90"}:
        profile_lengths = estimate_profile_matched_lengths(
            pose, log_lengths, factor, offset, fps
        )
        profile_report = (
            "- 画像実軌跡（滑り込み）を角速度プロファイルへ合わせた必要値: "
            f"`Lstart` {profile_lengths['slip_inclusive_image_lstart_mm']:.2f} mm / "
            f"`Lend` {profile_lengths['slip_inclusive_image_lend_mm']:.2f} mm"
        )
    else:
        profile_lengths = {}
        profile_report = (
            "- 90 x 90 mm変位を仮定する滑り込みLstart/Lend推定: "
            f"`{motion_name}` には非適用"
        )
    metrics = {
        "video": args.video.name,
        "log": args.log.name,
        "motion_name": motion_name,
        "front_marker_color": args.front_color,
        "video_fps": fps,
        "video_frames": len(pose),
        "marker_detection_rate": float(pose.marker_detected_raw.mean()),
        "aruco_complete_frame_rate": aruco_rate,
        "marker_reference_file": str(args.marker_reference) if args.marker_reference else None,
        "marker_calibration": pose.attrs["marker_calibration"],
        'body_geometry': pose.attrs['body_geometry'],
        "marker_separation_mean_mm": float(pose.marker_separation_mm.mean()),
        "marker_separation_std_mm": float(pose.marker_separation_mm.std()),
        "slow_motion_factor": factor,
        "log_time_zero_at_video_s": offset,
        "trajectory_fit_rmse_mm": fit_rmse,
        "trajectory_fit_median_error_mm": float(comparison.position_error_mm.median()),
        "trajectory_fit_max_error_mm": float(comparison.position_error_mm.max()),
        "turn_fit_rmse_mm": float(np.sqrt(np.mean(comparison.loc[turn_mask, "position_error_mm"] ** 2))),
        "moving_speed_rmse_mm_s": float(np.sqrt(np.mean(speed_error ** 2))),
        "moving_speed_correlation": float(
            comparison.loc[moving_mask, ["video_speed_true_mm_s", "log_speed_mm_s"]]
            .corr().iloc[0, 1]
        ),
        "video_central_turn_radius_mm": video_radius,
        "log_central_turn_radius_mm": log_radius,
        "central_turn_radius_difference_mm": video_radius - log_radius,
        "turn_led_detection_threshold_pixels": float(pose.attrs["turn_led_threshold"]),
        **led_geometry,
        **log_lengths,
        **profile_lengths,
        "video_heading_change_deg": float(
            np.degrees(pose.heading_rad.iloc[-20:].median() - pose.heading_rad.iloc[:20].median())
        ),
        "log_heading_change_deg": float(np.degrees(log.heading_local_rad.iloc[-1])),
        "log_turn_angle_deg": float(
            np.degrees(
                np.trapz(
                    log.omega_log_rad_s.iloc[turn_lo:turn_hi + 1],
                    log.time_log_s.iloc[turn_lo:turn_hi + 1],
                )
            )
        ),
        "video_endpoint_x_mm": float(pose.center_x_mm.iloc[-20:].median()),
        "video_endpoint_y_mm": float(pose.center_y_mm.iloc[-20:].median()),
    }
    (args.output / "metrics.json").write_text(
        json.dumps(metrics, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    front_label = {"blue": "青", "red": "赤", "magenta": "マゼンタ"}[args.front_color]
    report = f"""# {front_label}・黄マーカー動画と走行ログの比較

- 対象: `{args.video.name}` / `{args.log.name}`
- {front_label}・黄マーカー検出率: {metrics['marker_detection_rate'] * 100:.1f} %
- 4個のArUco同時検出率: {metrics['aruco_complete_frame_rate'] * 100:.1f} %
- 較正方式: {metrics['marker_calibration']['method']} / 較正成立フレーム率 {metrics['marker_calibration']['calibration_frame_rate'] * 100:.1f} %
- マーカー間隔: {metrics['marker_separation_mean_mm']:.2f} ± {metrics['marker_separation_std_mm']:.2f} mm
- 推定スローモーション倍率: {factor:.3f} 倍
- 動画とログの軌跡RMSE: {fit_rmse:.2f} mm（中央値 {metrics['trajectory_fit_median_error_mm']:.2f} mm）
- ターン区間RMSE: {metrics['turn_fit_rmse_mm']:.2f} mm
- 走行中の速度RMSE: {metrics['moving_speed_rmse_mm_s']:.1f} mm/s（相関 {metrics['moving_speed_correlation']:.4f}）
- 中央ターン半径: 動画 {video_radius:.2f} mm / ログ {log_radius:.2f} mm（差 {video_radius - log_radius:+.2f} mm）
- LED点灯区間: {led_geometry['led_start_video_s']:.3f}～{led_geometry['led_end_video_s']:.3f} s（動画時間）
- 動画のLED端→円弧接点（幾何推定）: 前 {led_geometry['front_distance_estimate_mm']:.2f} mm / 後 {led_geometry['rear_distance_estimate_mm']:.2f} mm
- ログのlengthリセット区間（`ego.length`）: 前 {log_lengths['log_measured_front_distance_mm']:.2f} mm / 後 {log_lengths['log_measured_rear_distance_mm']:.2f} mm
- 同区間の`ideal.length`: 前 {log_lengths['log_ideal_front_distance_mm']:.2f} mm / 後 {log_lengths['log_ideal_rear_distance_mm']:.2f} mm
{profile_report}
- 動画マーカー方位変化: {metrics['video_heading_change_deg']:.2f} deg
- ログ積分方位変化: {metrics['log_heading_change_deg']:.2f} deg

緑線は動画から得た{front_label}・黄マーカーの中点、赤線はログの `ego.velo` と
`ego.rad_velo` を積分し、時間・剛体位置合わせした軌跡です。位置合わせで
距離尺度は変更していません。

動画の幾何推定値は、LED点灯開始位置から円弧入口接線点まで、および円弧
出口接線点からLED消灯位置までを進行方向へ射影した値です。ログ値は
`ideal.length` のリセットから角速度立ち上がりまで、および角速度終了後の
次のlengthリセットまでにおける `ego.length` の最大値です。定義が異なる
ため、両者を分けて表示しています。

滑り込み画像値は、`ideal.rad_velo` の主プロファイル開始・終了を動画時刻へ
写し、その間のマーカー実変位をターン本体としたとき、90 x 90 mmの目標
変位を満たす前後直線長を解いた値です。
"""
    (args.output / "README.md").write_text(report, encoding="utf-8")


if __name__ == "__main__":
    main()
