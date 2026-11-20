"""動画の実フレームから9×9の赤い壁候補を求める。機体の地図へは反映しない。"""
from __future__ import annotations

import cv2
import numpy as np


def wall_observations(frame, image_to_cell):
    """柱の両端を避け、辺の中央を赤壁が連続して覆う割合を観測する。"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hue, saturation, value = cv2.split(hsv)
    red = (((hue <= 12) | (hue >= 160)) & (saturation >= 70) & (value >= 45)).astype(np.uint8)*255
    matrix = np.diag([100., 100., 1.]) @ image_to_cell
    size = (901, 901)
    mask = cv2.warpPerspective(red, matrix, size, flags=cv2.INTER_NEAREST)
    visible = cv2.warpPerspective(np.full(frame.shape[:2], 255, np.uint8), matrix, size, flags=cv2.INTER_NEAREST)
    rectified_value = cv2.warpPerspective(value, matrix, size, flags=cv2.INTER_NEAREST)
    if np.any(visible) and np.median(rectified_value[visible > 0]) < 100:
        white = ((saturation <= 65) & (value >= 120)).astype(np.uint8)*255
        mask |= cv2.warpPerspective(white, matrix, size, flags=cv2.INTER_NEAREST)
    if np.mean(mask > 0) > .35:
        return {}
    observations = {}
    for axis in ('H', 'V'):
        for line in range(10):
            for segment in range(9):
                lo, hi = segment*100+22, segment*100+79
                start, end = max(0, line*100-12), min(901, line*100+13)
                region = (slice(start, end), slice(lo, hi)) if axis == 'H' else (slice(lo, hi), slice(start, end))
                if np.mean(visible[region] > 0) < .98:
                    continue
                cross_axis = 0 if axis == 'H' else 1
                coverage = np.mean(np.count_nonzero(mask[region], axis=cross_axis) >= 2)
                observations[(axis, line, segment)] = float(coverage)
    return observations


def detect_video_walls(video, reference, register, detector, samples=16):
    """各サンプルでArUcoを実復号する。基準JSON内の過去の壁を代用しない。"""
    capture = cv2.VideoCapture(str(video))
    if not capture.isOpened():
        raise ValueError('壁検出用の動画を開けません: '+str(video))
    observations, frames = {}, []
    try:
        capture.set(cv2.CAP_PROP_ORIENTATION_AUTO, 0)
        count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
        indices = np.unique(np.linspace(0, max(0, count-1), min(samples, max(1, count)), dtype=int))
        aruco = detector()
        for index in indices:
            capture.set(cv2.CAP_PROP_POS_FRAMES, int(index))
            ok, frame = capture.read()
            if not ok:
                continue
            matrix, quality = register(frame, aruco, reference)
            if matrix is None:
                continue
            current = wall_observations(frame, matrix)
            if not current:
                continue
            frames.append(dict(frame=int(index), time_video_s=float(capture.get(cv2.CAP_PROP_POS_MSEC))/1000,
                               marker_ids=quality['marker_ids']))
            for key, coverage in current.items():
                observations.setdefault(key, []).append(coverage)
    finally:
        capture.release()
    walls = []
    for axis in ('H', 'V'):
        for line in range(10):
            for segment in range(9):
                values = observations.get((axis, line, segment), [])
                present, absent = sum(v >= .65 for v in values), sum(v <= .08 for v in values)
                state = 'wall' if present >= 2 else ('open' if absent >= 3 and present == 0
                                                    and all(v <= .08 for v in values) else 'unknown')
                walls.append(dict(axis=axis, grid_line=line, cell_segment=segment, state=state,
                                  observations=len(values), wall_votes=present, open_votes=absent))
    metadata = dict(selection='current_video_wall_candidates', source=str(video),
                    revision='この動画から検出した赤・白壁候補', edge_count=180,
                    counts={state:sum(w['state'] == state for w in walls) for state in ('wall', 'open', 'unknown')},
                    frames=frames, physical_layout_verified=False,
                    limitations=['黒い床上の赤・白壁が対象。遮蔽・反射で未検出になり得る。壁候補は元動画で照合してください'])
    return walls, metadata
