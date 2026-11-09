"""基準画像と共通する任意のArUco 2枚以上の頂点から盤面を射影補正する。"""
from __future__ import annotations

import hashlib
import json

CORNER_NAMES = ("TL", "TR", "BR", "BL")


def refine_marker_vertices(frame, markers):
    """復号済みの頂点を元画像で精密化する。新しいIDは作らない。"""
    import copy
    import cv2
    import numpy as np
    result = copy.deepcopy(markers)
    marker_corners_by_id(result)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    for item in result.values():
        refined, accepted_ids, rejected = [], [], []
        item["decoded_ids"] = list(item.get("ids", []))
        item["unrefined_corners_px"] = copy.deepcopy(item.get("corners_px", []))
        for marker_id, quad in zip(item.get("ids", []), item.get("corners_px", [])):
            original = np.float32(quad).reshape(4, 1, 2)
            points = original.copy()
            if (points[:, 0, 0].min() < 4 or points[:, 0, 1].min() < 4
                    or points[:, 0, 0].max() >= gray.shape[1]-4
                    or points[:, 0, 1].max() >= gray.shape[0]-4):
                rejected.append(dict(id=marker_id, reason="refinement_image_margin"))
                continue
            cv2.cornerSubPix(gray, points, (3, 3), (-1, -1),
                             (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 40, .001))
            if not np.isfinite(points).all() or np.max(np.linalg.norm(points-original, axis=2)) > 3:
                rejected.append(dict(id=marker_id, reason="refinement_moved_beyond_three_pixel_window"))
                continue
            refined.append(points.reshape(4, 2).tolist())
            accepted_ids.append(marker_id)
        item["corners_px"] = refined
        item["ids"] = accepted_ids
        item["refinement_rejected"] = rejected
        item["corner_refinement"] = "subpix_original_win3"
    marker_corners_by_id(result)
    return result


def detect_registered_vertices(frame, profile):
    """全画面で復号し、未読の登録IDだけ基準近傍でも探索する。"""
    import cv2
    import numpy as np
    from pixel_camera_framing import inspect_rois
    corners, ids, _ = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)).detectMarkers(frame)
    found = {} if ids is None else {int(key): quad.reshape(4, 2).tolist()
                                    for key, quad in zip(ids.ravel(), corners)}
    for key, quad in profile["markers_px"].items():
        marker_id = int(key)
        if marker_id in found:
            continue
        points = np.asarray(quad)
        lo, hi = points.min(axis=0)-45, points.max(axis=0)+45
        box = [max(0, lo[0]), max(0, lo[1]), min(frame.shape[1], hi[0]), min(frame.shape[0], hi[1])]
        if box[2]-box[0] < 12 or box[3]-box[1] < 12:
            continue
        candidates = marker_corners_by_id(inspect_rois(frame, {"search": box}))
        # 別IDを推測して読み替えない。基準近傍は検索範囲であり、頂点を代入しない。
        if marker_id in candidates:
            found[marker_id] = candidates[marker_id].tolist()
    markers = {"full_frame_and_registered_search": dict(ids=list(found), corners_px=list(found.values()))}
    if profile.get("corner_refinement") == "subpix_original_win3":
        markers = refine_marker_vertices(frame, markers)
    return markers


def fit_grid_reference(checkpoints, markers, image_size=(1280, 720), board_side_mm=360.0, source=None,
                       minimum_points=12):
    """複数の既知格子点から基準を作る。検査の合否は別の点・動画で評価する。"""
    import cv2
    import numpy as np
    points = [point for point in checkpoints if point.get("accepted", True)]
    image = np.asarray([[point["image_x_px"], point["image_y_px"]] for point in points], dtype=float)
    world = np.asarray([[point["expected_x_mm"], point["expected_y_mm"]] for point in points], dtype=float)
    if minimum_points < 8:
        raise ValueError("at least eight distributed calibration points are required")
    if (len(points) < minimum_points or not np.isfinite(image).all() or not np.isfinite(world).all()
            or len(np.unique(world, axis=0)) != len(points)
            or np.any(np.ptp(world, axis=0) < board_side_mm*.5)):
        raise ValueError("grid calibration needs enough distinct, finite, widely spread points")
    if np.linalg.matrix_rank(np.column_stack([world, np.ones(len(world))])) < 3:
        raise ValueError("collinear grid calibration points")
    matrix, _ = cv2.findHomography(image, world, method=0)
    if matrix is None or not np.isfinite(matrix).all():
        raise ValueError("grid homography not found")
    world_quad = np.float64([[0, 0], [board_side_mm, 0], [board_side_mm, board_side_mm], [0, board_side_mm]])
    quad = cv2.perspectiveTransform(world_quad.reshape(-1, 1, 2), np.linalg.inv(matrix)).reshape(-1, 2)
    inspection = dict(corners=markers, board_reference=dict(
        available=True, method="measured_45mm_grid", points_px=dict(zip(CORNER_NAMES, quad.tolist()))))
    profile = create_marker_reference(inspection, image_size=image_size, board_side_mm=board_side_mm)
    # create_marker_referenceのfloat32四隅再計算を避け、全格子点による行列を保持する。
    profile["image_to_board_mm"] = (matrix/matrix[2, 2]).tolist()
    profile["corner_refinement"] = "subpix_original_win3"
    profile["source"] = source or {}
    residual = cv2.perspectiveTransform(image.reshape(-1, 1, 2), matrix).reshape(-1, 2)-world
    profile["grid_calibration"] = dict(training_unique_points=len(points),
        training_fit_rmse_mm=float(np.sqrt(np.mean(np.sum(residual**2, axis=1)))),
        training_fit_is_accuracy_validation=False,
        calibrated_range_mm=[world.min(axis=0).tolist(), world.max(axis=0).tolist()])
    return profile


def split_grid_checkpoints(checks, config):
    """画像上の残差を見ず、既知格子の配置と可視性だけで学習・検査点を分ける。"""
    import numpy as np
    valid = sorted((point for point in checks if point["accepted"]), key=lambda p: (p["row"], p["col"]))
    if config.get("training_policy", "fixed_indices") == "visible_spread":
        # 遮蔽で固定16点が不足しても、別の見えている格子を使う。検査点は必ず残す。
        count = min(config.get("training_point_count", 10),
                    len(valid)-config.get("minimum_heldout_points", 12))
        if count < config.get("minimum_training_points", 8):
            return [], valid
        world = np.array([[p["expected_x_mm"], p["expected_y_mm"]] for p in valid])
        distance = np.sum((world[:, None]-world[None, :])**2, axis=2)
        first, second = np.unravel_index(np.argmax(distance), distance.shape)
        selected = [int(first), int(second)]
        while len(selected) < count:
            scores = distance[:, selected].min(axis=1)
            scores[selected] = -1
            selected.append(int(np.argmax(scores)))
        selected = set(selected)
        return ([point for index, point in enumerate(valid) if index in selected],
                [point for index, point in enumerate(valid) if index not in selected])
    if config.get("training_policy", "fixed_indices") != "fixed_indices":
        raise ValueError("unknown grid training policy")
    training_ids = {tuple(point) for point in config["training_grid_indices"]}
    return ([p for p in valid if (p["col"], p["row"]) in training_ids],
            [p for p in valid if (p["col"], p["row"]) not in training_ids])


def refine_board_from_checkpoints(marker_result, checks, profile):
    """指定した格子点で補正し、別の格子点でそのフレームを検査する。"""
    import cv2
    import numpy as np
    if not marker_result.get("available"):
        return marker_result
    config = profile.get("grid_refinement", {})
    try:
        training, heldout = split_grid_checkpoints(checks, config)
        vertices = {"actual_registered": dict(ids=marker_result["used_marker_ids"],
            corners_px=[profile["markers_px"][str(key)] for key in marker_result["used_marker_ids"]])}
        fitted = fit_grid_reference(training, vertices, tuple(profile["image_size"]), profile["board_side_mm"],
                                     minimum_points=config.get("minimum_training_points", 8))
    except (ValueError, np.linalg.LinAlgError, cv2.error) as error:
        return dict(available=False, reason="insufficient_grid_calibration", detail=str(error))
    if len(heldout) < config.get("minimum_heldout_points", 12):
        return dict(available=False, reason="insufficient_independent_grid_checks")
    matrix = np.asarray(fitted["image_to_board_mm"])
    image = np.float64([[point["image_x_px"], point["image_y_px"]] for point in heldout])
    expected = np.float64([[point["expected_x_mm"], point["expected_y_mm"]] for point in heldout])
    measured = cv2.perspectiveTransform(image.reshape(-1, 1, 2), matrix).reshape(-1, 2)
    error = np.linalg.norm(measured-expected, axis=1)
    rms, p95 = float(np.sqrt(np.mean(error**2))), float(np.percentile(error, 95))
    quality = dict(training_points=len(training), heldout_points=len(heldout),
                   training_policy=config.get("training_policy", "fixed_indices"),
                   training_grid_indices=[[p["col"], p["row"]] for p in training],
                   heldout_grid_indices=[[p["col"], p["row"]] for p in heldout],
                   position_rmse_mm=rms, position_p95_mm=p95, position_max_mm=float(error.max()),
                   tolerance_rmse_mm=config.get("maximum_heldout_rmse_mm", .75),
                   tolerance_p95_mm=config.get("maximum_heldout_p95_mm", 1.5),
                   tolerance_max_mm=config.get("maximum_heldout_error_mm", 2.0))
    if (rms > quality["tolerance_rmse_mm"] or p95 > quality["tolerance_p95_mm"]
            or error.max() > quality["tolerance_max_mm"]):
        return dict(available=False, reason="independent_grid_accuracy_limit", grid_quality=quality)
    result = dict(marker_result)
    result.update(method="registered_markers_plus_distributed_grid", registration_mode="per_frame_grid",
                  homography_image_to_board_mm=fitted["image_to_board_mm"], points_px=fitted["board_points_px"],
                  point_sources={name: "current_distributed_grid_calibration" for name in CORNER_NAMES},
                  grid_quality=quality, metric_accuracy_evaluated=True,
                  metric_accuracy_verified=False)
    return result


def refine_board_grid(frame, marker_result, profile):
    if not marker_result.get("available") or not profile.get("grid_refinement", {}).get("enabled"):
        return marker_result
    from analysis.pixel_camera_accuracy import find_grid_checkpoints
    checks = find_grid_checkpoints(frame, marker_result["homography_image_to_board_mm"],
                                  profile["grid_refinement"]["pitch_mm"])
    return refine_board_from_checkpoints(marker_result, checks, profile)


def marker_corners_by_id(markers):
    """ROI名や画面上の隅ではなく、実検出IDをキーにする。"""
    import cv2
    import numpy as np
    found = {}
    for item in markers.values():
        ids, quads = item.get("ids", []), item.get("corners_px", [])
        if len(ids) != len(quads):
            raise ValueError("marker IDs and corner arrays do not match")
        for marker_id, quad in zip(ids, quads):
            marker_id = int(marker_id)
            points = np.asarray(quad, dtype=float)
            if marker_id in found:
                raise ValueError("duplicate detected marker ID")
            if points.shape != (4, 2) or not np.isfinite(points).all():
                raise ValueError("invalid marker corners")
            contour = points.astype(np.float32)
            if not cv2.isContourConvex(contour) or cv2.contourArea(contour) < 4:
                raise ValueError("degenerate marker quadrilateral")
            found[marker_id] = points
    return found


def create_marker_reference(inspection, image_size=(1280, 720), board_side_mm=360.0,
                            source_inspection=None, board_corner_ids=None):
    """過去に確定した盤面位置と、その画像で実検出した頂点だけを登録する。"""
    import cv2
    import numpy as np
    reference = inspection.get("board_reference", {})
    if not reference.get("available"):
        raise ValueError("reference image needs an established board calibration")
    found = marker_corners_by_id(inspection["corners"])
    if len(found) < 2:
        raise ValueError("reference image needs at least two decoded markers")
    quad = np.asarray([reference["points_px"][name] for name in CORNER_NAMES], dtype=np.float32)
    if quad.shape != (4, 2) or not np.isfinite(quad).all() or not cv2.isContourConvex(quad) or cv2.contourArea(quad) < 1000:
        raise ValueError("invalid reference board quadrilateral")
    if not np.isfinite(board_side_mm) or board_side_mm <= 0:
        raise ValueError("invalid board size")
    world = np.float32([[0, 0], [board_side_mm, 0], [board_side_mm, board_side_mm], [0, board_side_mm]])
    homography = cv2.getPerspectiveTransform(quad, world)
    profile = dict(schema_version=1, method="matched_marker_vertices", image_size=list(image_size),
                   markers_px={str(key): value.tolist() for key, value in sorted(found.items())},
                   board_points_px={name: quad[index].tolist() for index, name in enumerate(CORNER_NAMES)},
                   board_corner_ids=board_corner_ids or {"TL": 4, "TR": 6, "BR": 5, "BL": 7},
                   board_side_mm=float(board_side_mm), image_to_board_mm=homography.tolist(),
                   board_axes="origin at reference-image TL, x toward TR, y toward BL",
                   calibration_source_method=reference.get("method"),
                   metric_accuracy_verified=bool(reference.get("metric_accuracy_verified", False)),
                   source_inspection_json=str(source_inspection) if source_inspection else None,
                   source=inspection.get("source", {}))
    geometry = {key: profile[key] for key in ("markers_px", "board_points_px", "board_side_mm")}
    profile["reference_id"] = hashlib.sha256(json.dumps(geometry, sort_keys=True).encode("utf-8")).hexdigest()[:20]
    return profile


def register_marker_reference(markers, profile, image_size=(1280, 720)):
    """2枚なら8頂点、3枚なら12頂点。未読ID・固定位置の点は追加しない。"""
    import cv2
    import numpy as np
    try:
        if profile.get("grid_refinement", {}).get("enabled") and profile.get("markers_search_px"):
            # 精密化の成否とは別に、復号済みの元頂点を格子探索の初期位置に使う。
            markers = {name: dict(ids=item.get("decoded_ids", item["ids"]),
                                 corners_px=item.get("unrefined_corners_px", item["corners_px"]))
                       for name, item in markers.items()}
            profile = dict(profile, markers_px=profile["markers_search_px"])
        current = marker_corners_by_id(markers)
        registered = marker_corners_by_id({"reference": dict(
            ids=[int(key) for key in profile["markers_px"]], corners_px=list(profile["markers_px"].values()))})
        common = sorted(set(current) & set(registered))
        if len(common) < 2:
            return dict(available=False, reason="at_least_two_registered_markers_required", matched_marker_ids=common)
        image_points = np.vstack([current[key] for key in common])
        reference_points = np.vstack([registered[key] for key in common])
        if len(common) == 2:
            # 対応が確定した8頂点を全部使う。RANSACで1頂点を落とし、
            # 必要な8点未満にすることは避け、残差で採否を判断する。
            homography, mask = cv2.findHomography(image_points, reference_points, 0)
            estimator = "least_squares_eight_vertices"
        else:
            homography, mask = cv2.findHomography(image_points, reference_points, cv2.RANSAC, 1.5,
                                                  maxIters=2000, confidence=.995)
            estimator = "ransac_marker_vertices"
        if homography is None or mask is None or not np.isfinite(homography).all():
            return dict(available=False, reason="homography_not_found")
        inliers = mask.ravel().astype(bool)
        support = {key: int(inliers[index*4:index*4+4].sum()) for index, key in enumerate(common)}
        if inliers.sum() < 8 or sum(count >= 3 for count in support.values()) < 2:
            return dict(available=False, reason="insufficient_marker_inliers", inlier_corners_by_id=support)
        projected = cv2.perspectiveTransform(image_points.reshape(-1, 1, 2), homography).reshape(-1, 2)
        errors = np.linalg.norm(projected-reference_points, axis=1)
        rms = float(np.sqrt(np.mean(errors[inliers]**2)))
        if rms > 1.0 or errors[inliers].max() > 2.0:
            return dict(available=False, reason="reprojection_error", reprojection_rms_px=rms)
        reference_quad = np.float64([profile["board_points_px"][name] for name in CORNER_NAMES])
        inverse = np.linalg.inv(homography)
        if not np.isfinite(inverse).all():
            return dict(available=False, reason="singular_homography")
        quad = cv2.perspectiveTransform(reference_quad.reshape(-1, 1, 2), inverse).reshape(-1, 2)
        contour = quad.astype(np.float32)
        area = cv2.contourArea(contour)
        if not np.isfinite(quad).all() or not cv2.isContourConvex(contour) or not 1000 < area < image_size[0]*image_size[1]*4:
            return dict(available=False, reason="invalid_projected_board")
        image_to_board = np.asarray(profile["image_to_board_mm"]) @ homography
        image_to_board /= image_to_board[2, 2]
        if not np.isfinite(image_to_board).all():
            return dict(available=False, reason="invalid_board_transform")
        used = [key for key in common if support[key] > 0]
        return dict(available=True, method="registered_marker_vertices", reference_id=profile["reference_id"],
                    estimator=estimator,
                    matched_marker_ids=common, used_marker_ids=used,
                    correspondence_count=len(image_points), inlier_count=int(inliers.sum()),
                    inlier_corners_by_id=support, reprojection_rms_px=rms,
                    max_inlier_error_px=float(errors[inliers].max()),
                    homography_current_to_reference=homography.tolist(),
                    homography_image_to_board_mm=image_to_board.tolist(),
                    board_side_mm=profile["board_side_mm"],
                    points_px={name: quad[index].tolist() for index, name in enumerate(CORNER_NAMES)},
                    point_sources={name: "projected_from_matched_marker_vertices" for name in CORNER_NAMES},
                    board_corner_ids=profile.get("board_corner_ids", {}),
                    metric_accuracy_evaluated=bool(profile.get("metric_accuracy_evaluated", False)),
                    accuracy_evaluation=profile.get("accuracy_evaluation"),
                    metric_accuracy_verified=bool(profile.get("metric_accuracy_verified", False)))
    except (KeyError, TypeError, ValueError, np.linalg.LinAlgError, cv2.error) as error:
        return dict(available=False, reason="invalid_reference_or_corners", error=str(error))


def detect_board_reference(frame, markers, reference_profile=None):
    """盤面の特定の隅や格子線に依存せず、基準画像と共通のマーカーを使う。"""
    if reference_profile is None:
        return dict(available=False, reason="marker_reference_not_configured")
    result = register_marker_reference(markers, reference_profile, (frame.shape[1], frame.shape[0]))
    return refine_board_grid(frame, result, reference_profile)


def rectify_board(frame, correction, output_size=720):
    """保存済みの射影行列で盤面を正方形へ補正する。元動画は変更しない。"""
    import cv2
    import numpy as np
    if not correction.get("available"):
        raise ValueError("marker correction is unavailable")
    if output_size < 32:
        raise ValueError("output size must be at least 32")
    scale = (output_size-1)/correction["board_side_mm"]
    transform = np.diag([scale, scale, 1.0]) @ np.asarray(correction["homography_image_to_board_mm"])
    return cv2.warpPerspective(frame, transform, (output_size, output_size))
