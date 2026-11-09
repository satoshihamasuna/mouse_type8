"""盤面との高さ差を補正する。座標は基準盤面TL原点、x右・y下、z上のmm。"""
from __future__ import annotations

import copy

import numpy as np


def validate_geometry(geometry):
    if (geometry.get('camera_height_policy') == 'per_measurement'
            and geometry.get('camera_height_measurement_confirmed') is not True):
        raise ValueError('カメラ高さは測定ごとに --camera-height-mm で指定してください')
    centre = np.asarray(geometry.get('camera_center_board_mm'), dtype=float)
    if centre.shape != (3,) or not np.isfinite(centre).all() or centre[2] <= 0:
        raise ValueError('高さ補正にはカメラの盤面座標x/yと高さz [mm] が必要です')
    for key in ('front_height_mm', 'rear_height_mm'):
        height = geometry.get(key)
        if height is None or not np.isfinite(height) or not 0 <= height < centre[2]:
            raise ValueError(key + ' must be finite, nonnegative and below camera height')
    span = geometry.get('nominal_marker_span_mm')
    if span is not None and (not np.isfinite(span) or span <= 0):
        raise ValueError('nominal marker span must be finite and positive')
    offset = geometry.get('axle_offset_from_marker_midpoint_mm')
    if offset is not None:
        offset = np.asarray(offset, dtype=float)
        if offset.shape != (2,) or not np.isfinite(offset).all():
            raise ValueError('axle offset must be [forward_mm, left_mm]')
    return centre


def geometry_for_measurement(geometry, camera_height_mm=None, camera_foot_mm=None):
    """機体固有の幾何を保持し、撮影ごとのカメラ条件を別のコピーへ確定する。"""
    result = copy.deepcopy(geometry)
    if result.get('camera_height_policy') == 'per_measurement' and camera_height_mm is None:
        raise ValueError('カメラ高さは測定ごとに --camera-height-mm で指定してください')
    previous = result.get('camera_center_board_mm')
    height = camera_height_mm
    if height is None:
        height = previous[2] if previous is not None else result.get('camera_height_mm')
    if height is None or not np.isfinite(height) or height <= 0:
        raise ValueError('カメラの盤面からの高さを正のmmで指定してください')
    foot = camera_foot_mm
    if foot is None:
        foot = result.get('camera_foot_board_mm')
        if foot is None and previous is not None:
            foot = previous[:2]
    foot = np.asarray(foot, dtype=float)
    if foot.shape != (2,) or not np.isfinite(foot).all():
        raise ValueError('カメラの真下の盤面座標x/y [mm] が必要です。画像中心で自動代用しません')
    result.update(camera_height_mm=float(height), camera_foot_board_mm=foot.tolist(),
        camera_center_board_mm=[float(foot[0]), float(foot[1]), float(height)],
        camera_height_measurement_confirmed=True,
        camera_height_source='measurement_argument' if camera_height_mm is not None else 'geometry_configuration',
        camera_foot_source='measurement_argument' if camera_foot_mm is not None else 'geometry_configuration')
    validate_geometry(result)
    return result


def correct_height(floor_xy, height_mm, camera_center_board_mm):
    """カメラから床上の見かけ位置への光線を、z=height_mmで交差させる。"""
    camera = np.asarray(camera_center_board_mm, dtype=float)
    points = np.asarray(floor_xy, dtype=float)
    if (camera.shape != (3,) or points.shape[-1:] != (2,) or not np.isfinite(camera).all()
            or not np.isfinite(points).all() or camera[2] <= 0
            or not np.isfinite(height_mm) or not 0 <= height_mm < camera[2]):
        raise ValueError('invalid floor point, marker height or camera centre')
    return camera[:2]+(1-height_mm/camera[2])*(points-camera[:2])


def apparent_floor(corrected_xy, height_mm, camera_center_board_mm):
    """補正点を元の床面上の見かけ位置へ戻す（画像上の表示用）。"""
    camera = np.asarray(camera_center_board_mm, dtype=float)
    correct_height(corrected_xy, height_mm, camera)
    return camera[:2]+(np.asarray(corrected_xy)-camera[:2])/(1-height_mm/camera[2])


def pose_from_corrected_markers(front_xy, rear_xy, geometry):
    front, rear = np.asarray(front_xy, dtype=float), np.asarray(rear_xy, dtype=float)
    if front.shape != (2,) or rear.shape != (2,) or not np.isfinite([front, rear]).all():
        raise ValueError('invalid corrected marker coordinates')
    vector = front-rear
    span = float(np.linalg.norm(vector))
    if span < 1:
        raise ValueError('body marker separation below one millimetre')
    midpoint = (front+rear)/2
    unit = vector/span
    axle = None
    offset = geometry.get('axle_offset_from_marker_midpoint_mm')
    if offset is not None:
        offset = np.asarray(offset, dtype=float)
        if offset.shape != (2,) or not np.isfinite(offset).all():
            raise ValueError('axle offset must be finite [forward_mm, left_mm]')
        forward, left = offset
        # 盤面のy軸は画像の下向き。前方が画像下なら機体左は画像右。
        axle = midpoint+forward*unit+left*np.array([unit[1], -unit[0]])
    return dict(front_xy_mm=front, rear_xy_mm=rear, marker_midpoint_xy_mm=midpoint,
                axle_xy_mm=axle, heading_rad=float(np.arctan2(vector[1], vector[0])),
                marker_separation_mm=span,
                center_kind='axle' if axle is not None else 'marker_midpoint')


def correct_body_markers(front_floor_xy, rear_floor_xy, geometry):
    camera = validate_geometry(geometry)
    front = correct_height(front_floor_xy, geometry['front_height_mm'], camera)
    rear = correct_height(rear_floor_xy, geometry['rear_height_mm'], camera)
    return pose_from_corrected_markers(front, rear, geometry)


def relative_marker_midpoint(front_floor_xy, rear_floor_xy, camera_height_mm,
                             front_height_mm=10.0, rear_height_mm=1.0, valid=None):
    """固定カメラ・一定高さの動画内で、真下位置に依存しない中点変位を求める。"""
    front, rear = np.asarray(front_floor_xy, dtype=float), np.asarray(rear_floor_xy, dtype=float)
    if front.ndim != 2 or front.shape[1:] != (2,) or front.shape != rear.shape:
        raise ValueError('front and rear coordinates must be matching N by 2 arrays')
    if (not np.isfinite(camera_height_mm) or camera_height_mm <= 0
            or not np.isfinite([front_height_mm, rear_height_mm]).all()
            or not 0 <= min(front_height_mm, rear_height_mm)
            or max(front_height_mm, rear_height_mm) >= camera_height_mm):
        raise ValueError('invalid camera or marker heights')
    accepted = np.isfinite(front).all(axis=1) & np.isfinite(rear).all(axis=1)
    if valid is not None:
        mask = np.asarray(valid, dtype=bool)
        if mask.shape != (len(front),):
            raise ValueError('valid mask length does not match marker observations')
        accepted &= mask
    indices = np.flatnonzero(accepted)
    if not len(indices):
        raise ValueError('no valid marker observations')
    # 真の中点 = 下式 + (h_front+h_rear)/(2H) * camera_foot。
    # 同じ動画の開始値を引けば、未知の定数camera_footの項は相殺される。
    translated = ((1-front_height_mm/camera_height_mm)*front
                  + (1-rear_height_mm/camera_height_mm)*rear)/2
    relative = translated-translated[indices[0]]
    relative[~accepted] = np.nan
    return dict(displacement_xy_mm=relative, valid=accepted, origin_index=int(indices[0]),
                coordinate_kind='relative_marker_midpoint', camera_foot_required=False,
                absolute_position_available=False, height_corrected_heading_available=False,
                fixed_camera_and_height_within_video_required=True)
