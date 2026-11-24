"""機体そのものの外観（シャシー輪郭・内部バッテリー・前後キーポイント色分布）を学習し、
マーカー欠測時の補完・背景ノイズ誤ペアリング排除・局所ROI重心精密化を行うモジュール。
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

import cv2
import numpy as np
import pandas as pd

PATCH_SIZE = 64
SPAN_PX = 36.0
REAR_CANON_XY = np.array([PATCH_SIZE / 2.0 - SPAN_PX / 2.0, PATCH_SIZE / 2.0], dtype=np.float32)
FRONT_CANON_XY = np.array([PATCH_SIZE / 2.0 + SPAN_PX / 2.0, PATCH_SIZE / 2.0], dtype=np.float32)


def make_body_masks() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """正規化64x64パッチ上のマスクを生成する。
    - body_core_mask: 前後マーカー領域を除いた機体本体（黒シャシー・タイヤ・内部黄色バッテリー・モーターマウント）
    - front_roi_mask: 前マーカー中心付近（半径4.0 px）
    - rear_roi_mask: 後マーカー中心付近（半径4.0 px）
    """
    yy, xx = np.mgrid[0:PATCH_SIZE, 0:PATCH_SIZE].astype(np.float32)
    body_ellipse = (((xx - 32.0) / 24.0) ** 2 + ((yy - 32.0) / 14.0) ** 2) <= 1.0
    front_dist = np.hypot(xx - FRONT_CANON_XY[0], yy - FRONT_CANON_XY[1])
    rear_dist = np.hypot(xx - REAR_CANON_XY[0], yy - REAR_CANON_XY[1])
    front_roi = front_dist <= 4.0
    rear_roi = rear_dist <= 4.0
    body_core = body_ellipse & (front_dist > 5.0) & (rear_dist > 5.0)
    return body_core, front_roi, rear_roi


BODY_CORE_MASK, FRONT_ROI_MASK, REAR_ROI_MASK = make_body_masks()


def extract_canonical_patch(frame_bgr: np.ndarray, front_px: np.ndarray, rear_px: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """前後2点 (front_px, rear_px) を (50, 32), (14, 32) に揃えた 64x64 正規化機体パッチを切り出す。"""
    front_px = np.asarray(front_px, dtype=np.float32)
    rear_px = np.asarray(rear_px, dtype=np.float32)
    mid = 0.5 * (front_px + rear_px)
    vec = front_px - rear_px
    dist = float(np.linalg.norm(vec))
    if dist < 4.0:
        return np.zeros((PATCH_SIZE, PATCH_SIZE, 3), dtype=np.uint8), np.eye(2, 3, dtype=np.float32)
    angle_deg = math.degrees(math.atan2(float(vec[1]), float(vec[0])))
    scale = SPAN_PX / dist
    M = cv2.getRotationMatrix2D((float(mid[0]), float(mid[1])), angle_deg, scale)
    M[0, 2] += (PATCH_SIZE / 2.0) - float(mid[0])
    M[1, 2] += (PATCH_SIZE / 2.0) - float(mid[1])
    patch = cv2.warpAffine(frame_bgr, M, (PATCH_SIZE, PATCH_SIZE), flags=cv2.INTER_LINEAR)
    return patch, M.astype(np.float32)


def patch_to_features(patch_bgr: np.ndarray) -> np.ndarray:
    """64x64 BGRパッチを4チャネル特徴 [Lab_L, Lab_a-128, Lab_b-128, HSV_S] に変換する。"""
    lab = cv2.cvtColor(patch_bgr, cv2.COLOR_BGR2LAB).astype(np.float32)
    hsv = cv2.cvtColor(patch_bgr, cv2.COLOR_BGR2HSV).astype(np.float32)
    return np.dstack([lab[:, :, 0], lab[:, :, 1] - 128.0, lab[:, :, 2] - 128.0, hsv[:, :, 1]])


def angle_diff_rad(a: float, b: float) -> float:
    return float((a - b + math.pi) % (2.0 * math.pi) - math.pi)


def relaxed_detection_config(colour: str, marker_detection: dict | None) -> dict:
    """機体モデルによる誤検出フィルタを前提に、色抜け・ブラー救済用の緩和色検出設定を返す。"""
    base = dict((marker_detection or {}).get(colour, {}))
    sat = base.get("minimum_saturation", 50)
    val = base.get("minimum_value", 65)
    base["minimum_saturation"] = max(10, int(round(sat * 0.55)))
    base["minimum_value"] = max(45, int(round(val * 0.75)))
    base["minimum_area_px"] = 2
    base["opening_kernel_size"] = 0
    if colour == "yellow":
        if "hue_ranges" not in base:
            base["hue_ranges"] = [[18, 65]]
        else:
            hr = base["hue_ranges"]
            low = min(b[0] for b in hr)
            high = max(b[1] for b in hr)
            base["hue_ranges"] = [[max(15, low - 7), min(72, high + 15)]]
    elif colour == "green":
        hr = base.get("hue_ranges", [[40, 85]])
        low = min(b[0] for b in hr)
        high = max(b[1] for b in hr)
        base["hue_ranges"] = [[max(35, low - 10), min(105, high + 10)]]
    return {colour: base}


def refine_keypoint_native_image(
    frame_lab: np.ndarray,
    frame_hsv: np.ndarray,
    center_px: np.ndarray,
    mean_vec: np.ndarray,
    std_vec: np.ndarray,
    radius_px: float,
) -> np.ndarray:
    """元画像解像度の局所円形ROI内で、学習済み色分布の連続重み付き重心をサブピクセル計算する。"""
    h, w = frame_lab.shape[:2]
    r_int = max(3, int(math.ceil(radius_px)))
    cx, cy = float(center_px[0]), float(center_px[1])
    x0, x1 = max(0, int(round(cx)) - r_int), min(w, int(round(cx)) + r_int + 1)
    y0, y1 = max(0, int(round(cy)) - r_int), min(h, int(round(cy)) + r_int + 1)
    if x1 <= x0 or y1 <= y0:
        return np.asarray(center_px, dtype=np.float32).copy()

    yy, xx = np.mgrid[y0:y1, x0:x1].astype(np.float32)
    dist2 = (xx - cx) ** 2 + (yy - cy) ** 2
    roi_mask = dist2 <= (radius_px ** 2)
    if not roi_mask.any():
        return np.asarray(center_px, dtype=np.float32).copy()

    lab_sub = frame_lab[y0:y1, x0:x1]
    hsv_sub = frame_hsv[y0:y1, x0:x1]
    feat = np.dstack([
        lab_sub[:, :, 0],
        lab_sub[:, :, 1] - 128.0,
        lab_sub[:, :, 2] - 128.0,
        hsv_sub[:, :, 1],
    ])
    pix = feat[roi_mask]
    scale = np.array([2.5, 1.0, 1.0, 1.2], dtype=np.float32) * std_vec
    d2 = np.sum(((pix - mean_vec) / scale) ** 2, axis=1)
    rx = xx[roi_mask]
    ry = yy[roi_mask]
    spatial_d2 = dist2[roi_mask] / ((0.65 * radius_px) ** 2 + 1e-6)
    weights = np.exp(-0.5 * (d2 + 0.30 * spatial_d2))
    thresh = float(np.percentile(weights, 65))
    keep = weights >= max(thresh, 1e-4)
    if not keep.any() or float(weights[keep].sum()) < 1e-4:
        return np.asarray(center_px, dtype=np.float32).copy()
    w_keep = weights[keep] ** 2
    return np.array([
        float(np.sum(rx[keep] * w_keep) / np.sum(w_keep)),
        float(np.sum(ry[keep] * w_keep) / np.sum(w_keep)),
    ], dtype=np.float32)


class LearnedBodyKeypointModel:
    """機体本体のマルチチャネル外観テンプレートと前後キーポイント色分布モデル。"""

    def __init__(
        self,
        mean_feat: np.ndarray,
        front_lab_mean: np.ndarray,
        front_lab_std: np.ndarray,
        rear_lab_mean: np.ndarray,
        rear_lab_std: np.ndarray,
        span_px_median: float,
        metadata: dict | None = None,
    ):
        self.mean_feat = np.asarray(mean_feat, dtype=np.float32)
        self.front_lab_mean = np.asarray(front_lab_mean, dtype=np.float32)
        self.front_lab_std = np.maximum(np.asarray(front_lab_std, dtype=np.float32), 6.0)
        self.rear_lab_mean = np.asarray(rear_lab_mean, dtype=np.float32)
        self.rear_lab_std = np.maximum(np.asarray(rear_lab_std, dtype=np.float32), 6.0)
        self.span_px_median = float(span_px_median)
        self.metadata = dict(metadata or {})
        arrays = (self.mean_feat,self.front_lab_mean,self.front_lab_std,self.rear_lab_mean,self.rear_lab_std)
        if (self.mean_feat.shape!=(64,64,4) or any(value.shape!=(4,) for value in arrays[1:])
                or not all(np.isfinite(value).all() for value in arrays)
                or not math.isfinite(self.span_px_median) or self.span_px_median<4):
            raise ValueError('機体外観モデルの形状または数値が不正です')

        self.core_vecs = []
        for c in range(4):
            v = self.mean_feat[:, :, c][BODY_CORE_MASK]
            v = v - v.mean()
            norm = float(np.linalg.norm(v)) + 1e-6
            self.core_vecs.append(v / norm)

    @classmethod
    def fit_from_frames(
        cls,
        frames_and_pairs: list[tuple[np.ndarray, np.ndarray, np.ndarray]],
        metadata: dict | None = None,
    ) -> "LearnedBodyKeypointModel":
        if not frames_and_pairs:
            raise ValueError("at least one (frame, front_px, rear_px) sample is required to train body model")
        feats, front_vals, rear_vals, spans = [], [], [], []
        for frame, f_px, r_px in frames_and_pairs:
            f_px = np.asarray(f_px, dtype=np.float32)
            r_px = np.asarray(r_px, dtype=np.float32)
            patch, _ = extract_canonical_patch(frame, f_px, r_px)
            feat = patch_to_features(patch)
            feats.append(feat)
            front_vals.append(feat[FRONT_ROI_MASK].mean(axis=0))
            rear_vals.append(feat[REAR_ROI_MASK].mean(axis=0))
            spans.append(float(np.linalg.norm(f_px - r_px)))
        mean_feat = np.median(np.stack(feats, axis=0), axis=0).astype(np.float32)
        f_arr = np.stack(front_vals, axis=0)
        r_arr = np.stack(rear_vals, axis=0)
        meta = dict(metadata or {}, sample_count=len(frames_and_pairs))
        return cls(
            mean_feat=mean_feat,
            front_lab_mean=np.median(f_arr, axis=0),
            front_lab_std=np.std(f_arr, axis=0) * 1.5 + 8.0,
            rear_lab_mean=np.median(r_arr, axis=0),
            rear_lab_std=np.std(r_arr, axis=0) * 1.5 + 8.0,
            span_px_median=float(np.median(spans)),
            metadata=meta,
        )

    def save(self, path: Path) -> None:
        path = Path(path)
        if path.exists():
            raise FileExistsError('学習済みモデルを上書きしません: '+str(path))
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            path,
            mean_feat=self.mean_feat,
            front_lab_mean=self.front_lab_mean,
            front_lab_std=self.front_lab_std,
            rear_lab_mean=self.rear_lab_mean,
            rear_lab_std=self.rear_lab_std,
            span_px_median=np.float32(self.span_px_median),
            metadata_json=json.dumps(self.metadata, ensure_ascii=False),
        )

    @classmethod
    def load(cls, path: Path) -> "LearnedBodyKeypointModel":
        with np.load(Path(path), allow_pickle=False) as data:
            meta = json.loads(str(data["metadata_json"])) if "metadata_json" in data else {}
            return cls(mean_feat=data['mean_feat'],front_lab_mean=data['front_lab_mean'],
                front_lab_std=data['front_lab_std'],rear_lab_mean=data['rear_lab_mean'],
                rear_lab_std=data['rear_lab_std'],span_px_median=float(data['span_px_median']),metadata=meta)

    def validate_identity(self,mouse,front_colour,rear_colour):
        for key,expected in (('mouse',mouse),('front_colour',front_colour),('rear_colour',rear_colour)):
            recorded = self.metadata.get(key)
            if expected is not None and recorded is not None and expected!=recorded:
                raise ValueError(f'学習モデルの{key}が今回の機体・色と異なります')

    def score_patch(self, patch_bgr: np.ndarray) -> tuple[float, float, float]:
        """正規化64x64パッチの (総合スコア, 機体本体NCC, 前後マーカー色尤度) を返す。"""
        feat = patch_to_features(patch_bgr)
        weights = (0.35, 0.15, 0.25, 0.25)
        body_ncc = 0.0
        for c, w in enumerate(weights):
            v = feat[:, :, c][BODY_CORE_MASK]
            v_zm = v - v.mean()
            norm = float(np.linalg.norm(v_zm))
            if norm < 1e-3:
                continue
            body_ncc += w * float(np.dot(v_zm / norm, self.core_vecs[c]))

        l_std = float(np.std(feat[:, :, 0][BODY_CORE_MASK]))
        contrast_factor = min(1.0, l_std / 20.0)
        body_ncc *= contrast_factor

        f_pix = feat[FRONT_ROI_MASK][:, 1:]
        r_pix = feat[REAR_ROI_MASK][:, 1:]
        f_dist2 = np.sum(((f_pix - self.front_lab_mean[1:]) / self.front_lab_std[1:]) ** 2, axis=1)
        r_dist2 = np.sum(((r_pix - self.rear_lab_mean[1:]) / self.rear_lab_std[1:]) ** 2, axis=1)
        f_prob = float(np.exp(-0.5 * np.percentile(f_dist2, 25)))
        r_prob = float(np.exp(-0.5 * np.percentile(r_dist2, 25)))
        marker_score = 0.5 * (f_prob + r_prob)

        total = 0.60 * body_ncc + 0.40 * marker_score
        return total, body_ncc, marker_score

    def score_patches(self, patches) -> np.ndarray:
        """score_patchと同じ式を候補単位でまとめる。候補数や採否閾値は削減しない。"""
        scores = []
        for start in range(0,len(patches),64):
            chunk = patches[start:start+64]
            features = patch_to_features(np.concatenate(chunk,axis=0)).reshape(len(chunk),64,64,4)
            body = np.zeros(len(chunk),np.float32)
            for channel,weight in enumerate((.35,.15,.25,.25)):
                vectors = features[:,:,:,channel][:,BODY_CORE_MASK]
                vectors = vectors-vectors.mean(axis=1,keepdims=True)
                norms = np.linalg.norm(vectors,axis=1)
                normalized = np.divide(vectors,norms[:,None],out=np.zeros_like(vectors),where=norms[:,None]>=1e-3)
                # 小さい候補行列の積でBLASの多数スレッドを起こさず、同じ全候補を評価する。
                body += weight*np.einsum('ij,j->i',normalized,self.core_vecs[channel],optimize=False)
            contrast = np.std(features[:,:,:,0][:,BODY_CORE_MASK],axis=1)
            body *= np.minimum(1.,contrast/20.)
            front = features[:,FRONT_ROI_MASK,1:]
            rear = features[:,REAR_ROI_MASK,1:]
            f_dist = np.sum(((front-self.front_lab_mean[1:])/self.front_lab_std[1:])**2,axis=-1)
            r_dist = np.sum(((rear-self.rear_lab_mean[1:])/self.rear_lab_std[1:])**2,axis=-1)
            marker = .5*(np.exp(-.5*np.percentile(f_dist,25,axis=1))+np.exp(-.5*np.percentile(r_dist,25,axis=1)))
            scores.append(np.column_stack((.60*body+.40*marker,body,marker)))
        return np.vstack(scores) if scores else np.empty((0,3))


def detect_pair_with_body_model(
    frame_bgr: np.ndarray,
    front_colour: str,
    rear_colour: str,
    marker_detection: dict | None,
    H_img_to_pose: np.ndarray,
    H_pose_to_img: np.ndarray,
    model: LearnedBodyKeypointModel,
    previous_center_mm: np.ndarray | None = None,
    previous_heading_rad: float | None = None,
    previous_velocity_mm: np.ndarray | None = None,
    body_geometry: dict | None = None,
    board_to_pose: np.ndarray | None = None,
    board_side_mm: float = 360.0,
    nominal_span_mm: float | None = 50.0,
    span_limits_mm: tuple[float, float] | None = None,
    min_body_ncc: float = 0.22,
    require_colour_pair: bool = False,
    refine_standard: bool = False,
    keypoint_model=None,
    center_margin_mm: float = 25.0,
) -> tuple[np.ndarray, np.ndarray, dict] | None:
    """機体外観モデル＋前後キーポイント局所精密化により (front_mm, rear_mm, info) を検出する。"""
    from video_marker_pose import colour_components, transform_points
    from pixel_camera_geometry import correct_body_markers
    if not math.isfinite(center_margin_mm) or not 0 <= center_margin_mm <= 90:
        raise ValueError('center margin must be within 0..90 mm')

    def inside_board(center):
        return (-center_margin_mm <= center[0] <= board_side_mm+center_margin_mm
            and -center_margin_mm <= center[1] <= board_side_mm+center_margin_mm)

    nom_span = nominal_span_mm
    if nom_span is None and body_geometry is not None:
        nom_span = body_geometry.get("nominal_marker_span_mm")
    if nom_span is None:
        nom_span = 50.0
    low, high = (0.65 * nom_span, 1.4 * nom_span) if span_limits_mm is None else tuple(span_limits_mm)

    f_std = colour_components(frame_bgr, front_colour, marker_detection)
    r_std = colour_components(frame_bgr, rear_colour, marker_detection)
    f_rel = colour_components(frame_bgr, front_colour, relaxed_detection_config(front_colour, marker_detection))
    r_rel = colour_components(frame_bgr, rear_colour, relaxed_detection_config(rear_colour, marker_detection))

    def merge_cands(c_std, c_rel):
        merged = [(pt, area, True) for pt, area in c_std]
        for pt, area in c_rel:
            if not any(np.linalg.norm(pt - p0) < 3.5 for p0, _, _ in merged):
                merged.append((pt, area, False))
        return merged

    f_cands = merge_cands(f_std, f_rel)
    r_cands = merge_cands(r_std, r_rel)

    pred_center_mm = None
    if previous_center_mm is not None:
        pred_center_mm = np.asarray(previous_center_mm, dtype=float)
        if previous_velocity_mm is not None:
            pred_center_mm = pred_center_mm + np.asarray(previous_velocity_mm, dtype=float)

    f_mm_all = transform_points([pt for pt, _, _ in f_cands], H_img_to_pose) if f_cands else np.empty((0, 2))
    r_mm_all = transform_points([pt for pt, _, _ in r_cands], H_img_to_pose) if r_cands else np.empty((0, 2))

    def plausible_indices(pts_mm):
        if not len(pts_mm):
            return []
        ok = (
            (pts_mm[:, 0] >= -45.0)
            & (pts_mm[:, 0] <= board_side_mm + 45.0)
            & (pts_mm[:, 1] >= -45.0)
            & (pts_mm[:, 1] <= board_side_mm + 45.0)
        )
        if pred_center_mm is not None:
            dist = np.linalg.norm(pts_mm - pred_center_mm, axis=1)
            if np.any(ok & (dist <= 75.0)):
                ok = ok & (dist <= 75.0)
        return np.flatnonzero(ok).tolist()

    f_idx_list = plausible_indices(f_mm_all)
    r_idx_list = plausible_indices(r_mm_all)

    best_candidate = None
    best_total_score = -1e9
    pair_candidates = []

    for i in f_idx_list:
        f_px, f_area, f_is_std = f_cands[i]
        f_mm0 = f_mm_all[i]
        for j in r_idx_list:
            r_px, r_area, r_is_std = r_cands[j]
            r_mm0 = r_mm_all[j]
            raw_sep = float(np.linalg.norm(f_mm0 - r_mm0))
            if not (low - 6.0 <= raw_sep <= high + 6.0):
                continue
            f_mm, r_mm = f_mm0, r_mm0
            if body_geometry is not None and board_to_pose is not None:
                board_pts = transform_points([f_mm0, r_mm0], np.linalg.inv(board_to_pose))
                corr = correct_body_markers(board_pts[0], board_pts[1], body_geometry)
                f_mm, r_mm = transform_points([corr["front_xy_mm"], corr["rear_xy_mm"]], board_to_pose)
            sep_mm = float(np.linalg.norm(f_mm - r_mm))
            if not (low - 3.0 <= sep_mm <= high + 3.0):
                continue
            centre_mm = 0.5 * (f_mm + r_mm)
            if not inside_board(centre_mm):
                continue

            balance = min(f_area, r_area) / max(f_area, r_area)
            std_bonus = (0.35 if f_is_std else 0.0) + (0.35 if r_is_std else 0.0)
            geom_score = 0.30 * math.log1p(f_area + r_area) + 0.45 * balance - 0.018 * (sep_mm - nom_span) ** 2 + std_bonus
            motion_penalty = 0.0
            heading_rad = math.atan2(f_mm[1] - r_mm[1], f_mm[0] - r_mm[0])
            if pred_center_mm is not None:
                step_mm = float(np.linalg.norm(centre_mm - pred_center_mm))
                motion_penalty += 0.12 * max(0.0, step_mm - 18.0) + 0.02 * step_mm
            if previous_heading_rad is not None:
                dtheta_deg = abs(math.degrees(angle_diff_rad(heading_rad, previous_heading_rad)))
                motion_penalty += 0.15 * max(0.0, dtheta_deg - 18.0)

            pair_candidates.append((f_px,r_px,f_is_std,r_is_std,geom_score-motion_penalty))

    for start in range(0,len(pair_candidates),64):
        chunk = pair_candidates[start:start+64]
        patches = [extract_canonical_patch(frame_bgr,row[0],row[1])[0] for row in chunk]
        scores = model.score_patches(patches)
        for row,(_,body_ncc,marker_prob) in zip(chunk,scores):
            if body_ncc<min_body_ncc:
                continue
            combined = 6.*body_ncc+3.*marker_prob+row[4]
            if combined>best_total_score:
                best_total_score = combined
                best_candidate = (*row[:4],float(body_ncc),float(marker_prob),'pair_body_scored')

    # 片側マーカーのみ検出時または機体相関が低い場合の単一マーカー起点＋機体向き探索
    pivot_ncc_thresh = max(0.32, min_body_ncc + 0.10)
    tracking_info = {}
    if (not require_colour_pair and keypoint_model is not None and pred_center_mm is not None
            and previous_heading_rad is not None
            and (best_candidate is None or best_candidate[4] < pivot_ncc_thresh)):
        # 直前姿勢は切り出し位置にだけ使用し、今の画像からCNNで前後点を求める。
        direction = np.array([math.cos(previous_heading_rad), math.sin(previous_heading_rad)])
        guess = transform_points([pred_center_mm+.5*nom_span*direction,
            pred_center_mm-.5*nom_span*direction], H_pose_to_img)
        front_nn, rear_nn, cnn_info = keypoint_model.refine(frame_bgr, *guess)
        native = np.asarray([front_nn, rear_nn], dtype=np.float32)
        if np.isfinite(native).all():
            points_nn = transform_points(native, H_img_to_pose)
            centre_nn = points_nn.mean(axis=0)
            span_nn = float(np.linalg.norm(points_nn[0]-points_nn[1]))
            angle_nn = math.atan2(*(points_nn[0]-points_nn[1])[::-1])
            patch_nn, _ = extract_canonical_patch(frame_bgr, *native)
            _, ncc_nn, marker_nn = model.score_patch(patch_nn)
            spread = np.asarray(cnn_info.get('cnn_spread_canonical_px', [np.inf, np.inf]))
            if (low <= span_nn <= high and inside_board(centre_nn)
                    and np.linalg.norm(centre_nn-pred_center_mm) <= 20.
                    and abs(math.degrees(angle_diff_rad(angle_nn,previous_heading_rad))) <= 60.
                    and np.isfinite(spread).all() and (spread <= 4.5).all()
                    and ncc_nn >= pivot_ncc_thresh):
                best_candidate = (*native,False,False,float(ncc_nn),float(marker_nn),'cnn_body_tracking')
                tracking_info = dict(cnn_info,keypoint_cnn_applied=True)
    if not require_colour_pair and ((best_candidate is None) or (best_candidate[4] < pivot_ncc_thresh)):
        if previous_heading_rad is not None:
            angles = [previous_heading_rad + math.radians(d) for d in range(-36, 37, 3)]
        else:
            angles = [math.radians(d) for d in range(0, 360, 8)]

        pivot_candidates = []
        def add_pivot(front_px,rear_px,front_std,rear_std,centre_mm,angle,method):
            if not inside_board(centre_mm):
                return
            penalty = 0.
            if pred_center_mm is not None:
                step = float(np.linalg.norm(centre_mm-pred_center_mm))
                penalty += .12*max(0.,step-18.)+.02*step
            if previous_heading_rad is not None:
                turn = abs(math.degrees(angle_diff_rad(angle,previous_heading_rad)))
                penalty += .15*max(0.,turn-18.)
            pivot_candidates.append((front_px,rear_px,front_std,rear_std,penalty,method))

        for i in sorted(f_idx_list, key=lambda idx: f_cands[idx][1], reverse=True)[:4]:
            f_px, _, f_is_std = f_cands[i]
            f_mm0 = f_mm_all[i]
            for ang in angles:
                r_mm_guess = f_mm0 - nom_span * np.array([math.cos(ang), math.sin(ang)], dtype=np.float32)
                r_px_guess = transform_points([r_mm_guess], H_pose_to_img)[0]
                centre_mm = 0.5 * (f_mm0 + r_mm_guess)
                add_pivot(f_px,r_px_guess,f_is_std,False,centre_mm,ang,'front_pivot_body_search')

        for j in sorted(r_idx_list, key=lambda idx: r_cands[idx][1], reverse=True)[:4]:
            r_px, _, r_is_std = r_cands[j]
            r_mm0 = r_mm_all[j]
            for ang in angles:
                f_mm_guess = r_mm0 + nom_span * np.array([math.cos(ang), math.sin(ang)], dtype=np.float32)
                f_px_guess = transform_points([f_mm_guess], H_pose_to_img)[0]
                centre_mm = 0.5 * (f_mm_guess + r_mm0)
                add_pivot(f_px_guess,r_px,False,r_is_std,centre_mm,ang,'rear_pivot_body_search')

        for start in range(0,len(pivot_candidates),64):
            chunk = pivot_candidates[start:start+64]
            patches = [extract_canonical_patch(frame_bgr,row[0],row[1])[0] for row in chunk]
            for row,(_,body_ncc,marker_prob) in zip(chunk,model.score_patches(patches)):
                if body_ncc < pivot_ncc_thresh:
                    continue
                combined = 6.*body_ncc+3.*marker_prob+1.-row[4]
                if combined > best_total_score:
                    best_total_score = combined
                    best_candidate = (*row[:4],float(body_ncc),float(marker_prob),row[5])

    if best_candidate is None:
        return None

    f_px_init, r_px_init, f_is_std, r_is_std, body_ncc, marker_prob, method = best_candidate
    span_px = float(np.linalg.norm(f_px_init - r_px_init))
    roi_rad = max(4.0, 0.14 * span_px)
    if method=='cnn_body_tracking' or (f_is_std and r_is_std and method == "pair_body_scored" and not refine_standard):
        f_px_final, r_px_final = np.asarray(f_px_init, dtype=np.float32), np.asarray(r_px_init, dtype=np.float32)
    else:
        frame_lab = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2LAB).astype(np.float32)
        frame_hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV).astype(np.float32)
        f_px_final = (
            np.asarray(f_px_init, dtype=np.float32)
            if f_is_std and not refine_standard
            else refine_keypoint_native_image(frame_lab, frame_hsv, f_px_init, model.front_lab_mean, model.front_lab_std, roi_rad)
        )
        r_px_final = (
            np.asarray(r_px_init, dtype=np.float32)
            if r_is_std and not refine_standard
            else refine_keypoint_native_image(frame_lab, frame_hsv, r_px_init, model.rear_lab_mean, model.rear_lab_std, roi_rad)
        )

    neural_info = dict(keypoint_cnn_applied=False)
    neural_info.update(tracking_info)
    if keypoint_model is not None and method=='pair_body_scored':
        neural_front,neural_rear,neural_info_extra = keypoint_model.refine(frame_bgr,f_px_init,r_px_init)
        neural_info.update(neural_info_extra)
        visibility = np.asarray(neural_info_extra['cnn_visibility'])
        displacement = np.linalg.norm(np.asarray([neural_front,neural_rear])-np.asarray([f_px_init,r_px_init]),axis=1)
        neural_patch,_ = extract_canonical_patch(frame_bgr,neural_front,neural_rear)
        _,neural_ncc,_ = model.score_patch(neural_patch)
        if (np.isfinite(displacement).all() and (visibility>=.5).all()
                and (displacement<=max(4.,.12*span_px)).all() and neural_ncc>=max(min_body_ncc,body_ncc-.03)):
            f_px_final,r_px_final = neural_front,neural_rear
            neural_info['keypoint_cnn_applied'] = True
    pts_mm = transform_points([f_px_final, r_px_final], H_img_to_pose)
    if body_geometry is not None and board_to_pose is not None:
        board_pts = transform_points(pts_mm, np.linalg.inv(board_to_pose))
        corr = correct_body_markers(board_pts[0], board_pts[1], body_geometry)
        pts_mm = transform_points([corr["front_xy_mm"], corr["rear_xy_mm"]], board_to_pose)

    final_sep = float(np.linalg.norm(pts_mm[0]-pts_mm[1]))
    if not np.isfinite(pts_mm).all() or not low<=final_sep<=high or not inside_board(pts_mm.mean(axis=0)):
        return None
    final_patch,_ = extract_canonical_patch(frame_bgr,f_px_final,r_px_final)
    _,final_ncc,_ = model.score_patch(final_patch)
    if final_ncc<min_body_ncc:
        return None

    return pts_mm[0], pts_mm[1], dict(
        method=method,
        body_ncc=float(body_ncc),
        marker_prob=float(marker_prob),
        front_px=f_px_final,
        rear_px=r_px_final,
        front_colour_observed=method not in ('rear_pivot_body_search','cnn_body_tracking'),
        rear_colour_observed=method not in ('front_pivot_body_search','cnn_body_tracking'),
        refined_body_ncc=float(final_ncc),
        **neural_info,
    )


def harvest_training_samples_from_trial(trial_json_path: Path, max_frames: int = 30) -> tuple[list[tuple[np.ndarray, np.ndarray, np.ndarray]], dict]:
    """保存済み trial.json と full/apparent_floor_marker_pose.csv から高信頼フレームを自動抽出する。"""
    sys.path.insert(0, str(Path(__file__).resolve().parent / "analysis"))
    from video_marker_pose import marker_registration, registration_quad, pose_world_corners, transform_points

    trial_dir = Path(trial_json_path).resolve().parent
    trial = json.loads(Path(trial_json_path).read_text(encoding="utf-8"))
    pose_csv = trial_dir / "full/apparent_floor_marker_pose.csv"
    geom_path = trial_dir / "full/body_geometry_used.json"
    ref_path = trial_dir / "full/marker_reference_used.json"
    if not pose_csv.exists():
        pose_csv = trial_dir / "quick/apparent_floor_marker_pose.csv"
        geom_path = trial_dir / "quick/body_geometry_used.json"
        ref_path = trial_dir / "quick/marker_reference_used.json"

    geom = json.loads(geom_path.read_text(encoding="utf-8"))
    ref = json.loads(ref_path.read_text(encoding="utf-8"))
    df = pd.read_csv(pose_csv)
    nom_span = geom.get("nominal_marker_span_mm", 50.0) or 50.0
    side = ref.get("board_side_mm", 360.0)
    world_corners = pose_world_corners(side)

    valid = df[df.marker_detected_raw & df.calibration_valid].copy()
    step = np.hypot(valid.center_x_mm.diff().fillna(0.0), valid.center_y_mm.diff().fillna(0.0))
    span_err = (valid.marker_separation_mm - nom_span).abs()
    good = valid[(step < 10.0) & (span_err < 5.0)]
    if len(good) < 3:
        good = valid
    target_frames = set(good.iloc[np.linspace(0, len(good) - 1, min(max_frames, len(good)), dtype=int)]["frame"].astype(int).tolist())
    rows_by_frame = {int(r.frame): r for _, r in good.iterrows() if int(r.frame) in target_frames}

    cap = cv2.VideoCapture(str(trial["video_file"]))
    if not cap.isOpened():
        raise RuntimeError(f"cannot open video: {trial['video_file']}")
    cap.set(cv2.CAP_PROP_ORIENTATION_AUTO, 0)
    samples = []
    max_target = max(target_frames) if target_frames else -1
    idx = 0
    while idx <= max_target:
        ok = cap.grab()
        if not ok:
            break
        if idx in rows_by_frame:
            ok, frame = cap.retrieve()
            if ok:
                row = rows_by_frame[idx]
                reg = marker_registration(frame, ref)
                quad = registration_quad(reg)
                if quad is not None:
                    H = cv2.getPerspectiveTransform(quad, world_corners)
                    H_inv = np.linalg.inv(H)
                    pts_px = transform_points([[row.blue_x_mm, row.blue_y_mm], [row.yellow_x_mm, row.yellow_y_mm]], H_inv)
                    samples.append((frame, pts_px[0], pts_px[1]))
        idx += 1
    cap.release()
    meta = dict(
        mouse=trial.get("mouse"),
        front_colour=geom.get("front_colour"),
        rear_colour=geom.get("rear_colour", "yellow"),
        source_trial=str(Path(trial_json_path).resolve()),
        source_video=str(trial["video_file"]),
    )
    return samples, meta


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    train_p = sub.add_parser("train", help="保存済み trial.json から機体外観＋キーポイントモデルを学習・保存する")
    train_p.add_argument("--trial", type=Path, nargs="+", required=True, help="1つ以上の trial.json パス")
    train_p.add_argument("--max-frames-per-trial", type=int, default=30)
    train_p.add_argument("--output", type=Path, required=True, help="出力 .npz ファイルパス")

    args = parser.parse_args()
    if args.command == "train":
        if args.output.exists():
            parser.error('既存モデルは上書きしません。新しい--outputを指定してください')
        all_samples = []
        metas = []
        for t_path in args.trial:
            samples, meta = harvest_training_samples_from_trial(t_path, max_frames=args.max_frames_per_trial)
            all_samples.extend(samples)
            metas.append(meta)
        identities = {(meta.get('mouse'),meta.get('front_colour'),meta.get('rear_colour')) for meta in metas}
        if len(identities)!=1 or None in next(iter(identities)):
            parser.error('機体A/Bと前後色が同じ試行だけを一緒に学習してください')
        combined_meta = dict(
            mouse=metas[0].get("mouse") if metas else None,
            front_colour=metas[0].get("front_colour") if metas else None,
            rear_colour=metas[0].get("rear_colour") if metas else None,
            sources=metas,
        )
        model = LearnedBodyKeypointModel.fit_from_frames(all_samples, metadata=combined_meta)
        model.save(args.output)
        print(json.dumps(dict(
            status="TRAINED",
            output=str(args.output.resolve()),
            sample_count=len(all_samples),
            span_px_median=model.span_px_median,
            front_lab_mean=model.front_lab_mean.tolist(),
            rear_lab_mean=model.rear_lab_mean.tolist(),
            metadata=combined_meta,
        ), ensure_ascii=False, indent=2))
        return 0
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
