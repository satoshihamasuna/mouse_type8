"""AR正対補正した盤面全体から機体の前後キーポイントを直接推定するグローバルCNN実験モジュール。

従来の色ペア検出や前フレーム追跡に依存せず、暗い旋回・モーションブラー時の再捕捉能力を
Colab GPU 等で検証するための実験用検出器。実観測（colour_pair）とは厳密に区別し、
出力は常に estimated として扱う。単独で盤面較正は行わない。
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import sys

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from body_appearance_model import FRONT_CANON_XY, PATCH_SIZE, REAR_CANON_XY, SPAN_PX, patch_to_features
from body_keypoint_cnn import resolve_device, torch_runtime


def board_features(rectified_bgr: np.ndarray) -> np.ndarray:
    """正対盤面BGR画像を4ch正規化特徴 [Lab_L/255, (a-128)/64, (b-128)/64, HSV_S/255] へ変換する。"""
    feat = patch_to_features(rectified_bgr) / np.array([255.0, 64.0, 64.0, 255.0], np.float32)
    return np.moveaxis(feat, -1, 0).astype(np.float32)


def build_board_network(base_channels: int = 24):
    """AR正対盤面画像 (B, 4, S, S) から (heatmap: B, 2, S/4, S/4, presence: B, 2) を出力する軽量U-Net。"""
    torch = torch_runtime()
    nn = torch.nn
    c = int(base_channels)

    def conv_block(in_ch: int, out_ch: int, stride: int = 1):
        return nn.Sequential(
            nn.Conv2d(in_ch, out_ch, 3, stride=stride, padding=1, bias=False),
            nn.GroupNorm(min(8, out_ch), out_ch),
            nn.SiLU(inplace=True),
            nn.Conv2d(out_ch, out_ch, 3, stride=1, padding=1, bias=False),
            nn.GroupNorm(min(8, out_ch), out_ch),
            nn.SiLU(inplace=True),
        )

    class BoardKeypointHeatmapNet(nn.Module):
        def __init__(self):
            super().__init__()
            self.stem = conv_block(4, c, stride=2)         # S/2
            self.down1 = conv_block(c, c * 2, stride=2)    # S/4
            self.down2 = conv_block(c * 2, c * 3, stride=2)  # S/8
            self.down3 = conv_block(c * 3, c * 4, stride=2)  # S/16
            self.up2 = conv_block(c * 4 + c * 3, c * 3)    # S/8
            self.up1 = conv_block(c * 3 + c * 2, c * 2)    # S/4
            self.heatmap_head = nn.Sequential(
                nn.Conv2d(c * 2, c, 3, padding=1),
                nn.SiLU(inplace=True),
                nn.Conv2d(c, 2, 1),
            )
            self.presence_head = nn.Sequential(
                nn.AdaptiveAvgPool2d(1),
                nn.Flatten(),
                nn.Linear(c * 4, c),
                nn.SiLU(inplace=True),
                nn.Linear(c, 2),
            )

        def forward(self, x):
            s2 = self.stem(x)
            s4 = self.down1(s2)
            s8 = self.down2(s4)
            s16 = self.down3(s8)
            u8 = torch.nn.functional.interpolate(s16, size=s8.shape[-2:], mode='bilinear', align_corners=False)
            u8 = self.up2(torch.cat((u8, s8), dim=1))
            u4 = torch.nn.functional.interpolate(u8, size=s4.shape[-2:], mode='bilinear', align_corners=False)
            u4 = self.up1(torch.cat((u4, s4), dim=1))
            return self.heatmap_head(u4), self.presence_head(s16)

    return BoardKeypointHeatmapNet()


def _make_soft_patch_alpha() -> np.ndarray:
    yy, xx = np.mgrid[0:PATCH_SIZE, 0:PATCH_SIZE].astype(np.float32)
    rx, ry = 27.0, 18.0
    dist = np.sqrt(((xx - 32.0) / rx) ** 2 + ((yy - 32.0) / ry) ** 2)
    alpha = np.clip((1.05 - dist) / 0.25, 0.0, 1.0)
    return alpha.astype(np.float32)


_PATCH_ALPHA = _make_soft_patch_alpha()


def _default_synthetic_board(board_px: int, rng: np.random.Generator) -> np.ndarray:
    """背景フレームがない場合のフォールバック用模擬盤面（暗い床＋薄い白格子）。"""
    base = int(rng.integers(25, 95))
    board = np.full((board_px, board_px, 3), base, dtype=np.uint8)
    grid_color = int(min(255, base + rng.integers(35, 110)))
    step = max(16, board_px // 8)
    for pos in range(step, board_px, step):
        cv2.line(board, (pos, 0), (pos, board_px - 1), (grid_color, grid_color, grid_color), 1)
        cv2.line(board, (0, pos), (board_px - 1, pos), (grid_color, grid_color, grid_color), 1)
    noise = rng.normal(0, float(rng.uniform(2.0, 8.0)), size=board.shape)
    return np.clip(board.astype(np.float32) + noise, 0, 255).astype(np.uint8)


def _apply_motion_blur_and_photometric(
    image: np.ndarray,
    rng: np.random.Generator,
    *,
    robust: bool,
    max_blur_px: int,
) -> np.ndarray:
    out = image.astype(np.float32)
    gain = float(rng.uniform(0.18 if robust else 0.65, 1.35))
    bias = float(rng.uniform(-14.0, 14.0))
    out = np.clip(out * gain + bias, 0, 255)
    if robust and rng.random() < 0.5:
        gamma = float(rng.uniform(0.65, 1.65))
        out = np.clip(((out / 255.0) ** gamma) * 255.0, 0, 255)
    out_u8 = out.astype(np.uint8)
    if rng.random() < (0.45 if robust else 0.2):
        k = int(rng.choice([3, 5]))
        out_u8 = cv2.GaussianBlur(out_u8, (k, k), float(rng.uniform(0.4, 1.4)))
    if rng.random() < (0.55 if robust else 0.2) and max_blur_px >= 3:
        size = int(rng.integers(3, max_blur_px + 1)) | 1
        kernel = np.zeros((size, size), np.float32)
        angle = float(rng.uniform(0, math.pi))
        vec = np.array([math.cos(angle), math.sin(angle)]) * (size - 1) / 2.0
        center = np.array([size // 2, size // 2])
        p0 = tuple(np.rint(center - vec).astype(int))
        p1 = tuple(np.rint(center + vec).astype(int))
        cv2.line(kernel, p0, p1, 1.0, 1)
        s = float(kernel.sum())
        if s > 0:
            kernel /= s
            out_u8 = cv2.filter2D(out_u8, -1, kernel)
    if robust and rng.random() < 0.5:
        sigma = float(rng.uniform(2.0, 12.0))
        out_u8 = np.clip(out_u8.astype(np.float32) + rng.normal(0, sigma, size=out_u8.shape), 0, 255).astype(np.uint8)
    return out_u8


def compose_synthetic_board_sample(
    patch_bgr: np.ndarray | None,
    background_bgr: np.ndarray,
    *,
    board_px: int,
    board_side_mm: float,
    nominal_span_mm: float,
    rng: np.random.Generator,
    robust: bool = True,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """実機体パッチを正対盤面背景の任意位置・角度へ合成し、(画像, 前後座標[board_px], presence)を返す。"""
    board = cv2.resize(background_bgr, (board_px, board_px), interpolation=cv2.INTER_AREA)
    if patch_bgr is None:
        max_blur = max(3, int(round(board_px * 10.0 / max(board_side_mm, 1.0))))
        board = _apply_motion_blur_and_photometric(board, rng, robust=robust, max_blur_px=max_blur)
        return board, np.zeros((2, 2), np.float32), np.zeros(2, np.float32)

    span_board_px = (nominal_span_mm / board_side_mm) * board_px * float(rng.uniform(0.92, 1.08))
    scale = span_board_px / SPAN_PX
    margin = max(14.0, 0.7 * span_board_px)
    cx = float(rng.uniform(margin, max(margin + 1.0, board_px - margin)))
    cy = float(rng.uniform(margin, max(margin + 1.0, board_px - margin)))
    angle_deg = float(rng.uniform(0.0, 360.0))

    # パッチ側で片側遮蔽・色抜けを模擬
    work_patch = patch_bgr.copy()
    if robust and rng.random() < 0.35:
        hide_idx = int(rng.integers(0, 2))
        pt = FRONT_CANON_XY if hide_idx == 0 else REAR_CANON_XY
        cv2.circle(work_patch, tuple(np.rint(pt).astype(int)), int(rng.integers(4, 7)), (45, 45, 45), -1)

    M = cv2.getRotationMatrix2D((PATCH_SIZE / 2.0, PATCH_SIZE / 2.0), -angle_deg, scale)
    M[0, 2] += cx - PATCH_SIZE / 2.0
    M[1, 2] += cy - PATCH_SIZE / 2.0
    warped_patch = cv2.warpAffine(work_patch, M, (board_px, board_px), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
    warped_alpha = cv2.warpAffine(_PATCH_ALPHA, M, (board_px, board_px), flags=cv2.INTER_LINEAR, borderValue=0.0)
    alpha3 = warped_alpha[:, :, None]
    composed = np.clip(warped_patch.astype(np.float32) * alpha3 + board.astype(np.float32) * (1.0 - alpha3), 0, 255).astype(np.uint8)

    # 盤面全体の明るさ低下・モーションブラー・高ISOノイズを適用
    max_blur = max(5, int(round(board_px * (10.0 if robust else 4.0) / max(board_side_mm, 1.0))))
    composed = _apply_motion_blur_and_photometric(composed, rng, robust=robust, max_blur_px=max_blur)

    canon_pts = np.asarray([FRONT_CANON_XY, REAR_CANON_XY], dtype=np.float32)
    board_pts = cv2.transform(canon_pts[None], M)[0].astype(np.float32)
    return composed, board_pts, np.ones(2, np.float32)


def extract_board_backgrounds_from_video(
    video_path: Path,
    reference: dict,
    *,
    board_px: int = 256,
    max_frames: int = 12,
    start_s: float = 0.0,
    end_s: float | None = None,
) -> list[np.ndarray]:
    """学習動画からAR正対盤面フレームを抽出し、ピクセル中央値で機体を取り除いた背景を作る。"""
    from video_marker_pose import iter_video_frames, marker_registration, pose_world_corners, registration_quad

    side = float(reference.get('board_side_mm', 360.0))
    dst = np.float32([[0, 0], [board_px - 1, 0], [0, board_px - 1], [board_px - 1, board_px - 1]])
    # pose_world_corners は [[0, side], [side, side], [0, 0], [side, 0]] なので、
    # board_px 上でも同じ対応順 (x=0..board_px-1, y_pose=side..0) に合わせてそのまま変換できる
    world = pose_world_corners(side)
    pose_to_rect = cv2.getPerspectiveTransform(world, dst)

    warped = []
    time_range = (start_s, end_s) if end_s is not None else None
    for _, frame, _ in iter_video_frames(Path(video_path), True, time_range, stride=8):
        reg = marker_registration(frame, reference)
        quad = registration_quad(reg)
        if quad is None:
            continue
        H = cv2.getPerspectiveTransform(quad, world)
        rect = cv2.warpPerspective(frame, pose_to_rect @ H, (board_px, board_px))
        warped.append(rect)
        if len(warped) >= max_frames:
            break
    if not warped:
        return []
    median_bg = np.median(np.stack(warped, axis=0), axis=0).astype(np.uint8)
    return [median_bg]


class BoardKeypointDetector:
    """AR正対盤面全体から前後キーポイントを直接推定するグローバルCNN検出器。"""

    def __init__(
        self,
        network,
        metadata: dict,
        *,
        device: str = 'cpu',
        min_presence: float = 0.45,
        max_spread_hm_px: float = 3.5,
    ):
        self.device = resolve_device(device)
        self.network = network.to(self.device).eval()
        self.metadata = dict(metadata)
        self.board_px = int(self.metadata.get('board_px', 256))
        self.base_channels = int(self.metadata.get('base_channels', 24))
        self.min_presence = float(min_presence)
        self.max_spread_hm_px = float(max_spread_hm_px)

    @classmethod
    def load(cls, path: Path, *, device: str = 'cpu', min_presence: float = 0.45, max_spread_hm_px: float = 3.5):
        torch = torch_runtime()
        saved = torch.load(Path(path), map_location='cpu', weights_only=True)
        if saved.get('schema_version') != 1 or saved.get('backend') != 'mouse_rectified_board_heatmap_cnn':
            raise ValueError('未対応の盤面グローバルCNNモデルです')
        meta = json.loads(saved['metadata_json'])
        network = build_board_network(base_channels=int(meta.get('base_channels', 24)))
        network.load_state_dict(saved['state_dict'], strict=True)
        return cls(network, meta, device=device, min_presence=min_presence, max_spread_hm_px=max_spread_hm_px)

    def save(self, path: Path) -> None:
        torch = torch_runtime()
        path = Path(path)
        if path.exists() or path.with_suffix('.json').exists():
            raise FileExistsError('学習済み盤面CNNを上書きしません: ' + str(path))
        path.parent.mkdir(parents=True, exist_ok=True)
        cpu_state = {k: v.detach().cpu() for k, v in self.network.state_dict().items()}
        torch.save(
            dict(
                schema_version=1,
                backend='mouse_rectified_board_heatmap_cnn',
                state_dict=cpu_state,
                metadata_json=json.dumps(self.metadata, ensure_ascii=False),
            ),
            path,
        )
        path.with_suffix('.json').write_text(json.dumps(self.metadata, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')

    def predict_rectified(self, rectified_bgr: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """正対盤面画像から (points_board_px[2,2], presence[2], spread_hm_px[2]) を推定する。"""
        torch = torch_runtime()
        if rectified_bgr.shape[:2] != (self.board_px, self.board_px):
            rectified_bgr = cv2.resize(rectified_bgr, (self.board_px, self.board_px), interpolation=cv2.INTER_AREA)
        feat = board_features(rectified_bgr)
        with torch.inference_mode():
            tensor = torch.from_numpy(feat[None]).to(self.device)
            heatmap, presence = self.network(tensor)
            hm_h, hm_w = heatmap.shape[-2:]
            logits = heatmap[0].detach().cpu().reshape(2, -1)
            presence_prob = torch.sigmoid(presence[0].detach().cpu()).numpy()
            peak = logits.argmax(dim=1)
            yy, xx = torch.meshgrid(torch.arange(hm_h), torch.arange(hm_w), indexing='ij')
            coords = torch.stack((xx.flatten(), yy.flatten()), dim=1).to(torch.float32)
            points = []
            spreads = []
            scale_x = self.board_px / float(hm_w)
            scale_y = self.board_px / float(hm_h)
            for kp in range(2):
                dist2 = ((coords - coords[peak[kp]]) ** 2).sum(dim=1)
                selected = dist2 <= 25.0
                weights = torch.softmax(logits[kp, selected], dim=0)
                pt_hm = (weights[:, None] * coords[selected]).sum(dim=0)
                spread = (weights * ((coords[selected] - pt_hm) ** 2).sum(dim=1)).sum().sqrt()
                points.append([(float(pt_hm[0]) + 0.5) * scale_x, (float(pt_hm[1]) + 0.5) * scale_y])
                spreads.append(float(spread))
            return np.asarray(points, np.float32), presence_prob.astype(np.float32), np.asarray(spreads, np.float32)

    def __call__(self, frame_bgr, H_img_to_pose, H_pose_to_img, state, case_context):
        """evaluate_body_detectors.evaluate の extra_detectors コールバック互換インターフェース。"""
        from video_marker_pose import pose_world_corners, transform_points

        side = float(case_context['board_side_mm'])
        geometry = case_context['geometry']
        nom_span = float(geometry.get('nominal_marker_span_mm') or 50.0)
        limits = geometry.get('marker_span_limits_mm')
        low, high = (0.65 * nom_span, 1.4 * nom_span) if limits is None else (float(limits[0]), float(limits[1]))

        world = pose_world_corners(side)
        dst = np.float32([[0, 0], [self.board_px - 1, 0], [0, self.board_px - 1], [self.board_px - 1, self.board_px - 1]])
        pose_to_rect = cv2.getPerspectiveTransform(world, dst)
        rect_to_pose = np.linalg.inv(pose_to_rect)

        rectified = cv2.warpPerspective(frame_bgr, pose_to_rect @ H_img_to_pose, (self.board_px, self.board_px))
        pts_rect, presence, spreads = self.predict_rectified(rectified)
        if not np.isfinite(pts_rect).all() or not np.isfinite(presence).all() or not np.isfinite(spreads).all():
            return None
        if float(presence.min()) < self.min_presence or float(spreads.max()) > self.max_spread_hm_px:
            return None

        pts_pose = transform_points(pts_rect, rect_to_pose)
        span_mm = float(np.linalg.norm(pts_pose[0] - pts_pose[1]))
        center_mm = pts_pose.mean(axis=0)
        if not (low <= span_mm <= high):
            return None
        if not (-5.0 <= center_mm[0] <= side + 5.0 and -5.0 <= center_mm[1] <= side + 5.0):
            return None

        info = dict(
            method='board_cnn_global',
            front_colour_observed=False,
            rear_colour_observed=False,
            keypoint_cnn_applied=True,
            board_cnn_presence=presence.tolist(),
            board_cnn_spread_hm_px=spreads.tolist(),
        )
        return pts_pose[0], pts_pose[1], info

    @classmethod
    def fit_from_patches(
        cls,
        patches_bgr: list[np.ndarray],
        metadata: dict,
        *,
        backgrounds_bgr: list[np.ndarray] | None = None,
        board_side_mm: float = 360.0,
        nominal_span_mm: float = 50.0,
        board_px: int = 256,
        base_channels: int = 24,
        dataset_size: int = 320,
        negative_ratio: float = 0.15,
        steps: int = 500,
        batch_size: int = 16,
        lr: float = 1e-3,
        seed: int = 29,
        robust: bool = True,
        device: str = 'auto',
        progress=None,
    ) -> 'BoardKeypointDetector':
        """抽出済み64x64正規化パッチ群と盤面背景から、盤面全体キーポイントCNNを学習する。"""
        if len(patches_bgr) < 3:
            raise ValueError('盤面CNNの学習には3枚以上の正規化パッチが必要です')
        if board_px < 64 or board_px % 16 != 0:
            raise ValueError('board_pxは64以上の16の倍数にしてください')
        torch = torch_runtime()
        selected_device = resolve_device(device)
        torch.manual_seed(seed)
        if selected_device.startswith('cuda'):
            torch.cuda.manual_seed_all(seed)
        rng = np.random.default_rng(seed)

        hm_size = board_px // 4
        xx, yy = np.meshgrid(np.arange(hm_size, dtype=np.float32), np.arange(hm_size, dtype=np.float32))
        sigma_hm = 1.6

        xs, targets, presences = [], [], []
        n_total = max(64, int(dataset_size))
        for idx in range(n_total):
            bg = (
                backgrounds_bgr[int(rng.integers(0, len(backgrounds_bgr)))]
                if backgrounds_bgr
                else _default_synthetic_board(board_px, rng)
            )
            is_negative = rng.random() < negative_ratio
            patch = None if is_negative else patches_bgr[idx % len(patches_bgr)]
            img, pts_board, pres = compose_synthetic_board_sample(
                patch,
                bg,
                board_px=board_px,
                board_side_mm=board_side_mm,
                nominal_span_mm=nominal_span_mm,
                rng=rng,
                robust=robust,
            )
            heat = []
            for kp in range(2):
                if pres[kp] > 0.5:
                    gx = pts_board[kp, 0] / 4.0 - 0.5
                    gy = pts_board[kp, 1] / 4.0 - 0.5
                    g = np.exp(-((xx - gx) ** 2 + (yy - gy) ** 2) / (2.0 * sigma_hm ** 2))
                    g /= max(float(g.sum()), 1e-8)
                else:
                    g = np.full((hm_size, hm_size), 1.0 / (hm_size * hm_size), dtype=np.float32)
                heat.append(g.astype(np.float32))
            xs.append(board_features(img))
            targets.append(heat)
            presences.append(pres)

        x_t = torch.from_numpy(np.asarray(xs, np.float32)).to(selected_device)
        t_t = torch.from_numpy(np.asarray(targets, np.float32)).flatten(2).to(selected_device)
        p_t = torch.from_numpy(np.asarray(presences, np.float32)).to(selected_device)

        network = build_board_network(base_channels=base_channels).to(selected_device).train()
        optimizer = torch.optim.AdamW(network.parameters(), lr=float(lr), weight_decay=1e-4)
        scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=max(1, steps))
        loss_value = None
        for step in range(steps):
            sel = torch.randint(0, n_total, (min(batch_size, n_total),), device=selected_device)
            logits, pres_logits = network(x_t[sel])
            kl = -(t_t[sel] * torch.nn.functional.log_softmax(logits.flatten(2), dim=-1)).sum(dim=-1)
            pos_weights = p_t[sel]
            hm_loss = (kl * pos_weights).sum() / pos_weights.sum().clamp_min(1.0)
            pres_loss = torch.nn.functional.binary_cross_entropy_with_logits(pres_logits, p_t[sel])
            loss = hm_loss + 0.5 * pres_loss
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            optimizer.step()
            scheduler.step()
            loss_value = float(loss.detach().cpu())
            if progress and (step == 0 or (step + 1) % 100 == 0 or step + 1 == steps):
                progress(step + 1, loss_value)

        meta = dict(
            metadata,
            backend='mouse_rectified_board_heatmap_cnn',
            board_px=int(board_px),
            base_channels=int(base_channels),
            board_side_mm=float(board_side_mm),
            nominal_span_mm=float(nominal_span_mm),
            patch_count=len(patches_bgr),
            dataset_size=n_total,
            steps=int(steps),
            batch_size=int(batch_size),
            lr=float(lr),
            seed=int(seed),
            robust_augmentation=bool(robust),
            training_loss=loss_value,
            training_device=selected_device,
            torch_version=str(torch.__version__),
            label_type='synthetic_board_from_colour_pair_patches',
            independent_accuracy_verified=False,
        )
        return cls(network, meta, device=selected_device)


def make_hybrid_board_detector(board_detector: BoardKeypointDetector):
    """通常の色ペア(legacy)を最優先し、色ペアが欠測したフレームのみ盤面グローバルCNNで補うハイブリッド検出器。"""
    from video_marker_pose import colour_components, select_marker_pair

    class HybridBoardDetector:
        def __init__(self, inner: BoardKeypointDetector):
            self.inner = inner
            self.metadata = dict(inner.metadata)

        def __call__(self, frame_bgr, H_img_to_pose, H_pose_to_img, state, case_context):
            geometry = case_context['geometry']
            side = case_context['board_side_mm']
            pair = select_marker_pair(
                colour_components(frame_bgr, case_context['front_colour'], geometry.get('marker_detection')),
                colour_components(frame_bgr, case_context['rear_colour'], geometry.get('marker_detection')),
                H_img_to_pose,
                state['center'],
                board_side_mm=side,
                nominal_marker_span_mm=geometry.get('nominal_marker_span_mm'),
                marker_span_limits_mm=geometry.get('marker_span_limits_mm'),
            )
            if pair is not None:
                return pair[0], pair[1], dict(
                    method='colour_pair',
                    front_colour_observed=True,
                    rear_colour_observed=True,
                    keypoint_cnn_applied=False,
                )
            return self.inner(frame_bgr, H_img_to_pose, H_pose_to_img, state, case_context)

    return HybridBoardDetector(board_detector)


def load_patches_directory(patch_dir: Path) -> tuple[list[np.ndarray], dict]:
    """train_turn_appearance.py が出力したパッチフォルダーを読み込む。"""
    patch_dir = Path(patch_dir)
    saved = json.loads((patch_dir / 'training.json').read_text(encoding='utf-8'))
    patches = []
    for label in saved['labels']:
        img = cv2.imread(str(patch_dir / label['patch']))
        if img is None or img.shape != (PATCH_SIZE, PATCH_SIZE, 3):
            raise ValueError('学習パッチ画像が不正です: ' + str(patch_dir / label['patch']))
        patches.append(img)
    return patches, saved['metadata']
