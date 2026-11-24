"""機体そのものの外観＋前後キーポイントモデルの学習・誤検出排除・欠測救済のホストテスト。"""
from __future__ import annotations

from pathlib import Path
import sys
import tempfile
import unittest

try:
    import cv2
    import numpy as np
    if not hasattr(cv2, "aruco") or not hasattr(cv2.aruco, "ArucoDetector"):
        raise ImportError("OpenCV with ArucoDetector is required")
except ImportError as error:
    raise unittest.SkipTest(str(error))

TOOLS = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(TOOLS), str(TOOLS / "analysis")]
from body_appearance_model import (
    LearnedBodyKeypointModel,
    detect_pair_with_body_model,
    extract_canonical_patch,
)
import video_marker_pose as pose


def draw_synthetic_mouse(
    canvas: np.ndarray,
    center_px: tuple[float, float],
    heading_deg: float,
    span_px: float = 50.0,
    front_bgr: tuple[int, int, int] = (40, 220, 40),
    rear_bgr: tuple[int, int, int] = (30, 30, 230),
    draw_front: bool = True,
    draw_rear: bool = True,
) -> tuple[np.ndarray, np.ndarray]:
    """実機と同様に、黒シャシー＋タイヤ＋内部黄色バッテリー＋前後マーカーを持つ模擬機体を描画する。"""
    cx, cy = float(center_px[0]), float(center_px[1])
    ang = np.radians(heading_deg)
    fwd = np.array([np.cos(ang), np.sin(ang)], dtype=np.float32)
    right = np.array([-np.sin(ang), np.cos(ang)], dtype=np.float32)
    center = np.array([cx, cy], dtype=np.float32)

    # 1. 黒い機体シャシー (楕円)
    cv2.ellipse(
        canvas,
        (int(round(cx)), int(round(cy))),
        (int(round(span_px * 0.62)), int(round(span_px * 0.36))),
        float(heading_deg),
        0,
        360,
        (25, 25, 28),
        -1,
        cv2.LINE_AA,
    )
    # 2. 後方の銀色モーターマウント領域 (x_body ~ -0.18 * span)
    motor_c = center - 0.16 * span_px * fwd
    cv2.circle(canvas, (int(round(motor_c[0])), int(round(motor_c[1]))), int(round(span_px * 0.16)), (165, 165, 170), -1, cv2.LINE_AA)

    # 3. 中央前寄りの鮮やかな黄色バッテリー (x_body ~ +0.14 * span)
    batt_c = center + 0.14 * span_px * fwd
    pts = np.array([
        batt_c - 0.11 * span_px * fwd - 0.18 * span_px * right,
        batt_c + 0.11 * span_px * fwd - 0.18 * span_px * right,
        batt_c + 0.11 * span_px * fwd + 0.18 * span_px * right,
        batt_c - 0.11 * span_px * fwd + 0.18 * span_px * right,
    ], dtype=np.int32)
    cv2.fillConvexPoly(canvas, pts, (15, 230, 250), cv2.LINE_AA)

    # 4. 前後マーカー
    front_px = center + 0.5 * span_px * fwd
    rear_px = center - 0.5 * span_px * fwd
    if draw_rear:
        cv2.circle(canvas, (int(round(rear_px[0])), int(round(rear_px[1]))), 5, rear_bgr, -1, cv2.LINE_AA)
    if draw_front:
        cv2.circle(canvas, (int(round(front_px[0])), int(round(front_px[1]))), 5, front_bgr, -1, cv2.LINE_AA)
    return front_px, rear_px


class BodyAppearanceModelTests(unittest.TestCase):
    def test_batched_scores_preserve_individual_candidate_scores(self):
        model = self.train_synthetic_model()
        rng = np.random.default_rng(31)
        patches = [rng.integers(0,256,(64,64,3),dtype=np.uint8) for _ in range(70)]
        patches.append(np.zeros((64,64,3),np.uint8))
        expected = np.array([model.score_patch(patch) for patch in patches])
        np.testing.assert_allclose(model.score_patches(patches),expected,atol=2e-6,rtol=2e-6)

    def test_identity_mismatch_and_invalid_arrays_are_rejected(self):
        model = self.train_synthetic_model()
        with self.assertRaises(ValueError):
            model.validate_identity('A','green','red')
        with self.assertRaises(ValueError):
            LearnedBodyKeypointModel(np.zeros((32,32,4)),*[np.zeros(4)]*4,50)

    def test_cnn_cannot_move_observed_keypoints_to_background(self):
        class BadRefiner:
            def refine(self,frame,front,rear):
                return front+100,rear+100,dict(cnn_visibility=[.99,.99])
        model = self.train_synthetic_model()
        frame = self.make_board()
        expected,_ = draw_synthetic_mouse(frame,(300,250),90,span_px=55)
        H = cv2.getPerspectiveTransform(pose.detect_aruco_quad(frame),pose.pose_world_corners(360))
        found = detect_pair_with_body_model(frame,'green','red',None,H,np.linalg.inv(H),model,
            nominal_span_mm=50,span_limits_mm=(40,65),require_colour_pair=True,keypoint_model=BadRefiner())
        self.assertIsNotNone(found)
        self.assertFalse(found[2]['keypoint_cnn_applied'])
        np.testing.assert_allclose(pose.transform_points([found[0]],np.linalg.inv(H))[0],expected,atol=2)

    def test_cnn_tracking_needs_current_body_evidence_and_is_estimated(self):
        model = self.train_synthetic_model()
        frame = self.make_board()
        front,rear = draw_synthetic_mouse(frame,(300,250),90,span_px=55,
            draw_front=False,draw_rear=False)
        class Refiner:
            def refine(self,image,front_guess,rear_guess):
                return front,rear,dict(cnn_visibility=[.1,.1],cnn_spread_canonical_px=[2.,2.])
        H = cv2.getPerspectiveTransform(pose.detect_aruco_quad(frame),pose.pose_world_corners(360))
        true_pose = pose.transform_points([front,rear],H)
        previous = true_pose.mean(axis=0)
        heading = np.arctan2(*(true_pose[0]-true_pose[1])[::-1])
        options = dict(previous_center_mm=previous,previous_heading_rad=heading,
            nominal_span_mm=50,span_limits_mm=(40,65),keypoint_model=Refiner())
        found = detect_pair_with_body_model(frame,'green','red',None,H,np.linalg.inv(H),model,**options)
        self.assertIsNotNone(found)
        self.assertEqual(found[2]['method'],'cnn_body_tracking')
        self.assertFalse(found[2]['front_colour_observed'])
        self.assertFalse(found[2]['rear_colour_observed'])
        # 直前位置とCNNの高い出力だけでは、現在の空白画像を検出扱いにしない。
        blank = self.make_board()
        self.assertIsNone(detect_pair_with_body_model(blank,'green','red',None,H,np.linalg.inv(H),model,**options))

    def test_inferred_keypoint_is_not_reported_as_raw_observation(self):
        model = self.train_synthetic_model(front_bgr=(25,25,235),rear_bgr=(40,220,240))
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            model.save(folder/'model.npz')
            video = folder/'occlusion.avi'
            writer = cv2.VideoWriter(str(video),cv2.VideoWriter_fourcc(*'MJPG'),240,(600,600))
            for index in range(6):
                frame = self.make_board()
                draw_synthetic_mouse(frame,(300,250+index),90,span_px=55,
                    front_bgr=(25,25,235),rear_bgr=(40,220,240),draw_rear=index!=3)
                writer.write(frame)
            writer.release()
            result,_,_,_ = pose.read_video_pose(video,'red',rear_colour='yellow',
                nominal_marker_span_mm=50,marker_span_limits_mm=(40,65),
                marker_detection={'body_model':{'enabled':True,'model_file':str(folder/'model.npz')}})
            self.assertFalse(result.loc[3,'marker_detected_raw'])
            self.assertTrue(result.loc[3,'marker_estimated_raw'])
            self.assertEqual(result.attrs['body_detection']['estimated_pair_frames'],1)

    def make_board(self) -> np.ndarray:
        image = np.full((600, 600, 3), 125, dtype=np.uint8)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        for marker_id, (x, y) in zip((5, 7, 6, 4), ((50, 50), (500, 50), (50, 500), (500, 500))):
            marker = cv2.aruco.generateImageMarker(dictionary, marker_id, 50)
            image[y:y + 50, x:x + 50] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
        return image

    def train_synthetic_model(self, front_bgr=(60, 190, 60), rear_bgr=(30, 30, 230)) -> LearnedBodyKeypointModel:
        samples = []
        for idx, (cx, cy, ang) in enumerate(((260, 260, 0.0), (280, 290, 45.0), (310, 300, 90.0), (290, 320, 135.0))):
            frame = self.make_board()
            f_px, r_px = draw_synthetic_mouse(frame, (cx, cy), ang, span_px=55.0, front_bgr=front_bgr, rear_bgr=rear_bgr)
            samples.append((frame, f_px, r_px))
        return LearnedBodyKeypointModel.fit_from_frames(samples, metadata={"mouse": "B"})

    def test_body_model_rejects_background_distractor_and_recovers_faint_front_marker(self):
        """B_green 2000mm/s で発生した「前マーカー色抜け＋盤面上の孤立緑ノイズへの誤ペアリング」を再現し、
        機体モデルが背景ノイズを棄却して真の機体上の前マーカーを検出できることを検証する。
        """
        model = self.train_synthetic_model()
        frame = self.make_board()
        # 機体は (300, 250) で下向き (+90 deg)。前マーカーは彩度が低く小さいため標準閾値では落ちる設定にする
        true_front, true_rear = draw_synthetic_mouse(
            frame, (300, 250), 90.0, span_px=55.0,
            front_bgr=(115, 145, 115),  # 淡い緑 (S~53 < 70 で標準colour_componentsでは未検出)
            rear_bgr=(25, 25, 235),
        )
        # 後マーカー true_rear=(300, 222.5) から距離約55 px の背景 (252, 195) に、大きくて鮮やかな緑ノイズを置く
        distractor_px = np.array([252.0, 195.0], dtype=np.float32)
        cv2.circle(frame, (int(distractor_px[0]), int(distractor_px[1])), 7, (40, 230, 40), -1, cv2.LINE_AA)

        quad = pose.detect_aruco_quad(frame)
        H = cv2.getPerspectiveTransform(quad, pose.pose_world_corners(360.0))
        H_inv = np.linalg.inv(H)

        # 1. 従来の select_marker_pair は背景の鮮やかな緑ノイズ distractor_px を選んでしまう
        legacy_pair = pose.select_marker_pair(
            pose.colour_components(frame, "green"),
            pose.colour_components(frame, "red"),
            H, None, nominal_marker_span_mm=50.0, marker_span_limits_mm=(40.0, 65.0),
        )
        self.assertIsNotNone(legacy_pair)
        legacy_front_px = pose.transform_points([legacy_pair[0]], H_inv)[0]
        self.assertLess(float(np.linalg.norm(legacy_front_px - distractor_px)), 3.0)

        # 2. 機体外観＋キーポイントモデルは背景ノイズのパッチNCCが低いため棄却し、真の機体前マーカーを検出する
        res = detect_pair_with_body_model(
            frame, "green", "red", None, H, H_inv, model,
            nominal_span_mm=50.0, span_limits_mm=(40.0, 65.0),
        )
        self.assertIsNotNone(res)
        front_mm, rear_mm, info = res
        detected_front_px = pose.transform_points([front_mm], H_inv)[0]
        detected_rear_px = pose.transform_points([rear_mm], H_inv)[0]
        np.testing.assert_allclose(detected_front_px, true_front, atol=2.5)
        np.testing.assert_allclose(detected_rear_px, true_rear, atol=2.0)
        self.assertGreater(info["body_ncc"], 0.45)

    def test_single_marker_pivot_recovers_pose_when_rear_marker_is_occluded(self):
        """片方のマーカーが完全に消えたフレームでも、残ったマーカーを起点とする機体テンプレート回転探索で姿勢を復元できる。"""
        model = self.train_synthetic_model(front_bgr=(25, 25, 235), rear_bgr=(40, 220, 240))
        frame = self.make_board()
        # 後マーカーを描画しない (draw_rear=False)
        true_front, true_rear = draw_synthetic_mouse(
            frame, (310, 280), 90.0, span_px=55.0,
            front_bgr=(25, 25, 235), rear_bgr=(40, 220, 240),
            draw_front=True, draw_rear=False,
        )
        quad = pose.detect_aruco_quad(frame)
        H = cv2.getPerspectiveTransform(quad, pose.pose_world_corners(360.0))
        H_inv = np.linalg.inv(H)

        # 従来法は yellow が0個のため None になる
        self.assertIsNone(pose.select_marker_pair(
            pose.colour_components(frame, "red"),
            pose.colour_components(frame, "yellow"),
            H, None, nominal_marker_span_mm=50.0, marker_span_limits_mm=(40.0, 65.0),
        ))

        # 機体モデルは front_pivot_body_search により機体向きを特定して復元する
        res = detect_pair_with_body_model(
            frame, "red", "yellow", None, H, H_inv, model,
            nominal_span_mm=50.0, span_limits_mm=(40.0, 65.0),
        )
        self.assertIsNotNone(res)
        front_mm, rear_mm, info = res
        self.assertEqual(info["method"], "front_pivot_body_search")
        detected_front_px = pose.transform_points([front_mm], H_inv)[0]
        detected_rear_px = pose.transform_points([rear_mm], H_inv)[0]
        np.testing.assert_allclose(detected_front_px, true_front, atol=2.5)
        np.testing.assert_allclose(detected_rear_px, true_rear, atol=5.5)

        # 測定用の両色必須モードでは、外観から補った後点を観測値にしない。
        self.assertIsNone(detect_pair_with_body_model(
            frame, "red", "yellow", None, H, H_inv, model,
            nominal_span_mm=50.0, span_limits_mm=(40.0, 65.0),
            require_colour_pair=True,
        ))

    def test_save_load_and_read_video_pose_integration(self):
        """モデルの保存・読込みと read_video_pose(marker_detection={'body_model': ...}) の統合動作を検証する。"""
        model = self.train_synthetic_model()
        with tempfile.TemporaryDirectory() as folder:
            folder_path = Path(folder)
            model_path = folder_path / "model_B.npz"
            model.save(model_path)
            loaded = LearnedBodyKeypointModel.load(model_path)
            np.testing.assert_allclose(loaded.mean_feat, model.mean_feat, atol=1e-5)

            video_path = folder_path / "synthetic_run.avi"
            writer = cv2.VideoWriter(str(video_path), cv2.VideoWriter_fourcc(*"MJPG"), 240, (600, 600))
            self.assertTrue(writer.isOpened())
            for k in range(6):
                frame = self.make_board()
                # フレーム3だけは前マーカーが淡く、かつ背景に緑ノイズがある難例フレームにする
                f_color = (95, 145, 95) if k == 3 else (50, 210, 50)
                draw_synthetic_mouse(frame, (300, 230 + k * 8), 90.0, span_px=55.0, front_bgr=f_color, rear_bgr=(25, 25, 235))
                if k == 3:
                    cv2.circle(frame, (252, 195), 7, (40, 230, 40), -1, cv2.LINE_AA)
                writer.write(frame)
            writer.release()

            df_body, _, _, _ = pose.read_video_pose(
                video_path,
                "green",
                rear_colour="red",
                nominal_marker_span_mm=50.0,
                marker_span_limits_mm=(40.0, 65.0),
                marker_detection={"body_model": {"enabled": True, "model_file": str(model_path),
                                                "require_colour_pair": True}},
            )
            self.assertTrue(df_body.marker_detected_raw.all())
            # 全6フレームで x 座標がほぼ一定（背景ノイズ x=252 へ飛ばない）ことを確認
            self.assertLess(float(df_body.center_x_mm.std()), 3.0)


if __name__ == "__main__":
    unittest.main()
