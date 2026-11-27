"""Google Colab向け動画認識率検証パイプライン・バンドル出力・盤面グローバルCNNのホストテスト。"""
from __future__ import annotations

import ast
import json
from pathlib import Path
import sys
import tempfile
import unittest
import zipfile

ROOT = Path(__file__).resolve().parents[2]
TOOLS = ROOT / 'tools'
sys.path[:0] = [str(TOOLS), str(TOOLS / 'analysis'), str(Path(__file__).resolve().parent)]


class ColabNotebookSyntaxTests(unittest.TestCase):
    def test_notebook_json_and_python_cells_are_valid(self):
        nb_path = TOOLS / 'colab' / 'mouse_recognition_colab.ipynb'
        self.assertTrue(nb_path.is_file())
        data = json.loads(nb_path.read_text(encoding='utf-8'))
        self.assertEqual(data.get('nbformat'), 4)
        cells = data.get('cells', [])
        code_cells = [c for c in cells if c.get('cell_type') == 'code']
        self.assertGreaterEqual(len(code_cells), 6)
        for idx, cell in enumerate(code_cells):
            src = ''.join(cell.get('source', []))
            ast.parse(src, filename=f'mouse_recognition_colab.ipynb#cell{idx}')


def _check_vision_deps():
    import cv2
    import numpy as np
    import pandas as pd
    if not hasattr(cv2, 'aruco') or not hasattr(cv2.aruco, 'ArucoDetector'):
        raise ImportError('OpenCV with ArucoDetector is required')
    return cv2, np, pd


def _make_synthetic_board_and_reference(cv2, np):
    image = np.full((600, 600, 3), 125, dtype=np.uint8)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    placements = ((5, 50, 50), (7, 500, 50), (6, 50, 500), (4, 500, 500))
    for marker_id, x, y in placements:
        marker = cv2.aruco.generateImageMarker(dictionary, marker_id, 50)
        image[y:y + 50, x:x + 50] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    corners, ids, _ = detector.detectMarkers(image)
    inspection = dict(
        corners=dict(full=dict(
            ids=[int(i) for i in ids.flatten()],
            corners_px=[c[0].tolist() for c in corners],
        )),
        board_reference=dict(
            available=True,
            method='synthetic_test_board',
            points_px=dict(TL=[100.0, 100.0], TR=[500.0, 100.0], BR=[500.0, 500.0], BL=[100.0, 500.0]),
        ),
    )
    from pixel_camera_reference import create_marker_reference
    ref = create_marker_reference(inspection, image_size=(600, 600), board_side_mm=360.0)
    return image, ref


class ColabEvaluationAndBundleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            cls.cv2, cls.np, cls.pd = _check_vision_deps()
        except ImportError as error:
            raise unittest.SkipTest(str(error))

    def _make_board_and_reference(self):
        return _make_synthetic_board_and_reference(self.cv2, self.np)

    def _make_synthetic_video(self, path: Path, frames: list):
        cv2 = self.cv2
        h, w = frames[0].shape[:2]
        writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*'MJPG'), 240.0, (w, h))
        self.assertTrue(writer.isOpened())
        for frame in frames:
            writer.write(frame)
        writer.release()

    def test_collect_training_hashes_uses_recorded_sha_without_windows_file(self):
        from evaluate_body_detectors import collect_training_hashes
        fake_sha = 'a' * 64
        hashes = collect_training_hashes([{'source_video': 'C:\\nonexistent\\train.mp4', 'video_sha256': fake_sha}])
        self.assertEqual(hashes, {fake_sha})
        with self.assertRaises(ValueError):
            collect_training_hashes([{'note': 'missing both sha and path'}])
        with self.assertRaises(FileNotFoundError):
            collect_training_hashes([{'source_video': 'C:\\nonexistent\\missing_without_sha.mp4'}])

    def test_bundle_export_portable_evaluation_and_leakage_guard(self):
        from body_appearance_model import LearnedBodyKeypointModel, extract_canonical_patch
        from evaluate_body_detectors import evaluate
        from export_colab_recognition_bundle import export_bundle
        from pixel_camera_detection import file_sha256
        from test_body_appearance_model import draw_synthetic_mouse

        cv2 = self.cv2
        base_board, ref = self._make_board_and_reference()

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            # 学習モデル（元動画は実在しないWindowsパスだが video_sha256 を保持）
            train_samples = []
            patch_dir = root / 'model_patches'
            patch_dir.mkdir()
            patch_labels = []
            for idx, ang in enumerate((0.0, 45.0, 90.0, 135.0)):
                frame = base_board.copy()
                f_px, r_px = draw_synthetic_mouse(frame, (300, 300), ang, span_px=55.0)
                train_samples.append((frame, f_px, r_px))
                patch, _ = extract_canonical_patch(frame, f_px, r_px)
                pname = f'patch_{idx:04d}.png'
                cv2.imwrite(str(patch_dir / pname), patch)
                patch_labels.append(dict(patch=pname))

            train_sha = '1' * 64
            app_meta = dict(
                mouse='B',
                front_colour='green',
                rear_colour='red',
                sources=[dict(source_video='C:\\WindowsOnly\\train_video.mp4', video_sha256=train_sha)],
            )
            app_model = LearnedBodyKeypointModel.fit_from_frames(train_samples, metadata=app_meta)
            app_path = root / 'appearance.npz'
            app_model.save(app_path)
            (patch_dir / 'training.json').write_text(
                json.dumps(dict(metadata=app_meta, labels=patch_labels), ensure_ascii=False),
                encoding='utf-8',
            )

            # 独立評価用動画（フレーム2は前マーカーを欠落させて欠測挙動を確認）
            eval_frames = []
            for idx in range(5):
                frame = base_board.copy()
                draw_synthetic_mouse(frame, (295 + idx * 2, 285), 90.0, span_px=55.0, draw_front=idx != 2)
                eval_frames.append(frame)
            eval_video = root / 'eval_run.avi'
            self._make_synthetic_video(eval_video, eval_frames)

            geom_path = root / 'geometry_B.json'
            geom_path.write_text(
                json.dumps(dict(mouse='B', front_colour='green', rear_colour='red',
                                nominal_marker_span_mm=50.0, marker_span_limits_mm=[38.0, 68.0])),
                encoding='utf-8',
            )
            ref_path = root / 'reference_B.json'
            ref_path.write_text(json.dumps(ref), encoding='utf-8')

            manifest_path = root / 'manifest.json'
            manifest_path.write_text(
                json.dumps(dict(
                    variants=['legacy', 'learned_assist'],
                    cases=[dict(
                        label='B_synth',
                        kind='static',
                        video=str(eval_video),
                        geometry='geometry_B.json',
                        reference='reference_B.json',
                        window=[0.0, 1.0],
                        stride=1,
                    )],
                )),
                encoding='utf-8',
            )

            bundle_dir = root / 'colab_bundle_B'
            bundle_info = export_bundle(
                manifest_path,
                bundle_dir,
                appearance_model=app_path,
                patch_dirs=[patch_dir],
                create_zip=True,
            )
            self.assertEqual(bundle_info['mouse'], 'B')
            self.assertTrue((bundle_dir / 'bundle_manifest.json').is_file())
            self.assertTrue(bundle_dir.with_suffix('.zip').is_file())
            with zipfile.ZipFile(bundle_dir.with_suffix('.zip'), 'r') as zf:
                names = set(zf.namelist())
                self.assertIn('colab_bundle_B/manifest.json', names)
                self.assertIn('colab_bundle_B/bundle_manifest.json', names)

            # 既存出力は上書き拒否
            with self.assertRaises(FileExistsError):
                export_bundle(manifest_path, bundle_dir)

            # バンドル内の相対パスmanifest + extra_detectorsで評価が成立する
            exported_manifest = json.loads((bundle_dir / 'manifest.json').read_text(encoding='utf-8'))
            exported_manifest['variants'] = ['legacy', 'learned_assist', 'dummy_custom']

            def dummy_detector(frame, H, H_inv, state, ctx):
                return self.np.array([180.0, 205.0]), self.np.array([180.0, 155.0]), dict(method='dummy_est')

            eval_out = root / 'eval_out'
            report = evaluate(
                exported_manifest,
                bundle_dir / bundle_info['models']['appearance_model'],
                None,
                eval_out,
                base_dir=bundle_dir,
                extra_detectors={'dummy_custom': dummy_detector},
            )
            by_var = {r['variant']: r for r in report['results']}
            self.assertIn('aruco_calibration_rate', by_var['legacy'])
            self.assertIn('max_consecutive_missing_frames', by_var['legacy'])
            self.assertIn('method_counts', by_var['legacy'])
            self.assertEqual(by_var['legacy']['missing_given_calibration'], 1)
            self.assertEqual(by_var['legacy']['max_consecutive_missing_frames'], 1)
            # カスタム検出器はデフォルトで observed=0 / estimated=全AR成立フレーム になる
            self.assertEqual(by_var['dummy_custom']['observed_frames'], 0)
            self.assertEqual(by_var['dummy_custom']['estimated_frames'], by_var['dummy_custom']['calibrated_frames'])

            # 学習動画SHAが評価動画と一致する場合はバンドル出力も評価も拒否
            leaked_meta = dict(
                app_meta,
                sources=[dict(source_video='C:\\WindowsOnly\\leaked.mp4', video_sha256=file_sha256(eval_video))],
            )
            leaked_model = LearnedBodyKeypointModel.fit_from_frames(train_samples, metadata=leaked_meta)
            leaked_path = root / 'leaked.npz'
            leaked_model.save(leaked_path)
            with self.assertRaises(ValueError):
                export_bundle(manifest_path, root / 'leaked_bundle', appearance_model=leaked_path)
            with self.assertRaises(ValueError):
                evaluate(exported_manifest, leaked_path, None, root / 'leaked_eval', base_dir=bundle_dir)


class ColabGpuDetectorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            cls.cv2, cls.np, cls.pd = _check_vision_deps()
            from body_keypoint_cnn import torch_runtime
            torch_runtime()
        except (ImportError, RuntimeError) as error:
            raise unittest.SkipTest(str(error))

    def test_patch_cnn_device_options_and_board_detector_pipeline(self):
        from body_appearance_model import extract_canonical_patch
        from body_keypoint_cnn import BodyKeypointCNN, resolve_device
        from colab_board_detector import BoardKeypointDetector, make_hybrid_board_detector
        from evaluate_body_detectors import evaluate
        from test_body_appearance_model import draw_synthetic_mouse

        cv2 = self.cv2
        np = self.np

        self.assertEqual(resolve_device('cpu'), 'cpu')
        self.assertIn(resolve_device('auto'), ('cpu', 'cuda'))
        with self.assertRaises(ValueError):
            resolve_device('invalid_device')

        # 合成盤面とパッチ作成
        board, ref = _make_synthetic_board_and_reference(cv2, np)

        patches = []
        frame_samples = []
        for ang in (0.0, 60.0, 120.0, 180.0):
            f = board.copy()
            front, rear = draw_synthetic_mouse(f, (300, 300), ang, span_px=55.0)
            frame_samples.append((f, front, rear))
            p, _ = extract_canonical_patch(f, front, rear)
            patches.append(p)

        train_meta = dict(
            mouse='B',
            front_colour='green',
            rear_colour='red',
            sources=[dict(source_video='C:\\virtual\\train.mp4', video_sha256='2' * 64)],
        )

        # 1. BodyKeypointCNN の新引数 (device, batch_size, max_augmented) とCPU互換保存
        patch_cnn = BodyKeypointCNN.fit_from_frames(
            frame_samples,
            train_meta,
            steps=10,
            batch_size=8,
            max_augmented=192,
            device='cpu',
        )
        self.assertEqual(patch_cnn.metadata['training_device'], 'cpu')

        # 2. BoardKeypointDetector の軽量学習・保存・読込・評価ハーネス連携
        board_det = BoardKeypointDetector.fit_from_patches(
            patches,
            train_meta,
            board_side_mm=360.0,
            nominal_span_mm=50.0,
            board_px=128,
            base_channels=8,
            dataset_size=64,
            steps=15,
            batch_size=8,
            device='cpu',
            robust=True,
        )
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            saved_pt = root / 'board_cnn.pt'
            board_det.save(saved_pt)
            with self.assertRaises(FileExistsError):
                board_det.save(saved_pt)
            loaded_det = BoardKeypointDetector.load(saved_pt, device='cpu', min_presence=0.0, max_spread_hm_px=10.0)
            pts, pres, spr = loaded_det.predict_rectified(board)
            self.assertEqual(pts.shape, (2, 2))
            self.assertEqual(pres.shape, (2,))
            self.assertEqual(spr.shape, (2,))
            self.assertTrue(np.isfinite(pts).all())

            # 独立評価動画を作成し、appearance_file=None で legacy + board_cnn_global + hybrid_board_assist を比較
            video = root / 'eval_board.avi'
            writer = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*'MJPG'), 240.0, (600, 600))
            for idx in range(3):
                f = board.copy()
                draw_synthetic_mouse(f, (300, 290 + idx * 4), 90.0, span_px=55.0)
                writer.write(f)
            writer.release()

            geom_path = root / 'geom.json'
            geom_path.write_text(json.dumps(dict(mouse='B', front_colour='green', rear_colour='red',
                                                 nominal_marker_span_mm=50.0, marker_span_limits_mm=[30.0, 80.0])),
                                 encoding='utf-8')
            ref_path = root / 'ref.json'
            ref_path.write_text(json.dumps(ref), encoding='utf-8')

            manifest = dict(
                variants=['legacy', 'board_cnn_global', 'hybrid_board_assist'],
                cases=[dict(label='B_board_eval', kind='static', video=str(video),
                            geometry=str(geom_path), reference=str(ref_path), window=[0.0, 1.0], stride=1)],
            )
            hybrid = make_hybrid_board_detector(loaded_det)
            report = evaluate(
                manifest,
                None,
                None,
                root / 'board_eval_out',
                extra_detectors={'board_cnn_global': loaded_det, 'hybrid_board_assist': hybrid},
            )
            by_var = {r['variant']: r for r in report['results']}
            self.assertEqual(by_var['board_cnn_global']['observed_frames'], 0)
            self.assertEqual(by_var['hybrid_board_assist']['observed_frames'], 3)


if __name__ == '__main__':
    unittest.main()
