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


def _check_vision_deps():
    import cv2
    import numpy as np
    import pandas as pd
    if not hasattr(cv2, 'aruco') or not hasattr(cv2.aruco, 'ArucoDetector'):
        raise ImportError('OpenCV with ArucoDetector is required')
    return cv2, np, pd



class ColabEvaluationAndBundleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            cls.cv2, cls.np, cls.pd = _check_vision_deps()
        except ImportError as error:
            raise unittest.SkipTest(str(error))


    def test_collect_training_hashes_uses_recorded_sha_without_windows_file(self):
        from evaluate_body_detectors import collect_training_hashes
        fake_sha = 'a' * 64
        hashes = collect_training_hashes([{'source_video': 'C:\\nonexistent\\train.mp4', 'video_sha256': fake_sha}])
        self.assertEqual(hashes, {fake_sha})
        with self.assertRaises(ValueError):
            collect_training_hashes([{'note': 'missing both sha and path'}])
        with self.assertRaises(FileNotFoundError):
            collect_training_hashes([{'source_video': 'C:\\nonexistent\\missing_without_sha.mp4'}])



if __name__ == '__main__':
    unittest.main()
