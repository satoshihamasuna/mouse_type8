"""CPU CNNの学習・保存・別姿勢推論を既知座標の合成機体で確認する。"""
from pathlib import Path
import sys
import tempfile
import unittest

sys.path[:0] = [str(Path(__file__).resolve().parents[1]),str(Path(__file__).parent)]
from body_keypoint_cnn import BodyKeypointCNN,torch_runtime

try:
    torch_runtime()
except (ImportError,RuntimeError) as error:
    raise unittest.SkipTest(str(error))

import numpy as np
from body_appearance_model import extract_canonical_patch
from test_body_appearance_model import draw_synthetic_mouse


class KeypointCNNTests(unittest.TestCase):
    def test_training_generalizes_to_unseen_pose_and_safe_save_load(self):
        samples = []
        for angle in (0,45,90,135):
            frame = np.full((300,300,3),125,np.uint8)
            front,rear = draw_synthetic_mouse(frame,(150,150),angle,span_px=60)
            samples.append((frame,front,rear))
        model = BodyKeypointCNN.fit_from_frames(samples,dict(mouse='B',front_colour='green',rear_colour='red'),steps=100)
        self.assertLess(model.metadata['training_loss'],6)
        frame = np.full((300,300,3),125,np.uint8)
        front,rear = draw_synthetic_mouse(frame,(150,150),73,span_px=60)
        patch,_ = extract_canonical_patch(frame,front,rear)
        predicted,visibility,_ = model.predict_patch(patch)
        np.testing.assert_allclose(predicted,[[50,32],[14,32]],atol=3.5)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'model.pt'
            model.save(path)
            loaded = BodyKeypointCNN.load(path)
            np.testing.assert_allclose(loaded.predict_patch(patch)[0],predicted,atol=1e-5)
            with self.assertRaises(FileExistsError):
                model.save(path)


if __name__=='__main__':
    unittest.main()
