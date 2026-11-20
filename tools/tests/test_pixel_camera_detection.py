"""モデル自動選択・機体分離・試行への固定・移動後の再解析を機器なしで検証する。"""
from pathlib import Path
import json
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from pixel_camera_detection import file_sha256,resolve_geometry_detection,freeze_geometry_detection,load_detection_geometry


class DetectionSelectionTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.model = self.root/'body.npz'
        self.model.write_bytes(b'fixture model bytes')
        self.registry = self.root/'registry.json'
        self.registry.write_text(json.dumps(dict(schema_version=1,models={'B:green:red':dict(
            identity=['B','green','red'],model_file=str(self.model),model_sha256=file_sha256(self.model))})),encoding='utf-8')
        self.geometry = dict(mouse='B',front_colour='green',rear_colour='red')

    def test_automatic_selection_is_identity_specific_and_does_not_mutate_input(self):
        selected = resolve_geometry_detection(self.geometry,'auto',registry_path=self.registry)
        self.assertNotIn('marker_detection',self.geometry)
        self.assertEqual(selected['marker_detection']['body_model']['expected_mouse'],'B')
        self.assertTrue(selected['marker_detection']['body_model']['require_colour_pair'])
        with self.assertRaises(ValueError):
            resolve_geometry_detection(dict(self.geometry,mouse='A'),'auto',registry_path=self.registry)

    def test_changed_registry_model_does_not_silently_fall_back(self):
        self.model.write_bytes(b'changed model')
        with self.assertRaises(ValueError):
            resolve_geometry_detection(self.geometry,'auto',registry_path=self.registry)

    def test_frozen_trial_survives_source_retraining_and_archive_relocation(self):
        trial = self.root/'trial'
        selected = freeze_geometry_detection(self.geometry,trial,'auto',registry_path=self.registry)
        (trial/'body_geometry_used.json').write_text(json.dumps(selected),encoding='utf-8')
        self.model.write_bytes(b'retrained source must not affect previous trial')
        moved = self.root/'moved'
        trial.rename(moved)
        loaded = load_detection_geometry(moved/'body_geometry_used.json')
        model = Path(loaded['marker_detection']['body_model']['model_file'])
        self.assertTrue(model.is_relative_to(moved))
        self.assertEqual(model.read_bytes(),b'fixture model bytes')
        model.write_bytes(b'corrupted frozen file')
        with self.assertRaises(ValueError):
            load_detection_geometry(moved/'body_geometry_used.json')

    def test_missing_frozen_model_and_path_escape_are_rejected(self):
        file = self.root/'geometry.json'
        data = dict(self.geometry,marker_detection={'body_model':dict(enabled=True,model_file='unused',model_relative_file='../outside.npz')})
        file.write_text(json.dumps(data),encoding='utf-8')
        with self.assertRaises(ValueError):
            load_detection_geometry(file)
        data['marker_detection']['body_model']['model_relative_file']='missing.npz'
        file.write_text(json.dumps(data),encoding='utf-8')
        with self.assertRaises(ValueError):
            load_detection_geometry(file)

    def test_explicit_legacy_mode_does_not_load_or_require_a_model(self):
        geometry = dict(self.geometry,marker_detection={'body_model':{'enabled':True,'model_file':'missing'}})
        resolved = resolve_geometry_detection(geometry,'legacy')
        self.assertNotIn('body_model',resolved['marker_detection'])
        self.assertEqual(resolve_geometry_detection(self.geometry),self.geometry)


if __name__=='__main__':
    unittest.main()
