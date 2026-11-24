"""カメラの保存済み環境をGUIと解析子プロセスで共用する。接続・撮影はしない。"""
from __future__ import annotations

import json
import os
from pathlib import Path
import sys


def initialize_camera_environment(project=None, *, required=False):
    project = Path(project or Path(__file__).resolve().parents[1])
    config = project/'.tools/pixel_camera_runtime/environment.json'
    if not os.environ.get('PIXEL_CAMERA_REPO') and config.is_file():
        try:
            saved = json.loads(config.read_text(encoding='utf-8'))
            source = Path(saved['source_dir'])
            valid = (source/'python/pixel_camera/client.py').is_file()
        except (OSError, ValueError, TypeError, KeyError):
            valid = False
        if not valid:
            if required:
                raise RuntimeError('保存済みのカメラ環境がありません。接続・撮影条件のカメラ環境を確認してください')
        else:
            os.environ['PIXEL_CAMERA_REPO'] = str(source)
    from pixel_camera_capture import camera_workspace
    try:
        source = camera_workspace()
    except RuntimeError:
        if required:
            raise RuntimeError('Pixelカメラ環境がありません。PIXEL_CAMERA_REPOにカメラの保存先を指定してください')
        source = None
    paths = [project/'.tools'/f'camera_python{sys.version_info.major}{sys.version_info.minor}']
    if source:
        paths.append(source/'python')
    paths = [str(path.resolve()) for path in paths if path.is_dir()]
    for path in reversed(paths):
        if path not in sys.path:
            sys.path.insert(0, path)
    inherited = [path for path in os.environ.get('PYTHONPATH', '').split(os.pathsep) if path]
    os.environ['PYTHONPATH'] = os.pathsep.join(dict.fromkeys(paths+inherited))
    return source


def camera_client_options(project):
    runtime = Path(project)/'.tools/pixel_camera_runtime'
    return dict(runtime_dir=runtime) if (runtime/'broker_8765.json').is_file() else {}
