"""機体・前後色から検出モデルを選び、各試行へ使用モデルを固定する。機器操作なし。"""
from __future__ import annotations

from copy import deepcopy
import hashlib
import json
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_REGISTRY = Path(__file__).with_name('pixel_camera_detector_models.json')
SELECTION_MODES = ('profile', 'auto', 'legacy')


def file_sha256(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def project_path(path):
    value = Path(path)
    return value if value.is_absolute() else ROOT/value


def resolve_geometry_detection(geometry, mode='profile', *, registry_path=None):
    """既存JSONを変更せず解決する。モデル不足・不一致を黙って従来法へ戻さない。"""
    if mode not in SELECTION_MODES:
        raise ValueError('検出方式はprofile/auto/legacyを指定してください')
    result = deepcopy(geometry)
    detection = result.get('marker_detection', {})
    configured = detection.get('body_model', {})
    if mode=='legacy':
        detection.pop('body_model', None)
        result['detector_provenance'] = dict(selection_mode='legacy', backend='colour_components')
        return result
    automatic = mode=='auto' or configured.get('selection')=='auto'
    if not automatic:
        if configured and configured.get('enabled',True):
            for file_key,hash_key in (('model_file','model_sha256'),('keypoint_model_file','keypoint_model_sha256')):
                if configured.get(file_key):
                    source = project_path(configured[file_key]).resolve()
                    digest = file_sha256(source)
                    if configured.get(hash_key) and configured[hash_key]!=digest:
                        raise ValueError('使用モデルが保存時から変わっています: '+str(source))
                    configured[file_key],configured[hash_key] = str(source),digest
            if result.get('mouse'):
                configured['expected_mouse'] = result['mouse']
        return result
    identity = (result.get('mouse'), result.get('front_colour'), result.get('rear_colour', 'yellow'))
    if identity[0] not in ('A','B') or not identity[1] or identity[1]==identity[2]:
        raise ValueError('検出モデルの自動選択には機体A/Bと異なる前後色が必要です')
    registry_path = Path(registry_path or (DEFAULT_REGISTRY if DEFAULT_REGISTRY.is_file() else
        ROOT/'tools/logs/pixel_camera/detector_models/registry.json'))
    if not registry_path.is_file():
        raise ValueError('学習モデルが未登録です。独立動画で検証したモデルのregistry.jsonを用意してください: '+str(registry_path))
    registry = json.loads(registry_path.read_text(encoding='utf-8'))
    if registry.get('schema_version')!=1:
        raise ValueError('検出モデル一覧の版が未対応です')
    key = ':'.join(identity)
    entry = registry.get('models', {}).get(key)
    if not entry or entry.get('identity')!=list(identity):
        raise ValueError(f'{key}の学習モデルがありません。モデル登録を確認してください')
    cfg = dict(enabled=True, model_file=str(project_path(entry['model_file']).resolve()),
        model_sha256=entry['model_sha256'], expected_mouse=identity[0],
        require_colour_pair=entry.get('require_colour_pair', True), fallback_only=entry.get('fallback_only', False), auto_adapt=False)
    for name in ('keypoint_model_file','keypoint_model_sha256','min_body_ncc','center_margin_mm'):
        if name in entry:
            cfg[name] = str(project_path(entry[name]).resolve()) if name.endswith('_file') else entry[name]
    for file_key, hash_key in (('model_file','model_sha256'),('keypoint_model_file','keypoint_model_sha256')):
        if file_key not in cfg:
            continue
        if not cfg.get(hash_key) or file_sha256(cfg[file_key])!=cfg[hash_key]:
            raise ValueError(f'{key}の{file_key}が登録時から変わっています。再検証してください')
    result.setdefault('marker_detection', {})['body_model'] = cfg
    result['detector_provenance'] = dict(selection_mode='auto', identity=list(identity),
        registry_file=str(registry_path.resolve()), registry_sha256=file_sha256(registry_path),
        backend='body_template_and_cnn' if cfg.get('keypoint_model_file') else 'learned_body_template',
        validation=deepcopy(entry.get('validation', {})))
    return result


def freeze_geometry_detection(geometry, trial_folder, mode='profile', *, registry_path=None):
    """使用ファイルを試行内へコピーし、後の再学習から測定の根拠を保護する。"""
    result = resolve_geometry_detection(geometry, mode, registry_path=registry_path)
    cfg = result.get('marker_detection', {}).get('body_model', {})
    if not cfg.get('enabled', bool(cfg)):
        return result
    folder = Path(trial_folder).resolve()
    for file_key, hash_key, relative_key in (
        ('model_file','model_sha256','model_relative_file'),
        ('keypoint_model_file','keypoint_model_sha256','keypoint_model_relative_file')):
        if not cfg.get(file_key):
            continue
        source = project_path(cfg[file_key]).resolve()
        digest = file_sha256(source)
        if cfg.get(hash_key) and cfg[hash_key]!=digest:
            raise ValueError('モデルのハッシュが一致しません: '+str(source))
        target = folder/'detector_models'/(digest[:16]+source.suffix)
        target.parent.mkdir(parents=True, exist_ok=True)
        if target.exists():
            if file_sha256(target)!=digest:
                raise ValueError('試行内モデルを上書きしません')
        else:
            shutil.copy2(source,target)
        if file_sha256(target)!=digest or file_sha256(source)!=digest:
            raise ValueError('モデルの保存中にファイルが変わりました')
        cfg[file_key], cfg[hash_key] = str(target), digest
        cfg[relative_key] = target.relative_to(folder).as_posix()
    return result


def load_detection_geometry(path, mode='profile'):
    path = Path(path)
    geometry = json.loads(path.read_text(encoding='utf-8'))
    cfg = geometry.get('marker_detection', {}).get('body_model', {})
    for file_key, relative_key in (('model_file','model_relative_file'),
                                   ('keypoint_model_file','keypoint_model_relative_file')):
        if cfg.get(relative_key):
            local = (path.parent/cfg[relative_key]).resolve()
            if not local.is_relative_to(path.parent.resolve()):
                raise ValueError('試行内モデルの相対パスが保存先の外を指しています')
            if not local.is_file():
                raise ValueError('試行に固定したモデルがありません: '+str(local))
            cfg[file_key] = str(local)
    return resolve_geometry_detection(geometry, mode)
