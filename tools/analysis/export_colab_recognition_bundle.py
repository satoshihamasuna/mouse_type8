"""既存の評価manifest・動画・設定・学習パッチをGoogle Colab用ポータブルバンドルへ出力する。元資料は変更しない。"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil
import sys
import zipfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from evaluate_body_detectors import collect_training_hashes, read, resolve_portable_path
from pixel_camera_detection import file_sha256


def _safe_filename(name: str) -> str:
    cleaned = ''.join(ch if ch.isalnum() or ch in '._-' else '_' for ch in name)
    return cleaned or 'item'


def _copy_unique(src: Path, dst_dir: Path, copied_by_sha: dict[str, Path], *, prefix: str = '') -> tuple[Path, str]:
    src = Path(src).resolve()
    digest = file_sha256(src).lower()
    if digest in copied_by_sha:
        return copied_by_sha[digest], digest
    dst_dir.mkdir(parents=True, exist_ok=True)
    stem = _safe_filename((prefix + '_' if prefix else '') + src.stem)
    candidate = dst_dir / f'{stem}{src.suffix}'
    index = 2
    while candidate.exists():
        candidate = dst_dir / f'{stem}_{index}{src.suffix}'
        index += 1
    shutil.copy2(src, candidate)
    copied_by_sha[digest] = candidate
    return candidate, digest


def _rel_posix(path: Path, root: Path) -> str:
    return path.relative_to(root).as_posix()


def export_bundle(
    manifest_path: Path,
    output_dir: Path,
    *,
    appearance_model: Path | None = None,
    cnn_model: Path | None = None,
    patch_dirs: list[Path] | None = None,
    base_dir: Path | None = None,
    create_zip: bool = False,
) -> dict:
    """評価manifestと任意の学習済みモデル・パッチをColab向け相対パス構成で保存する。"""
    manifest_path = Path(manifest_path).resolve()
    output_dir = Path(output_dir)
    if output_dir.exists():
        raise FileExistsError('既存出力フォルダーを上書きしません: ' + str(output_dir))
    zip_path = output_dir.with_suffix('.zip')
    if create_zip and zip_path.exists():
        raise FileExistsError('既存ZIPファイルを上書きしません: ' + str(zip_path))

    effective_base = Path(base_dir).resolve() if base_dir is not None else manifest_path.parent
    raw_manifest = read(manifest_path)
    cases = raw_manifest.get('cases', [])
    if not cases:
        raise ValueError('manifestに評価casesがありません')

    output_dir.mkdir(parents=True, exist_ok=False)
    videos_dir = output_dir / 'videos'
    configs_dir = output_dir / 'configs'
    models_dir = output_dir / 'models'
    patches_root = output_dir / 'patches'

    copied_videos: dict[str, Path] = {}
    copied_configs: dict[str, Path] = {}
    file_hashes: dict[str, str] = {}
    identities: set[tuple[str | None, str | None, str | None]] = set()
    eval_video_hashes: set[str] = set()
    train_video_hashes: set[str] = set()

    portable_cases = []
    for case in cases:
        video_src = resolve_portable_path(case['video'], effective_base)
        geom_src = resolve_portable_path(case['geometry'], effective_base)
        ref_src = resolve_portable_path(case['reference'], effective_base)

        geom_data = read(geom_src)
        identity = (geom_data.get('mouse'), geom_data.get('front_colour'), geom_data.get('rear_colour', 'yellow'))
        identities.add(identity)

        video_dst, video_sha = _copy_unique(video_src, videos_dir, copied_videos)
        geom_dst, geom_sha = _copy_unique(geom_src, configs_dir, copied_configs, prefix='geometry')
        ref_dst, ref_sha = _copy_unique(ref_src, configs_dir, copied_configs, prefix='reference')

        eval_video_hashes.add(video_sha)
        file_hashes[_rel_posix(video_dst, output_dir)] = video_sha
        file_hashes[_rel_posix(geom_dst, output_dir)] = geom_sha
        file_hashes[_rel_posix(ref_dst, output_dir)] = ref_sha

        new_case = dict(case)
        new_case['video'] = _rel_posix(video_dst, output_dir)
        new_case['geometry'] = _rel_posix(geom_dst, output_dir)
        new_case['reference'] = _rel_posix(ref_dst, output_dir)
        portable_cases.append(new_case)

    if len(identities) != 1 or None in next(iter(identities)):
        shutil.rmtree(output_dir, ignore_errors=True)
        raise ValueError(f'機体A/Bまたは前後色が一致しないケースが混在しています: {sorted(identities)}')
    mouse, front_colour, rear_colour = next(iter(identities))

    bundled_models: dict[str, str | None] = {'appearance_model': None, 'cnn_model': None}
    if appearance_model is not None:
        from body_appearance_model import LearnedBodyKeypointModel
        app_src = resolve_portable_path(appearance_model, effective_base)
        loaded_app = LearnedBodyKeypointModel.load(app_src)
        loaded_app.validate_identity(mouse, front_colour, rear_colour)
        train_video_hashes |= collect_training_hashes(loaded_app.metadata.get('sources', []), base_dir=effective_base)
        models_dir.mkdir(parents=True, exist_ok=True)
        app_dst = models_dir / app_src.name
        shutil.copy2(app_src, app_dst)
        bundled_models['appearance_model'] = _rel_posix(app_dst, output_dir)
        file_hashes[bundled_models['appearance_model']] = file_sha256(app_dst).lower()

    if cnn_model is not None:
        cnn_src = resolve_portable_path(cnn_model, effective_base)
        models_dir.mkdir(parents=True, exist_ok=True)
        cnn_dst = models_dir / cnn_src.name
        shutil.copy2(cnn_src, cnn_dst)
        bundled_models['cnn_model'] = _rel_posix(cnn_dst, output_dir)
        file_hashes[bundled_models['cnn_model']] = file_sha256(cnn_dst).lower()
        sidecar = cnn_src.with_suffix('.json')
        if sidecar.is_file():
            sidecar_meta = read(sidecar)
            if any(sidecar_meta.get(k) != v for k, v in (('mouse', mouse), ('front_colour', front_colour), ('rear_colour', rear_colour))):
                shutil.rmtree(output_dir, ignore_errors=True)
                raise ValueError('CNNメタデータの機体・前後色が評価manifestと一致しません')
            train_video_hashes |= collect_training_hashes(sidecar_meta.get('sources', []), base_dir=effective_base)
            sidecar_dst = models_dir / sidecar.name
            shutil.copy2(sidecar, sidecar_dst)
            file_hashes[_rel_posix(sidecar_dst, output_dir)] = file_sha256(sidecar_dst).lower()

    bundled_patch_dirs: list[str] = []
    for raw_patch_dir in patch_dirs or []:
        pdir = resolve_portable_path(raw_patch_dir, effective_base)
        training_json = pdir / 'training.json'
        if not training_json.is_file():
            shutil.rmtree(output_dir, ignore_errors=True)
            raise FileNotFoundError('training.json が見つかりません: ' + str(training_json))
        saved = read(training_json)
        meta = saved.get('metadata', {})
        if any(meta.get(k) != v for k, v in (('mouse', mouse), ('front_colour', front_colour), ('rear_colour', rear_colour))):
            shutil.rmtree(output_dir, ignore_errors=True)
            raise ValueError('学習パッチの機体・前後色が評価manifestと一致しません: ' + str(pdir))
        train_video_hashes |= collect_training_hashes(meta.get('sources', []), base_dir=effective_base)
        dst_patch_dir = patches_root / _safe_filename(pdir.name)
        idx = 2
        while dst_patch_dir.exists():
            dst_patch_dir = patches_root / f'{_safe_filename(pdir.name)}_{idx}'
            idx += 1
        dst_patch_dir.mkdir(parents=True, exist_ok=False)
        shutil.copy2(training_json, dst_patch_dir / 'training.json')
        file_hashes[_rel_posix(dst_patch_dir / 'training.json', output_dir)] = file_sha256(dst_patch_dir / 'training.json').lower()
        for label in saved.get('labels', []):
            patch_src = pdir / label['patch']
            if not patch_src.is_file():
                shutil.rmtree(output_dir, ignore_errors=True)
                raise FileNotFoundError('学習パッチ画像が見つかりません: ' + str(patch_src))
            patch_dst = dst_patch_dir / label['patch']
            shutil.copy2(patch_src, patch_dst)
            file_hashes[_rel_posix(patch_dst, output_dir)] = file_sha256(patch_dst).lower()
        bundled_patch_dirs.append(_rel_posix(dst_patch_dir, output_dir))

    overlap = eval_video_hashes & train_video_hashes
    if overlap:
        shutil.rmtree(output_dir, ignore_errors=True)
        raise ValueError('学習動画と評価動画のSHA-256が重複しています: ' + ', '.join(sorted(overlap)))

    portable_manifest = dict(raw_manifest, cases=portable_cases)
    manifest_out = output_dir / 'manifest.json'
    manifest_out.write_text(json.dumps(portable_manifest, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    file_hashes['manifest.json'] = file_sha256(manifest_out).lower()

    bundle_meta = dict(
        schema_version=1,
        mouse=mouse,
        front_colour=front_colour,
        rear_colour=rear_colour,
        source_manifest=str(manifest_path),
        manifest='manifest.json',
        models=bundled_models,
        patch_dirs=bundled_patch_dirs,
        evaluation_video_sha256=sorted(eval_video_hashes),
        training_video_sha256=sorted(train_video_hashes),
        independent_video_split_verified=True,
        file_sha256=file_hashes,
    )
    bundle_meta_out = output_dir / 'bundle_manifest.json'
    bundle_meta_out.write_text(json.dumps(bundle_meta, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')

    if create_zip:
        with zipfile.ZipFile(zip_path, 'w', compression=zipfile.ZIP_DEFLATED) as archive:
            for item in sorted(output_dir.rglob('*')):
                if item.is_file():
                    archive.write(item, arcname=f'{output_dir.name}/{item.relative_to(output_dir).as_posix()}')
        bundle_meta['zip_file'] = str(zip_path.resolve())

    return bundle_meta


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True, help='evaluate_body_detectors.py用の評価manifest JSON')
    parser.add_argument('--appearance-model', type=Path, default=None, help='任意の学習済み外観モデル(.npz)')
    parser.add_argument('--cnn-model', type=Path, default=None, help='任意の学習済み小型CNN(.pt)')
    parser.add_argument('--patches', type=Path, nargs='*', default=None, help='任意のtrain_turn_appearance.py出力フォルダー')
    parser.add_argument('--base-dir', type=Path, default=None, help='manifest内相対パスの基準ディレクトリ')
    parser.add_argument('--output', type=Path, required=True, help='新規のバンドル出力先フォルダー')
    parser.add_argument('--zip', action='store_true', dest='create_zip', help='Google Driveアップロード用の.zipも生成')
    args = parser.parse_args()
    result = export_bundle(
        args.manifest,
        args.output,
        appearance_model=args.appearance_model,
        cnn_model=args.cnn_model,
        patch_dirs=args.patches,
        base_dir=args.base_dir,
        create_zip=args.create_zip,
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
