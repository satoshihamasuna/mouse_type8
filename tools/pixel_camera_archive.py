"""保存済みの動画・走行CSV・実機設定・解析を同じ試行フォルダーへまとめる。機器操作なし。"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import uuid


def file_hash(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def create_json(path, value):
    with Path(path).open('x', encoding='utf-8') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.write('\n')


def default_root(video):
    configured = os.environ.get('TYPE8_TRIALS_DIR')
    if configured:
        return Path(configured)
    parent = Path(video).parent
    if parent.name == 'remote' and parent.parent.name == 'recordings':
        return parent.parent.parent / 'trials'
    return parent / 'type8_trials'


def verify_archive(folder):
    folder = Path(folder).resolve()
    manifest = read_json(folder / 'manifest.json')
    for name, item in manifest['files'].items():
        path = (folder / name).resolve()
        if not path.is_relative_to(folder) or not path.is_file():
            raise ValueError('保存ファイルが見つかりません: ' + name)
        if path.stat().st_size != item['size_bytes'] or file_hash(path) != item['sha256']:
            raise ValueError('保存ファイルの照合が失敗しました: ' + name)
    return manifest


def archive_for_trial(trial_path):
    """元の試行から一式保存先を参照する。元のtrial.jsonは書き換えない。"""
    pointer = Path(trial_path).parent / 'archive.json'
    if pointer.exists():
        folder = Path(read_json(pointer)['archive_folder'])
        if folder.is_dir():
            return folder
    return None


def trial_files(folder):
    found = []
    for path in sorted(folder.rglob('*')):
        if path.is_symlink() or (getattr(path.lstat(), 'st_file_attributes', 0) & 0x400):
            raise ValueError('試行内のリンクはコピーしません: ' + str(path))
        if path.is_file() and path.relative_to(folder).as_posix() not in ('archive.json', 'archive_error.json'):
            found.append(path)
    return found


def verify_sources(manifest, trial_path):
    expected = set()
    for item in manifest['files'].values():
        if not item['source']:
            continue
        source = Path(item['source'])
        if not source.is_file() or file_hash(source) != item['sha256']:
            raise ValueError('元資料が保存時から変わっています。既存保存は上書きしません: ' + str(source))
        if source.is_relative_to(trial_path.parent):
            expected.add(source)
    if expected != set(trial_files(trial_path.parent)):
        raise ValueError('元試行のファイル構成が変わっています。既存保存は上書きしません')


def archive_trial(trial_path, root=None):
    trial_path = Path(trial_path).resolve()
    trial = read_json(trial_path)
    video = Path(trial['video_file']).resolve()
    csv_path = Path(trial['type8_log_csv']).resolve()
    settings_path = Path(trial['type8_settings_json']).resolve()
    capture_path = Path(trial['capture_json']).resolve()
    capture = read_json(capture_path)
    recording_id = trial.get('recording_id') or capture.get('recording_id') or video.stem
    if not re.fullmatch(r'[A-Za-z0-9_-]{1,160}', recording_id):
        raise ValueError('録画IDを保存フォルダー名にできません')
    root = Path(root if root is not None else default_root(video)).resolve()
    if root.is_relative_to(trial_path.parent):
        raise ValueError('元の試行フォルダー内へ再帰的に保存できません')
    expected_hash = capture.get('sha256') or capture.get('video_inspection', {}).get('sha256')
    if expected_hash and file_hash(video) != expected_hash:
        raise ValueError('元動画のSHA-256が撮影記録と一致しません')
    root.mkdir(parents=True, exist_ok=True)
    target = root / recording_id
    lock = root / ('.' + recording_id + '.archive.lock')
    # 同時保存と既存結果の上書きを防ぐ。中断時のpartialは証拠として残す。
    with lock.open('x', encoding='ascii') as stream:
        stream.write(str(os.getpid()))
    try:
        if target.exists():
            manifest = verify_archive(target)
            if (manifest['source_trial_sha256'] != file_hash(trial_path)
                    or manifest['files']['motion.csv']['sha256'] != file_hash(csv_path)
                    or manifest['files']['motion.settings.json']['sha256'] != file_hash(settings_path)):
                raise ValueError('同じ録画IDに異なる入力があります。既存保存は上書きしません')
            verify_sources(manifest, trial_path)
        else:
            stage = root / ('.' + recording_id + '.' + uuid.uuid4().hex[:8] + '.partial')
            stage.mkdir()
            files = {}

            def register(name, source=None, storage='copy'):
                path = stage / name
                files[name] = dict(sha256=file_hash(path), size_bytes=path.stat().st_size,
                                   source=str(source) if source else None, storage=storage)

            def copy(source, name):
                source = Path(source)
                destination = stage / name
                if destination.exists():
                    if file_hash(source) != file_hash(destination):
                        raise ValueError('保存ファイル名が衝突しました: ' + name)
                    return
                destination.parent.mkdir(parents=True, exist_ok=True)
                before = file_hash(source)
                shutil.copy2(source, destination)
                if before != file_hash(destination) or before != file_hash(source):
                    raise ValueError('コピー中に元資料が変わりました: ' + str(source))
                register(name, source.resolve())

            for source in trial_files(trial_path.parent):
                name = source.relative_to(trial_path.parent).as_posix()
                if source == trial_path:
                    name = 'source_trial.json'
                elif name == 'README.md':
                    name = 'source_README.md'
                copy(source, name)
            copy(csv_path, 'motion.csv')
            copy(settings_path, 'motion.settings.json')
            copy(capture_path, 'capture.json')
            # MP4の追加コピーを作らない。別ボリューム等で作れなければ元資料を残して失敗する。
            os.link(video, stage / 'video.mp4')
            if not os.path.samefile(video, stage / 'video.mp4'):
                raise ValueError('動画のハードリンクを確認できません')
            register('video.mp4', video, 'hardlink')
            metadata_path = Path(capture['metadata_file']).resolve() if capture.get('metadata_file') else None
            if metadata_path:
                for source in sorted(metadata_path.parent.glob(video.stem + '.*')):
                    if source.is_file() and source.suffix.lower() != '.mp4':
                        copy(source, 'camera/' + source.name)
                if 'camera/' + metadata_path.name not in files:
                    copy(metadata_path, 'camera/' + metadata_path.name)
            derived = dict(trial)
            derived.update(video_file=str(target / 'video.mp4'), type8_log_csv=str(target / 'motion.csv'),
                type8_settings_json=str(target / 'motion.settings.json'), capture_json=str(target / 'capture.json'),
                quick_report=str(target / 'quick/summary.json'), full_report=str(target / 'full/summary.json'),
                archive_source_trial_json=str(target / 'source_trial.json'))
            create_json(stage / 'trial.json', derived)
            register('trial.json', storage='derived_index')
            settings = read_json(settings_path)
            summary_path = stage / 'full/summary.json'
            summary = read_json(summary_path) if summary_path.exists() else {}
            text = ('# TYPE8 撮影記録\n\n'
                f"機体{trial['mouse']} / {trial['motion']} / {trial['preset']} mm/s / {recording_id}\n\n"
                '- [動画](video.mp4)\n- [走行CSV](motion.csv)\n- [実機設定](motion.settings.json)\n'
                '- [全フレーム結果](full/summary.json)\n- [撮影記録](capture.json)\n\n'
                'trial.jsonはこの保存先を参照する索引です。source_trial.jsonと解析結果は元資料を保持しています。\n'
                '解析JSON内の元の絶対パスと入力ハッシュは書き換えていません。manifest.jsonのsourceから対応を確認できます。\n'
                '動画は元MP4と同じ実体を参照するハードリンクで、追加の動画容量を消費しません。\n')
            with (stage / 'README.md').open('x', encoding='utf-8') as stream:
                stream.write(text)
            register('README.md', storage='generated')
            manifest = dict(schema_version=1, recording_id=recording_id, archived_at=datetime.now(timezone.utc).isoformat(),
                source_trial=str(trial_path), source_trial_sha256=file_hash(trial_path),
                mouse=trial['mouse'], motion=trial['motion'], preset=trial['preset'],
                parameters=settings.get('parameters', {}), camera_state=trial.get('camera_state'),
                detailed_stop_confirmed=summary.get('auto_stop_confirmed'),
                endpoint_forward_mm=summary.get('endpoint_forward_mm'), endpoint_right_mm=summary.get('endpoint_right_mm'),
                files=files, robot_commands_sent=False, source_files_deleted=False,
                analysis_provenance='Copied unchanged; derived trial.json is a relocated index, not the original analysis input.')
            create_json(stage / 'manifest.json', manifest)
            verify_sources(manifest, trial_path)
            verify_archive(stage)
            stage.rename(target)
        pointer = dict(archive_folder=str(target), manifest_sha256=file_hash(target / 'manifest.json'))
        pointer_path = trial_path.parent / 'archive.json'
        if pointer_path.exists() and read_json(pointer_path) != pointer:
            raise ValueError('元試行に異なる一式保存先が登録済みです')
        if not pointer_path.exists():
            create_json(pointer_path, pointer)
        return target
    finally:
        lock.unlink()


def write_index(root):
    root = Path(root)
    rows = []
    for folder in sorted(root.iterdir()):
        if folder.name.startswith('.') or not (folder / 'manifest.json').is_file():
            continue
        item = read_json(folder / 'manifest.json')
        rows.append(dict(recording_id=item['recording_id'], mouse=item['mouse'], motion=item['motion'],
            preset=item['preset'], **{key:item['parameters'].get(key) for key in ('r_min', 'lstart', 'lend')},
            endpoint_forward_mm=item['endpoint_forward_mm'], endpoint_right_mm=item['endpoint_right_mm'],
            detailed_stop_confirmed=item['detailed_stop_confirmed'],
            folder=folder.name, video=folder.name + '/video.mp4', csv=folder.name + '/motion.csv'))
    temporary = root / ('.index.' + uuid.uuid4().hex[:8] + '.tmp')
    if rows:
        with temporary.open('x', encoding='utf-8-sig', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader(); writer.writerows(rows)
        temporary.replace(root / 'index.csv')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial', type=Path, required=True, action='append')
    parser.add_argument('--root', type=Path)
    args = parser.parse_args()
    for path in args.trial:
        folder = archive_trial(path, args.root)
        write_index(folder.parent)
        print(str(folder), flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
