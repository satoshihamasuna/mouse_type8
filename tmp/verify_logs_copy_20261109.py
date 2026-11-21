import hashlib
import json
import os
from datetime import datetime, timezone
from pathlib import Path


ROOT = Path(r'C:\Users\sato1\Documents\git\mouse_type8_VScode\mouse_type8')
SOURCE = ROOT / 'tools/logs'
DESTINATION = Path(r'D:\mouse_type8_data\logs')
REPORT = ROOT / 'tmp/logs_copy_to_d_20261109_verified.json'
MANIFEST = ROOT / 'tmp/logs_copy_to_d_20261109_sha256.json'


def snapshot(base):
    files = {}
    directories = []
    for folder, children, names in os.walk(base, followlinks=False):
        for name in children + names:
            path = Path(folder) / name
            info = path.stat(follow_symlinks=False)
            if info.st_file_attributes & 0x400:
                raise RuntimeError(f'Reparse point found: {path}')
        for name in children:
            directories.append((Path(folder) / name).relative_to(base).as_posix())
        for name in names:
            path = Path(folder) / name
            info = path.stat()
            files[path.relative_to(base).as_posix()] = (info.st_size, info.st_mtime_ns)
    return files, sorted(directories)


def digest(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b''):
            result.update(block)
    return result.hexdigest()


source_before, source_dirs = snapshot(SOURCE)
destination_before, destination_dirs = snapshot(DESTINATION)
if source_dirs != destination_dirs or source_before.keys() != destination_before.keys():
    raise RuntimeError('Source and destination paths differ.')

manifest = []
for relative in sorted(source_before):
    if source_before[relative][0] != destination_before[relative][0]:
        raise RuntimeError(f'File sizes differ: {relative}')
    source_hash = digest(SOURCE / relative)
    destination_hash = digest(DESTINATION / relative)
    if source_hash != destination_hash:
        raise RuntimeError(f'SHA-256 differs: {relative}')
    manifest.append({'relative_path': relative, 'bytes': source_before[relative][0], 'sha256': source_hash})
    if len(manifest) % 1000 == 0:
        print(f'Verified {len(manifest)} / {len(source_before)} files', flush=True)

if snapshot(SOURCE) != (source_before, source_dirs):
    raise RuntimeError('Source changed during verification.')
if snapshot(DESTINATION) != (destination_before, destination_dirs):
    raise RuntimeError('Destination changed during verification.')
if SOURCE.is_symlink() or SOURCE.stat().st_file_attributes & 0x400:
    raise RuntimeError('Source directory was unexpectedly replaced by a link.')

MANIFEST.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
report = {
    'status': 'copied_and_sha256_verified',
    'verified_at_utc': datetime.now(timezone.utc).isoformat(),
    'source': str(SOURCE),
    'destination': str(DESTINATION),
    'files': len(manifest),
    'directories': len(source_dirs),
    'bytes': sum(row['bytes'] for row in manifest),
    'source_unchanged_during_verification': True,
    'destination_unchanged_during_verification': True,
    'source_is_ordinary_directory': True,
    'manifest': str(MANIFEST),
    'user_decision': 'Copy only; keep the original logs directory and running GUIs.',
}
REPORT.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
