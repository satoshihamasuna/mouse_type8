"""検証済み5ファイルだけを反映する。確認後の変更があれば上書きしない。"""
from pathlib import Path
import hashlib
import json

stage=Path(__file__).resolve().parent
manifest=json.loads((stage/'external_patch_manifest.json').read_text(encoding='utf-8'))
destination=Path(manifest['destination']).resolve()
expected=Path(r'C:\Users\sato1\Documents\git\Debug_camera').resolve()
if destination != expected:
    raise RuntimeError('Unexpected destination')
updates=[]
for entry in manifest['files']:
    target=(destination/entry['path']).resolve()
    if destination not in target.parents:
        raise RuntimeError('Target outside camera repository')
    current=hashlib.sha256(target.read_bytes()).hexdigest() if target.exists() else None
    if current != entry['before_sha256']:
        raise RuntimeError('File changed since review: '+entry['path'])
    updated=(stage/entry['path']).read_bytes()
    if hashlib.sha256(updated).hexdigest() != entry['after_sha256']:
        raise RuntimeError('Patch changed since review: '+entry['path'])
    updates.append((target,updated))
for target,updated in updates:
    target.write_bytes(updated)
print('Applied camera patch:',len(updates),'files')
