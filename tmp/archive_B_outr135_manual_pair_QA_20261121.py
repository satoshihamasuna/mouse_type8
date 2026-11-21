"""照合済みの解析資料だけを新規ZIPへ保存し、全項目をハッシュ検証する。"""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json
import zipfile

root=Path(__file__).resolve().parents[1]
base=root/'tools/logs/pixel_camera/turn1600_B_outr135_20261121_resume_v1'
qa=base/'geometry_same_candidate_three_review_v2_manual_pairs'
destination=Path('D:/TYPE8_Camera/analysis_reviews/20261121_B_OUTR135_1600_three_manual_pairs_v1.zip')
assert not destination.exists()
destination.parent.mkdir(parents=True,exist_ok=True)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
folders=[qa,base/'all_required_marker_pairs_review_v1',base/'three_trajectory_kink_review_v1',
         base/'geometry_same_candidate_three_review_v1']
files=[p for folder in folders for p in sorted(folder.rglob('*')) if p.is_file()]
files += [root/'tmp/finalize_B_outr135_manual_pair_QA_20261121.py',
          root/'tmp/review_B_outr135_all_required_pairs_20261121.py',Path(__file__)]
entries={str(p.relative_to(root)).replace('\\','/'):dict(source=str(p),sha256=sha(p),bytes=p.stat().st_size) for p in files}
manifest=dict(recorded_at=datetime.now(timezone.utc).isoformat(),purpose='B1600右out135・保存動画の実マーカー照合後の別解析資料',
    entries=entries,hardware_operations=0,source_archives_modified=False,
    source_trial_archives=['D:/TYPE8_Camera/trials/20261121_002308_240fps_805adbf7',
      'D:/TYPE8_Camera/trials/reanalysis_rear_hue165/20261121_002308_240fps_805adbf7',
      'D:/TYPE8_Camera/trials/20261121_005727_240fps_32951d02',
      'D:/TYPE8_Camera/trials/20261121_013835_240fps_c74b0981'])
with zipfile.ZipFile(destination,'x',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as archive:
    for name,entry in entries.items():archive.write(entry['source'],arcname=name)
    archive.writestr('archive_manifest.json',json.dumps(manifest,ensure_ascii=False,indent=2))
with zipfile.ZipFile(destination) as archive:
    assert archive.testzip() is None
    assert len(archive.namelist())==len(entries)+1
    for name,entry in entries.items():
        assert hashlib.sha256(archive.read(name)).hexdigest()==entry['sha256']
        assert sha(Path(entry['source']))==entry['sha256']
result=dict(archive=str(destination),archive_sha256=sha(destination),verified_files=len(entries),
    archive_bytes=destination.stat().st_size,all_entry_hashes_verified=True,source_files_unchanged=True,
    hardware_operations=0,recorded_at=datetime.now(timezone.utc).isoformat())
with (base/'manual_pair_QA_archive.json').open('x',encoding='utf-8') as stream:
    json.dump(result,stream,ensure_ascii=False,indent=2);stream.write('\n')
print(json.dumps(result,ensure_ascii=False),flush=True)
