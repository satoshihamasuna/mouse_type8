from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
source=(ROOT/'tmp/archive_outr45_cminus1p7_lengths15_20261121.py').read_text(encoding='utf-8')
mapping={
 'lengths_cminus1p7_Ls15p4_Le15p9_20261121_01':'retry_lengths_cminus1p7_Ls17p0_Le17p4_20261121_02',
 'A_OUTR45_1800_cminus1p7_Ls15p4_Le15p9_20261121_001':'A_OUTR45_1800_cminus1p7_Ls17p0_Le17p4_20261121_002',
 'repeat_cminus1p7_Ls15p4_Le15p9_20261121_02':'repeat_lengths_cminus1p7_Ls17p0_Le17p4_20261121_03',
 'A_outr45_1800_cminus1p7_lengths15p4_15p9_20261121_v1':'A_outr45_1800_cminus1p7_lengths17_retry02_20261121_v1',
 'session_outr45_status_20261121_v12.json':'session_outr45_status_20261121_v14.json',
 'cminus1p7_lengths15_review_archive_receipt':'lengths17_retry02_review_archive_receipt',
 'ready_outr45_cminus1p7_lengths15_20261121.py':'ready_outr45_cminus1p7_lengths17_retry02_20261121.py',
 'analyze_outr45_cminus1p7_lengths15_20261121.py':'analyze_outr45_cminus1p7_lengths17_retry02_20261121.py',
 'confirm_off_outr45_cminus1p7_lengths15_20261121.py':'confirm_off_outr45_cminus1p7_lengths17_retry02_20261121.py',
 'review_outr45_cminus1p7_lengths15_tracking_20261121.py':'review_outr45_cminus1p7_lengths17_retry02_tracking_20261121.py',
 'finalize_outr45_cminus1p7_lengths15_20261121.py':'finalize_outr45_lengths17_retry02_20261121.py',
 'archive_outr45_cminus1p7_lengths15_20261121.py':'archive_outr45_lengths17_retry02_20261121.py',
}
for old,new in mapping.items():
    assert old in source,old
    source=source.replace(old,new)
source=source.replace("[p for p in NEXT.rglob('*') if p.is_file()]","[p for p in NEXT.glob('*') if p.is_file()]")
source=source.replace("files={}\n", "sources += [ROOT/'tmp/diagnose_outr45_lengths17_post_lend_20261121.py',ROOT/'tmp/prepare_outr45_lengths17_retry02_20261121.py',ROOT/'tmp/prepare_archive_outr45_lengths17_retry02_20261121.py']\nfiles={}\n")
with (ROOT/'tmp/archive_outr45_lengths17_retry02_20261121.py').open('x',encoding='utf-8') as file:file.write(source)
print('Prepared final archive script; no robot commands sent.')
