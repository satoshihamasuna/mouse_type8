from pathlib import Path
from datetime import datetime,timezone
import hashlib
import json
import os
import shutil
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from pixel_camera_archive import archive_trial,verify_archive,verify_sources
BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_outr45_20261120_v1'
STAGE=BASE/'fixed_lengths_cminus1p7_Ls19p9_Le22p1_20261121_01'
ID='A_OUTR45_1800_cminus1p7_Ls19p9_Le22p1_20261121_001'
RUN=STAGE/'local_execution'/('run_'+ID)
NEXT=BASE/'lengths_cminus1p7_Ls15p4_Le15p9_20261121_01'
def read(path):return json.loads(path.read_text(encoding='utf-8'))
def sha(path):
    with path.open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()
def save(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('x',encoding='utf-8') as file:
        json.dump(value,file,ensure_ascii=False,indent=2,allow_nan=False);file.write('\n')
assert read(RUN/'operator_postrun_confirmation.json')['both_powered_off']
raw_archive=archive_trial(RUN/'trial.json')
raw_manifest=verify_archive(raw_archive)
verify_sources(raw_manifest,RUN/'trial.json')
summary=read(RUN/'full/summary.json')
marker=STAGE/'local_execution/analysis'/(hashlib.sha256(os.path.normcase(str(RUN/'trial.json')).encode('utf-8')).hexdigest()+'.json')
save(marker,dict(state='ANALYSIS_REVIEW_REQUIRED',trial=str(RUN/'trial.json'),
    reports=dict(full=dict(returncode=0,summary=summary,summary_json=str(RUN/'full/summary.json'))),
    quick_omitted=True,full_stride=1,archive=str(raw_archive),errors={},
    independent_optical_review=str(STAGE/'midpoint_brake_clock_v1/midpoint_review.json'),
    parameter_adoption_allowed=False,robot_commands_sent=False))
target=Path('D:/TYPE8_Camera/analyses/A_outr45_1800_cminus1p7_19p9_22p1_20261121_v1')
target.mkdir(exist_ok=False)
sources=[p for p in STAGE.rglob('*') if p.is_file() and not p.is_relative_to(STAGE/'local_execution')]
sources += [p for p in (STAGE/'local_execution').glob('*.json') if p.is_file()]
sources += [marker]
sources += [p for p in NEXT.rglob('*') if p.is_file()]
sources += [BASE/'README.md',BASE/'session_outr45_status_20261121_v11.json',ROOT/'tmp/confirm_off_outr45_cminus1p7_20261121.py']
sources += [ROOT/p for p in ('tmp/ready_outr45_cminus1p7_20261121.py','tmp/analyze_outr45_cminus1p7_20261121.py',
    'tmp/review_outr45_cminus1p7_tracking_20261121.py','tmp/review_outr45_midpoint_saved_20261121_v5.py',
    'tmp/finalize_outr45_cminus1p7_20261121.py','tmp/archive_outr45_cminus1p7_20261121.py',
    'tools/analysis/pixel_camera_tuning_report.py','tools/analysis/pixel_camera_relative_pose.py',
    'tools/pixel_camera_geometry.py','tools/analysis/README.md','Task/Inc/turn_pose_ekf.h')]
files={}
for source in sorted(set(sources)):
    name=source.relative_to(ROOT).as_posix();destination=target/name
    destination.parent.mkdir(parents=True,exist_ok=True)
    before=sha(source);shutil.copy2(source,destination)
    assert before==sha(source)==sha(destination),name
    files[name]=dict(source=str(source),sha256=before,size_bytes=source.stat().st_size)
manifest=dict(schema_version=1,recorded_at_utc=datetime.now(timezone.utc).isoformat(),mouse='A',motion='out_r45',
    speed_mm_s=1800,source_trial=str(RUN/'trial.json'),raw_archive=str(raw_archive),
    raw_manifest_sha256=sha(raw_archive/'manifest.json'),raw_file_count=len(raw_manifest['files']),files=files,
    parameter_adoption_allowed=False,firmware_written=False,robot_commands_sent=False,
    source_files_deleted=False,next_candidate_executed=False)
save(target/'manifest.json',manifest)
for name,item in files.items():assert sha(target/name)==item['sha256'],name
verify_archive(raw_archive);verify_sources(raw_manifest,RUN/'trial.json')
receipt=dict(archive=str(target),manifest_sha256=sha(target/'manifest.json'),copied_and_verified_files=len(files),
    raw_archive_verified_files=len(raw_manifest['files']),robot_commands_sent=False)
save(BASE/'cminus1p7_review_archive_receipt20261121_v1.json',receipt)
print(json.dumps(receipt,ensure_ascii=False),flush=True)
