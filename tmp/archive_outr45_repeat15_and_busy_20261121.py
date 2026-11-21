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
STAGE=BASE/'repeat_cminus1p7_Ls15p4_Le15p9_20261121_02'
ID='A_OUTR45_1800_cminus1p7_Ls15p4_Le15p9_20261121_002'
RUN=STAGE/'local_execution'/('run_'+ID)
FAILED=BASE/'lengths_cminus1p7_Ls17p0_Le17p4_20261121_01'
CURRENT=BASE/'retry_lengths_cminus1p7_Ls17p0_Le17p4_20261121_02'
CURRENT_ID='A_OUTR45_1800_cminus1p7_Ls17p0_Le17p4_20261121_002'
def read(path):return json.loads(path.read_text(encoding='utf-8'))
def sha(path):
    with path.open('rb') as file:return hashlib.file_digest(file,'sha256').hexdigest()
def save(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('x',encoding='utf-8') as file:
        json.dump(value,file,ensure_ascii=False,indent=2,allow_nan=False);file.write('\n')
assert read(RUN/'operator_postrun_confirmation.json')['both_powered_off']
raw_archive=archive_trial(RUN/'trial.json');raw=verify_archive(raw_archive);verify_sources(raw,RUN/'trial.json')
marker=STAGE/'local_execution/analysis'/(hashlib.sha256(os.path.normcase(str(RUN/'trial.json')).encode()).hexdigest()+'.json')
save(marker,dict(state='ANALYSIS_REVIEW_REQUIRED',trial=str(RUN/'trial.json'),archive=str(raw_archive),
    reports=dict(full=dict(returncode=0,summary=read(RUN/'full/summary.json'),summary_json=str(RUN/'full/summary.json'))),
    quick_omitted=True,full_stride=1,errors={},parameter_adoption_allowed=False,robot_commands_sent=False))

def snapshot(name,sources,metadata,receipt_name):
    target=Path('D:/TYPE8_Camera/analyses')/name;target.mkdir(exist_ok=False)
    files={}
    for source in sorted(set(sources)):
        if not source.is_file():raise FileNotFoundError(source)
        relative=source.relative_to(ROOT).as_posix();destination=target/relative
        destination.parent.mkdir(parents=True,exist_ok=True)
        before=sha(source);shutil.copy2(source,destination)
        assert before==sha(source)==sha(destination),relative
        files[relative]=dict(source=str(source),sha256=before,size_bytes=source.stat().st_size)
    save(target/'manifest.json',dict(recorded_at_utc=datetime.now(timezone.utc).isoformat(),files=files,
        robot_commands_sent=False,source_files_deleted=False,**metadata))
    for relative,item in files.items():assert sha(target/relative)==item['sha256']
    receipt=dict(archive=str(target),manifest_sha256=sha(target/'manifest.json'),copied_and_verified_files=len(files),
        raw_archive_verified_files=len(raw['files']) if metadata.get('raw_archive') else 0,robot_commands_sent=False)
    save(BASE/receipt_name,receipt);print(json.dumps(receipt),flush=True)

sources=[p for p in STAGE.rglob('*') if p.is_file() and not p.is_relative_to(STAGE/'local_execution')]
sources += [p for p in (STAGE/'local_execution').glob('*.json') if p.is_file()]+[marker]
for folder in (FAILED,CURRENT):
    for name in ('candidate.settings.json','candidate_basis.json','retry_basis.json','config.json','static_pose_after_settle.py','run_recorded_once.py','input_validation.json'):
        if (folder/name).is_file():sources.append(folder/name)
sources += [BASE/'README.md',BASE/'session_outr45_status_20261121_v13.json']
sources += [ROOT/name for name in ('tmp/ready_outr45_cminus1p7_lengths15_repeat02_20261121.py',
 'tmp/analyze_outr45_cminus1p7_lengths15_repeat02_20261121.py','tmp/confirm_off_outr45_cminus1p7_lengths15_repeat02_20261121.py',
 'tmp/review_outr45_cminus1p7_lengths15_repeat02_tracking_20261121.py','tmp/review_outr45_midpoint_saved_20261121_v5.py',
 'tmp/prepare_outr45_cminus1p7_lengths17_20261121.py','tmp/finalize_outr45_lengths15_repeat02_and_busy_20261121.py',
 'tmp/archive_outr45_repeat15_and_busy_20261121.py','tools/analysis/pixel_camera_tuning_report.py','tools/analysis/pixel_camera_relative_pose.py','tools/pixel_camera_geometry.py')]
snapshot('A_outr45_1800_cminus1p7_lengths15_repeat02_20261121_v1',sources,
    dict(mouse='A',motion='out_r45',raw_archive=str(raw_archive),raw_manifest_sha256=sha(raw_archive/'manifest.json'),parameter_adoption_allowed=False),
    'cminus1p7_lengths15_repeat02_review_archive_receipt20261121_v1.json')
failed_sources=[p for p in FAILED.rglob('*') if p.is_file()]
failed_sources += [ROOT/'tools/logs/pixel_camera/20261121_044157_eadbbf32.capture.json',
    CURRENT/'local_execution'/('run_'+CURRENT_ID)/'operator_postrun_confirmation.json',
    ROOT/'tmp/ready_outr45_cminus1p7_lengths17_20261121.py',
    ROOT/'tmp/prepare_outr45_lengths17_retry02_20261121.py',
    ROOT/'tmp/archive_outr45_repeat15_and_busy_20261121.py']
snapshot('A_outr45_1800_lengths17_camera_BUSY_20261121_v1',failed_sources,
    dict(mouse='A',motion='out_r45',capture_failed=True,camera_error='BUSY',execution_write_attempted=False,
         video_created=False,request_not_replayed=True),
    'lengths17_camera_BUSY_archive_receipt20261121_v1.json')
verify_archive(raw_archive);verify_sources(raw,RUN/'trial.json')
