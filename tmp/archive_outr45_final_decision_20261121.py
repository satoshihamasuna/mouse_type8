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
STAGE=BASE/'repeat_lengths_cminus1p7_Ls17p0_Le17p4_20261121_03'
ID='A_OUTR45_1800_cminus1p7_Ls17p0_Le17p4_20261121_003'
RUN=STAGE/'local_execution'/('run_'+ID)
AUDIT=BASE/'firmware_outR45_adoption_20261121_v1'
ADOPTION=ROOT/'tools/turn_analysis/20261121_A_turn1800_OUTR45_adoption'
def read(path):return json.loads(path.read_text(encoding='utf-8'))
def sha(path):
    with path.open('rb') as file:return hashlib.file_digest(file,'sha256').hexdigest()
def save(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('x',encoding='utf-8') as file:
        json.dump(value,file,ensure_ascii=False,indent=2,allow_nan=False);file.write('\n')
assert read(RUN/'operator_postrun_confirmation.json')['both_powered_off']
assert read(ADOPTION/'firmware_verification.json')['passed']
raw_archive=archive_trial(RUN/'trial.json');raw=verify_archive(raw_archive);verify_sources(raw,RUN/'trial.json')
marker=STAGE/'local_execution/analysis'/(hashlib.sha256(os.path.normcase(str(RUN/'trial.json')).encode()).hexdigest()+'.json')
save(marker,dict(state='ANALYSIS_REVIEW_REQUIRED',trial=str(RUN/'trial.json'),archive=str(raw_archive),
    reports=dict(full=dict(returncode=0,summary=read(RUN/'full/summary.json'),summary_json=str(RUN/'full/summary.json'))),
    quick_omitted=True,full_stride=1,errors={},this_trial_adopted=False,
    provisional_adoption_record=str(ADOPTION/'adoption.json'),robot_commands_sent=False))

def snapshot(name,sources,receipt_name,metadata):
    target=Path('D:/TYPE8_Camera/analyses')/name;target.mkdir(exist_ok=False);files={}
    for source in sorted(set(sources)):
        relative=source.relative_to(ROOT).as_posix();destination=target/relative
        destination.parent.mkdir(parents=True,exist_ok=True)
        before=sha(source);shutil.copy2(source,destination)
        assert before==sha(source)==sha(destination),relative
        files[relative]=dict(source=str(source),sha256=before,size_bytes=source.stat().st_size)
    save(target/'manifest.json',dict(recorded_at_utc=datetime.now(timezone.utc).isoformat(),files=files,
        robot_commands_sent=False,source_files_deleted=False,**metadata))
    for relative,item in files.items():assert sha(target/relative)==item['sha256'],relative
    receipt=dict(archive=str(target),manifest_sha256=sha(target/'manifest.json'),copied_and_verified_files=len(files),
        raw_archive_verified_files=len(raw['files']),robot_commands_sent=False)
    save(BASE/receipt_name,receipt);print(json.dumps(receipt),flush=True)

sources=[p for p in STAGE.rglob('*') if p.is_file() and not p.is_relative_to(STAGE/'local_execution')]
sources += [p for p in (STAGE/'local_execution').glob('*.json') if p.is_file()]+[marker]
sources += [STAGE/'local_execution/jobs'/(ID+'.json')]
sources += list((STAGE/'local_execution'/('prepare_'+ID)).glob('*.json'))
sources += [BASE/'README.md',BASE/'session_outr45_status_20261121_v15.json']
sources += [ROOT/name for name in ('tmp/analyze_outr45_cminus1p7_lengths17_repeat03_20261121.py',
 'tmp/ready_outr45_cminus1p7_lengths17_repeat03_20261121.py','tmp/confirm_off_outr45_cminus1p7_lengths17_repeat03_20261121.py',
 'tmp/review_outr45_cminus1p7_lengths17_repeat03_tracking_20261121.py','tmp/review_outr45_midpoint_saved_20261121_v5.py',
 'tmp/diagnose_outr45_lengths17_repeat03_post_lend_20261121.py','tmp/visual_check_outr45_lengths17_pair_20261121.py',
 'tmp/review_outr45_board_fixed_pair_20261121.py','tmp/finalize_outr45_decision_20261121.py',
 'tmp/archive_outr45_final_decision_20261121.py','tools/analysis/pixel_camera_tuning_report.py','tools/analysis/pixel_camera_relative_pose.py','tools/pixel_camera_geometry.py')]
snapshot('A_outr45_1800_lengths17_repeat03_visual_and_decision_20261121_v1',sources,
    'lengths17_repeat03_final_archive_receipt20261121_v1.json',dict(mouse='A',motion='out_r45',
        raw_archive=str(raw_archive),raw_manifest_sha256=sha(raw_archive/'manifest.json'),this_trial_adopted=False))

firmware_sources=[p for p in AUDIT.rglob('*') if p.is_file()]+[p for p in ADOPTION.rglob('*') if p.is_file()]
firmware_sources += [ROOT/name for name in ('Params/Param_A/turn_1800/turn_1800.cpp','Params/Param_A/turn_1800/turn_1800.h',
 'Params/typedef_run_param.h','README.md','CMakePresets.json','CMakeLists.txt',
 'tmp/adopt_A_outr45_1800_20261121.py','tmp/build_A_outr45_adoption_20261121.py','tmp/verify_A_outr45_adoption_20261121.py',
 'tmp/prepare_selected_outr45_visual_review_20261121.py','tmp/visual_check_selected_outr45_lengths15_20261121.py',
 'tmp/finalize_outr45_decision_20261121.py','tmp/archive_outr45_final_decision_20261121.py')]
firmware_sources += [BASE/'session_outr45_status_20261121_v15.json']
snapshot('A_outr45_1800_provisional_firmware_20261121_v1',firmware_sources,
    'firmware_outR45_adoption_archive_receipt20261121_v1.json',dict(mouse='A',motion='out_r45',
        adopted_parameters=read(ADOPTION/'adoption.json')['parameters'],source_and_build_verified=True,
        adoption_status='PROVISIONAL_WITH_KNOWN_RESIDUAL',firmware_written=False))
verify_archive(raw_archive);verify_sources(raw,RUN/'trial.json')
