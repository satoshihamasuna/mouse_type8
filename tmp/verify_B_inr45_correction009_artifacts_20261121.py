"""保存元を再照合して、009の現在の手離し確認待ちを記録する。機器操作なし。"""
from pathlib import Path
from datetime import datetime,timezone
import json
import sys
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr45_20261120_v1'
sys.path.insert(0,str(ROOT/'tools'))
from pixel_camera_archive import verify_archive,verify_sources,file_hash
read=lambda p:json.loads(Path(p).read_text(encoding='utf-8'))
stage=BASE/'geometry03_correction_m0p4';validation=BASE/'red_hue160_validation_v1';report=read(validation/'review.json');archives=[]
for row in report['rows']:
 trial_path=Path(row['source_trial']);pointer=read(trial_path.parent/'archive.json');folder=Path(pointer['archive_folder'])
 manifest=verify_archive(folder);verify_sources(manifest,trial_path)
 assert file_hash(folder/'manifest.json')==pointer['manifest_sha256']
 archives.append(dict(run_id=row['run_id'],archive_folder=str(folder),archive_and_source_files_verified=True,camera_state=manifest['camera_state']))
for p in (validation/'review.json',stage/'offline_validation.json',stage/'preflight_read_only_20261121.json'):
 for filename,digest in read(p)['input_sha256'].items():assert file_hash(filename)==digest,(filename,p)
assert read(stage/'preflight_read_only_20261121.json')['ready_for_new_operator_readiness']
state_path=BASE/'resume_state.json';state=read(state_path);assert not state['next_request_sent']
state['readiness_question_pending']=dict(asked_at_utc=datetime.now(timezone.utc).isoformat(),
 request_id=state['next_request_id'],source='Current async question: coarse original placement, both ON, hands clear, one run',
 answered=False,prior_008_one_run_authorization_consumed=True)
state_path.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
with (stage/'final_artifact_verification.json').open('x',encoding='utf-8') as f:json.dump(dict(checked_at_utc=datetime.now(timezone.utc).isoformat(),
 archives=archives,analysis_and_candidate_input_hashes_match=True,offline_default_runner_passed=True,hardware_commands_sent=0,
 request_sent=False,readiness_question_pending=True,source_params_changed=False,firmware_written=False),f,ensure_ascii=False,indent=2)
print(json.dumps(dict(archives_verified=len(archives),input_hashes_match=True,request_sent=False,readiness_question_pending=True)))
