"""比較完了時点の資料を新しいD側保存先へ一式コピーし、全件を照合する。"""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json
import shutil
import sys

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_inr45_20261120_v1'
DEST=Path('D:/TYPE8_Camera/analyses/A_inr45_1800_refinement_20261121_final_v1')
assert DEST.resolve().parent==Path('D:/TYPE8_Camera/analyses').resolve()
DEST.mkdir(exist_ok=False)
sys.path.insert(0,str(ROOT/'tools'))
from pixel_camera_archive import verify_archive

def digest(path):
    with Path(path).open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()

files=[]
def copy_file(source,target):
    target.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(source,target)
    expected=digest(source)
    assert digest(target)==expected and source.stat().st_size==target.stat().st_size
    files.append(dict(source=str(source.resolve()),relative_path=target.relative_to(DEST).as_posix(),
        sha256=expected,size_bytes=source.stat().st_size))

def copy_folder(source,target):
    for path in sorted(source.rglob('*')):
        if path.is_file():copy_file(path,target/path.relative_to(source))

for folder in ['refinement_comparison_final20261121_v1','spatial_lines_diagnostic_v3',
               'mid_status_retry_assets','single_turn_candidate_3run_review_v1','mid_identity_checked_01']:
    copy_folder(BASE/folder,DEST/folder)
copy_file(BASE/'session_final_status20261121.json',DEST/'session_final_status20261121.json')

identifiers=['A_INR45_1800_Ls1_Le39p3_20261120_001',
    'A_INR45_1800_matched_baseline_retry1915_20261120_001',
    'A_INR45_1800_mid_status_retry_20261120_002','A_INR45_1800_mid_exact_repeat_20261120_002',
    'A_INR45_1800_mid_identity_checked_20261121_001']
for identifier in identifiers:
    source=BASE/'parallel_analysis'/identifier
    copy_folder(source,DEST/'parallel_analysis'/identifier)

stages=['refine_Ls1_Le39p3_01','matched_baseline1910_01','matched_baseline_retry1915_01',
        'mid_status_retry_02','mid_exact_repeat_02','mid_final_repeat_01','mid_identity_checked_01']
archives=[]
for stage in stages:
    local=BASE/stage/'local_execution'
    for source in sorted((local/'operator_results').glob('*.json')):
        copy_file(source,DEST/'operator_results'/stage/source.name)
    for run in local.glob('run_*'):
        pointer=run/'archive.json'
        if pointer.exists():
            archive=Path(json.loads(pointer.read_text(encoding='utf-8'))['archive_folder'])
            verify_archive(archive)
            archives.append(str(archive))
correction=BASE/'mid_pretransfer_01/local_execution/preflight_before_ble_A_INR45_1800_mid_pretransfer_20261120_001/operator_machine_correction.json'
copy_file(correction,DEST/'wrong_machine_static_capture/operator_machine_correction.json')
copy_file(BASE/'pretransfer_static_failure_review_v1/static_frame_3p8.png',DEST/'wrong_machine_static_capture/static_frame_3p8.png')

sources=['tools/pixel_camera_tuning.py','tools/tests/test_pixel_camera_tuning.py',
    'tools/pixel_camera_tuning_cli.py','tools/pixel_camera_tuning_mcp.py','tools/pixel_camera_README.md',
    'tools/ble/mouse_ble_gateway.py','tools/tests/test_mouse_ble_transport.py','tools/ble/README.md',
    'tools/analysis/video_marker_pose.py','tools/marker_detection.py','tools/analysis/pixel_camera_relative_pose.py',
    'tools/pixel_camera_geometry.py','tools/analysis/pixel_camera_tuning_report.py','tools/pixel_camera_detection.py',
    'tmp/run_inr45_camera_first_20261120.py','tmp/review_inr45_spatial_lines_20261120.py',
    'tmp/review_inr45_midpoint_s20_saved_20261120.py','tmp/review_inr45_confirmed_stop_saved_20261120.py',
    'tmp/parallel_inr45_candidate_tracking_20261120.py','tmp/check_inr45_identity_guard_20261121.py',
    'tmp/collect_inr45_refinement_review_20261120.py',str(Path(__file__).relative_to(ROOT))]
for name in sources:copy_file(ROOT/name,DEST/'source_snapshot'/name)

manifest=dict(created_at_utc=datetime.now(timezone.utc).isoformat(),files=files,
    verified_file_count=len(files),verified_trial_archives=archives,robot_commands_sent=0,
    firmware_changed=False,status='COMPARISON_COMPLETE_NEW_CANDIDATE_NOT_SELECTED')
with (DEST/'manifest.json').open('x',encoding='utf-8') as stream:
    json.dump(manifest,stream,ensure_ascii=False,indent=2);stream.write('\n')
receipt=dict(destination=str(DEST),verified_files=len(files),manifest_sha256=digest(DEST/'manifest.json'),
    verified_trial_archives=archives)
with (BASE/'final_bundle_receipt20261121_v1.json').open('x',encoding='utf-8') as stream:
    json.dump(receipt,stream,ensure_ascii=False,indent=2);stream.write('\n')
print(json.dumps(receipt,ensure_ascii=False,indent=2))
