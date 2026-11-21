"""検証済み後赤色相候補の保存動画再解析を別フォルダーに準備。実機操作なし。"""
from pathlib import Path
import hashlib
import json
import shutil
root=Path(__file__).resolve().parents[1]
base=root/'tools/logs/pixel_camera/turn1600_B_outr135_20261121_resume_v1'
run=base/'geometry_c1p0_Ls5p3_Le31_01/local_execution/run_B_OUTR135_1600_c1p0_Ls5p3_Le31_green_20261121_001'
out=run.parent/(run.name+'_reanalysis_hue165_v1')
validation_path=base/'rear_hue165_validation_v1/sample_validation.json'
read=lambda path:json.loads(path.read_text(encoding='utf-8-sig'))
validation=read(validation_path);assert validation['state']=='SAMPLED_VALIDATION_PASSED_FULL_REPROCESS_REQUIRED'
candidate=Path(validation['candidate'])
assert read(candidate)['marker_detection']['red']['hue_ranges']==[[0,10],[165,179]]
assert not out.exists()
files=('saved_1ms.csv','saved_1ms.settings.json','telemetry.csv','telemetry.settings.json','motion.csv','motion.settings.json',
    'marker_reference_used.json','requested.settings.json','events.jsonl','operator_confirmation.json')
assert all((run/name).is_file() for name in files)
out.mkdir()
for name in files:shutil.copyfile(run/name,out/name)
shutil.copyfile(candidate,out/'body_geometry_used.json')
trial=read(run/'trial.json')
trial['analysis_profile_override']=dict(scope='saved_video_analysis_only',original_trial=str(run/'trial.json'),
    original_body_geometry=str(run/'body_geometry_used.json'),candidate_geometry=str(candidate),
    candidate_geometry_sha256=hashlib.sha256(candidate.read_bytes()).hexdigest(),sample_validation=str(validation_path),
    original_failed_analysis_retained=True,capture_settings_and_raw_sources_unchanged=True,
    actual_new_observations_required=True,inferred_or_interpolated_points_counted_as_observations=False)
trial['full_report']=str(out/'full/summary.json');trial['quick_report']=str(out/'quick/summary.json')
with (out/'trial.json').open('x',encoding='utf-8') as stream:json.dump(trial,stream,ensure_ascii=False,indent=2)
with (out/'provenance.json').open('x',encoding='utf-8') as stream:
    json.dump(dict(original_trial=str(run/'trial.json'),original_trial_sha256=hashlib.sha256((run/'trial.json').read_bytes()).hexdigest(),
        original_files_sha256={name:hashlib.sha256((run/name).read_bytes()).hexdigest() for name in files},
        analysis_only=True,hardware_operations=0,original_run_is_not_modified=True),stream,ensure_ascii=False,indent=2)
print(str(out),flush=True)
