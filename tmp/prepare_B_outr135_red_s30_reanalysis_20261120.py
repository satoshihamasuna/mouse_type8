"""採否前の検証済み色候補で、原本を保持した別索引の再解析を準備。機器操作なし。"""
from pathlib import Path
import hashlib
import json
import shutil
ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_outr135_20261120_v1'
RUN=BASE/'baseline01/local_execution/run_B_OUTR135_1600_baseline_green_20261120_003'
OUT=RUN/'rear_red_s30_reanalysis_v1'
VALIDATION=BASE/'rear_red_s30_validation_v1/sample_validation.json'
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
validation=read(VALIDATION);assert validation['state']=='SAMPLED_VALIDATION_PASSED_FULL_REPROCESS_REQUIRED'
candidate=Path(validation['candidate']);assert read(candidate)['marker_detection']['green']['minimum_area_px']==15
assert read(candidate)['marker_detection']['red']['minimum_saturation']==30
assert not OUT.exists();OUT.mkdir()
files=('saved_1ms.csv','saved_1ms.settings.json','telemetry.csv','telemetry.settings.json','motion.csv','motion.settings.json',
    'marker_reference_used.json','requested.settings.json','events.jsonl','operator_confirmation.json')
for filename in files:shutil.copyfile(RUN/filename,OUT/filename)
shutil.copyfile(candidate,OUT/'body_geometry_used.json')
trial=read(RUN/'trial.json')
trial['analysis_profile_override']=dict(scope='saved_video_analysis_only',original_trial=str(RUN/'trial.json'),
    original_body_geometry=str(RUN/'body_geometry_used.json'),candidate_geometry=str(candidate),
    candidate_geometry_sha256=hashlib.sha256(candidate.read_bytes()).hexdigest(),sample_validation=str(VALIDATION),
    original_failed_analysis_retained=True,capture_settings_and_raw_sources_unchanged=True,
    actual_new_observations_required=True,inferred_or_interpolated_points_counted_as_observations=False)
trial['full_report']=str(OUT/'full/summary.json');trial['quick_report']=str(OUT/'quick/summary.json')
with (OUT/'trial.json').open('x',encoding='utf-8') as file:json.dump(trial,file,ensure_ascii=False,indent=2)
with (OUT/'provenance.json').open('x',encoding='utf-8') as file:json.dump(dict(original_trial_sha256=hashlib.sha256((RUN/'trial.json').read_bytes()).hexdigest(),
    original_files_sha256={name:hashlib.sha256((RUN/name).read_bytes()).hexdigest() for name in files},
    analysis_only=True,hardware_operations=0),file,ensure_ascii=False,indent=2)
print(str(OUT))
