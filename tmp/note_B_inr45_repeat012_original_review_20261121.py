"""012の取得時条件と元の不成立判定を、派生校正候補から区別して保管する。"""
from pathlib import Path
import hashlib
import json
import shutil
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr45_20261120_v1'
RUN=BASE/'geometry05_repeat01_correction_m1p2/local_execution/run_B_INR45_1600_m1p2_repeat01_green_20261121_012'
profile=BASE/'body_geometry_B_green_red_h160_s30_candidate_v5.json';copy=RUN/'red_s30_pilot_v1/candidate_geometry_used.json'
assert not copy.exists();shutil.copyfile(profile,copy)
read=lambda p:json.loads(Path(p).read_text(encoding='utf-8-sig'))
assert not read(RUN/'operator_charging_condition.json')['charging_completed_before_trial']
record=dict(camera_capture_passed=True,original_v4_required_observation_rate=read(RUN/'full/report.json')['required_motion_window_quality']['observation_rate'],
 original_relative_geometry_accepted=False,original_rejection_record='quality_rejection_record.json',
 charging_completed_before_trial=False,operator_conditions='operator_charging_condition.json',
 s30_pilot_actual_observed_pairs=read(RUN/'red_s30_pilot_v1/pilot.json')['candidate_observed'],
 s30_candidate_snapshot='red_s30_pilot_v1/candidate_geometry_used.json',s30_profile_sha256=hashlib.sha256(copy.read_bytes()).hexdigest(),
 s30_full_heldout_validation_pending=True,derived_validation_destination=str(BASE/'red_s30_validation_v1'),
 original_source_geometry_unchanged=True,original_failure_reclassified=False,hardware_commands_sent=0)
with (RUN/'original_review_and_candidate_scope.json').open('x',encoding='utf-8') as f:json.dump(record,f,ensure_ascii=False,indent=2)
print('Original v4 rejection and before-charge condition retained; S30 pilot snapshot saved')
