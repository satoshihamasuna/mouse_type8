"""未走行の+0.5度候補を、今回用S35検出と新しいIDで準備。機器へ接続しない。"""
from pathlib import Path
import copy
import hashlib
import json
import shutil
import sys

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_r180_20261120_v1'
OLD=BASE/'angle_c0p5_01'
OUT=BASE/'angle_c0p5_s35_02'
VALIDATION=BASE/'green_s35_validation_v3'
read=lambda p:json.loads(Path(p).read_text(encoding='utf-8-sig'))


def save(path,value):
    with path.open('x',encoding='utf-8') as output:
        json.dump(value,output,ensure_ascii=False,indent=2,allow_nan=False)


OUT.mkdir(exist_ok=False)
config=read(OLD/'config.json')
source=VALIDATION/'body_geometry_B_green_front_s35_v3.json'
shutil.copyfile(source,OUT/'body_geometry_B_green_front_s35_v3.json')
config['geometry']=str(OUT/'body_geometry_B_green_front_s35_v3.json')
save(OUT/'config.json',config)
candidate=read(OLD/'candidate.settings.json')
candidate['source']+=' 最初の要求は静止照合で終了し実機未送信。次は今回条件用S35を別検証し、新規IDで同じ角度候補を比較する。'
save(OUT/'candidate.settings.json',candidate)
shutil.copyfile(OLD/'static_pose_after_settle.py',OUT/'static_pose_after_settle.py')
identifier='B_R180_1600_c0p5_s35_green_20261120_002'
code=(OLD/'run_recorded_once.py').read_text(encoding='utf-8')
assert code.count('B_R180_1600_c0p5_green_20261120_001')==1
code=code.replace('B_R180_1600_c0p5_green_20261120_001',identifier)
compile(code,str(OUT/'run_recorded_once.py'),'exec')
with (OUT/'run_recorded_once.py').open('x',encoding='utf-8') as output:output.write(code)
plan=read(OLD/'placement_plan.json')
plan.update(state='DETECTOR_S35_CANDIDATE_PREPARED_NOT_EXECUTED',
    candidate=str(OUT/'candidate.settings.json'),config=str(OUT/'config.json'),
    next_unsent_request_id=identifier,current_placement_confirmed=True,
    current_clearance_confirmed=False,current_hands_clear_confirmed=False,
    power_state='BOTH_OFF_USER_CONFIRMED',execution_commands_sent=0,settings_write_commands_sent=0)
save(OUT/'placement_plan.json',plan)
validation=read(VALIDATION/'static_validation.json')
save(OUT/'detector_basis.json',dict(state='CURRENT_CONDITION_CANDIDATE_FRESH_OFF_VALIDATION_REQUIRED',
    original_failed_request='B_R180_1600_c0p5_green_20261120_001',original_request_execution_write_attempted=False,
    settings_same_as_unexecuted_candidate=True,green_minimum_saturation=35,
    static_validation=validation,all_historical_static_cases_passed=False,
    scope='今回の通電時静止と前回の通電時静止、独立OFF02は既存条件を通過。反射が強い元のOFF01は不合格のまま保持。汎用検出として採用せず、新しいOFF試写と通電時静止・走行全区間を毎回照合する。',
    thresholds_for_position_heading_observation_unchanged=True,
    source_geometry=str(source),source_geometry_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    source_validation=str(VALIDATION/'static_validation.json'),source_params_changed=False))
sys.path.insert(0,str(ROOT/'tools'))
from pixel_camera_tuning_mcp import McpTuningService
McpTuningService(OUT/'offline_validation',{}).prepare(str(OUT/'config.json'),
    str(OUT/'candidate.settings.json'),plan['placement'],None,20.)
print(json.dumps(dict(state='PREPARED_NOT_EXECUTED',folder=str(OUT),request_id=identifier),ensure_ascii=False),flush=True)
