"""動画で確認したB右V90の入口/出口長候補を準備。実機操作なし。"""
from pathlib import Path
import copy
import json
import shutil
import sys

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT/'tools/logs/pixel_camera/turn1600_B_rv90_20261120_v1'
OLD = BASE/'ff_accdec_0p0006_01'
RUN = OLD/'local_execution/run_B_RV90_1600_ff0p0006_green_20261120_001'
NEW = BASE/'geometry_ff0p0006_Ls3p3_Le18p9_01'
RID = 'B_RV90_1600_geom3p3_18p9_ff0p0006_20261120_001'
read = lambda path: json.loads(path.read_text(encoding='utf-8'))


def save(path, value):
    with path.open('x', encoding='utf-8') as file:
        json.dump(value, file, ensure_ascii=False, indent=2, allow_nan=False)
        file.write('\n')


assert not NEW.exists()
mid = read(RUN/'midpoint_raw_wheel_clock_v1/midpoint_review.json')
assert mid['independent_optical_geometry_eligible']
assert abs(mid['exit_path_direction_change_deg']-90) < .5
assert mid['independent_clock_required_motion_quality']['observation_rate'] >= .95
candidate = copy.deepcopy(read(OLD/'candidate.settings.json'))
assert candidate['parameters']['lstart'] == 2.2 and candidate['parameters']['lend'] == 20.83
candidate['parameters'].update(lstart=3.3, lend=18.9)
candidate['source'] = 'B1600右V90・FF0.000600の実測入口相対中点から長さ候補3.307/18.893mmを求め、測定精度に合わせ3.3/18.9mmで比較。角度補正+1.6度・半径40mm・PID/FF/吸引は維持。'
NEW.mkdir()
save(NEW/'candidate.settings.json',candidate)
for name in ['baseline.settings.json','config.json']:
    shutil.copyfile(OLD/name,NEW/name)
wrapper=(OLD/'run_recorded_once.py').read_text(encoding='utf-8')
assert wrapper.count('B_RV90_1600_ff0p0006_green_20261120_001')==1
wrapper=wrapper.replace('B_RV90_1600_ff0p0006_green_20261120_001',RID)
compile(wrapper,str(NEW/'run_recorded_once.py'),'exec')
(NEW/'run_recorded_once.py').write_text(wrapper,encoding='utf-8',newline='\n')
plan=dict(mouse='B',motion='r_v90',preset_mm_s=1600,request_id=RID,
    changes=dict(lstart=[2.2,3.3],lend=[20.83,18.9]),parameters=candidate['parameters'],
    pid=candidate['pid'],feedforward=candidate['feedforward'],suction=candidate['suction'],
    source_trial=str(RUN/'trial.json'),source_review=str(RUN/'midpoint_raw_wheel_clock_v1/midpoint_review.json'),
    relative_exit_deg=mid['exit_path_direction_change_deg'],original_length_solution=mid['length_only_candidate_at_current_angle'],
    placement=read(OLD/'comparison_plan.json')['placement'],post_diagonal_accel_m_s2=25.,
    operator_ready=False,execution_commands_sent=0,power_state='BOTH_OFF_USER_CONFIRMED',hardware_operations=0,
    source_params_changed=False,firmware_write_performed=False,
    scope='実測入口相対の前後マーカー中点。盤面固定の設置/出口/停止点、機軸・車軸・BのEKF校正とは区別。',
    acceptance='同じ品質条件の動画でLend相対位置/出口角度と末尾を確認。まず新しい形状を1回測定し、その後同一値の独立3本で再現性を確認する。',
    retained_limitations=['FF0.000600で旋回中RMSは改善したが、波形後のRMSは悪化。末尾も比較する。',
        '元summaryのauto_stop_confirmed falseは保持。全telemetry/後半静止/操作者停止を別に検査。',
        '停止点の約6〜8mmの余走はLendの長さ解へ埋め込まない。後段減速と盤面固定の停止は別評価。'])
save(NEW/'comparison_plan.json',plan)
sys.path.insert(0,str(ROOT/'tools'))
from pixel_camera_tuning_mcp import McpTuningService
payload=McpTuningService(NEW/'offline_validation',{}).prepare(str(NEW/'config.json'),str(NEW/'candidate.settings.json'),plan['placement'],25.,None)
save(NEW/'offline_validation.json',dict(state='VALIDATED_NOT_EXECUTED',payload=payload,hardware_operations=0))
state=read(BASE/'resume_state.json')
state.update(state='FF_COMPARISON_REVIEWED_GEOMETRY_STEP_READY_BOTH_OFF',next_stage=str(NEW),next_request_id=RID,
    next_request_sent=False,execution_pending=False,operator_ready=False,power_state='BOTH_OFF_USER_CONFIRMED')
(BASE/'resume_state.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(state='VALIDATED_NOT_EXECUTED',folder=str(NEW),changes=plan['changes'],hardware_operations=0),ensure_ascii=False))
