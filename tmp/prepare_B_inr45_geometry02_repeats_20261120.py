"""B1600 inR45の形状候補を変更せず、独立反復2本を準備する。"""
from pathlib import Path
import copy
import hashlib
import json
import shutil

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr45_20261120_v1'
SOURCE=BASE/'geometry02'
RUN=SOURCE/'local_execution/run_B_INR45_1600_geometry02_green_20261120_006'
read=lambda p:json.loads(p.read_text(encoding='utf-8'))
def save(p,v):
    with p.open('x',encoding='utf-8') as f:json.dump(v,f,ensure_ascii=False,indent=2,allow_nan=False)
actual=read(RUN/'saved_1ms.settings.json')
geom=read(RUN/'midpoint_raw_wheel_clock_v3/midpoint_review.json')
tracking=read(RUN/'tracking_1ms_review_v3/verification.json')
assert geom['independent_optical_geometry_eligible']
assert read(RUN/'operator_after_run.json')['both_powered_off']
stages=[]
for number,serial in ((1,7),(2,8)):
    OUT=BASE/f'geometry02_repeat_{number:02d}';OUT.mkdir(exist_ok=False)
    candidate=copy.deepcopy(read(SOURCE/'candidate.settings.json'))
    for key in ('parameters','pid','feedforward','suction','turn_count'):
        candidate[key]=copy.deepcopy(actual[key])
    candidate['source']='B1600 inR45 7.8/34.5/-1.5度、角加減速FF0.000750・角速度Ki0.005の同一条件反復。基準006から設定変更なし。'
    save(OUT/'candidate.settings.json',candidate)
    for name in ('config.json','coarse_placement_check.py'):
        shutil.copyfile(SOURCE/name,OUT/name)
    identifier=f'B_INR45_1600_geometry02_repeat{number:02d}_green_20261120_{serial:03d}'
    script=(SOURCE/'run_recorded_once.py').read_text(encoding='utf-8').replace('B_INR45_1600_geometry02_green_20261120_006',identifier)
    (OUT/'run_recorded_once.py').write_text(script,encoding='utf-8')
    plan=read(SOURCE/'placement_plan.json')
    plan.update(request_id=identifier,operator_ready=False,current_placement_confirmed=False,
        execution_commands_sent=0,settings_write_commands_sent=0,
        comparison=dict(phase='same_candidate_repeatability',changed_fields=[],source_trial=str(RUN/'trial.json')))
    save(OUT/'placement_plan.json',plan)
    assert all(candidate[key]==actual[key] for key in ('parameters','pid','feedforward','suction','turn_count'))
    save(OUT/'offline_validation.json',dict(state='VALIDATED_NOT_EXECUTED',changed_fields=[],
        expected_same_as_trial=str(RUN/'trial.json'),turn_count=1,operator_ready=False,hardware_commands_sent=0,
        source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (
            RUN/'saved_1ms.settings.json',RUN/'midpoint_raw_wheel_clock_v3/midpoint_review.json')}))
    stages.append(dict(stage=str(OUT),request_id=identifier,request_sent=False))
review=BASE/'geometry02_single_review_v1';review.mkdir(exist_ok=False)
save(review/'review.json',dict(state='SINGLE_GEOMETRY_CANDIDATE_REVIEWED_REPEATS_PENDING',
    mouse='B',motion='in_r45',preset_mm_s=1600,settings=actual,source_trial=str(RUN/'trial.json'),
    geometry=geom,tracking=tracking,operator_result=read(RUN/'operator_after_run.json'),
    original_summary=read(RUN/'full/summary.json'),
    same_candidate_runs=1,repeat_goal=3,remaining_repeats=2,next_stages=stages,
    physical_power_state='BOTH_OFF_USER_CONFIRMED',source_params_changed=False,firmware_written=False,
    decision='Lendの相対位置差は0.885mmまで減少。出口角46.204度は目標より1.204度大きい。同一条件を反復し、単発のずれへ追加補正を重ねる前に再現性を確認する。',
    limits=['入口相対の前後マーカー中点。盤面固定の設置/停止・真の車軸/yaw・連続接続の合格ではない。',
        '元summaryのauto_stop=falseを保持。撮影制御の静止1.413秒、後半動画、操作者の停止/OFFを別に示す。',
        'BのEKF係数は未同定。ファーム/Paramsへ未反映。']))
(review/'README.md').write_text('# 機体B turn1600 右in45：形状候補の単発結果\n\n'
    'Lstart7.8 mm・Lend34.5 mm・補正−1.5度、半径−55 mm、吸引650、角加減速FF0.000750、角速度Ki0.005。\n\n'
    '動画の入口相対出口角46.204度。Lend中点は右44.359/前180.611 mm（目標45/180）で、差は右−0.641/前+0.611 mm、距離0.885 mm。'
    '角度が1.204度大きいため採用確定せず、同じ条件をあと2本反復してから判断する。\n\n'
    '全フレームの必要動作区間は前後観測約98.6%・毎コマ実ArUco登録100%。入口raw wheelだけで時刻を合わせ、時計fit RMS0.384mm・半窓差0.755ms。'
    '保存1msログ190点/drop0、角速度RMS4.926rad/s、出力飽和0%、最低電圧7.391V。'
    '撮影制御の1.413秒静止・後半静止動画・操作者の両方OFF/接触なしを記録。元summaryのauto_stop=falseは保持。\n\n'
    '反復はgeometry02_repeat_01、geometry02_repeat_02。各回で元225/45mm・前緑下向きへ概略配置し、干渉なし・両方ON・手離しを新しく確認する。'
    '同じ実行IDは再送しない。ファーム/Params変更なし。BのEKF同定・左・連続接続は未確認。\n',encoding='utf-8')
state=read(BASE/'resume_state.json')
state.update(state='GEOMETRY02_SINGLE_CANDIDATE_REPEATS_PENDING',stage=str(SOURCE),
    request_id='B_INR45_1600_geometry02_green_20261120_006',request_sent=True,operator_ready=False,
    physical_power_state='BOTH_OFF_USER_CONFIRMED',execution_pending=False,execution_commands_sent=4,
    next_stage=stages[0]['stage'],next_settings=str(Path(stages[0]['stage'])/'candidate.settings.json'),
    next_request_id=stages[0]['request_id'],next_request_sent=False,next_operator_ready=False,
    same_candidate_runs=1,remaining_same_candidate_repeats=2,repeat_stages=stages,
    latest_aggregate_review=str(review/'review.json'),
    next_action='設定を変えず独立反復2本。毎回元225/45mm・前緑下向きへ概略配置し、両方ON・手離しを確認後に一回だけ取得。')
(BASE/'resume_state.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
with (BASE/'README.md').open('a',encoding='utf-8') as f:f.write('\n形状006を一回取得。入口相対出口角46.204度、Lend差右−0.641/前+0.611mm（距離0.885mm）。同一候補をあと2本反復して角度・位置の再現性を確認する。詳細geometry02_single_review_v1/README.md。両方OFF・接触なし確認。ファーム/Params変更なし。\n')
print(json.dumps(stages,ensure_ascii=False))
