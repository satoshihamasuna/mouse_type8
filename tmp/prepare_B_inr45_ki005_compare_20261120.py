"""B1600 inR45のIゲインだけを下げる未実行比較候補を作る。"""
from pathlib import Path
import copy
import hashlib
import json
import shutil

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr45_20261120_v1'
PREVIOUS=BASE/'ff_om_accdec0p00075_01'
RUN=PREVIOUS/'local_execution/run_B_INR45_1600_ffaccdec750_green_20261120_003'
OUT=BASE/'ff750_omki0p005_01'
OUT.mkdir(exist_ok=False)
read=lambda p:json.loads(p.read_text(encoding='utf-8'))
def save(p,v):
    with p.open('x',encoding='utf-8') as f:json.dump(v,f,ensure_ascii=False,indent=2,allow_nan=False)
actual=read(RUN/'saved_1ms.settings.json')
candidate=copy.deepcopy(read(PREVIOUS/'candidate.settings.json'))
for key in ('parameters','pid','feedforward','suction','turn_count'):
    candidate[key]=copy.deepcopy(actual[key])
assert actual['pid']['om_ki']==.01
candidate['pid']['om_ki']=.005
candidate['source']='B1600 inR45・角加減速FF0.000750の比較走行で残るピーク超過と後揺れに対し、角速度Iゲインだけを0.010→0.005へ下げる未実行候補。改善は未確認。'
save(OUT/'candidate.settings.json',candidate)
for name in ('config.json','coarse_placement_check.py'):
    shutil.copyfile(PREVIOUS/name,OUT/name)
identifier='B_INR45_1600_ff750_ki005_green_20261120_004'
script=(PREVIOUS/'run_recorded_once.py').read_text(encoding='utf-8')
script=script.replace('B_INR45_1600_ffaccdec750_green_20261120_003',identifier)
(OUT/'run_recorded_once.py').write_text(script,encoding='utf-8')
plan=read(PREVIOUS/'placement_plan.json')
plan.update(request_id=identifier,operator_ready=False,current_placement_confirmed=False,
    execution_commands_sent=0,settings_write_commands_sent=0,
    comparison=dict(changed_field='pid.om_ki',before=.01,after=.005,
        previous_trial=str(RUN/'trial.json'),
        reason='加減速FFで旋回中RMSは減少したがピークと後揺れが残る。Iゲインを半減し、追従・ピーク・後揺れ・出口角を一回比較する。',
        improvement_confirmed=False))
save(OUT/'placement_plan.json',plan)
differences={group:{k:[actual[group][k],v] for k,v in candidate[group].items() if actual[group][k]!=v}
    for group in ('parameters','pid','feedforward','suction')}
differences={k:v for k,v in differences.items() if v}
assert differences=={'pid':{'om_ki':[.01,.005]}}
save(OUT/'offline_validation.json',dict(state='VALIDATED_NOT_EXECUTED',differences=differences,
    turn_count=candidate['turn_count'],operator_ready=False,hardware_commands_sent=0,
    source_hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (
        RUN/'saved_1ms.csv',RUN/'saved_1ms.settings.json',RUN/'tracking_1ms_review_v3/verification.json')}))
print(OUT)
