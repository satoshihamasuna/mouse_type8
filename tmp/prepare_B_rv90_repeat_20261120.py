"""B1600右V90の同一候補を独立試行用に複製する。実機操作なし。"""
from pathlib import Path
import argparse
import json
import re
import shutil
import sys

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT/'tools/logs/pixel_camera/turn1600_B_rv90_20261120_v1'
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('number',type=int,choices=range(2,10))
parser.add_argument('--source',required=True)
args=parser.parse_args()
assert re.fullmatch(r'geometry_[A-Za-z0-9_]+_[0-9]{2}',args.source)
OLD=BASE/args.source
NEW=BASE/f'{args.source[:-3]}_{args.number:02d}'
read=lambda p:json.loads(p.read_text(encoding='utf-8'))
assert read(OLD/'config.json')['address']=='E1:7F:86:21:84:73', 'Current B bridge address is required'
ORIGINAL=read(OLD/'comparison_plan.json')['request_id']
RID=ORIGINAL.rsplit('_',1)[0]+f'_{args.number:03d}'


def save(path,value):
    with path.open('x',encoding='utf-8') as f:json.dump(value,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')


assert not NEW.exists()
NEW.mkdir()
for name in ['candidate.settings.json','baseline.settings.json','config.json']:
    shutil.copyfile(OLD/name,NEW/name)
for name in ['static_pose_after_settle.py','static_5s_revision.json']:
    if (OLD/name).exists():
        shutil.copyfile(OLD/name,NEW/name)
text=(OLD/'run_recorded_once.py').read_text(encoding='utf-8')
original=ORIGINAL
assert text.count(original)==1
text=text.replace(original,RID)
compile(text,str(NEW/'run_recorded_once.py'),'exec')
(NEW/'run_recorded_once.py').write_text(text,encoding='utf-8',newline='\n')
plan=read(OLD/'comparison_plan.json')
plan.update(request_id=RID,independent_attempt=args.number,changes={},settings_identical_to=str(OLD/'candidate.settings.json'),
    operator_ready=False,execution_commands_sent=0,hardware_operations=0,power_state='BOTH_OFF_USER_CONFIRMED',
    reason='同じ候補の新規測定。入口/出口直線RMS0.75mm以下などの品質条件を変更せず、品質不合格の試行を成功本数に含めない。')
save(NEW/'comparison_plan.json',plan)
sys.path.insert(0,str(ROOT/'tools'))
from pixel_camera_tuning_mcp import McpTuningService
payload=McpTuningService(NEW/'offline_validation',{}).prepare(str(NEW/'config.json'),str(NEW/'candidate.settings.json'),plan['placement'],25.,None)
save(NEW/'offline_validation.json',dict(state='VALIDATED_NOT_EXECUTED',payload=payload,hardware_operations=0))
state=read(BASE/'resume_state.json')
state.update(state='IDENTICAL_GEOMETRY_REPEAT_READY_BOTH_OFF',next_stage=str(NEW),next_request_id=RID,
    next_request_sent=False,execution_pending=False,operator_ready=False,power_state='BOTH_OFF_USER_CONFIRMED')
(BASE/'resume_state.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(state='VALIDATED_NOT_EXECUTED',folder=str(NEW),request_id=RID,settings_identical=True,hardware_operations=0),ensure_ascii=False))
