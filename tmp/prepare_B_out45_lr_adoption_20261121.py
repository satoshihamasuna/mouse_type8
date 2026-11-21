"""左右out45の反映前資料を退避し、既存Bビルド手順を今回の保存先へ用意。実機操作なし。"""
from pathlib import Path
from datetime import datetime,timezone
import hashlib,json,shutil
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'tools/turn_analysis/20261121_B_turn1600_OUT45_LR_adoption'
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_outr45_20261121_v1';OUT.mkdir(exist_ok=False)
read=lambda p:json.loads(Path(p).read_text(encoding='utf-8-sig'))
selected=read(BASE/'selected_m0p9_three_review_v1/selected.settings.json');decision=read(BASE/'selected_m0p9_three_review_v1/decision.json')
assert selected['mouse']=='B' and selected['motion_name']=='out_r45' and selected['parameters']['correction']==-.9
assert decision['rows'][-1]['relative_geometry_eligible'] and not decision['rows'][1]['relative_geometry_eligible']
paths=['Params/Param_B/turn_1600/turn_1600.cpp','Params/Param_B/turn_1600/turn_1600.h','README.md',
 'Params/Param_A/turn_1600/turn_1600.cpp']
for rel in paths:
 target=OUT/'before'/rel;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/rel,target)
for name in ('selected.settings.json','decision.json','verification.json'):
 shutil.copyfile(BASE/'selected_m0p9_three_review_v1'/name,OUT/('tuning_'+name))
source=(ROOT/'tmp/build_B_out45_ff_adoption_20261121.py').read_text(encoding='utf-8-sig')
source=source.replace('20261121_B_turn1600_OUT45_FF_adoption',OUT.name)
compile(source,'build_B_out45_lr_adoption_20261121.py','exec')
with (ROOT/'tmp/build_B_out45_lr_adoption_20261121.py').open('x',encoding='utf-8') as f:f.write(source)
with (OUT/'preparation.json').open('x',encoding='utf-8') as f:
 json.dump(dict(prepared_at_utc=datetime.now(timezone.utc).isoformat(),mouse='B',user_request='右と左にはんえいして',
  right_parameters=selected['parameters'],left_mirror=dict(velo=1.6,r_min=60.,lstart=21.8,lend=16.5,degree=45.,correction=.9),
  source_decision=str(BASE/'selected_m0p9_three_review_v1/decision.json'),left_hardware_tested=False,
  firmware_write_performed=False,robot_commands_sent=0,
  before_sha256={rel:hashlib.sha256((ROOT/rel).read_bytes()).hexdigest() for rel in paths}),f,ensure_ascii=False,indent=2)
print('B out45 right -0.9 / left +0.9 prepared; preexisting differences and tuning evidence preserved')
