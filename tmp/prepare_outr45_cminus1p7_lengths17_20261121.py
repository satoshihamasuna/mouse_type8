from pathlib import Path
from datetime import datetime,timezone
import json
import math
import os
import shutil
import subprocess

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_outr45_20261120_v1'
STAGE=BASE/'repeat_cminus1p7_Ls15p4_Le15p9_20261121_02'
OLD_ID='A_OUTR45_1800_cminus1p7_Ls15p4_Le15p9_20261121_002'
NEXT=BASE/'lengths_cminus1p7_Ls17p0_Le17p4_20261121_01'
NEXT_ID='A_OUTR45_1800_cminus1p7_Ls17p0_Le17p4_20261121_001'
def read(path):return json.loads(path.read_text(encoding='utf-8'))
def save(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('x',encoding='utf-8') as file:json.dump(value,file,ensure_ascii=False,indent=2);file.write('\n')
rows=[]
for folder in (BASE/'lengths_cminus1p7_Ls15p4_Le15p9_20261121_01',STAGE):
    review=read(folder/'midpoint_brake_clock_v1/midpoint_review.json')
    assert review['independent_optical_geometry_eligible']
    rows.append(dict(stage=folder.name,angle_deg=review['exit_path_direction_change_deg'],
        residual_mm=review['midpoint_Lend_residual_mm'],error_norm_mm=math.hypot(*review['midpoint_Lend_residual_mm']),
        projection=review['length_only_candidate_at_current_angle']))
ls=sum(row['projection']['request_Lstart_mm'] for row in rows)/2
le=sum(row['projection']['request_Lend_mm'] for row in rows)/2
assert round(ls,1)==17.0 and round(le,1)==17.4
save(STAGE/'same_setting_repeat_comparison.json',dict(rows=rows,mean_angle_deg=sum(row['angle_deg'] for row in rows)/2,
    mean_projected_lstart_mm=ls,mean_projected_lend_mm=le,parameter_adoption_allowed=False))
NEXT.mkdir(exist_ok=False)
settings=read(STAGE/'candidate.settings.json')
settings['source']='Unexecuted two-run mean length projection from c-1.7deg 15.4/15.9mm: video45.430/44.708deg, residuals(-0.967,-3.563)/(-1.105,-1.689)mm.'
settings['parameters'].update(lstart=17.0,lend=17.4)
save(NEXT/'candidate.settings.json',settings)
save(NEXT/'candidate_basis.json',dict(source_comparison=str(STAGE/'same_setting_repeat_comparison.json'),
    mean_projected_lstart_mm=ls,mean_projected_lend_mm=le,chosen_lstart_mm=17.0,chosen_lend_mm=17.4,
    fixed_correction_deg=-1.7,executed=False,parameter_adoption_allowed=False,
    assumptions=['Same measured turn shape; entry/exit straight translations are unverified after changing lengths.',
        'Two measurements do not validate continuous/left/board-fixed accuracy.']))
for name in ('config.json','static_pose_after_settle.py'):shutil.copy2(STAGE/name,NEXT/name)
wrapper=(STAGE/'run_recorded_once.py').read_text(encoding='utf-8');assert wrapper.count(OLD_ID)==1
with (NEXT/'run_recorded_once.py').open('x',encoding='utf-8') as file:file.write(wrapper.replace(OLD_ID,NEXT_ID))
validation=subprocess.run([str(ROOT/'../../Debug_camera/.venv/Scripts/python.exe'),str(NEXT/'run_recorded_once.py')],
    cwd=ROOT,env=dict(os.environ,PYTHONUTF8='1'),capture_output=True,text=True,encoding='utf-8')
save(NEXT/'input_validation.json',dict(returncode=validation.returncode,stdout=validation.stdout,stderr=validation.stderr,
    robot_commands_sent=False));assert validation.returncode==0,validation.stderr
print(json.dumps(dict(lstart=ls,lend=le,next_validation=validation.returncode,robot_commands_sent=False)),flush=True)
