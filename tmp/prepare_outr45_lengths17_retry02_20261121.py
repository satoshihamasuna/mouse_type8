from pathlib import Path
import json
import os
import shutil
import subprocess

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_outr45_20261120_v1'
OLD=BASE/'lengths_cminus1p7_Ls17p0_Le17p4_20261121_01'
NEW=BASE/'retry_lengths_cminus1p7_Ls17p0_Le17p4_20261121_02'
OLD_ID='A_OUTR45_1800_cminus1p7_Ls17p0_Le17p4_20261121_001'
NEW_ID='A_OUTR45_1800_cminus1p7_Ls17p0_Le17p4_20261121_002'
def save(path,value):
    with path.open('x',encoding='utf-8') as file:json.dump(value,file,ensure_ascii=False,indent=2);file.write('\n')
NEW.mkdir(exist_ok=False)
for name in ('candidate.settings.json','candidate_basis.json','config.json','static_pose_after_settle.py'):
    shutil.copy2(OLD/name,NEW/name)
save(NEW/'retry_basis.json',dict(previous_request_id=OLD_ID,previous_execution_write_attempted=False,
    previous_error='Camera BUSY: B capture began after preflight. No A motion.',settings_changed=False,
    user_instruction='B側の録画走行を停止し、Aを続ける',new_request_id=NEW_ID,
    requires_fresh_operator_ready=True,automatic_execution_replay_allowed=False))
wrapper=(OLD/'run_recorded_once.py').read_text(encoding='utf-8');assert wrapper.count(OLD_ID)==1
with (NEW/'run_recorded_once.py').open('x',encoding='utf-8') as file:file.write(wrapper.replace(OLD_ID,NEW_ID))
for name in ('ready','analyze','confirm_off'):
    source=ROOT/f'tmp/{name}_outr45_cminus1p7_lengths17_20261121.py'
    target=ROOT/f'tmp/{name}_outr45_cminus1p7_lengths17_retry02_20261121.py'
    text=source.read_text(encoding='utf-8').replace(OLD.name,NEW.name).replace(OLD_ID,NEW_ID)
    text=text.replace('review_outr45_cminus1p7_lengths17_tracking_20261121.py','review_outr45_cminus1p7_lengths17_retry02_tracking_20261121.py')
    with target.open('x',encoding='utf-8') as file:file.write(text)
result=subprocess.run([str(ROOT/'../../Debug_camera/.venv/Scripts/python.exe'),str(NEW/'run_recorded_once.py')],
    cwd=ROOT,env=dict(os.environ,PYTHONUTF8='1'),capture_output=True,text=True,encoding='utf-8')
save(NEW/'input_validation.json',dict(returncode=result.returncode,stdout=result.stdout,stderr=result.stderr,robot_commands_sent=False))
assert result.returncode==0,result.stderr
print('New request prepared and input-validated; no camera/BLE/run commands sent.',flush=True)
