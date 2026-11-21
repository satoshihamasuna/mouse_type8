"""旧parallel候補wrapperを保存後、operator-readyで必ず実行前停止する。"""
from pathlib import Path
import hashlib
import json
import subprocess

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_inr135_20261120_v1'
OLD=BASE/'parallel_analysis/inr135_c3p5_angle_candidate_v1'
BACKUP=OLD/'superseded_wrapper_backups_v1'
BACKUP.mkdir(exist_ok=False)
PYTHON='C:/Users/sato1/Documents/git/Debug_camera/.venv/Scripts/python.exe'
results=[]
for number in (1,2,3):
    stage=OLD/f'angle_c3p5_{number:02d}'
    wrapper=stage/'run_recorded_once.py'
    original=wrapper.read_bytes()
    backup=BACKUP/f'angle_c3p5_{number:02d}_run_recorded_once.py'
    with backup.open('xb') as file:file.write(original)
    source=original.decode('utf-8')
    needle='args = parser.parse_args()\nservice ='
    assert source.count(needle)==1
    replacement=("args = parser.parse_args()\n"
        "if args.operator_ready:\n"
        "    raise SystemExit('SUPERSEDED_STAGE: execution disabled; use BASE/angle_c3p5_01..03 with original requested settings')\n"
        "service =")
    updated=source.replace(needle,replacement)
    compile(updated,str(wrapper),'exec')
    with wrapper.open('w',encoding='utf-8',newline='\n') as file:file.write(updated)
    files_before={str(path.relative_to(stage)) for path in stage.rglob('*') if path.is_file()}
    trial=subprocess.run([PYTHON,str(wrapper),'--operator-ready'],capture_output=True,timeout=30,
        creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
    files_after={str(path.relative_to(stage)) for path in stage.rglob('*') if path.is_file()}
    output=(trial.stdout+trial.stderr).decode('utf-8',errors='replace')
    assert trial.returncode != 0 and 'SUPERSEDED_STAGE' in output
    assert files_before==files_after
    results.append(dict(stage=str(stage),source_wrapper_backup=str(backup),
        before_sha256=hashlib.sha256(original).hexdigest(),after_sha256=hashlib.sha256(wrapper.read_bytes()).hexdigest(),
        operator_ready_returncode=trial.returncode,operator_ready_output=output.strip(),
        no_new_execution_files=True,hardware_commands_sent=0))
with (BACKUP/'guard_verification.json').open('x',encoding='utf-8') as file:
    json.dump(dict(results=results,original_wrappers_preserved=True,new_BASE_stages_untouched=True,
        raw_data_untouched=True),file,ensure_ascii=False,indent=2);file.write('\n')
print(json.dumps(results,ensure_ascii=False,indent=2),flush=True)
