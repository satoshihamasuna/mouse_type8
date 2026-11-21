from pathlib import Path
import json
import os
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'tools/logs/pixel_camera/turn1800_A_outr45_20261120_v1/firmware_out45_lr_adoption_20261121_v1'
command=[sys.executable,'-m','unittest','discover','-s','tools/tests','-p','test_run_param_linkage.py','-v']
with (OUT/'host_test.stdout.txt').open('x',encoding='utf-8') as file:
    result=subprocess.run(command,cwd=ROOT,env=dict(os.environ,PYTHONUTF8='1'),stdout=file,stderr=subprocess.STDOUT)
text=(OUT/'host_test.stdout.txt').read_text(encoding='utf-8')
passed=result.returncode==0 and 'Ran 1 test' in text and 'OK' in text and 'skipped' not in text.lower()
with (OUT/'host_test_result.json').open('x',encoding='utf-8') as file:
    json.dump(dict(command=command,returncode=result.returncode,passed=passed,tests=1,skipped=0 if passed else None),file,indent=2)
print(text,flush=True)
raise SystemExit(0 if passed else 1)
