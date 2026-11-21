"""既存のBプリセットを導入済みSTM32ツールでConfigure/Build。書込み・実機接続なし。"""
from pathlib import Path
import json
import os
import re
import subprocess
import time
ROOT=Path(__file__).resolve().parents[1]
OUT=Path('C:\\Users\\sato1\\Documents\\git\\mouse_type8_VScode\\mouse_type8\\tools\\logs\\pixel_camera\\turn1600_B_outr135_20261121_resume_v1\\firmware_OUT135_adoption_20261121_v1')
CACHE=ROOT/'build/Debug-MouseB/CMakeCache.txt'


def entry(name):
    result=re.search(r'^'+re.escape(name)+r':[^=]*=(.*)$',CACHE.read_text(encoding='utf-8'),re.M)
    if result is None:raise ValueError(name)
    return result[1]


cmake=Path(entry('CMAKE_COMMAND'));ninja=Path(entry('CMAKE_MAKE_PROGRAM'));toolchain=Path(entry('CMAKE_NM')).parent
assert cmake.is_file() and ninja.is_file() and (toolchain/'arm-none-eabi-gcc.exe').is_file()
assert entry('MOUSE_TARGET')=='B'
environment=dict(os.environ)
environment['PATH']=os.pathsep.join([str(cmake.parent),str(ninja.parent),str(toolchain),environment.get('PATH','')])
results={}
for stage,command in [('configure',[str(cmake),'--preset','Debug-MouseB']),
    ('build',[str(cmake),'--build','--preset','Debug-MouseB','--parallel','4'])]:
    start=time.monotonic()
    with (OUT/(stage+'.stdout.txt')).open('x',encoding='utf-8') as output:
        process=subprocess.run(command,cwd=ROOT,env=environment,stdout=output,stderr=subprocess.STDOUT,
            creationflags=subprocess.CREATE_NO_WINDOW)
    results[stage]=dict(command=command,returncode=process.returncode,seconds=time.monotonic()-start)
    with (OUT/'build_status.json').open('w',encoding='utf-8') as output:json.dump(results,output,ensure_ascii=False,indent=2)
    print(stage+': '+('PASS' if process.returncode==0 else 'FAIL'),flush=True)
    if process.returncode:raise SystemExit(process.returncode)
    assert entry('MOUSE_TARGET')=='B' and entry('CMAKE_BUILD_TYPE')=='Debug'
print('Debug-MouseB build completed; no firmware write',flush=True)
