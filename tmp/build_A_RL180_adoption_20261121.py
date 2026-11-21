"""既存Debug-MouseAの導入済みツールでConfigure→Buildし、結果を保存する。"""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json
import os
import re
import subprocess

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'tools/logs/pixel_camera/turn1800_A_r180_20261120_v1/firmware_RL180_adoption_20261121_v1'
cache=(ROOT/'build/Debug-MouseA/CMakeCache.txt').read_text(encoding='utf-8')
def cached(name):
    return Path(re.search(r'^'+re.escape(name)+r':[^=]+=(.+)$',cache,re.M)[1].strip())
cmake=cached('CMAKE_COMMAND')
ninja=cached('CMAKE_MAKE_PROGRAM')
compiler=cached('CMAKE_CXX_COMPILER_AR').parent/'arm-none-eabi-g++.exe'
assert all(p.is_file() for p in (cmake,ninja,compiler))
env=os.environ.copy()
env['PATH']=';'.join([str(compiler.parent),str(ninja.parent),str(cmake.parent),env.get('PATH','')])
versions={str(p):subprocess.check_output([str(p),'--version'],env=env,text=True).splitlines()[0] for p in (cmake,ninja,compiler)}
source=ROOT/'Params/Param_A/turn_1800/turn_1800.cpp'
source_sha=hashlib.sha256(source.read_bytes()).hexdigest()
result=dict(started_at=datetime.now(timezone.utc).isoformat(),preset='Debug-MouseA',
    tools=versions,wrapper_unavailable='cube-cmake is not on PATH; using the same installed CMake/Ninja/Arm toolchain.',
    source_sha256_before=source_sha,commands=[])
for name,command in [('configure',[str(cmake),'--preset','Debug-MouseA','-DCMAKE_MAKE_PROGRAM='+ninja.as_posix()]),
                     ('build',[str(cmake),'--build','--preset','Debug-MouseA','--parallel','4'])]:
    print('Running '+name+' for Debug-MouseA',flush=True)
    with (OUT/(name+'.stdout.txt')).open('x',encoding='utf-8') as log:
        completed=subprocess.run(command,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,
            creationflags=subprocess.CREATE_NO_WINDOW)
    result['commands'].append(dict(step=name,command=command,returncode=completed.returncode))
    print(name+' exit='+str(completed.returncode),flush=True)
    if completed.returncode:break
result['source_sha256_after']=hashlib.sha256(source.read_bytes()).hexdigest()
result['source_unchanged_during_build']=result['source_sha256_after']==source_sha
result['passed']=len(result['commands'])==2 and all(r['returncode']==0 for r in result['commands']) and result['source_unchanged_during_build']
result['finished_at']=datetime.now(timezone.utc).isoformat()
if result['passed']:
    result['elf_sha256']=hashlib.sha256((ROOT/'build/Debug-MouseA/mouse_type8.elf').read_bytes()).hexdigest()
with (OUT/'build_result.json').open('x',encoding='utf-8') as f:json.dump(result,f,indent=2);f.write('\n')
print(json.dumps(result,indent=2),flush=True)
raise SystemExit(0 if result['passed'] else 1)
