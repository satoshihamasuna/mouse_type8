"""B左V90の鏡映反映を既存のBビルド・パラメータリンク検証で確認する。実機操作なし。"""
from pathlib import Path
from datetime import datetime,timezone
import argparse,hashlib,json,os,subprocess,sys

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'tools/logs/pixel_camera/turn1600_B_rv90_20261120_v1/left_mirror_source_validation_20261121'
parser=argparse.ArgumentParser();parser.add_argument('action',choices=['configure','build','linkage']);args=parser.parse_args()
if args.action=='configure':OUT.mkdir(exist_ok=False)
assert OUT.is_dir()
cmake=Path('C:/Users/sato1/AppData/Local/stm32cube/bundles/cmake/4.3.1+st.1/bin')
ninja=Path('C:/Users/sato1/AppData/Local/stm32cube/bundles/ninja/1.13.2+st.1/bin')
gcc=Path('C:/Users/sato1/AppData/Local/stm32cube/bundles/gnu-tools-for-stm32/14.3.1+st.2/bin')
commands=dict(configure=[str(cmake/'cmake.exe'),'--preset','Debug-MouseB'],
    build=[str(cmake/'cmake.exe'),'--build','--preset','Debug-MouseB','--parallel','4'],
    linkage=[sys.executable,'-m','unittest','discover','-s','tools/tests','-p','test_run_param_linkage.py','-v'])
paths=[ROOT/'Params/Param_B/turn_1600/turn_1600.cpp',ROOT/'Params/Param_B/turn_1600/turn_1600.h']
hashes=lambda:{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
before=hashes();start=datetime.now(timezone.utc).isoformat();env=dict(os.environ,PYTHONUTF8='1')
env['PATH']=os.pathsep.join([str(cmake),str(ninja),str(gcc),env.get('PATH','')])
command=commands[args.action];print('RUNNING '+args.action,flush=True)
with (OUT/(args.action+'.log')).open('xb') as output:
    result=subprocess.run(command,cwd=ROOT,env=env,stdout=output,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
after=hashes();record=dict(action=args.action,command=command,started_at=start,finished_at=datetime.now(timezone.utc).isoformat(),
    returncode=result.returncode,source_before=before,source_after=after,source_unchanged_during_check=before==after,
    preset='Debug-MouseB' if args.action!='linkage' else None,host_linkage_targets=['A','B'] if args.action=='linkage' else [],
    log=str(OUT/(args.action+'.log')),hardware_operations=0)
with (OUT/(args.action+'.json')).open('x',encoding='utf-8') as f:json.dump(record,f,indent=2)
print(json.dumps(record),flush=True)
raise SystemExit(result.returncode if result.returncode else (0 if before==after else 2))
