"""OUT135 baseline001完了を待ち、保存1ms/赤前点だけを各1回解析。実機/MCP機能なし。"""
from pathlib import Path
import ast
from datetime import datetime, timezone
import hashlib
import json
import os
import subprocess
import time

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_outr135_20261120_v1'
IDENTIFIER='A_OUTR135_1800_baseline_20261120_001'
RUN=BASE/'baseline01/local_execution'/('run_'+IDENTIFIER)
OUT=BASE/'parallel_analysis/baseline001_early'
PYTHON=Path('D:/venv/venv3-11/Scripts/python.exe')
FRONT=ROOT/'tmp/parallel_front_run_review_20261120.py'
TRACKING=ROOT/'tmp/parallel_outr135_tracking_20261120.py'


def read(path):return json.loads(path.read_text(encoding='utf-8-sig'))


def save(path,value):
    with path.open('x',encoding='utf-8') as stream:json.dump(value,stream,ensure_ascii=False,indent=2,allow_nan=False)


def stamp():return datetime.now(timezone.utc).isoformat()


if OUT.exists():raise FileExistsError('既存watcher出力は上書きしません: '+str(OUT))
for path in (PYTHON,FRONT,TRACKING):
    if not path.is_file():raise FileNotFoundError(path)
for path in (FRONT,TRACKING):ast.parse(path.read_text(encoding='utf-8-sig'))
OUT.mkdir(parents=True)
save(OUT/'watch_started.json',dict(state='WAITING_EXPECTED_COMPLETED_SOURCE',request_id=IDENTIFIER,
    source_run=str(RUN),started_at=stamp(),hardware_operations=0,regular_MCP_analysis_started=False,
    timeout_minutes=60,automatic_capture_retry=False,other_request_substitution=False))
print('WATCHING '+IDENTIFIER+'; source read only; hardware/MCP operations0',flush=True)
deadline=time.monotonic()+3600
while time.monotonic()<deadline:
    path=RUN/'trial.json'
    needed=[RUN/name for name in ('saved_1ms.csv','saved_1ms.settings.json','telemetry.csv','marker_reference_used.json','body_geometry_used.json')]
    if path.is_file() and all(value.is_file() and value.stat().st_size for value in needed):
        try:trial=read(path)
        except (OSError,json.JSONDecodeError):time.sleep(3);continue
        if trial.get('state')=='FAILED':
            save(OUT/'early_completion.json',dict(state='SOURCE_FAILED_NO_RETRY',source_trial=str(path),hardware_operations=0))
            raise SystemExit(2)
        if (trial.get('state')=='CAPTURED_REVIEW_REQUIRED' and trial.get('finished_at')
                and trial.get('saved_log',{}).get('status')=='saved' and Path(trial.get('video_file','')).is_file()):break
    time.sleep(3)
else:
    save(OUT/'early_completion.json',dict(state='WAIT_TIMEOUT_NO_ANALYSIS',source_run=str(RUN),hardware_operations=0,finished_at=stamp()))
    raise SystemExit(3)
settings=read(RUN/'saved_1ms.settings.json')
if settings.get('mouse')!='A' or settings.get('motion_name')!='out_r135' or settings.get('turn_count')!=1:
    raise RuntimeError('固定IDの機体/motion/回数が一致しません。別runへ置換しません')
originals=[RUN/name for name in ('trial.json','saved_1ms.csv','saved_1ms.settings.json','telemetry.csv')]
hashes={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in originals}
save(OUT/'source_ready.json',dict(state='SOURCE_COMPLETE_START_SAVED_ANALYSIS',source_trial=str(RUN/'trial.json'),
    request_id=IDENTIFIER,settings=settings,ready_at=stamp(),source_sha256=hashes,hardware_operations=0))
print('SOURCE COMPLETE; running explicit saved1ms tracking then front-onset once',flush=True)
commands=[('tracking',[str(PYTHON),str(TRACKING),str(RUN)]),
    ('front',[str(PYTHON),str(FRONT),'--run',str(RUN),'--output',str(OUT/'front_early_v1'),
       '--motion','out_r135','--front-onset','--plan',str(BASE/'baseline01/placement_plan.json')])]
codes={};headlines={}
for name,command in commands:
    with (OUT/(name+'_analysis.stdout.txt')).open('x',encoding='utf-8') as stream:
        try:
            completed=subprocess.run(command,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT,timeout=900,
                creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0),env=dict(os.environ,PYTHONUTF8='1',OPENCV_FOR_THREADS_NUM='2'))
            codes[name]=completed.returncode
        except subprocess.TimeoutExpired:codes[name]='TIMEOUT'
    if name=='tracking' and (OUT/'tracking_1ms/verification.json').is_file():
        metrics=read(OUT/'tracking_1ms/verification.json')
        headlines[name]=dict(measured_period_ms=metrics['measured_period_ms'],metrics=metrics['nominal_1ms_tracking_metrics'])
    if name=='front' and (OUT/'front_early_v1/front_reference.json').is_file():
        report=read(OUT/'front_early_v1/front_reference.json')
        headlines[name]={key:report.get(key) for key in ('state','observation_rate','motion_only_observation_rate',
            'entry_window_observation_rate','exit_window_observation_rate','path_direction_change_deg',
            'measured_front_Lend_right_forward_mm','midpoint_nominal_target_right_forward_mm','target_source')}
    print(json.dumps(dict(analysis=name,returncode=codes[name],headline=headlines.get(name)),ensure_ascii=False),flush=True)
unchanged=all(hashlib.sha256(Path(path).read_bytes()).hexdigest()==value for path,value in hashes.items())
save(OUT/'early_completion.json',dict(state='SAVED_ANALYSIS_COMPLETED' if all(value==0 for value in codes.values()) else 'SAVED_ANALYSIS_HAS_REJECTION_OR_ERROR',
    request_id=IDENTIFIER,source_trial=str(RUN/'trial.json'),returncodes=codes,headlines=headlines,
    source_hashes_unchanged=unchanged,hardware_operations=0,regular_MCP_analysis_started=False,
    parameter_adoption_allowed=False,finished_at=stamp()))
print('WATCHER DONE; analysis only; source unchanged='+str(unchanged),flush=True)
