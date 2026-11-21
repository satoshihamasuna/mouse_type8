"""固定IDの撮影完了だけを待ち、保存データ解析を各1回だけ行う。実機機能なし。"""
from pathlib import Path
import json
import subprocess
import time

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_outr45_20261120_v1'
RUN=BASE/'baseline01/local_execution/run_A_OUTR45_1800_baseline_20261120_001'
OUTPUT=BASE/'parallel_analysis'
PYTHON=Path('D:/venv/venv3-11/Scripts/python.exe')
config=json.loads((BASE/'baseline01/config.json').read_text(encoding='utf-8'))
expected_reference_path=Path(config['reference'])
expected_reference=json.loads(expected_reference_path.read_text(encoding='utf-8'))
assert expected_reference['reference_id']=='2e156d45a602615f4d49'
assert set(expected_reference['markers_px'])=={'2','3'}
assert expected_reference.get('source_full_search_revision',{}).get('no_generated_vertices') is True
assert 'marker_search_rois_px' not in expected_reference
deadline=time.monotonic()+1800
print('Awaiting completed A_OUTR45_1800_baseline_20261120_001 saved trial; hardware operations0',flush=True)
while time.monotonic()<deadline:
    path=RUN/'trial.json'
    if path.is_file() and (RUN/'saved_1ms.csv').is_file() and (RUN/'saved_1ms.settings.json').is_file():
        try:
            trial=json.loads(path.read_text(encoding='utf-8'))
        except (json.JSONDecodeError,OSError):
            time.sleep(3);continue
        if trial.get('state')=='CAPTURED_REVIEW_REQUIRED' and trial.get('saved_log',{}).get('status')=='saved':
            break
        if trial.get('state')=='FAILED':
            raise RuntimeError('Original trial failed; no automatic capture retry or analysis substitution')
    time.sleep(3)
else:
    raise TimeoutError('No completed expected capture appeared within30min; no analysis executed')
saved_reference=json.loads((RUN/'marker_reference_used.json').read_text(encoding='utf-8'))
if saved_reference!=expected_reference:
    raise RuntimeError('Saved run reference differs from the current original-physical-frame full-search reference; no analysis executed')
print('Completed expected source found; starting saved1ms review once',flush=True)
codes={}
commands=[('tracking',[str(PYTHON),str(ROOT/'tmp/parallel_outr45_tracking_20261120.py'),str(RUN)]),
    ('front',[str(PYTHON),str(ROOT/'tmp/parallel_front_run_review_20261120.py'),
        '--run',str(RUN),'--output',str(OUTPUT/'front_early_v1'),'--motion','out_r45','--front-onset',
        '--midpoint-target','31.819805153394636','222.73863607376245'])]
for name,command in commands:
    with (OUTPUT/(name+'_early.stdout.txt')).open('x',encoding='utf-8') as stream:
        completed=subprocess.run(command,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT,timeout=600,
            creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
    codes[name]=completed.returncode
    print(f'{name} saved analysis returncode={completed.returncode}',flush=True)
with (OUTPUT/'early_completion.json').open('x',encoding='utf-8') as stream:
    json.dump(dict(source_trial=str(RUN/'trial.json'),returncodes=codes,hardware_operations=0,
        regular_MCP_analysis_started=False,source_modified=False,
        expected_reference=str(expected_reference_path),saved_reference_matches=True),stream,ensure_ascii=False,indent=2)
print('Expected source analysed once; no physical execution or MCP request',flush=True)
