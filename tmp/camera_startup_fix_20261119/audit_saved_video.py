from pathlib import Path
import importlib.util
import json
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools'))
from analysis import maze_video_pose as current
from analysis.maze_wall_detection import detect_video_walls

folder=Path(__file__).resolve().parent
baseline=folder/'maze_video_pose_before.py'
baseline.write_bytes(subprocess.check_output(['git','show','HEAD:tools/analysis/maze_video_pose.py'],cwd=ROOT))
spec=importlib.util.spec_from_file_location('maze_pose_before',baseline)
before=importlib.util.module_from_spec(spec);spec.loader.exec_module(before)
trial_path=ROOT/'tools/logs/20261118_230857_686674_ble_explore/camera_20261118_230907_881394/trial.json'
trial=json.loads(trial_path.read_text(encoding='utf-8'))
reference=json.loads(Path(trial['position_analysis']['reference']).read_text(encoding='utf-8'))
result=dict(trial=str(trial_path),video=trial['video_file'],mouse='A',stride=4)
for label,module in (('before',before),('after',current)):
    start=time.perf_counter()
    rows=module.track(trial['video_file'],reference,stride=4,cell_mm=90.,mouse='A',progress=False)
    (folder/(label+'_rows.json')).write_text(json.dumps(rows,ensure_ascii=False),encoding='utf-8')
    result[label]=dict(seconds=time.perf_counter()-start,frames=len(rows),valid=sum(r['pose_valid'] for r in rows),
                       calibration=sum(r['calibration_valid'] for r in rows))
    print(label,result[label],flush=True)
walls,metadata=detect_video_walls(trial['video_file'],reference,current.registration,current.detector)
result['walls']=metadata
old={(w['axis'],w['grid_line'],w['cell_segment']):w['state'] for w in reference['wall_segments']}
result['wall_changes']=[dict(w,previous=old.get((w['axis'],w['grid_line'],w['cell_segment'])))
                        for w in walls if w['state']!=old.get((w['axis'],w['grid_line'],w['cell_segment']))]
(folder/'saved_video_audit.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
(folder/'video_wall_candidates.json').write_text(json.dumps(dict(wall_segments=walls,detection=metadata),ensure_ascii=False,indent=2),encoding='utf-8')
print('walls',metadata['counts'],'changed',len(result['wall_changes']),flush=True)
