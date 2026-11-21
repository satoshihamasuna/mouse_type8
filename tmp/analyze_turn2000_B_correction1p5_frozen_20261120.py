"""取得時に保存した検出ソースを用い、補正+1.5°の保存動画を解析する。実機通信なし。"""
from pathlib import Path
import hashlib
import importlib.util
import json
import shutil
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'tools'),str(ROOT/'tools/analysis')]
PREP = ROOT/'tools/logs/pixel_camera/turn2000_B_r90_20261120_v1/next_correction_plus1p5_20261120_v1'
RUN = PREP/'run_session01/run_B_R90_2000_correction1p5_20261120_001'
SNAP = RUN/'analysis_source_used'
SNAP.mkdir(exist_ok=False)

# 取得前に保存済みの検出3モジュールを使う。関連する読み取り・数値処理もここで固定する。
sources = {}
for name in ('video_marker_pose.py','body_appearance_model.py','marker_detection.py'):
    source = PREP/'analysis_source_snapshot'/name
    shutil.copyfile(source,SNAP/name)
    sources[name] = dict(source=str(source),sha256=hashlib.sha256((SNAP/name).read_bytes()).hexdigest())
for name in ('pixel_camera_geometry','pixel_camera_reference','pixel_camera_detection',
             'pixel_camera_capture','pixel_camera_turn_capture','pixel_camera_tuning',
             'myshell_debug','pixel_camera_profiles','log_protocol'):
    source = ROOT/'tools'/(name+'.py')
    shutil.copyfile(source,SNAP/source.name)
    sources[source.name] = dict(source=str(source),sha256=hashlib.sha256((SNAP/source.name).read_bytes()).hexdigest())
for name in ('pixel_camera_relative_pose','pixel_camera_tuning_report'):
    source = ROOT/'tools/analysis'/(name+'.py')
    shutil.copyfile(source,SNAP/source.name)
    sources[source.name] = dict(source=str(source),sha256=hashlib.sha256((SNAP/source.name).read_bytes()).hexdigest())
(SNAP/'manifest.json').write_text(json.dumps(sources,indent=2)+'\n',encoding='utf-8')


def frozen(name):
    spec = importlib.util.spec_from_file_location(name,SNAP/(name+'.py'))
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


for name in ('pixel_camera_geometry','pixel_camera_reference','marker_detection',
             'body_appearance_model','video_marker_pose','pixel_camera_detection'):
    frozen(name)
# summarize()から使う停止判定とCSV読出しも保存済みコードへ固定する。
for name in ('pixel_camera_turn_capture','pixel_camera_tuning'):
    frozen(name)
relative = frozen('pixel_camera_relative_pose')
report = frozen('pixel_camera_tuning_report')
trial = json.loads((RUN/'trial.json').read_text(encoding='utf-8'))
start,end = report.analysis_video_window(trial)
sys.argv = [str(SNAP/'pixel_camera_relative_pose.py'),'--trial',str(RUN/'trial.json'),
    '--marker-reference',str(RUN/'marker_reference_used.json'),'--body-geometry',str(RUN/'body_geometry_used.json'),
    '--camera-height-mm','1500','--height-source','approximate_for_capture','--fixed-camera','--captured-trial',
    '--video-start',str(start),'--video-end',str(end),'--frame-stride','1','--output',str(RUN/'full')]
relative.main()
print(json.dumps(report.summarize(RUN/'full'),ensure_ascii=True),flush=True)
