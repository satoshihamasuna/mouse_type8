"""補正+1.0°の相対軌跡を、保存済みの数値処理で評価する。実機通信なし。"""
from pathlib import Path
import importlib.util
import runpy
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'tools/analysis')]
relative_run='next_correction_plus1p0_20261120_v1/run_session01/run_B_R90_2000_correction1p0_20261120_001'
run=ROOT/'tools/logs/pixel_camera/turn2000_B_r90_20261120_v1'/relative_run
snapshot=run/'analysis_source_used'
for name in ('pixel_camera_geometry','pixel_camera_reference','marker_detection','body_appearance_model','video_marker_pose'):
    spec=importlib.util.spec_from_file_location(name,snapshot/(name+'.py'))
    module=importlib.util.module_from_spec(spec);sys.modules[name]=module;spec.loader.exec_module(module)
sys.argv=['review_turn2000_B_video_20261120.py','--output',str(run/'geometry_review_v1'),
          '--run','correction1p0='+relative_run]
runpy.run_path(str(ROOT/'tmp/review_turn2000_B_video_20261120.py'),run_name='__main__')
