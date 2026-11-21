"""B180度の検出条件を既存4本の静止動画と既取得走行で検証。実機操作なし。"""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor,as_completed
import argparse
import copy
import json
import os
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_r180_20261120_v1'
OUT=BASE/'green_s35_validation_v3'
RUN=BASE/'baseline01/local_execution/run_B_R180_1600_baseline_green_20261120_001'
FAILED=BASE/'angle_c0p5_01/local_execution/prepare_B_R180_1600_c0p5_green_20261120_001'
read=lambda p:json.loads(Path(p).read_text(encoding='utf-8-sig'))
ENV=dict(os.environ,PYTHONUTF8='1',OPENCV_FOR_THREADS_NUM='4')
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('action',choices=('prepare','static','moving'))
args=parser.parse_args()


def save(path,value):
    with path.open('x',encoding='utf-8') as output:
        json.dump(value,output,ensure_ascii=False,indent=2,allow_nan=False)


if args.action=='prepare':
    OUT.mkdir(exist_ok=False)
    geometry=read(BASE/'body_geometry_B_green_front_s45_v2.json')
    geometry['marker_detection']['green']['minimum_saturation']=35
    geometry['marker_detection_source']='未採用候補S35: S25は車体反射と連結、S45は弱い緑の一部を切る。実マスクの視認と固定17点の診断に基づきS35で分離。静止4本と既取得全走行を同じ採否条件で別々に検証する。Hue/明度/面積/開演算/赤は維持。'
    save(OUT/'body_geometry_B_green_front_s35_v3.json',geometry)
    cases={
        'original_off01':BASE/'off_static01/plan.json',
        'independent_off02':BASE/'off_static02_s45/plan.json',
        'baseline_powered':BASE/'baseline01/local_execution/prepare_B_R180_1600_baseline_green_20261120_001/static_plan.json',
        'failed_c0p5_powered':FAILED/'static_plan.json',
    }
    for name,source in cases.items():
        folder=OUT/name;folder.mkdir()
        plan=copy.deepcopy(read(source));plan['geometry']=str(OUT/'body_geometry_B_green_front_s35_v3.json')
        save(folder/'plan.json',plan)
    save(OUT/'basis.json',dict(state='OFFLINE_CANDIDATE_VALIDATION_PENDING',
        cases={k:str(v) for k,v in cases.items()},acceptance_limits_changed=False,
        original_profiles_and_failed_results_preserved=True,hardware_commands_sent=0))
    print(OUT,flush=True)
elif args.action=='static':
    def inspect(folder):
        with (folder/'stdout.txt').open('x',encoding='utf-8') as log:
            subprocess.run([sys.executable,str(BASE/'baseline01/static_pose_after_settle.py'),'measure',
                str(folder/'plan.json'),str(folder/'static_check.json')],check=True,timeout=180,
                stdout=log,stderr=subprocess.STDOUT,env=ENV,creationflags=subprocess.CREATE_NO_WINDOW)
        result=read(folder/'static_check.json')
        return dict(case=folder.name,**{k:result.get(k) for k in ('passed','observed','frames',
            'center_board_mm','heading_deg','heading_jitter_rms_deg','failure_reasons')})
    results=[]
    with ThreadPoolExecutor(max_workers=2) as executor:
        futures=[executor.submit(inspect,OUT/name) for name in
            ('original_off01','independent_off02','baseline_powered','failed_c0p5_powered')]
        for future in as_completed(futures):
            result=future.result();results.append(result);print(json.dumps(result,ensure_ascii=False),flush=True)
    save(OUT/'static_validation.json',dict(all_passed=all(r['passed'] for r in results),results=results))
elif args.action=='moving':
    sys.path.insert(0,str(ROOT/'tools/analysis'))
    from pixel_camera_tuning_report import analysis_video_window,summarize
    trial=read(RUN/'trial.json');start,end=analysis_video_window(trial)
    folder=RUN/'full_s35_v3'
    assert not folder.exists()
    command=[sys.executable,str(ROOT/'tools/analysis/pixel_camera_relative_pose.py'),
        '--trial',str(RUN/'trial.json'),'--marker-reference',str(RUN/'marker_reference_used.json'),
        '--body-geometry',str(OUT/'body_geometry_B_green_front_s35_v3.json'),
        '--camera-height-mm',str(trial['camera_height_mm']),'--height-source','approximate_for_capture',
        '--fixed-camera','--captured-trial','--video-start',str(start),'--video-end',str(end),
        '--frame-stride','1','--output',str(folder)]
    with (OUT/'moving.stdout.txt').open('x',encoding='utf-8') as log:
        subprocess.run(command,check=True,stdout=log,stderr=subprocess.STDOUT,env=ENV,
            creationflags=subprocess.CREATE_NO_WINDOW,timeout=900)
    try:
        summary=summarize(folder)
    except Exception as error:
        summary=dict(status='NEEDS_OPERATOR_REVIEW',auto_stop_confirmed=False,error=str(error))
        save(folder/'summary.json',summary)
    report=read(folder/'report.json')
    save(OUT/'moving_validation.json',dict(required_motion_quality=report['required_motion_window_quality'],
        trajectory_fit_rmse_mm=report['trajectory_fit_rmse_mm'],summary=summary,
        original_s45_analysis_unchanged=True,geometry=str(OUT/'body_geometry_B_green_front_s35_v3.json')))
    print(json.dumps(dict(state='MOVING_REVIEW_COMPLETE',quality=report['required_motion_window_quality']),ensure_ascii=False),flush=True)
