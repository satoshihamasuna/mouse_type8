"""R180の角度だけ次候補3stageをprepare。新stageだけに最新静止窓hookを適用。実機操作なし。"""
from pathlib import Path
import ast
import argparse
from copy import deepcopy
import hashlib
import json
import os
import subprocess
import runpy
import shutil

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_r180_20261120_v1'
SOURCE=BASE/'finish02_01'
RUN=SOURCE/'local_execution/run_A_R180_1800_finish02_c3p5_r48_Le31p5_20261120_001'
IN135=ROOT/'tools/logs/pixel_camera/turn1800_A_inr135_20261120_v1/baseline01/run_recorded_once.py'
INSPECT=ROOT/'tools/analysis/pixel_camera_static_window.py'
REVIEW=BASE/'parallel_finish/front_red_finish02_final/scope_review.json'
OUT=BASE/'parallel_finish/angle_c4p25_preparation_20261120'
STAGES=[BASE/f'angle_c4p25_{i:02d}' for i in (1,2,3)]
ASSET_TEMPLATE=BASE/'corr3p5_r49_Le34p5_01'
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def save(path,value):
    with path.open('x',encoding='utf-8') as stream:json.dump(value,stream,ensure_ascii=False,indent=2,allow_nan=False)


def function(tree):
    return next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='dense_static_check')


parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--resume-preparation',action='store_true',help='同一basisがありstage未生成の自分の準備だけ再開')
args=parser.parse_args()
if (OUT.exists() and not args.resume_preparation) or any(p.exists() for p in STAGES):raise FileExistsError('既存候補/根拠は上書きしません')
actual,review=read(RUN/'saved_1ms.settings.json'),read(REVIEW)
assert actual['parameters']==dict(velo=1.8,r_min=-48.,lstart=7.52,lend=31.5,degree=-180.,correction=3.5,pre_accel=6.5)
hook_source=IN135.read_text(encoding='utf-8-sig');hook_tree=ast.parse(hook_source);hook_function=function(hook_tree)
hook_nodes=hook_function.body[1:6]
assert isinstance(hook_nodes[1],ast.ImportFrom) and hook_nodes[1].module=='pixel_camera_static_window'
assert any(isinstance(n,ast.Name) and n.id=='inspect_static_window' for n in ast.walk(hook_nodes[2]))
lines=hook_source.splitlines(keepends=True)
hook=''.join(lines[hook_nodes[0].lineno-1:hook_nodes[-1].end_lineno])
assert 'inspect_static_window(manifest)' in hook and "window_quality['passed'] is not True" in hook
files=[RUN/'saved_1ms.settings.json',SOURCE/'run_recorded_once.py',SOURCE/'static_pose_after_settle.py',
       SOURCE/'static_settle_policy.json',IN135,INSPECT,REVIEW,ROOT/'Task/Src/init_motion.cpp']
hashes={str(p):sha(p) for p in files}
basis=dict(mouse='A',motion='long_r180',preset_speed_mm_s=1800,state='UNEXECUTED_ANGLE_ONLY_LINE_DIAGNOSTIC_CANDIDATE',
    source_run=str(RUN),source_settings=str(RUN/'saved_1ms.settings.json'),source_review=str(REVIEW),
    source_parameters=actual['parameters'],candidate_parameters=dict(actual['parameters'],correction=4.25),
    geometry_change=dict(correction_delta_deg=.75,r_min_delta_mm=0.,lstart_delta_mm=0.,lend_delta_mm=0.),
    diagnostic_return_path_angle_deg=review['return_path_direction_deg'],diagnostic_error_deg=review['return_path_error_deg'],
    diagnostic_return_spacing_at_forward0_mm=89.11961632810613,
    diagnostic_scope='line diagnostic only; not full-turn geometry acceptance or calibrated body yaw',
    original_midpoint_required_observation_rate=review['original_midpoint_required_observation_rate'],original_midpoint_passed=False,
    front_pure_motion_observation_rate=review['front_motion_only_rate'],front_pure_motion_passed=False,
    front_required_rate=review['front_required_rate'],half_span_scenarios=review['half_span_scenarios'],
    keep_lengths_reason='Unknown halfspan r20/25/30 shifts front Lend forward target50/40/30mm; diagnostic alone does not identify Lstart/Lend. Width atforward0 is89.120mm, close to nominal90; keep radius and lengths fixed.',
    correction_sign_basis='Task/Src/init_motion.cpp motion_degree=degree+degree_correction. Right degree-180: correction+0.75 reduces commanded right rotation magnitude0.75deg.',
    simple_angle_prediction_deg=review['return_path_direction_deg']-.75,
    prediction_is_measured_response=False,prediction_is_validated_causal_gain=False,
    preserved=['actual PID','actual feedforward','suction enabled/duty','velocity','degree','pre_accel','start placement','camera','post straight acceleration20'],
    static_window_s=[3.3,4.95],static_frame_stride=2,
    static_hook_source=str(IN135),static_hook_scope='Only newstage dense_static_check capture-quality block; recording manifest unmodified, original recording failure retained, V2 pose checks unchanged.',
    operator_ready=False,physical_execution=False,parameter_adoption_allowed=False,
    limitations=['Pure motion front94.667% and original midpoint93.197% remain failures.',
        '180.967deg is observed red-path return direction, not absolute body yaw.',
        'Explicit diagnostic exit window and marker-selection failures on anotherR180 prevent whole-turn adoption.',
        'Angle response to correction is not independently fitted; this is a single-variable trial proposal only.'],source_sha256=hashes)
if OUT.exists():assert read(OUT/'basis.json')==basis,'再開basisが元の準備と一致しません'
else:OUT.mkdir();save(OUT/'basis.json',basis)
command=['C:/Users/sato1/Documents/git/Debug_camera/.venv/Scripts/python.exe',str(ROOT/'tmp/mouse_tuning_prepare_candidate_20261120.py'),
    '--base',str(ASSET_TEMPLATE),'--source-run',str(RUN),'--name','angle_c4p25_','--id-prefix','A_R180_1800_angle_c4p25_r48_Le31p5',
    '--correction','4.25','--r-min','-48','--lstart','7.52','--lend','31.5','--count','3','--date','20261120','--basis',str(OUT/'basis.json')]
result=subprocess.run(command,check=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,env=dict(os.environ,PYTHONUTF8='1'))
save(OUT/'stage_preparation.json',json.loads(result.stdout))
helper=runpy.run_path(str(ROOT/'tmp/mouse_tuning_prepare_candidate_20261120.py'))
for stage in STAGES:
    phase=read(stage/'phase.json');identifier=phase['request_id']
    before,_,_,_=helper['change_default']((SOURCE/'run_recorded_once.py').read_text(encoding='utf-8-sig'),identifier)
    wrapper=stage/'run_recorded_once.py'
    for filename in ('config.json','static_pose_after_settle.py','static_settle_policy.json'):
        shutil.copyfile(SOURCE/filename,stage/filename)
    plan=read(SOURCE/'placement_plan.json')
    plan.update(candidate=str(stage/'candidate.settings.json'),config=str(stage/'config.json'),request_id=identifier,next_request_id=identifier,
                operator_ready=False,next_candidate_executed=False,execution_pending=False,current_placement_confirmed=False,current_clearance_confirmed=False)
    (stage/'placement_plan.json').write_text(json.dumps(plan,ensure_ascii=False,indent=2),encoding='utf-8')
    phase.update(source_stage=str(SOURCE),static_gate_asset_template=str(ASSET_TEMPLATE))
    (stage/'phase.json').write_text(json.dumps(phase,ensure_ascii=False,indent=2),encoding='utf-8')
    with (stage/'run_recorded_once.pre_static_window_hook.py').open('x',encoding='utf-8') as stream:stream.write(before)
    with (stage/'static_window_hook_source_IN135.py').open('x',encoding='utf-8') as stream:stream.write(hook_source)
    tree=ast.parse(before);old_function=function(tree);old_if=old_function.body[1]
    assert isinstance(old_if,ast.If) and "recording['state']" in ast.get_source_segment(before,old_if)
    src=before.splitlines(keepends=True);after=''.join(src[:old_if.lineno-1])+hook+''.join(src[old_if.end_lineno:])
    expected=deepcopy(tree);function(expected).body[1:2]=deepcopy(hook_nodes)
    after_tree=ast.parse(after)
    assert ast.dump(expected,include_attributes=False)==ast.dump(after_tree,include_attributes=False)
    assert 'stationary_video_window_s=(3.3, 4.95), stationary_frame_stride=2' in after
    assert (stage/'static_pose_after_settle.py').read_bytes()==(SOURCE/'static_pose_after_settle.py').read_bytes()
    assert read(stage/'static_settle_policy.json')['stationary_frame_stride']==2
    compile(after,str(wrapper),'exec')
    wrapper.write_text(after,encoding='utf-8',newline='\n')
    save(stage/'static_window_hook_validation.json',dict(state='AST_VALIDATED_NOT_EXECUTED',
        source_IN135=str(IN135),source_IN135_sha256=hashes[str(IN135)],original_wrapper_sha256=sha(stage/'run_recorded_once.pre_static_window_hook.py'),
        updated_wrapper_sha256=sha(wrapper),only_dense_static_check_quality_block_changed=True,
        static_window_s=[3.3,4.95],frame_stride=2,static_pose_wrapper_unchanged=True,
        whole_manifest_reclassification=False,pose_gate_unchanged=True,operator_ready=False,physical_execution=False))
assert all(sha(Path(path))==digest for path,digest in hashes.items())
print(json.dumps(dict(state='PREPARED_NOT_EXECUTED',basis=str(OUT/'basis.json'),stages=[str(p) for p in STAGES],source_hashes_unchanged=True),indent=2))
