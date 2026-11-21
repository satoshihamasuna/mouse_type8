"""通信負荷を下げた同一out135候補と、切断調査・変更検証の根拠を保存する。"""
from pathlib import Path
from datetime import datetime, timezone
import ast
import copy
import difflib
import hashlib
import json
import shutil
import pandas as pd

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_outr135_20261120_v1'
OUT=BASE/'telemetry_period_recovery_20261121_v1'
PROBE=BASE/'communication_probe_20261121_v1'
SOURCE=BASE/'c2p1_Ls0_Le37_03'
OLD_ID='A_OUTR135_1800_c2p1_Ls0_Le37_20261120_003'
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def save(path,value):
    with path.open('x',encoding='utf-8') as file:
        json.dump(value,file,ensure_ascii=False,indent=2,allow_nan=False);file.write('\n')


operator=read(SOURCE/'local_execution/operator_results'/f'{OLD_ID}.json')
assert operator['both_powered_off'] and operator['placement_changed'] and not operator['ran']
prior=read(SOURCE/'candidate.settings.json')
assert prior['parameters']['lstart']==0 and prior['parameters']['lend']==37 and prior['parameters']['correction']==2.1
stages=[]
for index in (1,2,3):
    stage=BASE/f'c2p1_Ls0_Le37_stream10_{index:02}'
    stage.mkdir(exist_ok=False)
    candidate=copy.deepcopy(prior)
    candidate['source']='Same unexecuted Ls0/Le37/c2.1 candidate; continuous telemetry10ms recovery only, retained log1ms. Not adopted.'
    save(stage/'candidate.settings.json',candidate)
    for name in ('body_geometry_A_out135_yellow_s20.json','static_pose_after_settle.py','static_settle_policy.json'):
        shutil.copyfile(SOURCE/name,stage/name)
    config=read(SOURCE/'config.json')
    config.update(geometry=str(stage/'body_geometry_A_out135_yellow_s20.json'),stream_period_ms=10)
    save(stage/'config.json',config)
    identifier=f'A_OUTR135_1800_c2p1_Ls0_Le37_s10_20261121_{index:03}'
    wrapper=(SOURCE/'run_recorded_once.py').read_text(encoding='utf-8-sig')
    assert wrapper.count(OLD_ID)==1;wrapper=wrapper.replace(OLD_ID,identifier);ast.parse(wrapper)
    with (stage/'run_recorded_once.py').open('x',encoding='utf-8') as file:file.write(wrapper)
    plan=read(SOURCE/'placement_plan.json')
    plan.update(state='STREAM10_CANDIDATE_READY_CURRENT_PLACEMENT_REQUIRED',request_id=identifier,next_request_id=identifier,
        candidate=str(stage/'candidate.settings.json'),config=str(stage/'config.json'),stream_period_ms=10,
        retained_log_period_ms=1,candidate_requested_parameters=candidate['parameters'],
        operator_ready=False,execution_pending=False,next_candidate_executed=False,
        current_placement_confirmed=False,current_clearance_confirmed=False,current_hands_clear_confirmed=False,
        power_state='Both_OFF_confirmed_placement_changed_after_communication_probe',both_powered_off_confirmed=True,
        physical_placement_changed=True,battery_charge_requested=False,battery_charge_status='last_read7819mV; re-read before execution',
        source_params_changed=False,last_failure_review=str(OUT/'verification.json'),
        guide=str(BASE/'c2p1_Ls0_Le37_preparation_v1/start_placement_guide.png'))
    save(stage/'placement_plan.json',plan);stages.append(dict(stage=str(stage),trial_id=identifier))

files=['tools/pixel_camera_tuning.py','tools/pixel_camera_turn_capture.py','tools/pixel_camera_tuning_cli.py',
    'tools/pixel_camera_tuning_mcp.py','tools/tests/test_camera_startup.py','tools/tests/test_pixel_camera_tuning.py',
    'tools/tests/test_pixel_camera_tuning_cli.py','tools/tests/test_pixel_camera_tuning_mcp.py','tools/pixel_camera_README.md']
patch=[];hashes={}
for name in files:
    p=ROOT/name;before=OUT/('before_'+p.name)
    p.write_bytes(p.read_bytes().replace(b'\r\n',b'\n').replace(b'\n',b'\r\n'))
    patch.extend(difflib.unified_diff(before.read_text(encoding='utf-8').splitlines(True),p.read_text(encoding='utf-8').splitlines(True),fromfile='a/'+name,tofile='b/'+name))
    shutil.copyfile(p,OUT/('after_'+p.name))
    if p.suffix=='.py':compile(p.read_text(encoding='utf-8'),str(p),'exec')
    hashes[name]=dict(before=sha(before),after=sha(p))
with (OUT/'patch_from_this_task.patch').open('x',encoding='utf-8') as file:file.write(''.join(patch))
idle=pd.read_csv(PROBE/'idle_stream_5ms_20s.csv')
assert idle['ideal.velo'].abs().max()==0 and idle['ideal.rad_velo'].abs().max()==0
diagnostic=read(PROBE/'link_diagnostics_after_idle_stream.json')
failures=[]
for index in (1,2,3):
    rid=f'A_OUTR135_1800_c2p1_Ls0_Le37_20261120_{index:03}'
    path=BASE/f'c2p1_Ls0_Le37_{index:02}/local_execution/jobs/{rid}.json'
    job=read(path);assert job['result']['execution_write_attempted'] is False
    failures.append(dict(request_id=rid,job=str(path),job_sha256=sha(path),error=job['result']['error']))
counts={'test_camera_startup.py':6,'test_pixel_camera_tuning_cli.py':28,'test_pixel_camera_tuning_mcp.py':7,
    'test_pixel_camera_turn_capture.py':26,'test_pixel_camera_tuning.py':27,'test_mouse_ble_transport.py':38}
assert sum(counts.values())==132
save(OUT/'verification.json',dict(mouse='A',motion='out_r135',preset_mm_s=1800,
    state='SOFTWARE_VERIFIED_STREAM10_HARDWARE_RECOVERY_NOT_YET_VERIFIED',
    operator_current_state=operator,unexecuted_attempts=failures,read_only_shell_probe=read(PROBE/'result.json'),
    idle_5ms_probe=dict(rows=len(idle),elapsed_mcu_ms=int(idle.time_ms.iloc[-1]-idle.time_ms.iloc[0]),
        ideal_motion_zero=True,lost=int(idle.lost_frames.sum()),crc_errors_reported_by_cli=0),
    bridge_diagnostics=diagnostic,
    evidence_limit='Ring highwater reached capacity and cumulative dropped bytes were nonzero after disconnected sessions. Loss timing is unknown; post-disconnect stream lease may contribute. Underlying radio/OS cause not established.',
    change='Optional stream_period_ms for MCP/CLI;10ms selected, default5ms retained. Gateway flag, health readback, trial and motion sidecar agree. RAM-held log stays1ms.',
    validation=dict(suites=counts,total_passed=132,skips=0,
        covers=['invalid period offline rejection','10ms subprocess argument','MCP period propagation',
            '10ms trial/motion sidecars with1ms retained log','period mismatch blocks execution','single execute and no ambiguous replay','GUI defaults']),
    before_new_feature_tests=dict(tests=6,errors=5,reason='new constructor argument not yet implemented'),
    firmware_changed=False,turn_candidate_parameters_changed=False,quality_thresholds_relaxed=False,
    gui_environment_note='GUI test ran outside sandbox for Tcl initialization; no real devices.',
    new_stages=stages,current_operator_ready=False,hardware_operations_for_preparation=0,source_sha256=hashes))
root=BASE/'preparation.json';shutil.copyfile(root,OUT/'before_root_preparation.json')
plan=read(BASE/'c2p1_Ls0_Le37_stream10_01/placement_plan.json')
plan.update(updated_at=datetime.now(timezone.utc).isoformat(),last_request_id=OLD_ID,last_request_executed=False,
    last_saved_log_status='no_motion_capture',last_successful_capture_id='A_OUTR135_1800_c2p1_Ls0_Le35p2_20261120_003',
    camera_health_report=str(OUT/'phone_health_before_on.json'))
root.write_text(json.dumps(plan,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(prepared=True,stream_period_ms=10,retained_log_period_ms=1,tests_passed=132,stages=stages,hardware_operations=0),ensure_ascii=False))
