"""未走行失敗を保持し、既存通信ベンチだけの有限試験を準備する。実機操作なし。"""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json
root=Path(__file__).resolve().parents[1]
base=root/'tools/logs/pixel_camera/turn1600_B_outr135_20261121_resume_v1'
stage=base/'repeat_c1p0_Ls5p3_Le31_02'
identifier='B_OUTR135_1600_c1p0_Ls5p3_Le31_repeat_green_20261121_002'
run=stage/'local_execution'/('run_'+identifier)
read=lambda path:json.loads(path.read_text(encoding='utf-8-sig'))
def save(path,value):
    with path.open('x',encoding='utf-8') as stream:
        json.dump(value,stream,ensure_ascii=False,indent=2,allow_nan=False);stream.write('\n')
trial=read(run/'trial.json');assert trial['state']=='FAILED' and trial['execution_write_attempted'] is False
events=[json.loads(line) for line in (run/'events.jsonl').read_text(encoding='utf-8').splitlines()]
commands=[event['command'] for event in events if event['event']=='shell_write_attempt']
assert not any(' exe' in command for command in commands)
assert commands[-1].startswith('debug turn out_r135 set ')
stderr=stage/'local_execution'/('transport_'+identifier)/'gateway.stderr.txt'
trace=stderr.read_text(encoding='utf-8',errors='replace')
assert 'BLE disconnected; in-flight data will not be replayed' in trace
assert 'asyncio.exceptions.CancelledError' not in trace
save(base/'repeat002_ble_failure_review.json',dict(recorded_at=datetime.now(timezone.utc).isoformat(),request_id=identifier,
    state='FAILED_DURING_TURN_PARAMETER_WRITE_BEFORE_EXECUTION',source_trial=str(run/'trial.json'),
    error=trial['error'],shell_commands_attempted=commands,execution_commands_sent=0,physical_run_count=0,
    partial_parameter_command_possible=True,settings_fully_verified=False,recording_started=False,
    cleanup_fix_active=True,secondary_cancelled_writer_error_absent=True,
    operator_reply='両方OFF・配置そのまま',power_state='BOTH_OFF_USER_CONFIRMED',
    successful_same_candidate_runs=2,automatic_replay=False,
    next_policy='走行を止め、停止中の通信ベンチでリンク診断と配信周期を確認する。'))
phase_path=stage/'phase.json';phase=read(phase_path)
phase.update(state='FAILED_DURING_TURN_PARAMETER_WRITE_BEFORE_EXECUTION',next_candidate_executed=False,
    execution_pending=False,physical_run_count=0,review=str(base/'repeat002_ble_failure_review.json'))
phase_path.write_text(json.dumps(phase,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
comm=base/'communication_only_v1';assert not comm.exists();comm.mkdir()
command=[str(root/'tools/ble/mouse_ble_benchmark.py'),'--mouse','B','--address','E1:7F:86:21:84:73',
    '--enter-shell','--periods','20,10,5','--duration','20','--settle-seconds','5','--output-root',str(comm/'results')]
plan=dict(prepared_at=datetime.now(timezone.utc).isoformat(),request_id='B_BLE_COMM_ONLY_20261121_001',
    state='PREPARED_NOT_CONNECTED',mouse='B',ble_address='E1:7F:86:21:84:73',command=command,
    periods_ms=[20,10,5],duration_per_period_seconds=20,settle_seconds=5,
    allowed_shell_commands=['shell','explore show'],allowed_binary_operations=['log start/renew','log stop'],
    robot_motion_commands=0,parameter_write_commands=0,camera_operations=0,automatic_reconnect=False,
    mode_requirement='explore showの実機応答でmouse:B state:idleを確認。段階失敗時は以降を実行しない。',
    purpose='5ms取得での繰り返す切断を、停止中の帯域/リンク条件/中継診断として切り分ける。',
    limits='停止状態の試験であり、設定送受信との同時負荷や走行中の安定性を保証しない。',
    source_sha256={name:hashlib.sha256((root/name).read_bytes()).hexdigest() for name in
        ('tools/ble/mouse_ble_benchmark.py','tools/ble/mouse_ble_gateway.py','tools/ble/mouse_ble_log.py')})
save(comm/'plan.json',plan)
launcher='''"""停止中のB通信ベンチのみ。通常起動は計画の検証だけ。"""
from pathlib import Path
import argparse
from datetime import datetime, timezone
import hashlib
import json
import subprocess
import sys
folder=Path(__file__).resolve().parent
root=next(path for path in folder.parents if (path/'tools/ble/mouse_ble_benchmark.py').is_file())
plan=json.loads((folder/'plan.json').read_text(encoding='utf-8-sig'))
for name,digest in plan['source_sha256'].items():
    assert hashlib.sha256((root/name).read_bytes()).hexdigest()==digest, '準備後にソースが変わっています。再確認してください。'
assert plan['mouse']=='B' and plan['ble_address']=='E1:7F:86:21:84:73'
parser=argparse.ArgumentParser()
parser.add_argument('--operator-ready',action='store_true')
args=parser.parse_args()
if not args.operator_ready:
    print('VALIDATED_COMMUNICATION_ONLY; no connection or robot command sent')
    raise SystemExit(0)
with (folder/'started.json').open('x',encoding='utf-8') as stream:
    json.dump(dict(started_at=datetime.now(timezone.utc).isoformat(),request_id=plan['request_id'],motion_commands=0),stream,indent=2)
with (folder/'stdout.txt').open('x',encoding='utf-8') as output:
    result=subprocess.run([sys.executable]+plan['command'],cwd=root,stdout=output,stderr=subprocess.STDOUT)
with (folder/'completion.json').open('x',encoding='utf-8') as stream:
    json.dump(dict(finished_at=datetime.now(timezone.utc).isoformat(),returncode=result.returncode,motion_commands=0),stream,indent=2)
print(json.dumps(dict(returncode=result.returncode,motion_commands=0),ensure_ascii=False),flush=True)
raise SystemExit(result.returncode)
'''
with (comm/'run_communication_only.py').open('x',encoding='utf-8',newline='\n') as stream:stream.write(launcher)
state_path=base/'resume_state.json';state=read(state_path)
save(base/'resume_state_before_communication_only_v1.json',state)
state.update(state='TURN_REPEAT002_FAILED_BOTH_OFF_COMMUNICATION_ONLY_PREPARED',
    power_state='BOTH_OFF_USER_CONFIRMED',operator_power_off_confirmation='両方OFF・配置そのまま',
    operator_ready=False,current_placement_confirmed=True,current_hands_clear_confirmed=False,
    execution_pending=False,ble_connected=False,last_request_id=identifier,
    last_request_result='FAILED_DURING_TURN_PARAMETER_WRITE_BEFORE_EXECUTION',last_error=trial['error'],
    last_execution_write_attempted=False,execution_commands_sent=0,successful_same_candidate_runs=2,
    next_action_kind='communication_only_no_motion',next_stage=str(comm),next_request_id=plan['request_id'],
    next_request_sent=False,communication_plan=str(comm/'plan.json'),
    next_action='走行なし。今回の両方ON/停止/手離し確認後、競合を再確認して既存B通信ベンチを20→10→5ms各20秒だけ行い、終了/失敗後に両方OFFを案内。')
state_path.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
with (base/'README.md').open('a',encoding='utf-8') as stream:
    stream.write('\nrepeat_002もターン設定送信中のBLE切断で未走行。終了処理修正は有効で、本来のBLE切断RuntimeErrorだけを保存し、重複CancelledErrorはない。走行指令0/録画開始0/実走0、操作者は両方OFF・配置保持を確認。成功本数は2本のまま。次は走行を伴わない既存mouse_ble_benchmarkでB実機idle確認後、20→10→5msを各20秒の有限試験とする。リンク/中継/欠落/CRC/実周期を保存し、段階失敗時は以降を行わない。ツールの許可/実行指令は送らず、現在ON確認前は未接続。詳細communication_only_v1/plan.json。\n')
print(str(comm/'run_communication_only.py'))
