"""実機を使わず接続引継ぎの終了処理を確認し、未実行試行を記録する。"""
from pathlib import Path
import ast
import hashlib
import json
import secrets
import sys

import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT/'tools/logs/pixel_camera/turn1600_B_r90_20261120_v1'
RUN = BASE/'cli_session05/run_B_R90_1600_Lstart13p6_green_20261120_002'
sys.path[:0] = [str(ROOT/'tools'), str(ROOT/'tools/analysis')]
import pixel_camera_tuning_cli as cli


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def write_new(path, value):
    with path.open('x',encoding='utf-8') as output:
        json.dump(value,output,ensure_ascii=False,indent=2)


source = ROOT/'tmp/turn1600_B_r90_device_prepare_single_link_20261120.py'
tree = ast.parse(source.read_text(encoding='utf-8'))
function = next(node for node in tree.body if isinstance(node,ast.FunctionDef) and node.name=='serve_prepared_session')
code = compile(ast.Module(body=[function],type_ignores=[]),str(source),'exec')
test_results = []
for simulated_failure in (False, True):
    class FakeSession:
        def __init__(self):
            self.close_count = 0
        def close(self):
            self.close_count += 1
    fake = FakeSession()
    servers = []
    class FakeServer:
        server_port = 8767
        closed = False
        def __init__(self, service):
            self.service = service
            assert service.session is fake
        def shutdown(self):
            pass
        def serve_forever(self, **kwargs):
            if simulated_failure:
                raise RuntimeError('SIMULATED_SERVER_FAILURE_NO_HARDWARE')
            self.service.close()
        def server_close(self):
            self.closed = True
    def make_server(service, token, port):
        assert port == 8767 and token
        server = FakeServer(service)
        servers.append(server)
        return server
    namespace = dict(Path=Path, BASE=BASE, secrets=secrets, json=json,
        utc_now=lambda:'TEST_ONLY', save=lambda name,value:None)
    exec(code,namespace)
    original = cli.make_http_server
    cli.make_http_server = make_server
    folder = BASE/('single_link_lifecycle_failure_test' if simulated_failure else 'single_link_lifecycle_close_test')
    try:
        try:
            namespace['serve_prepared_session'](fake,folder)
        except RuntimeError as error:
            assert simulated_failure and str(error)=='SIMULATED_SERVER_FAILURE_NO_HARDWARE'
        assert fake.close_count == 1
        assert servers[0].closed and not (folder/'service.lock').exists()
        test_results.append(dict(simulated_failure=simulated_failure, passed=True,
            existing_session_reused=True, closed_once=True, lock_removed=True, hardware_used=False))
    finally:
        cli.make_http_server = original
write_new(BASE/'single_link_lifecycle_verification.json',dict(tests=test_results,
    source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(), hardware_connection_stability_proven=False))

trial = read(RUN/'trial.json')
capture = read(Path(trial['capture_json']))
close = read(BASE/'cli_session05/jobs/close_after_green02_noexecution_001.json')
assert close['state']=='DONE'
assert trial['execution_write_attempted'] is False and trial['camera_run_start_confirmed'] is False
assert trial['execution_resend_count']==0
events = [json.loads(line) for line in (RUN/'events.jsonl').read_text(encoding='utf-8').splitlines()]
assert not any(event['event'] in ('execution_write_attempt','camera_permit_confirmed') for event in events)
telemetry = pd.read_csv(RUN/'telemetry.csv')
assert not ((telemetry['ideal.velo'].abs()>.01) | (telemetry['ideal.rad_velo'].abs()>.01)).any()
recovered = Path(capture['recovered_video_file'])
assert recovered.is_file()
qa = dict(mouse='B', motion='long_r90', preset_mm_s=1600,
    request_id='B_R90_1600_Lstart13p6_green_20261120_002',
    state='NOT_EXECUTED_BLE_DISCONNECTED_BEFORE_ARM',
    execution_write_attempted=False, camera_permit_armed=False, execution_resend_count=0,
    complete_commanded_motion=False, included_in_repeat_count=False,
    error=capture['error'], gateway_error='RuntimeError: BLE disconnected; in-flight data will not be replayed',
    low_battery_rejection=False, last_pre_recording_battery_mv=7356,
    recovered_video_file=str(recovered), recovered_video_sha256=hashlib.sha256(recovered.read_bytes()).hexdigest(),
    recovered_video_contains_tuning_run=False, original_trial_and_capture_preserved=True,
    strict_static_alignment_passed=False, relative_shape_analysis_performed=False,
    operator_stopped_both_off=True, ble_close_done=True,
    source_params_changed=False, firmware_write_performed=False,
    next_action='旧要求を再送しない。新規ON・経路・手離しの確認後、別ID003を設定確認と同じBLE接続で準備する。',
    source_hashes={str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest() for path in
        [RUN/'trial.json', Path(trial['capture_json']), RUN/'events.jsonl', RUN/'telemetry.csv']})
write_new(RUN/'unexecuted_attempt_review.json',qa)
write_new(RUN/'operator_postrun_confirmation.json',dict(stopped=True,both_off=True,
    user_reply='停止・両方OFF', source='current asynchronous user reply', execution_sent=False))

state = read(BASE/'resume_state.json')
state.update(state='GREEN_REPEAT02_NOT_EXECUTED_BLE_DROP_NEW_ATTEMPT_PREPARED_BOTH_OFF',
    power_state='BOTH_OFF_USER_CONFIRMED', robot_connected=False,
    operator_power_confirmation='2026-11-20 user confirmed stopped and both OFF after unexecuted002',
    execution_pending=False, pending_request_id=None, pending_request_sent=False,
    last_attempt_request_id=qa['request_id'], last_attempt_execution_sent=False,
    last_attempt_quality_review=str(RUN/'unexecuted_attempt_review.json'),
    current_position='Start placement maintained; no commanded motion in attempted002.',
    current_hands_clear_confirmed=False, next_unsent_request_id='B_R90_1600_Lstart13p6_green_20261120_003',
    next_request_sent=False, same_setting_repeat_count=1,
    device_prepare_script='tmp/turn1600_B_r90_device_prepare_single_link_20261120.py',
    next_action='新しい1回に対する両方ON・経路干渉なし・UART配線・手離しの返答を待つ。準備と撮影は同じBLE接続を使い、002は再送しない。')
(BASE/'resume_state.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(lifecycle_tests_passed=len(test_results),execution_sent=False,
    complete_runs=4, green_same_condition_runs=1,both_off=True,next_request_sent=False)))
