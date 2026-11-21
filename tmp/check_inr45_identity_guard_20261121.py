"""実機に接続せず、今回の運用ラッパーの機体照合を検証する。"""
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch
from hashlib import sha256
import ast
import json

ROOT=Path(__file__).resolve().parents[1]
STAGE=ROOT/'tools/logs/pixel_camera/turn1800_A_inr45_20261120_v1/mid_identity_checked_01'
wrapper=STAGE/'run_recorded_once.py'
tree=ast.parse(wrapper.read_text(encoding='utf-8'))
cut=next(i for i,n in enumerate(tree.body) if isinstance(n,ast.Assign)
    and any(isinstance(t,ast.Name) and t.id=='parser' for t in n.targets))
namespace=dict(__file__=str(wrapper),__name__='identity_guard_test')
exec(compile(ast.Module(body=tree.body[:cut],type_ignores=[]),str(wrapper),'exec'),namespace)

class Shell:
    def __init__(self,response):self.response=response;self.queries=[];self.sent=[]
    def query(self,command,predicate):
        self.queries.append(command)
        assert command=='explore show'
        if isinstance(self.response,Exception):raise self.response
        assert predicate(self.response)
        return self.response
    def send(self,command):
        self.sent.append(command)
        raise AssertionError('Identity check must precede all settings writes')

cases=[
    ('A_idle','EXPLORE_CAP protocol:1 mouse:A state:idle',True),
    ('B_idle','EXPLORE_CAP protocol:1 mouse:B state:idle',False),
    ('A_running','EXPLORE_CAP protocol:1 mouse:A state:running',False),
    ('missing_mouse','EXPLORE_CAP protocol:1 state:idle',False),
    ('missing_state','EXPLORE_CAP protocol:1 mouse:A',False),
    ('unknown_protocol','EXPLORE_CAP protocol:2 mouse:A state:idle',False),
    ('read_timeout',TimeoutError('No identity response'),False),
]
results=[]
with TemporaryDirectory(dir=ROOT/'tmp') as temporary:
    for label,response,accepted in cases:
        device=namespace['RecordedDevice'](Path(temporary)/label,dict(mouse='A'))
        device.modes_recorded=True
        shell=Shell(response)
        with patch.object(namespace['TuningService'],'read_settings',return_value=dict(verified_values=True)) as readback:
            try:
                device.read_settings(shell,'in_r45')
                assert accepted
                device.read_settings(shell,'in_r45')
                assert readback.call_count==2
            except (RuntimeError,TimeoutError):
                assert not accepted
                readback.assert_not_called()
        assert shell.queries==['explore show'] and not shell.sent
        path=device.folder/'firmware_identity.json'
        if not isinstance(response,Exception):
            record=json.loads(path.read_text(encoding='utf-8'))
            assert record['verified']==accepted and record['expected_mouse']=='A'
        results.append(dict(case=label,accepted=accepted,settings_writes=0,passed=True))

expected=json.loads((STAGE.parent/'mid_status_retry_02/candidate.settings.json').read_text(encoding='utf-8'))
actual=json.loads((STAGE/'candidate.settings.json').read_text(encoding='utf-8'))
assert all(expected[k]==actual[k] for k in ['parameters','pid','feedforward','suction','turn_count'])
value=dict(cases=results,passed=len(results),skipped=0,robot_commands_sent=0,
    requested_settings_exact_match=True,wrapper_sha256=sha256(wrapper.read_bytes()).hexdigest())
with (STAGE/'identity_guard_validation.json').open('x',encoding='utf-8') as stream:
    json.dump(value,stream,indent=2);stream.write('\n')
print(json.dumps(value,indent=2))
