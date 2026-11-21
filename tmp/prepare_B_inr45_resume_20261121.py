"""B1600 inR45の残り反復をオフラインで準備する。BLE接続・設定送信・走行はしない。"""
from pathlib import Path
from datetime import datetime,timezone
import argparse
import hashlib
import json
import os
import shutil
import socket
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr45_20261120_v1'
STAGE=BASE/'geometry02_repeat_02'
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--name',default='preparation_20261121_001')
args=parser.parse_args()
assert args.name.startswith('preparation_20261121_') and args.name.isidentifier()
OUT=BASE/args.name;OUT.mkdir(exist_ok=False)
read=lambda p:json.loads(p.read_text(encoding='utf-8'))
def save(p,v):
    with p.open('x',encoding='utf-8') as f:json.dump(v,f,ensure_ascii=False,indent=2,allow_nan=False)
config=read(STAGE/'config.json');candidate=read(STAGE/'candidate.settings.json')
ble_port=int(config.get('ble_port',8766))
assert ble_port in (8766,8767)
state=read(BASE/'resume_state.json');identifier=state['next_request_id']
assert identifier=='B_INR45_1600_geometry02_repeat02_green_20261120_008'
assert not state['next_request_sent']
assert not (STAGE/'local_execution/jobs'/f'{identifier}.json').exists()
assert not (STAGE/'local_execution'/f'{identifier}.stdout.txt').exists()
assert config['mouse']==candidate['mouse']=='B' and config['address']=='E1:7F:86:21:84:73'
assert candidate['motion_name']=='in_r45' and candidate['preset_speed_mm_s']==1600
assert candidate['turn_count']==1 and candidate['suction']==dict(enabled=True,duty=650)
assert candidate['parameters']['lstart']==7.8 and candidate['parameters']['lend']==34.5 and candidate['parameters']['correction']==-1.5
comparison=read(BASE/'geometry02_2run_review_v1/review.json')
for row in comparison['rows']:
    assert all(candidate[key]==row['settings'][key] for key in ('parameters','pid','feedforward','suction','turn_count'))
for key in ('reference','geometry'):
    assert Path(config[key]).is_file()
    shutil.copyfile(config[key],OUT/(key+'.snapshot.json'))
geometry=read(Path(config['geometry']));reference=read(Path(config['reference']))
assert geometry['mouse']=='B' and geometry['front_colour']=='green' and geometry['rear_colour']=='red'
assert geometry['reference_id']==reference['reference_id']
for name in ('config.json','candidate.settings.json','placement_plan.json','ble_binding.json'):
    shutil.copyfile(STAGE/name,OUT/name)

# 未送信の起動処理だけを更新し、使用中ポートには接続しない。
runner=STAGE/'run_recorded_once.py'
shutil.copyfile(runner,OUT/'run_recorded_once.before_preparation.py')
source=runner.read_text(encoding='utf-8')
needle='        return super().prepare(address, port=EXPECTED_BLE_PORT, progress=progress)'
assert source.count(needle)==1
if 'BLE_PORT_AVAILABLE_BEFORE_CONNECT' not in source:
    replacement='''        # BLE_PORT_AVAILABLE_BEFORE_CONNECT: 他の機体の既存接続を保持する。
        import socket
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
                if hasattr(socket, 'SO_EXCLUSIVEADDRUSE'):
                    probe.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
                probe.bind(('127.0.0.1', EXPECTED_BLE_PORT))
        except OSError as error:
            raise RuntimeError(f'BLE{EXPECTED_BLE_PORT}は使用中です。既存接続は操作せず、この試行を停止します') from error
'''+needle
    source=source.replace(needle,replacement)
    runner.write_text(source,encoding='utf-8')
compile(source,str(runner),'exec')
flags=subprocess.CREATE_NO_WINDOW if os.name=='nt' else 0
env=dict(os.environ,PYTHONUTF8='1',TYPE8_ANALYSIS_PYTHON='D:/venv/venv3-11/Scripts/python.exe')
validated=subprocess.run([sys.executable,str(runner)],capture_output=True,text=True,encoding='utf-8',
    env=env,creationflags=flags,timeout=30)
(OUT/'offline_validation.stdout.txt').write_text(validated.stdout,encoding='utf-8')
(OUT/'offline_validation.stderr.txt').write_text(validated.stderr,encoding='utf-8')
assert validated.returncode==0 and 'VALIDATED; no connection or robot command sent' in validated.stdout

sys.path.insert(0,str(ROOT/'tools'))
import pixel_camera_tuning_cli  # 既存のBLE依存パスも登録する。
from pixel_camera_tuning import check_analysis_environment,analysis_python
import av
check_analysis_environment()
from pixel_camera_capture import camera_workspace
sys.path.insert(0,str(camera_workspace()/'python'))
from pixel_camera import PixelCameraClient
camera=PixelCameraClient(connection='wifi',port=8765,auto_start=False)
camera_status=camera.get_status()  # /api/statusの読取りだけ。録画・configureは送らない。
device=camera_status.get('device',{})
filtered=dict(connected=camera_status.get('connected'),device_id=camera_status.get('device_id'),
    device={key:device.get(key) for key in ('phase','pending_count','owner','recording_id','exposure_compensation_steps','zoom_ratio','torch','focus_lock')},
    read_only=True,configure_sent=False,recording_started=False)
save(OUT/'camera_status.json',filtered)
port_free=True;port_error=None
try:
    with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as probe:
        if hasattr(socket,'SO_EXCLUSIVEADDRUSE'):probe.setsockopt(socket.SOL_SOCKET,socket.SO_EXCLUSIVEADDRUSE,1)
        probe.bind(('127.0.0.1',ble_port))
except OSError as error:
    port_free=False;port_error=str(error)
files=[STAGE/'config.json',STAGE/'candidate.settings.json',runner,Path(config['reference']),Path(config['geometry']),
    ROOT/'tools/pixel_camera_tuning.py',ROOT/'tools/pixel_camera_tuning_mcp.py',ROOT/'tools/ble/README.md',
    ROOT/'tmp/postprocess_B_inr45_20261120.py',ROOT/'tmp/summarize_B_inr45_geometry02_repeats_20261120.py']
assert all(p.is_file() for p in files)
result=dict(prepared_at_utc=datetime.now(timezone.utc).isoformat(),mouse='B',motion='in_r45',preset_mm_s=1600,
    pending_request_id=identifier,request_sent=False,offline_validation_passed=True,
    parameters=candidate['parameters'],pid=candidate['pid'],feedforward=candidate['feedforward'],suction=candidate['suction'],
    same_as_previous_two_runs=True,same_candidate_runs=2,remaining_repeats=1,
    ble=dict(address=config['address'],advertised_name='HY0020-MOUSE-B',port=ble_port,
        address_guard_present=True,port_guard_present=True,port_free_at_check=port_free,port_error=port_error,
        connections_started=0,commands_sent=0),
    camera=filtered,camera_ready_at_check=bool(filtered['connected'] and device.get('phase')=='IDLE'),
    analysis_environment_ready=True,analysis_python=str(analysis_python()),
    current_physical_power_state_unconfirmed=True,charging_completion_confirmed=state.get('charging_completion_confirmed',False),
    current_operator_ready=False,firmware_written=False,source_parameters_changed_by_this_preparation=False,
    source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files})
save(OUT/'checks.json',result)
state.update(state='REPEAT008_PREPARED_20261121',latest_preparation=str(OUT/'checks.json'),
    next_operator_ready=False,execution_pending=False,next_request_sent=False,
    current_physical_power_state_unconfirmed=True,
    next_action='充電完了後、同じ盤/撮影条件・前緑後赤を確認し、元225/45mm・前下向きへ概略配置。干渉なし・両方ON・手離しの今回確認後、新Bアドレスで電圧/全候補値を読み返して008を一回だけ取得。')
(BASE/'resume_state.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
(OUT/'README.md').write_text('# 機体B turn1600 in45：2026-11-21の再開準備\n\n'
    '残りは同一候補の反復1本。Lstart7.8 mm、Lend34.5 mm、補正−1.5度、半径−55 mm、吸引650。角加減速FF0.000750、角速度Ki0.005。前2本の実機値との一致を確認済み。\n\n'
    f'接続先はB専用E1:7F:86:21:84:73（HY0020-MOUSE-B）、BLE{ble_port}。カメラは8765。使用中のBLEポートへは接続せず停止する確認を未送信の起動処理に追加。既存接続・取得済みログは保持。\n\n'
    '保存撮影条件は高さ1500 mm、前緑/後赤、EV−1.5・1.8倍・ライトOFF・AF固定。解析環境、設定、参照JSON、未使用要求IDを確認。カメラは状態照会だけで、設定変更・録画を行っていない。\n\n'
    '充電完了と現在の設置/電源状態は未確認。元225/45 mm・前下向きへおおよそ配置し、今回の干渉/停止余裕、両方ON・手離しを確認後に一回取得。入口は動画で相対補正し、各評価フレームの実ArUco参照と全フレーム品質を維持する。\n\n'
    '準備ではBLE接続・set/exe・ファーム書込みは0件。機体/制御ソースを変更していないためビルドは不要。新BLE経由の電圧・実機設定・走行は再開時の確認事項。\n',encoding='utf-8')
print(json.dumps(dict(preparation=str(OUT),offline_validated=True,camera_ready=result['camera_ready_at_check'],
    ble_port_free=port_free,ble_address=config['address'],request_sent=False),ensure_ascii=False),flush=True)
