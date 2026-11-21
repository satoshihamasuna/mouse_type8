"""B1600右90°の実機値を読み、RAMの単体候補を照合する。exeは送らない。"""
from pathlib import Path
import copy
import json
import math
import sys
import time
import secrets

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'.tools/ble_python311'),str(ROOT/'tools')]
from myshell_debug import DebugGui, PARAM_RE
from pixel_camera_tuning import LiveSession, settings_set_command, setting_differences
from pixel_camera_turn_capture import Shell, utc_now

BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_r90_20261120_v1'
FOLDER=BASE/sys.argv[1]
FOLDER.mkdir(exist_ok=False)
KEYS=('distance','acc','max_velo','end_velo','sp_kp','sp_ki','sp_kd','om_kp','om_ki','om_kd',
      'ff_sp_velo','ff_sp_accel','ff_sp_bias','ff_om_velo','ff_om_accel','ff_om_decel','ff_om_bias')


def parse_straight(line):
    match=PARAM_RE.search(line)
    if not match:
        raise ValueError('直進実機値を取得できない')
    value={key:float(match[key]) for key in KEYS}
    value.update(suction=int(match['suction']),duty=int(match['duty']))
    if not all(math.isfinite(v) for v in value.values()):
        raise ValueError('非有限の実機値')
    return value


def command(value):
    return 'debug straight set '+' '.join(f'{value[k]:.7g}' for k in KEYS)+f" {value['suction']} {value['duty']}"


def save(name,value):
    with (FOLDER/name).open('x',encoding='utf-8') as stream:
        json.dump(value,stream,ensure_ascii=False,indent=2,allow_nan=False)
        stream.write('\n')


def serve_prepared_session(prepared_session, service_folder):
    """準備時の同じ接続を既存CLIへ渡す。ここでは接続・走行を開始しない。"""
    from pixel_camera_tuning_cli import TuningService, load_config, make_http_server, save_json
    service_folder = Path(service_folder).resolve()
    if not service_folder.is_relative_to(BASE.resolve()):
        raise ValueError('今回の測定フォルダー外へ保存しない')
    service_folder.mkdir(exist_ok=False)
    lock = service_folder/'service.lock'
    lock.write_text(utc_now(), encoding='utf-8')
    server = None
    service = None
    try:
        config = load_config(BASE/'config.green_front.v1.json')
        service = TuningService(service_folder, config, session=prepared_session,
            confirmation_source='fresh user ON/clearance/hands-clear confirmation for unique request')
        save_json(service_folder/'config.used.json', config)
        token = secrets.token_hex(24)
        server = make_http_server(service, token, 8767)
        service.on_close = server.shutdown
        save_json(service_folder/'service.json', dict(host='127.0.0.1', port=server.server_port,
            token=token, session=str(service_folder), started_at=utc_now()))
        print(json.dumps(dict(state='READY_SAME_PREPARED_BLE_LINK', session=str(service_folder))), flush=True)
        server.serve_forever(poll_interval=.2)
    finally:
        if server is not None:
            server.server_close()
        if service is not None:
            with service.operation_lock:
                service.close()
        else:
            prepared_session.close()
        lock.unlink(missing_ok=True)
        save('handoff_closure.json', dict(ble_disconnected=True, finished_at=utc_now()))


result=dict(mouse='B',motion='long_r90',started_at=utc_now(),execution_commands_sent=0,
            straight_settings_write_commands_sent=0,turn_settings_write_commands_sent=0)
session=LiveSession(ROOT,FOLDER/'transport')
try:
    session.prepare('C1:B5:60:87:91:73')
    with (FOLDER/'shell.bin').open('xb') as output,(FOLDER/'events.jsonl').open('x',encoding='utf-8') as events:
        def event(name,**fields):
            events.write(json.dumps(dict(at=utc_now(),event=name,**fields))+'\n');events.flush()
            if name=='battery':
                result['battery']=fields
        shell=Shell(session.connection,output,event,char_delay=.08)
        shell.send('shell');time.sleep(.5);shell.check_battery()
        result['camera_capability']=shell.query('debug camera show',lambda v:v.startswith('DEBUG_CAMERA_CAP '))
        for name,prefix in [('wall_ekf','WALL_EKF '),('wall_edge','WALL_EDGE '),('turn_ekf','TURN_EKF ')]:
            try:
                result[name]=shell.query(name+' show',lambda v:v.startswith(prefix),timeout=3)
            except TimeoutError:
                result[name]='未対応または応答なし。モードを推測しない。'
        def turn(query):
            raw=shell.query(query,lambda v:v.startswith('DEBUG_TURN_PARAM_X1000 long_r90 '))
            value=DebugGui._parse_device_settings(raw,'long_r90',query)
            if not value or value.get('error'):
                raise RuntimeError('Bの右90°設定を読み返せない')
            value.update(mouse='B',mouse_identity_source='user_supplied')
            settings_set_command(value)
            return value
        result['turn_before']=turn('debug turn long_r90 show')
        # RAMの既存値を保存してから1600テーブルを明示的に選ぶ。
        result['preset1600']=turn('debug turn long_r90 preset 1600')
        if result['preset1600']['preset_speed_mm_s']!=1600:
            raise RuntimeError('1600プリセットが選択されていない')
        requested_path=Path(sys.argv[2]) if len(sys.argv)>2 else BASE/'candidate.settings.json'
        requested=json.loads(requested_path.read_text(encoding='utf-8'))
        if requested['mouse']!='B' or requested['motion_name']!='long_r90' or requested['preset_speed_mm_s']!=1600:
            raise RuntimeError('対象機体・ターン・速度が一致しない')
        if len(sys.argv)>2 and requested['feedforward']!=result['preset1600']['feedforward']:
            raise RuntimeError('前回候補と今回のB実機FFが異なる。比較条件を変えずに中止。')
        requested['feedforward']=copy.deepcopy(result['preset1600']['feedforward'])
        requested['source']='今回のB1600実機FFを保持し、指定した候補をRAMへ設定する。未実走。'
        result['requested_settings_file']=str(requested_path.resolve())
        save('requested.settings.json',requested)
        straight=parse_straight(shell.query('debug straight show',lambda v:v.startswith('DEBUG_PARAM straight ')))
        result['straight_before']=straight
        result['restore_straight_command']=command(straight)
        wanted=dict(straight,acc=20.)
        if straight['acc']!=20.:
            result['straight_settings_write_commands_sent']=1
            shell.send(command(wanted));shell.wait_line(lambda v:v=='DEBUG_PARAM_SET_DONE',timeout=8)
        after=parse_straight(shell.query('debug straight show',lambda v:v.startswith('DEBUG_PARAM straight ')))
        if any(abs(after[k]-wanted[k])>1e-6 for k in wanted):
            raise RuntimeError('直進の読み返し不一致。再送しない。')
        result['straight_after']=after
        if setting_differences(result['preset1600'],requested):
            result['turn_settings_write_commands_sent']=1
            shell.send(settings_set_command(requested))
            shell.wait_line(lambda v:v=='DEBUG_TURN_SET_DONE',timeout=8)
        actual=turn('debug turn long_r90 show')
        differences=setting_differences(actual,requested,quantized=True)
        if differences:
            raise RuntimeError('ターン読み返し不一致: '+','.join(differences))
        save('actual.settings.json',actual)
        result.update(state='PREPARED_VERIFIED_NO_EXECUTION',turn_actual=actual)
except Exception as error:
    result.update(state='PREPARE_FAILED',error=type(error).__name__+': '+str(error))
finally:
    handoff = result.get('state') == 'PREPARED_VERIFIED_NO_EXECUTION' and len(sys.argv)>3
    if not handoff:
        session.close()
    result.update(ble_disconnected=not handoff, finished_at=utc_now(), same_link_handoff_requested=handoff)
    save('result.json',result)
    print(json.dumps({key:result.get(key) for key in ('state','battery','error','same_link_handoff_requested')},ensure_ascii=False),flush=True)
if result['state']!='PREPARED_VERIFIED_NO_EXECUTION':
    raise SystemExit(1)
if handoff:
    try:
        serve_prepared_session(session, BASE/sys.argv[3])
    finally:
        session.close()
