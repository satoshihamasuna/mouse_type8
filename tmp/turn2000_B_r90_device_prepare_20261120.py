"""B2000右90°の実機値を読み、RAMの単体候補を照合する。exeは送らない。"""
from pathlib import Path
import copy
import json
import math
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'.tools/ble_python311'),str(ROOT/'tools')]
from myshell_debug import DebugGui, PARAM_RE
from pixel_camera_tuning import LiveSession, settings_set_command, setting_differences
from pixel_camera_turn_capture import Shell, utc_now

BASE=ROOT/'tools/logs/pixel_camera/turn2000_B_r90_20261120_v1'
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
        # モード選択側に未完の受信行が残っていても、空改行で区切る。走行はしない。
        shell.send('');time.sleep(.2)
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
        if result['turn_before']['preset_speed_mm_s'] not in (700, 2000):
            raise RuntimeError('別条件のRAM設定を検出。今回の2000調整と他の実機操作が重複していないか確認する。preset/set/exeは送らない。')
        # RAMの既存値を保存してから2000テーブルを明示的に選ぶ。
        result['preset2000']=turn('debug turn long_r90 preset 2000')
        if result['preset2000']['preset_speed_mm_s']!=2000:
            raise RuntimeError('2000プリセットが選択されていない')
        requested_path=Path(sys.argv[2]) if len(sys.argv)>2 else BASE/'candidate.settings.json'
        requested=json.loads(requested_path.read_text(encoding='utf-8'))
        if requested['mouse']!='B' or requested['motion_name']!='long_r90' or requested['preset_speed_mm_s']!=2000:
            raise RuntimeError('対象機体・ターン・速度が一致しない')
        if len(sys.argv)>2 and requested['feedforward']!=result['preset2000']['feedforward']:
            raise RuntimeError('前回候補と今回のB実機FFが異なる。比較条件を変えずに中止。')
        requested['feedforward']=copy.deepcopy(result['preset2000']['feedforward'])
        requested['source']='今回のB2000実機FFを保持し、指定した候補をRAMへ設定する。未実走。'
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
        if setting_differences(result['preset2000'],requested):
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
    session.close();result.update(ble_disconnected=True,finished_at=utc_now());save('result.json',result)
    print(json.dumps(result,ensure_ascii=False),flush=True)
if result['state']!='PREPARED_VERIFIED_NO_EXECUTION':
    raise SystemExit(1)
