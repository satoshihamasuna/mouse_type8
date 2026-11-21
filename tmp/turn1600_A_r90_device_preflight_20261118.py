"""機体A右90°の実機値を読み取る。set/exeは送らず接続を閉じる。"""
from pathlib import Path
import json
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
sys.path.insert(0, str(ROOT / '.tools' / f'ble_python{sys.version_info.major}{sys.version_info.minor}'))
from pixel_camera_tuning import LiveSession, settings_set_command
from pixel_camera_turn_capture import Shell, debug_motion_prefix, device_motion_name, utc_now
from myshell_debug import DebugGui

BASE = ROOT / 'tools/logs/pixel_camera/turn1600_A_r90_20261118_frame18_v1'
FOLDER = BASE / (sys.argv[1] if len(sys.argv) > 1 else 'device_preflight01')
FOLDER.mkdir(exist_ok=False)
session = LiveSession(ROOT, FOLDER / 'transport')
result = dict(mouse='A', motion='long_r90', execution_commands_sent=0,
              settings_write_commands_sent=0, started_at=utc_now())
try:
    session.prepare('C1:B5:60:87:91:73')
    with (FOLDER / 'shell.bin').open('xb') as output, (FOLDER / 'events.jsonl').open('x', encoding='utf-8') as events:
        def event(name, **fields):
            events.write(json.dumps(dict(at=utc_now(), event=name, **fields)) + '\n')
            events.flush()
            if name == 'battery':
                result['battery'] = fields
        shell = Shell(session.connection, output, event, char_delay=.08)
        shell.send('shell')
        time.sleep(.5)
        shell.check_battery()
        result['camera_capability'] = shell.query('debug camera show', lambda line: line.startswith('DEBUG_CAMERA_CAP '))
        result['wall_ekf'] = shell.query('wall_ekf show', lambda line: line.startswith('WALL_EKF '))
        result['wall_edge'] = shell.query('wall_edge show', lambda line: line.startswith('WALL_EDGE '))
        command = debug_motion_prefix('long_r90') + ' show'
        line = shell.query(command, lambda value: value.startswith('DEBUG_TURN_PARAM_X1000 ' + device_motion_name('long_r90') + ' '))
        settings = DebugGui._parse_device_settings(line, 'long_r90', command)
        if not settings or settings.get('error'):
            raise RuntimeError('実機設定を取得できません')
        settings.update(mouse='A', mouse_identity_source='user_supplied')
        settings_set_command(settings)
        result['settings'] = settings
        result['state'] = 'READ_COMPLETE'
except Exception as error:
    result.update(state='READ_FAILED', error=type(error).__name__ + ': ' + str(error))
finally:
    session.close()
    result.update(ble_disconnected=True, finished_at=utc_now())
    (FOLDER / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
