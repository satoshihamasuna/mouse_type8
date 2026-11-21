"""機体A右180度の実機値を照会し、試験用出口減速を20m/s²へ設定。走行指令なし。"""
from pathlib import Path
import json
import math
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / '.tools/ble_python311'), str(ROOT / 'tools')]
from myshell_debug import DebugGui, PARAM_RE
from pixel_camera_tuning import LiveSession, settings_set_command
from pixel_camera_turn_capture import Shell, utc_now

BASE = ROOT / 'tools/logs/pixel_camera/turn1600_A_r180_20261118_v1'
FOLDER = BASE / sys.argv[1]
FOLDER.mkdir(exist_ok=False)
KEYS = ('distance', 'acc', 'max_velo', 'end_velo', 'sp_kp', 'sp_ki', 'sp_kd',
        'om_kp', 'om_ki', 'om_kd', 'ff_sp_velo', 'ff_sp_accel', 'ff_sp_bias',
        'ff_om_velo', 'ff_om_accel', 'ff_om_decel', 'ff_om_bias')


def parse_straight(line):
    match = PARAM_RE.search(line)
    if not match:
        raise ValueError('直進試験の実機値を取得できません')
    value = {key: float(match[key]) for key in KEYS}
    value.update(suction=int(match['suction']), duty=int(match['duty']))
    if not all(math.isfinite(number) for number in value.values()):
        raise ValueError('直進試験の実機値に非有限値があります')
    return value


def command(value):
    return 'debug straight set ' + ' '.join(f'{value[key]:.7g}' for key in KEYS) \
        + f" {value['suction']} {value['duty']}"


session = LiveSession(ROOT, FOLDER / 'transport')
result = dict(mouse='A', motion='long_r180', execution_commands_sent=0,
    straight_settings_write_commands_sent=0, turn_settings_write_commands_sent=0,
    started_at=utc_now(), requested_post_straight_accel_m_s2=20.)
try:
    session.prepare('C1:B5:60:87:91:73')
    with (FOLDER / 'shell.bin').open('xb') as output, \
            (FOLDER / 'events.jsonl').open('x', encoding='utf-8') as events:
        def event(name, **fields):
            events.write(json.dumps(dict(at=utc_now(), event=name, **fields)) + '\n')
            events.flush()
            if name == 'battery':
                result['battery'] = fields
        shell = Shell(session.connection, output, event, char_delay=.08)
        shell.send('shell')
        time.sleep(.5)
        shell.check_battery()
        result['camera_capability'] = shell.query('debug camera show',
            lambda line: line.startswith('DEBUG_CAMERA_CAP '))
        result['wall_ekf'] = shell.query('wall_ekf show',
            lambda line: line.startswith('WALL_EKF '))
        result['wall_edge'] = shell.query('wall_edge show',
            lambda line: line.startswith('WALL_EDGE '))
        try:
            result['turn_ekf'] = shell.query('turn_ekf show',
                lambda line: line.startswith('TURN_EKF '), timeout=2)
        except TimeoutError:
            result['turn_ekf'] = 'No TURN_EKF response in current firmware'
        query = 'debug turn long_r180 show'
        line = shell.query(query,
            lambda value: value.startswith('DEBUG_TURN_PARAM_X1000 long_r180 '))
        settings = DebugGui._parse_device_settings(line, 'long_r180', query)
        if not settings or settings.get('error'):
            raise RuntimeError('右180度の実機値を取得できません')
        settings.update(mouse='A', mouse_identity_source='user_supplied')
        settings_set_command(settings)
        result['turn_settings_before'] = settings
        before_line = shell.query('debug straight show',
            lambda line: line.startswith('DEBUG_PARAM straight '))
        before = parse_straight(before_line)
        wanted = dict(before, acc=20.)
        assert {key for key in before if before[key] != wanted[key]} <= {'acc'}
        result.update(straight_settings_before=before,
            straight_settings_before_raw=before_line,
            restore_straight_settings_command=command(before))
        if before['acc'] != wanted['acc']:
            result['straight_settings_write_commands_sent'] = 1
            shell.send(command(wanted))
            shell.wait_line(lambda line: line == 'DEBUG_PARAM_SET_DONE', timeout=8)
        after_line = shell.query('debug straight show',
            lambda line: line.startswith('DEBUG_PARAM straight '))
        after = parse_straight(after_line)
        if any(abs(after[key] - wanted[key]) > 1e-6 for key in wanted):
            raise RuntimeError('直進試験の出口減速条件の読み戻しが一致しません。再送しません。')
        result.update(straight_settings_after=after,
            straight_settings_after_raw=after_line,
            straight_other_fields_preserved=True,
            post_condition_applied_and_verified=True, state='PREPARED_NO_EXECUTION')
except Exception as error:
    result.update(state='PREPARE_FAILED', error=type(error).__name__ + ': ' + str(error))
finally:
    session.close()
    result.update(ble_disconnected=True, finished_at=utc_now())
    with (FOLDER / 'result.json').open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
    print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
if result['state'] != 'PREPARED_NO_EXECUTION':
    raise SystemExit(1)
