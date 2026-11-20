"""CodexからPixel撮影付き単体ターンを調整するローカルstdioサーバー。"""
from __future__ import annotations

import argparse
from contextlib import redirect_stdout
from hashlib import sha256
import json
import math
import os
from pathlib import Path
import site
import subprocess
import sys
import threading
from typing import Annotated, Any

from pixel_camera_tuning_cli import ROOT, TuningService, load_config, request_id, save_json
from pixel_camera_tuning import LiveSession, analysis_python, analyze_trial, hidden_process
from pixel_camera_turn_capture import utc_now

site.addsitedir(str(ROOT / '.tools/mouse-tuning-mcp'))
from mcp.server.fastmcp import FastMCP
from mcp.types import ToolAnnotations
from pydantic import BaseModel, Field, FiniteFloat, StrictBool

SESSION = ROOT / 'tools/logs/pixel_camera/mcp_tuning'
POSITIVE = Annotated[float, Field(gt=0, allow_inf_nan=False)]
BATTERY_PERCENT = Annotated[int, Field(ge=0, le=100, strict=True)]


class Placement(BaseModel):
    """盤面の左上が原点。右がX正・下がY正、右0°・下90°。"""
    x_mm: FiniteFloat
    y_mm: FiniteFloat
    heading_deg: FiniteFloat
    position_tolerance_mm: POSITIVE
    heading_tolerance_deg: POSITIVE


def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def static_check(camera, service, folder, placement):
    from pixel_camera_capture import record_trial

    manifest, recording = record_trial(camera, ROOT, seconds=3, mouse=service.config['mouse'],
        label=folder.name + '_static', camera_settings=service.camera_settings, capture_profile='tuning')
    if recording['state'] != 'PASSED':
        raise RuntimeError(f'静止撮影に失敗しました: {manifest}')
    plan = folder / 'static_plan.json'
    result = folder / 'static_check.json'
    save_json(plan, dict(video=recording['video_file'], camera_manifest=str(manifest), placement=placement,
                         reference=service.config['reference'], geometry=service.config['geometry']))
    with (folder / 'static.stdout.txt').open('x', encoding='utf-8') as output:
        subprocess.run([str(analysis_python()), str(ROOT / 'tools/analysis/pixel_camera_static_pose.py'),
                        str(plan), str(result)], stdout=output, stderr=subprocess.STDOUT,
                        env=dict(os.environ, PYTHONUTF8='1'), creationflags=hidden_process(),
                        check=True, timeout=120)
    result_value = read_json(result)
    if result_value['passed'] is not True:
        raise RuntimeError(f'開始位置または向きを照合できません: {result}')
    return result_value


def verify_acceleration(shell, motion, expected):
    from myshell_debug import PARAM_RE

    query = f'debug {motion} show'
    predicate = lambda line: line.startswith(f'DEBUG_PARAM {motion} ')

    def read():
        line = shell.query(query, predicate)
        match = PARAM_RE.search(line)
        if match is None:
            raise RuntimeError(f'{motion}の実機設定を読み取れません')
        settings = {key: (int(value) if key in ('suction', 'duty') else float(value))
                    for key, value in match.groupdict().items()}
        if not all(map(math.isfinite, settings.values())):
            raise RuntimeError(f'{motion}の実機設定に非有限値があります')
        return settings

    before = read()
    desired = dict(before, acc=expected)
    if before == desired:
        return dict(before=before, verified=before, write_count=0)
    shell.send(f'debug {motion} set ' + ' '.join(f'{value:.7g}' for value in desired.values()))
    shell.wait_line(lambda line: line == 'DEBUG_PARAM_SET_DONE', timeout=8)
    verified = read()
    if verified != desired:
        raise RuntimeError(f'{motion}の実機設定の読み戻しが一致しません')
    return dict(before=before, verified=verified, write_count=1)


class McpTuningService(TuningService):
    """CLIと同じ排他・永続要求IDの下で、接続から切断までを1件として行う。"""

    def capture_service(self, config, session=None):
        return TuningService(self.folder, config, session, confirmation_source='explicit MCP operator_ready')

    def analysis_marker(self, trial):
        key = sha256(os.path.normcase(str(trial)).encode('utf-8')).hexdigest()
        return self.folder / 'analysis' / (key + '.json')

    def operator_record(self, identifier):
        return self.folder / 'operator_results' / (request_id(identifier) + '.json')

    def prepare(self, config_path, settings_path, placement, diagonal_acceleration, straight_acceleration=None):
        config = load_config(Path(config_path))
        if not isinstance(config.get('address'), str) or not config['address'].strip():
            raise ValueError('configにHY0020のBLEアドレスを指定してください')
        settings = read_json(settings_path)
        settings = settings.get('actual_settings', settings)
        device = self.capture_service(config)
        device.trial_request(settings, True)
        return dict(config=config, settings=settings, placement=placement,
                    post_diagonal_accel_m_s2=diagonal_acceleration,
                    post_straight_accel_m_s2=straight_acceleration, defer_analysis=True)

    def validate(self, action, payload):
        if action != 'capture':
            raise ValueError('MCPの実機操作は単体1回の走行です')
        if payload.get('operator_ready') is not True:
            raise ValueError('今回の配置・走行範囲・両方ON・手離しを確認してください')

    def perform(self, action, payload, identifier, progress):
        config, settings = dict(payload['config']), payload['settings']
        folder = self.folder / ('prepare_' + identifier)
        session = LiveSession(ROOT, self.folder / ('transport_' + identifier),
                              stream_period_ms=config.get('stream_period_ms', 5))
        result = dict(capture_started=False, execution_write_attempted=False, preparation_folder=str(folder))
        # MCPのstdoutにはSDKが書くJSON-RPCメッセージだけを流す。
        with redirect_stdout(sys.stderr):
            try:
                folder.mkdir()
                for key in ('reference', 'geometry'):
                    source = folder / (key + '.used.json')
                    save_json(source, read_json(config[key]))
                    config[key] = str(source)
                device = self.capture_service(config, session)
                progress('camera_check', {})
                session.prepare_camera()
                progress('static_check', {})
                result['static_check'] = static_check(session.camera, device, folder, payload['placement'])
                progress('connecting', {})
                session.prepare(config['address'])
                progress('device_check', {})
                with device.shell(folder) as shell:
                    motion, preset = settings['motion_name'], settings['preset_speed_mm_s']
                    if device.read_settings(shell, motion)['preset_speed_mm_s'] != preset:
                        device.read_settings(shell, motion, preset)
                    for segment in ('diagonal', 'straight'):
                        runtime = payload.get(f'post_{segment}_accel_m_s2')
                        if runtime is not None:
                            result[f'{segment}_acceleration'] = verify_acceleration(shell, segment, runtime)
                save_json(folder / 'device_check.json', result)
                result.update(capture_started=True, execution_write_attempted=None)
                result.update(device.perform(action, payload, identifier, progress))
                result['execution_write_attempted'] = result['capture'].get('execution_write_attempted')
                result['state'] = result['capture']['state']
            except Exception as error:
                result.update(state='FAILED', error=f'{type(error).__name__}: {error}')
            finally:
                try:
                    session.close()
                    result['ble_disconnected'] = True
                except Exception as error:
                    result.update(ble_disconnected=False, close_error=str(error))
        result['operator_next_step'] = '走行を再送せず、実機が停止していることを確認し、機体とHY0020を両方OFFにしてください'
        return result


def postprocess(trial, marker):
    folder = trial.parent
    result = dict(state='RUNNING', trial_json=str(trial), reports={}, errors={},
                  analysis_json=str(marker), parameter_adoption_allowed=False)
    save_json(marker, result)
    for stride, name in ((4, 'quick'), (1, 'full')):
        summary = folder / name / 'summary.json'
        try:
            if not summary.is_file():
                result['reports'][name] = dict(returncode=analyze_trial(folder, stride))
            if summary.is_file():
                result['reports'].setdefault(name, {})['summary'] = read_json(summary)
                result['reports'][name]['summary_json'] = str(summary)
        except Exception as error:
            result['errors'][name] = str(error)
    result['state'] = 'ARCHIVING'
    save_json(marker, result)
    try:
        from pixel_camera_archive import archive_for_trial, archive_trial, verify_archive, write_index

        archive = archive_for_trial(trial)
        if archive is None:
            archive = archive_trial(trial)
            write_index(archive.parent)
        else:
            verify_archive(archive)
        result['archive'] = str(archive)
    except Exception as error:
        result['errors']['archive'] = str(error)
    result['state'] = 'REVIEW_REQUIRED'
    if result['errors'] or any(item.get('returncode', 0) or not item.get('summary', {}).get('auto_stop_confirmed')
                               for item in result['reports'].values()):
        result['state'] = 'ANALYSIS_REVIEW_REQUIRED'
    save_json(marker, result)


def create_server(service):
    server = FastMCP('mouse-turn-tuning', instructions=(
        'Pixel撮影付きマイクロマウス単体ターン調整。'
        '機体の電源ONを依頼する前にcheck_camera_healthで端末の残量・給電を確認する。'
        'run_onceには今回の操作者確認が必要。終了・失敗時は停止と両方OFFを操作者に確認し、'
        '同じ走行要求を再送せず、保存済み資料は電源OFFの確認と並行して解析できる。'))

    @server.tool(annotations=ToolAnnotations(readOnlyHint=True, openWorldHint=False))
    def check_camera_health(adb: str | None = None, serial: str | None = None,
                            min_level: BATTERY_PERCENT = 20) -> dict[str, Any]:
        """電源ONの依頼前にカメラ端末の残量・給電状態を読む。録画・設定変更・走行はしない。

        min_levelは撮影時間に合わせて指定する運用上の残量余裕で、端末性能の保証値ではない。
        未検出・複数端末・残量不明は合格にしない。prepare_trialは従来どおりオフライン。
        """
        from pixel_camera_health import inspect_camera_health

        result = inspect_camera_health(adb=adb, serial=serial, min_level=min_level)
        return dict(result, checked_at=utc_now())

    @server.tool(annotations=ToolAnnotations(readOnlyHint=True))
    def prepare_trial(config_path: str, settings_path: str, placement: Placement,
                      post_diagonal_accel_m_s2: POSITIVE | None = None,
                      post_straight_accel_m_s2: POSITIVE | None = None) -> dict[str, Any]:
        """設定JSONと盤面参照を検証し、対象機体、ターン、候補値と開始位置を返す。実機に接続しない。"""
        plan = service.prepare(config_path, settings_path, placement.model_dump(),
                               post_diagonal_accel_m_s2, post_straight_accel_m_s2)
        return dict(state='VALIDATED', settings=plan['settings'], placement=plan['placement'],
                    post_diagonal_accel_m_s2=post_diagonal_accel_m_s2,
                    post_straight_accel_m_s2=post_straight_accel_m_s2)

    @server.tool(annotations=ToolAnnotations(destructiveHint=True, idempotentHint=False))
    def run_once(request_id: str, config_path: str, settings_path: str, placement: Placement,
                 operator_ready: StrictBool, post_diagonal_accel_m_s2: POSITIVE | None = None,
                 post_straight_accel_m_s2: POSITIVE | None = None) -> dict[str, Any]:
        """今回の配置・経路の干渉なし・両方ON・手離しを操作者が伝えた場合に単体1回を受付。

        要求IDを指定し、新しい静止撮影と実機照合を通過したときだけ走行する。
        応答が途切れても再送せず、元のIDをget_trial_statusに渡す。
        """
        plan = service.prepare(config_path, settings_path, placement.model_dump(),
                               post_diagonal_accel_m_s2, post_straight_accel_m_s2)
        return service.submit('capture', dict(plan, operator_ready=operator_ready), request_id)

    @server.tool(annotations=ToolAnnotations(readOnlyHint=True))
    def get_trial_status(trial_id: str | None = None, trial_json: str | None = None) -> dict[str, Any]:
        """要求IDの状態、動画・ログ、解析・操作者確認を返す。既存CLIの解析はtrial_jsonで照会できる。"""
        state = {}
        if trial_id is not None:
            state.update(service.status(trial_id))
            state.pop('request', None)
            trial_json = state.get('result', {}).get('trial_json', trial_json)
            record = service.operator_record(trial_id)
            if record.is_file():
                state['operator_result'] = read_json(record)
        elif trial_json is None:
            return dict(busy=service.operation_lock.locked(), session=str(service.folder))
        if trial_json is not None:
            trial = Path(trial_json).resolve()
            marker = service.analysis_marker(trial)
            state['trial_json'] = str(trial)
            if trial_id is None:
                state['state'] = read_json(trial)['state']
            with service.state_lock:
                if marker.is_file():
                    state['analysis'] = read_json(marker)
        return state

    @server.tool(annotations=ToolAnnotations(openWorldHint=False, destructiveHint=False, idempotentHint=True))
    def analyze_saved_trial(trial_json: str) -> dict[str, Any]:
        """取得済みtrial.jsonから速報・全フレーム解析と一式保存をバックグラウンドで開始する。"""
        trial = Path(trial_json).resolve()
        value = read_json(trial)
        if trial.name != 'trial.json' or value.get('state') != 'CAPTURED_REVIEW_REQUIRED':
            raise ValueError('走行・取得を完了した単体のtrial.jsonを指定してください')
        marker = service.analysis_marker(trial)
        marker.parent.mkdir(exist_ok=True)
        pending = dict(state='ACCEPTED', trial_json=str(trial), analysis_json=str(marker))
        with service.state_lock:
            try:
                with marker.open('x', encoding='utf-8') as output:
                    json.dump(pending, output, ensure_ascii=False)
            except FileExistsError:
                return read_json(marker)
        threading.Thread(target=postprocess, args=(trial, marker), daemon=False).start()
        return pending

    @server.tool(annotations=ToolAnnotations(openWorldHint=False, destructiveHint=False))
    def record_operator_result(trial_id: str, both_powered_off: StrictBool,
                               contact_detected: StrictBool) -> dict[str, Any]:
        """走行後に操作者から聞いた機体・HY0020両方の電源OFFと穴・壁への接触有無を保存する。"""
        job = service.status(request_id(trial_id))
        if job['state'] not in ('DONE', 'FAILED'):
            raise ValueError('走行要求の完了後に確認を保存してください')
        record = service.operator_record(trial_id)
        record.parent.mkdir(exist_ok=True)
        value = dict(request_id=trial_id, both_powered_off=both_powered_off, contact_detected=contact_detected,
                     source='operator_report', confirmed_at=utc_now())
        with service.state_lock:
            save_json(record, value)
        return value

    return server


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, default=SESSION)
    args = parser.parse_args()
    os.chdir(ROOT)
    service = McpTuningService(args.session, {})
    lock = service.folder / 'service.lock'
    with lock.open('x', encoding='utf-8') as stream:
        stream.write(utc_now())
    try:
        create_server(service).run(transport='stdio')
    finally:
        with service.operation_lock:
            lock.unlink(missing_ok=True)


if __name__ == '__main__':
    main()
