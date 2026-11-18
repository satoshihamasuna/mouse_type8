"""既存BLE探索とPixel録画の順序制御。GUI以外からも状態を検証できる。"""
from __future__ import annotations
import asyncio
import csv
from datetime import datetime
import html
import json
import math
from pathlib import Path
import subprocess
import sys
import threading
import time

TOOLS = Path(__file__).resolve().parents[1]
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

from pixel_camera_profiles import configure_camera


def default_maze_reference(directory=None):
    directory = Path(directory) if directory is not None else TOOLS / 'logs/pixel_camera'
    current = directory / 'board_9x9_current_reference.json'
    return current if current.is_file() else directory / 'board_9x9_reference.json'


def camera_client():
    from pixel_camera_capture import camera_workspace
    workspace = camera_workspace()
    if str(workspace / 'python') not in sys.path:
        sys.path.insert(0, str(workspace / 'python'))
    from pixel_camera import PixelCameraClient
    # The camera GUI establishes USB reverse. Never start another BLE gateway on 8765.
    return PixelCameraClient(connection='wifi', host='127.0.0.1', port=8765, auto_start=False, timeout=4)


def owned(status, recording_id, owner):
    d = status.get('device', {})
    return (status.get('connected') is True and d.get('phase') == 'RECORDING'
            and d.get('recording_id') == recording_id and d.get('owner') == owner)


def ready_for_gesture(snapshot, status, recording_id, owner, cancelled=False):
    age = snapshot.get('sample_age_seconds')
    return (not cancelled and owned(status, recording_id, owner)
            and snapshot.get('state') == 'waiting_sensor' and snapshot.get('streaming') is True
            and snapshot.get('rows', 0) > 0 and isinstance(age, (float, int)) and 0 <= age < .5)


def save_json(path, value):
    # Same-directory replacement avoids partial JSON reads by the review command.
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    temporary.replace(path)


def load_camera_preferences(path=None):
    """撮影比較で保存したローカル値をGUIへ表示する。ここでは端末へ送信しない。"""
    path = Path(path) if path is not None else TOOLS / 'logs/pixel_camera/explore_capture_preferences.json'
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
        if not isinstance(value,dict) or value.get('schema') != 1:
            return {}
        ev, torch = value['camera_settings']['ev'], value['camera_settings']['torch']
        if isinstance(ev, bool) or not isinstance(ev, (int,float)) or not math.isfinite(ev) or not -4 <= ev <= 4:
            return {}
        return dict(ev=ev, torch=torch) if isinstance(torch,bool) else {}
    except (OSError, ValueError, KeyError, TypeError):
        return {}


class ExploreCameraRun:
    def __init__(self, session, emit, client_factory=camera_client, finalizer=None, interval=.2, analysis_config=None,
                 camera_settings=None):
        self.session, self.emit, self.client_factory = session, emit, client_factory
        self.finalizer = finalizer or finalize_trial
        self.interval = interval
        self.thread = None
        self.active = False
        self.cancelled = threading.Event()
        self.result_received = threading.Event()
        self.lock = threading.RLock()
        self.camera_lock = threading.Lock()
        self.client = self.recording_id = self.folder = None
        self.started = self.stop_sent = self.ready = False
        self.latest = {}
        self.result = None
        self.clock_samples = []
        self.motion_anchor = None
        self.ledger = None
        self.analysis_config = analysis_config
        self.camera_settings = camera_settings

    def event(self, name, **value):
        with self.lock:
            if self.ledger:
                self.ledger.write(json.dumps(dict(event=name, monotonic=time.monotonic(),
                    at=datetime.now().astimezone().isoformat(), **value), ensure_ascii=False) + '\n')
                self.ledger.flush()

    def observe(self, kind, data):
        if not self.active:
            return
        with self.lock:
            if kind == 'telemetry':
                self.latest = data
                values = data.get('values', {})
                if self.started and self.motion_anchor is None and (
                        abs(values.get('ideal.velo', 0)) > .005 or abs(values.get('ideal.rad_velo', 0)) > .05):
                    self.motion_anchor = dict(host_monotonic=time.monotonic(), time_ms=data['time_ms'])
                    self.event('first_motion_observed', **self.motion_anchor)
            elif self.started and kind in ('result', 'run_error', 'error'):
                self.result = dict(kind=kind, data=data)
                self.result_received.set()
                self.event('ble_' + kind, data=data)
            elif kind in ('state', 'phase', 'command_commit', 'capability', 'battery'):
                self.event('ble_' + kind, data=data)
            if kind == 'state' and data != 'waiting_sensor':
                self.ready = False
                self.emit('camera_ready', False)

    def cancel(self):
        self.cancelled.set()
        self.ready = False
        self.emit('camera_ready', False)
        if self.started and not self.stop_sent:
            self.stop_sent = True
            self.session.action('stop')
            self.event('mouse_stop_requested')

    def start(self, options):
        if self.active:
            raise ValueError('録画付き走行は既に準備中です')
        if not self.session.ready(options):
            raise ValueError('BLE接続・電圧・探索対応を先に確認してください')
        if not 1 <= options.limit_seconds <= 225:
            raise ValueError('録画付き走行の走行制限は1～225秒です（録画に75秒の準備・待機余白）')
        options.command()
        self.active = True
        self.cancelled.clear(); self.result_received.clear()
        self.started = self.stop_sent = self.ready = False
        self.result = self.motion_anchor = None
        self.clock_samples = []
        self.thread = threading.Thread(target=self._work, args=(options,), daemon=True, name='Explore camera')
        self.thread.start()

    def status(self):
        before = time.monotonic()
        with self.camera_lock:
            value = self.client.get_status()
        after = time.monotonic()
        elapsed = value.get('device', {}).get('elapsed_seconds')
        if self.recording_id and isinstance(elapsed, (int, float)) and math.isfinite(elapsed):
            clock = dict(pc_before=before, pc_after=after, camera_elapsed_s=elapsed,
                         recording_id=value.get('device', {}).get('recording_id'))
            self.clock_samples.append(clock)
            self.event('camera_clock', **clock)
        return value

    def check_before_commit(self):
        if self.cancelled.is_set():
            raise RuntimeError('開始を取り消しました')
        value = self.status()
        if not owned(value, self.recording_id, self.client.client_id):
            raise RuntimeError('録画ID・所有者・録画中状態が一致しません')
        elapsed = value['device'].get('elapsed_seconds')
        if (not isinstance(elapsed, (int, float)) or not math.isfinite(elapsed)
                or not 0 <= elapsed < self.duration - self.options.limit_seconds - 5):
            raise RuntimeError('探索全体を撮影する残り時間が不足しています')
        snap = self.session.snapshot()
        age = snap.get('sample_age_seconds')
        if not isinstance(age, (int, float)) or age >= .5 or not snap.get('rows') or not snap.get('streaming'):
            raise RuntimeError('新しいBLEログを確認できません')
        if snap.get('crc_errors', 0) != self.baseline_crc or snap.get('lost', 0) != self.baseline_lost:
            raise RuntimeError('準備中にBLEログの欠落・CRCエラーが発生しました')
        self.event('camera_commit_guard_passed', mcu_time_ms=snap.get('time_ms'), sequence=snap.get('sequence'))

    async def guard(self):
        await asyncio.to_thread(self.check_before_commit)

    def _work(self, options):
        self.options = options
        self.duration = options.limit_seconds + 75
        stop_attempted = False
        trial = dict(schema=1, options=vars(options), recording_limit_seconds=self.duration,
                     status='PREPARING', started_at=datetime.now().astimezone().isoformat(),
                     sync_method='PC receive/camera elapsed initial estimate; manual video alignment required')
        trial['run_type'] = 'shortest' if hasattr(options, 'profile') else 'explore'
        trial['capture_profile'] = 'maze_9x9'
        trial['position_analysis'] = self.analysis_config
        try:
            self.folder = self.session.directory / ('camera_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
            self.folder.mkdir()
            self.ledger = (self.folder / 'events.jsonl').open('x', encoding='utf-8')
            if self.analysis_config and self.analysis_config.get('reference'):
                reference = json.loads(Path(self.analysis_config['reference']).read_text(encoding='utf-8'))
                snapshot = self.folder / 'marker_reference_used.json'
                save_json(snapshot, reference)
                self.analysis_config = dict(self.analysis_config, reference=str(snapshot.resolve()))
                trial['position_analysis'] = self.analysis_config
            save_json(self.folder / 'trial.json', trial)
            self.emit('camera_status', 'カメラを確認中（まだ手をかざさないでください）')
            self.client = self.client_factory()
            status = self.status()
            if not status.get('connected') or status.get('device', {}).get('phase') != 'IDLE':
                raise RuntimeError('PixelカメラをPCへ接続し、録画・転送後の待機状態にしてください')
            maximum = status['device'].get('max_recording_seconds', 30)
            if maximum < self.duration:
                raise RuntimeError(f'端末の録画上限は{maximum}秒です。探索対応のカメラアプリが必要です')
            trial['camera_configuration'] = configure_camera(self.client, self.camera_settings)
            save_json(self.folder / 'trial.json', trial)
            self.event('recording_start_attempt', seconds=self.duration)
            self.recording_id = self.client.start_recording(self.duration)  # Never retry this request.
            trial.update(recording_id=self.recording_id, camera_owner=self.client.client_id)
            save_json(self.folder / 'trial.json', trial)
            deadline = time.monotonic() + 20
            while True:
                status = self.status()
                if owned(status, self.recording_id, self.client.client_id):
                    break
                if self.cancelled.is_set() or time.monotonic() > deadline:
                    raise RuntimeError('録画中状態を確認できませんでした。探索は開始しません')
                time.sleep(self.interval)
            if self.cancelled.is_set():
                raise RuntimeError('開始を取り消しました')
            self.session.start_guard = self.guard
            snap = self.session.snapshot()
            self.baseline_crc, self.baseline_lost = snap.get('crc_errors', 0), snap.get('lost', 0)
            self.started = True
            self.event('exploration_start_queued', options=vars(options))
            self.session.action('start', options)
            self.emit('camera_status', '録画中・BLEログと手かざし待ちを確認中')
            prepared_at = time.monotonic(); waiting_at = None
            while not self.result_received.is_set():
                status = self.status()
                if not owned(status, self.recording_id, self.client.client_id):
                    raise RuntimeError('録画が終了または切断されたため探索停止を要求します')
                snap = self.session.snapshot()
                good = ready_for_gesture(snap, status, self.recording_id, self.client.client_id, self.cancelled.is_set())
                good = good and snap.get('crc_errors', 0) == self.baseline_crc and snap.get('lost', 0) == self.baseline_lost
                if good != self.ready:
                    self.ready = good; self.emit('camera_ready', good)
                    self.event('gesture_ok' if good else 'gesture_ok_revoked', mcu_time_ms=snap.get('time_ms'), sequence=snap.get('sequence'))
                if snap.get('state') == 'waiting_sensor':
                    waiting_at = waiting_at or time.monotonic()
                    if time.monotonic() - waiting_at > 30:
                        self.cancel(); self.emit('camera_status', '手かざし待ち30秒で中止・保存中')
                elif snap.get('state') == 'running':
                    self.emit('camera_status', '探索・撮影中')
                elif snap.get('state') == 'disconnected':
                    raise RuntimeError('BLE接続が切れました。録画資料を保持します')
                if snap.get('streaming') and (snap.get('sample_age_seconds') or 0) > 1:
                    raise RuntimeError('BLEログが1秒以上途絶えたため停止を要求します')
                if waiting_at is None and time.monotonic() - prepared_at > 25:
                    raise RuntimeError('実機の手かざし待ち応答を確認できません')
                if self.cancelled.is_set() and not self.stop_sent:
                    self.cancel()
                if self.cancelled.is_set() and time.monotonic() - prepared_at > self.duration:
                    raise RuntimeError('中止後の応答がありません')
                time.sleep(self.interval)
            trial['result'] = self.result
            if self.result['kind'] != 'result':
                raise RuntimeError('実機の探索終了を確認できません: ' + str(self.result['data']))
            self.emit('camera_ready', False)
            self.emit('camera_status', '走行終了・録画の後余白を保存中')
            time.sleep(1.2)
            status = self.status()
            if owned(status, self.recording_id, self.client.client_id):
                stop_attempted = True
                self.event('recording_stop_attempt')
                trial['stop_response'] = self.client.stop_recording()  # No retry if response is lost.
                trial['controlled_stop'] = True
            self.emit('camera_status', '動画を転送・照合中。走行終了後は機体・HY0020の電源をOFFにできます')
            result = self.client.wait_for_transfer(timeout=120, recording_id=self.recording_id)
            trial.update(status='TRANSFERRED', video_file=str(result.file_path), metadata_file=str(result.metadata_path),
                         sha256=result.sha256, size=result.size, source_deleted=result.source_deleted,
                         frame_count=result.frame_count, frame_count_source=result.frame_count_source,
                         ble_directory=str(self.session.directory), run_settings=str(self.session._run_settings_path),
                         clock_samples=self.clock_samples, motion_anchor=self.motion_anchor,
                         ble_quality=dict(lost=self.session.snapshot().get('lost',0)-self.baseline_lost,
                                          crc_errors=self.session.snapshot().get('crc_errors',0)-self.baseline_crc))
            save_json(self.folder / 'trial.json', trial)
            self.emit('camera_status', '保存完了・動画とログの比較画面を作成中（機体の電源は不要です）')
            self.finalizer(self.folder / 'trial.json')
            review = self.folder / 'review.html'
            if self.analysis_config:
                self.emit('camera_status', '位置・向き・旋回別の比較を解析中（機体の電源は不要です）')
                try:
                    review = analyze_trial(self.folder / 'trial.json', self.analysis_config)
                except Exception as error:
                    save_json(self.folder / 'position_analysis_error.json', dict(error=str(error), config=self.analysis_config))
                    self.emit('camera_status', '動画とログは保存済み。位置解析は要確認: ' + str(error))
            verified = json.loads((self.folder / 'trial.json').read_text(encoding='utf-8')).get('video_validation_passed', False)
            self.emit('camera_done', dict(trial=str(self.folder / 'trial.json'), review=str(review), verified=verified,
                                         position_available=review.parent != self.folder,
                                         position_requested=bool(self.analysis_config)))
        except Exception as error:
            trial.update(status='FAILED', error=str(error), recording_id=self.recording_id, stop_attempted=stop_attempted,
                         clock_samples=self.clock_samples, motion_anchor=self.motion_anchor)
            self.event('failure', detail=str(error))
            self.cancel()
            try:
                if self.client and not self.recording_id:
                    status = self.status()
                    device = status.get('device', {})
                    if (status.get('connected') and device.get('owner') == self.client.client_id
                            and device.get('phase') in ('PREPARING', 'RECORDING') and device.get('recording_id')):
                        self.recording_id = device['recording_id']
                        trial['recording_id'] = self.recording_id
                        self.event('unknown_start_recovered_by_owner', recording_id=self.recording_id)
                if self.client and self.recording_id and not stop_attempted:
                    status = self.status()
                    d = status.get('device', {})
                    if (status.get('connected') and d.get('recording_id') == self.recording_id
                            and d.get('owner') == self.client.client_id and d.get('phase') in ('PREPARING', 'RECORDING')):
                        stop_attempted = True
                        trial['stop_attempted'] = True
                        self.client.stop_recording()
                if self.client and self.recording_id:
                    result = self.client.wait_for_transfer(timeout=60, recording_id=self.recording_id)
                    trial.update(recovered_video_file=str(result.file_path), recovered_metadata_file=str(result.metadata_path))
            except Exception as recovery:
                trial['recovery_error'] = str(recovery)
            if self.folder:
                save_json(self.folder / 'trial.json', trial)
            self.emit('camera_failure', str(error))
        finally:
            self.ready = False; self.emit('camera_ready', False)
            self.session.start_guard = None
            with self.lock:
                if self.ledger:
                    self.ledger.close(); self.ledger = None
            self.active = False
            self.emit('camera_finished', None)


def finalize_trial(trial_path):
    from pixel_camera_capture import camera_workspace, camera_python
    subprocess.run([str(camera_python(camera_workspace())), str(Path(__file__).resolve()), '--verify', str(trial_path)],
                   check=True, creationflags=subprocess.CREATE_NO_WINDOW if sys.platform == 'win32' else 0)
    build_review(trial_path)


def analysis_python():
    """既存の解析環境を優先する。GUIから別の環境も選択できる。"""
    candidates = [Path('D:/venv/venv3-11/Scripts/python.exe'), Path(sys.executable)]
    return next((p for p in candidates if p.is_file()), Path(sys.executable))


def analyze_trial(trial_path, config):
    output = trial_path.parent / ('position_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    command = [config['python'], str(TOOLS / 'analysis/maze_video_pose.py'), '--trial', str(trial_path),
               '--reference', config['reference'], '--output', str(output), '--stride', str(config.get('stride', 4))]
    if config.get('cell_mm') is not None:
        command += ['--cell-mm', str(config['cell_mm'])]
    with (trial_path.parent / (output.name + '.log')).open('x',encoding='utf-8') as log:
        result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT,
            creationflags=subprocess.CREATE_NO_WINDOW if sys.platform == 'win32' else 0)
    if result.returncode:
        raise RuntimeError(f'位置解析を完了できませんでした: {output.name}.log を確認してください')
    return output / 'review.html'


def verify_video(trial_path):
    from pixel_camera_capture import camera_workspace
    sys.path.insert(0, str(camera_workspace() / 'python'))
    from pixel_camera.models import TransferResult
    from pixel_camera.stability import inspect_recording
    trial = json.loads(trial_path.read_text(encoding='utf-8'))
    result = TransferResult(recording_id=trial['recording_id'], file_path=Path(trial['video_file']),
        metadata_path=Path(trial['metadata_file']), sha256=trial['sha256'], size=trial['size'],
        frame_count=trial['frame_count'], frame_count_source=trial['frame_count_source'], source_deleted=trial['source_deleted'])
    inspection = inspect_recording(result, trial['recording_limit_seconds'])
    metadata = json.loads(result.metadata_path.read_text(encoding='utf-8'))
    trial['camera_capture_controls'] = {key:metadata.get(key) for key in
        ('ev_steps_requested', 'ev_step', 'ev_steps_result_values', 'torch_requested', 'flash_mode_result_values', 'focus_lock')}
    if (trial.get('controlled_stop') and trial.get('stop_response', {}).get('accepted') is True
            and metadata.get('stop_reason') == 'USER_OR_ACTIVITY_STOP'):
        window = metadata.get('recording_window', {})
        if (window.get('origin') == 'first_encoded_frame_pts' and window.get('timestamps_rescaled') is False
                and metadata.get('requested_duration_seconds') == trial['recording_limit_seconds']):
            inspection['fixed_duration_errors'] = list(inspection['errors'])
            replace = {'UNEXPECTED_STOP_REASON', 'FIRST_FRAME_DURATION_NOT_CONFIRMED', 'VIDEO_DURATION_MISMATCH', 'PTS_SPAN_MISMATCH'}
            inspection['errors'] = [e for e in inspection['errors'] if e not in replace]
            extracted = metadata.get('mp4', {}).get('sample_timing', {}).get('span_seconds')
            if not isinstance(extracted, (int, float)) or abs(extracted - inspection['pts_span_seconds']) > .00001:
                inspection['errors'].append('CONTROLLED_STOP_MP4_PTS_MISMATCH')
            inspection['passed'] = not inspection['errors']
    save_json(trial_path.parent / 'video_validation.json', inspection)
    trial['video_validation_passed'] = inspection['passed']
    save_json(trial_path, trial)


def build_review(trial_path):
    trial = json.loads(trial_path.read_text(encoding='utf-8'))
    settings = json.loads(Path(trial['run_settings']).read_text(encoding='utf-8'))
    source = Path(trial['ble_directory']) / 'telemetry.csv'
    lo, hi = settings['telemetry_start']['time_ms'], settings['telemetry_end']['time_ms']
    with source.open(encoding='utf-8') as f:
        rows = [r for r in csv.DictReader(f) if lo <= int(r['time_ms']) <= hi]
    moving = [r for r in rows if abs(float(r['ideal.velo'])) > .005 or abs(float(r['ideal.rad_velo'])) > .05]
    if not moving:
        raise RuntimeError('走行指令が記録されていません。動画・設定・イベントは保存されています')
    zero = int(moving[0]['time_ms'])
    anchor = trial.get('motion_anchor')
    clocks = [c for c in trial['clock_samples'] if c['recording_id'] == trial['recording_id']]
    offset = 0.
    if anchor and clocks:
        pc = anchor['host_monotonic'] + (zero - anchor['time_ms']) / 1000
        clock = min(clocks, key=lambda c: abs((c['pc_before'] + c['pc_after']) / 2 - pc))
        offset = max(0., clock['camera_elapsed_s'] + pc - (clock['pc_before'] + clock['pc_after']) / 2)
    fields = ['ideal.velo', 'ego.velo', 'ideal.rad_velo', 'ego.rad_velo', 'sen_l.distance', 'sen_r.distance']
    data = [[(int(r['time_ms']) - zero) / 1000] + [float(r[k]) for k in fields] for r in rows[::4]]
    assert all(math.isfinite(v) for row in data for v in row)
    save_json(trial_path.parent / 'sync_initial.json', dict(video_time_at_mcu_origin_s=offset, mcu_origin_ms=zero,
        source='PC receive/camera elapsed estimate, not hardware synchronization', manual_alignment_required=True))
    payload = json.dumps(dict(samples=data, offset=offset, zero=zero), allow_nan=False)
    video = html.escape(Path(trial['video_file']).as_uri(), quote=True)
    page = '''<!doctype html><meta charset="utf-8"><title>探索動画・BLEログ比較</title>
<style>body{font:16px sans-serif;margin:20px;background:#f2f5f8;color:#172331}video{width:min(100%,960px);max-height:58vh;background:black}canvas{width:100%;height:170px;background:white}button,input{font:inherit;padding:6px}p{max-width:1000px}.box{max-width:1100px}</style>
<div class="box"><h1>探索動画・BLEログ比較</h1><p>同期はPC受信時刻からの初期推定です。動画の走り始め・青LEDと波形を比較し、ずれを調整してください。元CSVは実MCU時刻のまま保存しています。波形は表示用に4点ごとに間引いています。</p>
<video id="video" controls src="VIDEO_URI"></video><p>MCUの動作開始に対応する動画時刻 [秒]：<input id="offset" type="number" step="0.005" value="0"><button id="align">現在の動画時刻を動作開始に合わせる</button> <button id="save">同期設定を保存</button></p>
<p id="readout"></p><canvas id="speed" width="1100" height="170"></canvas><canvas id="yaw" width="1100" height="170"></canvas><p>青＝計測・推定、灰＝指令。グラフをクリックすると対応する動画時刻へ移動します。位置・姿勢の精度を検証するには盤面校正とマーカー追跡が別途必要です。</p></div>
<script>const D=PAYLOAD,V=document.getElementById('video'),O=document.getElementById('offset');O.value=D.offset.toFixed(3);
function draw(id,a,b,label){const c=document.getElementById(id),x=c.getContext('2d'),s=D.samples,t=V.currentTime-Number(O.value);x.clearRect(0,0,c.width,c.height);const t0=s[0][0],t1=s[s.length-1][0];let mn=0,mx=0;for(const r of s){mn=Math.min(mn,r[a],r[b]);mx=Math.max(mx,r[a],r[b]);}const span=Math.max(mx-mn,.1);mn-=span*.1;mx+=span*.1;for(const [k,col] of [[a,'#7b8794'],[b,'#2375b5']]){x.strokeStyle=col;x.beginPath();s.forEach((r,i)=>{let px=45+(r[0]-t0)/(t1-t0)*1040,py=145-(r[k]-mn)/(mx-mn)*115;i?x.lineTo(px,py):x.moveTo(px,py)});x.stroke()}x.strokeStyle='#c44';let px=45+(t-t0)/(t1-t0)*1040;x.beginPath();x.moveTo(px,20);x.lineTo(px,150);x.stroke();x.fillStyle='#172331';x.fillText(label,10,15);c.onclick=e=>{const f=(e.clientX-c.getBoundingClientRect().left)/c.clientWidth;V.currentTime=Math.max(0,t0+(f*1100-45)/1040*(t1-t0)+Number(O.value));};}
function tick(){draw('speed',1,2,'速度 [m/s]');draw('yaw',3,4,'角速度 [rad/s]');const t=V.currentTime-Number(O.value);let r=D.samples.reduce((p,q)=>Math.abs(q[0]-t)<Math.abs(p[0]-t)?q:p);document.getElementById('readout').textContent=`動画 ${V.currentTime.toFixed(3)} s / MCU ${(D.zero+t*1000).toFixed(0)} ms / 速度 ${r[2].toFixed(3)} m/s / 角速度 ${r[4].toFixed(3)} rad/s / 左右距離 ${r[5].toFixed(1)}, ${r[6].toFixed(1)} mm`;requestAnimationFrame(tick);}document.getElementById('align').onclick=()=>O.value=V.currentTime.toFixed(3);document.getElementById('save').onclick=()=>{const a=document.createElement('a');a.href=URL.createObjectURL(new Blob([JSON.stringify({video_time_at_mcu_origin_s:Number(O.value),mcu_origin_ms:D.zero,source:'manual GUI alignment'},null,2)],{type:'application/json'}));a.download='sync_manual.json';a.click();};tick();</script>'''
    (trial_path.parent / 'review.html').write_text(page.replace('VIDEO_URI', video).replace('PAYLOAD', payload), encoding='utf-8')


if __name__ == '__main__':
    if len(sys.argv) == 3 and sys.argv[1] == '--verify':
        verify_video(Path(sys.argv[2]))
