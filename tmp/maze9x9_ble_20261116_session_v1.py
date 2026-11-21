"""既存の探索・録画クラスを使う今回限りの9×9確認。start.json受理前は走行しない。"""
import argparse
import asyncio
from datetime import datetime, timezone, timedelta
import json
from pathlib import Path
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
sys.path.insert(0, str(ROOT/'tools/ble'))
sys.path.insert(0, str(ROOT/'tools/analysis'))
from pixel_camera_profiles import load_camera_profile


def save(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    temporary.replace(path)


def preflight(folder):
    from pixel_camera_capture import camera_workspace, record_trial
    sys.path.insert(0, str(camera_workspace()/'python'))
    from pixel_camera import PixelCameraClient
    folder.mkdir(parents=True, exist_ok=False)
    settings = load_camera_profile('maze_9x9')
    camera = PixelCameraClient(connection='wifi', host='127.0.0.1', port=8765, auto_start=False)
    status = camera.get_status()
    if not status.get('connected') or status.get('device', {}).get('phase') != 'IDLE':
        raise RuntimeError('カメラが接続済みの待機状態ではありません')
    manifest, result = record_trial(camera, ROOT, seconds=3, label='9x9_ArUco_preflight_no_robot_command',
        camera_settings=settings, capture_profile='maze_9x9')
    save(folder/'capture.json', dict(manifest=str(manifest), **result))
    if result['state'] != 'PASSED':
        raise RuntimeError(result.get('error', '確認動画の検査に失敗しました'))
    subprocess.run(['D:/venv/venv3-11/Scripts/python.exe', '-X', 'utf8', str(Path(__file__).resolve()),
        'inspect', '--output', str(folder)], check=True)


def inspect(folder):
    import cv2
    import maze_video_pose as pose
    captured = json.loads((folder/'capture.json').read_text(encoding='utf-8'))
    reference_path = ROOT/'tools/logs/pixel_camera/board_9x9_reference.json'
    reference = json.loads(reference_path.read_text(encoding='utf-8'))
    save(folder/'marker_reference_used.json', reference)
    capture = cv2.VideoCapture(captured['video_file'])
    capture.set(cv2.CAP_PROP_ORIENTATION_AUTO, 0)
    rows = []
    index = 0
    last = None
    try:
        aruco = pose.detector()
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            pts = capture.get(cv2.CAP_PROP_POS_MSEC)/1000
            if index % 4 == 0:
                matrix, evidence = pose.registration(frame, aruco, reference)
                rows.append(dict(frame=index, pts=pts, valid=matrix is not None, **evidence))
                last = frame
            index += 1
    finally:
        capture.release()
    recent = [row for row in rows if row['pts'] >= rows[-1]['pts']-1] if rows else []
    rate = sum(row['valid'] for row in recent)/len(recent) if recent else 0
    report = dict(passed=bool(len(recent) >= 50 and rate >= .95), samples=len(recent),
        calibration_rate=rate, frames_decoded=index, reference=str(reference_path),
        video=captured['video_file'], settings=captured['camera_configuration'], samples_all=rows,
        robot_commands_sent=0)
    if last is not None:
        cv2.imwrite(str(folder/'preflight_frame.jpg'), last)
    save(folder/'aruco_preflight.json', report)
    print(json.dumps({key:value for key,value in report.items() if key != 'samples_all'}, ensure_ascii=False), flush=True)


async def connect(folder, address):
    from mouse_ble_explore import ExploreSession, ExploreOptions
    from explore_camera import ExploreCameraRun
    folder.mkdir(parents=True, exist_ok=False)
    camera_run = None
    latest = {}
    lock = threading.Lock()
    started_once = False
    connected_at = time.monotonic()
    def emit(kind, data):
        nonlocal camera_run
        if camera_run is not None:
            camera_run.observe(kind, data)
        if kind not in ('telemetry', 'bridge_status'):
            print(json.dumps(dict(event=kind, data=data), ensure_ascii=False), flush=True)
        with lock:
            latest[kind] = data
    session = ExploreSession(address=address, period_ms=5, output_root=folder, emit=emit)
    task = asyncio.create_task(session.run())
    try:
        while not task.done():
            with lock:
                status = dict(session.snapshot(), directory=str(session.directory), events=dict(latest),
                    capability=session.capability, shortest_capability=session.shortest_capability,
                    recording_active=bool(camera_run and camera_run.active), start_consumed=started_once)
            save(folder/'status.json', status)
            command = folder/'start.json'
            if command.exists() and not started_once:
                command.replace(folder/'start.consumed.json')
                started_once = True  # エラー・途中送信・応答消失でも再送しない。
                request = json.loads((folder/'start.consumed.json').read_text(encoding='utf-8'))
                if request.get('operator_ready') is not True or not session.ready():
                    raise RuntimeError('今回の設置確認または実機接続の準備が不足しています')
                options = ExploreOptions(**request['options'])
                options.command()
                analysis = dict(reference=str(ROOT/'tools/logs/pixel_camera/board_9x9_reference.json'),
                    python='D:/venv/venv3-11/Scripts/python.exe', cell_mm=90, stride=4,
                    marker_colours_source='run_settings.firmware.mouse', height_correction=False)
                camera_run = ExploreCameraRun(session, emit, analysis_config=analysis,
                    camera_settings=load_camera_profile('maze_9x9'))
                camera_run.start(options)
            if (folder/'stop.json').exists():
                if camera_run and camera_run.active:
                    camera_run.cancel()
                session.action('stop')
                (folder/'stop.json').replace(folder/'stop.consumed.json')
            if (folder/'disconnect.json').exists():
                if camera_run and camera_run.active:
                    camera_run.cancel()
                session.action('disconnect')
                (folder/'disconnect.json').replace(folder/'disconnect.consumed.json')
            if not started_once and time.monotonic()-connected_at > 180:
                session.action('disconnect')
            if started_once and camera_run and not camera_run.active and session.state == 'idle':
                session.action('disconnect')
            await asyncio.sleep(.3)
        await task
    finally:
        if not task.done():
            session.action('disconnect')
            try:
                await asyncio.wait_for(task, 8)
            except Exception:
                task.cancel()
        save(folder/'final.json', dict(snapshot=session.snapshot(), events=latest, start_consumed=started_once,
            directory=str(session.directory), finished_at=datetime.now(timezone(timedelta(hours=9))).isoformat()))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=('preflight','inspect','connect'))
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--address', default='C1:B5:60:87:91:73')
    args = parser.parse_args()
    if args.action == 'connect':
        asyncio.run(connect(args.output.resolve(), args.address))
    else:
        globals()[args.action](args.output.resolve())
