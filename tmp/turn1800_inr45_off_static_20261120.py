"""機体Aの右45入口を、BLEへ接続せずOFF静止撮影で照合する。"""
from pathlib import Path
import argparse
import json
import os
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from pixel_camera_capture import camera_workspace, record_trial
from pixel_camera_tuning import analysis_python

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--stage', type=Path, required=True)
parser.add_argument('--name', required=True)
parser.add_argument('--manifest', type=Path, help='撮影済みの静止資料を再利用する')
args = parser.parse_args()
stage = args.stage.resolve()
assert stage.is_relative_to(ROOT / 'tools/logs/pixel_camera/turn1800_A_inr45_20261120_v1')
assert args.name.isidentifier()
out = stage / args.name
out.mkdir(exist_ok=False)
config = json.loads((stage / 'config.json').read_text(encoding='utf-8'))
assert config['mouse'] == 'A'
sys.path.insert(0, str(camera_workspace() / 'python'))
from pixel_camera import PixelCameraClient
if args.manifest:
    manifest = args.manifest.resolve()
    recording = json.loads(manifest.read_text(encoding='utf-8'))
else:
    camera = PixelCameraClient(connection='wifi', port=8765, auto_start=False)
    try:
        status = camera.get_status()
        assert status.get('connected') and status.get('device', {}).get('phase') == 'IDLE'
        manifest, recording = record_trial(camera, ROOT, seconds=5, mouse='A',
            label='inr45_OFF_' + args.name + '_static', camera_settings=config['camera_settings'],
            capture_profile='tuning')
    finally:
        camera.disconnect()
sys.path.insert(0, str(ROOT / 'tools/analysis'))
from pixel_camera_static_window import inspect_static_window
quality = inspect_static_window(manifest)
with (out / 'static_window_quality.json').open('x', encoding='utf-8') as f:
    json.dump(quality, f, ensure_ascii=False, indent=2)
if not quality['passed']:
    # 初回のラベル末尾の誤りだけは、元の失敗を残して同じ動画を独立検査する。
    assert quality['failure_reasons'] == ['not_a_five_second_tuning_static_capture']
    assert recording['state'] == 'PASSED' and recording['requested_seconds'] == 5
    assert recording['capture_profile'] == 'tuning' and recording['type8_log_csv'] is None
    from pixel_camera_static_window import assess_decoded_pts, decode_video_frames, file_sha256
    video = Path(recording['video_file'])
    assert file_sha256(video) == recording['sha256'] == recording['video_inspection']['sha256']
    independent = assess_decoded_pts(decode_video_frames(video))
    independent.update(source_manifest=str(manifest), source_label=recording['label'],
        original_label_check_failure_retained=True, video_sha256=recording['sha256'])
    with (out / 'independent_static_window_quality.json').open('x', encoding='utf-8') as f:
        json.dump(independent, f, ensure_ascii=False, indent=2)
    assert independent['passed'], '静止評価区間の独立検査が不成立です'
plan = dict(video=recording['video_file'], camera_manifest=str(manifest),
    reference=config['reference'], geometry=config['geometry'],
    placement=dict(x_mm=225., y_mm=45., heading_deg=90.,
                   position_tolerance_mm=5., heading_tolerance_deg=2.),
    stationary_video_window_s=[3.3, 4.95], stationary_frame_stride=4)
with (out / 'static_plan.json').open('x', encoding='utf-8') as f:
    json.dump(plan, f, ensure_ascii=False, indent=2)
with (out / 'static.stdout.txt').open('x', encoding='utf-8') as output:
    subprocess.run([str(analysis_python()), str(stage / 'static_pose_after_settle.py'),
        'measure', str(out / 'static_plan.json'), str(out / 'static_check.json')],
        stdout=output, stderr=subprocess.STDOUT, check=True, timeout=120,
        env=dict(os.environ, PYTHONUTF8='1', OPENCV_FOR_THREADS_NUM='4'),
        creationflags=subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0)
result = json.loads((out / 'static_check.json').read_text(encoding='utf-8'))
print(json.dumps(dict(output=str(out), passed=result['passed'],
    center_board_mm=result.get('center_board_mm'), heading_deg=result.get('heading_deg'),
    failures=result['failure_reasons'], observed_fraction=result.get('observed_fraction'),
    robot_commands_sent=0), ensure_ascii=False), flush=True)
