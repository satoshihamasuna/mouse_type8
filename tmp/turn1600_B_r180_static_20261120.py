"""B1600右180度のカメラだけの静止照合。BLE・機体コマンドは使用しない。"""
from pathlib import Path
import argparse
import json
import os
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tools/logs/pixel_camera/turn1600_B_r180_20261120_v1'
sys.path[:0] = [str(ROOT/'tools')]
from pixel_camera_capture import camera_workspace, record_trial
from pixel_camera_tuning import analysis_python, hidden_process

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('action', choices=('record','inspect'))
parser.add_argument('--tag', required=True)
parser.add_argument('--config',type=Path,default=BASE/'baseline01/config.json')
args = parser.parse_args()
if not args.tag.replace('_','').isalnum():
    raise ValueError('英数字と_のタグを指定する')
folder = BASE / args.tag
read = lambda path: json.loads(Path(path).read_text(encoding='utf-8-sig'))


def save(path, value):
    with path.open('x',encoding='utf-8') as output:
        json.dump(value,output,ensure_ascii=False,indent=2,allow_nan=False)


if args.action == 'record':
    folder.mkdir(exist_ok=False)
    sys.path.insert(0,str(camera_workspace()/'python'))
    from pixel_camera import PixelCameraClient
    client = PixelCameraClient(connection='wifi',port=8765,auto_start=False)
    status = client.get_status()
    if not status.get('connected') or status.get('device',{}).get('phase') != 'IDLE':
        raise RuntimeError('カメラの接続・IDLEを確認できない。自動再試行しない。')
    config_path=args.config.resolve()
    if not config_path.is_relative_to(BASE.resolve()):
        raise ValueError('今回の測定フォルダー内のconfigを指定する')
    config = read(config_path)
    manifest,recording = record_trial(client,ROOT,seconds=5,mouse='B',
        label='B_R180_1600_'+args.tag+'_static_camera_only',
        camera_settings=config['camera_settings'],capture_profile='tuning')
    save(folder/'capture.json',dict(manifest=str(manifest),**recording))
    if recording.get('video_file'):
        save(folder/'plan.json',dict(video=recording['video_file'],
            camera_manifest=str(manifest),reference=config['reference'],geometry=config['geometry'],
            placement=read(BASE/'baseline01/placement_plan.json')['placement'],
            stationary_video_window_s=[3.3,4.95],stationary_frame_stride=2))
    print(json.dumps(dict(state=recording['state'],video=recording.get('video_file'),
        error=recording.get('error'),robot_commands_sent=0),ensure_ascii=False),flush=True)
    raise SystemExit(0 if recording['state']=='PASSED' else 1)

capture = read(folder/'capture.json')
if capture['state'] != 'PASSED':
    raise RuntimeError('録画全体の時刻・転送・復号検査が未合格。結果を保持して確認する。')
result = folder/'static_check.json'
if result.exists():
    raise FileExistsError('既存の静止結果は上書きしない')
with (folder/'inspect.stdout.txt').open('x',encoding='utf-8') as output:
    subprocess.run([str(analysis_python()),str(BASE/'baseline01/static_pose_after_settle.py'),
        'measure',str(folder/'plan.json'),str(result)],check=True,timeout=120,
        stdout=output,stderr=subprocess.STDOUT,creationflags=hidden_process(),
        env=dict(os.environ,PYTHONUTF8='1',OPENCV_FOR_THREADS_NUM='4'))
checked = read(result)
summary = {key:checked.get(key) for key in ('passed','frames','observed','center_board_mm',
    'heading_deg','failure_reasons','representative_heading_error_deg','heading_jitter_rms_deg')}
print(json.dumps(summary,ensure_ascii=False),flush=True)
