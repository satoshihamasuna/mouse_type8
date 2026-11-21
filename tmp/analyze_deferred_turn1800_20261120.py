"""保存済みの取得専用試行を解析・一式保存する。機体とカメラには接続しない。"""
from pathlib import Path
import json
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from pixel_camera_tuning import analyze_trial
from pixel_camera_tuning_cli import save_json
from pixel_camera_turn_capture import utc_now
from pixel_camera_archive import archive_trial, verify_archive, write_index

folder = (ROOT / sys.argv[1]).resolve()
base = ROOT / 'tools/logs/pixel_camera/turn1800_A_r90_20261119_v1'
assert folder.is_relative_to(base.resolve())
trial = json.loads((folder / 'trial.json').read_text(encoding='utf-8'))
assert trial['state'] == 'CAPTURED_REVIEW_REQUIRED'
assert trial['camera_run_start_confirmed'] and trial['execution_write_attempted']
assert trial['saved_log']['status'] == 'saved'
pending_path = folder / 'deferred_analysis.json'
pending = json.loads(pending_path.read_text(encoding='utf-8'))
assert pending['state'] == 'PENDING'
result_path = folder.parent.parent / 'offline_analysis_result.json'
if result_path.exists():
    raise FileExistsError(result_path)
lock = folder / 'postprocessing.lock'
with lock.open('x', encoding='utf-8') as stream:
    stream.write(utc_now())
try:
    pending.update(state='RUNNING', started_at=utc_now())
    save_json(pending_path, pending)
    codes = {}
    for stride in (4, 1):
        codes[str(stride)] = analyze_trial(folder, stride)
    pending.update(state='ANALYZED' if all(code == 0 for code in codes.values())
        else 'ANALYSIS_REVIEW_REQUIRED', returncodes=codes, finished_at=utc_now())
    save_json(pending_path, pending)
finally:
    # 自分のロックだけを除き、解析出力は失敗時も保持する。
    lock.unlink()
result = dict(trial_json=str(folder / 'trial.json'), returncodes=codes,
    archive_pending=True, device_connections_opened=0, execution_commands_sent=0,
    finished_at=utc_now())
with result_path.open('x', encoding='utf-8') as stream:
    json.dump(result, stream, ensure_ascii=False, indent=2)
    stream.write('\n')
print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
