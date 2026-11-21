"""完成済みfull解析を読み、R180仕上げ1本の相対光学評価を新規保存する。"""
from pathlib import Path
import hashlib
import json
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tools/logs/pixel_camera/turn1800_A_r180_20261120_v1'
REVIEW_ROOT = BASE / 'parallel_finish'
STAGE = BASE / 'finish02_01'
REQUEST = 'A_R180_1800_finish02_c3p5_r48_Le31p5_20261120_001'
RELATIVE_RUN = 'finish02_01/local_execution/run_' + REQUEST
RUN = BASE / RELATIVE_RUN
RUNTIME = REVIEW_ROOT / 'finish02_runtime_conditions.json'
ADAPTER = ROOT / 'tmp/parallel_inr135_r180_finish02_relative_adapter_20261120.py'


def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def save(path, value):
    with path.open('x', encoding='utf-8') as file:
        json.dump(value, file, ensure_ascii=False, indent=2, allow_nan=False)
        file.write('\n')


assert (RUN / 'full/report.json').is_file(), 'rootのfull解析完了待ち。二重解析しない。'
report = read(RUN / 'full/report.json')
quality = report['required_motion_window_quality']
if not (RUN / 'full/summary.json').is_file() or not quality['accepted_for_relative_review']:
    save(REVIEW_ROOT / 'finish02_full_quality_excluded.json', dict(
        source=str(RUN / 'full/report.json'), summary_present=(RUN / 'full/summary.json').is_file(),
        required_motion_window_quality=quality, parameter_adoption_allowed=False,
        reason='元の品質条件を維持し、不合格またはsummary欠落のfullを光学採用しない。'))
    raise SystemExit('Full quality not eligible; excluded without lowering thresholds')
job_file = STAGE / 'local_execution/jobs' / (REQUEST + '.json')
modes_file = STAGE / 'local_execution/runtime_modes.json'
job = read(job_file)
modes = read(modes_file)
verified = job['result']['straight_acceleration']['verified']
assert verified['acc'] == 20.
settings = read(RUN / 'saved_1ms.settings.json')
runtime = dict(post_straight_profile=verified, pre_run_mm=90., post_run_mm=90.,
    requested_turn_pre_accel_m_s2=settings['parameters']['pre_accel'], post_accel_m_s2=verified['acc'],
    wall_ekf=modes['wall_ekf show'], wall_edge=modes['wall_edge show'], turn_ekf=modes['turn_ekf show'],
    sources=[str(modes_file), str(job_file)],
    source_sha256={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in (modes_file, job_file)})
if RUNTIME.exists():
    assert read(RUNTIME) == runtime
else:
    save(RUNTIME, runtime)
original_path = ROOT / 'tmp/review_turn1800_r180_repeats_20261120.py'
original = original_path.read_text(encoding='utf-8')
old = "    return original if original.exists() else run.parent.parent/'runtime_conditions_for_review.json'"
replacement = ("    if run.parent.parent.name == 'finish02_01':\n"
               "        return BASE/'parallel_finish/finish02_runtime_conditions.json'\n" + old)
assert original.count(old) == 1
if ADAPTER.exists():
    assert ADAPTER.read_text(encoding='utf-8') == original.replace(old, replacement)
else:
    with ADAPTER.open('x', encoding='utf-8') as file:
        file.write(original.replace(old, replacement))
relative = REVIEW_ROOT / 'finish02_relative_v1'
optical = REVIEW_ROOT / 'finish02_optical_clock_v1'
commands = [
    [sys.executable, str(ADAPTER), '--output', str(relative), '--run', 'finish02=' + RELATIVE_RUN],
    [sys.executable, str(ROOT / 'tmp/align_turn1800_pre_straight_20261120.py'), '--review', str(relative),
     '--output', str(optical), '--labels', 'finish02', '--optical-r180']]
for i, command in enumerate(commands):
    with (REVIEW_ROOT / f'finish02_saved_review_{i+1:02d}.stdout.txt').open('x', encoding='utf-8') as log:
        subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, check=True,
            creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
result = read(optical / 'report.json')['results'][0]
print(json.dumps(result, ensure_ascii=False, indent=2))
