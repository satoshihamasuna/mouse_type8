"""原FAILEDを保持し、検証済み動画と5ms CSVの復旧記録をMCPの保存済み解析へ渡す。"""
from pathlib import Path
import asyncio
import copy
import csv
import hashlib
import json
import os
import shutil
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
import pixel_camera_tuning_mcp as tuning
from pixel_camera_archive import file_hash
from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_r90_20261119_v1'
STAGE=BASE/'repeat_c1p25_03'
SOURCE=STAGE/'local_execution/run_A_R90_1800_c1p25_repeat03_20261120_001'
source_trial=SOURCE/'trial.json'
original=json.loads(source_trial.read_text(encoding='utf-8'))
assert original['state']=='FAILED' and original['saved_log']['status']=='failed'
assert original['camera_state']=='PASSED' and original['camera_run_start_confirmed']
assert original['execution_write_attempted'] and original['log_quality']['full_motion_retained']
assert original['log_quality']['lost_frames']==0 and original['log_quality']['crc_errors_during_trial']==0
assert original['recording_stop']['log_stop_evidence']['confirmed']
capture=json.loads(Path(original['capture_json']).read_text(encoding='utf-8'))
expected=capture.get('sha256') or capture.get('video_inspection',{}).get('sha256')
assert expected and file_hash(original['video_file'])==expected
settings=json.loads((SOURCE/'motion.settings.json').read_text(encoding='utf-8'))
assert settings['mouse']==original['mouse']=='A' and settings['motion_name']=='long_r90'
with (SOURCE/'motion.csv').open(encoding='utf-8',newline='') as f:
    raw=list(csv.DictReader(f))
assert float(raw[0]['time_ms'])<original['log_quality']['motion_start_time_ms']
assert float(raw[-1]['time_ms'])>original['log_quality']['motion_end_time_ms']
with (SOURCE/'telemetry.csv').open(encoding='utf-8',newline='') as f:
    continuous=list(csv.DictReader(f))
assert float(continuous[-1]['time_ms'])>=original['recording_stop']['log_stop_evidence']['stationary_start_time_ms']+1000

OUT=STAGE/'partial_review_v1'; OUT.mkdir(exist_ok=False)
for name in ('motion.csv','motion.settings.json','telemetry.csv','telemetry.settings.json',
             'body_geometry_used.json','marker_reference_used.json','events.jsonl','pc.shell.bin',
             'requested.settings.json','operator_confirmation.json'):
    shutil.copy2(SOURCE/name,OUT/name)
shutil.copy2(source_trial,OUT/'original_failed_trial.json')
review=copy.deepcopy(original)
# 取得全体のFAILEDは原記録に保持。別文書は取得済みの動画/5msのみを表す未採用の解析入力。
review.update(state='CAPTURED_REVIEW_REQUIRED',partial_capture_review=True,
    type8_log_csv=str(OUT/'motion.csv'),type8_settings_json=str(OUT/'motion.settings.json'),
    quick_report=str(OUT/'quick/summary.json'),full_report=str(OUT/'full/summary.json'),
    original_trial_json=str(source_trial),original_trial_state='FAILED',
    original_trial_sha256=file_hash(source_trial),retained_1ms_available=False,
    precise_Lend_timing_available=False,
    recovery_scope='Video and continuous5ms only; saved1ms failed. Original error and saved_log fields retained. No additional execution.',
    recovery_checks=dict(video_sha256_verified=True,motion_coverage_and_zero_loss_verified=True,
        recorded_postroll_stop_confirmed=True,machine_and_settings_matched=True,source_unchanged=True),
    parameter_adoption_allowed=False)
with (OUT/'trial.json').open('x',encoding='utf-8') as f:
    json.dump(review,f,ensure_ascii=False,indent=2)
assert file_hash(source_trial)==review['original_trial_sha256']


def value(result):
    assert not result.isError,result
    parsed=result.structuredContent
    if parsed is None:
        parsed=json.loads(next(c.text for c in result.content if c.type=='text'))
    if 'result' in parsed and 'state' not in parsed:
        parsed=parsed['result']
    return parsed


async def analyze():
    params=StdioServerParameters(command=sys.executable,
        args=[str(ROOT/'tools/pixel_camera_tuning_mcp.py'),'--session',str(STAGE/'partial_review_mcp')],
        env=dict(os.environ,TYPE8_TRIALS_DIR='D:/TYPE8_Camera/partial_reviews'))
    with (STAGE/'mcp_partial_review.stderr.txt').open('x',encoding='utf-8') as log:
        async with stdio_client(params,errlog=log) as streams:
            async with ClientSession(*streams) as client:
                await client.initialize()
                accepted=value(await client.call_tool('analyze_saved_trial',dict(trial_json=str(OUT/'trial.json'))))
                print('MCP analyze_saved_trial: partial video/5ms review accepted',flush=True)
                while True:
                    status=value(await client.call_tool('get_trial_status',dict(trial_json=str(OUT/'trial.json'))))
                    analysis=status.get('analysis',{})
                    if analysis.get('state') not in (None,'ACCEPTED','RUNNING','ARCHIVING'):
                        print(json.dumps(dict(state=analysis['state'],errors=analysis.get('errors'),
                            archive=analysis.get('archive')),ensure_ascii=False),flush=True)
                        break
                    await asyncio.sleep(15)


asyncio.run(analyze())
