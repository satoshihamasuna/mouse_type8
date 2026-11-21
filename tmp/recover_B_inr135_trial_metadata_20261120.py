"""原本を保持し、欠落した試行情報を実機設定・送信記録から別索引へ復元する。実機操作なし。"""
from pathlib import Path
import argparse
import asyncio
import copy
import hashlib
import json
import os
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT/'tools/logs/pixel_camera/turn1600_B_inr135_20261120_v1'
SESSION = BASE/'baseline01/local_execution'
RID = 'B_INR135_1600_baseline_green_20261120_002'
RUN = SESSION/('run_'+RID)
OUT = RUN/'recovered_metadata_v1'
sys.path[:0] = [str(ROOT/'tools'), str(ROOT/'tools/analysis')]


def read(path): return json.loads(path.read_text(encoding='utf-8-sig'))


def save(path, value):
    with path.open('x', encoding='utf-8', newline='\n') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')


def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest()


def prepare():
    assert not OUT.exists()
    original = read(RUN/'trial.json')
    actual = read(RUN/'saved_1ms.settings.json')
    events = [json.loads(line) for line in (RUN/'events.jsonl').read_text(encoding='utf-8').splitlines()]
    attempts = [item for item in events if item['event']=='execution_write_attempt']
    assert len(attempts)==1 and actual['mouse']=='B' and actual['motion_name']=='in_r135'
    assert actual['preset_speed_mm_s']==1600 and actual['turn_count']==1
    assert original['state']=='CAPTURED_REVIEW_REQUIRED' and original['camera_state']=='PASSED'
    operator = read(BASE/'operator_ready_baseline002.json')
    config = read(BASE/'baseline01/config.json')
    status = read(SESSION/('transport_'+RID)/'telemetry.status.json')
    snapshot = read(RUN/'body_geometry_used.json')
    assert status['period_ms']==5 and config['mouse']==actual['mouse']==snapshot['mouse']
    restored = copy.deepcopy(original)
    restored.update(motion=actual['motion_name'],preset=actual['preset_speed_mm_s'],turn_count=actual['turn_count'],
        mouse_identity_source=actual['mouse_identity_source'],requested_suction=read(RUN/'requested.settings.json')['suction'],
        stream_period_ms=status['period_ms'],camera_height_mm=config['height_mm'],height_source='approximate_for_capture',
        camera_fixed_confirmed=read(BASE/'operator_ready_baseline001.json')['same_board_and_camera_height_1500mm'],
        execution_resend_count=len(attempts)-1,physical_240fps_verified=False,parameter_adoption_allowed=False,
        recording_stop_mode='motion_end_plus_postroll',recording_limit_seconds=20,recording_postroll_seconds=1.,
        capture_profile='tuning',camera_settings=config['camera_settings'],live_session=str(SESSION/('transport_'+RID)),
        quick_report=str(OUT/'quick/summary.json'),full_report=str(OUT/'full/summary.json'))
    restored['metadata_recovery'] = dict(reason='capture_connectedの参照/機体JSON保存ループが試行情報変数valueを上書き。原本を保持し、別索引のみ復元。',
        original_trial=str(RUN/'trial.json'),original_trial_sha256=sha(RUN/'trial.json'),
        motion_preset_count_source=str(RUN/'saved_1ms.settings.json'),
        resend_count_source=str(RUN/'events.jsonl'),execution_write_attempts=len(attempts),
        camera_height_and_profile_source=str(BASE/'baseline01/config.json'),operator_source=str(BASE/'operator_ready_baseline002.json'),
        original_quality_states_retained=True,source_trial_modified=False,hardware_operations=0)
    OUT.mkdir()
    filenames = ('saved_1ms.csv','saved_1ms.settings.json','telemetry.csv','telemetry.settings.json',
        'motion.csv','motion.settings.json','marker_reference_used.json','body_geometry_used.json',
        'requested.settings.json','events.jsonl','operator_confirmation.json')
    for filename in filenames:
        shutil.copyfile(RUN/filename, OUT/filename)
    save(OUT/'trial.json', restored)
    save(OUT/'metadata_recovery_provenance.json', dict(input_sha256={str(RUN/name):sha(RUN/name)
        for name in (*filenames,'trial.json')},recovered_trial_sha256=sha(OUT/'trial.json'),
        original_preserved=True,source_params_changed=False))
    print('Prepared separate recovered trial index; original evidence preserved', flush=True)


def review(stride):
    from pixel_camera_tuning_report import analysis_video_window, summarize
    kind = 'full' if stride==1 else 'quick'
    target = OUT/kind
    assert not target.exists()
    provenance = read(RUN/kind/'apparent_floor_marker_pose.provenance.json')
    start, end = analysis_video_window(read(OUT/'trial.json'))
    assert provenance['frame_stride']==stride and provenance['video_window']==[start,end]
    command = [sys.executable,str(ROOT/'tools/analysis/pixel_camera_relative_pose.py'),
        '--trial',str(OUT/'trial.json'),'--marker-reference',str(OUT/'marker_reference_used.json'),
        '--body-geometry',str(OUT/'body_geometry_used.json'),'--camera-height-mm','1500',
        '--height-source','approximate_for_capture','--fixed-camera','--captured-trial',
        '--video-start',str(start),'--video-end',str(end),'--frame-stride',str(stride),
        '--pose-cache',str(RUN/kind/'apparent_floor_marker_pose.csv'),'--output',str(target)]
    with (OUT/(kind+'_from_cache.stdout.txt')).open('x',encoding='utf-8') as log:
        result = subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,env=dict(os.environ,PYTHONUTF8='1'),
            creationflags=subprocess.CREATE_NO_WINDOW,check=False)
    if result.returncode:
        raise RuntimeError(f'{kind}の復元索引解析は要確認: returncode={result.returncode}')
    try:
        summary = summarize(target)
    except Exception as error:
        summary=dict(status='NEEDS_OPERATOR_REVIEW',auto_stop_confirmed=False,error=str(error))
        save(target/'summary.json',summary)
    print(json.dumps(dict(kind=kind,summary=summary),ensure_ascii=False),flush=True)


async def operator_result():
    from pixel_camera_tuning_mcp import McpTuningService, create_server
    service = McpTuningService(SESSION,{})
    result = await create_server(service).call_tool('record_operator_result',dict(trial_id=RID,both_powered_off=True,contact_detected=False))
    print(str(result),flush=True)
    state=read(BASE/'resume_state.json')
    state.update(state='BASELINE002_CAPTURED_BOTH_OFF_METADATA_RECOVERY',power_state='BOTH_OFF_USER_CONFIRMED',
        contact_detected=False,ble_connected=False,execution_pending=False,execution_commands_sent=1,execution_resend_count=0,
        last_request_id=RID,last_run=str(RUN),metadata_recovery=str(OUT),next_request_id=None,next_request_sent=False,
        operator_power_off_confirmation='両方OFF・接触なし',current_placement_confirmed=False,current_hands_clear_confirmed=False,
        next_action='保存動画の原本キャッシュから復元索引で全フレームを評価し、初回の出口方向と長さ候補を求める。両方OFFを維持。')
    (BASE/'resume_state.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')


parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('action',choices=('prepare','quick','full','operator'))
args=parser.parse_args()
if args.action=='prepare': prepare()
elif args.action=='operator': asyncio.run(operator_result())
else: review(1 if args.action=='full' else 4)
