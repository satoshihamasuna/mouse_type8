"""操作者確認をMCP経由で保存し、既存MCPの共通処理で保存資料を解析する。実機接続なし。"""
from pathlib import Path
from datetime import datetime, timezone
import asyncio
import json
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
import pixel_camera_tuning_mcp as tuning
from pixel_camera_archive import archive_trial, verify_archive, write_index
from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_r90_20261119_v1'
cases=[('repeat_c1p25_02','A_R90_1800_c1p25_repeat02_20261120_003'),
       ('repeat_c1p25_03','A_R90_1800_c1p25_repeat03_20261120_001')]


async def record_operator():
    for stage,rid in cases:
        params=StdioServerParameters(command=sys.executable,args=[str(ROOT/'tools/pixel_camera_tuning_mcp.py'),
            '--session',str(BASE/stage/'local_execution')])
        async with stdio_client(params) as streams:
            async with ClientSession(*streams) as client:
                await client.initialize()
                result=await client.call_tool('record_operator_result',dict(trial_id=rid,
                    both_powered_off=True,contact_detected=False))
                assert not result.isError,result
        print('MCP record_operator_result: '+rid,flush=True)


asyncio.run(record_operator())
stage=BASE/'repeat_c1p25_03'
with (stage/'operator_stop.json').open('x',encoding='utf-8') as f:
    json.dump(dict(recorded_at_utc=datetime.now(timezone.utc).isoformat(),
        source='Operator: first option completed (1完了)',both_off=True,stopped=True,contact=False),f,ensure_ascii=False,indent=2)
p=BASE/'preparation.json'; state=json.loads(p.read_text(encoding='utf-8'))
assert state['execution_commands_sent']==9
run3=stage/'local_execution/run_A_R90_1800_c1p25_repeat03_20261120_001'
trial3=json.loads((run3/'trial.json').read_text(encoding='utf-8'))
assert trial3['execution_write_attempted'] and trial3['camera_state']=='PASSED'
assert trial3['log_quality']['full_motion_retained'] and trial3['saved_log']['status']=='failed'
state.update(status='THREE_RUNS_ACQUIRED_PARTIAL_1MS_ON_THIRD',current_phase='BATCH_SAVED_ANALYSIS',
    power_state='BothOFF_confirmed',operator_contact=False,operator_ready=False,execution_pending=False,
    current_hands_clear_confirmed=False,ble_disconnected=True,execution_commands_sent=10,
    completed_execution_count=10,completed_same_c1p25_Ls7p8_Le25_runs=3,last_completed_trial=str(run3/'trial.json'),
    full_analysis_pending_for_repeat02=False,full_analysis_pending_for_repeat03=True,
    current_placement_confirmed=False,current_clearance_confirmed=False,next_candidate_executed=True,
    current_location_state='Stopped at third run endpoint, bothOFF confirmed',
    third_run_retained_1ms_available=False,third_run_video_and_5ms_available=True)
p.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

# アーカイブの索引更新は直列化する。
run2=BASE/'repeat_c1p25_02/local_execution/run_A_R90_1800_c1p25_repeat02_20261120_003'
archive=archive_trial(run2/'trial.json'); verify_archive(archive); write_index(archive.parent)
print('Second-run archive verified',flush=True)
marker=stage/'mcp_saved_analysis_partial.json'
assert not marker.exists() and not (run3/'full.stdout.txt').exists()
# FAILED原記録は維持。動画と5ms CSVを既存の共通解析へ明示的に渡す。
tuning.postprocess(run3/'trial.json',marker)
result=json.loads(marker.read_text(encoding='utf-8'))
print(json.dumps(dict(state=result['state'],errors=result['errors'],archive=result.get('archive')),
    ensure_ascii=False),flush=True)
