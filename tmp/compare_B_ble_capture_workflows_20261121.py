"""利用可能なローカルB試行だけを読み、取得条件とBLE結果を比較する。実機操作なし。"""
from pathlib import Path
from datetime import datetime, timezone
import json
root=Path(__file__).resolve().parents[1]
ours=root/'tools/logs/pixel_camera/turn1600_B_outr135_20261121_resume_v1'
other=root/'tools/logs/pixel_camera/turn1600_B_rv90_20261120_v1'
out=ours/'other_B_workflow_comparison_v1';assert not out.exists();out.mkdir()
def read(path):return json.loads(path.read_text(encoding='utf-8-sig'))
groups={}
for name,base in [('out135',ours),('rV90',other)]:
    rows=[]
    for path in base.glob('*/local_execution/jobs/*.json'):
        job=read(path);config=job.get('request',{}).get('config',{})
        if config.get('address','').upper()!='E1:7F:86:21:84:73':continue
        local=path.parent.parent;identifier=job['id'];transport=local/('transport_'+identifier)
        if not (transport/'gateway.stdout.txt').is_file():continue
        result=job.get('result',{})
        trace=(transport/'gateway.stderr.txt').read_text(encoding='utf-8',errors='replace') if (transport/'gateway.stderr.txt').is_file() else ''
        events=[]
        for directory in ('prepare_'+identifier,'run_'+identifier):
            source=local/directory/'events.jsonl'
            if source.is_file():events += [json.loads(line) for line in source.read_text(encoding='utf-8-sig').splitlines()]
        commands=[event['command'] for event in events if event['event']=='shell_write_attempt']
        health_path=transport/'telemetry.health.jsonl'
        health=[json.loads(line) for line in health_path.read_text(encoding='utf-8').splitlines()] if health_path.is_file() else []
        completed=result.get('state')=='CAPTURED_REVIEW_REQUIRED'
        ble_cut='RuntimeError: BLE disconnected; in-flight data will not be replayed' in trace
        rows.append(dict(job=str(path),request_id=identifier,started_at=job.get('started_at'),state=result.get('state'),
            config_address=config['address'],stream_period_ms=config.get('stream_period_ms',5),
            char_delay=config.get('char_delay'),captured=completed,ble_disconnected_error=ble_cut,
            secondary_cancelled_error='asyncio.exceptions.CancelledError' in trace,
            execution_attempts=sum(event['event']=='execution_write_attempt' for event in events),
            last_shell_command=commands[-1] if commands else None,shell_write_attempts=len(commands),
            last_health=health[-1] if health else None,
            observed_health_span_s=health[-1]['monotonic']-health[0]['monotonic'] if len(health)>1 else None))
    rows.sort(key=lambda row:row['started_at'] or '')
    groups[name]=dict(attempts_with_B_gateway=len(rows),captured=sum(row['captured'] for row in rows),
        ble_disconnected_errors=sum(row['ble_disconnected_error'] for row in rows),other_results=sum(not row['captured'] and not row['ble_disconnected_error'] for row in rows),
        periods_ms=sorted(set(row['stream_period_ms'] for row in rows)),char_delays=sorted(set(row['char_delay'] for row in rows if row['char_delay'] is not None)),rows=rows)
result=dict(compared_at=datetime.now(timezone.utc).isoformat(),groups=groups,
    scope='この共有リポジトリで参照できるB用E1アドレスの録画付き取得のみ。チャット全体や機体Aの成功率ではない。',
    same_shared_gateway_implementation=True,
    observed_wrapper_difference='out135は独自performで先に静止撮影し、標準performの静止確認をキャッシュ結果へ差替え。rV90は標準MCP perform。現在の標準perform自体がcamera-before-BLEのため独自層は重複。',
    cause_identified=False,period_alone_explains_difference=False,
    interpretation='同アドレス/5ms/0.08sでもrV90の取得率が高い場合、機体本体や周期だけが原因とは断定できない。独自取得層を標準へ揃え、同時送受信条件とリンク診断を切り分ける。',
    robot_connections_attempted=0,robot_commands_sent=0)
with (out/'comparison.json').open('x',encoding='utf-8') as stream:json.dump(result,stream,ensure_ascii=False,indent=2)
print(json.dumps({key:{name:value for name,value in group.items() if name!='rows'} for key,group in groups.items()},ensure_ascii=False),flush=True)
