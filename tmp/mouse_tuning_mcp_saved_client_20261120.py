"""ローカルMCPの操作者結果保存・保存済み解析を呼ぶ。実機接続なし。"""
from pathlib import Path
import argparse
import asyncio
import json
import os
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
import pixel_camera_tuning_mcp
from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--session',type=Path,required=True)
parser.add_argument('--trial',type=Path)
parser.add_argument('--off-no-contact-id')
parser.add_argument('--powered-no-contact-id')
args=parser.parse_args()
if args.off_no_contact_id and args.powered_no_contact_id:
    parser.error('電源OFFと通電中の結果を同時に指定できません')


def unpack(result):
    assert not result.isError,result
    data=result.structuredContent or json.loads(next(c.text for c in result.content if c.type=='text'))
    return data['result'] if 'result' in data and 'state' not in data else data


async def main():
    params=StdioServerParameters(command=sys.executable,args=[str(ROOT/'tools/pixel_camera_tuning_mcp.py'),
        '--session',str(args.session.resolve())],env=dict(os.environ))
    async with stdio_client(params) as streams:
        async with ClientSession(*streams) as client:
            await client.initialize()
            identifier=args.off_no_contact_id or args.powered_no_contact_id
            if identifier:
                unpack(await client.call_tool('record_operator_result',dict(trial_id=identifier,
                    both_powered_off=bool(args.off_no_contact_id),contact_detected=False)))
                job=unpack(await client.call_tool('get_trial_status',dict(trial_id=identifier)))
                result=job.get('result',{})
                stage=args.session.resolve().parent
                runtime_file=stage/'runtime_conditions_for_review.json'
                modes_file=args.session.resolve()/'runtime_modes.json'
                acceleration=result.get('straight_acceleration') or result.get('diagonal_acceleration')
                if result.get('trial_json') and acceleration and modes_file.exists() and not runtime_file.exists():
                    trial=json.loads(Path(result['trial_json']).read_text(encoding='utf-8'))
                    settings=json.loads(Path(trial['type8_settings_json']).read_text(encoding='utf-8'))
                    plan=json.loads((stage/'placement_plan.json').read_text(encoding='utf-8'))
                    modes=json.loads(modes_file.read_text(encoding='utf-8'))
                    profile=acceleration['verified']
                    profile_key='post_straight_profile' if result.get('straight_acceleration') else 'post_diagonal_profile'
                    runtime=dict(pre_run_mm=plan['pre_run_mm'],post_run_mm=plan['post_run_mm'],
                        requested_turn_pre_accel_m_s2=settings['parameters']['pre_accel'],post_accel_m_s2=profile['acc'],
                        wall_ekf=modes['wall_ekf show'],wall_edge=modes['wall_edge show'],turn_ekf=modes['turn_ekf show'],
                        sources=[str(modes_file),str(args.session.resolve()/'jobs'/(identifier+'.json'))])
                    runtime[profile_key]=profile
                    with runtime_file.open('x',encoding='utf-8') as output:
                        json.dump(runtime,output,ensure_ascii=False,indent=2)
                print('MCP operator result saved',flush=True)
            if args.trial:
                trial=str(args.trial.resolve())
                unpack(await client.call_tool('analyze_saved_trial',dict(trial_json=trial)))
                print('MCP saved analysis accepted',flush=True)
                while True:
                    state=unpack(await client.call_tool('get_trial_status',dict(trial_json=trial)))
                    analysis=state.get('analysis',{})
                    if analysis.get('state') not in (None,'ACCEPTED','RUNNING','ARCHIVING'):
                        print(json.dumps(dict(state=analysis['state'],errors=analysis.get('errors'),
                            archive=analysis.get('archive')),ensure_ascii=False),flush=True)
                        break
                    await asyncio.sleep(15)


asyncio.run(main())
