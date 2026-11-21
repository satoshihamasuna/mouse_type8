"""公開stdio MCPで電源ON依頼前のカメラ電池確認だけを行い、新規JSONへ保存する。"""
from pathlib import Path
import argparse
import asyncio
import json
import os
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import pixel_camera_tuning_mcp  # リポジトリ内のMCP依存を登録する
from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--session', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--adb')
parser.add_argument('--serial')
parser.add_argument('--min-level', type=int, default=20)
args = parser.parse_args()
assert not args.output.exists(), '既存の確認結果は上書きしません'


async def main():
    params = StdioServerParameters(command=sys.executable,
        args=[str(ROOT / 'tools/pixel_camera_tuning_mcp.py'), '--session', str(args.session.resolve())],
        env=dict(os.environ, PYTHONIOENCODING='utf-8'))
    async with stdio_client(params) as streams:
        async with ClientSession(*streams) as client:
            await client.initialize()
            result = await client.call_tool('check_camera_health',
                dict(adb=args.adb, serial=args.serial, min_level=args.min_level))
            assert not result.isError, 'カメラ電池確認ツールが失敗しました'
            value = result.structuredContent or json.loads(next(c.text for c in result.content if c.type == 'text'))
            if 'result' in value and 'passed' not in value:
                value = value['result']
            with args.output.open('x', encoding='utf-8') as output:
                json.dump(value, output, ensure_ascii=False, indent=2, allow_nan=False)
            print(json.dumps(value, ensure_ascii=False), flush=True)


asyncio.run(main())
