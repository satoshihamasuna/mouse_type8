"""保存済み設定から独立した単体候補stageを準備・オフライン検証する。

実機操作機能は持たない。--count はstage数で、各stageのturn_countは常に1。
例（Pixel camera用Pythonで実行）:
  python tmp/mouse_tuning_prepare_candidate_20261120.py --base <source-stage>
    --source-run <saved-run> --name candidate --id-prefix A_INR45_1800_candidate
    --correction 0.5 --count 3 --date 20261120 --basis <review.json>
出力はbaseと同じ親のcandidate01/02/03。--output-baseで出力親を指定できる。
"""
from __future__ import annotations

import argparse
import ast
from copy import deepcopy
from datetime import datetime, timedelta, timezone
from hashlib import sha256
import json
import math
from pathlib import Path
import re
import shutil
import sys

ROOT = Path(__file__).resolve().parents[1]
KEYS = ('mouse', 'motion_name', 'preset_speed_mm_s', 'parameters', 'pid',
        'feedforward', 'suction', 'turn_count', 'schema_version', 'mouse_identity_source')
COPY_FILES = ('config.json', 'static_pose_after_settle.py')
OPTIONAL_COPY_FILES = ('static_settle_policy.json', 'static_gate_v2_source.py',
                       'static_gate_v2_validation.json', 'coarse_placement_check.py')


def inside(value):
    path = Path(value).resolve()
    if not path.is_relative_to(ROOT):
        raise ValueError(f'workspace外のパスは使用できません: {path}')
    return path


def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def write(path, value):
    with path.open('x', encoding='utf-8', newline='\n') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')


def digest(path):
    return sha256(path.read_bytes()).hexdigest()


def literal(node):
    if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == 'dict':
        if node.args or any(k.arg is None for k in node.keywords):
            raise ValueError('placementのdict展開は対応外です')
        return {k.arg: ast.literal_eval(k.value) for k in node.keywords}
    return ast.literal_eval(node)


def wrapper_info(source):
    tree = ast.parse(source)
    defaults = [k.value for n in ast.walk(tree) if isinstance(n, ast.Call)
                and isinstance(n.func, ast.Attribute) and n.func.attr == 'add_argument'
                and n.args and isinstance(n.args[0], ast.Constant)
                and n.args[0].value == '--trial-id' for k in n.keywords if k.arg == 'default']
    placements = [n.value for n in ast.walk(tree) if isinstance(n, ast.Assign)
                  and any(isinstance(t, ast.Name) and t.id == 'placement' for t in n.targets)]
    prepares = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
                and isinstance(n.func, ast.Attribute) and n.func.attr == 'prepare']
    if len(defaults) != 1 or len(placements) != 1 or len(prepares) != 1:
        raise ValueError('wrapperのtrial-id/placement/prepareを一意に特定できません')
    call = prepares[0]
    if len(call.args) != 5 or call.keywords:
        raise ValueError('wrapperのprepare引数形式が対応外です')
    return tree, defaults[0], literal(placements[0]), literal(call.args[3]), literal(call.args[4])


def change_default(source, identifier):
    tree, node, placement, diagonal, straight = wrapper_info(source)
    if not isinstance(node, ast.Constant) or not isinstance(node.value, str):
        raise ValueError('trial-idのdefaultは文字列リテラルである必要があります')
    lines = source.splitlines(keepends=True)
    if node.lineno != node.end_lineno:
        raise ValueError('複数行のtrial-id defaultは対応外です')
    index = node.lineno - 1
    line = lines[index].encode('utf-8')
    lines[index] = (line[:node.col_offset] + repr(identifier).encode('utf-8')
                    + line[node.end_col_offset:]).decode('utf-8')
    result = ''.join(lines)
    other, changed, _, _, _ = wrapper_info(result)
    changed.value = node.value
    if ast.dump(tree, include_attributes=False) != ast.dump(other, include_attributes=False):
        raise ValueError('wrapperのtrial-id以外に差分があります')
    compile(result, '<candidate wrapper>', 'exec')
    return result, placement, diagonal, straight


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--base', required=True, help='コピー元stage')
    parser.add_argument('--source-run', required=True, help='saved_1ms.settings.jsonがあるrun、または設定JSON')
    parser.add_argument('--settings', help='actual設定JSONを明示指定')
    parser.add_argument('--name', required=True, help='新stage名prefix（01/02/03を付加）')
    parser.add_argument('--id-prefix', required=True)
    parser.add_argument('--output-base', help='出力親。省略時はbaseの親')
    parser.add_argument('--count', type=int, default=3, help='独立stage数（物理ターン回数ではない）')
    parser.add_argument('--date', default=datetime.now(timezone(timedelta(hours=9))).strftime('%Y%m%d'))
    parser.add_argument('--basis', action='append', default=[], help='候補の根拠資料。複数指定可')
    for key in ('correction', 'r-min', 'lstart', 'lend'):
        parser.add_argument('--' + key, type=float)
    args = parser.parse_args()
    base, source_run = inside(args.base), inside(args.source_run)
    output = inside(args.output_base) if args.output_base else base.parent
    settings_path = inside(args.settings) if args.settings else (
        inside(source_run / 'saved_1ms.settings.json') if source_run.is_dir() else source_run)
    basis = [inside(value) for value in args.basis]
    if not 1 <= args.count <= 99:
        raise ValueError('--countは1〜99のstage数です')
    if not re.fullmatch(r'[A-Za-z0-9_-]+', args.name):
        raise ValueError('--nameには英数字・_・-だけを指定してください')
    if not re.fullmatch(r'[A-Za-z0-9_-]+', args.id_prefix):
        raise ValueError('--id-prefixには英数字・_・-だけを指定してください')
    if not re.fullmatch(r'\d{8}', args.date):
        raise ValueError('--dateはYYYYMMDDです')
    datetime.strptime(args.date, '%Y%m%d')
    stages = [inside(output / f'{args.name}{i:02d}') for i in range(1, args.count + 1)]
    # 全出力の存在確認を最初に行い、衝突時は一切書き込まない。
    if any(path.exists() for path in stages):
        raise FileExistsError('既存stageは上書きしません: ' + ', '.join(str(p) for p in stages if p.exists()))
    if any(path.is_relative_to(base) or base.is_relative_to(path) for path in stages):
        raise ValueError('コピー元stageと出力stageを入れ子にはできません')
    copy_files = COPY_FILES + tuple(name for name in OPTIONAL_COPY_FILES if (base / name).is_file())
    active_gate = inside(ROOT / 'tools/analysis/pixel_camera_static_pose.py')
    source_files = [inside(base / name) for name in (*copy_files, 'run_recorded_once.py', 'placement_plan.json')]
    source_files.append(active_gate)
    source_files += [settings_path, *basis]
    for path in source_files:
        if not path.is_file():
            raise FileNotFoundError(path)
    before_hashes = {str(path): digest(path) for path in source_files}
    settings = read(settings_path)
    settings = settings.get('actual_settings', settings)
    missing = [key for key in KEYS if key not in settings]
    if missing:
        raise ValueError(f'actual設定に必須キーがありません: {missing}')
    candidate = {key: deepcopy(settings[key]) for key in KEYS}
    candidate['turn_count'] = 1
    changes = {key: getattr(args, key) for key in ('correction', 'r_min', 'lstart', 'lend')
               if getattr(args, key) is not None}
    if not all(math.isfinite(value) for value in changes.values()):
        raise ValueError('geometry変更値は有限値で指定してください')
    candidate['parameters'].update(changes)
    source_wrapper = (base / 'run_recorded_once.py').read_text(encoding='utf-8-sig')
    wrapper_info(source_wrapper)
    config = read(base / 'config.json')
    for key in ('reference', 'geometry'):
        inside(config[key])
    sys.path.insert(0, str(ROOT / 'tools'))
    from pixel_camera_tuning_mcp import McpTuningService
    from pixel_camera_tuning_cli import request_id
    identifiers = [request_id(f'{args.id_prefix}_{args.date}_{i:03d}') for i in range(1, args.count + 1)]
    prepared = []
    for stage, identifier in zip(stages, identifiers):
        wrapper, placement, diagonal, straight = change_default(source_wrapper, identifier)
        prepared.append((stage, identifier, wrapper, placement, diagonal, straight))
    output.mkdir(parents=True, exist_ok=True)
    summary = []
    for stage, identifier, wrapper, placement, diagonal, straight in prepared:
        stage.mkdir()
        for filename in copy_files:
            shutil.copyfile(base / filename, stage / filename)
        shutil.copyfile(active_gate, stage / 'static_gate_active_source.py')
        (stage / 'run_recorded_once.py').write_text(wrapper, encoding='utf-8', newline='\n')
        write(stage / 'candidate.settings.json', candidate)
        plan = read(base / 'placement_plan.json')
        plan.update(candidate=str(stage / 'candidate.settings.json'), config=str(stage / 'config.json'),
                    request_id=identifier, next_request_id=identifier, next_unsent_request_id=identifier,
                    operator_ready=False, next_candidate_executed=False,
                    execution_pending=False, current_placement_confirmed=False,
                    current_clearance_confirmed=False, count=1)
        write(stage / 'placement_plan.json', plan)
        phase = read(base / 'phase.json') if (base / 'phase.json').is_file() else {}
        phase.update(state='PREPARED_NOT_EXECUTED', candidate=str(stage / 'candidate.settings.json'),
                     config=str(stage / 'config.json'), request_id=identifier, next_request_id=identifier,
                     operator_ready=False, next_candidate_executed=False, execution_pending=False,
                     source_run=str(source_run), source_settings=str(settings_path),
                     source_stage=str(base), basis=[str(path) for path in basis],
                     geometry_changes=changes, turn_count=1)
        write(stage / 'phase.json', phase)
        # 公式prepareのみ。submit/perform/LiveSessionやwrapperの実行は行わない。
        service = McpTuningService(stage / 'local_execution', {})
        payload = service.prepare(str(stage / 'config.json'), str(stage / 'candidate.settings.json'),
                                  placement, diagonal, straight)
        if payload['settings'] != candidate:
            raise RuntimeError('prepare結果の設定が候補と一致しません')
        for key in ('pid', 'feedforward', 'suction'):
            if candidate[key] != settings[key]:
                raise RuntimeError(f'{key}がsource actualと一致しません')
        files = [p.relative_to(stage).as_posix() for p in stage.rglob('*') if p.is_file()]
        if any('operator_ready' in p or 'transport' in p or p.endswith('.stdout.txt')
               or p.startswith('local_execution/jobs/') for p in files):
            raise RuntimeError('実行用の出力を検出しました')
        result = dict(stage=str(stage), request_id=identifier, state='VALIDATED_NOT_EXECUTED',
                      operator_ready=False, turn_count=1, preserved=['pid', 'feedforward', 'suction'],
                      placement=placement, post_diagonal_accel_m_s2=diagonal,
                      post_straight_accel_m_s2=straight, geometry_changes=changes,
                      source_hashes=before_hashes, basis=[str(path) for path in basis],
                      source_unchanged=all(digest(Path(path)) == value for path, value in before_hashes.items()),
                      no_execution_job_or_transport=True)
        write(stage / 'offline_validation.json', result)
        summary.append(result)
    print(json.dumps(dict(stages=summary, physical_execution=False), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
