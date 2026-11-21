"""作成済みA1800 out_r45を公式prepareで照合し、実行せず結果を保存する。"""
from pathlib import Path
import ast
import hashlib
import json
import sys

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_outr45_20261120_v1'
STAGE=BASE/'baseline01'
sys.path.insert(0,str(ROOT/'tools'))
from pixel_camera_tuning_mcp import McpTuningService

read=lambda p:json.loads(p.read_text(encoding='utf-8'))
plan=read(STAGE/'placement_plan.json')
source=read(BASE/'source_extraction.json')
candidate=read(STAGE/'candidate.settings.json')
wrapper=(STAGE/'run_recorded_once.py').read_text(encoding='utf-8')
ast.parse(wrapper);ast.parse((STAGE/'static_pose_after_settle.py').read_text(encoding='utf-8'))
assert "default='A_OUTR45_1800_baseline_20261120_001'" in wrapper
assert 'placement, None, 20.)' in wrapper and 'stationary_frame_stride=4' in wrapper
assert 'position_tolerance_mm=3.5' in wrapper and 'heading_deg=133.5' in wrapper
service=McpTuningService(STAGE/'local_execution',{})
payload=service.prepare(str(STAGE/'config.json'),str(STAGE/'candidate.settings.json'),plan['placement'],None,20.)
assert payload['settings']==candidate and payload['post_straight_accel_m_s2']==20.
assert payload['post_diagonal_accel_m_s2'] is None and 'operator_ready' not in payload
assert candidate['parameters']['r_min']==-55 and candidate['parameters']['lstart']==22.66
assert candidate['feedforward']['ff_sp_turn_accel_mag']==candidate['feedforward']['ff_sp_turn_velo_sq']==0.
for p,digest in source['sources_sha256'].items():
    assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==digest
files=[p.relative_to(STAGE).as_posix() for p in STAGE.rglob('*') if p.is_file()]
assert not any(p.startswith('local_execution/jobs/') or 'transport' in p for p in files)
record=dict(state='VALIDATED_NOT_EXECUTED',mouse='A',motion='out_r45',
    request_id=plan['next_request_id'],official_McpTuningService_prepare_passed=True,
    wrapper_without_operator_ready_passed=True,ast_parse_passed=True,
    payload=payload,operator_ready=False,hardware_operations=0,
    no_execution_job_or_transport=True,source_hashes_unchanged=True,
    current_placement_confirmed=False,current_clearance_confirmed=False,current_hands_clear_confirmed=False,
    fresh_device_show_verified=False,source_profile_fresh_readback_required=True,
    source_profile_sha256=source['sources_sha256'][str(ROOT/'Params/Param_A/turn_1800/turn_1800.cpp')],
    candidate_sha256=hashlib.sha256((STAGE/'candidate.settings.json').read_bytes()).hexdigest())
with (STAGE/'offline_validation.json').open('x',encoding='utf-8') as f:
    json.dump(record,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')
print('MCP prepare, wrapper syntax, source hashes and non-execution checks passed',flush=True)
