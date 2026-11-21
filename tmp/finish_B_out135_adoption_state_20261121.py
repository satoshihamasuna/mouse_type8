"""採用後の再開先を測定済み31.0mmへそろえ、資料のリンクを検査する。"""
from pathlib import Path
import hashlib
import json
import re

root=Path(__file__).resolve().parents[1]
base=root/'tools/logs/pixel_camera/turn1600_B_outr135_20261121_resume_v1'
out=base/'firmware_OUT135_adoption_20261121_v1'
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
adoption=read(out/'adoption.json')
assert hashlib.sha256(Path(adoption['source']).read_bytes()).hexdigest()==adoption['source_sha256']
candidate=read(base/'standard_flow_c1p0_Ls5p3_Le31_01/candidate.settings.json')
assert candidate['parameters']['lstart']==5.3 and candidate['parameters']['lend']==31.0
assert candidate['parameters']['correction']==1.0
candidate_path=out/'adopted_right_candidate.settings.json'
with candidate_path.open('x',encoding='utf-8') as stream:
    json.dump(candidate,stream,ensure_ascii=False,indent=2);stream.write('\n')
state_path=base/'resume_state.json';state=read(state_path)
state.update(candidate=str(candidate_path),config=str(base/'standard_flow_c1p0_Ls5p3_Le31_01/config.json'),
    superseded_unexecuted_candidate=str(base/'refine_c1p0_Ls5p3_Le29p2_01/candidate.settings.json'),
    operator_ready=False,execution_pending=False,current_placement_confirmed=False,
    current_hands_clear_confirmed=False,next_stage=None,next_request_id=None)
state_path.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
doc=root/'tools/turn_analysis/20261121_B_turn1600_OUT135_adoption/README.md'
links=re.findall(r'\]\(([^)]+)\)',doc.read_text(encoding='utf-8'))
assert links and all((doc.parent/link.split('#')[0]).resolve().is_file() for link in links)
check=dict(source_hash_unchanged=True,document_links_checked=len(links),
    resume_candidate_Lend_mm=candidate['parameters']['lend'],unused_29p2_candidate_executed=False,
    additional_hardware_operations=0,firmware_write_performed=False)
with (out/'completion_checks.json').open('x',encoding='utf-8') as stream:
    json.dump(check,stream,ensure_ascii=False,indent=2)
print(json.dumps(check,ensure_ascii=False),flush=True)
