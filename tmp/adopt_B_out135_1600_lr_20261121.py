"""B1600左右out135へ測定済み31.0mm候補を反映。実機操作なし。"""
from pathlib import Path
from datetime import datetime, timezone
import difflib
import hashlib
import json
import re

root=Path(__file__).resolve().parents[1]
base=root/'tools/logs/pixel_camera/turn1600_B_outr135_20261121_resume_v1'
out=base/'firmware_OUT135_adoption_20261121_v1'
assert not out.exists()
source=root/'Params/Param_B/turn_1600/turn_1600.cpp'
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
measurement_path=base/'geometry_same_candidate_three_review_v2_manual_pairs/review.json'
measurement=read(measurement_path)
assert measurement['successful_same_candidate_runs']==3
assert measurement['settings']['parameters']==dict(velo=1.6,r_min=-41.0,lstart=5.3,lend=31.0,degree=-135.0,correction=1.0,pre_accel=6.5)
for row in measurement['runs']:
    assert sha(Path(row['geometry_review']))==row['geometry_review_sha256']
    assert row['required_observation_fraction']>=.95
    assert row['operator_result']['both_powered_off'] and row['operator_result']['contact_detected'] is False
out.mkdir()
before=source.read_bytes()
(out/'turn_1600.before.cpp').write_bytes(before)
newline=b'\r\n' if before.count(b'\r\n')==before.count(b'\n') else b'\n'
text=before.decode('utf-8')
names=['slalom_outR135_1600_table','slalom_outL135_1600_table']
changes={
 'const t_turn_param_table slalom_outR135_1600_table = {1.60f,-41.0f,1.00f,31.50f,-135.0f,Turn_R,1.90f};':
 'const t_turn_param_table slalom_outR135_1600_table = {1.60f,-41.0f,5.3f,31.0f,-135.0f,Turn_R,1.0f};',
 'const t_turn_param_table slalom_outL135_1600_table = {1.60f, 41.0f,1.00f,31.50f, 135.0f,Turn_L,-1.90f};':
 'const t_turn_param_table slalom_outL135_1600_table = {1.60f, 41.0f,5.3f,31.0f, 135.0f,Turn_L,-1.0f};',
}
after=before
for old,new in changes.items():
    assert after.count(old.encode())==1,old
    after=after.replace(old.encode(),b'// '+old.encode()+newline+new.encode())
anchor=b'//k = 250'+newline+b'const t_pid_gain sp_gain_turnOut135_1600'
assert after.count(anchor)==1
comment=('// 2026-11-21 B右135度出口の暫定採用値。1600 mm/s・吸引duty650の単体3本、Lstart/Lend = 5.3/31.0 mm。\n'
 '// 右補正+1.0度、左はユーザー指定の鏡映-1.0度。左の実走・連続接続、停止点差は未確認/残課題。\n'
 '// 根拠: tools/turn_analysis/20261121_B_turn1600_OUT135_adoption/README.md\n').encode().replace(b'\n',newline)
after=after.replace(anchor,comment+anchor)
def other_code(value):
    value=re.sub(r'/\*.*?\*/|//[^\n]*','',value,flags=re.S)
    for name in names:
        value,count=re.subn(r'const\s+t_turn_param_table\s+'+name+r'\s*=\s*\{[^}]+\};','',value)
        assert count==1,name
    return re.sub(r'\s+',' ',value).strip()
assert other_code(text)==other_code(after.decode('utf-8'))
assert source.read_bytes()==before,'反映前に別作業がソースを更新しました'
source.write_bytes(after)
(out/'turn_1600.after.cpp').write_bytes(after)
with (out/'our_source_change.patch').open('x',encoding='utf-8') as stream:
    stream.writelines(difflib.unified_diff(text.splitlines(keepends=True),after.decode('utf-8').splitlines(keepends=True),
        fromfile='turn_1600.before.cpp',tofile=str(source.relative_to(root))))
request=dict(state='SOURCE_UPDATED_BUILD_PENDING',recorded_at=datetime.now(timezone.utc).isoformat(),
    user_request='反映して，左もね',mouse='B',preset_mm_s=1600,source=str(source),
    source_before_sha256=sha(out/'turn_1600.before.cpp'),source_after_sha256=sha(source),
    changed_tables=names,other_active_code_unchanged=True,pid_and_ff_unchanged=True,
    right_measurement_record=str(measurement_path),right_measurement_record_sha256=sha(measurement_path),
    right_values=dict(velo=1.6,r_min=-41.0,lstart=5.3,lend=31.0,degree=-135.0,correction=1.0),
    left_values=dict(velo=1.6,r_min=41.0,lstart=5.3,lend=31.0,degree=135.0,correction=-1.0),
    left_source='ユーザー指定による右測定値の鏡映',left_physical_run_verified=False,
    continuous_run_verified=False,firmware_write_performed=False,
    unused_refinement_Lend_mm=29.2,unused_refinement_executed=False)
with (out/'request.json').open('x',encoding='utf-8') as stream:json.dump(request,stream,ensure_ascii=False,indent=2)
old_build=(root/'tmp/build_B_in135_adoption_20261120.py').read_text(encoding='utf-8-sig')
old_out="OUT=ROOT/'tools/logs/pixel_camera/turn1600_B_inr135_20261120_v1/firmware_IN135_adoption_20261120_v1'"
assert old_build.count(old_out)==1
new_build=old_build.replace(old_out,f'OUT=Path({str(out)!r})')
build_script=root/'tmp/build_B_out135_adoption_20261121.py'
assert not build_script.exists()
build_script.write_text(new_build,encoding='utf-8')
state_path=base/'resume_state.json';state=read(state_path)
state.update(state='OUT135_LR_SOURCE_ADOPTED_BUILD_PENDING',next_stage=None,next_request_id=None,
    next_request_sent=False,execution_pending=False,operator_ready=False,
    next_action_kind='source_adoption_validation_no_physical_run',
    next_action='Bビルドと左右out135のELF/ホスト参照を検証する。29.2mm候補の走行要求はユーザーの反映指定で取り下げ。',
    source_adopted=True,left_source_adopted_by_user_requested_mirror=True,left_physical_run_verified=False,
    source_adoption_request=str(out/'request.json'),firmware_write_performed=False,current_ram_firmware_updated=False,
    superseded_unexecuted_request_id='B_OUTR135_1600_c1p0_Ls5p3_Le29p2_refine_20261121_001')
state_path.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
phase_path=base/'refine_c1p0_Ls5p3_Le29p2_01/phase.json'
phase=read(phase_path);phase.update(state='NOT_EXECUTED_SUPERSEDED_BY_SOURCE_ADOPTION',operator_ready=False,
    execution_pending=False,next_request_id=None,reason='ユーザー指定で測定済み31.0mm候補を左右へソース反映する。')
phase_path.write_text(json.dumps(phase,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(request,ensure_ascii=False),flush=True)
