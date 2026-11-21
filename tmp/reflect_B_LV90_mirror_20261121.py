"""ユーザー指定によりB1600左V90を右の暫定値から鏡映する。実機操作なし。"""
from pathlib import Path
from datetime import datetime, timezone
import difflib, hashlib, json

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'tools/turn_analysis/20261121_B_turn1600_LV90_mirror'
BACKUP=ROOT/'tmp/B_LV90_source_before_mirror_20261121'
OUT.mkdir(exist_ok=True)
assert not (OUT/'evidence.json').exists(), '既存の反映記録は上書きしない'
BACKUP.mkdir(exist_ok=False)
cpp=ROOT/'Params/Param_B/turn_1600/turn_1600.cpp'
header=ROOT/'Params/Param_B/turn_1600/turn_1600.h'
before={p:p.read_bytes() for p in (cpp,header)}
def replace(data,old,new):
    old=old.replace('\n','\r\n').encode('utf-8')
    new=new.replace('\n','\r\n').encode('utf-8')
    assert data.count(old)==1,old
    return data.replace(old,new)

data=replace(before[cpp],
    '// 根拠・残差・未確認事項: tools/turn_analysis/20261121_B_turn1600_RV90_adoption/README.md',
    '// 根拠・残差・未確認事項: tools/turn_analysis/20261121_B_turn1600_RV90_adoption/README.md\n'
    '// 左V90はユーザー指定で長さ・FFを鏡映し、補正+1.20度。左の実走は未確認。\n'
    '// 左反映の記録: tools/turn_analysis/20261121_B_turn1600_LV90_mirror/README.md')
data=replace(data,'const t_ff_gain ff_gain_turnRV90_1600 = {',
    'const t_ff_gain ff_gain_turnLV90_1600 = {\n'
    '\t0.661202f,0.052929f,0.0f,\n'
    '\t0.009256f,0.000800f,0.000700f,0.0f,\n'
    '\t0.0f,0.0f,0.0f\n'
    '};\nconst t_ff_gain ff_gain_turnRV90_1600 = {')
old='const t_turn_param_table slalom_LV90_1600_table = {1.60f, 40.0f,2.20f,20.83f, 90.0f,Turn_L,-1.60f};'
data=replace(data,old,'// '+old+'\nconst t_turn_param_table slalom_LV90_1600_table = {1.60f, 40.0f,6.0f,17.4f, 90.0f,Turn_L, 1.20f};')
data=replace(data,
    'const t_param param_LV90_1600 = {&slalom_LV90_1600_table,&sp_gain_turnV90_1600,&om_gain_turnV90_1600};',
    'const t_param param_LV90_1600 = {&slalom_LV90_1600_table,&sp_gain_turnV90_1600,&om_gain_turnV90_1600,&ff_gain_turnLV90_1600};')
after={cpp:data,header:replace(before[header],'extern const t_ff_gain ff_gain_turnRV90_1600;',
    'extern const t_ff_gain ff_gain_turnLV90_1600;\nextern const t_ff_gain ff_gain_turnRV90_1600;')}
changes={}
patch=[]
for path in before:
    rel=path.relative_to(ROOT)
    saved=BACKUP/rel
    saved.parent.mkdir(parents=True,exist_ok=True)
    saved.write_bytes(before[path])
    assert path.read_bytes()==before[path], '他の作業による変更を上書きしない'
    path.write_bytes(after[path])
    changes[str(rel)]=dict(before_sha256=hashlib.sha256(before[path]).hexdigest(),
        after_sha256=hashlib.sha256(after[path]).hexdigest(),backup=str(saved))
    patch.extend(difflib.unified_diff(before[path].decode('utf-8').splitlines(keepends=True),
        after[path].decode('utf-8').splitlines(keepends=True),fromfile='before/'+rel.as_posix(),tofile='after/'+rel.as_posix()))
(OUT/'source_patch.diff').write_text(''.join(patch),encoding='utf-8',newline='')
record=dict(recorded_at=datetime.now(timezone.utc).isoformat(),mouse='B',motion='l_v90',preset_mm_s=1600,
    user_instruction='ひだりもはんえいして',status='USER_REQUESTED_MIRROR_UNTESTED_LEFT',
    mirrored_from=str(ROOT/'tools/turn_analysis/20261121_B_turn1600_RV90_adoption/evidence.json'),
    parameters=dict(velo=1.6,r_min=40.0,lstart=6.0,lend=17.4,degree=90.0,correction=1.2),
    feedforward=dict(sp_velo=0.661202,sp_accel=0.052929,sp_bias=0.0,om_velo=0.009256,
        om_accel=0.000800,om_decel=0.000700,om_bias=0.0,om_jerk=0.0,sp_turn_accel_mag=0.0,sp_turn_velo_sq=0.0),
    left_motion_tested=False,right_angular_goal_passed=False,firmware_write_performed=False,
    hardware_operations=0,source_changes=changes,validation_state='PENDING')
(OUT/'evidence.json').write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
helper=(ROOT/'tmp/check_B_rv90_reflection_20261121.py').read_text(encoding='utf-8')
helper=helper.replace('source_validation_20261121','left_mirror_source_validation_20261121')
helper=helper.replace('B右V90の暫定反映','B左V90の鏡映反映')
(ROOT/'tmp/check_B_LV90_mirror_20261121.py').write_text(helper,encoding='utf-8')
print(json.dumps(dict(status=record['status'],source_changes=changes,hardware_operations=0),ensure_ascii=False))
