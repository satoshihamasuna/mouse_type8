"""機体A・1800左右180度の暫定値を局所反映する。書込み・走行は行わない。"""
from pathlib import Path
import difflib
import hashlib
import json
import shutil

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_r180_20261120_v1'
OUT=BASE/'firmware_RL180_adoption_20261121_v1'
assert not OUT.exists()
selected=json.loads((BASE/'selection_20261121_v1/selected.settings.json').read_text(encoding='utf-8'))
assert selected['parameters']==dict(velo=1.8,r_min=-48.2,lstart=7.52,lend=31.5,degree=-180.,correction=2.,pre_accel=6.5)
old_cpp='''#if PARAM_A_USE_PINK_LED_TURN_LENGTHS
// Pink LED interval (20260726 Lstart=Lend=0 videos, R median copied to L/R).
const t_turn_param_table slalom_L180_1800_table = {1.80f, 48.0f,7.52,37.68, 180.0f,Turn_L};
const t_turn_param_table slalom_R180_1800_table = {1.80f,-48.0f,7.52,37.68,-180.0f,Turn_R};
#else
const t_turn_param_table slalom_L180_1800_table = {1.80f, 48.0f,7.52,37.68, 180.0f,Turn_L};
const t_turn_param_table slalom_R180_1800_table = {1.80f,-48.0f,7.52,37.68,-180.0f,Turn_R};
#endif'''
new_cpp='''// 2026-11-21: 機体A・1800 R180の実測済み暫定値を依頼により左右へ採用。
// 根拠: tools/turn_analysis/20261121_A_turn1800_RL180_adoption/README.md
// 左は右の鏡映で実機未確認。論理角±180度を維持し、補正の符号を反転する。
// 幅約3 mmの残差・速度変動・連続接続は未解決。旧マーカー長切替から独立。
const t_turn_param_table slalom_L180_1800_table = {1.80f, 48.2f,7.52f,31.5f, 180.0f,Turn_L,-2.0f};
const t_turn_param_table slalom_R180_1800_table = {1.80f,-48.2f,7.52f,31.5f,-180.0f,Turn_R, 2.0f};'''
old_h='''#if PARAM_A_USE_PINK_LED_TURN_LENGTHS
// Pink LED interval (20260726 Lstart=Lend=0 videos, R median copied to L/R).
extern const t_turn_param_table slalom_L180_1800_table;
extern const t_turn_param_table slalom_R180_1800_table;
#else
extern const t_turn_param_table slalom_L180_1800_table;
extern const t_turn_param_table slalom_R180_1800_table;
#endif'''
new_h='''// 2026-11-21採用の暫定値。左右の角度補正は鏡映し、旧マーカー長切替から独立。
extern const t_turn_param_table slalom_L180_1800_table;
extern const t_turn_param_table slalom_R180_1800_table;'''
edits=[]
for name,old,new in (('turn_1800.cpp',old_cpp,new_cpp),('turn_1800.h',old_h,new_h)):
    path=ROOT/'Params/Param_A/turn_1800'/name
    before=path.read_bytes();newline='\r\n' if b'\r\n' in before else '\n'
    a=old.replace('\n',newline).encode('utf-8');b=new.replace('\n',newline).encode('utf-8')
    assert before.count(a)==1,name
    edits.append((path,before,before.replace(a,b),newline))
OUT.mkdir()
for path,before,after,newline in edits:
    (OUT/(path.name+'.before')).write_bytes(before)
    path.write_bytes(after)
    (OUT/(path.name+'.after')).write_bytes(after)
    (OUT/(path.name+'.this_task.diff')).write_text(''.join(difflib.unified_diff(
        before.decode('utf-8').splitlines(keepends=True),after.decode('utf-8').splitlines(keepends=True),
        fromfile=path.name+'.before',tofile=path.name+'.after')),encoding='utf-8')
preparation=dict(mouse='A',scope='Only A left/right180 table definitions and declarations changed',
    user_request='左もその値を採用で',left_physical_test_performed=False,firmware_flashing_performed=False,
    selected_settings=str(BASE/'selection_20261121_v1/selected.settings.json'),
    sources={str(p):dict(before_sha256=hashlib.sha256(b).hexdigest(),after_sha256=hashlib.sha256(a).hexdigest(),newline=nl) for p,b,a,nl in edits},
    B_sha256=hashlib.sha256((ROOT/'Params/Param_B/turn_1800/turn_1800.cpp').read_bytes()).hexdigest())
with (OUT/'preparation.json').open('x',encoding='utf-8') as f:json.dump(preparation,f,ensure_ascii=False,indent=2)
template=(ROOT/'tmp/build_A_in45_lr_adoption_20261121.py').read_text(encoding='utf-8')
old="tools/logs/pixel_camera/turn1800_A_inr45_20261120_v1/firmware_in45_lr_adoption_20261121_v1"
assert template.count(old)==1
build=ROOT/'tmp/build_A_RL180_adoption_20261121.py'
assert not build.exists()
build.write_text(template.replace(old,OUT.relative_to(ROOT).as_posix()),encoding='utf-8')
print('A left/right180 tables updated; outside bytes and existing direction-specific gains preserved.')
