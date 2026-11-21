from pathlib import Path
from datetime import datetime,timezone
import hashlib
import json
import shutil

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_outr45_20261120_v1'
AUDIT=BASE/'firmware_out45_lr_adoption_20261121_v1'
DOC=ROOT/'tools/turn_analysis/20261121_A_turn1800_OUTR45_adoption'
AUDIT.mkdir(exist_ok=False)
files=['Params/Param_A/turn_1800/turn_1800.cpp','Params/Param_A/turn_1800/turn_1800.h',
 'Params/Param_B/turn_1800/turn_1800.cpp','Params/Param_B/turn_1800/turn_1800.h','README.md',
 'tools/turn_analysis/20261121_A_turn1800_OUTR45_adoption/README.md',
 'tools/turn_analysis/20261121_A_turn1800_OUTR45_adoption/adoption.json']
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
before={}
for name in files:
    source=ROOT/name;target=AUDIT/'before'/name;target.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(source,target);before[name]=sha(source)
def replace(name,old,new):
    path=ROOT/name;raw=path.read_bytes();assert sha(path)==before[name]
    nl='\r\n' if b'\r\n' in raw else '\n';a=old.replace('\n',nl).encode();b=new.replace('\n',nl).encode()
    assert raw.count(a)==1,(name,raw.count(a));path.write_bytes(raw.replace(a,b))
old='''#if PARAM_A_USE_PINK_LED_TURN_LENGTHS
// Pink LED interval (20260725 Lstart=Lend=0 videos, R median copied to L/R).
const t_turn_param_table slalom_outL45_1800_table = {1.80f, 55.0f,23.63,23.69, 45.0f,Turn_L};
#else
const t_turn_param_table slalom_outL45_1800_table = {1.80f, 55.0f,22.66,26.00, 45.0f,Turn_L};
#endif
// 2026-11-21: 機体A・右45度退出1800の実測2本から、依頼により暫定採用。
// 根拠: tools/turn_analysis/20261121_A_turn1800_OUTR45_adoption/README.md
// 入口相対の出口44.708〜45.430度、位置残差2.018〜3.692 mm。連続接続は未検証。
// 右だけ旧マーカー長切替から独立。論理角-45度、物理指令角-46.7度。
const t_turn_param_table slalom_outR45_1800_table = {1.80f,-55.0f,15.4f,15.9f,-45.0f,Turn_R,-1.7f};'''
new='''// 2026-11-21: 機体A・右45度退出1800の実測2本から暫定採用。左も依頼により鏡映。
// 根拠: tools/turn_analysis/20261121_A_turn1800_OUTR45_adoption/README.md
// 右の入口相対出口44.708〜45.430度、位置残差2.018〜3.692 mm。左の実走は未確認。
// 左右とも旧マーカー長切替から独立。論理角±45度、物理指令角±46.7度。
// 方向別PID・FFは既存の参照を維持する。連続接続は未検証。
const t_turn_param_table slalom_outL45_1800_table = {1.80f, 55.0f,15.4f,15.9f, 45.0f,Turn_L, 1.7f};
const t_turn_param_table slalom_outR45_1800_table = {1.80f,-55.0f,15.4f,15.9f,-45.0f,Turn_R,-1.7f};'''
replace(files[0],old,new)
old='''#if PARAM_A_USE_PINK_LED_TURN_LENGTHS
// Pink LED interval (20260725 Lstart=Lend=0 videos, R median copied to L/R).
extern const t_turn_param_table slalom_outL45_1800_table;
#else
extern const t_turn_param_table slalom_outL45_1800_table;
#endif
// 右45度退出は撮影方式フラグによらず、turn_1800.cppの暫定採用値を使う。
extern const t_turn_param_table slalom_outR45_1800_table;'''
new='''// 左右45度退出は撮影方式フラグによらず、turn_1800.cppの暫定採用値を使う。
extern const t_turn_param_table slalom_outL45_1800_table;
extern const t_turn_param_table slalom_outR45_1800_table;'''
replace(files[1],old,new)
for name in files[2:4]:assert sha(ROOT/name)==before[name]
data=json.loads((DOC/'adoption.json').read_text(encoding='utf-8'))
data['right_only_validation']=data['validation'];data['status']='PROVISIONAL_LR_SOURCE_UPDATED_VALIDATION_PENDING'
data['left_mirror_request']='ひだりもさいようして'
data['left_parameters']=dict(velo=1.8,r_min=55.,lstart=15.4,lend=15.9,degree=45.,degree_correction=1.7,physical_degree=46.7)
data['left_mirror_basis']='User-requested mirror of measured right geometry; preserve original left directional PID/FF.'
data['left_hardware_verified']=False;data['left_measured_runs']=0
data['left_added_at_utc']=datetime.now(timezone.utc).isoformat()
(DOC/'adoption.json').write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
text=(DOC/'README.md').read_text(encoding='utf-8')
text=text.replace('# 機体A・右45度出口1800の暫定採用','# 機体A・左右45度出口1800の暫定採用',1)
row='| 機体A・右45度出口 | 1800 mm/s | -55.0 mm | 15.4 mm | 15.9 mm | -45.0° | -1.7° | -46.7° |'
assert text.count(row)==1;text=text.replace(row,row+'\n| 機体A・左45度出口 | 1800 mm/s | +55.0 mm | 15.4 mm | 15.9 mm | +45.0° | +1.7° | +46.7° |',1)
old='PID・FFは測定時と同じ既存の右45度出口の参照を維持した。右だけ撮影方式フラグから独立させ、`mode_1800`と`mode_1800_acc_v2`へ反映する。`mode_1800_acc_v1`のOUT45は従来どおり無効。左側は今回変更していない。'
new='「ひだりもさいようして」の追加依頼により、左は右の形状を鏡映して追加した。左右とも撮影方式フラグから独立させ、既存の方向別PID・FF参照を維持する。`mode_1800`と`mode_1800_acc_v2`へ反映し、`mode_1800_acc_v1`のOUT45は従来どおり無効。左の実走は未確認であり、右の残差や再現性を左の実測結果とは扱わない。'
assert text.count(old)==1;text=text.replace(old,new,1).replace('Debug-MouseAのConfigure/Buildは成功。','右単独反映時は、Debug-MouseAのConfigure/Buildは成功。',1)
text+='\n\n## 左鏡映追加の検証\n\n機体AのDebugビルド・参照テスト・左右のELF値と補正符号を確認中。左の実走および本体書き込みは未実施。\n'
(DOC/'README.md').write_text(text,encoding='utf-8')
with (AUDIT/'preparation.json').open('x',encoding='utf-8') as file:json.dump(dict(before_sha256=before,
    source_after_application={name:sha(ROOT/name) for name in files[:2]},user_request='ひだりもさいようして'),file,ensure_ascii=False,indent=2)
print('Applied A left OUT45 mirror; right geometry and both directional PID/FF preserved.',flush=True)
