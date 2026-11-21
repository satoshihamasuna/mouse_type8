from pathlib import Path
from datetime import datetime,timezone
import hashlib
import json
import math
import shutil

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_outr45_20261120_v1'
OUT=ROOT/'tools/turn_analysis/20261121_A_turn1800_OUTR45_adoption'
AUDIT=BASE/'firmware_outR45_adoption_20261121_v1'
OUT.mkdir(exist_ok=False);AUDIT.mkdir(exist_ok=False)
def read(path):return json.loads(path.read_text(encoding='utf-8'))
def sha(path):
    with path.open('rb') as file:return hashlib.file_digest(file,'sha256').hexdigest()
def save(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('x',encoding='utf-8') as file:json.dump(value,file,ensure_ascii=False,indent=2);file.write('\n')
files=['Params/Param_A/turn_1800/turn_1800.cpp','Params/Param_A/turn_1800/turn_1800.h',
       'Params/Param_B/turn_1800/turn_1800.cpp','Params/Param_B/turn_1800/turn_1800.h','README.md']
before={}
for name in files:
    source=ROOT/name;destination=AUDIT/'before'/name;destination.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(source,destination);before[name]=sha(source)
save(AUDIT/'before_sha256.json',before)

measurements=[];input_hashes={}
for i,name in enumerate(('lengths_cminus1p7_Ls15p4_Le15p9_20261121_01','repeat_cminus1p7_Ls15p4_Le15p9_20261121_02'),1):
    stage=BASE/name;review=read(stage/'midpoint_brake_clock_v1/midpoint_review.json')
    run=Path(review['source_trial']).parent;actual=read(run/'saved_1ms.settings.json')
    assert actual['mouse']=='A' and actual['parameters']['lstart']==15.4 and actual['parameters']['lend']==15.9
    assert actual['parameters']['correction']==-1.7 and review['independent_optical_geometry_eligible']
    error=math.hypot(*review['midpoint_Lend_residual_mm'])
    measurements.append(dict(stage=name,trial=read(run/'trial.json')['recording_id'],
        angle_deg=review['exit_path_direction_change_deg'],residual_mm=review['midpoint_Lend_residual_mm'],
        error_norm_mm=error,battery_min_v=read(stage/'tracking_1ms_review_v1/verification.json')['battery_min_v']))
    sources={'trial.json':run/'trial.json','saved_1ms.csv':run/'saved_1ms.csv','saved_1ms.settings.json':run/'saved_1ms.settings.json',
        'midpoint_review.json':stage/'midpoint_brake_clock_v1/midpoint_review.json',
        'tracking_verification.json':stage/'tracking_1ms_review_v1/verification.json',
        'runtime_modes.json':stage/'local_execution/runtime_modes.json'}
    for dest,source in sources.items():
        target=OUT/'measurements'/f'run{i}'/dest;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,target)
        input_hashes[str(source)]=sha(source);assert sha(target)==sha(source)

def replace_bytes(name,old,new):
    path=ROOT/name;data=path.read_bytes();assert sha(path)==before[name]
    newline='\r\n' if b'\r\n' in data else '\n'
    old_bytes=old.replace('\n',newline).encode('utf-8');new_bytes=new.replace('\n',newline).encode('utf-8')
    assert data.count(old_bytes)==1,(name,data.count(old_bytes))
    path.write_bytes(data.replace(old_bytes,new_bytes))

old_cpp='''#if PARAM_A_USE_PINK_LED_TURN_LENGTHS
// Pink LED interval (20260725 Lstart=Lend=0 videos, R median copied to L/R).
const t_turn_param_table slalom_outL45_1800_table = {1.80f, 55.0f,23.63,23.69, 45.0f,Turn_L};
const t_turn_param_table slalom_outR45_1800_table = {1.80f,-55.0f,23.63,23.69,-45.0f,Turn_R};
#else
const t_turn_param_table slalom_outL45_1800_table = {1.80f, 55.0f,22.66,26.00, 45.0f,Turn_L};
const t_turn_param_table slalom_outR45_1800_table = {1.80f,-55.0f,22.66,26.00,-45.0f,Turn_R};
#endif'''
new_cpp='''#if PARAM_A_USE_PINK_LED_TURN_LENGTHS
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
replace_bytes(files[0],old_cpp,new_cpp)
old_h='''#if PARAM_A_USE_PINK_LED_TURN_LENGTHS
// Pink LED interval (20260725 Lstart=Lend=0 videos, R median copied to L/R).
extern const t_turn_param_table slalom_outL45_1800_table;
extern const t_turn_param_table slalom_outR45_1800_table;
#else
extern const t_turn_param_table slalom_outL45_1800_table;
extern const t_turn_param_table slalom_outR45_1800_table;
#endif'''
new_h='''#if PARAM_A_USE_PINK_LED_TURN_LENGTHS
// Pink LED interval (20260725 Lstart=Lend=0 videos, R median copied to L/R).
extern const t_turn_param_table slalom_outL45_1800_table;
#else
extern const t_turn_param_table slalom_outL45_1800_table;
#endif
// 右45度退出は撮影方式フラグによらず、turn_1800.cppの暫定採用値を使う。
extern const t_turn_param_table slalom_outR45_1800_table;'''
replace_bytes(files[1],old_h,new_h)
for name in files[2:4]:assert sha(ROOT/name)==before[name]

adoption=dict(schema_version=1,recorded_at_utc=datetime.now(timezone.utc).isoformat(),date_local='2026-11-21',
    status='PROVISIONAL_SOURCE_UPDATED_VALIDATION_PENDING',user_request='そろそろパラメータけっていできない?',
    request_context='継続中のファーム反映方針で、実測済みの最良候補を決定してソースへ暫定反映。',
    mouse='A',motion='out_r45',speed_mm_s=1800,
    parameters=dict(velo=1.8,r_min=-55.,lstart=15.4,lend=15.9,degree=-45.,degree_correction=-1.7,physical_degree=-46.7),
    conditions=dict(suction_duty=650,pre_accel_m_s2=6.5,post_straight_accel_m_s2=20.,camera_height_mm=1500),
    measurements=measurements,position_goal_2mm_met=False,provisional_selection=True,
    decision='13本の実走中、同条件2本で角度45±0.5度を満たした15.4/15.9mmを採用。17.0/17.4mmは2本とも約46.7度で除外。',
    limitations=['入口方向へ合わせた相対マーカー中点の単体評価。盤面固定の設置精度、絶対車軸・機軸は未校正。',
        '最大残差3.69mmで、2mm目標は未達。暫定採用であり完全調整済みとは扱わない。',
        '左側、連続/最短接続、更新ファーム書き込み後の走行は未確認。'],
    source_files=files[:2],input_sha256=input_hashes,firmware_written=False,new_ekf_parameters_adopted=False)
save(OUT/'adoption.json',adoption)
with (OUT/'README.md').open('x',encoding='utf-8') as file:file.write('''# 機体A・右45度出口1800の暫定採用

2026-11-21の「そろそろパラメータけっていできない?」に対し、これ以上の単発追込みを続けず、実測済みの最良候補を決定してファームソースへ反映した。

| 対象 | 速度 | 半径 | Lstart | Lend | 論理角 | 補正 | 物理指令角 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 機体A・右45度出口 | 1800 mm/s | -55.0 mm | 15.4 mm | 15.9 mm | -45.0° | -1.7° | -46.7° |

PID・FFは測定時と同じ既存の右45度出口の参照を維持した。右だけ撮影方式フラグから独立させ、`mode_1800`と`mode_1800_acc_v2`へ反映する。`mode_1800_acc_v1`のOUT45は従来どおり無効。左側は今回変更していない。

## 採用根拠

機体A、吸引650、前加速度6.5 m/s²、後直進20 m/s²、カメラ高さ概算1500 mm。各フレームの登録ArUcoと前赤・後黄の実検出から高さ補正した相対マーカー中点を求め、実測入口原点・方向に合わせて評価した。1 ms保持ログとMCU時刻を照合し、入口raw wheelだけで動画時計を合わせた。出口・終点・尺度へのフィットは行っていない。

| 実測 | 出口角 | Lend右残差 | Lend前残差 | 残差距離 |
| --- | ---: | ---: | ---: | ---: |
| 1本目 | 45.430° | -0.967 mm | -3.563 mm | 3.692 mm |
| 2本目 | 44.708° | -1.105 mm | -1.689 mm | 2.018 mm |

2本とも角度45±0.5度を満たす。位置2mm以内の目標は未達であり、最大3.69mmの残差を残した暫定採用とする。17.0/17.4mmは46.713°・46.636°と2本とも回り過ぎたため採らない。速度・電圧・設置が完全一致する比較ではなく、統計的優位や連続走行の検証とは扱わない。

元の`parameter_adoption_allowed:false`やnativeの失敗判定は変更しない。今回の実用上の暫定決定を別記録する。最終的な盤面固定の接続、絶対車軸/機軸、左、連続/最短は未確認。入口の設置傾きは調整値へ埋め込まない。EKF係数は変更していない。

- [採用記録・入力SHA-256](adoption.json)
- [測定1の設定・ログ・解析](measurements/run1/)
- [測定2の設定・ログ・解析](measurements/run2/)
- [全測定の経過](../../logs/pixel_camera/turn1800_A_outr45_20261120_v1/README.md)

## 検証

機体AのDebugビルド・既存リンクテスト・ELF内の値と参照を確認中。結果は追記する。本体Flashへの書き込みは未実施。
''')
save(AUDIT/'preparation.json',dict(adoption_folder=str(OUT),before_sha256=before,
    source_sha256_after_application={name:sha(ROOT/name) for name in files[:2]},selected_measurements=measurements))
print(json.dumps(dict(source_updated=True,target='A OUTR45 1800',selected_measurements=measurements),ensure_ascii=False),flush=True)
