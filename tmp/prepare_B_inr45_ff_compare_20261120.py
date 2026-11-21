"""B1600 inR45の角加減速FFだけを比較する候補。実機コマンドは送らない。"""
from pathlib import Path
import copy
import hashlib
import json
import shutil

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr45_20261120_v1'
PREVIOUS=BASE/'baseline02_relative'
RUN=PREVIOUS/'local_execution/run_B_INR45_1600_relative_green_20261120_002'
OUT=BASE/'ff_om_accdec0p00075_01'
OUT.mkdir(exist_ok=False)
read=lambda p:json.loads(p.read_text(encoding='utf-8'))
def save(p,v):
    with p.open('x',encoding='utf-8') as f:json.dump(v,f,ensure_ascii=False,indent=2,allow_nan=False)
actual=read(RUN/'saved_1ms.settings.json')
candidate=read(PREVIOUS/'candidate.settings.json')
for key in ('parameters','pid','feedforward','suction','turn_count'):
    candidate[key]=copy.deepcopy(actual[key])
candidate['feedforward'].update(ff_om_accel=.000750,ff_om_decel=.000750)
candidate['source']='B1600 inR45単発の加減速追従診断から、角加減速FFを共通倍率で約2倍にする未実行比較候補。速度FF・PID・形状・吸引は実機値を保持。'
save(OUT/'candidate.settings.json',candidate)
shutil.copyfile(PREVIOUS/'config.json',OUT/'config.json')
plan=read(PREVIOUS/'placement_plan.json')
identifier='B_INR45_1600_ffaccdec750_green_20261120_003'
plan.update(request_id=identifier,operator_ready=False,power_state='BOTH_OFF_USER_CONFIRMED',
    current_placement_confirmed=False,execution_commands_sent=0,settings_write_commands_sent=0,
    precise_static_pose_gate_used=False,coarse_position_tolerance_mm=10.,coarse_heading_tolerance_deg=5.,
    comparison=dict(changed_fields=['feedforward.ff_om_accel','feedforward.ff_om_decel'],
        shared_physical_factor='angular_acceleration_deceleration_feedforward_multiplier',
        before=.000378,after=.000750,source_trial=str(RUN/'trial.json'),
        diagnosis='立上り遅れとピーク超過。単発閉ループ推定は約0.00110だが各相の観測不足警告を保持し、中間候補で比較する。'))
save(OUT/'placement_plan.json',plan)
source=ROOT/'tools/logs/pixel_camera/turn1600_B_inr135_20261120_v1/geometry_c0p4_Ls10p5_Le17p2_02'
script=(source/'run_recorded_once.py').read_text(encoding='utf-8')
script=script.replace('in_r135','in_r45').replace('B135度入口','B45度入口')
script=script.replace('x_mm=315., y_mm=135.','x_mm=225., y_mm=45.')
script=script.replace('B_INR135_1600_c0p4_Ls10p5_Le17p2_green_20261120_002',identifier)
# 実機presetのFFを読む間は旧値を許す。set後の全値照合はcapture_connectedが行う。
script=script.replace("if abs(actual['feedforward'][key]-value) > 0.000002]",
    "if key not in ('ff_om_accel','ff_om_decel') and abs(actual['feedforward'][key]-value) > 0.000002]")
(OUT/'run_recorded_once.py').write_text(script,encoding='utf-8')
shutil.copyfile(source/'coarse_placement_check.py',OUT/'coarse_placement_check.py')
save(OUT/'candidate_basis.json',dict(state='PREPARED_NOT_EXECUTED',settings=candidate,
    tracking=read(RUN/'tracking_1ms_review_v3/verification.json'),
    ff_diagnostic=read(RUN/'ff_diagnostic.json'),
    hardware_commands_sent=0,operator_ready=False,
    source_hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (
        RUN/'saved_1ms.csv',RUN/'saved_1ms.settings.json',RUN/'tracking_1ms_review_v3/verification.json',RUN/'ff_diagnostic.json')},
    limitations=['閉ループ単発かつ各相観測不足。FFの同定完了・追従改善確認ではない。',
        '新規配置・両方ON・手離し確認後に一回だけ比較。現在の停止位置から元225/45mmへ戻す。',
        '幾何・角度補正は追従比較後に動画から再評価。BのEKFは未同定。']))
print(OUT)
