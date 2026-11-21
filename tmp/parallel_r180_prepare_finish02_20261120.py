"""機体A右180度の仕上げ候補を新規フォルダに準備する。実機接続なし。"""
from pathlib import Path
import hashlib
import json
import shutil

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tools/logs/pixel_camera/turn1800_A_r180_20261120_v1'
SOURCE = BASE / 'corr3p5_r49_Le34p5_02'
OUT = BASE / 'finish02_01'
REVIEW = BASE / 'parallel_finish/candidate_two_optical_clock_v1/report.json'
REQUEST = 'A_R180_1800_finish02_c3p5_r48_Le31p5_20261120_001'
OUT.mkdir(exist_ok=False)


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def save(name, value):
    with (OUT / name).open('x', encoding='utf-8', newline='\n') as file:
        json.dump(value, file, ensure_ascii=False, indent=2, allow_nan=False)
        file.write('\n')


measurements = read(REVIEW)['results']
assert len(measurements) == 2
assert all(item['independent_optical_geometry_eligible'] for item in measurements)
assert all(not item['original_encoder_path_review_eligible'] for item in measurements)
candidate = read(SOURCE / 'candidate.settings.json')
candidate['source'] = ('未実行の機体A turn1800右180度仕上げ候補。独立した入口生エンコーダ時計と光学直線を使用した'
    '候補1/2は角180.5059/180.5878deg、帰り幅93.0822/95.0938mm、Lend forward87.0342/88.6630mm。'
    '過大な幅に対しr_minを-49から-48mmへ、forward不足に対しLendを34.5から31.5mmへ変更。'
    '補正+3.5deg、Lstart7.52mm、PID/FF/吸引650は維持。相対マーカー中点と実測入口基準のみ。'
    '因果応答モデルや採用値ではなく次の単体試験候補。')
candidate['parameters']['r_min'] = -48.0
candidate['parameters']['lend'] = 31.5
save('candidate.settings.json', candidate)
save('config.json', read(SOURCE / 'config.json'))

plan = read(SOURCE / 'placement_plan.json')
for name in ('predicted_angle_deg', 'predicted_return_spacing_mm', 'baseline_apex_mm', 'candidate_apex_prediction_mm'):
    plan.pop(name, None)
plan.update(candidate=str(OUT / 'candidate.settings.json'), config=str(OUT / 'config.json'),
    next_request_id=REQUEST, power_state='Both_OFF_confirmed_by_parent_before_preparation',
    current_placement_confirmed=False, current_clearance_confirmed=False, operator_ready=False,
    next_candidate_executed=False, execution_pending=False,
    workflow='他ターンと交互に取得。今回の配置/ON/手離し確認、新規静止照合と実機設定読返しを経て1回のみ。全フレーム解析待ちは両方OFF。',
    placement_basis='使用済みのA右180度(315,135)mm・赤前下を再使用する提案。今回の穴/全接地点/車幅/旋回/停止余裕は再確認。',
    source_baseline_review=str(BASE / 'parallel_finish/candidate_two_relative_v1/video_comparison.json'),
    independent_clock_review=str(REVIEW), encoder_path_model_discrepancy_unresolved=True,
    evaluation_scope='相対マーカー中点と実測入口方向。絶対機軸yaw・車軸位置・盤面固定の合格は未確認。',
    identifiability='平行入出力では終点からLstart/Lendを独立同定できない。今回はLstart固定、頂点と位相対応を併記。',
    prediction_scope='半径1mm縮小・Lend3mm短縮の局所試験。実測された偏微分ではなく、形状/実機クリアランスを保証しない。')
save('placement_plan.json', plan)

wrapper = (SOURCE / 'run_recorded_once.py').read_text(encoding='utf-8')
old_request = 'A_R180_1800_c3p5_r49_Le34p5_repeat02_20261120_001'
assert wrapper.count(old_request) == 1
with (OUT / 'run_recorded_once.py').open('x', encoding='utf-8', newline='\n') as file:
    file.write(wrapper.replace(old_request, REQUEST))
for name in ('static_pose_after_settle.py', 'static_settle_policy.json'):
    shutil.copyfile(SOURCE / name, OUT / name)

save('preparation_evidence.json', dict(mouse='A', motion='long_r180', candidate_trial_id=REQUEST,
    source_stage=str(SOURCE), review=str(REVIEW),
    review_sha256=hashlib.sha256(REVIEW.read_bytes()).hexdigest(), input_runs=measurements,
    changed_parameters=dict(r_min=dict(before_mm=-49.0, after_mm=-48.0), lend=dict(before_mm=34.5, after_mm=31.5)),
    unchanged='correction+3.5deg,Lstart7.52mm,velo1.8m/s,PID,FF,suction650,camera1500mm/EV-1.5/zoom1.8/torchfalse/focuslock',
    excluded_trial='corr3p5_r49_Le34p5_03:full/summary.json欠落・required_motion_window不合格。品質95%は緩和しない。',
    parameter_adoption_allowed=False, source_params_changed=False, robot_commands_sent=0,
    camera_recording_started=False, operator_ready=False))
print(json.dumps(dict(stage=str(OUT), request_id=REQUEST, parameters=candidate['parameters']), ensure_ascii=False))
