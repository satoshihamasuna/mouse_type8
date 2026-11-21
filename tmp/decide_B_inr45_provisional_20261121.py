"""実走10回の時点で実測済み候補を暫定固定し、追加探索の予定を終了する。"""
from pathlib import Path
from datetime import datetime,timezone
import copy
import hashlib
import json
import numpy as np
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr45_20261120_v1'
OUT=BASE/'provisional_decision_20261121_v1';OUT.mkdir(exist_ok=False)
read=lambda p:json.loads(Path(p).read_text(encoding='utf-8-sig'))
def save(p,v):
 with Path(p).open('x',encoding='utf-8') as f:json.dump(v,f,ensure_ascii=False,indent=2,allow_nan=False)
executions=[]
for path in sorted(BASE.glob('*/local_execution/run_*/trial.json')):
 events=path.parent/'events.jsonl'
 count=sum(json.loads(line).get('event')=='execution_write_attempt' for line in events.read_text(encoding='utf-8').splitlines()) if events.exists() else 0
 if count:executions.append(dict(trial_json=str(path),execution_commands=count,camera_state=read(path)['camera_state']))
assert sum(row['execution_commands'] for row in executions)==10 and all(row['execution_commands']==1 for row in executions)
report_path=BASE/'red_s30_validation_v1/review.json';report=read(report_path);condition=report['current_condition']
assert condition['runs']==2 and condition['run_ids']==['011','012']
candidate=copy.deepcopy(read(BASE/'geometry05_repeat01_correction_m1p2/candidate.settings.json'))
assert candidate['parameters']==condition['parameters']
candidate.update(source='2026-11-21ユーザーの決定時期の指摘を受け、実測済み2本の右in45候補を暫定固定。残差と未確認事項はdecision.json。Params/ファーム未反映。',
 adoption_state='PROVISIONAL_FIXED_TESTED_CANDIDATE',evidence_report=str(report_path))
save(OUT/'selected.settings.json',candidate)
point=np.array(condition['mean_Lend_right_forward_mm']);residual=point-[45.,180.]
decision=dict(decided_at_utc=datetime.now(timezone.utc).isoformat(),mouse='B',motion='in_r45',preset_mm_s=1600,
 status='PROVISIONAL_PARAMETERS_FIXED_ADDITIONAL_EXPLORATION_ENDED',execution_count=10,execution_records=executions,
 selected_settings=str(OUT/'selected.settings.json'),same_setting_measured_runs=['011','012'],
 selection_reason='角度補正の応答が局所的に一定でないため、未実走値の内挿を最終値にせず、同設定2本を得た実測済み−1.2度候補を暫定固定する。',
 evidence=condition,mean_exit_error_from45_deg=condition['mean_exit_deg']-45.,mean_Lend_residual_mm=residual.tolist(),
 mean_Lend_residual_norm_mm=float(np.linalg.norm(residual)),
 remaining_limitations=['平均出口46.010度で目標45度に対し約+1.01度の残差。精密な最終合格の宣言ではない。',
  '012は充電未完了。充電後の再現性は未確認。追加確認を行う場合は同設定一回を上限とする方針で、実行は未依頼/未送信。',
  '右in45単体の入口相対マーカー中点。左・連続接続・絶対車軸/body yaw・BのEKFは未確認。',
  '元S40の012不成立と008全体FAILED、標準summary/auto_stop判定を保持。S30別解析の実検出による評価。',
  'PID/FFは既存の改善候補を保持し、角速度ピーク・後揺れは残っている。'],
 further_parameter_exploration_planned=False,additional_run_authorized=False,firmware_written=False,source_params_changed=False,
 physical_power_state='BOTH_OFF_USER_CONFIRMED',
 input_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (report_path,Path(__file__))})
save(OUT/'decision.json',decision)
text='# 機体B turn1600 右in45：暫定採用値の決定\n\n'
text+='実走10回。静止撮影・未実行の候補は含めない。ユーザーの決定時期の指摘を受け、追加の条件探索を終了し、同設定2本を測った実測済み候補を固定する。\n\n'
text+='速度1.6m/s、半径−55mm、Lstart7.8mm、Lend34.5mm、角度補正−1.2度、前走加速度設定6.5m/s²。角速度PID0.2/0.005/0、角加減速FF0.000750/0.000750、吸引650。全設定はselected.settings.json。\n\n'
text+=f"同設定011/012の出口は46.354/45.666度、平均{condition['mean_exit_deg']:.3f}度。Lend平均は右{point[0]:.3f}/前{point[1]:.3f}mm、目標45/180との差は{residual[0]:+.3f}/{residual[1]:+.3f}mm（距離{np.linalg.norm(residual):.3f}mm）。Lendの成分範囲は0.149/0.051mm。\n\n"
text+='これは右単体の暫定固定値。平均角度約+1度と位置約2mmの残差、充電後の再現性・左・連続接続は残る。検証済みS30解析プロファイルを使用。元の取得/不成立判定は保持し、Params/ファームへは反映していない。\n\n'
text+='準備済み013は未送信の任意確認として保管し、必要な追加走行として扱わない。充電完了の回答だけで走行を再開しない。現在は両方OFF。\n'
(OUT/'README.md').write_text(text,encoding='utf-8')
state_path=BASE/'resume_state.json';state=read(state_path);save(OUT/'resume_state.before_decision.json',state)
unused=dict(stage=state['next_stage'],request_id=state['next_request_id'],request_sent=False,
 status='OPTIONAL_UNREQUESTED_CONFIRMATION_NOT_REQUIRED_FOR_PROVISIONAL_DECISION')
state.update(state='PROVISIONAL_PARAMETERS_FIXED_NO_FURTHER_RUN_PLANNED',parameter_decision=str(OUT/'decision.json'),
 selected_settings=str(OUT/'selected.settings.json'),unused_prepared_validation=unused,next_stage=None,next_request_id=None,next_settings=None,
 next_request_sent=False,next_operator_ready=False,operator_ready=False,execution_pending=False,
 remaining_current_correction_repeats=0,further_parameter_exploration_planned=False,
 next_action='右in45の実測済み暫定値を固定済み。追加実走予定なし。反映/最終確認は新たな依頼に応じて行う。充電完了の返答だけでは走行を再開しない。',
 readiness_question_pending=None,physical_power_state='BOTH_OFF_USER_CONFIRMED')
state_path.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
with (BASE/'README.md').open('a',encoding='utf-8') as f:f.write('\nユーザーの決定時期の指摘を受け、実走10回の時点で実測済み−55/7.8/34.5/−1.2度を暫定固定。追加探索を終了。provisional_decision_20261121_v1/README.md。013は任意未依頼の確認候補として保管し、自動再開しない。Params/ファーム未反映。\n')
print(json.dumps(dict(execution_count=10,selected_parameters=candidate['parameters'],mean_exit_deg=condition['mean_exit_deg'],mean_position_error_mm=float(np.linalg.norm(residual)),additional_run_planned=False),ensure_ascii=False))
