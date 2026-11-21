"""照合済み3本からLendだけを小さくする比較候補を準備。実機操作なし。"""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json
import math
import statistics
import subprocess
import sys

root=Path(__file__).resolve().parents[1]
base=root/'tools/logs/pixel_camera/turn1600_B_outr135_20261121_resume_v1'
out=base/'refinement_Lend29p2_plan_v1'
assert not out.exists()
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
comparison_path=base/'geometry_same_candidate_three_review_v2_manual_pairs/review.json'
comparison=read(comparison_path)
assert comparison['successful_same_candidate_runs']==3
assert comparison['settings']['parameters']['lstart']==5.3
assert comparison['settings']['parameters']['lend']==31.0
assert comparison['settings']['parameters']['correction']==1.0
rows=[]
for run in comparison['runs']:
    review_path=Path(run['geometry_review'])
    assert sha(review_path)==run['geometry_review_sha256']
    geometry=read(review_path)
    assert geometry['independent_optical_geometry_eligible']
    target=geometry['target_midpoint_Lend_right_forward_mm']
    point=geometry['midpoint_Lend_right_forward_mm']
    theta=math.radians(geometry['exit_path_direction_change_deg'])
    direction=[math.sin(theta),math.cos(theta)]
    residual=[point[i]-target[i] for i in (0,1)]
    projection=sum(residual[i]*direction[i] for i in (0,1))
    rows.append(dict(request_id=run['request_id'],review=str(review_path),review_sha256=sha(review_path),
        residual_right_forward_mm=residual,exit_unit_right_forward=direction,
        excess_along_exit_mm=projection,observed_Lend_error_mm=math.hypot(*residual)))
mean_excess=statistics.mean(row['excess_along_exit_mm'] for row in rows)
new_lend=round(31.0-mean_excess,1)
assert new_lend==29.2
for row in rows:
    predicted=[row['residual_right_forward_mm'][i]+(new_lend-31.0)*row['exit_unit_right_forward'][i] for i in (0,1)]
    row.update(predicted_residual_right_forward_mm=predicted,
        predicted_Lend_error_mm=math.hypot(*predicted))
    assert row['predicted_Lend_error_mm']<row['observed_Lend_error_mm']
out.mkdir()
plan=dict(recorded_at=datetime.now(timezone.utc).isoformat(),mouse='B',motion='out_r135',preset_mm_s=1600,
    state='OFFLINE_PREPARATION',user_request='out135調整したい',
    source_comparison=str(comparison_path),source_comparison_sha256=sha(comparison_path),
    model='同じ出口方向・速度を保ったまま、Lendの変化量だけ出口単位方向へ平行移動すると仮定。実測入口相対のマーカー中点。',
    changed_parameter='parameters.lend',old_Lend_mm=31.0,new_Lend_mm=new_lend,
    unrounded_Lend_mm=31.0-mean_excess,mean_excess_along_exit_mm=mean_excess,
    projection_population_sd_mm=statistics.pstdev(r['excess_along_exit_mm'] for r in rows),
    per_run=rows,held_parameters=dict(lstart_mm=5.3,correction_deg=1.0,r_min_mm=-41.0,velo_m_s=1.6),
    pid_ff_suction_modes_and_camera_held=True,
    rationale='3本すべてで出口方向への残差が正。角度のばらつきに対し微小な補正角変更はせず、まずLendだけを変更する。',
    limitations=['予測値は新候補の実測ではない。1回の比較後、同じ品質条件で評価する。',
        '停止点の約8mm差は後走の別課題。停止点を目標にLendを選んでいない。',
        'BのEKF係数・車軸/機軸の絶対校正・左出口は未確認。'],
    next_stage=str(base/'refine_c1p0_Ls5p3_Le29p2_01'),
    next_request_id='B_OUTR135_1600_c1p0_Ls5p3_Le29p2_refine_20261121_001',
    current_operator_ready=False,hardware_operations=0,firmware_source_changed=False)
plan_path=out/'plan.json'
with plan_path.open('x',encoding='utf-8') as stream:json.dump(plan,stream,ensure_ascii=False,indent=2)
source_stage=base/'standard_flow_c1p0_Ls5p3_Le31_01'
source_run=Path(comparison['runs'][-1]['actual_trial']).parent
command=[str(Path('C:/Users/sato1/Documents/git/Debug_camera/.venv/Scripts/python.exe')),
    str(root/'tmp/mouse_tuning_prepare_candidate_20261120.py'),
    '--base',str(source_stage),'--source-run',str(source_run),
    '--name','refine_c1p0_Ls5p3_Le29p2_','--id-prefix','B_OUTR135_1600_c1p0_Ls5p3_Le29p2_refine',
    '--count','1','--date','20261121','--lend',str(new_lend),'--basis',str(plan_path)]
with (out/'prepare.stdout.txt').open('x',encoding='utf-8') as stream:
    result=subprocess.run(command,stdout=stream,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
assert result.returncode==0,result.returncode
stage=Path(plan['next_stage'])
settings=read(stage/'candidate.settings.json')
previous=read(source_run/'saved_1ms.settings.json')
for key in ('pid','feedforward','suction','mouse','motion_name','preset_speed_mm_s'):
    assert settings[key]==previous[key],key
assert {k:v for k,v in settings['parameters'].items() if k!='lend'}=={k:v for k,v in previous['parameters'].items() if k!='lend'}
assert settings['parameters']['lend']==29.2
assert read(stage/'config.json')['address']=='E1:7F:86:21:84:73'
wrapper=(stage/'run_recorded_once.py').read_text(encoding='utf-8')
assert '    def perform(' not in wrapper
assert "placement = dict(x_mm=180., y_mm=45., heading_deg=45." in wrapper
validation=read(stage/'offline_validation.json')
assert validation['no_execution_job_or_transport'] and validation['source_unchanged']
state_path=base/'resume_state.json';state=read(state_path)
state.update(state='LEND29P2_REFINEMENT_PREPARED_AWAITING_CURRENT_READINESS',
    next_stage=str(stage),next_request_id=plan['next_request_id'],next_request_sent=False,
    candidate=str(stage/'candidate.settings.json'),config=str(stage/'config.json'),
    operator_ready=False,current_placement_confirmed=False,current_hands_clear_confirmed=False,
    execution_pending=False,next_action_kind='single_run_comparison_after_current_readiness',
    refinement_plan=str(plan_path),
    next_action='同じ180/45mm・前緑右下45°へ配置。新しい干渉なし・両方ON・手離し確認後に1回だけ比較する。')
state_path.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
(out/'README.md').write_text('# 機体B1600 右out135・Lendだけを29.2 mmへ変更する比較\n\n'
    '前回3本はLstart5.3/Lend31.0 mm、補正+1.0°。実マーカー照合後の各出口残差を各出口進行方向へ射影し、平均の進み過ぎだけをLendから引いた。Lstart・半径・角度補正・PID/FF・吸引・前後走条件・撮影条件は維持する。\n\n'
    f'計算上のLendは{31.0-mean_excess:.4f} mm、比較用に29.2 mmへ丸めた。新候補は未走行であり、予測を改善実績としない。停止点差は別に記録する。根拠と各試行の予測は[plan.json](plan.json)。\n\n'
    '標準MCPの静止撮影→BLE接続→設定照合→単体1回→切断を継承する。BLEはB用E1:7F:86:21:84:73。現時点で実機操作・ソース反映・書込みは0件。\n',encoding='utf-8')
print(json.dumps(dict(stage=str(stage),request_id=plan['next_request_id'],new_Lend_mm=new_lend,
    unrounded_Lend_mm=31.0-mean_excess,per_run=rows,physical_execution=False),ensure_ascii=False),flush=True)
