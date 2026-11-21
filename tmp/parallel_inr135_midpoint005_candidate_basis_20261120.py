"""中点残差と未同定の角度感度を分け、同時補正の保守的候補を数値比較する。"""
from pathlib import Path
import hashlib
import json
import re
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_inr135_20261120_v1'
OUT=BASE/'parallel_analysis/A_INR135_1800_baseline_20261120_005/midpoint_raw_wheel_clock_v1'
read=lambda path:json.loads(path.read_text(encoding='utf-8-sig'))
review=read(OUT/'midpoint_review.json')
settings=read(BASE/'baseline01/candidate.settings.json')
text=(ROOT/'Params/turn_table.h').read_text(encoding='utf-8')
table=np.fromstring(re.search(r'accel_table\[1001\]\s*=\s*\{([^}]+)\}',text)[1],sep=',')
integral=float(re.search(r'accel_Integral\s*=\s*([.\d]+)',text)[1])
assert len(table)==1001
p=settings['parameters']


def nominal_lend(correction):
    phi=np.deg2rad(abs(p['degree']+correction))
    peak=p['velo']*1000/abs(p['r_min'])
    duration=phi/(peak*integral)
    t=np.linspace(0,duration,1001)
    theta=np.r_[0,np.cumsum((table[:-1]+table[1:])*peak/2*np.diff(t))]
    middle=(theta[:-1]+theta[1:])/2
    curve=np.array([np.sum(p['velo']*1000*np.sin(middle)*np.diff(t)),np.sum(p['velo']*1000*np.cos(middle)*np.diff(t))])
    return np.array([0.,90+p['lstart']])+curve+p['lend']*np.array([np.sin(theta[-1]),np.cos(theta[-1])])


baseline=nominal_lend(0.)
changed=nominal_lend(3.5)
delta=changed-baseline
measured=np.array(review['midpoint_Lend_right_forward_mm'])
target=np.array(review['target_midpoint_Lend_right_forward_mm'])
predicted_angle=review['exit_path_direction_change_deg']-3.5
direction=np.array([np.sin(np.deg2rad(predicted_angle)),np.cos(np.deg2rad(predicted_angle))])
angle_only=measured+delta
delta_le=(target[0]-angle_only[0])/direction[0]
delta_ls=target[1]-angle_only[1]-delta_le*direction[1]
scenarios=[]
for ls,le,label in ((22.85,34.81,'angle_only'),(7.85,34.81,'conservative_Lstart_minus15'),
                   (5.85,34.81,'Lstart_minus17'),(p['lstart']+delta_ls,p['lend']+delta_le,'full_unvalidated_nominal_response_solution')):
    predicted=angle_only+np.array([0.,ls-p['lstart']])+(le-p['lend'])*direction
    scenarios.append(dict(label=label,requested_parameters=dict(correction=3.5,r_min=-40.,lstart=float(ls),lend=float(le)),
        predicted_midpoint_Lend_right_forward_mm=predicted.tolist(),predicted_residual_mm=(predicted-target).tolist(),
        prediction_is_measured_causal_response=False))
result=dict(mouse='A',motion='in_r135',source_review=str(OUT/'midpoint_review.json'),
    original_requested_parameters=p,observed_midpoint_Lend_mm=measured.tolist(),residual_mm=(measured-target).tolist(),
    requested_angle_correction_deg=3.5,predicted_path_angle_deg=predicted_angle,
    ideal_constant_speed_table_endpoint_before_mm=baseline.tolist(),ideal_constant_speed_table_endpoint_after_mm=changed.tolist(),
    ideal_angle_only_endpoint_delta_mm=delta.tolist(),angle_delta_model='Firmware angular table, ideal constant speed1.8m/s, radius40mm. No measured slip/control response fit.',
    angle_changed_length_only_delta=dict(delta_Lstart_mm=float(delta_ls),delta_Lend_mm=float(delta_le)),
    candidate_scenarios=scenarios,
    recommendation='往復回数を減らすなら補正+3.5とLstartを15mm短縮する要求Ls7.85/Le34.81/r-40の同時1回候補を推す。現角度の全量−20.2mmや未同定角度モデルの全量解は採用せず、約75%の入口長さ補正に留める。',
    rationale=['中点入口前進残差+15.815mmと出口線ΔLs−19.687mmは時計依存の小誤差より大きい。',
        '理想角度感度では+3.5度による終端変位は数mmで、角度だけの1本では入口長さ不足が残る見込み。',
        'Leは現要求を維持し、同時に変更する幾何項目は補正とLstartだけに絞る。',
        '同時候補は未実行の局所試験。角度感度と滑りの実応答は未同定なので、rms/観測/前走時計/接触なしと中点Lendを再評価する。'],
    new_stage_created=False,hardware_operations=0,parameter_adoption_allowed=False,
    old_angle_only_stages_unchanged=True,
    source_sha256={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in
        (OUT/'midpoint_review.json',BASE/'baseline01/candidate.settings.json',ROOT/'Params/turn_table.h',Path(__file__))})
with (OUT/'simultaneous_length_candidate_basis.json').open('x',encoding='utf-8') as file:
    json.dump(result,file,ensure_ascii=False,indent=2,allow_nan=False);file.write('\n')
print(json.dumps(result,ensure_ascii=False,indent=2),flush=True)
