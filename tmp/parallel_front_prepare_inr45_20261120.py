"""IN45赤前点r25参照による長さだけの次候補と独立3stageを準備。実機操作なし。"""
from pathlib import Path
import hashlib
import json
import subprocess
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_inr45_20261120_v1'
RUN=BASE/'baseline01/local_execution/run_A_INR45_1800_baseline_20261120_002'
OUT=BASE/'parallel_front_candidate_Ls2p0_Le40'
FRONT=BASE/'parallel_front_reference/front_reference.json'
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
settings,front=read(RUN/'saved_1ms.settings.json'),read(FRONT)
prefix='front_Ls2p0_Le40_'
paths=[BASE/f'{prefix}{i:02d}' for i in (1,2,3)]
if OUT.exists() or any(p.exists() for p in paths): raise FileExistsError('既存の候補/根拠は上書きしません')
phi=np.radians(front['path_direction_change_deg'])
direction=np.array([np.sin(phi),np.cos(phi)])
normal=np.array([-direction[1],direction[0]])
before=np.array(front['measured_front_Lend_right_forward_mm'])
delta_ls=2.-settings['parameters']['lstart']
delta_le=40.-settings['parameters']['lend']
after=before+np.array([0.,delta_ls])+delta_le*direction
scenarios=[]
for radius in (20.,25.,30.):
    target=np.array([45.,180.])+radius*np.array([np.sin(np.pi/4),np.cos(np.pi/4)-1])
    error=after-target
    scenarios.append(dict(assumed_half_span_mm=radius,front_target_right_forward_mm=target.tolist(),
        predicted_candidate_front_Lend_right_forward_mm=after.tolist(),predicted_error_right_forward_mm=error.tolist(),
        predicted_error_norm_mm=float(np.linalg.norm(error)),predicted_along_exit_error_mm=float(error@direction),
        predicted_cross_exit_error_mm=float(error@normal),radius_is_measured=False))
OUT.mkdir()
basis=dict(mouse='A',motion='in_r45',preset_speed_mm_s=1800,state='UNEXECUTED_FRONT_REFERENCE_LENGTH_CANDIDATE',
    source_run=str(RUN),source_settings=str(RUN/'saved_1ms.settings.json'),source_front_review=str(FRONT),
    candidate_parameters=dict(correction=0.,r_min=-53.,lstart=2.,lend=40.),
    unchanged_settings=['velo','degree','pre_accel','pid','feedforward','suction','camera','start placement','post diagonal acceleration'],
    source_measured_front_Lend_right_forward_mm=before.tolist(),delta_lstart_mm=float(delta_ls),delta_lend_mm=float(delta_le),
    prediction_formula='candidate_F_Lend = measured_F_Lend + delta_Lstart*(0,1) + delta_Lend*(sin(measured_exit_path_angle),cos(measured_exit_path_angle))',
    measured_exit_path_angle_deg=front['path_direction_change_deg'],angle_error_from45_deg=front['path_direction_change_deg']-45,
    correction_unchanged_reason='Measured path44.649deg is within0.5deg of45. Keep correction0/radius-53 fixed to isolate length-only trial; this is path direction, not calibrated body yaw.',
    half_span_scenarios=scenarios,nominal_half_span_mm=25.,nominal_half_span_is_approximate=True,
    nominal_case=scenarios[1],observed_point='front-red independent relative displacement, not midpoint/axle',
    observation_rate=front['observation_rate'],clock=front['clock'],
    causal_model_validated=False,parameter_adoption_allowed=False,operator_ready=False,physical_execution=False,
    future_validation='One independent single-turn per stage; compare actually registered front-red >=95%, line quality and pre-wheel-only clock; review all three repeats and contact/stop reports.',
    limitations=['Prediction only translates the observed profile along entry/exit; dynamics, slip and saturation changes are not fitted.',
        'r20/25/30 are sensitivity scenarios, not confidence limits or measured marker radii.',
        'Body/axle yaw, camera foot, metric scale and fixed-board placement remain unverified.',
        'Nominal25 prediction does not guarantee physical clearance or repeatability.'],
    source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (RUN/'saved_1ms.settings.json',FRONT,Path(__file__))})
with (OUT/'basis.json').open('x',encoding='utf-8') as stream:json.dump(basis,stream,ensure_ascii=False,indent=2)
command=[str(Path('C:/Users/sato1/Documents/git/Debug_camera/.venv/Scripts/python.exe')),
    str(ROOT/'tmp/mouse_tuning_prepare_candidate_20261120.py'),'--base',str(BASE/'baseline01'),
    '--source-run',str(RUN),'--name',prefix,'--id-prefix','A_INR45_1800_front_Ls2p0_Le40',
    '--correction','0','--r-min','-53','--lstart','2.0','--lend','40.0','--count','3','--date','20261120','--basis',str(OUT/'basis.json')]
completed=subprocess.run(command,stdout=subprocess.PIPE,stderr=subprocess.PIPE,check=True)
with (OUT/'stage_preparation.json').open('xb') as stream:stream.write(completed.stdout)
print(json.dumps(dict(basis=str(OUT/'basis.json'),stages=[str(p) for p in paths],nominal_prediction=scenarios[1],physical_execution=False),ensure_ascii=False,indent=2))
