"""IN135の元要求値を保ち、correctionだけ+3.5にしたBASE直下3stageを準備する。"""
from pathlib import Path
import copy
import hashlib
import json
import sys

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_inr135_20261120_v1'
RUN=BASE/'baseline01/local_execution/run_A_INR135_1800_baseline_20261120_005'
OLD=BASE/'parallel_analysis/inr135_c3p5_angle_candidate_v1'
OUT=BASE/'parallel_analysis/inr135_c3p5_angle_candidate_v2'
REQUESTED=BASE/'baseline01/candidate.settings.json'
read=lambda path:json.loads(path.read_text(encoding='utf-8-sig'))
OUT.mkdir(exist_ok=False)
requested=read(REQUESTED);actual=read(RUN/'saved_1ms.settings.json')
assert requested['parameters']['lstart']==22.85 and requested['parameters']['lend']==34.81
basis=copy.deepcopy(read(OLD/'basis.json'))
basis.update(source_requested_settings=str(REQUESTED),source_saved_readback=str(RUN/'saved_1ms.settings.json'),
    candidate_request_policy='元要求値を再送しcorrectionだけ変更。同一要求→同一切捨てreadbackを各captureで確認する。',
    changed_parameters=dict(correction=dict(before_deg=0.,candidate_deg=3.5)),
    unchanged_geometry={key:requested['parameters'][key] for key in ('r_min','lstart','lend','degree','velo','pre_accel')},
    requested_parameters_before=requested['parameters'],saved_actual_parameters_before=actual['parameters'],
    do_not_resend_quantized_readback=True,
    supersedes=str(OLD/'basis.json'),source_sha256={str(path):hashlib.sha256(path.read_bytes()).hexdigest()
        for path in (REQUESTED,RUN/'saved_1ms.settings.json',OLD/'basis.json',Path(__file__))})
with (OUT/'basis.json').open('x',encoding='utf-8') as file:
    json.dump(basis,file,ensure_ascii=False,indent=2,allow_nan=False);file.write('\n')
with (OLD/'SUPERSEDED_NOT_FOR_EXECUTION.json').open('x',encoding='utf-8') as file:
    json.dump(dict(state='SUPERSEDED_NOT_FOR_EXECUTION',reason='元要求値を維持するBASE直下stageへ置換。旧saved量子化値の再送を避ける。',
        replacement_basis=str(OUT/'basis.json'),replacement_stages=[str(BASE/f'angle_c3p5_{i:02d}') for i in (1,2,3)],
        old_stages_never_executed=True),file,ensure_ascii=False,indent=2);file.write('\n')
sys.path.insert(0,str(ROOT/'tmp'))
import mouse_tuning_prepare_candidate_20261120 as prepare
prepare.COPY_FILES=tuple(name for name in prepare.COPY_FILES if (BASE/'baseline01'/name).is_file())
sys.argv=[str(ROOT/'tmp/mouse_tuning_prepare_candidate_20261120.py'),'--base',str(BASE/'baseline01'),
    '--source-run',str(RUN),'--settings',str(REQUESTED),'--name','angle_c3p5_',
    '--id-prefix','A_INR135_1800_c3p5_req','--output-base',str(BASE),
    '--correction','3.5','--count','3','--date','20261120','--basis',str(OUT/'basis.json')]
prepare.main()
for number in (1,2,3):
    stage=BASE/f'angle_c3p5_{number:02d}'
    candidate=read(stage/'candidate.settings.json')
    assert candidate['parameters']==dict(requested['parameters'],correction=3.5)
    assert all(candidate[key]==requested[key] for key in ('pid','feedforward','suction'))
    plan=read(stage/'placement_plan.json')
    plan['static_apparent_reference_heading_deg']=90.
    plan['placement']['heading_deg']=90.
    plan['static_reference_basis']='Nominal planned board heading90deg. Earlier empirical89.2604deg is not used for acceptance.'
    plan['heading_acceptance_version']=3;plan['heading_target_basis']='nominal_planned_board_heading'
    plan['source_requested_settings']=str(REQUESTED)
    plan['readback_policy']='元要求値と切捨て表示値を区別する。各captureで同一要求→同一readbackを確認。'
    with (stage/'placement_plan.json').open('w',encoding='utf-8') as file:
        json.dump(plan,file,ensure_ascii=False,indent=2,allow_nan=False);file.write('\n')
print(json.dumps(dict(basis=str(OUT/'basis.json'),new_stages=[str(BASE/f'angle_c3p5_{i:02d}') for i in (1,2,3)],
    request_parameters=dict(requested['parameters'],correction=3.5),physical_execution=False),ensure_ascii=False),flush=True)
