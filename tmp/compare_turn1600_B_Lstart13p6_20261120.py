"""B1600右90°のLstartだけを変更した2試行を、保存資料で比較する。"""
from pathlib import Path
import copy
import hashlib
import json
import re

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_r90_20261120_v1'
OUT=BASE/'compare_Lstart12_to13p6_v1'


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


before_folder=BASE/'baseline001_video_review_v1'
after_folder=BASE/'Lstart13p6_001_video_review_v1'
before=read(before_folder/'video_comparison.json')['comparisons'][0]
after=read(after_folder/'video_comparison.json')['comparisons'][0]
before_run=Path(before['source_trial']).parent
after_run=Path(after['source_trial']).parent
before_settings=read(before_run/'saved_1ms.settings.json')
after_settings=read(after_run/'saved_1ms.settings.json')
assert before_settings['mouse']==after_settings['mouse']=='B'
assert before_settings['preset_speed_mm_s']==after_settings['preset_speed_mm_s']==1600
differences=[]
for group in ('parameters','pid','feedforward','suction'):
    for key,value in before_settings[group].items():
        if value!=after_settings[group][key]:
            differences.append(group+'.'+key)
assert differences==['parameters.lstart']
assert before_settings['parameters']['lstart']==12. and after_settings['parameters']['lstart']==13.6
before_prepared=read(BASE/'device_prepare03/result.json')
after_prepared=read(BASE/'device_prepare_Lstart13p6_01/result.json')
assert before_prepared['straight_after']==after_prepared['straight_after']
for key in ('wall_ekf','wall_edge','turn_ekf'):
    assert re.search(r'mode:(\w+)',before_prepared[key]).group(1)==re.search(r'mode:(\w+)',after_prepared[key]).group(1)
assert digest(before_run/'marker_reference_used.json')==digest(after_run/'marker_reference_used.json')
profiles=[]
for run in [before_run,after_run]:
    profile=read(run/'full/body_geometry_used.json')
    profile.pop('measurement_video_file',None)
    profiles.append(profile)
assert profiles[0]==profiles[1]
OUT.mkdir(exist_ok=False)
fig,axes=plt.subplots(1,3,figsize=(14,4.5),layout='constrained')
for label,folder,case,colour in [('Lstart12.0',before_folder,before,'tab:blue'),
                                ('Lstart13.6',after_folder,after,'tab:orange')]:
    path=pd.read_csv(folder/(case['label']+'_relative_trajectory.csv'))
    good=path.detected.to_numpy(bool)
    axes[0].plot(path.right_mm[good],path.forward_mm[good],label=label,color=colour)
    axes[1].bar(label,case['exit_line_offset_from270_mm'],color=colour)
    axes[2].bar(label,case['path_rotation_error_from90_deg'],color=colour)
axes[0].axhline(270,color='black',linestyle='--',linewidth=.8)
axes[0].scatter(90,270,color='black',marker='x',label='Turn-end reference')
axes[0].set(xlabel='Right displacement [mm]',ylabel='Forward displacement [mm]')
axes[0].axis('equal');axes[0].legend(fontsize=8)
axes[1].set(ylabel='Exit-line forward offset [mm]')
axes[2].set(ylabel='Entry-to-exit rotation minus90 [deg]')
for axis in axes:
    axis.grid(alpha=.25)
for axis in axes[1:]:
    axis.axhline(0,color='black',linewidth=.8)
fig.savefig(OUT/'comparison.png',dpi=160)
plt.close(fig)
result=dict(mouse='B',motion='long_r90',preset_mm_s=1600,suction=after_settings['suction'],
    only_changed_turn_field=differences[0],before=before,after=after,
    lstart_change_mm=1.6,
    observed_exit_line_shift_mm=after['exit_line_offset_from270_mm']-before['exit_line_offset_from270_mm'],
    absolute_exit_line_error_reduced=abs(after['exit_line_offset_from270_mm'])<abs(before['exit_line_offset_from270_mm']),
    same_pid_ff_suction_straight_settings_and_modes=True,same_camera_reference_and_geometry=True,
    all_relative_reviews_eligible=before['relative_review_eligible'] and after['relative_review_eligible'],
    power_state='Both OFF confirmed after each trial',ble_disconnected=True,
    source_params_changed=False,parameter_adoption_allowed=False,
    repeat_count_per_setting=1,repeatability_established=False,
    limitations=['入口方向に合わせた相対マーカー中点の比較。盤面固定・絶対車軸精度は未確認。',
        '各設定1本ずつで、Lstart変更の効果と再現性を分離して確定したものではない。',
        'Lendを停止終点だけから決めない。同期の±5ms感度と出口直線を併記。',
        'BのEKF係数は同定していない。AのK・雑音を流用しない。'],
    input_sha256={str(p.relative_to(ROOT)):digest(p) for p in
        [before_folder/'video_comparison.json',after_folder/'video_comparison.json',
         before_run/'saved_1ms.settings.json',after_run/'saved_1ms.settings.json']})
with (OUT/'comparison.json').open('x',encoding='utf-8') as output:
    json.dump(result,output,ensure_ascii=False,indent=2,allow_nan=False)
    output.write('\n')
print(json.dumps({key:result[key] for key in ['only_changed_turn_field','observed_exit_line_shift_mm',
    'absolute_exit_line_error_reduced','all_relative_reviews_eligible']},ensure_ascii=False),flush=True)
print(json.dumps({key:after[key] for key in ['entry_to_exit_path_rotation_deg','exit_line_offset_from270_mm',
    'final_relative_right_forward_mm','motion_observation_rate','stationary_scatter_rms_mm']},ensure_ascii=False),flush=True)
