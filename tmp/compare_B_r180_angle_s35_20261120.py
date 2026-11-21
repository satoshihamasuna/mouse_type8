"""B180度の同じS35検出・入口相対基準で、角度補正だけの比較を保存する。"""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_r180_20261120_v1'
read=lambda p:json.loads(Path(p).read_text(encoding='utf-8-sig'))
cases=[('補正＋2.0°',BASE/'baseline001_video_s35_apex26_v4'),
       ('補正＋0.5°',BASE/'angle_c0p5_s35_002_video_review_v2')]
inputs=[]
for label,folder in cases:
    review=read(folder/'video_comparison.json')['comparisons'][0]
    run=Path(review['source_trial']).parent
    settings=read(run/'saved_1ms.settings.json')
    geometry=read(review['source_profile'])
    inputs.append(dict(label=label,folder=folder,review=review,settings=settings,
        detection=geometry['marker_detection'],tracking=read(run/'tracking_1ms_review_v2/verification.json')))
first,second=inputs
assert first['detection']==second['detection']
for group in ('pid','feedforward','suction','turn_count','preset_speed_mm_s','mouse','motion_name'):
    assert first['settings'][group]==second['settings'][group],group
for key in first['settings']['parameters']:
    if key!='correction':assert first['settings']['parameters'][key]==second['settings']['parameters'][key],key
OUT=BASE/'angle_correction_comparison_s35_v1';OUT.mkdir(exist_ok=False)
plt.rcParams['font.family']='Yu Gothic'
fig,axes=plt.subplots(1,2,figsize=(10,4.7),layout='constrained')
rows=[]
for case in inputs:
    r=case['review'];xy_path=case['folder']/'baseline001_relative_trajectory.csv'
    if not xy_path.exists():xy_path=case['folder']/'c0p5_002_relative_trajectory.csv'
    import pandas as pd
    path=pd.read_csv(xy_path);valid=path.detected.to_numpy(bool)
    axes[0].plot(path.right_mm[valid],path.forward_mm[valid],label=case['label'])
    rows.append(dict(label=case['label'],correction_deg=case['settings']['parameters']['correction'],
        rotation_deg=r['entry_to_exit_path_rotation_deg'],spacing_mm=r['exit_line_right_at_forward45_mm'],
        apex_mm=r['forward_apex_mm'],observation_rate=r['motion_observation_rate'],
        relative_review_eligible=r['relative_review_eligible'],
        entry_board_heading_deg=r['entry_board_heading_deg'],exit_board_heading_deg=r['exit_board_heading_deg'],
        battery_min_v=case['tracking']['battery_min_v'],
        tracking_rms_rad_s=case['tracking']['tracking']['tracking_rms_rad_s'],
        source_review=str(case['folder']/'video_comparison.json')))
axes[0].axvline(90,c='gray',ls=':',lw=.8);axes[0].axhline(185,c='gray',ls=':',lw=.8)
axes[0].set(xlabel='入口から右方向 [mm]',ylabel='入口から前方向 [mm]',title='実測入口に合わせた相対軌跡')
axes[0].axis('equal');axes[0].legend();axes[0].grid(alpha=.3)
values=[r['rotation_deg'] for r in rows]
axes[1].bar([r['label'] for r in rows],values,color=['tab:blue','tab:orange'])
axes[1].axhline(180,c='black',ls='--',lw=1,label='目標180°')
axes[1].set(ylim=(min(values+[180])-1,max(values+[180])+1),ylabel='入口→出口の回転量 [deg]',title='出口直線の方向')
for i,value in enumerate(values):axes[1].text(i,value+.1,f'{value:.2f}°',ha='center')
axes[1].legend();axes[1].grid(axis='y',alpha=.3)
fig.savefig(OUT/'comparison.png',dpi=150);plt.close(fig)
with (OUT/'comparison.json').open('x',encoding='utf-8') as output:
    json.dump(dict(mouse='B',motion='long_r180',preset_mm_s=1600,comparisons=rows,
        only_commanded_turn_parameter_changed='correction',same_detection_verified=True,
        measured_relative_rotation_change_deg=rows[1]['rotation_deg']-rows[0]['rotation_deg'],
        caveats=['独立反復は各条件1本。電圧と開始姿勢は異なる。','実測入口基準の形状比較。盤面固定・絶対車軸精度の合格ではない。'],
        parameter_adoption_allowed=False,source_params_changed=False),output,ensure_ascii=False,indent=2)
print(json.dumps(rows,ensure_ascii=False),flush=True)
