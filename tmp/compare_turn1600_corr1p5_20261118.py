"""機体A・Lstart12の補正+1.0度3本と+1.5度1本を比較。実機操作なし。"""
from pathlib import Path
import copy
import hashlib
import json

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tools/logs/pixel_camera/turn1600_A_r90_20261118_frame18_v1'
PREVIOUS = BASE / 'repeatability_corr1_Ls12p0_03'
CURRENT = BASE / 'repeatability_corr1p5_Ls12p0_01'
OUT = BASE / 'comparison_corr1p5_Ls12p0_v1'


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


previous = read(PREVIOUS / 'video_comparison.json')
current = read(CURRENT / 'video_comparison.json')
assert previous['repeat_count'] == 3 and current['repeat_count'] == 1
assert previous['all_relative_reviews_eligible'] and current['all_relative_reviews_eligible']
control_fingerprints = []
profile_fingerprints = []
reference_fingerprints = []
cases = []
for source, document, correction, colour in (
        (PREVIOUS, previous, 1., 'tab:blue'),
        (CURRENT, current, 1.5, 'tab:orange')):
    for case in document['comparisons']:
        assert case['correction_deg'] == correction
        run = Path(case['source_trial']).parent
        settings = read(run / 'saved_1ms.settings.json')
        controls = copy.deepcopy({key: settings[key] for key in (
            'mouse', 'motion_name', 'preset_speed_mm_s', 'turn_count',
            'parameters', 'pid', 'feedforward', 'suction')})
        assert controls['parameters'].pop('correction') == correction
        assert controls['parameters']['lstart'] == 12.
        assert controls['parameters']['lend'] == 26.31
        control_fingerprints.append(json.dumps(controls, sort_keys=True))
        geometry = read(run / 'full/body_geometry_used.json')
        # 対象動画の記録欄は各試行で異なる。測定・検出条件は全項目照合する。
        geometry.pop('measurement_video_file', None)
        profile_fingerprints.append(hashlib.sha256(
            json.dumps(geometry, sort_keys=True).encode('utf-8')).hexdigest())
        reference_fingerprints.append(hashlib.sha256(
            (run / 'marker_reference_used.json').read_bytes()).hexdigest())
        cases.append((source, case, correction, colour))
assert len(set(control_fingerprints)) == 1
assert len(set(profile_fingerprints)) == 1
assert len(set(reference_fingerprints)) == 1

OUT.mkdir(exist_ok=False)
fig, axes = plt.subplots(1, 3, figsize=(14, 4.4), layout='constrained')
labels = []
for number, (source, case, correction, colour) in enumerate(cases):
    path = pd.read_csv(source / (case['label'] + '_relative_trajectory.csv'))
    valid = path.detected.to_numpy(bool)
    label = f'c+{correction:.1f} {case["label"]}'
    labels.append(label)
    axes[0].plot(path.right_mm[valid], path.forward_mm[valid],
        color=colour, alpha=.65 if correction == 1. else 1.,
        linewidth=1.3 if correction == 1. else 2.2, label=label)
    axes[1].scatter(number, case['path_rotation_error_from90_deg'], color=colour)
    axes[2].bar(number, case['exit_line_offset_from270_mm'], color=colour, alpha=.8)
axes[0].axhline(270, color='black', linestyle='--', linewidth=.8)
axes[0].scatter(90, 270, marker='x', color='black', label='Nominal turn-end plan')
axes[0].set(xlabel='Right displacement [mm]', ylabel='Forward displacement [mm]')
axes[0].axis('equal')
axes[0].legend(fontsize=7)
for ax in axes[1:]:
    ax.axhline(0, color='black', linewidth=.8)
    ax.set_xticks(range(len(cases)), labels, rotation=30, ha='right', fontsize=8)
axes[1].set(ylabel='Entry-to-exit path rotation minus90 [deg]')
axes[2].set(ylabel='Exit-line forward offset at right90mm [mm]')
for ax in axes:
    ax.grid(alpha=.25)
fig.savefig(OUT / 'video_path_comparison.png', dpi=160)
plt.close(fig)

new = current['comparisons'][0]
deltas = {}
for key in ('entry_to_exit_path_rotation_deg', 'exit_line_offset_from270_mm'):
    deltas[key] = new[key] - previous['repeat_statistics'][key]['mean']
document = dict(mouse='A', motion='long_r90', Lstart_mm=12., Lend_mm=26.31,
    previous_correction_deg=1., current_correction_deg=1.5,
    previous_review=str(PREVIOUS / 'video_comparison.json'),
    current_review=str(CURRENT / 'video_comparison.json'),
    previous_repeat_count=3, current_repeat_count=1,
    previous_statistics=previous['repeat_statistics'], current_metrics=new,
    current_minus_previous_mean=deltas,
    only_changed_device_setting='parameters.correction',
    other_device_settings_identical=True, identical_analysis_geometry=True,
    excluded_geometry_metadata_fields=['measurement_video_file'],
    identical_registered_board_reference=True, all_relative_reviews_eligible=True,
    reference_frame='Relative marker midpoint and measured entry direction.',
    new_candidate_repeatability_established=False,
    source_params_changed=False, parameter_adoption_allowed=False,
    power_state=current['power_state'], ble_disconnected=current['ble_disconnected'],
    execution_pending=False,
    limitations=['New correction has only one run; comparison to3run mean does not establish a causal effect or repeatability.',
        'Fixed-board trajectory, absolute axle position and height-corrected body yaw remain unverified.',
        'Original false automatic-stop summaries remain preserved; late stable windows reviewed separately.'],
    source_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    input_sha256={str(folder.relative_to(ROOT) / 'video_comparison.json'):
        hashlib.sha256((folder / 'video_comparison.json').read_bytes()).hexdigest()
        for folder in (PREVIOUS, CURRENT)})
with (OUT / 'comparison.json').open('x', encoding='utf-8') as stream:
    json.dump(document, stream, ensure_ascii=False, indent=2, allow_nan=False)
    stream.write('\n')
print(json.dumps(dict(current_metrics={key: new[key] for key in (
    'entry_to_exit_path_rotation_deg', 'exit_line_offset_from270_mm',
    'encoder_model_fit_rmse_mm', 'motion_observation_rate', 'motion_calibration_rate',
    'relative_review_eligible')}, current_minus_previous_mean=deltas), indent=2))
