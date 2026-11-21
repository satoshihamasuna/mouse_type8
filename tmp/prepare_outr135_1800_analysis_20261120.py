"""既存の実測中点・時計評価をout135の形状と目標へ適用する測定用コピー。"""
from pathlib import Path
import ast
import json
import hashlib

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tools/logs/pixel_camera/turn1800_A_outr135_20261120_v1'
source = ROOT / 'tmp/review_inr135_midpoint_trial_20261120.py'
target = ROOT / 'tmp/review_outr135_midpoint_trial_20261120.py'
text = source.read_text(encoding='utf-8-sig')
replacements = {
    "BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_inr135_20261120_v1'":
        "BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_outr135_20261120_v1'",
    "turn_index=np.flatnonzero(raw['motion.pattern'].to_numpy()==11)[0]":
        "turn_index=np.flatnonzero(raw['motion.pattern'].to_numpy()==13)[0]",
    'target=np.array([90.,135.])': 'target=np.array([135./np.sqrt(2),225./np.sqrt(2)])',
    '目標中点90/135mm': '目標中点95.46/159.10mm',
    "title=f'IN135 midpoint / {angle:.3f}°'": "title=f'OUT135 midpoint / {angle:.3f}°'",
}
for before, after in replacements.items():
    assert text.count(before) == 1, before
    text = text.replace(before, after)
assert text.count("'in_r135'") == 2
text = text.replace("'in_r135'", "'out_r135'")
ast.parse(text)
with target.open('x', encoding='utf-8', newline='\n') as file:
    file.write(text)
source_tracking = ROOT / 'tmp/parallel_outr135_tracking_20261120.py'
target_tracking = ROOT / 'tmp/review_outr135_tracking_trial_20261120.py'
text = source_tracking.read_text(encoding='utf-8-sig')
before = "OUT = ROOT / 'tools/logs/pixel_camera/turn1800_A_outr135_20261120_v1/parallel_analysis/baseline001_early/tracking_1ms'"
assert text.count(before) == 1
text = text.replace(before, "OUT = Path(sys.argv[2]).resolve()\nassert OUT.is_relative_to((ROOT/'tools/logs/pixel_camera/turn1800_A_outr135_20261120_v1').resolve())")
ast.parse(text)
with target_tracking.open('x', encoding='utf-8', newline='\n') as file:
    file.write(text)
report = dict(mouse='A', motion='out_r135', pattern=13,
    nominal_Lend_right_forward_mm=[135/2**.5,225/2**.5],
    target_basis='planned start180/135 heading45deg to Lend225/315mm, projected onto right/forward axes',
    scope='Relative marker midpoint; nonzero-parameter comparison; no Lstart=Lend=0 inverse method.',
    quality_limits_unchanged=True, code_syntax_validated=True, hardware_operations=0,
    adapted_scripts={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (target,target_tracking)})
with (BASE / 'resume_20261120_v1/analysis_preparation.json').open('x', encoding='utf-8') as file:
    json.dump(report, file, ensure_ascii=False, indent=2)
print('OUT135 tracking and relative midpoint analysis prepared; hardware operations: 0')
