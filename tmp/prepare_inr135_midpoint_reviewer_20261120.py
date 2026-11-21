"""既存の中点評価を今回の明示runと保存先に適用する測定用コピーを作る。"""
from pathlib import Path
import ast

root = Path(__file__).resolve().parents[1]
source = root / 'tmp/parallel_inr135_midpoint005_20261120.py'
output = root / 'tmp/review_inr135_midpoint_trial_20261120.py'
text = source.read_text(encoding='utf-8-sig')
replacements = {
    "RUN=BASE/'baseline01/local_execution/run_A_INR135_1800_baseline_20261120_005'":
        "RUN=Path(sys.argv[1]).resolve()",
    "OUT=BASE/'parallel_analysis/A_INR135_1800_baseline_20261120_005/midpoint_raw_wheel_clock_v1'":
        "OUT=Path(sys.argv[2]).resolve()\nassert OUT.is_relative_to(BASE.resolve())",
    'request_Lstart_mm=float(22.85+delta_ls)':
        "request_Lstart_mm=float(settings['parameters']['lstart']+delta_ls)",
    'request_Lend_mm=float(34.81+delta_le)':
        "request_Lend_mm=float(settings['parameters']['lend']+delta_le)",
    'existing_angle_correction_candidate_deg=3.5':
        "current_angle_correction_deg=settings['parameters']['correction']",
    "title=f'IN135005 midpoint / {angle:.3f}°'":
        "title=f'IN135 midpoint / {angle:.3f}°'",
    'OUT.mkdir(exist_ok=False)':
        "result['current_parameters']=settings['parameters']\n"
        "result['source_analysis_template']=str(ROOT/'tmp/parallel_inr135_midpoint005_20261120.py')\n"
        "result['limitations']=[\n"
        "    '全raw観測率と時計品質を保持。独立hardware同期・絶対尺度の校正ではない。',\n"
        "    '高さ一定を仮定した相対マーカー中点。車軸・機軸の絶対校正ではない。',\n"
        "    '長さ解は今回の出口方向・半径・角度補正を維持する局所候補。変更後は再計測する。',\n"
        "    '元のsummaryと軌跡モデルRMSは別評価として保持。'\n"
        "]\nOUT.mkdir(exist_ok=False)",
}
for before, after in replacements.items():
    assert text.count(before) == 1, before
    text = text.replace(before, after)
ast.parse(text)
with output.open('x', encoding='utf-8', newline='\n') as file:
    file.write(text)
print(output)
