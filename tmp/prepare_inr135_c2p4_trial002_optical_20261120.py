"""保持ログを失った002を、元動画の相対方向だけで診断するコピーを準備する。"""
from pathlib import Path
import ast

root = Path(__file__).resolve().parents[1]
source = root / 'tmp/review_inr135_resume_optical_20261120.py'
output = root / 'tmp/review_inr135_c2p4_trial002_optical_20261120.py'
text = source.read_text(encoding='utf-8-sig')
changes = {
    "RUN = BASE / 'c3p5_Ls7p85_01/local_execution/run_A_INR135_1800_c3p5_Ls7p85_req_20261120_001'":
        "RUN = BASE / 'c2p4_Ls2p5_Le26_02/local_execution/run_A_INR135_1800_c2p4_Ls2p5_Le26_req_20261120_002'",
    "OUT = BASE / 'resume_20261120_inr135_v1/optical_diagnostic'":
        "OUT = BASE / 'c2p4_Ls2p5_Le26_02/optical_diagnostic_v1'",
    "trial['state'] == 'INCOMPLETE_LOG'": "trial['state'] == 'FAILED'",
    "state='OPTICAL_DIAGNOSTIC_ONLY_INCOMPLETE_LOG'": "state='OPTICAL_DIAGNOSTIC_ONLY_MISSING_SAVED_1MS_LOG'",
    "BASE / 'c3p5_Ls7p85_01/candidate.settings.json'": "BASE / 'c2p4_Ls2p5_Le26_02/candidate.settings.json'",
    '1 msログ未取得・連続ログ終端欠損のため通常の試行品質は不合格のまま。':
        '1 msログ未取得のため通常の試行品質は不合格のまま。連続ログは停止まで保存できている。',
}
for before, after in changes.items():
    assert text.count(before) == 1, before
    text = text.replace(before, after)
ast.parse(text)
with output.open('x', encoding='utf-8', newline='\n') as file:
    file.write(text)
print('Video-only diagnostic prepared; original FAILED state preserved; no robot commands')
