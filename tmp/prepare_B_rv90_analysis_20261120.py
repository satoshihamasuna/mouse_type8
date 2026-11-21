"""既存のB保存解析を右V90へ明示入力で準備する。実機操作なし。"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
old_base = 'turn1600_B_outr135_20261120_v1'
new_base = 'turn1600_B_rv90_20261120_v1'
old_id = 'B_OUTR135_1600_baseline_green_20261120_001'
new_id = 'B_RV90_1600_baseline_green_20261120_001'
names = [
    ('review_turn1600_B_outr135_tracking_20261120.py', 'review_turn1600_B_rv90_tracking_20261120.py'),
    ('review_turn1600_B_outr135_midpoint_20261120.py', 'review_turn1600_B_rv90_midpoint_20261120.py'),
    ('review_B_outr135_saved_full_20261120.py', 'review_B_rv90_saved_full_20261120.py'),
]
for before, after in names:
    text = (ROOT/'tmp'/before).read_text(encoding='utf-8-sig')
    text = text.replace(old_base, new_base).replace(old_id, new_id)
    text = text.replace("'out_r135'", "'r_v90'").replace('右135度出口', '右V90').replace('右135°出口', '右V90')
    text = text.replace('review_turn1600_B_outr135_midpoint_20261120.py', 'review_turn1600_B_rv90_midpoint_20261120.py')
    if 'midpoint' in after:
        edits = [
            ("raw['motion.pattern'].to_numpy()==13", "raw['motion.pattern'].to_numpy()==15"),
            ('target=np.array([67.5*np.sqrt(2),112.5*np.sqrt(2)])',
             'target=np.array([45*np.sqrt(2),135*np.sqrt(2)])'),
            ('exit_path_error_from135_deg=angle-135.', 'exit_path_error_from90_deg=angle-90.'),
            ("label='Lend目標 95.46/159.10mm'", "label='Lend目標 63.64/190.92mm'"),
            ('90mm直進後走の全制御区間', '63.64mm斜め後走の全制御区間'),
            ('IN135005の検証済みcacheから高さ補正中点と入口raw wheel時計を単発評価する。',
             'B1600右V90の検証済みcacheから高さ補正中点と入口raw wheel時計を単発評価する。'),
        ]
        for old, new in edits:
            assert text.count(old) == 1, old
            text = text.replace(old, new)
    if 'saved_full' in after:
        text = text.replace("start=max(default_start,trial['video_motion_start_estimate_s']+.6)", 'start=default_start')
        text = text.replace('# 同じB1600/吸引650の基準動画ではRUN_START推定から実動開始まで約1.4秒。\n# 余白を残して解析開始を+0.6秒に置き、全走行・前後の観測条件は解析結果で検査する。',
                            '# V90の初回は既定の全評価窓を用い、前後の静止・全走行を保持する。')
        text = text.replace('同条件の初回実動遅延を参考に吸引待ちの前半を除外。必要走行区間の包含・観測95%・各フレーム実ArUco登録と前後静止観測の条件は維持。',
                            '初回の既定評価窓を維持。必要走行区間の包含・観測95%・各フレーム実ArUco登録と前後静止観測を検査する。')
    compile(text, after, 'exec')
    output = ROOT/'tmp'/after
    with output.open('x', encoding='utf-8', newline='\n') as stream:
        stream.write(text)
print('Prepared B1600 right V90 saved-data analysis; hardware operations: 0')
