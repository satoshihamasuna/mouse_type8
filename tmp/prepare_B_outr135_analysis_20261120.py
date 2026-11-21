"""検証済みの保存解析を今回B1600右135度出口へ明示入力で限定する。機器操作なし。"""
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
old_base='turn1600_B_inr135_20261120_v1'
new_base='turn1600_B_outr135_20261120_v1'
old_id='B_INR135_1600_baseline_green_20261120_002'
new_id='B_OUTR135_1600_baseline_green_20261120_001'
for before_name,after_name in [('review_turn1600_B_inr135_tracking_20261120.py','review_turn1600_B_outr135_tracking_20261120.py'),
    ('review_turn1600_B_inr135_midpoint_20261120.py','review_turn1600_B_outr135_midpoint_20261120.py'),
    ('review_B_inr135_saved_full_20261120.py','review_B_outr135_saved_full_20261120.py')]:
    source=(ROOT/'tmp'/before_name).read_text(encoding='utf-8-sig')
    source=source.replace(old_base,new_base).replace(old_id,new_id)
    source=source.replace("'in_r135'","'out_r135'").replace('右135度入口','右135度出口').replace('右135°入口','右135°出口')
    if 'midpoint' in after_name:
        assert source.count("raw['motion.pattern'].to_numpy()==11")==1
        source=source.replace("raw['motion.pattern'].to_numpy()==11","raw['motion.pattern'].to_numpy()==13")
        assert source.count('target=np.array([90.,135.])')==1
        source=source.replace('target=np.array([90.,135.])','target=np.array([67.5*np.sqrt(2),112.5*np.sqrt(2)])')
        assert source.count('exit_window=[lend_time+.015,pts(stop)-.025]')==1
        source=source.replace('exit_window=[lend_time+.015,pts(stop)-.025]',
            "exit_window=[lend_time,pts(stop)]  # 90mm直進後走の全制御区間。15点/30mm/RMSの品質条件は維持。")
        source=source.replace("label='目標中点90/135mm'","label='Lend目標 95.46/159.10mm'")
        source=source.replace('機体B1600 右135度出口 /','機体B1600 右135度出口 /')
    if 'saved_full' in after_name:
        source=source.replace("str(BASE/'baseline002_review.json')","str(ROOT/'tools/logs/pixel_camera/turn1600_B_inr135_20261120_v1/baseline002_review.json')")
        source=source.replace('review_turn1600_B_inr135_midpoint_20261120.py','review_turn1600_B_outr135_midpoint_20261120.py')
    compile(source,after_name,'exec')
    output=ROOT/'tmp'/after_name
    assert not output.exists()
    output.write_text(source,encoding='utf-8',newline='\n')
print('Prepared B1600 out_r135 tracking/full/midpoint analysis; hardware operations: 0')
