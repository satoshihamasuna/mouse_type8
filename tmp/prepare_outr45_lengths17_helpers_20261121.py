from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
mapping={
 'repeat_cminus1p7_Ls15p4_Le15p9_20261121_02':'lengths_cminus1p7_Ls17p0_Le17p4_20261121_01',
 'A_OUTR45_1800_cminus1p7_Ls15p4_Le15p9_20261121_002':'A_OUTR45_1800_cminus1p7_Ls17p0_Le17p4_20261121_001',
 'review_outr45_cminus1p7_lengths15_repeat02_tracking_20261121.py':'review_outr45_cminus1p7_lengths17_tracking_20261121.py',
}
for name in ('ready','analyze','confirm_off'):
    source=ROOT/f'tmp/{name}_outr45_cminus1p7_lengths15_repeat02_20261121.py'
    target=ROOT/f'tmp/{name}_outr45_cminus1p7_lengths17_20261121.py'
    text=source.read_text(encoding='utf-8')
    for old,new in mapping.items():text=text.replace(old,new)
    if name=='ready':
        text=text.replace('15.4','17.0').replace('15.9','17.4')
        text=text.replace('search370_B_20261121_v1/right_baseline01/local_execution/jobs','search370_B_20261121_v1')
        text=text.replace("other.glob('*.json')","other.glob('*/local_execution/jobs/*.json')")
    with target.open('x',encoding='utf-8') as file:file.write(text)
print('Prepared validated-template helpers; no robot commands sent.')
