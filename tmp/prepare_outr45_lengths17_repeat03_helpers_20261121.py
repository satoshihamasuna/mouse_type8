from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
mapping={
 'retry_lengths_cminus1p7_Ls17p0_Le17p4_20261121_02':'repeat_lengths_cminus1p7_Ls17p0_Le17p4_20261121_03',
 'A_OUTR45_1800_cminus1p7_Ls17p0_Le17p4_20261121_002':'A_OUTR45_1800_cminus1p7_Ls17p0_Le17p4_20261121_003',
 'review_outr45_cminus1p7_lengths17_retry02_tracking_20261121.py':'review_outr45_cminus1p7_lengths17_repeat03_tracking_20261121.py',
}
for name in ('ready','analyze','confirm_off'):
    text=(ROOT/f'tmp/{name}_outr45_cminus1p7_lengths17_retry02_20261121.py').read_text(encoding='utf-8')
    for old,new in mapping.items():text=text.replace(old,new)
    if name=='ready':
        text=text.replace("operator_answers=['B側の録画走行を停止し、Aを続ける','両方ON・配置維持・手離し・範囲確認済み']", "operator_answers=['配置電源ON済み','B側停止・手離し・走行範囲確認済み']")
    with (ROOT/f'tmp/{name}_outr45_cminus1p7_lengths17_repeat03_20261121.py').open('x',encoding='utf-8') as file:file.write(text)
print('Prepared current-run helpers; no robot commands sent.')
