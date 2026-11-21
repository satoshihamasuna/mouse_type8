from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
source=(ROOT/'tmp/visual_check_outr45_lengths17_pair_20261121.py').read_text(encoding='utf-8')
source=source.replace("OUT=BASE/'repeat_lengths_cminus1p7_Ls17p0_Le17p4_20261121_03/visual_trajectory_review_v1'",
    "OUT=BASE/'firmware_outR45_adoption_20261121_v1/selected_visual_review_v1'")
mapping={
 'retry_lengths_cminus1p7_Ls17p0_Le17p4_20261121_02':'lengths_cminus1p7_Ls15p4_Le15p9_20261121_01',
 'A_OUTR45_1800_cminus1p7_Ls17p0_Le17p4_20261121_002':'A_OUTR45_1800_cminus1p7_Ls15p4_Le15p9_20261121_001',
 'repeat_lengths_cminus1p7_Ls17p0_Le17p4_20261121_03':'repeat_cminus1p7_Ls15p4_Le15p9_20261121_02',
 'A_OUTR45_1800_cminus1p7_Ls17p0_Le17p4_20261121_003':'A_OUTR45_1800_cminus1p7_Ls15p4_Le15p9_20261121_002',
}
for old,new in mapping.items():source=source.replace(old,new)
with (ROOT/'tmp/visual_check_selected_outr45_lengths15_20261121.py').open('x',encoding='utf-8') as file:file.write(source)
print('Prepared selected-run visual review; original data retained.')
