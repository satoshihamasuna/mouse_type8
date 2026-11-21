from pathlib import Path
from datetime import datetime,timezone
import json

ROOT=Path(__file__).resolve().parents[1]
STAGE=ROOT/'tools/logs/pixel_camera/turn1800_A_outr45_20261120_v1/repeat_lengths_cminus1p7_Ls17p0_Le17p4_20261121_03'
ID='A_OUTR45_1800_cminus1p7_Ls17p0_Le17p4_20261121_003'
record=dict(request_id=ID,both_powered_off=True,contact_detected=False,source='operator_report',
    confirmed_at=datetime.now(timezone.utc).isoformat(),operator_answer='停止・接触なし・両方OFFで待機')
for path in (STAGE/'local_execution/operator_results'/(ID+'.json'),
             STAGE/'local_execution'/('run_'+ID)/'operator_postrun_confirmation.json'):
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('x',encoding='utf-8') as file:
        json.dump(record,file,ensure_ascii=False,indent=2);file.write('\n')
print('Operator confirmed stopped, no contact, both OFF; saved to two new files.')
