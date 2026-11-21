from pathlib import Path
import os
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'tools/logs/pixel_camera/turn1800_A_outr45_20261120_v1/firmware_out45_lr_adoption_20261121_v1'
text=(ROOT/'tmp/build_A_outr45_adoption_20261121.py').read_text(encoding='utf-8')
text=text.replace('firmware_outR45_adoption_20261121_v1','firmware_out45_lr_adoption_20261121_v1')
target=ROOT/'tmp/build_A_out45_lr_adoption_20261121.py'
with target.open('x',encoding='utf-8') as file:file.write(text)
raise SystemExit(subprocess.call([sys.executable,str(target)],cwd=ROOT,env=dict(os.environ,PYTHONUTF8='1')))
