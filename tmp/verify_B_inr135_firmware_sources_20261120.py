"""今回の準備時ハッシュと現在のファームソースを照合する。実機操作なし。"""
from pathlib import Path
import hashlib
import json
import shutil
ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr135_20261120_v1'
OUT=BASE/'geometry_candidate_review_v1'
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
checks={}
for name,expected in read(BASE/'preparation.json')['source_hashes'].items():
    if name.replace('\\','/').startswith(('Params/','System/','Task/')):
        actual=hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
        checks[name]=dict(expected_sha256=expected,actual_sha256=actual,unchanged=actual==expected)
assert len(checks)==4 and all(item['unchanged'] for item in checks.values())
with (OUT/'firmware_source_verification.json').open('x',encoding='utf-8') as file:
    json.dump(dict(source=str(BASE/'preparation.json'),checks=checks,hardware_operations=0),file,ensure_ascii=False,indent=2)
path=OUT/'review.json';shutil.copyfile(path,OUT/'review_before_source_hash_verification.json')
review=read(path)
review['firmware_source_hashes_unchanged']={name:item['unchanged'] for name,item in checks.items()}
review['firmware_source_verification']=str(OUT/'firmware_source_verification.json')
path.write_text(json.dumps(review,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
print(json.dumps(review['firmware_source_hashes_unchanged'],ensure_ascii=False))
