from pathlib import Path
from datetime import datetime,timezone
import hashlib
import json
import shutil

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_outr45_20261120_v1'
AUDIT=BASE/'firmware_out45_lr_adoption_20261121_v1'
DOC=ROOT/'tools/turn_analysis/20261121_A_turn1800_OUTR45_adoption'
def read(path):return json.loads(path.read_text(encoding='utf-8'))
def sha(path):
    with path.open('rb') as file:return hashlib.file_digest(file,'sha256').hexdigest()
def save(path,value):
    with path.open('x',encoding='utf-8') as file:json.dump(value,file,ensure_ascii=False,indent=2);file.write('\n')
verification=read(DOC/'firmware_lr_verification.json');assert verification['passed']
assert sha(ROOT/'Params/Param_A/turn_1800/turn_1800.cpp')==verification['source_sha256']
path=BASE/'README.md';old=path.read_text(encoding='utf-8');heading,tail=old.split('\n',1)
entry='''## 2026-11-21の左45度退出への鏡映追加

「ひだりもさいようして」により、左もLstart15.4 / Lend15.9 mm・半径+55 mm・角度補正+1.7度へ暫定採用した。論理角+45度、物理指令角+46.7度。右の採用値-55 mm・補正-1.7度は維持し、左右の既存方向別PID/FF参照も保持した。左右とも撮影方式フラグから独立させた。

Debug-MouseAのConfigure/Build、参照共有ホストテスト1件・スキップ0、生成ARM ELFの左右の形状・補正符号・PID/FF・mode_1800/acc_v2参照を確認済み。左実走と本体書き込みは未実施。右の実測2本を鏡映した仮定で、右の残差を左の実測結果とは扱わない。[採用記録](../../../turn_analysis/20261121_A_turn1800_OUTR45_adoption/README.md)と[状態v16](session_outr45_status_20261121_v16.json)を参照。

'''
path.write_text(heading+'\n\n'+entry+tail.lstrip('\n'),encoding='utf-8')
target=Path('D:/TYPE8_Camera/analyses/A_outr45_1800_provisional_LR_firmware_20261121_v1')
target.mkdir(exist_ok=False)
sources=[p for p in AUDIT.rglob('*') if p.is_file()]+[p for p in DOC.rglob('*') if p.is_file()]
sources += [ROOT/name for name in ('Params/Param_A/turn_1800/turn_1800.cpp','Params/Param_A/turn_1800/turn_1800.h',
 'README.md','Params/typedef_run_param.h','CMakePresets.json','CMakeLists.txt',
 'tmp/apply_A_out45_left_mirror_20261121.py','tmp/run_A_out45_lr_validation_20261121.py',
 'tmp/build_A_out45_lr_adoption_20261121.py','tmp/test_A_out45_lr_linkage_20261121.py',
 'tmp/verify_A_out45_lr_adoption_20261121.py','tmp/archive_A_out45_lr_adoption_20261121.py')]
sources += [BASE/'README.md',BASE/'session_outr45_status_20261121_v16.json']
files={}
for source in sorted(set(sources)):
    name=source.relative_to(ROOT).as_posix();destination=target/name
    destination.parent.mkdir(parents=True,exist_ok=True);before=sha(source);shutil.copy2(source,destination)
    assert before==sha(source)==sha(destination),name
    files[name]=dict(source=str(source),sha256=before,size_bytes=source.stat().st_size)
save(target/'manifest.json',dict(recorded_at_utc=datetime.now(timezone.utc).isoformat(),mouse='A',motion='OUT45_LR',
    status='USER_REQUESTED_PROVISIONAL_LEFT_MIRROR_SOURCE_BUILD_ELF_VERIFIED',files=files,
    source_files_deleted=False,robot_commands_sent=False,firmware_written=False,left_hardware_verified=False,
    previous_right_archive=read(BASE/'firmware_outR45_adoption_archive_receipt20261121_v1.json')))
for name,item in files.items():assert sha(target/name)==item['sha256'],name
receipt=dict(archive=str(target),manifest_sha256=sha(target/'manifest.json'),copied_and_verified_files=len(files),
    robot_commands_sent=False,firmware_written=False)
save(BASE/'firmware_out45_LR_adoption_archive_receipt20261121_v1.json',receipt)
print(json.dumps(receipt),flush=True)
