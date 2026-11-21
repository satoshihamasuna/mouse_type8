"""指定out45試行の取得根拠を保存する。実機操作なし、既存資料は上書きしない。"""
from pathlib import Path
from datetime import datetime,timezone
import hashlib,json,sys
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_outr45_20261121_v1'
STAGE=(BASE/sys.argv[1]).resolve();identifier=sys.argv[2]
assert STAGE.is_relative_to(BASE) and '/' not in identifier and '\\' not in identifier
SESSION=STAGE/'local_execution';RUN=SESSION/('run_'+identifier)
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
operator=read(RUN/'operator_after_run.json')
assert operator['both_powered_off'] and operator['no_contact']
assert read(STAGE/'transport_binding_verified_20261121.json')['verified_binding']
sources={'runtime_modes.json':SESSION/'runtime_modes.json','preset_readback.settings.json':SESSION/'preset_readback.settings.json',
 'transport_binding_verified.json':STAGE/'transport_binding_verified_20261121.json',
 'charging_completion_confirmed.json':STAGE/'charging_completion_confirmed.json',
 'operator_ready.json':STAGE/f'operator_ready_{identifier}.json','camera_health_before_on.json':STAGE/'camera_health_20261121.json',
 'candidate_before_run.settings.json':STAGE/'candidate.settings.json','analysis_policy_before_run.json':BASE/'analysis_policy.json',
 'placement_plan_before_run.json':STAGE/'placement_plan.json'}
digests={}
for name,source in sources.items():
 data=source.read_bytes()
 with (RUN/name).open('xb') as f:f.write(data)
 digests[name]=dict(source=str(source),sha256=hashlib.sha256(data).hexdigest())
actual=read(RUN/'saved_1ms.settings.json');candidate=read(STAGE/'candidate.settings.json')
assert actual['mouse']=='B' and actual['motion_name']=='out_r45' and actual['preset_speed_mm_s']==1600
assert all(actual[k]==candidate[k] for k in ('parameters','pid','feedforward','suction','turn_count'))
events=[json.loads(row) for row in (RUN/'events.jsonl').read_text(encoding='utf-8').splitlines()]
assert len([e for e in events if e['event']=='execution_write_attempt'])==1
with (RUN/'acquisition_evidence.json').open('x',encoding='utf-8') as f:
 json.dump(dict(saved_at_utc=datetime.now(timezone.utc).isoformat(),mouse='B',motion='out_r45',preset=1600,
  execution_commands=1,actual_settings_match=True,physical_power='BOTH_OFF_USER_CONFIRMED',no_contact=True,evidence=digests,
  source_params_changed=False,firmware_write_performed=False),f,ensure_ascii=False,indent=2)
print('B out45 acquisition evidence saved; one execution; both OFF; no hardware operation')
