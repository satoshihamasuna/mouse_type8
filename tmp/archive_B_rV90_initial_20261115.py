from pathlib import Path
import hashlib,json,shutil
ROOT=Path.cwd();BASE=ROOT/'tools/turn_analysis/20261114_B_turn500_seed_from_A'
files=[]
for name in ['A_rV90_reference_check','rV90_start315_90_run01_review','rV90_start315_90_run01_dense','rV90_start315_90_run01_control','rV90_start315_90_run01_static_cache']:
    files.extend((p,Path(name)/p.relative_to(BASE/name)) for p in (BASE/name).rglob('*') if p.is_file() and '__pycache__' not in p.parts)
for p in BASE.iterdir():
    if p.is_file() and ('rV90' in p.name or 'r_V90' in p.name) and 'archive_verification' not in p.name:
        files.append((p,Path('plans_and_tools')/p.name))
for name in ['nominal_inR135.py','body_geometry_B_zoom2_v8.json','zoom_camera_validation_v8.json']:
    files.append((BASE/name,Path('plans_and_tools')/name))
for session in ['tuning_cli_B_rV90_Aseed_20261115_prepared','tuning_cli_B_rV90_start315_90_20261115_prepared']:
    source=ROOT/'tools/logs/pixel_camera'/session
    for name in ['power_off_confirmation.json','operator_ready.json','static_preflight.json','static_preflight_check.json','static_preflight_frame.png','B_rV90_candidate.settings.json']:
        files.append((source/name,Path('operator_and_static')/session/name))
files.append((ROOT/'.agents/skills/mouse-turn-tuning/references/start-placements.md',Path('current_start_registry.md')))
archives=[]
for recording,kind in [('20261115_202348_240fps_47da931f','STATIC_ONLY_NO_MOTOR_SUBMISSION'),('20261115_203212_240fps_dd7cb6b2','ONE_PHYSICAL_RV90_TRIAL')]:
    output=Path('D:/TYPE8_Camera/trials')/recording/'B_rV90_initial_review_and_next_candidate'
    output.mkdir(parents=True,exist_ok=False)
    entries=[]
    for source,relative in files:
        target=output/relative;target.parent.mkdir(parents=True,exist_ok=True)
        assert not target.exists();shutil.copy2(source,target)
        digest=hashlib.sha256(source.read_bytes()).hexdigest()
        assert hashlib.sha256(target.read_bytes()).hexdigest()==digest
        entries.append(dict(source=str(source),archive=str(target),sha256=digest))
    result=dict(state='COPIED_AND_SHA256_VERIFIED',recording_kind=kind,files=entries,
        physical_recording_id='20261115_203212_240fps_dd7cb6b2',static_only_recording_id='20261115_202348_240fps_47da931f',
        original_failure_and_reference_copies_preserved=True)
    (output/'manifest.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    archives.append(dict(recording_id=recording,archive=str(output),files=len(entries),kind=kind))
(BASE/'rV90_initial_and_static_archive_verification.json').write_text(json.dumps(dict(state='TWO_RECORDING_SUPPLEMENTS_SHA256_VERIFIED',archives=archives),indent=2)+'\n',encoding='utf-8')
print(json.dumps(archives))
