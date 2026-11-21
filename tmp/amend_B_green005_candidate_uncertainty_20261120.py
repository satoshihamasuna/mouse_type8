"""未適用候補の不確かさと1項目変更を確認する。機器操作なし。"""
from pathlib import Path
import hashlib
import json
import shutil
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from pixel_camera_tuning import settings_set_command, setting_differences

BASE = ROOT/'tools/logs/pixel_camera/turn1600_B_r90_20261120_v1'
RUN = BASE/'cli_session07/run_B_R90_1600_Lstart13p6_green_20261120_005'
REVIEW = BASE/'green_Lstart13p6_three_review_v1'


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def write(path,value):
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')


assert not (RUN/'archive.json').exists()
actual = read(BASE/'device_prepare_green_Lstart13p6_05/actual.settings.json')
candidate = read(BASE/'candidate_green_correction1p0_Lstart13p6.settings.json')
settings_set_command(candidate)
differences = setting_differences(actual,candidate,quantized=True)
assert len(differences)==1, differences
for key in ('mouse','motion_name','preset_speed_mm_s','turn_count','pid','feedforward','suction'):
    assert actual[key]==candidate[key],key
assert actual['parameters']['correction']==1.5 and candidate['parameters']['correction']==1.0
assert {k:v for k,v in actual['parameters'].items() if k!='correction'}=={k:v for k,v in candidate['parameters'].items() if k!='correction'}
comparison = read(REVIEW/'video_comparison.json')
voltage = []
for case in comparison['comparisons']:
    tracking = read(Path(case['source_trial']).parent/'tracking_1ms_review_v2/verification.json')
    voltage.append(dict(label=case['label'],battery_min_v=tracking['battery_min_v'],
        turn_speed_mean_mm_s=tracking['turn_speed_mean_mm_s']))
change = read(REVIEW/'correction1p0_candidate_not_applied.json')
change.update(all_three_under90_basis='事前に固定した代表の入口・出口窓での測定値。',
    robust_under90_in_all_sensitivity_windows=False,
    sensitivity_limitation='005は入口窓感度で89.81〜90.05度。全ての窓で90度未満とは断定しない。0.5度変更の効果は候補の独立反復で評価する。',
    baseline_battery_and_speed=voltage,offline_settings_validated=True,only_numeric_change_verified=True)
for path in (REVIEW/'correction1p0_candidate_not_applied.json',
             RUN/'video_shape_review_v1/correction1p0_candidate_not_applied.json'):
    write(path,change)
for path in (RUN/'manual_quality_review.json',REVIEW/'green005_manual_quality_review.json',
             RUN/'video_shape_review_v1/green005_manual_quality_review.json'):
    qa = read(path)
    qa['next_candidate_not_applied'] = change
    qa['analysis_source_change_note'] = '任意のbody_model対応が追加された解析実装。今回の3動画の同一プロファイルにはbody_model設定がなく、従来の2色選択経路のみを使用。'
    write(path,qa)
destination = RUN/'analysis_sources'/Path(__file__).name
shutil.copy2(__file__,destination)
manifest_path = RUN/'analysis_sources/source_hashes.json'
manifest = read(manifest_path)
manifest[str(Path(__file__).relative_to(ROOT))] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
write(manifest_path,manifest)
print(json.dumps(dict(candidate_validated=True,only_numeric_change=differences,
    baseline_battery_and_speed=voltage,applied=False),ensure_ascii=False))
