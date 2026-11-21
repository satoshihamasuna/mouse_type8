"""B1600学習動画から独立した保存動画の比較条件を固定する。機器操作なし。"""
from pathlib import Path
import json
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT/'tools/logs/pixel_camera/turn1600_B_r90_20261120_v1'
OUT = BASE/'vision_improvement_20261120_v1'


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


cases = []
for label,tag in [('static_noisy','green_Lstart13p6_repeat03_placed01'),
                  ('static_recent','green_correction1p0_placed02')]:
    capture = read(BASE/(tag+'_static_capture.json'))
    cases.append(dict(label=label,kind='static',video=capture['video_file'],
        geometry=str(BASE/'body_geometry_B_green_front_v1.json'),reference=str(BASE/'marker_reference_B_current_rotated.json'),
        window=[1.5,2.3],stride=4,review_frames=[400,480]))
for label,relative,window,frames in [
    ('B1600_004','cli_session06/run_B_R90_1600_Lstart13p6_green_20261120_004',[9.35,9.9],[2304,2320]),
    ('B1600_005','cli_session07/run_B_R90_1600_Lstart13p6_green_20261120_005',[9.63,10.14],[2368,2384]),
    ('B1600_corr1','cli_session08/run_B_R90_1600_Lstart13p6_corr1p0_green_20261120_001',[9.08,9.62],[2236,2252])]:
    folder = BASE/relative
    trial = read(folder/'trial.json')
    cases.append(dict(label=label,kind='motion',trial=str(folder/'trial.json'),video=trial['video_file'],
        geometry=str(folder/'body_geometry_used.json'),reference=str(folder/'marker_reference_used.json'),
        window=window,stride=4,review_frames=frames))
hard = ROOT/'tools/logs/pixel_camera/turn2000_B_r90_20261120_v1/cli_green_session01/run_B_R90_2000_omKi005_green_20261120_001'
trial = read(hard/'trial.json')
pose = pd.read_csv(hard/'full/apparent_floor_marker_pose.csv')
stamps = pose[pose.frame.between(2172,2200)].time_video_s
assert len(stamps)>10
cases.append(dict(label='B2000_known_mispair',kind='motion',trial=str(hard/'trial.json'),
    video=trial['video_file'],geometry=str(hard/'body_geometry_used.json'),reference=str(hard/'marker_reference_used.json'),
    window=[float(stamps.min())-.0001,float(stamps.max())+.0001],stride=1,review_frames=[2185,2186,2193,2194]))
manifest = dict(scope='SCREENING_FIXED_WINDOWS_NOT_WHOLE_VIDEO_ACCEPTANCE',
    training_trial=str(BASE/'cli_session04/run_B_R90_1600_Lstart13p6_green_20261120_001/trial.json'),
    cases=cases,positive_accuracy_ground_truth='No independent subpixel manual labels; compare coverage/jumps/static scatter and inspect source-frame identity.')
with (OUT/'screening_manifest.json').open('x',encoding='utf-8') as output:
    json.dump(manifest,output,ensure_ascii=False,indent=2)
print(json.dumps(dict(cases=len(cases),independent_train_trial=manifest['training_trial'])))
