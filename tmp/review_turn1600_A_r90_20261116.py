"""保存済みの一回走行だけを点検する。実機・カメラには接続しない。"""
import json
from pathlib import Path

import cv2
import numpy as np
import pandas as pd
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
RUN = ROOT / 'tools/logs/pixel_camera/tuning_gui_20261116_041226_ee682b/run_041508_bc4f21'
OUT = RUN / 'offline_review_v1'
OUT.mkdir(exist_ok=True)
trial = json.loads((RUN / 'trial.json').read_text(encoding='utf-8'))
targets = [7.5, 8, 8.5, 9, 9.5, 10, 10.5, 11, 12]
sheet = Image.new('RGB', (1280, 792), 'white')
video = cv2.VideoCapture(trial['video_file'])
video.set(cv2.CAP_PROP_ORIENTATION_AUTO, 0)
for index, timestamp in enumerate(targets):
    video.set(cv2.CAP_PROP_POS_MSEC, timestamp * 1000)
    ok, frame = video.read()
    if not ok:
        raise RuntimeError(f'Video frame unavailable at {timestamp}')
    tile = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)).resize((426, 240))
    x, y = (index % 3) * 426, (index // 3) * 264
    sheet.paste(tile, (x, y + 24))
    ImageDraw.Draw(sheet).text((x+8, y+5), f'video requested {timestamp:.3f} s', fill='black')
video.release()
sheet.save(OUT / 'video_contact_sheet.jpg')
raw = pd.read_csv(trial['type8_log_csv'])
turn = abs(raw['ideal.rad_velo']) > .01
moving = abs(raw['ideal.velo']) > .01
metrics = dict(rows=len(raw), maximum_gap_ms=float(raw.time_ms.diff().max()),
    ideal_velocity_max_m_s=float(raw['ideal.velo'].max()),
    measured_velocity_max_m_s=float(raw['ego.velo'].max()),
    moving_mcu_ms=raw.loc[moving, 'time_ms'].iloc[[0, -1]].tolist(),
    turning_mcu_ms=raw.loc[turn, 'time_ms'].iloc[[0, -1]].tolist(),
    turn_measured_speed_mean_m_s=float(raw.loc[turn, 'ego.velo'].mean()),
    turn_angular_tracking_rms_rad_s=float(np.sqrt(np.mean((raw.loc[turn, 'ego.rad_velo']-raw.loc[turn, 'ideal.rad_velo'])**2))),
    commanded_angle_integral_deg=float(np.rad2deg(np.trapezoid(raw['ideal.rad_velo'], raw.time_ms / 1000))),
    measured_angle_integral_deg=float(np.rad2deg(np.trapezoid(raw['ego.rad_velo'], raw.time_ms / 1000))))
(OUT / 'telemetry_metrics.json').write_text(json.dumps(metrics, indent=2), encoding='utf-8')
preparation = ROOT / 'tools/logs/pixel_camera/turn1600_A_r90_20261116_preflight_v1/preparation.json'
prep = json.loads(preparation.read_text(encoding='utf-8'))
prep.update(placement_confirmed=True, robot_power='Both OFF confirmed by user after one completed trial',
    robot_commands_sent=1, execution_write_count=1, execution_resend_count=0,
    trial_json=str(RUN / 'trial.json'), suction_runtime_verified=True,
    ble_disconnected=True, parameter_adoption_allowed=False)
preparation.write_text(json.dumps(prep, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
print(json.dumps(metrics, ensure_ascii=False, indent=2))
print(OUT / 'video_contact_sheet.jpg')
