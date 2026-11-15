"""既存の青色LED点灯とログのターン入口リセットを照合する。実機操作なし。"""
import hashlib
import json
from pathlib import Path

import numpy as np
import pandas as pd

ROOT=Path(__file__).resolve().parents[3]
OUTPUT=Path(__file__).resolve().parent


def read(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def review():
    manifest=ROOT/"tools/turn_analysis/20261112_A_speed_prediction_v2/model.json"
    model=read(manifest)
    rows=[]
    for motion,entries in model["inputs"].items():
        for entry in entries:
            paths=list(entry["input_sha256"])
            csv=Path(next(p for p in paths if p.endswith("motion.csv")))
            full=Path(next(p for p in paths if p.endswith("normalized_trajectory.csv"))).parent
            raw=pd.read_csv(csv)
            pose=pd.read_csv(full/"apparent_floor_marker_pose.csv")
            report=read(full/"report.json")
            start=entry["settings"]["log"]["motion_start_time_ms"]
            t=(raw.time_ms.to_numpy()-start)/1000
            video_old_t=pose.time_video_s.to_numpy()-report["log_time_zero_at_video_s"]+(raw.time_ms.iloc[0]-start)/1000
            omega=abs(raw["ideal.rad_velo"].to_numpy())
            turning=np.flatnonzero(omega>.02*omega.max())
            pre=180 if motion in ("long_l90","long_r90") else (90 if motion.startswith("in_") or motion.endswith("180") else 90*np.sqrt(2))
            reset=np.flatnonzero((np.diff(raw["ego.length"],prepend=0)<-.5*pre)&(t>0)&(t<t[turning[0]])&(raw["ideal.velo"].to_numpy()>.4))
            if len(reset)!=1:
                raise ValueError(f"Ambiguous approach-to-turn transition: {entry['id']}")
            i=int(reset[0])
            blue=pose.turn_led_blue_pixels.to_numpy()
            background=blue[(video_old_t>-.15)&(video_old_t<.08)]
            if len(background)<10:
                raise ValueError("Insufficient LED-off background")
            threshold=max(5,float(np.percentile(background,99))+3)
            high=blue>threshold
            on=np.flatnonzero(high)
            groups=np.split(on,np.flatnonzero(np.diff(on)>1)+1)
            groups=[g for g in groups if len(g)>=5 and abs(video_old_t[g[0]]-t[i])<.08]
            if len(groups)!=1:
                raise ValueError(f"Ambiguous persistent LED onset: {entry['id']}")
            g=groups[0]; first=int(g[0]); last=int(g[-1])
            if first==0 or last+1==len(pose):
                raise ValueError("LED transition outside analyzed video")
            log_mid=(t[i-1]+t[i])/2
            video_mid=(video_old_t[first-1]+video_old_t[first])/2
            rows.append(dict(motion=motion,trial=entry["id"],led_threshold_pixels=threshold,
                log_entry_before_s=float(t[i-1]),log_entry_after_s=float(t[i]),
                video_on_before_old_alignment_s=float(video_old_t[first-1]),video_on_after_old_alignment_s=float(video_old_t[first]),
                video_on_before_pts_s=float(pose.time_video_s.iloc[first-1]),video_on_after_pts_s=float(pose.time_video_s.iloc[first]),
                onset_correction_to_mcu_ms=float((log_mid-video_mid)*1000),
                interval_width_log_ms=float((t[i]-t[i-1])*1000),
                interval_width_video_ms=float((video_old_t[first]-video_old_t[first-1])*1000),
                video_led_off_old_alignment_s=float((video_old_t[last]+video_old_t[last+1])/2),
                log_angular_end_s=float(t[turning[-1]]),log_csv=str(csv),video_pose_csv=str(full/"apparent_floor_marker_pose.csv")))
    frame=pd.DataFrame(rows)
    frame.to_csv(OUTPUT/"led_onset_review.csv",index=False)
    frame.groupby("motion").onset_correction_to_mcu_ms.agg(["count","median","min","max"]).to_csv(OUTPUT/"summary.csv")
    provenance=dict(status="LED_ONSET_REVIEW_PROVISIONAL",count=len(frame),
        manifest_sha256=hashlib.sha256(manifest.read_bytes()).hexdigest(),
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        onset_correction_min_ms=float(frame.onset_correction_to_mcu_ms.min()),
        onset_correction_median_ms=float(frame.onset_correction_to_mcu_ms.median()),
        onset_correction_max_ms=float(frame.onset_correction_to_mcu_ms.max()),
        limitations=["LED GPIO timestamps are absent; MCU event is bracketed from approach-to-Lstart ego.length reset.",
            "Existing cached blue-pixel observations are used; exposure and rolling shutter are not corrected.",
            "The 80 ms search window selects the expected turn LED transition; it is not a hardware timestamp bound.",
            "Firmware source lights the complete Lstart/turn/Lend segment, but other indication writes can overwrite GPIO; off edge is not used for synchronization."])
    (OUTPUT/"report.json").write_text(json.dumps(provenance,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    print(json.dumps(provenance,ensure_ascii=True,indent=2))


if __name__=="__main__":
    review()
