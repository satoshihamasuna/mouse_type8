"""単純積分・吸引別滑りEKF・動画軌跡を同一座標/時刻で比較するHTMLを作る。"""
from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import pandas as pd

from turn_pose_ekf import PROFILE_FILE
from turn_slip_conditions import replay_conditioned


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def integrate_simple(times, velocity, gyro, bias=0., gain=1.):
    """同じ車輪速度・零点・実時刻で横滑り0のオドメトリを積分する。"""
    t, u, w = map(lambda x: np.asarray(x, float), (times, velocity, gyro))
    if not (len(t) == len(u) == len(w)) or len(t) < 2 or not np.isfinite([t,u,w]).all() or np.any(np.diff(t) <= 0):
        raise ValueError("Invalid simple integration inputs")
    dt = np.diff(t)
    angle = np.r_[0., np.cumsum(.5*(w[:-1]+w[1:]-2*bias)*gain*dt)]
    mid = .5*(angle[:-1]+angle[1:])
    distance = .5*(u[:-1]+u[1:])*dt
    return np.column_stack([np.r_[0., np.cumsum(distance*np.cos(mid))],
                            np.r_[0., np.cumsum(distance*np.sin(mid))]])*1000


def interpolate_valid(times, xy, query, maximum_gap=.020):
    """欠測を20msより長くまたがない線形補間。範囲外を外挿しない。"""
    t, points, q = np.asarray(times,float), np.asarray(xy,float), np.atleast_1d(query).astype(float)
    if len(t) < 2 or np.any(np.diff(t) <= 0):
        raise ValueError("Invalid interpolation timestamps")
    right = np.searchsorted(t,q,side="left")
    clipped = np.clip(right,0,len(t)-1)
    exact = abs(t[clipped]-q) < 1e-9
    hi = np.clip(right,1,len(t)-1)
    lo = hi-1
    accepted = exact | ((q >= t[0]) & (q <= t[-1]) & (t[hi]-t[lo] <= maximum_gap))
    output = np.column_stack([np.interp(q,t,points[:,i]) for i in range(points.shape[1])])
    output[~accepted] = np.nan
    return output


def video_reference(frame, first_time, turn_first, handedness):
    """動画の始点と入口方向だけで座標を決める。終点・推定経路へfitしない。"""
    valid = frame.detected.to_numpy(bool) & np.isfinite(frame[["center_x_mm","center_y_mm"]]).all(axis=1).to_numpy()
    t = frame.log_time_s.to_numpy(float)
    xy = frame[["center_x_mm","center_y_mm"]].to_numpy(float)
    # Same measured entry-direction interval used by the optical slip analysis.
    entry = valid & np.isfinite(frame.course_rad) & frame.time_s.between(max(.02,turn_first-.15),turn_first-.025).to_numpy()
    if entry.sum() < 5:
        raise ValueError("Insufficient common entry direction")
    heading = math.atan2(np.sin(frame.loc[entry,"course_rad"]).mean(),np.cos(frame.loc[entry,"course_rad"]).mean())
    basis = np.array([[math.cos(heading),-handedness*math.sin(heading)],
                      [math.sin(heading),handedness*math.cos(heading)]])
    origin = interpolate_valid(t[valid],xy[valid],first_time)[0]
    if not np.isfinite(origin).all():
        raise ValueError("Video origin is missing")
    local = (xy-origin) @ basis
    local[~valid] = np.nan
    return local,valid,dict(origin_source_xy_mm=origin.tolist(),entry_angle_source_rad=heading,
                            method="video_origin_and_measured_straight_entry_only",scale_fitted=False,
                            separate_estimator_alignment=False,entry_observations=int(entry.sum()))


def error_metrics(video_time, video_xy, prediction_time, prediction_xy, turn_interval):
    predicted = interpolate_valid(prediction_time,prediction_xy,video_time)
    valid = np.isfinite(video_xy).all(axis=1) & np.isfinite(predicted).all(axis=1)
    if valid.sum() < 10:
        raise ValueError("Insufficient common trajectory samples")
    error = np.linalg.norm(predicted-video_xy,axis=1)
    turn = valid & (video_time >= turn_interval[0]) & (video_time <= turn_interval[1])
    if turn.sum() < 5:
        raise ValueError("Insufficient common turn samples")
    return dict(rmse_mm=float(np.sqrt(np.mean(error[valid]**2))),
                turn_rmse_mm=float(np.sqrt(np.mean(error[turn]**2))),
                maximum_mm=float(np.max(error[valid])),endpoint_mm=float(error[np.flatnonzero(valid)[-1]]),
                observations=int(valid.sum()),turn_observations=int(turn.sum())),error


def prepare_case(source, review, profiles, source_report):
    paths = [Path(p) for p in source["sources_sha256"]]
    for path in paths:
        if digest(path) != source["sources_sha256"][str(path)]:
            raise ValueError("Input changed: "+str(path))
    find = lambda ending: next(p for p in paths if p.name == ending)
    csv, settings_path, old_report_path = find("motion.csv"),find("motion.settings.json"),find("report.json")
    settings = json.loads(settings_path.read_text(encoding="utf-8"))
    old_report = json.loads(old_report_path.read_text(encoding="utf-8"))
    trial_path = csv.parent/"trial.json"
    trial = json.loads(trial_path.read_text(encoding="utf-8"))
    label = source["trial"]
    observations_path = review/label/"observations.csv"
    diagnostics_path = review/label/"diagnostics.json"
    observation = pd.read_csv(observations_path)
    diagnostics = json.loads(diagnostics_path.read_text(encoding="utf-8"))
    raw = pd.read_csv(csv)
    suction = settings["suction"]["enabled"]
    states, estimator_info = replay_conditioned(raw,suction,profiles)
    start = settings["log"]["motion_start_time_ms"]
    log_time = (states.time_ms.to_numpy()-start)/1000
    time = log_time-log_time[0]
    omega = np.interp(states.time_ms,raw.time_ms,raw["ego.rad_velo"])
    simple = integrate_simple(time,states.velocity_m_s,omega,estimator_info["gyro_bias_rad_s"],estimator_info["parameters"]["gyro_gain"])
    modeled = states[["x_mm","y_mm"]].to_numpy(float)
    local_video,valid,reference = video_reference(observation,log_time[0],diagnostics["turn_first_s"],
                                                 diagnostics["frame_handedness_vs_left_positive_gyro"])
    video_time = observation.log_time_s.to_numpy()-log_time[0]
    turn_interval = [diagnostics["turn_first_s"]-log_time[0],diagnostics["turn_last_s"]-log_time[0]]
    keep = valid & (video_time >= 0) & (video_time <= time[-1])
    both = []
    for prediction in (simple,modeled):
        metrics,error = error_metrics(video_time[keep],local_video[keep],time,prediction,turn_interval)
        both.append((metrics,error))
    # User-facing axes: right, forward. Internal estimator axes: forward, left.
    display = lambda xy: np.column_stack([-xy[:,1],xy[:,0]])
    shown_simple,shown_model,shown_video = map(display,(simple,modeled,local_video))
    # Preserve missing rows in the browser so it never draws across absent video.
    chosen = (video_time >= -.04) & (video_time <= time[-1]+.04)
    video_rows=[]
    for i in np.flatnonzero(chosen):
        video_rows.append([round(float(video_time[i]),6),
                           round(float(shown_video[i,0]),4) if valid[i] else None,
                           round(float(shown_video[i,1]),4) if valid[i] else None])
    trajectory = np.column_stack([time,shown_simple,shown_model,states.yaw_deg,states.beta_deg])
    # Explicit side-by-side metrics use the same observed frames and transform.
    comparison = pd.DataFrame(dict(time_s=video_time[keep],video_right_mm=shown_video[keep,0],
                                   video_forward_mm=shown_video[keep,1],simple_error_mm=both[0][1],model_error_mm=both[1][1]))
    video_file = Path(trial["video_file"])
    # t_log = video_pts - video_motion_start + measured_heading_time_shift.
    video_zero = old_report["required_motion_window_quality"]["video_motion_start_s"]-diagnostics["time_shift_ms"]/1000+log_time[0]
    return dict(id=label,motion=source["motion"],speed=round(source["speed_m_s"]*1000),suction=suction,
                duty=settings["suction"]["duty"],parameters=settings["parameters"],k=estimator_info["parameters"]["k_m_s2"],
                duration=float(time[-1]),turn=turn_interval,video_zero_s=video_zero,video_uri=video_file.as_uri(),
                video_file=str(video_file),video_available=video_file.exists(),reference=reference,
                metrics=dict(simple=both[0][0],model=both[1][0]),
                trajectory=np.round(trajectory,6).tolist(),video=video_rows,
                errors=np.round(comparison[["time_s","simple_error_mm","model_error_mm"]].to_numpy(),6).tolist(),
                calibration_includes_trial=True,sync_method="shared optical-heading time alignment; no separate trajectory fit",
                inputs_sha256={str(p):digest(p) for p in paths+[observations_path,diagnostics_path,trial_path]},
                source_report=str(source_report)),comparison


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--review",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():
        parser.error("出力先には新しいディレクトリを指定してください")
    source_report=args.review/"report.json"
    source=json.loads(source_report.read_text(encoding="utf-8"))
    profiles=json.loads(PROFILE_FILE.read_text(encoding="utf-8"))
    args.output.mkdir(parents=True)
    cases=[]
    summary=[]
    for item in source["sources"]:
        case,comparison=prepare_case(item,args.review,profiles,source_report)
        cases.append(case)
        comparison.to_csv(args.output/(case["id"]+"_errors.csv"),index=False)
        summary.append(dict(trial=case["id"],speed_mm_s=case["speed"],suction=case["suction"],motion=case["motion"],
                            **{"simple_"+k:v for k,v in case["metrics"]["simple"].items()},
                            **{"model_"+k:v for k,v in case["metrics"]["model"].items()}))
    payload=dict(cases=cases,generated_date="2026-11-18",mouse="A")
    template=Path(__file__).with_suffix(".html")
    html=template.read_text(encoding="utf-8").replace("__PAYLOAD__",json.dumps(payload,ensure_ascii=False,separators=(",",":"),allow_nan=False).replace("</","<\\/"))
    (args.output/"compare.html").write_text(html,encoding="utf-8")
    pd.DataFrame(summary).to_csv(args.output/"summary.csv",index=False)
    manifest=deepcopy(payload)
    for case in manifest["cases"]:
        for key in ("trajectory","video","errors"):case.pop(key)
    manifest["source_sha256"]={str(p.resolve()):digest(p) for p in
                               (Path(__file__),template,PROFILE_FILE,source_report,
                                Path(__file__).with_name("turn_slip_conditions.py"),Path(__file__).with_name("turn_pose_ekf.py"))}
    (args.output/"manifest.json").write_text(json.dumps(manifest,ensure_ascii=False,indent=2,allow_nan=False)+"\n",encoding="utf-8")
    print(pd.DataFrame(summary).groupby(["speed_mm_s","motion"])[["simple_rmse_mm","model_rmse_mm"]].median().to_string())


if __name__=="__main__":
    main()
