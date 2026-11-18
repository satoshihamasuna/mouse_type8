"""機体A turn500の保存測定から、別速度の未採用パラメータを予測する。

実機へ接続しない。入力はturn_500.hの根拠JSONが指す各3本だけに固定する。
500での試行除外交差検証と、識別できない速度依存性のシナリオを併記する。
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path
import re

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.integrate import cumulative_trapezoid
from scipy.optimize import least_squares
from scipy.signal import lfilter


ROOT = Path(__file__).resolve().parents[2]
DT = .001
MOTIONS = ("long_r90", "long_l90", "long_r180", "long_l180",
           "in_r45", "in_l45", "out_r45", "out_l45", "in_r135", "in_l135",
           "out_r135", "out_l135", "r_v90", "l_v90")
FF_FIELDS = ("ff_sp_velo", "ff_sp_accel", "ff_sp_bias", "ff_om_velo", "ff_om_accel",
             "ff_om_decel", "ff_om_bias", "ff_om_jerk", "ff_sp_turn_accel_mag", "ff_sp_turn_velo_sq")


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def write_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_profile():
    text = (ROOT / "Params/turn_table.h").read_text(encoding="utf-8")
    match = re.search(r"accel_table\[1001\]\s*=\s*\{([^}]+)\}", text)
    table = np.fromstring(match[1], sep=",")
    integral = float(re.search(r"accel_Integral\s*=\s*([.\d]+)", text)[1])
    if len(table) != 1001 or not np.isfinite(table).all():
        raise ValueError("Invalid firmware turn table")
    return table, integral


TABLE, INTEGRAL = load_profile()


def distances(motion):
    """myshell.cpp shell_debug_turn_pre/post_runと同じ単体試験距離。"""
    diagonal = 90 / np.sqrt(2)
    if motion in ("long_r90", "long_l90"):
        return 180., 180.
    if motion in ("long_r180", "long_l180"):
        return 90., 90.
    if motion.startswith("in_"):
        return 90., 2 * diagonal
    if motion.startswith("out_"):
        return 2 * diagonal, 90.
    if motion in ("r_v90", "l_v90"):
        return 2 * diagonal, diagonal
    raise ValueError(motion)


def commands(settings, clock=1., delay_s=0.):
    p = settings["parameters"]
    v = p["velo"]
    pre, post = distances(settings["motion_name"])
    accel = max(p["pre_accel"], v * v / (2 * pre / 1000))
    pre_time = (pre + p["lstart"]) / (1000 * v) + v / (2 * accel)
    peak = -1000 * v / p["r_min"]  # 右回り正。ファームウェアとは逆。
    duration = abs(np.deg2rad(p["degree"] + p["correction"]) / (INTEGRAL * peak))
    post_accel, floor = 6.5, .15
    coast = (post + p["lend"]) / (1000 * v) - v / (2 * post_accel)
    if coast < 0 or v < floor:
        raise ValueError("Post distance or speed outside model range")
    end = pre_time + duration + coast + (v - floor) / post_accel + floor / (2 * post_accel)
    time = np.arange(0, end * clock + max(0., delay_s) + .501, DT)
    nominal = (time - delay_s) / clock
    vc = np.clip(accel * nominal, 0, v)
    post_mask = nominal >= pre_time + duration
    vc[post_mask] = np.clip(v - post_accel * np.maximum(0, nominal[post_mask] - pre_time - duration - coast), floor, v)
    vc[nominal >= end] = 0
    wc = np.zeros_like(time)
    turn = (nominal >= pre_time) & (nominal <= pre_time + duration)
    wc[turn] = peak * np.interp((nominal[turn] - pre_time) / duration * 1000, np.arange(1001), TABLE)
    return time, vc, wc, end * clock + delay_s


def first_order(command, tau):
    if tau <= 1e-7:
        return np.asarray(command).copy()
    decay = np.exp(-DT / tau)
    return lfilter([1 - decay], [1, -decay], command)


def predict(settings, model, exponent=1.):
    """設定と固定済み係数だけを使用。試験対象のCSV・映像を読まない。"""
    gv, gw, tau_beta_500, brake_tau, delay = model["spatial"]
    time, vc, wc, end = commands(settings, model["clock_ratio"], delay)
    # 停止の瞬間を1 ms格子へ丸めると、形状の微小変更で終点が不連続に
    # 飛ぶ。停止前応答を正確な停止時刻へ補間してから惰走を接続する。
    # 0.15 m/sはSetIdeal_straightの停止直前の指令下限。
    moving = first_order(np.where(time >= end, .15, vc), model["tau_v_s"])
    v = gv * moving
    after_stop = time >= end
    v[after_stop] = gv * np.interp(end, time, moving) * np.exp(-(time[after_stop] - end) / brake_tau)
    w = gw * first_order(wc, model["tau_w_s"])
    tau_beta = tau_beta_500 * (settings["parameters"]["velo"] / .5) ** exponent
    beta = -tau_beta * first_order(w, tau_beta)
    psi = cumulative_trapezoid(w, time, initial=0)
    right = 1000 * cumulative_trapezoid(v * np.sin(psi + beta), time, initial=0)
    forward = 1000 * cumulative_trapezoid(v * np.cos(psi + beta), time, initial=0)
    return pd.DataFrame(dict(time_s=time, right_mm=right, forward_mm=forward, heading_rad=psi,
                             velocity_m_s=v, omega_rad_s=w, beta_rad=beta,
                             command_velocity_m_s=vc, command_omega_rad_s=wc)), end


def load_runs(header):
    """候補コメントの根拠をたどり、各3本だけを選択。途中調整ランは混ぜない。"""
    runs, evidence = {}, {}
    paths = re.findall(r"tools/[^\r\n]+candidate.json", header.read_text(encoding="utf-8"))
    if len(paths) != 14:
        raise ValueError("Expected 14 evidence files in turn_500.h")
    for relative in paths:
        candidate_path = ROOT / relative
        candidate = read_json(candidate_path)
        motion = candidate["motion"]
        comparison_path = Path(candidate.get("comparison_json", candidate.get("source_comparison")))
        comparison = read_json(comparison_path)
        trials = candidate.get("trials", candidate.get("source_trials"))
        if trials is None:
            trials = [str(Path(p).with_name("trial.json")) for p in candidate["readback_settings_json"]]
        if len(trials) != 3 or motion in runs:
            raise ValueError(f"Expected exactly three unique trials: {motion}")
        evidence[motion] = dict(candidate_json=str(candidate_path), comparison_json=str(comparison_path),
                                candidate_sha256=digest(candidate_path), comparison_sha256=digest(comparison_path),
                                source_limitations=candidate.get("limitations", candidate.get("unresolved_validation")))
        loaded = []
        for i, trial_path in enumerate(trials):
            trial_path = Path(trial_path)
            trial = read_json(trial_path)
            settings_path = Path(trial["type8_settings_json"])
            csv_path = Path(trial["type8_log_csv"])
            settings = read_json(settings_path)
            analysis = trial_path.parent / "full"
            if candidate.get("analyses"):
                analysis = Path(candidate["analyses"][i]) / "full"
            report_path, summary_path = analysis / "report.json", analysis / "summary.json"
            report, summary = read_json(report_path), read_json(summary_path)
            if (settings.get("mouse") != "A" or settings["motion_name"] != motion
                    or settings["parameters"]["velo"] != .5 or settings["suction"]["enabled"]
                    or settings.get("turn_count") != 1 or report["frame_stride"] != 1
                    or not summary["auto_stop_confirmed"]
                    or not report["required_motion_window_quality"]["accepted_for_relative_review"]
                    or summary["motion_observation_rate"] < .95 or summary["trajectory_fit_rmse_mm"] > 8):
                raise ValueError(f"Input conditions / video quality failed: {trial_path}")
            if loaded:
                for group in ("parameters", "pid", "feedforward", "suction"):
                    if settings[group] != loaded[0]["settings"][group]:
                        raise ValueError(f"Mixed settings in {motion}: {group}")
            raw = pd.read_csv(csv_path)
            required = ["time_ms", "sequence", "ideal.velo", "ego.velo", "ideal.rad_velo", "ego.rad_velo"]
            if (not np.isfinite(raw[required].to_numpy(float)).all()
                    or not raw.time_ms.diff().iloc[1:].between(1, 10).all()
                    or not raw.sequence.diff().iloc[1:].eq(1).all()):
                raise ValueError(f"Invalid log time / sequence / numeric values: {csv_path}")
            start = settings["log"]["motion_start_time_ms"]
            end = (settings["log"]["motion_end_time_ms"] - start) / 1000
            raw["elapsed_s"] = (raw.time_ms - start) / 1000
            pose_path = analysis / "normalized_trajectory.csv"
            pose = pd.read_csv(pose_path)
            good = pose.detected.astype(bool) & pose.time_from_motion_start_s.between(0, end + .15)
            pose = pose.loc[good].copy()
            if (len(pose) < 30 or not np.isfinite(pose[["time_from_motion_start_s", "right_mm", "forward_mm"]]).all().all()
                    or not pose.time_from_motion_start_s.diff().iloc[1:].gt(0).all()):
                raise ValueError(f"Invalid video trajectory: {pose_path}")
            video = Path(trial["video_file"])
            if not video.is_file():
                raise ValueError(f"Missing source video: {video}")
            inputs = [trial_path, settings_path, csv_path, report_path, summary_path, pose_path]
            loaded.append(dict(id=trial_path.parent.name, trial_json=str(trial_path), settings=settings,
                               raw=raw, pose=pose, summary=summary, end_s=end,
                               input_sha256={str(p): digest(p) for p in inputs},
                               video=str(video), video_size=video.stat().st_size,
                               log_period_ms=settings["log"]["period_ms"],
                               battery_min_V=float(raw.Battery.min()), battery_max_V=float(raw.Battery.max())))
        runs[motion] = loaded
    if set(runs) != set(MOTIONS):
        raise ValueError("Motion set does not match all 14 turn types")
    return runs, evidence


def identify_tracking(runs):
    result = {}
    for state, c, y in (("v", "ideal.velo", "ego.velo"), ("w", "ideal.rad_velo", "ego.rad_velo")):
        packed = []
        for run in runs:
            time = np.arange(-.1, run["end_s"] + .15, DT)
            raw = run["raw"]
            packed.append((time, np.interp(time, raw.elapsed_s, raw[c]), np.interp(time, raw.elapsed_s, raw[y])))
        def residual(p):
            return np.concatenate([(p[0] * first_order(cmd, p[1]) - obs)[(t > .02) & (t < run["end_s"])]
                                   for run, (t, cmd, obs) in zip(runs, packed)])
        fit = least_squares(residual, [1, .005], bounds=([.85, .0002], [1.15, .05]), x_scale=[1, .01])
        result[f"tau_{state}_s"] = float(fit.x[1])
        result[f"telemetry_{state}_gain"] = float(fit.x[0])
        result[f"telemetry_{state}_rmse"] = float(np.sqrt(np.mean(residual(fit.x) ** 2)))
    periods = np.concatenate([r["raw"]["irq.period_us"].to_numpy() for r in runs])
    valid = np.isfinite(periods) & (periods > 500) & (periods < 2000)
    if not valid.any():
        raise ValueError("No measured IRQ period")
    result["clock_ratio"] = float(np.median(periods[valid]) / 1000)
    result["irq_period_excluded"] = int((~valid).sum())
    return result


def path_error(run, prediction, stride=1):
    pose = run["pose"].iloc[::stride]
    xy = np.column_stack([np.interp(pose.time_from_motion_start_s, prediction.time_s, prediction[c])
                          for c in ("right_mm", "forward_mm")])
    return xy - pose[["right_mm", "forward_mm"]].to_numpy()


def identify(runs):
    model = identify_tracking(runs)
    def residual(p):
        errors = []
        for run in runs:
            pred, _ = predict(run["settings"], dict(model, spatial=p))
            errors.extend(path_error(run, pred, 3).ravel())
            endpoint = pred[["right_mm", "forward_mm"]].iloc[-1].to_numpy()
            actual = [run["summary"]["endpoint_right_mm"], run["summary"]["endpoint_forward_mm"]]
            errors.extend(3 * (endpoint - actual))
        return np.array(errors)
    fit = least_squares(residual, [1, .985, .003, .035, .005],
                        bounds=([.85, .90, 0, .005, -.03], [1.15, 1.08, .05, .15, .05]),
                        x_scale=[1, 1, .01, .03, .01], loss="soft_l1", f_scale=2, max_nfev=160)
    normalized = fit.jac / np.maximum(np.linalg.norm(fit.jac, axis=0), 1e-12)
    model.update(spatial=fit.x.tolist(), success=bool(fit.success), active_bounds=fit.active_mask.tolist(),
                 normalized_jacobian_condition=float(np.linalg.cond(normalized)),
                 training_trials=[r["trial_json"] for r in runs])
    return model


def score(run, model):
    pred, end = predict(run["settings"], model)
    error = path_error(run, pred)
    endpoint = pred[["right_mm", "forward_mm"]].iloc[-1].to_numpy()
    actual = np.array([run["summary"]["endpoint_right_mm"], run["summary"]["endpoint_forward_mm"]])
    return dict(trial=run["trial_json"], path_rmse_mm=float(np.sqrt(np.mean(np.sum(error ** 2, axis=1)))),
                path_max_mm=float(np.max(np.linalg.norm(error, axis=1))),
                endpoint_error_mm=float(np.linalg.norm(endpoint - actual)),
                endpoint_right_error_mm=float(endpoint[0] - actual[0]),
                endpoint_forward_error_mm=float(endpoint[1] - actual[1]),
                planned_end_s=end, observed_end_s=run["end_s"])


def features(prediction, is180):
    values = prediction[["right_mm", "forward_mm"]].iloc[-1].tolist()
    if is180:
        values.append(float(prediction.forward_mm.max()))
    return np.array(values)


def transfer(settings, model, speed, exponent=1.):
    """500のモデル軌道を保持。180度は頂点も保持して3変数を分離する。"""
    changed = copy.deepcopy(settings)
    changed["parameters"]["velo"] = speed / 1000
    is180 = settings["motion_name"] in ("long_r180", "long_l180")
    base, _ = predict(settings, model)
    target = features(base, is180)
    p = changed["parameters"]
    keys = ["lstart", "lend"] if not is180 else ["r_min", "lstart", "lend"]
    sign = np.sign(p["r_min"])
    initial = [p[k] for k in keys]
    low, high = ([0, 0], [120, 120]) if not is180 else ([min(sign * 30, sign * 60), 0, 0], [max(sign * 30, sign * 60), 120, 120])
    def residual(x):
        p.update(zip(keys, x))
        pred, _ = predict(changed, model, exponent)
        return features(pred, is180) - target
    fit = least_squares(residual, initial, bounds=(low, high), diff_step=1e-3,
                        x_scale="jac", max_nfev=100, ftol=1e-9, xtol=1e-9, gtol=1e-9)
    # 実機の形状設定と同じ0.001 mm刻みで出力する。
    p.update({k: round(float(v), 3) for k, v in zip(keys, fit.x)})
    pred, end = predict(changed, model, exponent)
    changed.pop("log", None)
    changed["preset_speed_mm_s"] = speed
    changed["source"] = "offline_prediction_not_device_readback"
    if "captured_at" in changed:
        changed["reference_captured_at"] = changed.pop("captured_at")
    changed.pop("raw_response", None)
    changed.pop("command", None)
    return changed, pred, dict(solver_success=bool(fit.success), active_bounds=fit.active_mask.tolist(),
                              feature_residual_mm=(features(pred, is180) - target).tolist(), planned_end_s=end)


def write_header(path, speed, candidates):
    guard = f"MOUSE_A_PREDICTED_TURN_{speed}_H_"
    lines = [f"// 機体A / {speed} mm/s / モデル予測候補。実機未確認・未採用。",
             "// 生成元: tools/analysis/turn_speed_prediction.py。測定済みRAM PID/FFを保持。",
             "// Paramsやmode配列へは未接続。既存の300/500設定との混用を避ける。",
             f"#ifndef {guard}", f"#define {guard}", '#include "Params/typedef_run_param.h"', ""]
    for motion in MOTIONS:
        s = candidates[motion][speed]
        p, pid, ff = s["parameters"], s["pid"], s["feedforward"]
        name = f"predicted_{motion}_{speed}"
        fmt = lambda x: f"{float(x):.10g}" + ("f" if "." in f"{float(x):.10g}" or "e" in f"{float(x):.10g}" else ".0f")
        lines += [f"const static t_pid_gain {name}_sp = {{" + ",".join(fmt(pid[k]) for k in ("sp_kp", "sp_ki", "sp_kd")) + "};",
                  f"const static t_pid_gain {name}_om = {{" + ",".join(fmt(pid[k]) for k in ("om_kp", "om_ki", "om_kd")) + "};",
                  f"const static t_ff_gain {name}_ff = {{" + ",".join(fmt(ff.get(k, 0)) for k in FF_FIELDS) + "};",
                  f"const static t_turn_param_table {name}_table = {{" + ",".join(fmt(p[k]) for k in ("velo", "r_min", "lstart", "lend", "degree"))
                  + (",Turn_R," if p["degree"] < 0 else ",Turn_L,") + fmt(p["correction"]) + "};",
                  f"const static t_param {name} = {{&{name}_table,&{name}_sp,&{name}_om,&{name}_ff}};", ""]
    lines += [f"#endif // {guard}", ""]
    path.write_text("\n".join(lines), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--header", type=Path, default=ROOT / "Params/Param_A/turn_500/turn_500.h")
    parser.add_argument("--output", type=Path, required=True, help="未作成の保存先")
    args = parser.parse_args()
    if args.output.exists():
        parser.error("出力先が既に存在します。新しい保存先を指定してください")
    runs, evidence = load_runs(args.header)
    args.output.mkdir(parents=True)
    models, candidates, metrics, rows, sensitivity = {}, {}, [], [], []
    fig, axes = plt.subplots(4, 4, figsize=(16, 17), constrained_layout=True)
    valfig, valaxes = plt.subplots(4, 4, figsize=(16, 17), constrained_layout=True)
    for motion, ax, vax in zip(MOTIONS, axes.flat, valaxes.flat):
        selected = runs[motion]
        # 各試行を丸ごと除外。撮影が同一条件なので別速度の検証にはならない。
        cv = []
        for i, run in enumerate(selected):
            trained = identify([r for j, r in enumerate(selected) if i != j])
            metric = dict(motion=motion, split="leave_one_trial_out", **score(run, trained))
            cv.append(metric)
            metrics.append(metric)
        model = identify(selected)
        models[motion] = model
        settings = selected[0]["settings"]
        base, _ = predict(settings, model)
        candidates[motion] = {}
        for run in selected:
            metric = dict(motion=motion, split="refit_all_three", **score(run, model))
            metrics.append(metric)
            vax.plot(run["pose"].right_mm, run["pose"].forward_mm, color="#777777", alpha=.6, lw=1)
        vax.plot(base.right_mm, base.forward_mm, color="#111111", label="500 fit", lw=2)
        vax.set_title(f"{motion}\nLOO RMSE {np.mean([m['path_rmse_mm'] for m in cv]):.2f} mm")
        ax.plot(base.right_mm, base.forward_mm, color="black", label="500 fitted reference", lw=2)
        for speed, color in ((300, "#296ca8"), (400, "#14956d"), (600, "#cc7033")):
            candidate, pred, solver = transfer(settings, model, speed)
            candidates[motion][speed] = candidate
            folder = args.output / str(speed)
            folder.mkdir(exist_ok=True)
            pred.to_csv(folder / f"{motion}_trajectory.csv", index=False)
            write_json(folder / f"{motion}.settings.json", dict(status="PREDICTED_UNVERIFIED", settings=candidate,
                       model_motion=motion, model_file="../model.json", speed_law_exponent=1,
                       reference_speed_mm_s=500, **solver))
            ax.plot(pred.right_mm, pred.forward_mm, color=color, label=str(speed), lw=1.3)
            p = candidate["parameters"]
            row = dict(motion=motion, speed_mm_s=speed, radius_mm=p["r_min"], lstart_mm=p["lstart"], lend_mm=p["lend"],
                       logical_degree=p["degree"], correction_deg=p["correction"],
                       delta_lstart_mm=p["lstart"] - settings["parameters"]["lstart"],
                       delta_lend_mm=p["lend"] - settings["parameters"]["lend"],
                       predicted_endpoint_right_mm=float(pred.right_mm.iloc[-1]),
                       predicted_endpoint_forward_mm=float(pred.forward_mm.iloc[-1]),
                       peak_yaw_rad_s=float(abs(pred.command_omega_rad_s).max()),
                       peak_lateral_accel_m_s2=float(abs(pred.command_velocity_m_s * pred.command_omega_rad_s).max()),
                       transfer_residual_max_mm=float(max(abs(x) for x in solver["feature_residual_mm"])),
                       loo_path_rmse_mean_mm=float(np.mean([m["path_rmse_mm"] for m in cv])),
                       loo_endpoint_max_mm=float(max(m["endpoint_error_mm"] for m in cv)))
            alternative_endpoints = []
            # 固定した主候補を別の速度依存仮説で再予測。パラメータ再調整との差も残す。
            for exponent in (-1, 0, 1):
                alt, altpred, _ = transfer(settings, model, speed, exponent)
                fixed_pred, _ = predict(candidate, model, exponent)
                difference = fixed_pred[["right_mm", "forward_mm"]].iloc[-1].to_numpy() - features(base, False)
                alternative_endpoints.append(difference)
                sensitivity.append(dict(motion=motion, speed_mm_s=speed, tau_speed_exponent=exponent,
                    alternative_radius_mm=alt["parameters"]["r_min"], alternative_lstart_mm=alt["parameters"]["lstart"],
                    alternative_lend_mm=alt["parameters"]["lend"],
                    fixed_candidate_endpoint_shift_mm=float(np.linalg.norm(difference)),
                    fixed_candidate_right_shift_mm=float(difference[0]), fixed_candidate_forward_shift_mm=float(difference[1])))
            row["speed_law_endpoint_spread_max_mm"] = float(max(np.linalg.norm(v) for v in alternative_endpoints))
            rows.append(row)
        for a in (ax, vax):
            a.set_aspect("equal", adjustable="datalim")
            a.grid(alpha=.25)
            a.set_xlabel("right [mm]")
            a.set_ylabel("forward [mm]")
        ax.set_title(motion)
        print(f"{motion}: 3 inputs; LOO path RMSE {np.mean([m['path_rmse_mm'] for m in cv]):.2f} mm", flush=True)
    for a in list(axes.flat)[14:] + list(valaxes.flat)[14:]:
        a.set_visible(False)
    axes.flat[0].legend(fontsize=8)
    valaxes.flat[0].legend(fontsize=8)
    fig.suptitle("Mouse A: speed-transfer candidates (unverified)\nPreserve fitted 500 endpoints; 180 turns also preserve forward apex", fontsize=14)
    valfig.suptitle("500 mm/s: video marker midpoint vs model fitted to all 3 trials\nTitles show leave-one-trial-out errors, not new-speed validation", fontsize=14)
    fig.savefig(args.output / "predicted_paths.png", dpi=150)
    valfig.savefig(args.output / "validation_500.png", dpi=150)
    plt.close(fig)
    plt.close(valfig)
    pd.DataFrame(rows).to_csv(args.output / "parameters.csv", index=False)
    pd.DataFrame(metrics).to_csv(args.output / "validation.csv", index=False)
    pd.DataFrame(sensitivity).to_csv(args.output / "speed_law_sensitivity.csv", index=False)
    for speed in (300, 400, 600):
        write_header(args.output / f"turn_{speed}_candidate.h", speed, candidates)
    sources = [Path(__file__), args.header, ROOT / "Params/turn_table.h", ROOT / "Task/Src/motion.cpp",
               ROOT / "Task/Src/turn_motion.cpp", ROOT / "Task/Src/init_motion.cpp", ROOT / "System/Src/myshell.cpp"]
    write_json(args.output / "model.json", dict(status="PREDICTED_UNVERIFIED", mouse="A", reference_speed_mm_s=500,
        spatial_parameter_order=["combined_velocity_gain", "combined_yaw_gain", "tau_beta_500_s", "brake_tau_s", "phase_delay_s"],
        model=models, evidence=evidence, source_sha256={str(p): digest(p) for p in sources},
        inputs={m: [{k: r[k] for k in ("id", "trial_json", "settings", "input_sha256", "video", "video_size",
                                       "log_period_ms", "battery_min_V", "battery_max_V")} for r in rr] for m, rr in runs.items()},
        speed_law="tau_beta(v) = tau_beta_500 * (v / 0.5)**q; main q=1, sensitivity q=-1,0,1",
        transfer_objective="Preserve model-predicted 500 endpoint; 180 additionally preserves forward apex. Not an accuracy test.",
        limitations=["別速度の観測はなく、速度依存性・予測精度は未検証。シナリオ幅は信頼区間ではない。",
            "同定値は制御・撮影を含む有効値。横滑り・時刻同期・目印位置・ゲイン誤差を独立に識別していない。",
            "動画の初期直進方向と軌跡による時刻同期を利用。係数除外交差検証は盲検実験ではない。",
            "取得ELF同一性は未確認。IRQ実周期で公称指令を換算し、距離による状態切替は時間式で近似。",
            "相対目印中点の予測。絶対車軸位置・高さ補正済みyaw・壁あり連続走行は未検証。",
            "PID/FFは測定時RAM係数の継承。別速度で最適なFF/PIDを同定したものではない。",
            "既存の300プリセットと新候補は制御係数が異なる。400/600プリセットのファームウェア追加は未実施。",
            "in_L135の再現ばらつき、out_L135の出口角の余裕不足は元測定から引き継ぐ。"] ))
    (args.output / "analysis_source.py").write_bytes(Path(__file__).read_bytes())
    print(str(args.output), flush=True)


if __name__ == "__main__":
    main()
