"""Analyze translational speed FF and voltage response inside fast turns."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.optimize import least_squares, lsq_linear
from scipy.signal import savgol_filter


ROOT = Path(__file__).resolve().parents[2]
LOG_DIR = ROOT / "tools" / "logs"
PHASE = np.linspace(0.0, 100.0, 201)
PERIOD_S = 0.001
ACCEL_INTEGRAL = 0.7043
FF_LIMIT_V = 1.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--speeds", type=int, nargs="+", default=[1600, 1800, 2000])
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def fingerprint(settings: dict) -> str:
    selected = {
        "motion": settings.get("motion_name"),
        "parameters": settings.get("parameters"),
        "pid": settings.get("pid"),
        "feedforward": settings.get("feedforward"),
        "suction": settings.get("suction"),
    }
    return json.dumps(selected, sort_keys=True)


def latest_groups(speeds: list[int]) -> dict[tuple[int, str], list[tuple[Path, dict]]]:
    groups: dict[tuple[int, str, str], list[tuple[Path, dict]]] = {}
    for path in sorted(LOG_DIR.glob("*.settings.json")):
        settings = json.loads(path.read_text(encoding="utf-8"))
        params = settings.get("parameters", {})
        speed = int(round(float(params.get("velo", 0.0)) * 1000.0))
        motion = settings.get("motion_name", "")
        csv_path = path.with_name(path.name[:-len(".settings.json")] + ".csv")
        if speed not in speeds or not motion or not csv_path.exists():
            continue
        groups.setdefault((speed, motion, fingerprint(settings)), []).append((csv_path, settings))
    selected: dict[tuple[int, str], list[tuple[Path, dict]]] = {}
    for (speed, motion, _), group in groups.items():
        key = (speed, motion)
        if key not in selected or max(p.stat().st_mtime for p, _ in group) > max(
                p.stat().st_mtime for p, _ in selected[key]):
            selected[key] = group
    return selected


def regions(mask: np.ndarray) -> list[tuple[int, int]]:
    edges = np.diff(np.pad(mask.astype(np.int8), (1, 1)))
    return list(zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)))


def interp(values: np.ndarray) -> np.ndarray:
    return np.interp(PHASE, np.linspace(0.0, 100.0, len(values)), values)


def fit_fixed_effect(samples: list[dict], constrained: bool) -> dict:
    alpha = np.concatenate([s["alpha_mag"] for s in samples])
    omega2 = np.concatenate([s["omega2"] for s in samples])
    target = np.concatenate([s["required_v"] for s in samples])
    fixed = np.zeros((len(target), len(samples)))
    offset = 0
    for column, sample in enumerate(samples):
        fixed[offset:offset + len(PHASE), column] = 1.0
        offset += len(PHASE)
    design = np.column_stack((alpha, omega2, fixed))
    finite = np.all(np.isfinite(design), axis=1) & np.isfinite(target)
    mask = finite.copy()
    coefficient = np.zeros(design.shape[1])
    for _ in range(6):
        if constrained:
            lower = np.full(design.shape[1], -np.inf)
            upper = np.full(design.shape[1], np.inf)
            lower[0], upper[0] = -0.002, 0.0
            lower[1], upper[1] = 0.0, 0.005
            coefficient = lsq_linear(design[mask], target[mask], bounds=(lower, upper)).x
        else:
            coefficient, *_ = np.linalg.lstsq(design[mask], target[mask], rcond=None)
        residual = target - design @ coefficient
        center = np.median(residual[mask])
        scale = 1.4826 * np.median(np.abs(residual[mask] - center))
        if scale <= np.finfo(float).eps:
            break
        new_mask = finite & (np.abs(residual - center) <= 3.5 * scale)
        if np.array_equal(mask, new_mask):
            break
        mask = new_mask
    residual = target - design @ coefficient
    return {
        "k_alpha": float(coefficient[0]),
        "k_omega2": float(coefficient[1]),
        "rmse_v": float(np.sqrt(np.mean(np.square(residual[mask])))),
        "samples": int(np.count_nonzero(mask)),
        "rejected": int(len(mask) - np.count_nonzero(mask)),
    }


def voltage_response_fit(samples: list[dict]) -> dict:
    best = None
    for lag_ms in range(21):
        rows, target = [], []
        for sample in samples:
            lag = int(round(lag_ms / 1000.0 / sample["dt_s"]))
            if lag >= len(PHASE) - 10:
                continue
            source = slice(None, -lag) if lag else slice(None)
            response = slice(lag, None) if lag else slice(None)
            rows.append(np.column_stack((
                sample["motor_v"][source], sample["speed_delta"][source],
                sample["alpha_mag"][source], sample["omega2"][source],
                np.ones(len(sample["motor_v"][source])),
            )))
            target.append(sample["acceleration"][response])
        x, y = np.concatenate(rows), np.concatenate(target)
        finite = np.all(np.isfinite(x), axis=1) & np.isfinite(y)
        coefficient, *_ = np.linalg.lstsq(x[finite], y[finite], rcond=None)
        residual = y[finite] - x[finite] @ coefficient
        rmse = float(np.sqrt(np.mean(np.square(residual))))
        if best is None or rmse < best[0]:
            best = (rmse, lag_ms, coefficient, x[finite], y[finite])
    assert best is not None
    rmse, lag_ms, coefficient, design, target = best
    scale = np.std(design, axis=0)
    scale[scale <= np.finfo(float).eps] = 1.0
    denominator = np.sum(np.square(target - np.mean(target)))
    residual = target - design @ coefficient
    return {
        "lag_ms": int(lag_ms),
        "voltage_gain_mps2_per_v": float(coefficient[0]),
        "speed_term_per_s": float(coefficient[1]),
        "alpha_mag_term": float(coefficient[2]),
        "omega2_term": float(coefficient[3]),
        "rmse_mps2": rmse,
        "r_squared": float(1.0 - np.sum(np.square(residual)) / denominator),
        "condition_number": float(np.linalg.cond(design / scale)),
        "valid_positive_gain": bool(coefficient[0] > 0.05),
    }


def simulate_increment(voltage: np.ndarray, dt_s: float, a_speed: float,
                       b_voltage: float, lag_ms: int) -> np.ndarray:
    lag = int(round(lag_ms / 1000.0 / dt_s))
    response = np.zeros_like(voltage)
    for index in range(len(voltage) - 1):
        delayed = voltage[index - lag] if index >= lag else 0.0
        response[index + 1] = response[index] + dt_s * (
            a_speed * response[index] + b_voltage * delayed
        )
    return response


def response_recommendation(samples: list[dict], response: dict, ff_limit_v: float) -> dict:
    usable = (PHASE >= 3.0) & (PHASE <= 98.0)
    a_speed = response["speed_term_per_s"]
    b_voltage = response["voltage_gain_mps2_per_v"]

    def predicted(coefficients: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        residuals, voltages = [], []
        for sample in samples:
            voltage = np.clip(coefficients[0] * sample["alpha_mag"]
                              + coefficients[1] * sample["omega2"],
                              -ff_limit_v, ff_limit_v)
            correction = simulate_increment(voltage, sample["dt_s"], a_speed,
                                            b_voltage, response["lag_ms"])
            residuals.append((sample["speed_delta"] + correction)[usable])
            voltages.append(voltage)
        return np.concatenate(residuals), np.concatenate(voltages)

    fit = least_squares(
        lambda coefficient: predicted(coefficient)[0],
        x0=np.array([-0.00005, 0.00005]),
        bounds=(np.array([-0.002, 0.0]), np.array([0.0, 0.005])),
        loss="soft_l1", f_scale=0.01,
    )
    full = fit.x
    conservative = 0.5 * full
    before = np.concatenate([sample["speed_delta"][usable] for sample in samples])
    after_full, full_voltage = predicted(full)
    after_half, half_voltage = predicted(conservative)
    return {
        "model_based_full": {
            "k_alpha": float(full[0]), "k_omega2": float(full[1]),
            "predicted_ripple_rms_mm_s": float(np.sqrt(np.mean(after_full ** 2)) * 1000.0),
            "ff_min_v": float(np.min(full_voltage)), "ff_max_v": float(np.max(full_voltage)),
        },
        "conservative_half_step": {
            "k_alpha": float(conservative[0]), "k_omega2": float(conservative[1]),
            "predicted_ripple_rms_mm_s": float(np.sqrt(np.mean(after_half ** 2)) * 1000.0),
            "ff_min_v": float(np.min(half_voltage)), "ff_max_v": float(np.max(half_voltage)),
        },
        "measured_ripple_rms_mm_s": float(np.sqrt(np.mean(before ** 2)) * 1000.0),
        "ff_limit_v": ff_limit_v,
    }


def analyze_group(speed: int, motion: str, group: list[tuple[Path, dict]], output: Path) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    settings = group[-1][1]
    params, ff = settings["parameters"], settings["feedforward"]
    direction = 1.0 if float(params["degree"]) > 0.0 else -1.0
    omega_max = abs(float(params["velo"]) / (float(params["r_min"]) / 1000.0))
    # Thresholding at 1 rad/s removes a few samples from both profile tails.
    expected_ms = round(abs(math.radians(float(params["degree"])) /
                            (ACCEL_INTEGRAL * omega_max) * 1000.0)) - 4
    tolerance = max(4, math.ceil(expected_ms * 0.08))
    samples, rows = [], []
    for csv_path, _ in group:
        frame = pd.read_csv(csv_path).apply(pd.to_numeric, errors="coerce")
        signed_omega = frame["ideal.rad_velo"].to_numpy(float)
        normalized_omega = direction * signed_omega
        ideal_alpha = np.gradient(signed_omega, PERIOD_S)
        alpha_mag = np.sign(signed_omega) * ideal_alpha
        ideal_speed = frame["ideal.velo"].to_numpy(float)
        ego_speed = frame["ego.velo"].to_numpy(float)
        ideal_accel = frame["ideal.accel"].to_numpy(float)
        feedback = frame["sp_feedback"].to_numpy(float)
        logged_ff = frame["sp_feedforward"].to_numpy(float)
        battery = frame["Battery"].to_numpy(float)
        right = np.clip(frame["V_r"].to_numpy(float), -battery, battery)
        left = np.clip(frame["V_l"].to_numpy(float), -battery, battery)
        motor_common = (right - left) / 2.0
        saturated = ((np.abs(frame["V_r"].to_numpy(float)) >= battery) |
                     (np.abs(frame["V_l"].to_numpy(float)) >= battery))
        base_ff = (float(ff.get("ff_sp_bias", 0.0)) * np.sign(ideal_speed)
                   + float(ff.get("ff_sp_velo", 0.0)) * ideal_speed
                   + float(ff.get("ff_sp_accel", 0.0)) * ideal_accel)
        turn_ff = logged_ff - base_ff
        smooth_speed = savgol_filter(ego_speed, 11, 2, mode="interp")
        acceleration = np.gradient(smooth_speed, PERIOD_S)
        segments = [(a, b) for a, b in regions(normalized_omega > 1.0)
                    if abs((b - a) - expected_ms) <= tolerance]
        for turn_number, (start, end) in enumerate(segments, 1):
            pre = slice(max(0, start - 45), max(1, start - 8))
            fb_zero = float(np.median(feedback[pre]))
            motor_zero = float(np.median(motor_common[pre]))
            error_zero = float(np.median((ego_speed - ideal_speed)[pre]))
            sl = slice(start, end)
            sample = {
                "source": f"{csv_path.name}#{turn_number}",
                "dt_s": (end - start - 1) * PERIOD_S / (len(PHASE) - 1),
                "alpha_mag": interp(alpha_mag[sl]),
                "omega2": interp(np.square(signed_omega[sl])),
                "feedback": interp(feedback[sl] - fb_zero),
                "turn_ff": interp(turn_ff[sl]),
                "required_v": interp(turn_ff[sl] + feedback[sl] - fb_zero),
                "motor_v": interp(motor_common[sl] - motor_zero),
                "saturated": interp(saturated[sl].astype(float)),
                "speed_error": interp(ideal_speed[sl] - ego_speed[sl]),
                "speed_delta": interp((ego_speed[sl] - ideal_speed[sl]) - error_zero),
                "acceleration": interp(acceleration[sl]),
            }
            samples.append(sample)
            ripple = sample["speed_error"] - np.mean(sample["speed_error"])
            rows.append({
                "file": csv_path.name, "turn": turn_number,
                "speed_error_mean_mm_s": float(np.mean(sample["speed_error"]) * 1000.0),
                "speed_error_rms_mm_s": float(np.sqrt(np.mean(sample["speed_error"] ** 2)) * 1000.0),
                "speed_ripple_rms_mm_s": float(np.sqrt(np.mean(ripple ** 2)) * 1000.0),
                "feedback_rms_v": float(np.sqrt(np.mean(sample["feedback"] ** 2))),
                "required_voltage_rms_v": float(np.sqrt(np.mean(sample["required_v"] ** 2))),
                "motor_voltage_delta_rms_v": float(np.sqrt(np.mean(sample["motor_v"] ** 2))),
                "saturation_pct": float(100.0 * np.mean(sample["saturated"] >= 0.5)),
            })
    if not samples:
        raise ValueError(f"No matching turn sections: {speed} {motion}")
    metrics = pd.DataFrame(rows)
    metrics.to_csv(output / "turn_metrics.csv", index=False)
    unconstrained = fit_fixed_effect(samples, False)
    constrained = fit_fixed_effect(samples, True)
    response = voltage_response_fit(samples)
    summary = {
        "speed_mm_s": speed, "motion_name": motion,
        "source_files": [path.name for path, _ in group],
        "turn_count": len(samples), "settings": settings,
        "metrics_mean": {column: float(metrics[column].mean()) for column in metrics.columns
                         if column not in ("file", "turn")},
        "required_voltage_fit": unconstrained,
        "physical_sign_constrained_fit": constrained,
        "voltage_response_fit": response,
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    summary["_samples"] = samples

    phase = pd.DataFrame({"phase_pct": PHASE})
    for name in ("alpha_mag", "omega2", "feedback", "turn_ff", "required_v",
                 "motor_v", "speed_error", "speed_delta", "acceleration"):
        values = np.vstack([sample[name] for sample in samples])
        phase[f"{name}_mean"] = np.mean(values, axis=0)
        phase[f"{name}_std"] = np.std(values, axis=0)
    phase.to_csv(output / "phase_average.csv", index=False)

    fig, axes = plt.subplots(4, 1, figsize=(11, 12), sharex=True, constrained_layout=True)
    axes[0].plot(PHASE, phase["alpha_mag_mean"], label="sign(omega) * angular acceleration")
    twin = axes[0].twinx()
    twin.plot(PHASE, phase["omega2_mean"], color="tab:orange", label="omega^2")
    axes[0].set_ylabel("alpha_mag [rad/s^2]")
    twin.set_ylabel("omega^2 [rad^2/s^2]")
    axes[1].plot(PHASE, phase["required_v_mean"], label="required turn voltage (FF+PID)")
    axes[1].plot(PHASE, phase["feedback_mean"], label="speed PID delta", linestyle="--")
    axes[1].plot(PHASE, phase["motor_v_mean"], label="motor common voltage delta", linestyle=":")
    axes[1].set_ylabel("voltage [V]")
    axes[2].plot(PHASE, phase["speed_error_mean"] * 1000.0, label="target - ego")
    axes[2].fill_between(PHASE, (phase["speed_error_mean"] - phase["speed_error_std"]) * 1000.0,
                         (phase["speed_error_mean"] + phase["speed_error_std"]) * 1000.0, alpha=0.16)
    axes[2].set_ylabel("speed error [mm/s]")
    axes[3].plot(PHASE, phase["acceleration_mean"], label="measured longitudinal acceleration")
    axes[3].set_ylabel("acceleration [m/s^2]")
    axes[3].set_xlabel("turn phase [%]")
    for axis in axes:
        axis.axhline(0.0, color="0.45", linewidth=0.7)
        axis.grid(alpha=0.25)
        axis.legend(fontsize=8)
    fig.suptitle(f"{motion} @ {speed} mm/s: translational voltage and response")
    fig.savefig(output / "voltage_response.png", dpi=160)
    plt.close(fig)

    report = [
        f"# {motion} @ {speed} mm/s", "",
        f"- source logs: {len(group)}", f"- turns: {len(samples)}",
        f"- speed error RMS: {summary['metrics_mean']['speed_error_rms_mm_s']:.2f} mm/s",
        f"- speed ripple RMS: {summary['metrics_mean']['speed_ripple_rms_mm_s']:.2f} mm/s",
        f"- speed PID delta RMS: {summary['metrics_mean']['feedback_rms_v']:.4f} V",
        f"- constrained k_alpha: {constrained['k_alpha']:+.8f}",
        f"- constrained k_omega2: {constrained['k_omega2']:+.8f}",
        f"- voltage response gain: {response['voltage_gain_mps2_per_v']:+.3f} (m/s2)/V",
        f"- response fit R2: {response['r_squared']:.3f}",
        "", "The coefficient fit is diagnostic because these logs contain no independent translational-FF excitation.",
        "", "- [Voltage and response](voltage_response.png)",
        "- [Phase-average data](phase_average.csv)", "- [Turn metrics](turn_metrics.csv)",
    ]
    (output / "report.md").write_text("\n".join(report), encoding="utf-8")
    return summary


def main() -> int:
    args = parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    selected = latest_groups(args.speeds)
    summaries = []
    for (speed, motion), group in sorted(selected.items()):
        try:
            summary = analyze_group(speed, motion, group, args.output / str(speed) / motion)
        except ValueError as error:
            print(f"SKIP {speed} {motion}: {error}")
            continue
        summaries.append(summary)
        print(f"WROTE {speed} {motion}: {len(group)} logs, {summary['turn_count']} turns")
    rows = []
    for summary in summaries:
        row = {"speed_mm_s": summary["speed_mm_s"], "motion_name": summary["motion_name"],
               "turn_count": summary["turn_count"], **summary["metrics_mean"]}
        row.update({f"fit_{key}": value for key, value in summary["physical_sign_constrained_fit"].items()})
        row.update({f"response_{key}": value for key, value in summary["voltage_response_fit"].items()})
        rows.append(row)
    pd.DataFrame(rows).to_csv(args.output / "coefficient_summary.csv", index=False)
    speed_rows = []
    for speed in args.speeds:
        speed_samples = [sample for summary in summaries if summary["speed_mm_s"] == speed
                         for sample in summary["_samples"]]
        if not speed_samples:
            continue
        constrained = fit_fixed_effect(speed_samples, True)
        response = voltage_response_fit(speed_samples)
        recommendation = response_recommendation(speed_samples, response, FF_LIMIT_V)
        speed_rows.append({"speed_mm_s": speed, "turn_count": len(speed_samples),
                           **{f"fit_{key}": value for key, value in constrained.items()},
                           **{f"response_{key}": value for key, value in response.items()},
                           "measured_ripple_rms_mm_s": recommendation["measured_ripple_rms_mm_s"],
                           **{f"recommended_{key}": value for key, value in
                              recommendation["conservative_half_step"].items()}})
    pd.DataFrame(speed_rows).to_csv(args.output / "speed_summary.csv", index=False)
    serializable = [{key: value for key, value in summary.items() if key != "_samples"}
                    for summary in summaries]
    (args.output / "summary.json").write_text(json.dumps(serializable, indent=2), encoding="utf-8")
    print(f"WROTE {len(summaries)} condition reports to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
