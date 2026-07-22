"""Analyze search-turn translational voltage and response by speed/direction.

The report intentionally keeps two estimates separate:

1. required-voltage fit: configured turn FF + residual speed feedback;
2. response fit: measured common-mode motor voltage -> measured acceleration.

The first is useful for feedforward learning, while the second checks that the
voltage has the expected physical effect instead of blindly copying PID output.
"""

from __future__ import annotations

import argparse
import json
import re
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.optimize import least_squares, lsq_linear
from scipy.signal import savgol_filter


NAME_RE = re.compile(r"search_turn_(right|left)_(280|300|320|350|370|400)")
PHASE = np.linspace(0.0, 100.0, 201)
SP_TURN_FF_LIMIT_V = 0.35


@dataclass
class Turn:
    source: str
    cohort: str
    direction: str
    speed: int
    dt_s: float
    phase: np.ndarray
    omega: np.ndarray
    alpha_mag: np.ndarray
    omega_sq: np.ndarray
    sp_feedback: np.ndarray
    turn_ff: np.ndarray
    required_turn_voltage: np.ndarray
    motor_voltage_delta: np.ndarray
    speed_error: np.ndarray
    ego_speed_delta: np.ndarray
    acceleration: np.ndarray


def regions(mask: np.ndarray) -> list[tuple[int, int]]:
    edges = np.diff(np.pad(mask.astype(np.int8), (1, 1)))
    return list(zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)))


def interp_phase(values: np.ndarray) -> np.ndarray:
    old = np.linspace(0.0, 100.0, len(values))
    return np.interp(PHASE, old, values)


def median_window(values: np.ndarray, start: int, end: int) -> float:
    selected = values[max(0, start):max(0, end)]
    return float(np.median(selected)) if len(selected) else float(values[max(0, end)])


def load_file(path: Path, period_ms: float) -> list[Turn]:
    match = NAME_RE.search(path.stem)
    settings_path = path.with_suffix(".settings.json")
    if not match or not settings_path.exists():
        return []
    direction, speed_text = match.groups()
    settings = json.loads(settings_path.read_text(encoding="utf-8"))
    ff = settings.get("feedforward", {})
    if "ff_sp_turn_accel_mag" in ff:
        cohort = "current"
    elif "ff_sp_turn_accel_abs" in ff:
        cohort = "old_trial"
    else:
        cohort = "baseline"

    frame = pd.read_csv(path).apply(pd.to_numeric, errors="coerce")
    omega = frame["ideal.rad_velo"].to_numpy(float)
    alpha = np.zeros_like(omega)
    alpha[:-1] = np.diff(omega) / (period_ms / 1000.0)
    alpha_mag = np.sign(omega) * alpha
    ideal_speed = frame["ideal.velo"].to_numpy(float)
    ideal_accel = frame["ideal.accel"].to_numpy(float)
    ego_speed = frame["ego.velo"].to_numpy(float)
    sp_feedback = frame["sp_feedback"].to_numpy(float)
    sp_feedforward = frame["sp_feedforward"].to_numpy(float)
    motor_voltage = (
        frame["V_r"].to_numpy(float) - frame["V_l"].to_numpy(float)
    ) / 2.0

    base_ff = (
        float(ff.get("ff_sp_bias", 0.0)) * np.sign(ideal_speed)
        + float(ff.get("ff_sp_velo", 0.0)) * ideal_speed
        + float(ff.get("ff_sp_accel", 0.0)) * ideal_accel
    )
    turn_ff = sp_feedforward - base_ff
    active = np.abs(omega) >= 0.05
    output: list[Turn] = []
    for turn_index, (start, end) in enumerate(regions(active)):
        if end - start < 20:
            continue
        pre_start = max(0, start - 45)
        pre_end = max(0, start - 8)
        feedback_zero = median_window(sp_feedback, pre_start, pre_end)
        motor_zero = median_window(motor_voltage, pre_start, pre_end)
        error_zero = median_window(ideal_speed - ego_speed, pre_start, pre_end)

        velocity_smooth = savgol_filter(ego_speed, 11, 2, mode="interp")
        acceleration = np.gradient(velocity_smooth, period_ms / 1000.0)
        sl = slice(start, end)
        output.append(Turn(
            source=f"{path.name}#{turn_index + 1}",
            cohort=cohort,
            direction=direction,
            speed=int(speed_text),
            dt_s=(end - start - 1) * period_ms / 1000.0 / (len(PHASE) - 1),
            phase=PHASE,
            omega=interp_phase(omega[sl]),
            alpha_mag=interp_phase(alpha_mag[sl]),
            omega_sq=interp_phase(np.square(omega[sl])),
            sp_feedback=interp_phase(sp_feedback[sl] - feedback_zero),
            turn_ff=interp_phase(turn_ff[sl]),
            required_turn_voltage=interp_phase(turn_ff[sl] + sp_feedback[sl] - feedback_zero),
            motor_voltage_delta=interp_phase(motor_voltage[sl] - motor_zero),
            speed_error=interp_phase(ideal_speed[sl] - ego_speed[sl]),
            ego_speed_delta=interp_phase((ego_speed[sl] - ideal_speed[sl]) + error_zero),
            acceleration=interp_phase(acceleration[sl]),
        ))
    return output


def robust_fixed_effect_fit(turns: list[Turn], constrained: bool = False) -> dict[str, float]:
    alpha = np.concatenate([turn.alpha_mag for turn in turns])
    omega_sq = np.concatenate([turn.omega_sq for turn in turns])
    target = np.concatenate([turn.required_turn_voltage for turn in turns])
    fixed = np.zeros((len(target), len(turns)))
    offset = 0
    for column, turn in enumerate(turns):
        fixed[offset:offset + len(turn.phase), column] = 1.0
        offset += len(turn.phase)
    design = np.column_stack((alpha, omega_sq, fixed))
    mask = np.all(np.isfinite(design), axis=1) & np.isfinite(target)
    coefficient = np.zeros(design.shape[1])
    for _ in range(6):
        if constrained:
            lower = np.full(design.shape[1], -np.inf)
            upper = np.full(design.shape[1], np.inf)
            upper[0] = 0.0
            lower[1] = 0.0
            coefficient = lsq_linear(design[mask], target[mask], bounds=(lower, upper)).x
        else:
            coefficient, *_ = np.linalg.lstsq(design[mask], target[mask], rcond=None)
        residual = target - design @ coefficient
        selected = residual[mask]
        median = np.median(selected)
        scale = 1.4826 * np.median(np.abs(selected - median))
        if scale <= np.finfo(float).eps:
            break
        new_mask = np.all(np.isfinite(design), axis=1) & np.isfinite(target)
        new_mask &= np.abs(residual - median) <= 3.5 * scale
        if np.array_equal(mask, new_mask):
            break
        mask = new_mask
    residual = target - design @ coefficient
    rmse = float(np.sqrt(np.mean(np.square(residual[mask]))))
    return {
        "k_alpha": float(coefficient[0]),
        "k_omega2": float(coefficient[1]),
        "rmse_v": rmse,
        "samples": int(np.count_nonzero(mask)),
        "rejected": int(len(mask) - np.count_nonzero(mask)),
    }


def response_fit(turns: list[Turn]) -> dict[str, float | int | bool | None]:
    best = None
    for lag in range(0, 16):
        rows = []
        target = []
        for turn in turns:
            if lag:
                source = slice(0, -lag)
                response = slice(lag, None)
            else:
                source = response = slice(None)
            rows.append(np.column_stack((
                turn.motor_voltage_delta[source],
                turn.alpha_mag[source],
                turn.omega_sq[source],
                turn.ego_speed_delta[response],
                np.ones(len(turn.phase[source])),
            )))
            target.append(turn.acceleration[response])
        x = np.concatenate(rows)
        y = np.concatenate(target)
        finite = np.all(np.isfinite(x), axis=1) & np.isfinite(y)
        coefficient, *_ = np.linalg.lstsq(x[finite], y[finite], rcond=None)
        residual = y[finite] - x[finite] @ coefficient
        rmse = float(np.sqrt(np.mean(np.square(residual))))
        if best is None or rmse < best[0]:
            best = (rmse, lag, coefficient, x[finite])
    assert best is not None
    rmse, lag, coefficient, design = best
    voltage_gain = float(coefficient[0])
    scale = np.std(design, axis=0)
    scale[scale <= np.finfo(float).eps] = 1.0
    condition = float(np.linalg.cond(design / scale))
    valid = voltage_gain > 0.05 and condition < 1e7
    return {
        "lag_phase_samples": int(lag),
        "voltage_gain_mps2_per_v": voltage_gain,
        "k_alpha_from_response": float(-coefficient[1] / voltage_gain) if valid else None,
        "k_omega2_from_response": float(-coefficient[2] / voltage_gain) if valid else None,
        "rmse_mps2": rmse,
        "condition_number": condition,
        "valid": bool(valid),
    }


def robust_linear(design: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, float, np.ndarray]:
    finite = np.all(np.isfinite(design), axis=1) & np.isfinite(target)
    mask = finite.copy()
    coefficient = np.zeros(design.shape[1])
    for _ in range(6):
        coefficient, *_ = np.linalg.lstsq(design[mask], target[mask], rcond=None)
        residual = target - design @ coefficient
        selected = residual[mask]
        median = np.median(selected)
        scale = 1.4826 * np.median(np.abs(selected - median))
        if scale <= np.finfo(float).eps:
            break
        new_mask = finite & (np.abs(residual - median) <= 3.5 * scale)
        if np.array_equal(mask, new_mask):
            break
        mask = new_mask
    residual = target - design @ coefficient
    rmse = float(np.sqrt(np.mean(np.square(residual[mask]))))
    return coefficient, rmse, mask


def simulate_response(voltage: np.ndarray, dt_s: float, a_closed_loop: float,
                      b_voltage: float, lag_ms: int, initial: float = 0.0) -> np.ndarray:
    lag = int(round(lag_ms / 1000.0 / dt_s))
    response = np.zeros_like(voltage)
    response[0] = initial
    for index in range(len(voltage) - 1):
        input_index = index - lag
        delayed_voltage = voltage[input_index] if input_index >= 0 else 0.0
        response[index + 1] = response[index] + dt_s * (
            a_closed_loop * response[index] + b_voltage * delayed_voltage
        )
    return response


def identify_closed_loop_model(turns: list[Turn], experimental_cohorts=("old_trial",)) -> dict:
    paired = []
    for speed in (280, 300, 320, 350, 370, 400):
        for direction in ("right", "left"):
            baseline = [turn for turn in turns if turn.speed == speed and turn.direction == direction
                        and turn.cohort == "baseline"]
            if not baseline:
                continue
            base_v, _ = mean_std(baseline, "ego_speed_delta")
            base_ff, _ = mean_std(baseline, "turn_ff")
            for cohort in experimental_cohorts:
                trial = [turn for turn in turns if turn.speed == speed
                         and turn.direction == direction and turn.cohort == cohort]
                if not trial:
                    continue
                trial_v, _ = mean_std(trial, "ego_speed_delta")
                trial_ff, _ = mean_std(trial, "turn_ff")
                paired.append({
                    "name": f"{cohort}_{direction}_{speed}",
                    "dt_s": float(np.mean([turn.dt_s for turn in baseline + trial])),
                    "delta_v": trial_v - base_v,
                    "delta_ff": trial_ff - base_ff,
                })
    if not paired:
        raise ValueError("No baseline/experiment pairs are available for voltage-response identification")

    best = None
    for lag_ms in range(0, 21):
        def residual(parameters: np.ndarray) -> np.ndarray:
            a_closed_loop, b_voltage = parameters
            values = []
            for pair in paired:
                measured = savgol_filter(pair["delta_v"], 11, 2, mode="interp")
                predicted = simulate_response(
                    pair["delta_ff"], pair["dt_s"], a_closed_loop, b_voltage,
                    lag_ms, initial=float(measured[0]),
                )
                values.append(predicted[3:-3] - measured[3:-3])
            return np.concatenate(values)

        fit = least_squares(
            residual, x0=np.array([-8.0, 8.0]),
            bounds=(np.array([-100.0, 0.01]), np.array([-0.01, 100.0])),
            loss="soft_l1", f_scale=0.003,
        )
        raw_residual = residual(fit.x)
        rmse = float(np.sqrt(np.mean(np.square(raw_residual))))
        if best is None or rmse < best[0]:
            best = (rmse, lag_ms, fit.x, raw_residual)
    if best is None:
        raise ValueError("The added turn voltage did not identify a positive voltage-to-speed response")
    rmse, lag_ms, coefficient, residual = best
    measured_all = np.concatenate([
        savgol_filter(pair["delta_v"], 11, 2, mode="interp")[3:-3] for pair in paired
    ])
    denominator = np.sum(np.square(measured_all - np.mean(measured_all)))
    r_squared = 1.0 - np.sum(np.square(residual)) / denominator
    return {
        "a_closed_loop_per_s": float(coefficient[0]),
        "b_voltage_mps2_per_v": float(coefficient[1]),
        "lag_ms": int(lag_ms),
        "rmse_speed_mps": float(rmse),
        "r_squared": float(r_squared),
        "paired_groups": [pair["name"] for pair in paired],
        "pairs": paired,
    }


def simulate_basis(source: np.ndarray, dt_s: float, a_closed_loop: float,
                   b_voltage: float, lag_ms: int) -> np.ndarray:
    return simulate_response(source, dt_s, a_closed_loop, b_voltage, lag_ms)


def response_based_recommendation(turns: list[Turn], model: dict) -> dict:
    baseline = [turn for turn in turns if turn.cohort == "baseline"]
    v0, _ = mean_std(baseline, "ego_speed_delta")
    alpha, _ = mean_std(baseline, "alpha_mag")
    omega_sq, _ = mean_std(baseline, "omega_sq")
    dt_s = float(np.mean([turn.dt_s for turn in baseline]))
    alpha_response = simulate_basis(alpha, dt_s, model["a_closed_loop_per_s"],
                                    model["b_voltage_mps2_per_v"], model["lag_ms"])
    omega_response = simulate_basis(omega_sq, dt_s, model["a_closed_loop_per_s"],
                                    model["b_voltage_mps2_per_v"], model["lag_ms"])
    design = np.column_stack((alpha_response, omega_response))
    usable = (PHASE >= 3.0) & (PHASE <= 98.0)
    unconstrained, *_ = np.linalg.lstsq(design[usable], -v0[usable], rcond=None)
    constrained_linear = lsq_linear(
        design[usable], -v0[usable], bounds=([-0.002, 0.0], [0.0, 0.005])
    ).x
    predicted_unconstrained = v0 + design @ unconstrained

    def predict_with_limit(coefficients: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        voltage = np.clip(coefficients[0] * alpha + coefficients[1] * omega_sq,
                          -SP_TURN_FF_LIMIT_V, SP_TURN_FF_LIMIT_V)
        response = simulate_response(voltage, dt_s, model["a_closed_loop_per_s"],
                                     model["b_voltage_mps2_per_v"], model["lag_ms"])
        return v0 + response, voltage

    constrained_fit = least_squares(
        lambda coefficients: predict_with_limit(coefficients)[0][usable],
        x0=constrained_linear,
        bounds=(np.array([-0.002, 0.0]), np.array([0.0, 0.005])),
        loss="soft_l1", f_scale=0.003,
    )
    constrained = constrained_fit.x
    predicted_constrained, constrained_voltage = predict_with_limit(constrained)
    conservative = 0.5 * constrained
    predicted_conservative, conservative_voltage = predict_with_limit(conservative)
    rms_before = float(np.sqrt(np.mean(np.square(v0[usable]))) * 1000.0)
    rms_unconstrained = float(np.sqrt(np.mean(np.square(predicted_unconstrained[usable]))) * 1000.0)
    rms_constrained = float(np.sqrt(np.mean(np.square(predicted_constrained[usable]))) * 1000.0)
    rms_conservative = float(np.sqrt(np.mean(np.square(predicted_conservative[usable]))) * 1000.0)
    return {
        "unconstrained": {
            "k_alpha": float(unconstrained[0]), "k_omega2": float(unconstrained[1]),
            "predicted_speed_rms_mm_s": rms_unconstrained,
        },
        "physical_sign_constrained": {
            "k_alpha": float(constrained[0]), "k_omega2": float(constrained[1]),
            "predicted_speed_rms_mm_s": rms_constrained,
            "ff_min_v": float(np.min(constrained_voltage)),
            "ff_max_v": float(np.max(constrained_voltage)),
        },
        "recommended_conservative_half_step": {
            "k_alpha": float(conservative[0]), "k_omega2": float(conservative[1]),
            "predicted_speed_rms_mm_s": rms_conservative,
            "ff_min_v": float(np.min(conservative_voltage)),
            "ff_max_v": float(np.max(conservative_voltage)),
            "ff_limit_v": SP_TURN_FF_LIMIT_V,
        },
        "baseline_speed_rms_mm_s": rms_before,
        "predicted_speed_error_unconstrained": predicted_unconstrained,
        "predicted_speed_error_constrained": predicted_constrained,
        "predicted_speed_error_conservative": predicted_conservative,
    }


def metric_summary(turns: list[Turn]) -> dict[str, float | int]:
    error = np.concatenate([turn.speed_error for turn in turns])
    ripple = np.concatenate([turn.speed_error - np.mean(turn.speed_error) for turn in turns])
    feedback = np.concatenate([turn.sp_feedback for turn in turns])
    motor = np.concatenate([turn.motor_voltage_delta for turn in turns])
    fluctuation = np.concatenate([turn.ego_speed_delta for turn in turns])
    return {
        "turns": len(turns),
        "speed_error_mean_mm_s": float(np.mean(error) * 1000.0),
        "speed_error_rms_mm_s": float(np.sqrt(np.mean(np.square(error))) * 1000.0),
        "turn_fluctuation_rms_mm_s": float(np.sqrt(np.mean(np.square(fluctuation))) * 1000.0),
        "speed_ripple_rms_mm_s": float(np.sqrt(np.mean(np.square(ripple))) * 1000.0),
        "speed_error_peak_positive_mm_s": float(np.max(error) * 1000.0),
        "speed_error_peak_negative_mm_s": float(np.min(error) * 1000.0),
        "feedback_rms_v": float(np.sqrt(np.mean(np.square(feedback)))),
        "motor_voltage_delta_rms_v": float(np.sqrt(np.mean(np.square(motor)))),
    }


def mean_std(turns: list[Turn], field: str) -> tuple[np.ndarray, np.ndarray]:
    values = np.vstack([getattr(turn, field) for turn in turns])
    return np.mean(values, axis=0), np.std(values, axis=0)


def plot_group(group_dir: Path, title: str, cohorts: dict[str, list[Turn]],
               fits: dict[str, dict[str, dict[str, float]]], recommendation: dict | None) -> None:
    fig, axes = plt.subplots(4, 1, figsize=(11, 12), sharex=True, constrained_layout=True)
    colors = {"baseline": "tab:blue", "old_trial": "tab:orange", "current": "tab:green"}
    for cohort, turns in cohorts.items():
        color = colors[cohort]
        omega, _ = mean_std(turns, "omega")
        alpha, _ = mean_std(turns, "alpha_mag")
        axes[0].plot(PHASE, omega, color=color, label=f"{cohort} omega")
        axes[0].plot(PHASE, alpha / 20.0, color=color, linestyle="--", label=f"{cohort} alpha_mag / 20")

        required, required_std = mean_std(turns, "required_turn_voltage")
        configured, _ = mean_std(turns, "turn_ff")
        motor, _ = mean_std(turns, "motor_voltage_delta")
        axes[1].plot(PHASE, required, color=color, label=f"{cohort} required (FF+FB)")
        axes[1].fill_between(PHASE, required - required_std, required + required_std,
                             color=color, alpha=0.12)
        axes[1].plot(PHASE, configured, color=color, linestyle="--", label=f"{cohort} configured FF")
        axes[1].plot(PHASE, motor, color=color, linestyle=":", label=f"{cohort} motor common delta")

        error, error_std = mean_std(turns, "speed_error")
        axes[2].plot(PHASE, error * 1000.0, color=color, label=f"{cohort} target - ego")
        axes[2].fill_between(PHASE, (error - error_std) * 1000.0,
                             (error + error_std) * 1000.0, color=color, alpha=0.12)

        acceleration, acceleration_std = mean_std(turns, "acceleration")
        axes[3].plot(PHASE, acceleration, color=color, label=f"{cohort} measured acceleration")
        axes[3].fill_between(PHASE, acceleration - acceleration_std,
                             acceleration + acceleration_std, color=color, alpha=0.12)

    if recommendation is not None:
        axes[2].plot(
            PHASE, -recommendation["predicted_speed_error_conservative"] * 1000.0,
            color="tab:green", linewidth=2.0, linestyle="--",
            label="response-model prediction with 50% learning update",
        )
        axes[2].legend(fontsize=8, ncol=2)

    axes[0].set_ylabel("rad/s; rad/s^2 / 20")
    axes[1].set_ylabel("Voltage [V]")
    axes[2].set_ylabel("Speed error [mm/s]")
    axes[3].set_ylabel("Acceleration [m/s^2]")
    axes[3].set_xlabel("Turn phase [%]")
    axes[0].set_title(title)
    for axis in axes:
        axis.axhline(0.0, color="0.4", linewidth=0.7)
        axis.grid(alpha=0.25)
        axis.legend(fontsize=8, ncol=2)
    fig.savefig(group_dir / "time_response.png", dpi=160)
    plt.close(fig)

    fig, axes = plt.subplots(1, len(cohorts), figsize=(6 * len(cohorts), 4.8),
                             squeeze=False, constrained_layout=True)
    for axis, (cohort, turns) in zip(axes[0], cohorts.items()):
        required = np.concatenate([turn.required_turn_voltage for turn in turns])
        alpha = np.concatenate([turn.alpha_mag for turn in turns])
        omega_sq = np.concatenate([turn.omega_sq for turn in turns])
        fit = fits[cohort]["unconstrained"]
        predicted = fit["k_alpha"] * alpha + fit["k_omega2"] * omega_sq
        scatter = axis.scatter(predicted, required, c=np.tile(PHASE, len(turns)), s=5,
                               alpha=0.25, cmap="viridis")
        low = min(float(np.min(predicted)), float(np.min(required)))
        high = max(float(np.max(predicted)), float(np.max(required)))
        axis.plot((low, high), (low, high), color="0.25", linestyle="--")
        axis.set(title=f"{cohort}: voltage model", xlabel="modelled turn voltage [V]",
                 ylabel="required turn voltage [V]")
        axis.grid(alpha=0.25)
        fig.colorbar(scatter, ax=axis, label="Turn phase [%]")
    fig.savefig(group_dir / "voltage_model.png", dpi=160)
    plt.close(fig)


def write_group(output: Path, speed: int, direction: str, turns: list[Turn], model: dict) -> dict:
    group_dir = output / str(speed) / direction
    group_dir.mkdir(parents=True, exist_ok=True)
    cohorts = {
        cohort: [turn for turn in turns if turn.cohort == cohort]
        for cohort in ("baseline", "old_trial", "current")
        if any(turn.cohort == cohort for turn in turns)
    }
    result = {"speed_mm_s": speed, "direction": direction, "cohorts": {}}
    recommendation = response_based_recommendation(turns, model) \
        if any(turn.cohort == "baseline" for turn in turns) else None
    if recommendation is not None:
        result["response_based_recommendation"] = {
            "baseline_speed_rms_mm_s": recommendation["baseline_speed_rms_mm_s"],
            "unconstrained": recommendation["unconstrained"],
            "physical_sign_constrained": recommendation["physical_sign_constrained"],
            "recommended_conservative_half_step": recommendation["recommended_conservative_half_step"],
        }
    fits = {}
    phase_frame = pd.DataFrame({"phase_pct": PHASE})
    for cohort, selected in cohorts.items():
        unconstrained = robust_fixed_effect_fit(selected)
        constrained = robust_fixed_effect_fit(selected, constrained=True)
        response = response_fit(selected)
        metrics = metric_summary(selected)
        fits[cohort] = {"unconstrained": unconstrained, "constrained": constrained}
        result["cohorts"][cohort] = {
            "metrics": metrics,
            "required_voltage_fit": unconstrained,
            "physical_sign_constrained_fit": constrained,
            "voltage_response_fit": response,
            "sources": sorted({turn.source.split("#", 1)[0] for turn in selected}),
        }
        for field in ("omega", "alpha_mag", "omega_sq", "sp_feedback", "turn_ff",
                      "required_turn_voltage", "motor_voltage_delta", "speed_error",
                      "acceleration"):
            mean, std = mean_std(selected, field)
            phase_frame[f"{cohort}_{field}_mean"] = mean
            phase_frame[f"{cohort}_{field}_std"] = std
    if recommendation is not None:
        phase_frame["predicted_speed_error_unconstrained"] = recommendation["predicted_speed_error_unconstrained"]
        phase_frame["predicted_speed_error_constrained"] = recommendation["predicted_speed_error_constrained"]
        phase_frame["predicted_speed_error_conservative"] = recommendation["predicted_speed_error_conservative"]
    phase_frame.to_csv(group_dir / "phase_average.csv", index=False)
    (group_dir / "summary.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    plot_group(group_dir, f"search_turn {direction} {speed} mm/s", cohorts, fits, recommendation)
    report_lines = [
        f"# search_turn {direction} {speed} mm/s", "",
        "## Turn-section metrics", "",
        "| Cohort | Turns | Absolute error RMS [mm/s] | Ripple RMS [mm/s] | SP feedback RMS [V] |",
        "|---|---:|---:|---:|---:|",
    ]
    for cohort, data in result["cohorts"].items():
        metrics = data["metrics"]
        report_lines.append(
            f"| {cohort} | {metrics['turns']} | {metrics['speed_error_rms_mm_s']:.3f} "
            f"| {metrics['speed_ripple_rms_mm_s']:.3f} | {metrics['feedback_rms_v']:.5f} |"
        )
    if recommendation is not None:
        applied = recommendation["recommended_conservative_half_step"]
        report_lines.extend([
            "", "## Response-based FF recommendation", "",
            f"- k_alpha: `{applied['k_alpha']:+.8f}`",
            f"- k_omega2: `{applied['k_omega2']:+.8f}`",
            f"- predicted turn-induced speed RMS: `{applied['predicted_speed_rms_mm_s']:.3f} mm/s`",
            f"- predicted FF range: `{applied['ff_min_v']:+.3f}` to `{applied['ff_max_v']:+.3f} V`",
            "- learning rate: `50%`", "- controller FF limit: `+/-0.35 V`",
        ])
    report_lines.extend([
        "", "## Plots and numeric data", "",
        "- [Time response](time_response.png)",
        "- [Voltage model](voltage_model.png)",
        "- [Phase-average CSV](phase_average.csv)",
        "- [Full JSON](summary.json)", "",
    ])
    (group_dir / "report.md").write_text("\n".join(report_lines), encoding="utf-8")
    return result


def plot_voltage_response_validation(output: Path, model: dict) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(12, 5), constrained_layout=True)
    measured_all = []
    predicted_all = []
    for pair in model["pairs"]:
        dt_s = pair["dt_s"]
        delta_v = savgol_filter(pair["delta_v"], 11, 2, mode="interp")
        measured = np.gradient(delta_v, dt_s)
        simulated = simulate_basis(pair["delta_ff"], dt_s, model["a_closed_loop_per_s"],
                                   model["b_voltage_mps2_per_v"], model["lag_ms"])
        lag = int(round(model["lag_ms"] / 1000.0 / dt_s))
        delayed_voltage = np.zeros_like(pair["delta_ff"])
        if lag:
            delayed_voltage[lag:] = pair["delta_ff"][:-lag]
        else:
            delayed_voltage[:] = pair["delta_ff"]
        predicted = (model["a_closed_loop_per_s"] * delta_v
                     + model["b_voltage_mps2_per_v"] * delayed_voltage)
        measured_all.append(measured[3:-3])
        predicted_all.append(predicted[3:-3])
        axes[1].plot(PHASE, delta_v * 1000.0, alpha=0.55, label=f"{pair['name']} measured")
        axes[1].plot(PHASE, simulated * 1000.0, linestyle="--", alpha=0.65,
                     label=f"{pair['name']} model")
    measured = np.concatenate(measured_all)
    predicted = np.concatenate(predicted_all)
    axes[0].scatter(predicted, measured, s=7, alpha=0.25)
    low = min(float(np.min(predicted)), float(np.min(measured)))
    high = max(float(np.max(predicted)), float(np.max(measured)))
    axes[0].plot((low, high), (low, high), color="0.25", linestyle="--")
    axes[0].set(xlabel="Predicted delta acceleration [m/s^2]",
                ylabel="Measured delta acceleration [m/s^2]",
                title="Added FF voltage -> acceleration response")
    axes[1].set(xlabel="Turn phase [%]", ylabel="Experiment - baseline speed [mm/s]",
                title="Closed-loop response validation")
    for axis in axes:
        axis.grid(alpha=0.25)
    axes[1].legend(fontsize=7, ncol=2)
    fig.savefig(output / "voltage_response_validation.png", dpi=160)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("paths", nargs="+", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--period-ms", type=float, default=1.0)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    turns = [turn for path in args.paths for turn in load_file(path, args.period_ms)]
    # The deliberately sign-wrong old trial is the clean identification input.
    # Current runs are held out for validation so their controller improvement does
    # not move the plant model or the coefficient recommendation being evaluated.
    models = {"all": identify_closed_loop_model(turns)}
    for direction in ("right", "left"):
        models[direction] = identify_closed_loop_model(
            [turn for turn in turns if turn.direction == direction]
        )
    current_validation = {"all": identify_closed_loop_model(turns, ("current",))}
    for direction in ("right", "left"):
        current_validation[direction] = identify_closed_loop_model(
            [turn for turn in turns if turn.direction == direction], ("current",)
        )
    serializable_model = {
        name: {key: value for key, value in model.items() if key != "pairs"}
        for name, model in models.items()
    }
    (args.output / "voltage_response_model.json").write_text(
        json.dumps({
            "identification_old_trial": serializable_model,
            "held_out_current": {
                name: {key: value for key, value in model.items() if key != "pairs"}
                for name, model in current_validation.items()
            },
        }, indent=2), encoding="utf-8"
    )
    plot_voltage_response_validation(args.output, models["all"])
    summaries = []
    for speed in (280, 300, 320, 350, 370, 400):
        for direction in ("right", "left"):
            selected = [turn for turn in turns if turn.speed == speed and turn.direction == direction]
            if selected:
                summaries.append(write_group(args.output, speed, direction, selected, models[direction]))

    flat_rows = []
    for group in summaries:
        for cohort, data in group["cohorts"].items():
            row = {"speed_mm_s": group["speed_mm_s"], "direction": group["direction"],
                   "cohort": cohort}
            row.update(data["metrics"])
            row.update({f"fit_{key}": value for key, value in data["required_voltage_fit"].items()})
            row.update({f"constrained_{key}": value for key, value in data["physical_sign_constrained_fit"].items()})
            row.update({f"response_{key}": value for key, value in data["voltage_response_fit"].items()})
            recommendation = group.get("response_based_recommendation")
            if recommendation:
                row.update({
                    "recommended_k_alpha": recommendation["recommended_conservative_half_step"]["k_alpha"],
                    "recommended_k_omega2": recommendation["recommended_conservative_half_step"]["k_omega2"],
                    "recommended_predicted_speed_rms_mm_s": recommendation["recommended_conservative_half_step"]["predicted_speed_rms_mm_s"],
                    "recommended_ff_min_v": recommendation["recommended_conservative_half_step"]["ff_min_v"],
                    "recommended_ff_max_v": recommendation["recommended_conservative_half_step"]["ff_max_v"],
                })
            flat_rows.append(row)
    pd.DataFrame(flat_rows).to_csv(args.output / "coefficient_summary.csv", index=False)
    (args.output / "summary.json").write_text(json.dumps(summaries, indent=2), encoding="utf-8")
    print(f"Wrote {len(summaries)} group reports to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
