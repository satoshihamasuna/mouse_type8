"""Identify search-turn translational feedforward from myshell CSV logs.

The turn-induced common-mode voltage is modelled as

    delta_sp_voltage = K_alpha * sign(ideal.rad_velo) * ideal.rad_accel
                     + K_omega2 * ideal.rad_velo ** 2

For every turn, the straight-running total speed-control voltage
(`sp_feedforward + sp_feedback`) immediately before and after the turn is
linearly interpolated and removed.  This includes an already configured turn
FF in the identified requirement while preventing an error in the ordinary
straight feedforward from being learned as turn FF.
"""

from __future__ import annotations

import argparse
import json
import re
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd


NAME_RE = re.compile(r"search_turn_(right|left)_(280|300|320|350|370|400)")


@dataclass
class TurnData:
    source: str
    direction: str
    speed: int
    alpha_mag: np.ndarray
    omega_sq: np.ndarray
    correction: np.ndarray


def contiguous_regions(mask: np.ndarray) -> list[tuple[int, int]]:
    edges = np.diff(np.pad(mask.astype(np.int8), (1, 1)))
    starts = np.flatnonzero(edges == 1)
    ends = np.flatnonzero(edges == -1)
    return list(zip(starts, ends))


def baseline(values: np.ndarray, start: int, end: int, margin: int = 8,
             window: int = 35) -> np.ndarray:
    pre = values[max(0, start - margin - window):max(0, start - margin)]
    post = values[min(len(values), end + margin):min(len(values), end + margin + window)]
    pre_value = float(np.median(pre)) if len(pre) else float(values[start])
    post_value = float(np.median(post)) if len(post) else float(values[end - 1])
    return np.linspace(pre_value, post_value, end - start, endpoint=False)


def load_turns(path: Path, period_ms: float) -> list[TurnData]:
    match = NAME_RE.search(path.stem)
    if not match:
        return []
    direction, speed_text = match.groups()
    frame = pd.read_csv(path)
    omega = frame["ideal.rad_velo"].to_numpy(float)
    required_voltage = (
        frame["sp_feedforward"].to_numpy(float)
        + frame["sp_feedback"].to_numpy(float)
    )
    # The firmware constructs target acceleration with a one-sample forward
    # difference, so reproduce that timing here.  Matching its timing matters
    # more than smoothing the small half-float quantisation steps.
    alpha = np.zeros_like(omega)
    alpha[:-1] = np.diff(omega) / (period_ms / 1000.0)
    active = np.abs(omega) >= 0.05
    turns = []
    for start, end in contiguous_regions(active):
        if end - start < 20:
            continue
        turn_baseline = baseline(required_voltage, start, end)
        correction = required_voltage[start:end] - turn_baseline
        usable = np.isfinite(correction) & np.isfinite(alpha[start:end])
        if {"V_r", "V_l", "Battery"}.issubset(frame.columns):
            battery = frame["Battery"].to_numpy(float)[start:end]
            usable &= np.abs(frame["V_r"].to_numpy(float)[start:end]) < battery - 0.15
            usable &= np.abs(frame["V_l"].to_numpy(float)[start:end]) < battery - 0.15
        turns.append(TurnData(
            source=path.name,
            direction=direction,
            speed=int(speed_text),
            alpha_mag=(np.sign(omega[start:end]) * alpha[start:end])[usable],
            omega_sq=np.square(omega[start:end])[usable],
            correction=correction[usable],
        ))
    return turns


def robust_fit(turns: list[TurnData], iterations: int = 6,
               sigma: float = 3.5) -> dict[str, float]:
    x = np.column_stack((
        np.concatenate([turn.alpha_mag for turn in turns]),
        np.concatenate([turn.omega_sq for turn in turns]),
    ))
    y = np.concatenate([turn.correction for turn in turns])
    mask = np.ones(len(y), dtype=bool)
    coefficients = np.zeros(2)
    for _ in range(iterations):
        coefficients, *_ = np.linalg.lstsq(x[mask], y[mask], rcond=None)
        residual = y - x @ coefficients
        selected = residual[mask]
        median = np.median(selected)
        robust_std = 1.4826 * np.median(np.abs(selected - median))
        if robust_std <= np.finfo(float).eps:
            break
        new_mask = np.abs(residual - median) <= sigma * robust_std
        if np.array_equal(mask, new_mask):
            break
        mask = new_mask
    prediction = x @ coefficients
    residual = y - prediction
    rmse = float(np.sqrt(np.mean(np.square(residual[mask]))))
    null_rmse = float(np.sqrt(np.mean(np.square(y[mask]))))
    return {
        "k_alpha": float(coefficients[0]),
        "k_omega2": float(coefficients[1]),
        "samples": int(np.count_nonzero(mask)),
        "rejected": int(len(mask) - np.count_nonzero(mask)),
        "rmse": rmse,
        "null_rmse": null_rmse,
        "improvement_pct": 100.0 * (1.0 - rmse / null_rmse) if null_rmse else 0.0,
        "correction_min": float(np.min(prediction[mask])),
        "correction_max": float(np.max(prediction[mask])),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("paths", nargs="+", type=Path)
    parser.add_argument("--period-ms", type=float, default=1.0)
    parser.add_argument("--json", type=Path)
    args = parser.parse_args()

    turns = [turn for path in args.paths for turn in load_turns(path, args.period_ms)]
    if not turns:
        raise ValueError("No usable search turns found")

    results: dict[str, dict[str, float]] = {}
    groups = [("all", turns)]
    groups.extend((direction, [turn for turn in turns if turn.direction == direction])
                  for direction in ("right", "left"))
    groups.extend((f"{direction}_{speed}",
                   [turn for turn in turns
                    if turn.direction == direction and turn.speed == speed])
                  for speed in (280, 300, 320, 350, 370, 400)
                  for direction in ("right", "left"))
    for name, selected in groups:
        if selected:
            results[name] = robust_fit(selected)
            result = results[name]
            print(
                f"{name:10s} turns={len(selected):2d} samples={result['samples']:4.0f} "
                f"K_alpha={result['k_alpha']:+.8f} "
                f"K_omega2={result['k_omega2']:+.8f} "
                f"RMSE={result['rmse']:.5f} V "
                f"improvement={result['improvement_pct']:.1f}% "
                f"FF=[{result['correction_min']:+.3f},{result['correction_max']:+.3f}] V"
            )
    if args.json:
        args.json.write_text(json.dumps(results, indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
