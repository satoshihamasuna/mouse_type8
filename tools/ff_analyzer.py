"""Identify feedforward coefficients from myshell CSV logs.

The closed-loop controller output is treated as the voltage that the plant needed:

    required_sp = sp_feedforward + sp_feedback
    required_om = om_feedforward + om_feedback

Then the two independent models below are fitted with iterative MAD clipping.
Use --with-intercept to identify the forward-run magnitude of signed_bias:

    required_sp = signed_bias + K_sp_velo * ideal.velo + K_sp_accel * ideal.accel
    required_om = K_om_velo * ideal.rad_velo
                + K_om_accel * ideal.rad_accel  (angular-speed magnitude increasing)
                + K_om_decel * ideal.rad_accel  (angular-speed magnitude decreasing)

If ideal.rad_accel is not logged, it is derived from ideal.rad_velo per input file.
"""

from __future__ import annotations

import argparse
import glob
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np
import pandas as pd


LOG_DIR = Path(__file__).resolve().parent / "logs"
T_FF_GAIN_ORDER = (
    "sp_velo", "sp_accel", "sp_bias",
    "om_velo", "om_accel", "om_decel", "om_bias",
)
REQUIRED_COLUMNS = (
    "ideal.velo",
    "ideal.accel",
    "ideal.rad_velo",
    "sp_feedforward",
    "sp_feedback",
    "om_feedforward",
    "om_feedback",
)


@dataclass
class FitResult:
    names: tuple[str, ...]
    coefficients: np.ndarray
    sample_count: int
    rejected_count: int
    rmse: float
    r_squared: float
    condition_number: float
    intercept: float = 0.0

    def as_dict(self) -> dict[str, object]:
        return {
            "coefficients": {
                name: float(coefficient)
                for name, coefficient in zip(self.names, self.coefficients)
            },
            "intercept": self.intercept,
            "sample_count": self.sample_count,
            "rejected_count": self.rejected_count,
            "rmse_V": self.rmse,
            "r_squared": self.r_squared,
            "condition_number": self.condition_number,
        }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="myshell CSVログから速度・加速度FF係数を同定します。"
    )
    parser.add_argument("paths", nargs="*", type=Path, help="入力CSV（複数指定可）")
    parser.add_argument(
        "--latest", action="store_true", help="tools/logs内の最新CSVだけを使用（入力省略時も同じ）"
    )
    parser.add_argument(
        "--period-ms", type=float, default=2.0, help="ログ周期 [ms]（default: 2.0）"
    )
    parser.add_argument(
        "--sigma", type=float, default=3.5, help="MAD外れ値除去のしきい値"
    )
    parser.add_argument(
        "--iterations", type=int, default=5, help="外れ値除去の最大反復回数"
    )
    parser.add_argument(
        "--battery-margin", type=float, default=0.15,
        help="飽和除外用のバッテリー電圧余裕 [V]",
    )
    parser.add_argument(
        "--keep-saturated", action="store_true", help="出力飽和付近のサンプルも使用"
    )
    parser.add_argument(
        "--with-intercept", action="store_true",
        help="定数項も同定（通常はDEAD_Vと重複するため不要）",
    )
    parser.add_argument("--plot", action="store_true", help="同定結果をグラフ表示")
    parser.add_argument("--json", type=Path, help="結果をJSONへ保存")
    return parser.parse_args()


def resolve_paths(paths: Iterable[Path], latest: bool) -> list[Path]:
    resolved: list[Path] = []
    for path in paths:
        if path.is_dir():
            resolved.extend(sorted(path.glob("*.csv")))
        elif "*" in str(path):
            resolved.extend(Path(item) for item in sorted(glob.glob(str(path))))
        else:
            resolved.append(path)
    if not resolved:
        candidates = list(LOG_DIR.glob("*.csv"))
        if not candidates:
            raise FileNotFoundError(f"CSVログがありません: {LOG_DIR}")
        resolved = [max(candidates, key=lambda item: item.stat().st_mtime)]
    unique = list(dict.fromkeys(path.resolve() for path in resolved if path.exists()))
    if not unique:
        raise FileNotFoundError("指定されたCSVログが見つかりません。")
    return unique


def load_log(path: Path, period_ms: float) -> pd.DataFrame:
    frame = pd.read_csv(path)
    frame.columns = [str(column).strip() for column in frame.columns]
    missing = [name for name in REQUIRED_COLUMNS if name not in frame]
    if missing:
        raise ValueError(f"{path.name}: 必須列がありません: {', '.join(missing)}")
    frame = frame.apply(pd.to_numeric, errors="coerce")
    if len(frame) < 2:
        raise ValueError(f"{path.name}: データ行が2行未満です")
    if "ideal.rad_accel" not in frame:
        omega = frame["ideal.rad_velo"].to_numpy(dtype=float)
        frame["ideal.rad_accel"] = np.gradient(omega, period_ms / 1000.0)
    frame["source"] = path.name
    frame["source_row"] = np.arange(len(frame))
    return frame


def saturation_mask(frame: pd.DataFrame, margin: float) -> np.ndarray:
    if not {"V_r", "V_l", "Battery"}.issubset(frame.columns):
        return np.ones(len(frame), dtype=bool)
    battery = frame["Battery"].to_numpy(dtype=float)
    limit = np.maximum(battery - margin, 0.0)
    right = np.abs(frame["V_r"].to_numpy(dtype=float))
    left = np.abs(frame["V_l"].to_numpy(dtype=float))
    return (right < limit) & (left < limit)


def robust_fit(
    predictors: np.ndarray,
    target: np.ndarray,
    base_mask: np.ndarray,
    names: tuple[str, ...],
    sigma: float,
    iterations: int,
    with_intercept: bool,
) -> tuple[FitResult, np.ndarray, np.ndarray]:
    finite = np.isfinite(target) & np.all(np.isfinite(predictors), axis=1)
    mask = base_mask & finite
    if np.count_nonzero(mask) < 20:
        raise ValueError(f"{names}: 有効なサンプルが20点未満です。")

    design = predictors
    if with_intercept:
        design = np.column_stack((predictors, np.ones(len(predictors))))

    initial_count = int(np.count_nonzero(mask))
    coefficients = np.zeros(design.shape[1])
    for _ in range(max(1, iterations)):
        coefficients, *_ = np.linalg.lstsq(design[mask], target[mask], rcond=None)
        residual = target - design @ coefficients
        selected = residual[mask]
        median = float(np.median(selected))
        mad = float(np.median(np.abs(selected - median)))
        robust_std = 1.4826 * mad
        if robust_std <= np.finfo(float).eps:
            break
        new_mask = base_mask & finite & (np.abs(residual - median) <= sigma * robust_std)
        if np.array_equal(new_mask, mask):
            break
        if np.count_nonzero(new_mask) < design.shape[1] + 2:
            break
        mask = new_mask

    predicted = design @ coefficients
    residual = target - predicted
    selected_target = target[mask]
    selected_residual = residual[mask]
    rmse = float(np.sqrt(np.mean(np.square(selected_residual))))
    denominator = float(np.sum(np.square(selected_target - np.mean(selected_target))))
    r_squared = 1.0 - float(np.sum(np.square(selected_residual))) / denominator if denominator > 0 else float("nan")

    scale = np.std(design[mask], axis=0)
    scale[scale <= np.finfo(float).eps] = 1.0
    condition_number = float(np.linalg.cond(design[mask] / scale))
    result = FitResult(
        names=names,
        coefficients=coefficients[:len(names)],
        intercept=float(coefficients[len(names)]) if with_intercept else 0.0,
        sample_count=int(np.count_nonzero(mask)),
        rejected_count=initial_count - int(np.count_nonzero(mask)),
        rmse=rmse,
        r_squared=r_squared,
        condition_number=condition_number,
    )
    return result, predicted, mask


def excitation_mask(first: np.ndarray, second: np.ndarray, first_min: float, second_min: float) -> np.ndarray:
    return (np.abs(first) >= first_min) | (np.abs(second) >= second_min)


def phase_summary(frame: pd.DataFrame, usable: np.ndarray) -> None:
    acceleration = frame["ideal.accel"].to_numpy(dtype=float)
    angular_velocity = frame["ideal.rad_velo"].to_numpy(dtype=float)
    angular_acceleration = frame["ideal.rad_accel"].to_numpy(dtype=float)
    angular_decelerating = angular_velocity * angular_acceleration < 0.0
    groups = (
        ("SP steady", usable & (np.abs(acceleration) < 0.2), "sp_feedback"),
        ("SP accel/decel", usable & (np.abs(acceleration) >= 0.2), "sp_feedback"),
        ("OM steady", usable & (np.abs(angular_acceleration) < 2.0), "om_feedback"),
        ("OM acceleration", usable & ~angular_decelerating & (np.abs(angular_acceleration) >= 2.0), "om_feedback"),
        ("OM deceleration", usable & angular_decelerating & (np.abs(angular_acceleration) >= 2.0), "om_feedback"),
    )
    print("\nResidual feedback by phase (before refitting):")
    for label, mask, column in groups:
        values = frame[column].to_numpy(dtype=float)[mask]
        values = values[np.isfinite(values)]
        if len(values):
            rms = float(np.sqrt(np.mean(np.square(values))))
            print(f"  {label:16s} n={len(values):6d} mean={np.mean(values):+.5f} V  rms={rms:.5f} V")


def excitation_warnings(frame: pd.DataFrame, usable: np.ndarray) -> list[str]:
    velocity = frame["ideal.velo"].to_numpy(dtype=float)
    acceleration = frame["ideal.accel"].to_numpy(dtype=float)
    omega = frame["ideal.rad_velo"].to_numpy(dtype=float)
    alpha = frame["ideal.rad_accel"].to_numpy(dtype=float)
    angular_decelerating = omega * alpha < 0.0
    checks = (
        ("SP velocity", usable & (np.abs(velocity) >= 0.03) & (np.abs(acceleration) < 0.2)),
        ("SP acceleration", usable & (np.abs(acceleration) >= 0.2)),
        ("OM velocity", usable & (np.abs(omega) >= 0.1) & (np.abs(alpha) < 2.0)),
        ("OM acceleration", usable & ~angular_decelerating & (np.abs(alpha) >= 2.0)),
        ("OM deceleration", usable & angular_decelerating & (np.abs(alpha) >= 2.0)),
    )
    return [f"{name} excitation is sparse ({np.count_nonzero(mask)} samples)" for name, mask in checks
            if np.count_nonzero(mask) < 100]


def print_result(title: str, result: FitResult) -> None:
    print(f"\n{title}")
    for name, coefficient in zip(result.names, result.coefficients):
        print(f"  {name:18s} = {coefficient:.7g}")
    if result.intercept:
        print(f"  intercept          = {result.intercept:+.7g} V")
    print(
        f"  samples={result.sample_count}, rejected={result.rejected_count}, "
        f"RMSE={result.rmse:.5f} V, R2={result.r_squared:.5f}, cond={result.condition_number:.2f}"
    )


def bias_diagnostic(label: str, base_fit, intercept_fit) -> dict[str, object] | None:
    if not base_fit or not intercept_fit:
        return None
    base_result = base_fit[0]
    intercept_result = intercept_fit[0]
    improvement = 1.0 - intercept_result.rmse / base_result.rmse if base_result.rmse > 0 else 0.0
    diagnostic = {
        "intercept_V": intercept_result.intercept,
        "rmse_improvement_ratio": improvement,
        "coefficients": intercept_result.as_dict()["coefficients"],
    }
    if abs(intercept_result.intercept) >= 0.03 and improvement >= 0.15:
        print(
            f"WARNING: {label} has an unmodelled bias of {intercept_result.intercept:+.5f} V; "
            f"adding an intercept improves RMSE by {improvement * 100:.1f}%."
        )
        print("         Review DEAD_V or add a signed Coulomb-friction term before changing only the velocity coefficient.")
    return diagnostic


def plot_results(frame: pd.DataFrame, results: dict[str, tuple[np.ndarray, np.ndarray]]) -> None:
    import matplotlib.pyplot as plt

    channels = [(channel, f"required_{channel}") for channel in ("sp", "om") if channel in results]
    _figure, axes = plt.subplots(len(channels), 2, figsize=(13, 4.5 * len(channels)),
                                 constrained_layout=True, squeeze=False)
    for row, (channel, target_column) in enumerate(channels):
        predicted, mask = results[channel]
        target = frame[target_column].to_numpy(dtype=float)
        axes[row, 0].scatter(target[mask], predicted[mask], s=5, alpha=0.25)
        low = min(float(np.min(target[mask])), float(np.min(predicted[mask])))
        high = max(float(np.max(target[mask])), float(np.max(predicted[mask])))
        axes[row, 0].plot((low, high), (low, high), "k--", linewidth=1)
        axes[row, 0].set(title=f"{channel.upper()} required vs fitted", xlabel="required [V]", ylabel="fitted [V]")
        axes[row, 0].grid(True, alpha=0.3)
        axes[row, 1].hist((target - predicted)[mask], bins=80)
        axes[row, 1].set(title=f"{channel.upper()} residual", xlabel="required - fitted [V]", ylabel="count")
        axes[row, 1].grid(True, alpha=0.3)
    plt.show()


def fit_or_none(*args, **kwargs):
    try:
        return robust_fit(*args, **kwargs), None
    except ValueError as exc:
        return None, str(exc)


def main() -> int:
    args = parse_args()
    if args.period_ms <= 0 or args.sigma <= 0 or args.iterations <= 0:
        raise ValueError("period-ms, sigma, iterationsは正の値にしてください。")
    paths = resolve_paths(args.paths, args.latest)
    frames = []
    loaded_paths = []
    skipped = []
    for path in paths:
        try:
            frames.append(load_log(path, args.period_ms))
            loaded_paths.append(path)
        except (ValueError, pd.errors.EmptyDataError) as exc:
            skipped.append((path, str(exc)))
    if not frames:
        reasons = "; ".join(f"{path.name}: {reason}" for path, reason in skipped)
        raise ValueError(f"解析可能なCSVログがありません。{reasons}")
    frame = pd.concat(frames, ignore_index=True)

    frame["required_sp"] = frame["sp_feedforward"] + frame["sp_feedback"]
    frame["required_om"] = frame["om_feedforward"] + frame["om_feedback"]
    usable = np.ones(len(frame), dtype=bool)
    if not args.keep_saturated:
        usable &= saturation_mask(frame, args.battery_margin)

    velocity = frame["ideal.velo"].to_numpy(dtype=float)
    acceleration = frame["ideal.accel"].to_numpy(dtype=float)
    angular_velocity = frame["ideal.rad_velo"].to_numpy(dtype=float)
    angular_acceleration = frame["ideal.rad_accel"].to_numpy(dtype=float)
    angular_decelerating = angular_velocity * angular_acceleration < 0.0
    angular_accel_term = np.where(angular_decelerating, 0.0, angular_acceleration)
    angular_decel_term = np.where(angular_decelerating, angular_acceleration, 0.0)
    sp_mask = usable & excitation_mask(velocity, acceleration, 0.03, 0.2)
    om_mask = usable & excitation_mask(angular_velocity, angular_acceleration, 0.1, 2.0)

    sp_fit, sp_error = fit_or_none(
        np.column_stack((velocity, acceleration)),
        frame["required_sp"].to_numpy(dtype=float),
        sp_mask,
        ("FF_SP_VELO_COEF", "FF_SP_ACCEL_COEF"),
        args.sigma,
        args.iterations,
        args.with_intercept,
    )
    om_fit, om_error = fit_or_none(
        np.column_stack((angular_velocity, angular_accel_term, angular_decel_term)),
        frame["required_om"].to_numpy(dtype=float),
        om_mask,
        ("FF_OM_VELO_COEF", "FF_OM_ACCEL_COEF", "FF_OM_DECEL_COEF"),
        args.sigma,
        args.iterations,
        args.with_intercept,
    )
    if sp_fit is None and om_fit is None:
        raise ValueError(f"SP: {sp_error}; OM: {om_error}")

    print(f"Loaded {len(loaded_paths)} file(s), {len(frame)} rows, period={args.period_ms:g} ms")
    for path in loaded_paths:
        print(f"  {path}")
    for path, reason in skipped:
        print(f"SKIPPED: {path}: {reason}")
    phase_summary(frame, usable)
    if sp_fit:
        print_result("Translational feedforward", sp_fit[0])
    else:
        print(f"\nTranslational feedforward\n  unavailable: {sp_error}")
    if om_fit:
        print_result("Angular feedforward", om_fit[0])
    else:
        print(f"\nAngular feedforward\n  unavailable: {om_error}")
    suggested_values = None
    if sp_fit and om_fit:
        sp_result = sp_fit[0]
        om_result = om_fit[0]
        raw_values = (
            sp_result.coefficients[0], sp_result.coefficients[1],
            sp_result.intercept if args.with_intercept else 0.0,
            om_result.coefficients[0], om_result.coefficients[1], om_result.coefficients[2],
            om_result.intercept if args.with_intercept else 0.0,
        )
        suggested_values = tuple(max(0.0, float(value)) for value in raw_values)
    print("\nSuggested definitions:")
    if sp_fit:
        sp_velo = suggested_values[0] if suggested_values else max(0.0, float(sp_fit[0].coefficients[0]))
        sp_accel = suggested_values[1] if suggested_values else max(0.0, float(sp_fit[0].coefficients[1]))
        print(f"#define FF_SP_VELO_COEF  ({sp_velo:.7g}f)")
        print(f"#define FF_SP_ACCEL_COEF ({sp_accel:.7g}f)")
        if args.with_intercept:
            sp_bias = suggested_values[2] if suggested_values else max(0.0, float(sp_fit[0].intercept))
            print(f"#define FF_SP_BIAS_COEF  ({sp_bias:.7g}f)")
    if om_fit:
        om_velo = suggested_values[3] if suggested_values else max(0.0, float(om_fit[0].coefficients[0]))
        om_accel = suggested_values[4] if suggested_values else max(0.0, float(om_fit[0].coefficients[1]))
        om_decel = suggested_values[5] if suggested_values else max(0.0, float(om_fit[0].coefficients[2]))
        print(f"#define FF_OM_VELO_COEF  ({om_velo:.7g}f)")
        print(f"#define FF_OM_ACCEL_COEF ({om_accel:.7g}f)")
        print(f"#define FF_OM_DECEL_COEF ({om_decel:.7g}f)")
    if args.with_intercept:
        if om_fit:
            om_bias = suggested_values[6] if suggested_values else max(0.0, float(om_fit[0].intercept))
            print(f"#define FF_OM_BIAS_COEF  ({om_bias:.7g}f)")
        print("// Apply each BIAS coefficient with the sign of its velocity (or acceleration at launch).")
    if suggested_values:
        print("// t_ff_gain order: sp_velo, sp_accel, sp_bias, om_velo, om_accel, om_decel, om_bias")
        print("{" + ", ".join(f"{value:.7g}f" for value in suggested_values) + "};")
        clamped_names = [name for name, raw in zip(T_FF_GAIN_ORDER, raw_values) if raw < 0.0]
        if clamped_names:
            print(f"WARNING: negative suggested coefficients were constrained to zero: {', '.join(clamped_names)}")
    condition_numbers = [fit[0].condition_number for fit in (sp_fit, om_fit) if fit]
    if max(condition_numbers) > 30:
        print("\nWARNING: condition number is high. Use logs with independent steady and accel/decel phases.")
    for warning in excitation_warnings(frame, usable):
        print(f"WARNING: {warning}")

    bias_report = {}
    if args.with_intercept:
        if sp_fit:
            bias_report["translational"] = {"intercept_V": sp_fit[0].intercept}
        if om_fit:
            bias_report["angular"] = {"intercept_V": om_fit[0].intercept}
    else:
        sp_intercept_fit, _ = fit_or_none(
            np.column_stack((velocity, acceleration)), frame["required_sp"].to_numpy(dtype=float),
            sp_mask, ("FF_SP_VELO_COEF", "FF_SP_ACCEL_COEF"), args.sigma, args.iterations, True,
        )
        om_intercept_fit, _ = fit_or_none(
            np.column_stack((angular_velocity, angular_accel_term, angular_decel_term)),
            frame["required_om"].to_numpy(dtype=float), om_mask,
            ("FF_OM_VELO_COEF", "FF_OM_ACCEL_COEF", "FF_OM_DECEL_COEF"),
            args.sigma, args.iterations, True,
        )
        sp_bias = bias_diagnostic("SP", sp_fit, sp_intercept_fit)
        om_bias = bias_diagnostic("OM", om_fit, om_intercept_fit)
        if sp_bias:
            bias_report["translational"] = sp_bias
        if om_bias:
            bias_report["angular"] = om_bias

    suggested_t_ff_gain = (
        {name: value for name, value in zip(T_FF_GAIN_ORDER, suggested_values)}
        if suggested_values else None
    )
    report = {
        "files": [str(path) for path in loaded_paths],
        "skipped_files": [{"path": str(path), "reason": reason} for path, reason in skipped],
        "period_ms": args.period_ms,
        "translational": sp_fit[0].as_dict() if sp_fit else {"error": sp_error},
        "angular": om_fit[0].as_dict() if om_fit else {"error": om_error},
        "t_ff_gain_order": list(T_FF_GAIN_ORDER),
        "suggested_t_ff_gain": suggested_t_ff_gain,
        "bias_diagnostic": bias_report,
    }
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"\nJSON: {args.json}")
    if args.plot:
        plot_data = {}
        if sp_fit:
            plot_data["sp"] = (sp_fit[1], sp_fit[2])
        if om_fit:
            plot_data["om"] = (om_fit[1], om_fit[2])
        plot_results(frame, plot_data)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
