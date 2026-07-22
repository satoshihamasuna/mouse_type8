"""Generate reusable turn feedforward reports from myshell CSV/Settings logs.

Reports are grouped by speed and motion under tools/turn_analysis.  The latest
group of logs having identical motion, PID, feedforward, and suction settings is
used so experiments with different gains are never mixed.
"""

from __future__ import annotations

import argparse
import ast
import json
import math
import re
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[2]
LOG_DIR = ROOT / "tools" / "logs"
DEFAULT_OUTPUT = ROOT / "tools" / "turn_analysis"
PERIOD_S = 0.001
ACCEL_INTEGRAL = 0.7043
JERK_LIMIT_V = 2.0
FF_NAMES = (
    "sp_velo", "sp_accel", "sp_bias", "om_velo",
    "om_accel", "om_decel", "om_bias", "om_jerk",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--speed", type=int, required=True)
    parser.add_argument("--motion", nargs="*", default=["long_r90", "long_l90"])
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--project-header", type=Path)
    return parser.parse_args()


def settings_fingerprint(settings: dict) -> str:
    selected = {
        "motion": settings.get("motion_name"),
        "parameters": settings.get("parameters"),
        "pid": settings.get("pid"),
        "feedforward": settings.get("feedforward"),
        "suction": settings.get("suction"),
    }
    return json.dumps(selected, sort_keys=True)


def select_latest_group(speed: int, motion: str) -> list[tuple[Path, dict]]:
    groups: dict[str, list[tuple[Path, dict]]] = {}
    for path in sorted(LOG_DIR.glob("*.settings.json")):
        settings = json.loads(path.read_text(encoding="utf-8"))
        params = settings.get("parameters", {})
        # parameters.velo is authoritative; preset_speed_mm_s can be stale.
        if settings.get("motion_name") != motion:
            continue
        if abs(float(params.get("velo", 0.0)) * 1000.0 - speed) > 1.0:
            continue
        csv_path = path.with_name(path.name[:-len(".settings.json")] + ".csv")
        if csv_path.exists():
            groups.setdefault(settings_fingerprint(settings), []).append((csv_path, settings))
    if not groups:
        return []
    return max(groups.values(), key=lambda group: max(item[0].stat().st_mtime for item in group))


def turn_segments(ideal_omega: np.ndarray) -> list[tuple[int, int]]:
    # ideal_omega is direction-normalized by analyze_group. Ignore unrelated
    # opposite-direction turns that may also be present in the capture.
    core = ideal_omega > 1.0
    edges = np.diff(np.r_[False, core, False].astype(int))
    starts = np.where(edges == 1)[0]
    ends = np.where(edges == -1)[0] - 1
    # Firmware profile endpoints contribute two additional 1 ms samples/side.
    return [(max(0, a - 2), min(len(ideal_omega) - 1, b + 2)) for a, b in zip(starts, ends)]


def applied_angular_voltage(frame: pd.DataFrame) -> np.ndarray:
    battery = frame["Battery"].to_numpy(float)
    right = np.clip(frame["V_r"].to_numpy(float), -battery, battery)
    left = np.clip(frame["V_l"].to_numpy(float), -battery, battery)
    return (right + left) / 2.0


def response_fit(samples: list[dict], phase: str) -> dict:
    best = None
    for delay in range(9):
        voltage, omega, alpha = [], [], []
        for sample in samples:
            peak = sample["peak"]
            indices = (np.arange(sample["start"], peak + 1) if phase == "first"
                       else np.arange(peak + 1, sample["end"] + 1))
            indices = indices[indices + delay <= sample["end"]]
            voltage.extend(sample["voltage"][indices])
            omega.extend(sample["ego_omega"][indices + delay])
            alpha.extend(sample["ego_alpha"][indices + delay])
        voltage = np.asarray(voltage)
        omega = np.asarray(omega)
        alpha = np.asarray(alpha)
        design = np.column_stack((voltage, omega, np.ones(len(voltage))))
        coefficients = np.linalg.lstsq(design, alpha, rcond=None)[0]
        prediction = design @ coefficients
        denominator = np.sum(np.square(alpha - np.mean(alpha)))
        r_squared = 1.0 - np.sum(np.square(alpha - prediction)) / denominator
        candidate = (r_squared, delay, coefficients)
        if best is None or candidate[0] > best[0]:
            best = candidate
    assert best is not None
    return {
        "r_squared": float(best[0]),
        "delay_ms": int(best[1]),
        "voltage_to_alpha": float(best[2][0]),
        "omega_term": float(best[2][1]),
        "bias": float(best[2][2]),
    }


def analyze_group(group: list[tuple[Path, dict]], output: Path) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    settings = group[-1][1]
    direction = 1.0 if float(settings["parameters"]["degree"]) > 0 else -1.0
    ff = settings["feedforward"]
    params = settings["parameters"]
    omega_max = abs(float(params["velo"]) / (float(params["r_min"]) / 1000.0))
    profile_ms = abs(math.radians(float(params["degree"])) /
                     (ACCEL_INTEGRAL * omega_max) * 1000.0)
    expected_segment_ms = math.ceil(profile_ms) + 2
    rows, samples = [], []
    traces: dict[str, list[np.ndarray]] = {name: [] for name in (
        "ideal", "ego", "ff_velo", "ff_accel", "ff_bias", "ff_jerk",
        "pid", "applied", "ego_alpha", "post_omega", "post_voltage",
    )}

    for csv_path, _ in group:
        frame = pd.read_csv(csv_path)
        ideal = direction * frame["ideal.rad_velo"].to_numpy(float)
        ego = direction * frame["ego.rad_velo"].to_numpy(float)
        ideal_alpha = np.gradient(ideal, PERIOD_S)
        ego_alpha = np.gradient(ego, PERIOD_S)
        ideal_jerk = np.gradient(ideal_alpha, PERIOD_S)
        decel = ideal * ideal_alpha < 0.0
        ff_velo = float(ff["ff_om_velo"]) * ideal
        ff_accel = np.where(decel, float(ff["ff_om_decel"]), float(ff["ff_om_accel"])) * ideal_alpha
        ff_bias = float(ff["ff_om_bias"]) * np.where(np.abs(ideal) > 0.001, 1.0, np.sign(ideal_alpha))
        logged_ff = direction * frame["om_feedforward"].to_numpy(float)
        ff_jerk = logged_ff - ff_velo - ff_accel - ff_bias
        pid = direction * frame["om_feedback"].to_numpy(float)
        voltage = direction * applied_angular_voltage(frame)
        battery = frame["Battery"].to_numpy(float)

        segment_tolerance_ms = max(3, math.ceil(expected_segment_ms * 0.06))
        segments = [segment for segment in turn_segments(ideal)
                    if abs((segment[1] - segment[0] + 1) - expected_segment_ms)
                    <= segment_tolerance_ms]
        for number, (start, end) in enumerate(segments, 1):
            indices = np.arange(start, end + 1)
            peak = start + int(np.argmax(ideal[indices]))
            split = peak - start + 1
            error = ego[indices] - ideal[indices]
            # Do not count the beginning of the following turn as post-turn
            # straight response when repeated turns are closely spaced.
            next_start = segments[number][0] if number < len(segments) else len(frame)
            post = np.arange(end + 1, min(len(frame), end + 31, next_start))
            post_omega = ego[post]
            saturation = ((np.abs(frame["V_r"].to_numpy(float)[indices]) >= battery[indices]) |
                          (np.abs(frame["V_l"].to_numpy(float)[indices]) >= battery[indices]))
            rows.append({
                "file": csv_path.name, "turn": number, "start": start, "end": end,
                "duration_ms": len(indices), "ideal_peak_rad_s": float(np.max(ideal[indices])),
                "ego_peak_rad_s": float(np.max(ego[indices])),
                "peak_error_rad_s": float(np.max(ego[indices]) - np.max(ideal[indices])),
                "peak_delay_ms": int(np.argmax(ego[indices]) - np.argmax(ideal[indices])),
                "first_over_rad_s": float(np.max(error[:split])),
                "first_under_rad_s": float(np.min(error[:split])),
                "first_error_mean_rad_s": float(np.mean(error[:split])),
                "first_rms_rad_s": float(np.sqrt(np.mean(np.square(error[:split])))),
                "first_pid_mean_v": float(np.mean(pid[indices[:split]])),
                "second_over_rad_s": float(np.max(error[split:])),
                "second_under_rad_s": float(np.min(error[split:])),
                "second_error_mean_rad_s": float(np.mean(error[split:])),
                "second_rms_rad_s": float(np.sqrt(np.mean(np.square(error[split:])))),
                "second_pid_mean_v": float(np.mean(pid[indices[split:]])),
                "tracking_rms_rad_s": float(np.sqrt(np.mean(np.square(error)))),
                "angle_error_deg": float(np.sum(error) * PERIOD_S * 180.0 / math.pi),
                "pid_rms_v": float(np.sqrt(np.mean(np.square(pid[indices])))),
                "saturation_pct": float(100.0 * np.mean(saturation)),
                "first_saturation_pct": float(100.0 * np.mean(saturation[:split])),
                "second_saturation_pct": float(100.0 * np.mean(saturation[split:])),
                "post_peak_pos_rad_s": float(np.max(post_omega)) if len(post) else math.nan,
                "post_peak_neg_rad_s": float(np.min(post_omega)) if len(post) else math.nan,
                "post_rms_rad_s": float(np.sqrt(np.mean(np.square(post_omega)))) if len(post) else math.nan,
                "post_angle_deg": float(np.sum(post_omega) * PERIOD_S * 180.0 / math.pi) if len(post) else math.nan,
                "post_duration_ms": int(len(post)),
            })
            samples.append({
                "file": csv_path.name, "turn": number,
                "start": start, "end": end, "peak": peak, "voltage": voltage,
                "ego_omega": ego, "ego_alpha": ego_alpha,
            })
            for key, values in (
                ("ideal", ideal), ("ego", ego), ("ff_velo", ff_velo),
                ("ff_accel", ff_accel), ("ff_bias", ff_bias), ("ff_jerk", ff_jerk),
                ("pid", pid), ("applied", voltage), ("ego_alpha", ego_alpha),
            ):
                traces[key].append(values[indices])
            if len(post):
                post_omega_trace = np.full(30, np.nan)
                post_voltage_trace = np.full(30, np.nan)
                post_omega_trace[:len(post)] = ego[post]
                post_voltage_trace[:len(post)] = voltage[post]
                traces["post_omega"].append(post_omega_trace)
                traces["post_voltage"].append(post_voltage_trace)

    metrics = pd.DataFrame(rows)
    # Reject only gross turn-level faults from aggregate identification/reporting.
    def robust_ceiling(column: str) -> float:
        median = float(metrics[column].median())
        mad = float(np.median(np.abs(metrics[column] - median)))
        return median + 3.5 * 1.4826 * mad if mad > 0 else math.inf

    metrics["aggregate_used"] = (
        (metrics["tracking_rms_rad_s"] <= robust_ceiling("tracking_rms_rad_s")) &
        (metrics["peak_error_rad_s"] <= robust_ceiling("peak_error_rad_s"))
    )
    metrics.to_csv(output / "turn_metrics.csv", index=False)

    used = metrics[metrics["aggregate_used"]]
    used_keys = set(zip(used["file"], used["turn"]))
    used_samples = [sample for sample in samples if (sample["file"], sample["turn"]) in used_keys]
    summary = {
        "motion_name": settings["motion_name"],
        "speed_mm_s": int(round(float(settings["parameters"]["velo"]) * 1000.0)),
        "source_files": [path.name for path, _ in group],
        "settings": settings,
        "turn_count": int(len(metrics)),
        "aggregate_turn_count": int(len(used)),
        "metrics_mean": {name: float(used[name].mean()) for name in metrics.columns
                         if pd.api.types.is_numeric_dtype(metrics[name])},
        "response": {
            "first": response_fit(used_samples, "first"),
            "second": response_fit(used_samples, "second"),
        },
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")

    def stack(name: str) -> np.ndarray:
        return np.vstack(traces[name])

    def band(axis, name: str, label: str, color: str, x: np.ndarray) -> None:
        values = stack(name)
        valid = np.any(np.isfinite(values), axis=0)
        values = values[:, valid]
        valid_x = x[valid]
        axis.plot(valid_x, np.nanmean(values, axis=0), label=label, color=color)
        axis.fill_between(valid_x, np.nanmin(values, axis=0), np.nanmax(values, axis=0), color=color, alpha=0.14)

    length = min(len(item) for item in traces["ideal"])
    for name in list(traces):
        traces[name] = [item[:length] for item in traces[name]] if name not in ("post_omega", "post_voltage") else traces[name]
    x = np.arange(length)
    fig, axes = plt.subplots(4, 1, figsize=(12, 14))
    band(axes[0], "ideal", "ideal omega", "black", x)
    band(axes[0], "ego", "ego omega", "tab:blue", x)
    axes[0].set_ylabel("omega [rad/s]")
    axes[0].legend()
    for name, label, color in (
        ("ff_velo", "FF velocity", "tab:blue"), ("ff_accel", "FF accel/decel", "tab:orange"),
        ("ff_jerk", "FF jerk", "tab:red"), ("ff_bias", "FF bias", "tab:green"),
        ("pid", "PID", "tab:purple"),
    ):
        band(axes[1], name, label, color, x)
    axes[1].set_ylabel("controller term [V]")
    axes[1].legend(ncol=3)
    band(axes[2], "applied", "applied angular voltage", "tab:red", x)
    twin = axes[2].twinx()
    values = stack("ego_alpha")
    twin.plot(x, np.mean(values, axis=0), color="tab:blue", label="ego angular accel")
    twin.fill_between(x, np.min(values, axis=0), np.max(values, axis=0), color="tab:blue", alpha=0.14)
    axes[2].set_ylabel("angular voltage [V]")
    twin.set_ylabel("alpha [rad/s^2]")
    post_x = np.arange(1, 31)
    if traces["post_omega"]:
        band(axes[3], "post_omega", "post-turn omega", "tab:blue", post_x)
        band(axes[3], "post_voltage", "post-turn angular voltage", "tab:red", post_x)
    axes[3].axhline(0.5, color="gray", linestyle=":")
    axes[3].axhline(-0.5, color="gray", linestyle=":")
    axes[3].set_ylabel("omega [rad/s] / voltage [V]")
    axes[3].set_xlabel("time [ms]")
    axes[3].legend()
    for axis in axes:
        axis.grid(alpha=0.2)
    fig.suptitle(f"{settings['motion_name']} @ {summary['speed_mm_s']} mm/s")
    fig.tight_layout()
    fig.savefig(output / "response.png", dpi=160)
    plt.close(fig)

    mean = summary["metrics_mean"]
    report = f"""# {settings['motion_name']} @ {summary['speed_mm_s']} mm/s

- source logs: {len(group)}
- turns: {summary['turn_count']} (aggregate: {summary['aggregate_turn_count']})
- peak error: {mean['peak_error_rad_s']:+.4f} rad/s
- first-half RMS: {mean['first_rms_rad_s']:.4f} rad/s
- second-half RMS: {mean['second_rms_rad_s']:.4f} rad/s
- turn angle error: {mean['angle_error_deg']:+.4f} deg
- post-turn RMS: {mean['post_rms_rad_s']:.4f} rad/s
- post-turn angle: {mean['post_angle_deg']:+.4f} deg
- PID RMS: {mean['pid_rms_v']:.4f} V
- saturation: {mean['saturation_pct']:.3f}%
- voltage response delay: first {summary['response']['first']['delay_ms']} ms, second {summary['response']['second']['delay_ms']} ms
"""
    (output / "report.md").write_text(report, encoding="utf-8")
    return summary


def arithmetic_value(text: str) -> float:
    node = ast.parse(text.replace("f", ""), mode="eval")
    allowed = (ast.Expression, ast.Constant, ast.UnaryOp, ast.USub, ast.UAdd,
               ast.BinOp, ast.Add, ast.Sub, ast.Mult, ast.Div)
    if not all(isinstance(item, allowed) for item in ast.walk(node)):
        raise ValueError(text)
    return float(eval(compile(node, "<turn-param>", "eval"), {"__builtins__": {}}, {}))


def parse_profiles(header: Path) -> list[dict]:
    text = header.read_text(encoding="utf-8")
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    text = re.sub(r"//.*", "", text)
    table_pattern = re.compile(r"const static t_turn_param_table\s+(\w+)\s*=\s*\{([^}]+)\};")
    tables = {}
    for name, body in table_pattern.findall(text):
        values = [item.strip() for item in body.split(",")]
        tables[name] = {
            "velocity_m_s": arithmetic_value(values[0]), "r_min_mm": arithmetic_value(values[1]),
            "degree": arithmetic_value(values[4]),
        }
    gain_pattern = re.compile(r"const static t_ff_gain\s+(\w+)\s*=\s*\{([^}]+)\};")
    gains = {}
    for name, body in gain_pattern.findall(text):
        values = [arithmetic_value(item.strip()) for item in body.split(",")]
        gains[name] = {f"ff_{field}": value for field, value in zip(FF_NAMES, values)}
    param_pattern = re.compile(r"const static t_param\s+(\w+)\s*=\s*\{&(\w+),[^}]*,&(\w+)\};")
    motion_names = {
        "param_R90": "long_r90", "param_L90": "long_l90",
        "param_R180": "long_r180", "param_L180": "long_l180",
        "param_RV90": "r_v90", "param_LV90": "l_v90",
        "param_inR45": "in_r45", "param_inL45": "in_l45",
        "param_outR45": "out_r45", "param_outL45": "out_l45",
        "param_inR135": "in_r135", "param_inL135": "in_l135",
        "param_outR135": "out_r135", "param_outL135": "out_l135",
    }
    profiles = []
    for param, table, gain in param_pattern.findall(text):
        param_base = re.sub(r"_\d+$", "", param)
        if param_base in motion_names and table in tables and gain in gains:
            profiles.append({
                "motion_name": motion_names[param_base], "gain_name": gain,
                "feedforward": gains[gain], **tables[table],
            })
    return profiles


def load_turn_table() -> np.ndarray:
    text = (ROOT / "Params" / "turn_table.h").read_text(encoding="utf-8")
    body = re.search(r"accel_table\[1001\]\s*=\s*\{(.*?)\};", text, re.S).group(1)
    return np.asarray([float(item) for item in body.split(",") if item.strip()])


def project_profiles(header: Path, output: Path, references: dict[str, dict]) -> None:
    table = load_turn_table()
    rows = []
    for profile in parse_profiles(header):
        direction = "left" if profile["degree"] > 0 else "right"
        reference = references[direction]
        ff = profile["feedforward"]
        omega_max = abs(profile["velocity_m_s"] / (profile["r_min_mm"] / 1000.0))
        duration = abs(math.radians(profile["degree"]) / (ACCEL_INTEGRAL * omega_max) * 1000.0)
        times = np.arange(math.ceil(duration))
        positions = np.clip(times * 1000.0 / duration, 0.0, 1000.0)
        omega = omega_max * np.interp(positions, np.arange(1001), table)
        alpha = np.gradient(omega, PERIOD_S)
        jerk = np.gradient(alpha, PERIOD_S)
        decel = omega * alpha < 0.0
        jerk_ff = np.clip(float(ff["ff_om_jerk"]) * jerk, -JERK_LIMIT_V, JERK_LIMIT_V)
        angular_ff = (float(ff["ff_om_velo"]) * omega +
                      np.where(decel, float(ff["ff_om_decel"]), float(ff["ff_om_accel"])) * alpha +
                      float(ff["ff_om_bias"]) + jerk_ff)
        rows.append({
            **{key: value for key, value in profile.items() if key != "feedforward"},
            "duration_ms": duration, "sample_count": len(times),
            "omega_max_rad_s": omega_max, "alpha_max_rad_s2": float(np.max(alpha)),
            "alpha_min_rad_s2": float(np.min(alpha)), "jerk_abs_max_rad_s3": float(np.max(np.abs(jerk))),
            "predicted_om_ff_abs_max_v": float(np.max(np.abs(angular_ff))),
            "jerk_clipped_pct": float(100.0 * np.mean(np.abs(float(ff["ff_om_jerk"]) * jerk) >= JERK_LIMIT_V)),
            "source_turn90": reference["motion_name"],
            "om_velo": float(ff["ff_om_velo"]), "om_accel": float(ff["ff_om_accel"]),
            "om_decel": float(ff["ff_om_decel"]), "om_bias": float(ff["ff_om_bias"]),
            "om_jerk": float(ff["ff_om_jerk"]),
        })
    frame = pd.DataFrame(rows).sort_values("motion_name")
    frame.to_csv(output / "profile_projection.csv", index=False)
    fig, axes = plt.subplots(2, 1, figsize=(13, 9))
    axes[0].bar(frame["motion_name"], frame["omega_max_rad_s"], label="omega max")
    axes[0].set_ylabel("omega max [rad/s]")
    axes[1].bar(frame["motion_name"], frame["predicted_om_ff_abs_max_v"], label="predicted OM FF")
    axes[1].axhline(JERK_LIMIT_V, color="gray", linestyle=":", label="jerk term clamp reference")
    axes[1].set_ylabel("predicted OM FF peak [V]")
    for axis in axes:
        axis.tick_params(axis="x", rotation=45)
        axis.grid(axis="y", alpha=0.2)
    axes[1].legend()
    fig.tight_layout()
    fig.savefig(output / "profile_projection.png", dpi=160)
    plt.close(fig)


def main() -> None:
    args = parse_args()
    speed_output = args.output_root / str(args.speed)
    speed_output.mkdir(parents=True, exist_ok=True)
    summaries = {}
    for motion in args.motion:
        group = select_latest_group(args.speed, motion)
        if not group:
            print(f"SKIP {motion}: no matching logs")
            continue
        summary = analyze_group(group, speed_output / motion)
        summaries[motion] = summary
        print(f"WROTE {motion}: {len(group)} logs, {summary['turn_count']} turns")
    if args.project_header:
        references = {
            "right": summaries["long_r90"],
            "left": summaries["long_l90"],
        }
        project_profiles(args.project_header, speed_output, references)
        print(f"WROTE profile projection: {speed_output}")


if __name__ == "__main__":
    main()
