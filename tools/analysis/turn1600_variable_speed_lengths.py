"""LEGACY: calculate turn lengths with a modelled velocity/slip trajectory.

New tuning should use lzero_turn_lengths.py with Lstart=Lend=0 marker videos.
This module remains because regression tests and legacy comparisons import it.
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from turn_ff_report import select_latest_group, turn_segments  # noqa: E402
from turn_simulator import PRESETS, simulate_turn  # noqa: E402


ACCEL_PROFILE_INTEGRAL = 0.7043
K_BY_SIDE = {"R": 239.34375, "L": 260.464286}
# Fitted to the two long-R180 turns in PXL_20260722_175611299.mp4:
# the measured grid displacement is 90 mm while encoder/gyro odometry reports
# about 99.4 mm.  With measured u, omega, and beta replayed, this gain makes
# r_min=48 mm finish at the observed 90 mm lane.
VIDEO_CALIBRATED_BETA_SQ_GAIN = 3.4150269835847427
# Refit with the resultant-speed cos/sin model.  R/L gains required to make
# the latest long-180 replay finish at 90 mm are 3.091784 and 3.158871;
# their mean is used as one direction-independent contact-slip coefficient.
GROUND_TRANSLATION_SLIP_BETA_SQ_GAIN = 3.125327524776992

TURN_CASES = (
    ("long_turn90", "R", "long_r90", False),
    ("long_turn90", "L", "long_l90", False),
    ("long_turn180", "R", "long_r180", False),
    ("long_turn180", "L", "long_l180", False),
    ("turn_in45", "R", "in_r45", False),
    ("turn_in45", "L", "in_l45", False),
    ("turn_out45", "R", "out_r45", False),
    ("turn_out45", "L", "out_l45", False),
    ("turn_in135", "R", "in_r135", False),
    ("turn_in135", "L", "in_l135", False),
    ("turn_out135", "R", "out_r135", False),
    ("turn_out135", "L", "out_l135", False),
    ("turn_v90", "R", "r_v90", False),
    ("turn_v90", "L", "l_v90", False),
)


def repair_terminal_beta_reset(
    beta: np.ndarray,
    velocity: np.ndarray,
    yaw_rate: np.ndarray,
    slip_k: float,
) -> np.ndarray:
    """Replace a firmware-forced terminal beta zero by one continuous step.

    Some short-turn logs clear ``ego.turn_slip_theta`` on the final command
    sample.  That zero is a state reset, not a physical disappearance of side
    slip.  Continue the same beta dynamics for that sample; subsequent Lend
    samples are then decayed continuously by ``turn_simulator._coast``.
    """
    repaired = np.asarray(beta, dtype=float).copy()
    if (
        len(repaired) >= 2
        and abs(repaired[-1]) <= 1.0e-6
        and abs(repaired[-2]) >= 0.02
        and velocity[-1] > 0.0
    ):
        beta_dot = -slip_k * repaired[-2] / velocity[-1] - yaw_rate[-1]
        repaired[-1] = repaired[-2] + beta_dot * 0.001
    return repaired


def expected_segment_ms(settings: dict) -> int:
    params = settings["parameters"]
    omega_max = abs(float(params["velo"]) / (float(params["r_min"]) / 1000.0))
    duration_ms = (
        abs(math.radians(float(params["degree"])))
        / (ACCEL_PROFILE_INTEGRAL * omega_max)
        * 1000.0
    )
    return math.ceil(duration_ms) + 2


def mean_motion_profiles(
    log_motion: str, points: int = 201,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float, list[str], int]:
    velocity_profiles: list[np.ndarray] = []
    yaw_profiles: list[np.ndarray] = []
    beta_profiles: list[np.ndarray] = []
    yaw_angles: list[float] = []
    sources: list[str] = []
    side = next(side for _, side, name, _ in TURN_CASES if name == log_motion)
    slip_k = K_BY_SIDE[side]
    for csv_path, settings in select_latest_group(1600, log_motion):
        frame = pd.read_csv(csv_path)
        direction = 1.0 if float(settings["parameters"]["degree"]) > 0.0 else -1.0
        ideal_omega = direction * frame["ideal.rad_velo"].to_numpy(float)
        expected = expected_segment_ms(settings)
        tolerance = max(3, math.ceil(expected * 0.06))
        for start, end in turn_segments(ideal_omega):
            if abs((end - start + 1) - expected) > tolerance:
                continue
            velocity = frame["ego.velo"].to_numpy(float)[start:end + 1]
            yaw = direction * frame["ego.rad_velo"].to_numpy(float)[start:end + 1]
            beta = direction * frame["ego.turn_slip_theta"].to_numpy(float)[start:end + 1]
            beta = repair_terminal_beta_reset(beta, velocity, yaw, slip_k)
            phase = np.linspace(0.0, 1.0, points)
            source_phase = np.linspace(0.0, 1.0, len(velocity))
            velocity_profiles.append(np.interp(
                phase, source_phase, velocity,
            ))
            yaw_profiles.append(np.interp(
                phase, source_phase, yaw,
            ))
            beta_profiles.append(np.interp(
                phase, source_phase, beta,
            ))
            yaw_angles.append(float(np.sum(yaw) * 0.001))
            sources.append(csv_path.name)
    if not velocity_profiles:
        raise RuntimeError(f"no valid 1600 mm/s motion profile for {log_motion}")
    return (
        np.mean(velocity_profiles, axis=0),
        np.mean(yaw_profiles, axis=0),
        np.mean(beta_profiles, axis=0),
        float(np.mean(yaw_angles)),
        sorted(set(sources)),
        len(velocity_profiles),
    )


def mean_velocity_profile(log_motion: str, points: int = 201) -> tuple[np.ndarray, list[str], int]:
    velocity, _, _, _, sources, turns = mean_motion_profiles(log_motion, points)
    return velocity, sources, turns


def calculate() -> pd.DataFrame:
    preset = PRESETS["turn1600"]
    physical_dynamics = replace(
        preset.dynamics,
        lateral_velocity_position_scale=1.0,
        longitudinal_slip_projection=True,
        lateral_velocity_uses_tangent=False,
        ground_slip_beta_sq_gain=GROUND_TRANSLATION_SLIP_BETA_SQ_GAIN,
    )
    no_position_slip_dynamics = replace(
        preset.dynamics,
        lateral_velocity_position_scale=0.0,
        longitudinal_slip_projection=False,
    )
    rows = []
    cache = {}
    simulated_results = {}
    for motion, side, log_motion, proxy in TURN_CASES:
        profile, yaw_profile, beta_profile, yaw_angle, sources, turns = cache.setdefault(
            log_motion, mean_motion_profiles(log_motion)
        )
        motor = replace(preset.motor, slip_k=K_BY_SIDE[side])
        radius = preset.radii_mm[motion]
        constant = simulate_turn(
            motion, preset.velocity, motor, radius,
            preset.kp, preset.ki, preset.alpha, physical_dynamics,
        )
        ideal_reference = simulate_turn(
            motion, preset.velocity, motor, radius,
            preset.kp, preset.ki, preset.alpha, no_position_slip_dynamics,
            velocity_profile=profile,
            yaw_rate_profile=yaw_profile,
            yaw_angle_rad=yaw_angle,
        )
        variable = simulate_turn(
            motion, preset.velocity, motor, radius,
            preset.kp, preset.ki, preset.alpha, physical_dynamics,
            velocity_profile=profile,
            yaw_rate_profile=yaw_profile,
            beta_profile=beta_profile,
            yaw_angle_rad=yaw_angle,
        )
        simulated_results[(motion, side)] = variable
        body_samples = math.trunc(variable.duration_s / 0.001)
        body_beta = variable.beta_rad[:body_samples]
        contact_scale = np.maximum(
            0.0,
            1.0 - GROUND_TRANSLATION_SLIP_BETA_SQ_GAIN * body_beta ** 2,
        )
        translation_scale = contact_scale * np.cos(body_beta)
        predicted_lateral, predicted_forward, _, _ = _full_trajectory(
            variable, side, variable.start_length_mm, variable.end_length_mm,
            ideal=False,
        )
        ideal_lateral, ideal_forward, _, _ = _full_trajectory(
            ideal_reference,
            side,
            ideal_reference.start_length_mm,
            ideal_reference.end_length_mm,
            ideal=False,
        )
        target_lateral, target_forward = _target_endpoint(motion, side)
        rows.append({
            "motion": motion,
            "side": side,
            "radius_mm": radius,
            "slip_k": K_BY_SIDE[side],
            "velocity_source": log_motion,
            "velocity_proxy": proxy,
            "source_turns": turns,
            "source_files": ";".join(sources),
            "velocity_mean_m_s": float(np.mean(profile)),
            "velocity_min_m_s": float(np.min(profile)),
            "velocity_max_m_s": float(np.max(profile)),
            "translation_speed_mean_m_s": float(np.mean(
                variable.velocity_m_s[:body_samples] * translation_scale
            )),
            "translation_loss_mean_pct": float(100.0 * np.mean(1.0 - translation_scale)),
            "translation_loss_peak_pct": float(100.0 * np.max(1.0 - translation_scale)),
            "final_angle_rad": variable.final_angle_rad,
            "constant_start_mm": constant.start_length_mm,
            "constant_end_mm": constant.end_length_mm,
            "variable_start_mm": variable.start_length_mm,
            "variable_end_mm": variable.end_length_mm,
            "start_change_mm": variable.start_length_mm - constant.start_length_mm,
            "end_change_mm": variable.end_length_mm - constant.end_length_mm,
            "predicted_end_lateral_mm": float(predicted_lateral[-1]),
            "predicted_end_forward_mm": float(predicted_forward[-1]),
            "ideal_start_mm": ideal_reference.start_length_mm,
            "ideal_end_mm": ideal_reference.end_length_mm,
            "ideal_end_lateral_mm": float(ideal_lateral[-1]),
            "ideal_end_forward_mm": float(ideal_forward[-1]),
            "target_end_lateral_mm": target_lateral,
            "target_end_forward_mm": target_forward,
            "prediction_target_error_mm": float(math.hypot(
                predicted_lateral[-1] - target_lateral,
                predicted_forward[-1] - target_forward,
            )),
            "prediction_ideal_separation_mm": float(math.hypot(
                predicted_lateral[-1] - ideal_lateral[-1],
                predicted_forward[-1] - ideal_forward[-1],
            )),
        })
    frame = pd.DataFrame(rows)
    frame["adopted_start_mm"] = frame.groupby("motion")["variable_start_mm"].transform("mean")
    frame["adopted_end_mm"] = frame.groupby("motion")["variable_end_mm"].transform("mean")

    adopted_lateral = []
    adopted_forward = []
    adopted_error = []
    for row in frame.itertuples():
        lateral_path, forward_path, _, _ = _full_trajectory(
            simulated_results[(row.motion, row.side)],
            row.side,
            row.adopted_start_mm,
            row.adopted_end_mm,
            ideal=False,
        )
        lateral = float(lateral_path[-1])
        forward = float(forward_path[-1])
        adopted_lateral.append(lateral)
        adopted_forward.append(forward)
        adopted_error.append(math.hypot(
            lateral - row.target_end_lateral_mm,
            forward - row.target_end_forward_mm,
        ))
    frame["adopted_end_lateral_mm"] = adopted_lateral
    frame["adopted_end_forward_mm"] = adopted_forward
    frame["adopted_target_error_mm"] = adopted_error
    return frame


def write_report(frame: pd.DataFrame, path: Path) -> None:
    lines = [
        "# 1600 mm/s variable-speed slip length model", "",
        "- right slip k: 239.34375",
        "- left slip k: 260.464286",
        "- resultant-speed model: logged `ego.velo` is the ground-speed magnitude",
        "- body-forward component: `v_forward = ego.velo cos(beta)`",
        "- perpendicular lateral component: `v_lateral = ego.velo sin(beta)`",
        f"- contact translation scale: `1 - {GROUND_TRANSLATION_SLIP_BETA_SQ_GAIN:.6f} beta^2`",
        "- the same contact scale is applied to both components so that the slip angle remains beta",
        "- a firmware-forced terminal beta zero is replaced by one continuous beta-dynamics step",
        "- during Lend, beta decays continuously with `beta_dot = -k beta / ego.velo - omega`",
        "- the resultant ground-velocity direction is displaced from body heading by beta",
        "- measured `ego.velo`, `ego.rad_velo`, and logged beta profiles are replayed",
        "- velocity input: direction/motion-specific mean `ego.velo` profile", "",
        "| motion | side | r_min [mm] | mean ego.velo [m/s] | forward loss mean [%] | forward loss peak [%] | Lstart [mm] | Lend [mm] | predicted end (lat, fwd) [mm] | target end (lat, fwd) [mm] | target error [mm] |",
        "| --- | :---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in frame.itertuples():
        lines.append(
            f"| {row.motion} | {row.side} | {row.radius_mm:.2f} | {row.velocity_mean_m_s:.4f} | "
            f"{row.translation_loss_mean_pct:.3f} | {row.translation_loss_peak_pct:.3f} | "
            f"{row.adopted_start_mm:.3f} | {row.adopted_end_mm:.3f} | "
            f"({row.adopted_end_lateral_mm:.2f}, {row.adopted_end_forward_mm:.2f}) | "
            f"({row.target_end_lateral_mm:.2f}, {row.target_end_forward_mm:.2f}) | "
            f"{row.adopted_target_error_mm:.2f} |"
        )
    lines.extend((
        "", "## Trajectories", "",
        "![Trajectory overview](trajectories/overview.png)", "",
    ))
    for motion in dict.fromkeys(frame["motion"]):
        lines.append(f"- [{motion}](trajectories/{motion}.png)")
    lines.extend(("", "## Velocity components", ""))
    for motion in dict.fromkeys(frame["motion"]):
        lines.append(f"- [{motion}](velocity_profiles/{motion}.png)")
    lines.extend((
        "", "## Long-180 model comparison", "",
        "![Long-180 model comparison](long_turn180_model_comparison.png)", "",
    ))
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _trajectory_xy(result, side: str, *, ideal: bool = False) -> tuple[np.ndarray, np.ndarray]:
    if ideal:
        forward = np.asarray(result.ideal_x_mm) - result.ideal_x_mm[0]
        lateral = np.asarray(result.ideal_y_mm) - result.ideal_y_mm[0]
    else:
        forward = np.asarray(result.x_mm) - result.x_mm[0]
        lateral = np.asarray(result.y_mm) - result.y_mm[0]
    # simulate_turn generates the left-turn geometry. Mirror its local lateral
    # coordinate for a right turn while retaining the same forward axis.
    if side == "R":
        lateral = -lateral
    return lateral, forward


def _target_endpoint(motion: str, side: str) -> tuple[float, float]:
    lateral_magnitude, forward = {
        "long_turn90": (90.0, 90.0),
        "long_turn180": (90.0, 0.0),
        "turn_in45": (45.0, 90.0),
        "turn_out45": (90.0, 45.0),
        "turn_in135": (90.0, 45.0),
        "turn_out135": (90.0, -45.0),
        "turn_v90": (90.0, 0.0),
    }[motion]
    return (-lateral_magnitude if side == "R" else lateral_magnitude), forward


def _unit_tangent(lateral: np.ndarray, forward: np.ndarray, *, start: bool) -> np.ndarray:
    delta = np.column_stack((np.diff(lateral), np.diff(forward)))
    indices = range(len(delta)) if start else range(len(delta) - 1, -1, -1)
    for index in indices:
        norm = float(np.hypot(delta[index, 0], delta[index, 1]))
        if norm > 1.0e-9:
            return delta[index] / norm
    raise RuntimeError("trajectory has no non-zero tangent")


def _full_trajectory(
    result,
    side: str,
    start_length_mm: float,
    end_length_mm: float,
    *,
    ideal: bool,
) -> tuple[np.ndarray, np.ndarray, int, int]:
    lateral, forward = _trajectory_xy(result, side, ideal=ideal)
    # Some MATLAB-compatible geometry functions simulate part of Lend so yaw
    # and beta can decay before solving the remaining straight. Preserve that
    # coast instead of extending the last (still slipping) body tangent.
    turn_points = math.trunc(result.duration_s / 0.001) + 2
    heading_offset = (
        math.pi / 4.0
        if result.motion in {"turn_v90", "long_turn_v90"}
        else 0.0
    )
    start_heading = (
        math.pi / 4.0
        if result.motion in {"turn_out45", "turn_out135"}
        else 0.0
    ) + heading_offset
    end_heading = result.final_angle_rad + heading_offset
    side_sign = -1.0 if side == "R" else 1.0
    start_tangent = np.array([
        side_sign * math.sin(start_heading), math.cos(start_heading)
    ])
    end_tangent = np.array([
        side_sign * math.sin(end_heading), math.cos(end_heading)
    ])
    turn_entry = start_tangent * start_length_mm
    body_lateral = lateral - lateral[0] + turn_entry[0]
    body_forward = forward - forward[0] + turn_entry[1]
    turn_exit = min(turn_points - 1, len(body_lateral) - 1)
    available_coast_steps = max(0, len(body_lateral) - 1 - turn_exit)
    requested_coast_steps = max(0.0, end_length_mm / result.velocity)
    used_coast_steps = min(requested_coast_steps, float(available_coast_steps))
    whole_steps = int(math.floor(used_coast_steps + 1.0e-12))
    fraction = used_coast_steps - whole_steps

    last_index = turn_exit + whole_steps
    selected_lateral = body_lateral[:last_index + 1]
    selected_forward = body_forward[:last_index + 1]
    if fraction > 1.0e-9 and last_index + 1 < len(body_lateral):
        selected_lateral = np.r_[
            selected_lateral,
            body_lateral[last_index]
            + fraction * (body_lateral[last_index + 1] - body_lateral[last_index]),
        ]
        selected_forward = np.r_[
            selected_forward,
            body_forward[last_index]
            + fraction * (body_forward[last_index + 1] - body_forward[last_index]),
        ]

    simulated_coast_mm = available_coast_steps * result.velocity
    remaining_end_mm = max(0.0, end_length_mm - simulated_coast_mm)
    if remaining_end_mm > 1.0e-9:
        selected_lateral = np.r_[
            selected_lateral, selected_lateral[-1] + end_tangent[0] * remaining_end_mm,
        ]
        selected_forward = np.r_[
            selected_forward, selected_forward[-1] + end_tangent[1] * remaining_end_mm,
        ]

    full_lateral = np.r_[0.0, turn_entry[0], selected_lateral[1:]]
    full_forward = np.r_[0.0, turn_entry[1], selected_forward[1:]]
    entry_index = 1
    exit_index = turn_points
    return full_lateral, full_forward, entry_index, exit_index


def plot_trajectories(frame: pd.DataFrame, output: Path) -> None:
    preset = PRESETS["turn1600"]
    physical_dynamics = replace(
        preset.dynamics,
        lateral_velocity_position_scale=1.0,
        longitudinal_slip_projection=True,
        lateral_velocity_uses_tangent=False,
        ground_slip_beta_sq_gain=GROUND_TRANSLATION_SLIP_BETA_SQ_GAIN,
    )
    no_position_slip_dynamics = replace(
        preset.dynamics,
        lateral_velocity_position_scale=0.0,
        longitudinal_slip_projection=False,
    )
    profile_cache = {}
    results = {}
    for row in frame.itertuples():
        profile, yaw_profile, beta_profile, yaw_angle, _, _ = profile_cache.setdefault(
            row.velocity_source, mean_motion_profiles(row.velocity_source)
        )
        result = simulate_turn(
            row.motion,
            preset.velocity,
            replace(preset.motor, slip_k=row.slip_k),
            preset.radii_mm[row.motion],
            preset.kp,
            preset.ki,
            preset.alpha,
            physical_dynamics,
            velocity_profile=profile,
            yaw_rate_profile=yaw_profile,
            beta_profile=beta_profile,
            yaw_angle_rad=yaw_angle,
        )
        ideal_reference = simulate_turn(
            row.motion,
            preset.velocity,
            replace(preset.motor, slip_k=row.slip_k),
            preset.radii_mm[row.motion],
            preset.kp,
            preset.ki,
            preset.alpha,
            no_position_slip_dynamics,
            velocity_profile=profile,
            yaw_rate_profile=yaw_profile,
            yaw_angle_rad=yaw_angle,
        )
        results[(row.motion, row.side)] = (result, ideal_reference)

    trajectory_dir = output / "trajectories"
    trajectory_dir.mkdir(parents=True, exist_ok=True)
    motions = list(dict.fromkeys(frame["motion"]))
    for motion in motions:
        figure, axes = plt.subplots(
            1, 2, figsize=(10.5, 5.6), sharex=True, sharey=True,
            constrained_layout=True,
        )
        for axis, side in zip(axes, ("R", "L")):
            row = frame[(frame["motion"] == motion) & (frame["side"] == side)].iloc[0]
            result, ideal_reference = results[(motion, side)]
            ideal_x, ideal_y, ideal_entry, ideal_exit = _full_trajectory(
                ideal_reference,
                side,
                ideal_reference.start_length_mm,
                ideal_reference.end_length_mm,
                ideal=False,
            )
            predicted_x, predicted_y, predicted_entry, predicted_exit = _full_trajectory(
                result, side, row.adopted_start_mm, row.adopted_end_mm, ideal=False
            )
            target_x, target_y = _target_endpoint(motion, side)
            axis.plot(ideal_x, ideal_y, "--", color="0.25", linewidth=2.0,
                      label="same-profile, no position slip")
            axis.plot(predicted_x, predicted_y, color="#1874c9", linewidth=2.2, label="predicted")
            axis.scatter([0.0], [0.0], marker="o", color="0.1", s=28, zorder=4, label="start (0, 0)")
            axis.scatter([predicted_x[predicted_entry], predicted_x[predicted_exit]],
                         [predicted_y[predicted_entry], predicted_y[predicted_exit]],
                         marker="s", color="#1874c9", s=24, zorder=4, label="turn entry/exit")
            axis.scatter([ideal_x[-1]], [ideal_y[-1]], marker="x", color="0.25", s=45, zorder=4)
            axis.scatter([predicted_x[-1]], [predicted_y[-1]], marker="o",
                         color="#1874c9", s=38, zorder=4)
            axis.scatter([target_x], [target_y], marker="*", color="#d38b00", s=95,
                         zorder=5, label="ideal target")
            target_error = math.hypot(
                predicted_x[-1] - target_x, predicted_y[-1] - target_y
            )
            text_y = 0.97 if predicted_y[-1] < 0.0 else 0.03
            axis.text(
                0.03,
                text_y,
                f"start = (0.0, 0.0) mm\n"
                f"pred end = ({predicted_x[-1]:.1f}, {predicted_y[-1]:.1f}) mm\n"
                f"target = ({target_x:.1f}, {target_y:.1f}) mm\n"
                f"target error = {target_error:.1f} mm",
                transform=axis.transAxes,
                ha="left",
                va="top" if text_y > 0.5 else "bottom",
                bbox={"boxstyle": "round,pad=0.3", "facecolor": "white", "alpha": 0.86,
                      "edgecolor": "0.75"},
                zorder=5,
            )
            axis.set_title(
                f"{side}: k={row.slip_k:.2f}, mean u={row.velocity_mean_m_s:.3f} m/s\n"
                f"loss={row.translation_loss_mean_pct:.2f}% mean/"
                f"{row.translation_loss_peak_pct:.2f}% peak\n"
                f"shared Lstart={row.adopted_start_mm:.2f}, Lend={row.adopted_end_mm:.2f} mm"
            )
            axis.set_xlabel("lateral [mm]")
            axis.grid(True, alpha=0.28)
            axis.set_aspect("equal", adjustable="box")
        axes[0].set_ylabel("forward [mm]")
        axes[0].legend(loc="best")
        figure.suptitle(f"1600 mm/s {motion}: resultant speed + contact-slip prediction")
        figure.savefig(trajectory_dir / f"{motion}.png", dpi=180)
        plt.close(figure)

    figure, axes = plt.subplots(4, 2, figsize=(12, 19))
    for axis, motion in zip(axes.flat, motions):
        for side, color in (("R", "#d05a4e"), ("L", "#1874c9")):
            result, ideal_reference = results[(motion, side)]
            row = frame[(frame["motion"] == motion) & (frame["side"] == side)].iloc[0]
            ideal_x, ideal_y, _, _ = _full_trajectory(
                ideal_reference,
                side,
                ideal_reference.start_length_mm,
                ideal_reference.end_length_mm,
                ideal=False,
            )
            predicted_x, predicted_y, _, _ = _full_trajectory(
                result, side, row.adopted_start_mm, row.adopted_end_mm, ideal=False
            )
            target_x, target_y = _target_endpoint(motion, side)
            axis.plot(ideal_x, ideal_y, "--", color="0.4", linewidth=1.2,
                      label=f"{side} no position slip")
            axis.plot(predicted_x, predicted_y, color=color, linewidth=1.8,
                      label=f"{side} predicted")
            axis.scatter([target_x], [target_y], marker="*", color="#d38b00", s=35,
                         zorder=4, label=f"{side} target")
        axis.set_title(motion)
        axis.set_xlabel("lateral [mm]")
        axis.set_ylabel("forward [mm]")
        axis.grid(True, alpha=0.25)
        axis.set_aspect("equal", adjustable="datalim")
    for axis in axes.flat[len(motions):]:
        axis.axis("off")
    handles, labels = axes.flat[0].get_legend_handles_labels()
    figure.legend(handles, labels, loc="lower right")
    figure.suptitle("1600 mm/s resultant speed + contact-slip trajectories")
    figure.tight_layout(rect=(0, 0.025, 1, 0.98))
    figure.savefig(trajectory_dir / "overview.png", dpi=180)
    plt.close(figure)

    velocity_dir = output / "velocity_profiles"
    velocity_dir.mkdir(parents=True, exist_ok=True)
    for motion in motions:
        figure, axes = plt.subplots(
            1, 2, figsize=(11, 4.8), sharex=True, sharey=True,
            constrained_layout=True,
        )
        for axis, side in zip(axes, ("R", "L")):
            result, _ = results[(motion, side)]
            samples = min(
                math.trunc(result.duration_s / 0.001),
                len(result.time_s), len(result.velocity_m_s), len(result.beta_rad),
            )
            time_ms = result.time_s[:samples] * 1000.0
            encoder_u = result.velocity_m_s[:samples]
            contact_scale = np.maximum(
                0.0,
                1.0 - GROUND_TRANSLATION_SLIP_BETA_SQ_GAIN
                * result.beta_rad[:samples] ** 2,
            )
            forward_u = encoder_u * contact_scale * np.cos(result.beta_rad[:samples])
            lateral_u = encoder_u * contact_scale * np.sin(result.beta_rad[:samples])
            if side == "R":
                lateral_u = -lateral_u
            axis.plot(time_ms, encoder_u, color="0.35", linestyle="--", linewidth=1.6,
                      label="ego.velo (resultant)")
            axis.plot(time_ms, forward_u, color="#1874c9", linewidth=2.0,
                      label="ground forward velocity")
            axis.plot(time_ms, lateral_u, color="#d05a4e", linewidth=2.0,
                      label="ground lateral velocity")
            axis.axhline(0.0, color="0.65", linewidth=0.8)
            axis.set_title(
                f"{side}: forward loss mean "
                f"{100.0 * np.mean(1.0 - contact_scale * np.cos(result.beta_rad[:samples])):.2f}%"
            )
            axis.set_xlabel("time [ms]")
            axis.grid(True, alpha=0.25)
        axes[0].set_ylabel("velocity [m/s]")
        axes[0].legend(loc="best")
        figure.suptitle(f"1600 mm/s {motion}: resultant command and contact velocities")
        figure.savefig(velocity_dir / f"{motion}.png", dpi=180)
        plt.close(figure)

    comparison_specs = (
        ("no position slip", 0.0, False, False, 0.0, "0.35", "--"),
        ("projection only", 0.0, True, False, 0.0, "#2c9c69", "-"),
        ("lateral beta", 1.0, False, False, 0.0, "#d05a4e", "-"),
        ("body u + lateral u tan(beta)", 1.0, False, True, 0.0, "#d38b00", "-"),
        ("resultant ego.velo: cos/sin", 1.0, True, False, 0.0, "#8f65bd", "-"),
        ("resultant + fitted contact slip", 1.0, True, False,
         GROUND_TRANSLATION_SLIP_BETA_SQ_GAIN, "#111111", "-"),
        ("video calibrated", 1.0, True, False, VIDEO_CALIBRATED_BETA_SQ_GAIN, "#1874c9", "-"),
    )
    comparison_rows = []
    figure, axes = plt.subplots(
        1, 2, figsize=(11, 5.2), sharex=True, sharey=True,
        constrained_layout=True,
    )
    for axis, side in zip(axes, ("R", "L")):
        row = frame[(frame["motion"] == "long_turn180") & (frame["side"] == side)].iloc[0]
        profile, yaw_profile, beta_profile, yaw_angle, _, _ = profile_cache[row.velocity_source]
        for label, lateral_scale, projection, tangent, ground_gain, color, linestyle in comparison_specs:
            model = replace(
                preset.dynamics,
                lateral_velocity_position_scale=lateral_scale,
                longitudinal_slip_projection=projection,
                lateral_velocity_uses_tangent=tangent,
                ground_slip_beta_sq_gain=ground_gain,
            )
            result = simulate_turn(
                "long_turn180", preset.velocity,
                replace(preset.motor, slip_k=row.slip_k),
                preset.radii_mm["long_turn180"], preset.kp, preset.ki,
                preset.alpha, model, velocity_profile=profile,
                yaw_rate_profile=yaw_profile,
                beta_profile=beta_profile,
                yaw_angle_rad=yaw_angle,
            )
            lateral, forward, _, _ = _full_trajectory(
                result, side, result.start_length_mm, result.end_length_mm,
                ideal=False,
            )
            axis.plot(lateral, forward, color=color, linestyle=linestyle,
                      linewidth=2.0, label=label)
            axis.scatter([lateral[-1]], [forward[-1]], color=color, s=24, zorder=4)
            comparison_rows.append({
                "side": side,
                "model": label,
                "start_length_mm": result.start_length_mm,
                "end_length_mm": result.end_length_mm,
                "end_lateral_mm": float(lateral[-1]),
                "end_forward_mm": float(forward[-1]),
            })
        target_lateral, target_forward = _target_endpoint("long_turn180", side)
        axis.scatter([target_lateral], [target_forward], marker="*", color="#d38b00",
                     s=90, zorder=5, label="maze target")
        axis.set_title(side)
        axis.set_xlabel("lateral [mm]")
        axis.grid(True, alpha=0.25)
        axis.set_aspect("equal", adjustable="box")
    axes[0].set_ylabel("forward [mm]")
    axes[0].legend(loc="best")
    figure.suptitle("1600 mm/s long_turn180: positional slip-model comparison")
    figure.savefig(output / "long_turn180_model_comparison.png", dpi=180)
    plt.close(figure)
    pd.DataFrame(comparison_rows).to_csv(
        output / "long_turn180_model_comparison.csv", index=False
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "tools" / "turn_analysis" / "1600" / "resultant_speed_ground_slip_lengths",
    )
    args = parser.parse_args()
    frame = calculate()
    args.output.mkdir(parents=True, exist_ok=True)
    frame.to_csv(args.output / "lengths.csv", index=False)
    adopted = frame.groupby("motion", sort=False).agg(
        radius_mm=("radius_mm", "first"),
        lstart_mm=("adopted_start_mm", "first"),
        lend_mm=("adopted_end_mm", "first"),
        right_start_mm=("variable_start_mm", "first"),
        left_start_mm=("variable_start_mm", "last"),
        right_end_mm=("variable_end_mm", "first"),
        left_end_mm=("variable_end_mm", "last"),
        max_target_error_mm=("adopted_target_error_mm", "max"),
    ).reset_index()
    adopted.to_csv(args.output / "adopted_lengths.csv", index=False)
    write_report(frame, args.output / "report.md")
    plot_trajectories(frame, args.output)
    print(frame[[
        "motion", "side", "slip_k", "velocity_mean_m_s",
        "translation_loss_mean_pct", "translation_loss_peak_pct",
        "adopted_start_mm", "adopted_end_mm", "adopted_target_error_mm",
        "start_change_mm", "end_change_mm", "velocity_proxy",
    ]].to_string(index=False))


if __name__ == "__main__":
    main()
