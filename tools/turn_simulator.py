"""Micromouse turn simulator ported from ``matlab_turn2``.

The model intentionally keeps the MATLAB implementation's 1 ms update order,
units, acceleration table interpolation, PI speed loop, and turn-specific
geometry.  Longitudinal velocity is expressed in m/s; because the integration
period is 1 ms, adding it once to a position also gives the displacement in mm.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np


DT = 0.001
SQRT2_MATLAB = 1.414


@dataclass(frozen=True)
class MotorParameters:
    motor_r: float
    motor_ktr: float
    wheel_r: float
    gear_ratio: float
    mass: float
    slip_k: float


@dataclass(frozen=True)
class DynamicsModel:
    """Options separating the original MATLAB model from measured firmware behavior."""

    name: str
    closed_loop_speed: bool = False
    nonlinear_slip: bool = True
    lateral_velocity_position_scale: float = 1.0
    longitudinal_slip_projection: bool = False
    lateral_velocity_uses_tangent: bool = False
    ground_slip_beta_sq_gain: float = 0.0
    yaw_gain: float = 1.0
    yaw_time_constant_s: float = 0.0
    heading_uses_commanded_yaw: bool = True
    preserve_commanded_yaw_during_coast: bool = True


MATLAB_DYNAMICS = DynamicsModel(name="matlab")

# Identified from the latest same-setting 1600 mm/s long-R/L90 logs.  The
# firmware's position estimator integrates encoder speed and gyro heading, and
# deliberately does not add horizon_velo (Module/Src/interrupt.cpp).  Its speed
# feedforward also keeps the mean speed within 0.6% of the command.  The yaw
# gain/time constant reduce normalized yaw-profile RMS from 0.671 to 0.590
# rad/s over 25 turns; they are intentionally modest to avoid overfitting.
TURN1600_DYNAMICS = DynamicsModel(
    name="turn1600_measured",
    closed_loop_speed=True,
    nonlinear_slip=False,
    lateral_velocity_position_scale=0.0,
    yaw_gain=1.0081694800637893,
    yaw_time_constant_s=0.0005,
    heading_uses_commanded_yaw=False,
    preserve_commanded_yaw_during_coast=False,
)


@dataclass(frozen=True)
class Preset:
    motor: MotorParameters
    velocity: float
    radii_mm: dict[str, float]
    kp: float = 4.0
    ki: float = 0.01
    alpha: float = 1.0
    dynamics: DynamicsModel = MATLAB_DYNAMICS
    configured_lengths_mm: dict[str, tuple[float, float]] = field(default_factory=dict)


@dataclass
class TurnResult:
    motion: str
    model_name: str
    velocity: float
    radius_mm: float
    target_angle_rad: float
    duration_s: float
    start_length_mm: float
    end_length_mm: float
    final_angle_rad: float
    time_s: np.ndarray
    x_mm: np.ndarray
    y_mm: np.ndarray
    ideal_x_mm: np.ndarray
    ideal_y_mm: np.ndarray
    yaw_rate_rad_s: np.ndarray
    yaw_accel_rad_s2: np.ndarray
    beta_rad: np.ndarray
    beta_rate_rad_s: np.ndarray
    velocity_m_s: np.ndarray
    lateral_velocity_m_s: np.ndarray
    longitudinal_accel_m_s2: np.ndarray
    lateral_accel_m_s2: np.ndarray
    centripetal_accel_m_s2: np.ndarray

    def summary(self) -> dict[str, float | str | int]:
        return {
            "motion": self.motion,
            "model": self.model_name,
            "velocity_m_s": self.velocity,
            "radius_mm": self.radius_mm,
            "target_angle_deg": math.degrees(self.target_angle_rad),
            "profile_duration_s": self.duration_s,
            "samples": int(len(self.time_s)),
            "start_length_mm": self.start_length_mm,
            "end_length_mm": self.end_length_mm,
            "final_angle_deg": math.degrees(self.final_angle_rad),
        }


PRESETS = {
    "type6": Preset(
        MotorParameters(2.5, 0.8e-3, 7.5e-3, 52 / 8, 15e-3, 200.0),
        1.6,
        {"long_turn90": 52.0, "long_turn180": 42.0},
    ),
    "type7": Preset(
        MotorParameters(4.0, 0.7e-3, 7e-3, 40 / 7, 18e-3, 250.0),
        1.4,
        {"long_turn90": 53.0, "long_turn180": 48.0},
    ),
    "type7_kai": Preset(
        MotorParameters(2.5, 0.5e-3, 7e-3, 40 / 7, 18e-3, 250.0),
        2.0,
        {
            "long_turn90": 52.0,
            "long_turn180": 50.0,
            "turn_in45": 53.0,
            "turn_out45": 60.0,
            "turn_in135": 43.0,
            "turn_out135": 41.0,
            "turn_v90": 40.0,
        },
    ),
    "type8i": Preset(
        MotorParameters(3.5, 0.8e-3, 7.5e-3, 52 / 8, 20e-3, 350.0),
        2.0,
        {
            "long_turn90": 52.0,
            "long_turn180": 47.0,
            "turn_in45": 53.0,
            "turn_out45": 55.0,
            "turn_in135": 42.0,
            "turn_out135": 41.0,
            "turn_v90": 38.0,
        },
    ),
    "search": Preset(
        MotorParameters(2.5, 0.5e-3, 7e-3, 40 / 7, 18e-3, 75.0),
        0.28,
        {"turn90": 26.0},
    ),
    "turn1600": Preset(
        MotorParameters(3.5, 0.8e-3, 7.5e-3, 52 / 8, 20e-3, 250.0),
        1.6,
        {
            "long_turn90": 52.0,
            "long_turn180": 48.0,
            "turn_in45": 55.0,
            "turn_out45": 60.0,
            # Previous r_min: in135=42.5 mm, out135=41.0 mm.
            # Tuned with the measured 1600 mm/s velocity/slip model while
            # keeping both direction-specific Lstart/Lend non-negative.
            "turn_in135": 42.0,
            "turn_out135": 39.3,
            "turn_v90": 40.0,
        },
        kp=2.0,
        ki=0.016,
        dynamics=TURN1600_DYNAMICS,
        configured_lengths_mm={
            "long_turn90": (14.69, 34.06),
            "long_turn180": (10.79, 32.40),
            "turn_in45": (25.41, 23.05),
            "turn_out45": (21.00, 20.61),
            "turn_in135": (11.60, 25.47),
            "turn_out135": (12.47, 40.99),
            "turn_v90": (4.51, 23.78),
        },
    ),
}

MOTIONS = (
    "turn90",
    "long_turn90",
    "long_turn180",
    "turn_in45",
    "turn_out45",
    "turn_in135",
    "turn_out135",
    "turn_v90",
    "long_turn_v90",
)


def acceleration_table(branch_point: int = 400) -> tuple[np.ndarray, np.ndarray, float]:
    """Return the 1001-entry MATLAB ``acctable`` profile and its sum."""
    time_ms = np.arange(1001, dtype=float)
    t = time_ms / 1000.0
    alpha = 1000.0 / branch_point
    values = np.ones_like(t)
    rising = t < branch_point / 1000.0
    falling = t > (1000 - branch_point) / 1000.0
    tr = t[rising]
    tf = 1.0 - t[falling]
    values[rising] = (1.0 - np.cos(np.pi * tr * alpha * (2.5 - tr * alpha) / 1.5)) / 2.0
    values[falling] = (1.0 - np.cos(np.pi * tf * alpha * (2.5 - tf * alpha) / 1.5)) / 2.0
    return values, time_ms, float(np.sum(values) / 1000.0)


def _matlab_uint32(value: float) -> int:
    """MATLAB rounds a positive floating value when converting to uint32."""
    return int(math.floor(value + 0.5))


def _profile_value(table: np.ndarray, sample: int, duration_s: float) -> float:
    # Direct translation of the MATLAB indexing and weighting. ``sample`` is
    # MATLAB's 1-based loop variable i, whereas NumPy indexing is zero-based.
    normalized = (sample - 1.0) / (duration_s * 1000.0)
    integer = math.trunc(normalized)
    fraction = normalized - integer
    inverse = integer + 1.0 - normalized
    matlab_index = _matlab_uint32(sample / duration_s)
    if not 1 <= matlab_index < len(table):
        raise ValueError("turn duration produces an invalid acceleration-table index")
    return inverse * table[matlab_index - 1] + fraction * table[matlab_index]


class _State:
    def __init__(self, velocity: float, initial_theta: float) -> None:
        self.theta = initial_theta
        self.ideal_r = 0.0
        self.ideal_u = velocity
        self.ideal_v = 0.0
        self.u = velocity
        self.v = 0.0
        self.r = 0.0
        self.beta = 0.0
        self.beta_dot = 0.0
        self.sum_error = 0.0
        self.yaw_response = 0.0
        self.x = [0.0]
        self.y = [0.0]
        self.ideal_x = [0.0]
        self.ideal_y = [0.0]
        self.logs = {name: [] for name in (
            "r", "r_acc", "beta", "beta_dot", "u", "v",
            "u_dot", "v_dot", "u_acc",
        )}


def _advance(
    state: _State,
    velocity: float,
    motor: MotorParameters,
    kp: float,
    ki: float,
    alpha: float,
    new_r: float,
    dynamics: DynamicsModel,
    *,
    coast_transform: bool = False,
    beta_override: float | None = None,
) -> None:
    previous_r = state.r
    state.r = new_r
    r_acc = (state.r - previous_r) / DT
    previous_beta = state.beta
    if beta_override is None:
        denominator = 1.0 + state.beta * state.beta / 2.0 if dynamics.nonlinear_slip else 1.0
        state.beta = (state.beta / DT - state.r) / (1.0 / DT + motor.slip_k / (velocity * denominator))
        denominator = 1.0 + state.beta * state.beta / 2.0 if dynamics.nonlinear_slip else 1.0
        state.beta_dot = -motor.slip_k * state.beta / (velocity * denominator) - state.r
    else:
        state.beta = float(beta_override)
        state.beta_dot = (state.beta - previous_beta) / DT

    error = velocity - state.u
    state.sum_error += error
    pi_power = (
        2.0 * motor.motor_ktr / motor.wheel_r * motor.gear_ratio
        * (error * kp + state.sum_error * ki) / motor.motor_r / motor.mass
    )
    u_dot = -motor.slip_k * state.beta * state.beta * alpha + state.r * state.v + pi_power
    u_acc = state.r * state.v
    v_dot = -motor.slip_k * state.beta - state.r * state.u

    state.logs["r"].append(state.r)
    state.logs["r_acc"].append(r_acc)
    state.logs["beta"].append(state.beta)
    state.logs["beta_dot"].append(state.beta_dot)
    state.logs["u_dot"].append(u_dot)
    state.logs["v_dot"].append(v_dot)
    state.logs["u_acc"].append(u_acc)

    if dynamics.closed_loop_speed:
        # The measured turn1600 speed loop includes feedforward omitted by the
        # old MATLAB equations and holds the commanded speed to within 0.6%.
        state.u = velocity
        u_dot = 0.0
        state.logs["u_dot"][-1] = u_dot
    else:
        state.u += u_dot * DT
    state.v = state.u * state.beta
    state.logs["u"].append(state.u)
    state.logs["v"].append(state.v)
    heading_rate = state.ideal_r if dynamics.heading_uses_commanded_yaw else state.r
    state.theta += heading_rate * DT

    lateral_scale = dynamics.lateral_velocity_position_scale
    ground_scale = max(0.0, 1.0 - dynamics.ground_slip_beta_sq_gain * state.beta * state.beta)
    position_u = (
        ground_scale * state.u * math.cos(state.beta)
        if dynamics.longitudinal_slip_projection
        else state.u
    )
    position_v = (
        ground_scale * state.u * math.sin(state.beta)
        if dynamics.longitudinal_slip_projection
        else (
            state.u * math.tan(state.beta)
            if dynamics.lateral_velocity_uses_tangent
            else state.v
        )
    )

    if coast_transform:
        x_dot = position_u * math.cos(state.theta) + lateral_scale * position_v * math.sin(state.theta)
        y_dot = position_u * math.sin(state.theta) - lateral_scale * position_v * math.cos(state.theta)
        ideal_x_dot = state.ideal_u * math.cos(state.theta) + state.ideal_v * math.sin(state.theta)
        ideal_y_dot = state.ideal_u * math.sin(state.theta) - state.ideal_v * math.cos(state.theta)
    else:
        x_dot = position_u * math.cos(state.theta) - lateral_scale * position_v * math.sin(state.theta)
        y_dot = position_u * math.sin(state.theta) + lateral_scale * position_v * math.cos(state.theta)
        ideal_x_dot = state.ideal_u * math.cos(state.theta) - state.ideal_v * math.sin(state.theta)
        ideal_y_dot = state.ideal_u * math.sin(state.theta) + state.ideal_v * math.cos(state.theta)

    # m/s * 1 ms has the same numeric value as mm.
    state.x.append(state.x[-1] + x_dot)
    state.y.append(state.y[-1] + y_dot)
    state.ideal_x.append(state.ideal_x[-1] + ideal_x_dot)
    state.ideal_y.append(state.ideal_y[-1] + ideal_y_dot)


def _yaw_response(state: _State, command: float, dynamics: DynamicsModel) -> float:
    if dynamics.yaw_time_constant_s <= 0.0:
        state.yaw_response = dynamics.yaw_gain * command
    else:
        decay = math.exp(-DT / dynamics.yaw_time_constant_s)
        state.yaw_response = (
            decay * state.yaw_response
            + (1.0 - decay) * dynamics.yaw_gain * command
        )
    return state.yaw_response


def _coast(
    state: _State,
    distance_mm: float,
    velocity: float,
    motor: MotorParameters,
    kp: float,
    ki: float,
    dynamics: DynamicsModel,
    *,
    alternate_transform: bool = False,
) -> None:
    travelled = 0.0
    # Preserve the MATLAB behavior: r becomes zero, but ideal_r retains the
    # final table value and therefore continues to update theta in this loop.
    while travelled < distance_mm:
        if dynamics.preserve_commanded_yaw_during_coast:
            yaw_rate = 0.0
        else:
            state.ideal_r = 0.0
            yaw_rate = _yaw_response(state, 0.0, dynamics)
        _advance(
            state, velocity, motor, kp, ki, 1.0, yaw_rate, dynamics,
            coast_transform=alternate_transform,
        )
        travelled += state.u


def _target_and_heading(motion: str) -> tuple[float, float]:
    if motion in {"turn90", "long_turn90", "turn_v90", "long_turn_v90"}:
        target = math.pi / 2.0
    elif motion == "long_turn180":
        target = math.pi
    elif motion in {"turn_in45", "turn_out45"}:
        target = math.pi / 4.0
    elif motion in {"turn_in135", "turn_out135"}:
        target = 3.0 * math.pi / 4.0
    else:
        raise ValueError(f"unknown motion: {motion}")
    initial = math.pi / 4.0 if motion in {"turn_out45", "turn_out135"} else 0.0
    return target, initial


def _lengths_and_coast(
    motion: str,
    state: _State,
    velocity: float,
    motor: MotorParameters,
    kp: float,
    ki: float,
    dynamics: DynamicsModel,
) -> tuple[float, float]:
    x, y, theta = state.x[-1], state.y[-1], state.theta
    if motion == "long_turn90":
        end = (90.0 - y) / math.sin(theta)
        return 90.0 - end * math.cos(theta) - x, end
    if motion == "turn90":
        end = (90.0 - y) / math.sin(theta) - 45.0
        return 90.0 - end * math.cos(theta) - x - 45.0, end
    if motion == "long_turn180":
        start = 95.0 - max(state.x)
        return start, start + x
    if motion == "turn_in135":
        end = (90.0 - y) / math.sin(theta)
        return 45.0 - end * math.cos(theta) - x, end
    if motion == "turn_out135":
        start = (90.0 - y) / math.sin(math.pi / 4.0)
        return start, 45.0 + x + start * math.sin(math.pi / 4.0)
    if motion == "turn_in45":
        end = (45.0 - y) / math.sin(theta)
        _coast(state, end, velocity, motor, kp, ki, dynamics, alternate_transform=True)
        end_dash = (45.0 - state.y[-1]) / math.sin(state.theta)
        start = 90.0 - end_dash * math.cos(state.theta) - state.x[-1]
        return start, end + end_dash
    if motion == "turn_out45":
        start = (45.0 - x) / math.sin(math.pi / 4.0)
        end = 90.0 - y - start * math.sin(math.pi / 4.0)
        _coast(state, end, velocity, motor, kp, ki, dynamics)
        start = (45.0 - state.x[-1]) / math.sin(math.pi / 4.0)
        end = 90.0 - state.y[-1] - start * math.sin(math.pi / 4.0) + end
        return start, end
    if motion in {"turn_v90", "long_turn_v90"}:
        diagonal = (90.0 if motion == "turn_v90" else 180.0) / SQRT2_MATLAB
        end = (diagonal - y) / math.sin(theta)
        _coast(state, end, velocity, motor, kp, ki, dynamics)
        end = (diagonal - state.y[-1]) / math.sin(state.theta) + end
        start = diagonal - end * math.cos(state.theta) - state.x[-1]
        return start, end
    raise AssertionError(motion)


def _plot_coordinates(
    motion: str,
    state: _State,
    start_length: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    x = np.asarray(state.x)
    y = np.asarray(state.y)
    ideal_x = np.asarray(state.ideal_x)
    ideal_y = np.asarray(state.ideal_y)

    if motion in {"turn_v90", "long_turn_v90"}:
        x, y = (x - y) / SQRT2_MATLAB, (x + y) / SQRT2_MATLAB
        ideal_x, ideal_y = (
            (ideal_x - ideal_y) / SQRT2_MATLAB,
            (ideal_x + ideal_y) / SQRT2_MATLAB,
        )
        offset = start_length * math.sin(math.pi / 4.0)
        y_origin = -45.0 if motion == "turn_v90" else -135.0
        return (
            np.r_[offset, offset + x],
            np.r_[offset + y_origin, offset + y + y_origin],
            np.r_[offset, offset + ideal_x],
            np.r_[offset + y_origin, offset + ideal_y + y_origin],
        )
    if motion in {"turn_out45", "turn_out135"}:
        offset = start_length * math.cos(math.pi / 4.0)
        return (
            np.r_[offset, x + offset],
            np.r_[-45.0 + offset, y - 45.0 + offset],
            np.r_[offset, ideal_x + offset],
            np.r_[-45.0 + offset, ideal_y - 45.0 + offset],
        )
    return (
        np.r_[-45.0 + start_length, x - 45.0 + start_length],
        np.r_[-45.0, y - 45.0],
        np.r_[-45.0 + start_length, ideal_x - 45.0 + start_length],
        np.r_[-45.0, ideal_y - 45.0],
    )


def simulate_turn(
    motion: str,
    velocity: float,
    motor: MotorParameters,
    radius_mm: float,
    kp: float = 4.0,
    ki: float = 0.01,
    alpha: float = 1.0,
    dynamics: DynamicsModel = MATLAB_DYNAMICS,
    *,
    velocity_profile: np.ndarray | None = None,
    yaw_rate_profile: np.ndarray | None = None,
    beta_profile: np.ndarray | None = None,
    yaw_angle_rad: float | None = None,
) -> TurnResult:
    """Simulate one left turn using the equations in the MATLAB functions.

    ``velocity_profile`` optionally supplies the measured or predicted
    longitudinal velocity during the turn.  It is normalized to the generated
    yaw-profile sample count and is applied at every 1 ms dynamics update.
    """
    if motion not in MOTIONS:
        raise ValueError(f"motion must be one of {', '.join(MOTIONS)}")
    if velocity <= 0.0 or radius_mm <= 0.0:
        raise ValueError("velocity and radius_mm must be positive")
    target, initial_theta = _target_and_heading(motion)
    table, _, table_sum = acceleration_table()
    yaw_rate_max = velocity / (radius_mm / 1000.0)
    duration = target / (table_sum * yaw_rate_max)
    samples = math.trunc(duration / DT)
    if velocity_profile is None:
        sample_velocities = np.full(samples, velocity, dtype=float)
    else:
        source = np.asarray(velocity_profile, dtype=float).reshape(-1)
        if len(source) == 0 or not np.all(np.isfinite(source)) or np.any(source <= 0.0):
            raise ValueError("velocity_profile must contain finite positive values")
        sample_velocities = np.interp(
            np.linspace(0.0, 1.0, samples),
            np.linspace(0.0, 1.0, len(source)),
            source,
        )
    sample_yaw_rates = None
    if yaw_rate_profile is not None:
        source = np.asarray(yaw_rate_profile, dtype=float).reshape(-1)
        if len(source) == 0 or not np.all(np.isfinite(source)):
            raise ValueError("yaw_rate_profile must contain finite values")
        sample_yaw_rates = np.interp(
            np.linspace(0.0, 1.0, samples),
            np.linspace(0.0, 1.0, len(source)),
            source,
        )
        if yaw_angle_rad is not None:
            integral = float(np.sum(sample_yaw_rates) * DT)
            if abs(integral) < 1.0e-12:
                raise ValueError("yaw_rate_profile integral must be non-zero")
            sample_yaw_rates *= yaw_angle_rad / integral
    sample_betas = None
    if beta_profile is not None:
        source = np.asarray(beta_profile, dtype=float).reshape(-1)
        if len(source) == 0 or not np.all(np.isfinite(source)):
            raise ValueError("beta_profile must contain finite values")
        sample_betas = np.interp(
            np.linspace(0.0, 1.0, samples),
            np.linspace(0.0, 1.0, len(source)),
            source,
        )
    state = _State(velocity, initial_theta)

    for sample in range(1, samples + 1):
        state.ideal_r = _profile_value(table, sample, duration) * yaw_rate_max
        actual_r = (
            float(sample_yaw_rates[sample - 1])
            if sample_yaw_rates is not None
            else _yaw_response(state, state.ideal_r, dynamics)
        )
        _advance(
            state, float(sample_velocities[sample - 1]), motor, kp, ki,
            alpha, actual_r, dynamics,
            coast_transform=motion == "turn_in45",
            beta_override=(
                float(sample_betas[sample - 1])
                if sample_betas is not None
                else None
            ),
        )

    start, end = _lengths_and_coast(motion, state, velocity, motor, kp, ki, dynamics)
    x, y, ideal_x, ideal_y = _plot_coordinates(motion, state, start)
    count = len(state.logs["r"])
    return TurnResult(
        motion=motion,
        model_name=dynamics.name,
        velocity=velocity,
        radius_mm=radius_mm,
        target_angle_rad=target,
        duration_s=duration,
        start_length_mm=start,
        end_length_mm=end,
        final_angle_rad=state.theta,
        time_s=np.arange(1, count + 1, dtype=float) * DT,
        x_mm=x,
        y_mm=y,
        ideal_x_mm=ideal_x,
        ideal_y_mm=ideal_y,
        yaw_rate_rad_s=np.asarray(state.logs["r"]),
        yaw_accel_rad_s2=np.asarray(state.logs["r_acc"]),
        beta_rad=np.asarray(state.logs["beta"]),
        beta_rate_rad_s=np.asarray(state.logs["beta_dot"]),
        velocity_m_s=np.asarray(state.logs["u"]),
        lateral_velocity_m_s=np.asarray(state.logs["v"]),
        longitudinal_accel_m_s2=np.asarray(state.logs["u_dot"]),
        lateral_accel_m_s2=np.asarray(state.logs["v_dot"]),
        centripetal_accel_m_s2=np.asarray(state.logs["u_acc"]),
    )


# Function names retained for callers migrating directly from MATLAB.  Each
# wrapper returns the full result; L_start/L_end are available as attributes.
def turn90(velocity, motor, radius_mm, kp=4.0, ki=0.01, alpha=1.0) -> TurnResult:
    return simulate_turn("turn90", velocity, motor, radius_mm, kp, ki, alpha)


def long_turn90(velocity, motor, radius_mm, kp=4.0, ki=0.01, alpha=1.0) -> TurnResult:
    return simulate_turn("long_turn90", velocity, motor, radius_mm, kp, ki, alpha)


def long_turn180(velocity, motor, radius_mm, kp=4.0, ki=0.01, alpha=1.0) -> TurnResult:
    return simulate_turn("long_turn180", velocity, motor, radius_mm, kp, ki, alpha)


def turn_in45(velocity, motor, radius_mm, kp=4.0, ki=0.01, alpha=1.0) -> TurnResult:
    return simulate_turn("turn_in45", velocity, motor, radius_mm, kp, ki, alpha)


def turn_out45(velocity, motor, radius_mm, kp=4.0, ki=0.01, alpha=1.0) -> TurnResult:
    return simulate_turn("turn_out45", velocity, motor, radius_mm, kp, ki, alpha)


def turn_in135(velocity, motor, radius_mm, kp=4.0, ki=0.01, alpha=1.0) -> TurnResult:
    return simulate_turn("turn_in135", velocity, motor, radius_mm, kp, ki, alpha)


def turn_out135(velocity, motor, radius_mm, kp=4.0, ki=0.01, alpha=1.0) -> TurnResult:
    return simulate_turn("turn_out135", velocity, motor, radius_mm, kp, ki, alpha)


def turn_v90(velocity, motor, radius_mm, kp=4.0, ki=0.01, alpha=1.0) -> TurnResult:
    return simulate_turn("turn_v90", velocity, motor, radius_mm, kp, ki, alpha)


def long_turn_v90(velocity, motor, radius_mm, kp=4.0, ki=0.01, alpha=1.0) -> TurnResult:
    return simulate_turn("long_turn_v90", velocity, motor, radius_mm, kp, ki, alpha)


def draw_maze(axis) -> None:
    """Draw the same 90 mm grid, diagonals, and posts as ``draw_maze.m``."""
    start, end = -180.0, 90.0
    for coordinate in np.arange(start, end + 0.1, 45.0):
        axis.plot([start, end], [coordinate, coordinate], "k--", linewidth=0.6)
        axis.plot([coordinate, coordinate], [start, end], "k--", linewidth=0.6)
    for yy in (-90.0, 0.0, 90.0):
        for xx in (-90.0, 0.0, 90.0):
            axis.plot([-45 + xx, -90 + xx], [yy, -45 + yy], "k--", linewidth=0.6)
            axis.plot([-90 + xx, -45 + xx], [-45 + yy, -90 + yy], "k--", linewidth=0.6)
            axis.plot([-45 + xx, xx], [-90 + yy, -45 + yy], "k--", linewidth=0.6)
            axis.plot([xx, -45 + xx], [-45 + yy, yy], "k--", linewidth=0.6)
    from matplotlib.patches import Rectangle
    for xx in np.arange(start, end + 0.1, 90.0):
        for yy in np.arange(start, end + 0.1, 90.0):
            axis.add_patch(Rectangle((xx - 3, yy - 3), 6, 6, fill=False, edgecolor="r", linewidth=2))
    axis.set_aspect("equal", adjustable="box")


def plot_result(result: TurnResult):
    """Create figures equivalent to the two MATLAB diagnostic figures."""
    import matplotlib.pyplot as plt

    figure1, axes = plt.subplots(3, 2, figsize=(11, 12))
    draw_maze(axes[0, 0])
    axes[0, 0].plot(result.x_mm, result.y_mm, "b.", markersize=2, label="model")
    axes[0, 0].plot(result.ideal_x_mm, result.ideal_y_mm, "r.", markersize=2, label="ideal")
    axes[0, 0].set_title("turn simulation")
    axes[0, 0].legend()
    series = (
        (result.yaw_rate_rad_s, "r"),
        (result.beta_rad, "beta"),
        (result.beta_rate_rad_s, "beta dot"),
        (result.velocity_m_s, "u"),
        (result.lateral_velocity_m_s, "v"),
    )
    for axis, (values, title) in zip(axes.flat[1:], series):
        axis.plot(result.time_s, values, "g.", markersize=2)
        axis.set_title(title)
        axis.set_xlabel("time [s]")
    figure1.tight_layout()

    figure2, axes2 = plt.subplots(2, 2, figsize=(11, 8))
    diagnostics = (
        (result.yaw_accel_rad_s2, "r acc"),
        (result.centripetal_accel_m_s2, "u acc"),
        (result.longitudinal_accel_m_s2, "u dot"),
        (result.lateral_accel_m_s2, "v dot"),
    )
    for axis, (values, title) in zip(axes2.flat, diagnostics):
        axis.plot(result.time_s, values, "g.", markersize=2)
        axis.set_title(title)
        axis.set_xlabel("time [s]")
    figure2.tight_layout()
    return figure1, figure2


def write_csv(result: TurnResult, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields: dict[str, np.ndarray] = {
        "time_s": result.time_s,
        "yaw_rate_rad_s": result.yaw_rate_rad_s,
        "yaw_accel_rad_s2": result.yaw_accel_rad_s2,
        "beta_rad": result.beta_rad,
        "beta_rate_rad_s": result.beta_rate_rad_s,
        "velocity_m_s": result.velocity_m_s,
        "lateral_velocity_m_s": result.lateral_velocity_m_s,
        "longitudinal_accel_m_s2": result.longitudinal_accel_m_s2,
        "lateral_accel_m_s2": result.lateral_accel_m_s2,
        "centripetal_accel_m_s2": result.centripetal_accel_m_s2,
        # The plotted trajectory has one extra leading point.
        "x_mm": result.x_mm[2:],
        "y_mm": result.y_mm[2:],
        "ideal_x_mm": result.ideal_x_mm[2:],
        "ideal_y_mm": result.ideal_y_mm[2:],
    }
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(fields)
        writer.writerows(zip(*(fields[name] for name in fields)))


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="MATLAB matlab_turn2 compatible turn simulator")
    parser.add_argument("--preset", choices=PRESETS, default="type8i")
    parser.add_argument("--motion", choices=MOTIONS, default="long_turn90")
    parser.add_argument("--velocity", type=float, help="override velocity [m/s]")
    parser.add_argument("--radius", type=float, help="override minimum radius [mm]")
    parser.add_argument("--kp", type=float)
    parser.add_argument("--ki", type=float)
    parser.add_argument("--alpha", type=float)
    parser.add_argument("--plot", type=Path, help="save diagnostic plots using this filename stem")
    parser.add_argument("--csv", type=Path, help="write simulation samples to CSV")
    parser.add_argument("--show", action="store_true", help="show diagnostic plots interactively")
    return parser.parse_args()


def main() -> None:
    args = _arguments()
    preset = PRESETS[args.preset]
    radius = args.radius if args.radius is not None else preset.radii_mm.get(args.motion)
    if radius is None:
        raise SystemExit(f"preset {args.preset!r} has no radius for {args.motion}; specify --radius")
    result = simulate_turn(
        args.motion,
        args.velocity if args.velocity is not None else preset.velocity,
        preset.motor,
        radius,
        args.kp if args.kp is not None else preset.kp,
        args.ki if args.ki is not None else preset.ki,
        args.alpha if args.alpha is not None else preset.alpha,
        preset.dynamics,
    )
    summary = result.summary()
    configured = preset.configured_lengths_mm.get(args.motion)
    if configured:
        summary["configured_start_length_mm"] = configured[0]
        summary["configured_end_length_mm"] = configured[1]
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    if args.csv:
        write_csv(result, args.csv)
    if args.plot or args.show:
        figures = plot_result(result)
        if args.plot:
            args.plot.parent.mkdir(parents=True, exist_ok=True)
            suffix = args.plot.suffix or ".png"
            stem = args.plot.with_suffix("")
            for number, figure in enumerate(figures, 1):
                figure.savefig(stem.with_name(f"{stem.name}_{number}").with_suffix(suffix), dpi=160)
        if args.show:
            import matplotlib.pyplot as plt
            plt.show()


if __name__ == "__main__":
    main()
