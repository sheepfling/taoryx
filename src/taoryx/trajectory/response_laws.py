"""Reusable bounded attitude-response laws for pseudo-6DOF realizations."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .pseudo6dof_profiles import AxisResponseProfile


def _clamp(value: float, lower: float, upper: float) -> float:
    return min(upper, max(lower, value))
    ####


@dataclass(frozen=True, slots=True)
class AxisResponseState:
    """One angular position/rate state after a response-law update."""

    angle_rad: float
    rate_rad_s: float
    ####


def bounded_axis_acceleration(
    profile: AxisResponseProfile,
    state: AxisResponseState,
    command_angle_rad: float,
) -> float:
    """Return the continuous bounded angular acceleration for one response axis.

    The discrete helper below advances this same law with a semi-implicit
    Euler step.  Exposing its continuous right-hand side lets the common
    lower-tier adapter linearize the exact named response law rather than a
    separately invented approximation.
    """

    desired_rate = _clamp(
        (command_angle_rad - state.angle_rad) / profile.time_constant_s,
        -profile.maximum_rate_rad_s,
        profile.maximum_rate_rad_s,
    )
    return _clamp(
        (desired_rate - state.rate_rad_s) * profile.damping_ratio / profile.time_constant_s,
        -profile.maximum_acceleration_rad_s2,
        profile.maximum_acceleration_rad_s2,
    )
    ####


def step_bounded_axis_response(
    profile: AxisResponseProfile,
    state: AxisResponseState,
    command_angle_rad: float,
    dt_s: float,
) -> AxisResponseState:
    """Advance a bounded second-order-like axis response by one step.

    This is a response-law surrogate, not a moment equation.  The command is
    converted to a bounded desired rate, then the rate is moved toward that
    target with bounded angular acceleration.  The resulting position and
    rate are both limited by the declared profile.
    """

    if dt_s <= 0.0:
        raise ValueError("response-law step must be positive")
    acceleration = bounded_axis_acceleration(profile, state, command_angle_rad)
    rate = _clamp(
        state.rate_rad_s + acceleration * dt_s,
        -profile.maximum_rate_rad_s,
        profile.maximum_rate_rad_s,
    )
    angle = state.angle_rad + rate * dt_s
    return AxisResponseState(angle_rad=angle, rate_rad_s=rate)
    ####


def bounded_axis_rate_command(profile: AxisResponseProfile, angle_error_rad: float) -> float:
    """Return the native-runtime first-order rate command for one axis.

    The native kinematic bridge does not carry a persistent angular-rate
    state, so it cannot yet apply the second-order acceleration limit used by
    :func:`step_bounded_axis_response`.  This helper is the explicitly named
    compatibility law for that bridge: a first-order error response with a
    per-axis rate bound.  Keeping it here prevents the native provider from
    silently inventing a second response law.
    """

    if not math.isfinite(angle_error_rad):
        raise ValueError("angle error must be finite")
    requested = angle_error_rad / profile.time_constant_s
    return max(-profile.maximum_rate_rad_s, min(profile.maximum_rate_rad_s, requested))
    ####


__all__ = ["AxisResponseState", "bounded_axis_acceleration", "bounded_axis_rate_command", "step_bounded_axis_response"]
