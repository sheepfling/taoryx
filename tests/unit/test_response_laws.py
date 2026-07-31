"""Tests for the shared bounded pseudo-6DOF response law."""

from __future__ import annotations

import math

import pytest

from taoryx.trajectory.pseudo6dof_profiles import AxisResponseProfile
from taoryx.trajectory.response_laws import AxisResponseState, bounded_axis_rate_command, step_bounded_axis_response


def _profile() -> AxisResponseProfile:
    return AxisResponseProfile(
        time_constant_s=0.5,
        damping_ratio=0.75,
        maximum_rate_rad_s=0.4,
        maximum_acceleration_rad_s2=0.8,
    )


def test_response_law_is_bounded_and_finite() -> None:
    state = AxisResponseState(0.0, 0.0)
    for _ in range(100):
        state = step_bounded_axis_response(_profile(), state, math.pi, 0.05)

    assert math.isfinite(state.angle_rad)
    assert abs(state.rate_rad_s) <= 0.4
    ####


def test_response_law_reverses_without_instantaneous_rate_jump() -> None:
    profile = _profile()
    state = AxisResponseState(0.4, 0.35)
    next_state = step_bounded_axis_response(profile, state, -0.4, 0.05)

    assert next_state.rate_rad_s < state.rate_rad_s
    assert abs(next_state.rate_rad_s - state.rate_rad_s) <= profile.maximum_acceleration_rad_s2 * 0.05 + 1.0e-12
    ####


def test_response_law_rejects_nonpositive_step() -> None:
    with pytest.raises(ValueError, match="must be positive"):
        step_bounded_axis_response(_profile(), AxisResponseState(0.0, 0.0), 1.0, 0.0)
    ####


def test_native_first_order_compatibility_law_is_profile_bounded() -> None:
    profile = _profile()
    assert bounded_axis_rate_command(profile, 100.0) == pytest.approx(profile.maximum_rate_rad_s)
    assert bounded_axis_rate_command(profile, -100.0) == pytest.approx(-profile.maximum_rate_rad_s)
    assert bounded_axis_rate_command(profile, profile.time_constant_s * 0.25) == pytest.approx(0.25)
    ####
