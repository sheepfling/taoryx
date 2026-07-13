from __future__ import annotations

import math

import pytest

from taoryx.equations.frames import BodyAxes
from taoryx.equations.geodesy import CartesianVector3
from taoryx.guidance import proportional_navigation
from taoryx.radar import radar_observations, relative_vehicle_observations


def test_proportional_navigation_returns_expected_closing_command() -> None:
    result = proportional_navigation(
        CartesianVector3(0.0, 0.0, 0.0),
        CartesianVector3(10.0, 0.0, 0.0),
        CartesianVector3(100.0, 10.0, 0.0),
        CartesianVector3(0.0, 0.0, 0.0),
        3.0,
    )

    assert result.closure_velocity == pytest.approx(9.9503719)
    assert result.yaw_radians == pytest.approx(math.atan2(10.0, 100.0))
    assert result.ecfc_acceleration.x == pytest.approx(-math.sin(result.yaw_radians) * result.yaw_acceleration)
####


def test_radar_observations_include_derivatives_and_aspect() -> None:
    axes = BodyAxes(CartesianVector3(1.0, 0.0, 0.0), CartesianVector3(0.0, 1.0, 0.0), CartesianVector3(0.0, 0.0, 1.0))
    result = radar_observations(
        CartesianVector3(0.0, 0.0, 0.0),
        0.0,
        0.0,
        CartesianVector3(100.0, 0.0, 0.0),
        CartesianVector3(10.0, 0.0, 0.0),
        CartesianVector3(0.0, 0.0, 0.0),
        axes,
    )

    assert result.range == pytest.approx(100.0)
    assert result.range_rate == pytest.approx(10.0)
    assert result.azimuth_radians == pytest.approx(0.0)
####


def test_relative_vehicle_observations_use_target_minus_current_order() -> None:
    axes = BodyAxes(CartesianVector3(1.0, 0.0, 0.0), CartesianVector3(0.0, 1.0, 0.0), CartesianVector3(0.0, 0.0, 1.0))
    result = relative_vehicle_observations(
        CartesianVector3(0.0, 0.0, 0.0),
        CartesianVector3(1.0, 0.0, 0.0),
        CartesianVector3(10.0, 10.0, 0.0),
        CartesianVector3(0.0, 0.0, 0.0),
        axes,
    )

    assert result.relative_position == CartesianVector3(10.0, 10.0, 0.0)
    assert result.range == pytest.approx(math.sqrt(200.0))
    assert result.closure_velocity == pytest.approx(math.sqrt(2.0) / 2.0)
####
