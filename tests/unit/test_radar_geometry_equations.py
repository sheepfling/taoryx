from __future__ import annotations

import math

import pytest

from taoryx.equations import (
    BodyAxes,
    CartesianVector3,
    radar_aspect_cosine,
    radar_aspect_sine,
    radar_azimuth_acceleration,
    radar_azimuth_rate,
    radar_elevation_acceleration,
    radar_elevation_rate,
    radar_meridional_cosine,
    radar_meridional_sine,
    radar_range_acceleration,
    radar_range_rate,
    radar_relative_position,
    radar_unit_vector_acceleration,
    radar_unit_vector_rate,
    relative_vehicle_azimuth,
    relative_vehicle_elevation,
    relative_vehicle_position,
    relative_vehicle_unit_vector,
)


def _identity_body_axes() -> BodyAxes:
    return BodyAxes(
        CartesianVector3(1.0, 0.0, 0.0),
        CartesianVector3(0.0, 1.0, 0.0),
        CartesianVector3(0.0, 0.0, 1.0),
    )
####


def test_radar_rates_and_accelerations_follow_the_manual_formulas() -> None:
    station_longitude = 0.0
    station_latitude = 0.0
    relative_position = CartesianVector3(10.0, 0.0, 0.0)
    velocity = CartesianVector3(2.0, 0.0, 0.0)
    acceleration = CartesianVector3(3.0, 0.0, 0.0)

    unit_vector = radar_unit_vector_rate(relative_position, velocity)
    unit_vector_acceleration = radar_unit_vector_acceleration(relative_position, velocity, acceleration)

    assert radar_relative_position(CartesianVector3(4.0, 6.0, 9.0), CartesianVector3(1.0, 2.0, 3.0)) == CartesianVector3(3.0, 4.0, 6.0)
    assert radar_range_rate(relative_position, velocity) == pytest.approx(2.0)
    assert radar_range_acceleration(relative_position, velocity, acceleration) == pytest.approx(3.0)
    assert unit_vector == CartesianVector3(0.0, 0.0, 0.0)
    assert unit_vector_acceleration == CartesianVector3(0.0, 0.0, 0.0)

    relative_position = CartesianVector3(0.0, 3.0, 4.0)
    velocity = CartesianVector3(0.0, 0.0, 1.0)
    acceleration = CartesianVector3(0.0, 0.0, 0.0)

    assert radar_azimuth_rate(station_longitude, station_latitude, relative_position, velocity) == pytest.approx(-0.12)
    assert radar_azimuth_acceleration(station_longitude, station_latitude, relative_position, velocity, acceleration) == pytest.approx(0.0384)
    assert radar_elevation_rate(station_longitude, station_latitude, relative_position, velocity) == pytest.approx(0.0)
    assert radar_elevation_acceleration(station_longitude, station_latitude, relative_position, velocity, acceleration) == pytest.approx(0.0)
####


def test_radar_aspect_and_relative_vehicle_helpers_follow_the_manual_formulas() -> None:
    body_axes = _identity_body_axes()
    line_of_sight = CartesianVector3(1.0 / math.sqrt(2.0), 1.0 / math.sqrt(2.0), 0.0)

    assert radar_aspect_sine(line_of_sight, body_axes) == pytest.approx(1.0 / math.sqrt(2.0))
    assert radar_aspect_cosine(line_of_sight, body_axes) == pytest.approx(-1.0 / math.sqrt(2.0))
    assert radar_meridional_sine(line_of_sight, body_axes) == pytest.approx(0.0)
    assert radar_meridional_cosine(line_of_sight, body_axes) == pytest.approx(-1.0)
    assert relative_vehicle_position(CartesianVector3(5.0, 7.0, 9.0), CartesianVector3(1.0, 2.0, 3.0)) == CartesianVector3(4.0, 5.0, 6.0)
    assert relative_vehicle_unit_vector(CartesianVector3(0.0, 0.0, 5.0)) == CartesianVector3(0.0, 0.0, 1.0)
    assert relative_vehicle_azimuth(body_axes, CartesianVector3(3.0, 4.0, 0.0)) == pytest.approx(math.atan2(4.0, 3.0))
    assert relative_vehicle_elevation(body_axes, CartesianVector3(0.0, 0.0, 5.0)) == pytest.approx(-math.pi / 2.0)
####
