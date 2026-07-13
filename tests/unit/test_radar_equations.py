from __future__ import annotations

import math

import pytest

from taoryx.equations import (
    CartesianVector3,
    RadarAngles,
    radar_azimuth_elevation,
    radar_line_of_sight_unit_vector,
    radar_observation,
    radar_range_rate,
    radar_relative_position,
)


def test_radar_relative_position_and_line_of_sight() -> None:
    station = CartesianVector3(1.0, 2.0, 3.0)
    vehicle = CartesianVector3(4.0, 6.0, 9.0)

    relative = radar_relative_position(vehicle, station)
    los = radar_line_of_sight_unit_vector(relative)

    assert relative == CartesianVector3(3.0, 4.0, 6.0)
    assert math.sqrt(los.x * los.x + los.y * los.y + los.z * los.z) == pytest.approx(1.0)
####


def test_radar_azimuth_and_elevation_match_station_basis() -> None:
    station_longitude = 0.0
    station_latitude = 0.0
    relative = CartesianVector3(-12.0, 4.0, 3.0)

    angles = radar_azimuth_elevation(station_longitude, station_latitude, relative)

    assert angles == RadarAngles(
        math.atan2(4.0, 3.0),
        math.asin(-12.0 / 13.0),
    )
####


def test_radar_observation_and_range_rate_are_consistent() -> None:
    station = CartesianVector3(-10.0, 4.0, 2.0)
    vehicle = CartesianVector3(40.0, 34.0, 27.0)
    velocity = CartesianVector3(5.0, -2.0, 3.0)

    observation = radar_observation(vehicle, station, math.radians(0.0), math.radians(0.0))
    assert observation.relative_position == CartesianVector3(50.0, 30.0, 25.0)
    assert observation.range == pytest.approx(math.sqrt(50.0 * 50.0 + 30.0 * 30.0 + 25.0 * 25.0))
    assert radar_range_rate(observation.relative_position, velocity) == pytest.approx(
        (50.0 * 5.0 + 30.0 * -2.0 + 25.0 * 3.0) / observation.range
    )
####
