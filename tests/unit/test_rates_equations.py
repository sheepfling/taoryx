from __future__ import annotations

import pytest

from taoryx.equations import (
    CartesianVector3,
    altitude_acceleration_from_ecfc,
    altitude_rate_from_ecfc,
    ecfc_to_geodetic_position,
    geocentric_flight_path_rates_from_ecfc,
    geocentric_heading_rate_from_ecfc,
    geocentric_latitude_rate_from_ecfc,
    geocentric_vertical_flight_path_angle_rate_from_ecfc,
    geodetic_down_unit_vector_derivative,
    geodetic_flight_path_rates_from_ecfc,
    geodetic_heading_rate_from_ecfc,
    geodetic_latitude_rate_from_ecfc,
    geodetic_position_to_ecfc,
    geodetic_surface_normal_distance,
    geodetic_unit_vectors,
    geodetic_vertical_flight_path_angle_rate_from_ecfc,
    ground_speed_magnitude_from_ecfc,
    ground_speed_vector_from_ecfc,
    longitude_from_ecfc_position,
    longitude_rate_from_ecfc,
)


def test_longitude_and_latitude_rates_follow_the_manual_formulas() -> None:
    position = CartesianVector3(10.0, 0.0, 0.0)
    velocity = CartesianVector3(0.0, 0.0, 10.0)

    assert longitude_from_ecfc_position(position) == pytest.approx(0.0)
    assert longitude_rate_from_ecfc(position, velocity) == pytest.approx(0.0)
    assert geocentric_latitude_rate_from_ecfc(position, velocity) == pytest.approx(1.0)
    assert geocentric_latitude_rate_from_ecfc(position, velocity) == pytest.approx(
        1.0
    )
####


def test_geodetic_rates_and_ground_speed_follow_the_manual_formulas() -> None:
    equatorial_radius = 6_378_137.0
    eccentricity = 0.08181919084262149
    position = geodetic_position_to_ecfc(equatorial_radius, eccentricity, 0.0, 0.0, 1000.0)
    altitude_velocity = CartesianVector3(30.0, 0.0, 0.0)
    latitude_velocity = CartesianVector3(0.0, 0.0, 30.0)
    acceleration = CartesianVector3(1.0, 2.0, 3.0)

    geodetic_position = ecfc_to_geodetic_position(equatorial_radius, eccentricity, position.x, position.y, position.z)
    longitude_rate = longitude_rate_from_ecfc(position, latitude_velocity)
    latitude_rate = geodetic_latitude_rate_from_ecfc(equatorial_radius, eccentricity, position, latitude_velocity)
    down = geodetic_unit_vectors(geodetic_position.longitude_radians, geodetic_position.latitude_radians)[2]
    down_derivative = geodetic_down_unit_vector_derivative(
        geodetic_position.longitude_radians,
        geodetic_position.latitude_radians,
        longitude_rate,
        latitude_rate,
    )

    assert altitude_rate_from_ecfc(equatorial_radius, eccentricity, position, altitude_velocity) == pytest.approx(30.0)
    assert geodetic_latitude_rate_from_ecfc(equatorial_radius, eccentricity, position, latitude_velocity) == pytest.approx(
        30.0
        / (
            geodetic_position.altitude
            + geodetic_surface_normal_distance(equatorial_radius, eccentricity, 0.0)
            * (1.0 - eccentricity * eccentricity)
        )
    )
    assert geodetic_down_unit_vector_derivative(0.0, 0.0, 0.1, 0.2) == CartesianVector3(0.2, -0.1, 0.0)

    expected_altitude_acceleration = -(
        acceleration.x * down.x + acceleration.y * down.y + acceleration.z * down.z
    ) - (
        latitude_velocity.x * down_derivative.x
        + latitude_velocity.y * down_derivative.y
        + latitude_velocity.z * down_derivative.z
    )
    assert altitude_acceleration_from_ecfc(equatorial_radius, eccentricity, position, latitude_velocity, acceleration) == pytest.approx(expected_altitude_acceleration)

    ground_speed = ground_speed_vector_from_ecfc(equatorial_radius, eccentricity, position, altitude_velocity)
    assert ground_speed == CartesianVector3(0.0, 0.0, 0.0)
    assert ground_speed_magnitude_from_ecfc(equatorial_radius, eccentricity, position, altitude_velocity) == pytest.approx(0.0)
####


def test_flight_path_angle_rates_follow_the_manual_formulas() -> None:
    position = CartesianVector3(10.0, 0.0, 0.0)
    velocity = CartesianVector3(0.0, 10.0, 0.0)
    acceleration = CartesianVector3(-5.0, 0.0, 0.0)

    geocentric_rates = geocentric_flight_path_rates_from_ecfc(position, velocity, acceleration)
    geocentric_latitude_rate = geocentric_latitude_rate_from_ecfc(position, velocity)
    geocentric_longitude_rate = longitude_rate_from_ecfc(position, velocity)

    assert geocentric_longitude_rate == pytest.approx(1.0)
    assert geocentric_latitude_rate == pytest.approx(0.0)
    assert geocentric_rates == pytest.approx((1.0, -0.5))
    assert geocentric_vertical_flight_path_angle_rate_from_ecfc(position, velocity, acceleration) == pytest.approx(1.0)
    assert geocentric_heading_rate_from_ecfc(position, velocity, acceleration) == pytest.approx(-0.5)

    equatorial_radius = 10.0
    eccentricity = 0.1
    geodetic_position = geodetic_position_to_ecfc(equatorial_radius, eccentricity, 0.0, 0.0, 0.0)
    geodetic_rates = geodetic_flight_path_rates_from_ecfc(
        equatorial_radius,
        eccentricity,
        geodetic_position,
        velocity,
        acceleration,
    )
    geodetic_latitude_rate = geodetic_latitude_rate_from_ecfc(
        equatorial_radius,
        eccentricity,
        geodetic_position,
        velocity,
    )

    assert longitude_rate_from_ecfc(geodetic_position, velocity) == pytest.approx(1.0)
    assert geodetic_latitude_rate == pytest.approx(0.0)
    assert geodetic_rates == pytest.approx((1.0, -0.5))
    assert geodetic_vertical_flight_path_angle_rate_from_ecfc(
        equatorial_radius,
        eccentricity,
        geodetic_position,
        velocity,
        acceleration,
    ) == pytest.approx(1.0)
    assert geodetic_heading_rate_from_ecfc(
        equatorial_radius,
        eccentricity,
        geodetic_position,
        velocity,
        acceleration,
    ) == pytest.approx(-0.5)
####
