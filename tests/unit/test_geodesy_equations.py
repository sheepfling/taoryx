from __future__ import annotations

import math

import pytest

from taoryx.equations import (
    VelocityAngles,
    ecfc_to_geocentric_position,
    ecfc_to_geocentric_velocity,
    ecfc_to_geodetic_position,
    ecfc_to_geodetic_velocity,
    geocentric_position_to_ecfc,
    geocentric_unit_vectors,
    geocentric_velocity_angles_from_components,
    geocentric_velocity_components_from_angles,
    geocentric_velocity_to_ecfc,
    geodetic_position_to_ecfc,
    geodetic_surface_normal_distance,
    geodetic_unit_vectors,
    geodetic_velocity_angles_from_components,
    geodetic_velocity_components_from_angles,
    geodetic_velocity_to_ecfc,
    polar_radius_from_equatorial_radius,
)


def test_polar_radius_matches_ellipsoid_parameter_relation() -> None:
    equatorial_radius = 6_378_137.0
    eccentricity = 0.08181919084262149
    expected_flattening = 1.0 - math.sqrt(1.0 - eccentricity * eccentricity)

    assert polar_radius_from_equatorial_radius(equatorial_radius, eccentricity=eccentricity) == pytest.approx(6_356_752.314245179)
    assert polar_radius_from_equatorial_radius(equatorial_radius, flattening=expected_flattening) == pytest.approx(6_356_752.314245179)
####


def test_geocentric_position_round_trips_through_ecfc() -> None:
    position = geocentric_position_to_ecfc(10.0, math.radians(30.0), math.radians(20.0))
    recovered = ecfc_to_geocentric_position(position.x, position.y, position.z)

    assert recovered.radius == pytest.approx(10.0)
    assert recovered.longitude_radians == pytest.approx(math.radians(30.0))
    assert recovered.latitude_radians == pytest.approx(math.radians(20.0))
####


def test_geodetic_surface_geometry_matches_manual_formulas() -> None:
    equatorial_radius = 6_378_137.0
    eccentricity = 0.08181919084262149
    longitude = math.radians(-73.5)
    latitude = math.radians(40.0)
    altitude = 1234.5

    normal_distance = geodetic_surface_normal_distance(equatorial_radius, eccentricity, latitude)
    position = geodetic_position_to_ecfc(equatorial_radius, eccentricity, longitude, latitude, altitude)
    equatorial_projection = (normal_distance + altitude) * math.cos(latitude)

    assert position.x == pytest.approx(equatorial_projection * math.cos(longitude))
    assert position.y == pytest.approx(equatorial_projection * math.sin(longitude))
    assert position.z == pytest.approx((normal_distance * (1.0 - eccentricity * eccentricity) + altitude) * math.sin(latitude))
####


def test_geodetic_position_round_trips_through_ecfc() -> None:
    equatorial_radius = 6_378_137.0
    eccentricity = 0.08181919084262149
    longitude = math.radians(23.0)
    latitude = math.radians(51.0)
    altitude = 1425.0

    position = geodetic_position_to_ecfc(equatorial_radius, eccentricity, longitude, latitude, altitude)
    recovered = ecfc_to_geodetic_position(
        equatorial_radius,
        eccentricity,
        position.x,
        position.y,
        position.z,
    )

    assert recovered.longitude_radians == pytest.approx(longitude)
    assert recovered.latitude_radians == pytest.approx(latitude)
    assert recovered.altitude == pytest.approx(altitude)
####


def test_geodetic_unit_vectors_are_orthonormal() -> None:
    north, east, down = geodetic_unit_vectors(math.radians(12.0), math.radians(-33.0))

    def dot(left: tuple[float, float, float], right: tuple[float, float, float]) -> float:
        return sum(a * b for a, b in zip(left, right, strict=True))

    vectors = [
        (north.x, north.y, north.z),
        (east.x, east.y, east.z),
        (down.x, down.y, down.z),
    ]
    for vector in vectors:
        assert dot(vector, vector) == pytest.approx(1.0)
    assert dot(vectors[0], vectors[1]) == pytest.approx(0.0)
    assert dot(vectors[0], vectors[2]) == pytest.approx(0.0)
    assert dot(vectors[1], vectors[2]) == pytest.approx(0.0)
####


def test_geocentric_velocity_angles_and_transform_round_trip() -> None:
    components = geocentric_velocity_components_from_angles(750.0, math.radians(12.5), math.radians(67.0))
    angles = geocentric_velocity_angles_from_components(components)

    assert angles.speed == pytest.approx(750.0)
    assert angles.gamma_radians == pytest.approx(math.radians(12.5))
    assert angles.psi_radians == pytest.approx(math.radians(67.0))

    longitude = math.radians(25.0)
    latitude = math.radians(-18.0)
    ecfc = geocentric_velocity_to_ecfc(longitude, latitude, components)
    recovered = ecfc_to_geocentric_velocity(longitude, latitude, ecfc)

    assert recovered.x == pytest.approx(components.x)
    assert recovered.y == pytest.approx(components.y)
    assert recovered.z == pytest.approx(components.z)
####


def test_geocentric_velocity_zero_and_vertical_edge_cases() -> None:
    assert geocentric_velocity_angles_from_components(
        geocentric_velocity_components_from_angles(0.0, 0.0, 0.0)
    ) == VelocityAngles(0.0, 0.0, 0.0)

    vertical = geocentric_velocity_components_from_angles(100.0, math.radians(-90.0), math.radians(45.0))
    angles = geocentric_velocity_angles_from_components(vertical)

    assert angles.speed == pytest.approx(100.0)
    assert angles.gamma_radians == pytest.approx(math.radians(-90.0))
    assert angles.psi_radians == pytest.approx(0.0)
####


def test_geodetic_velocity_angles_and_transform_round_trip() -> None:
    components = geodetic_velocity_components_from_angles(820.0, math.radians(-8.0), math.radians(105.0))
    angles = geodetic_velocity_angles_from_components(components)

    assert angles.speed == pytest.approx(820.0)
    assert angles.gamma_radians == pytest.approx(math.radians(-8.0))
    assert angles.psi_radians == pytest.approx(math.radians(105.0))

    longitude = math.radians(-63.0)
    latitude = math.radians(41.0)
    ecfc = geodetic_velocity_to_ecfc(longitude, latitude, components)
    recovered = ecfc_to_geodetic_velocity(longitude, latitude, ecfc)

    assert recovered.x == pytest.approx(components.x)
    assert recovered.y == pytest.approx(components.y)
    assert recovered.z == pytest.approx(components.z)
####


def test_geocentric_unit_vectors_are_orthonormal() -> None:
    north, east, down = geocentric_unit_vectors(math.radians(18.0), math.radians(27.0))

    def dot(left: tuple[float, float, float], right: tuple[float, float, float]) -> float:
        return sum(a * b for a, b in zip(left, right, strict=True))

    vectors = [
        (north.x, north.y, north.z),
        (east.x, east.y, east.z),
        (down.x, down.y, down.z),
    ]
    for vector in vectors:
        assert dot(vector, vector) == pytest.approx(1.0)
    assert dot(vectors[0], vectors[1]) == pytest.approx(0.0)
    assert dot(vectors[0], vectors[2]) == pytest.approx(0.0)
    assert dot(vectors[1], vectors[2]) == pytest.approx(0.0)
####
