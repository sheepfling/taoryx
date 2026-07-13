from __future__ import annotations

import math

import pytest

from taoryx.contracts import (
    FlightPathAngle,
    Frame,
    FrameQuantityVector3,
    FrameVector3,
    GeodeticCoordinates,
    Heading,
    Latitude,
    Longitude,
    Quantity,
    Unit,
    Vector3,
)
from taoryx.coordinates import (
    ecfc_position_to_geodetic,
    geodetic_position_to_ecfc,
    geodetic_unit_vectors,
    geodetic_velocity_to_ecfc,
)
from taoryx.earth import ellipsoidal_surface_geometry, resolve_ellipsoid_parameters
from taoryx.state_rates import (
    altitude_acceleration,
    dynamic_pressure_derivatives,
    flight_path_angle_rates,
    geodetic_latitude_and_altitude_rates,
    ground_speed,
    longitude_and_geocentric_latitude_rates,
    mach_rate,
)


def test_geocentric_rates_match_east_and_north_motion() -> None:
    radius = 100.0
    position = FrameQuantityVector3(Vector3(radius, 0.0, 0.0), Frame.ECFC, Unit.KILOMETER)
    east = FrameVector3(Vector3(0.0, 5.0, 0.0), Frame.ECFC)
    north = FrameVector3(Vector3(0.0, 0.0, 5.0), Frame.ECFC)

    east_rates = longitude_and_geocentric_latitude_rates(position, east)
    north_rates = longitude_and_geocentric_latitude_rates(position, north)

    assert east_rates.longitude_rate == pytest.approx(5.0 / radius)
    assert east_rates.latitude_rate == pytest.approx(0.0)
    assert north_rates.longitude_rate == pytest.approx(0.0)
    assert north_rates.latitude_rate == pytest.approx(5.0 / radius)
####


def test_geocentric_longitude_rate_uses_zero_axis_convention() -> None:
    position = FrameQuantityVector3(Vector3(0.0, 0.0, 100.0), Frame.ECFC, Unit.KILOMETER)
    rates = longitude_and_geocentric_latitude_rates(position, FrameVector3(Vector3(1.0, 2.0, 3.0), Frame.ECFC))

    assert rates.longitude_rate == 0.0
####


def test_geodetic_rates_match_north_and_up_motion() -> None:
    parameters = resolve_ellipsoid_parameters(Quantity(6378.137, Unit.KILOMETER), flattening=1.0 / 298.257223563)
    source = geodetic_position_to_ecfc(
        GeodeticCoordinates(
            Longitude(math.radians(35.0)), Latitude(math.radians(42.0)), Quantity(10.0, Unit.KILOMETER)
        ),
        parameters,
    )
    geodetic = ecfc_position_to_geodetic(source, parameters)
    basis = geodetic_unit_vectors(geodetic.longitude, geodetic.latitude)
    velocity = geodetic_velocity_to_ecfc(100.0, FlightPathAngle(0.0), Heading(0.0), geodetic.longitude, geodetic.latitude)
    rates = geodetic_latitude_and_altitude_rates(source, velocity, parameters)
    geometry = ellipsoidal_surface_geometry(parameters, geodetic.latitude)
    sine = math.sin(geodetic.latitude.radians)
    denominator = 10.0 + geometry.normal_distance.value * (1.0 - parameters.eccentricity**2) / (1.0 - parameters.eccentricity**2 * sine**2)

    assert rates.latitude_rate == pytest.approx(100.0 / denominator)
    assert rates.altitude_rate == pytest.approx(0.0)
    assert basis.is_orthonormal()
####


def test_state_rates_reject_wrong_frames_and_origin() -> None:
    position = FrameQuantityVector3(Vector3(0.0, 0.0, 0.0), Frame.ECFC, Unit.KILOMETER)
    velocity = FrameVector3(Vector3(1.0, 0.0, 0.0), Frame.ECFC)
    with pytest.raises(ValueError, match="origin"):
        longitude_and_geocentric_latitude_rates(position, velocity)
    with pytest.raises(ValueError, match="position"):
        longitude_and_geocentric_latitude_rates(FrameQuantityVector3(Vector3(1.0, 0.0, 0.0), Frame.ECIC, Unit.KILOMETER), velocity)
####


def test_altitude_acceleration_matches_down_vector_derivative() -> None:
    parameters = resolve_ellipsoid_parameters(Quantity(6378.137, Unit.KILOMETER), flattening=1.0 / 298.257223563)
    source = geodetic_position_to_ecfc(
        GeodeticCoordinates(Longitude(math.radians(35.0)), Latitude(math.radians(42.0)), Quantity(10.0, Unit.KILOMETER)),
        parameters,
    )
    velocity = FrameVector3(Vector3(80.0, -20.0, 15.0), Frame.ECFC)
    acceleration = FrameVector3(Vector3(2.0, 3.0, -4.0), Frame.ECFC)
    geodetic = ecfc_position_to_geodetic(source, parameters)
    rates = geodetic_latitude_and_altitude_rates(source, velocity, parameters)
    projection = math.hypot(source.vector.x, source.vector.y)
    longitude_rate = (velocity.vector.y * source.vector.x - velocity.vector.x * source.vector.y) / (projection * projection)
    longitude = geodetic.longitude.radians
    latitude = geodetic.latitude.radians
    down = geodetic_unit_vectors(geodetic.longitude, geodetic.latitude).third
    down_derivative = Vector3(
        math.sin(latitude) * math.cos(longitude) * rates.latitude_rate + math.cos(latitude) * math.sin(longitude) * longitude_rate,
        math.sin(latitude) * math.sin(longitude) * rates.latitude_rate - math.cos(latitude) * math.cos(longitude) * longitude_rate,
        -math.cos(latitude) * rates.latitude_rate,
    )
    expected = -acceleration.vector.dot(down) - velocity.vector.dot(down_derivative)

    assert altitude_acceleration(source, velocity, acceleration, parameters) == pytest.approx(expected)
####


def test_altitude_acceleration_rejects_wrong_acceleration_frame() -> None:
    parameters = resolve_ellipsoid_parameters(Quantity(6378.137, Unit.KILOMETER), flattening=1.0 / 298.257223563)
    position = FrameQuantityVector3(Vector3(6378.137, 0.0, 0.0), Frame.ECFC, Unit.KILOMETER)
    velocity = FrameVector3(Vector3(0.0, 1.0, 0.0), Frame.ECFC)
    acceleration = FrameVector3(Vector3(0.0, 0.0, 1.0), Frame.ECIC)

    with pytest.raises(ValueError, match="acceleration"):
        altitude_acceleration(position, velocity, acceleration, parameters)
####


def test_dynamic_pressure_derivatives_follow_manual_approximation() -> None:
    result = dynamic_pressure_derivatives(1.0, 2.0, 3.0, 4.0, 5.0, 6.0)

    assert result.pressure == pytest.approx(8.0)
    assert result.pressure_rate == pytest.approx(36.0)
    assert result.pressure_acceleration == pytest.approx(140.0)
####


def test_dynamic_pressure_derivatives_reject_nonfinite_inputs() -> None:
    with pytest.raises(ValueError, match="finite"):
        dynamic_pressure_derivatives(math.inf, 0.0, 0.0, 1.0, 0.0, 0.0)
####


def test_mach_rate_matches_quotient_rule() -> None:
    assert mach_rate(340.0, 10.0, 320.0, 2.0) == pytest.approx((10.0 * 320.0 - 340.0 * 2.0) / 320.0**2)
####


def test_mach_rate_rejects_zero_speed_of_sound() -> None:
    with pytest.raises(ValueError, match="speed of sound"):
        mach_rate(1.0, 2.0, 0.0, 0.0)
####


def test_ground_speed_is_tangent_for_surface_motion() -> None:
    parameters = resolve_ellipsoid_parameters(Quantity(6378.137, Unit.KILOMETER), flattening=1.0 / 298.257223563)
    source = geodetic_position_to_ecfc(
        GeodeticCoordinates(Longitude(math.radians(10.0)), Latitude(math.radians(25.0)), Quantity(0.0, Unit.KILOMETER)),
        parameters,
    )
    velocity = geodetic_velocity_to_ecfc(100.0, FlightPathAngle(0.0), Heading(0.0), Longitude(math.radians(10.0)), Latitude(math.radians(25.0)))

    result = ground_speed(source, velocity, parameters)
    down = geodetic_unit_vectors(Longitude(math.radians(10.0)), Latitude(math.radians(25.0))).third

    assert result.vector.frame is Frame.ECFC
    assert result.vector.vector.dot(down) == pytest.approx(0.0, abs=1e-10)
    assert result.magnitude == pytest.approx(result.vector.vector.norm())
####


def test_flight_path_angle_rates_match_scalar_equation_registry() -> None:
    parameters = resolve_ellipsoid_parameters(Quantity(6378.137, Unit.KILOMETER), flattening=1.0 / 298.257223563)
    position = FrameQuantityVector3(Vector3(6378.137, 0.0, 0.0), Frame.ECFC, Unit.KILOMETER)
    velocity = FrameVector3(Vector3(0.0, 300.0, 100.0), Frame.ECFC)
    acceleration = FrameVector3(Vector3(-2.0, 4.0, 1.0), Frame.ECFC)
    result = flight_path_angle_rates(position, velocity, acceleration, parameters)

    from taoryx.equations import geocentric_flight_path_rates_from_ecfc, geodetic_flight_path_rates_from_ecfc

    expected_geocentric = geocentric_flight_path_rates_from_ecfc(position.vector, velocity.vector, acceleration.vector)
    expected_geodetic = geodetic_flight_path_rates_from_ecfc(
        parameters.equatorial_radius.value,
        parameters.eccentricity,
        position.vector,
        velocity.vector,
        acceleration.vector,
    )
    assert (result.geocentric_vertical_rate, result.geocentric_heading_rate) == pytest.approx(expected_geocentric)
    assert (result.geodetic_vertical_rate, result.geodetic_heading_rate) == pytest.approx(expected_geodetic)
####
