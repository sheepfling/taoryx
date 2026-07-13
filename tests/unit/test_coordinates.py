from __future__ import annotations

import math

import pytest

from taoryx.contracts import (
    Angle,
    Basis3,
    CoordinateOrder,
    FlightPathAngle,
    Frame,
    FrameQuantityVector3,
    FrameVector3,
    GeocentricCoordinates,
    GeodeticCoordinates,
    Heading,
    Latitude,
    Longitude,
    Quantity,
    Unit,
    Vector3,
)
from taoryx.coordinates import (
    ecfc_coords,
    ecfc_position_to_geocentric,
    ecfc_position_to_geodetic,
    ecfc_velocity_to_geocentric,
    ecfc_velocity_to_geodetic,
    ecic_coords,
    geocentric_position_to_ecfc,
    geocentric_unit_vectors,
    geocentric_velocity_to_ecfc,
    geodetic_position_to_ecfc,
    geodetic_unit_vectors,
    geodetic_velocity_to_ecfc,
    inertial_platform_coordinates,
    inertial_platform_to_ecic,
    tangent_plane_coordinates,
    tangent_plane_to_ecfc,
    tangent_plane_unit_vectors,
    velocity_frames,
)


def test_ecic_conversion_zero_rotation_preserves_framed_components() -> None:
    result = ecic_coords(
        FrameVector3(Vector3(10.0, -4.0, 3.0), Frame.ECFC),
        FrameVector3(Vector3(2.0, 5.0, -1.0), Frame.ECFC),
        FrameVector3(Vector3(12.0, -8.0, 4.0), Frame.ECFC),
        2.0,
        0.0,
        0.0,
        200.0,
        20.0,
    )

    assert result.rotation_angle_radians == 0.0
    assert result.position == FrameVector3(Vector3(10.0, -4.0, 3.0), Frame.ECIC)
    assert result.inertial_velocity == FrameVector3(Vector3(2.0, 5.0, -1.0), Frame.ECIC)
    assert result.inertial_acceleration == FrameVector3(Vector3(6.0, -4.0, 2.0), Frame.ECIC)
####


def test_ecic_conversion_rotates_position_and_preserves_norm() -> None:
    result = ecic_coords(
        FrameVector3(Vector3(1.0, 0.0, 2.0), Frame.ECFC),
        FrameVector3(Vector3(0.0, 1.0, 0.0), Frame.ECFC),
        FrameVector3(Vector3(0.0, 0.0, 0.0), Frame.ECFC),
        1.0,
        math.pi / 2.0,
        0.0,
        0.0,
        0.0,
    )

    assert result.position.vector.x == pytest.approx(0.0)
    assert result.position.vector.y == pytest.approx(1.0)
    assert result.position.vector.z == pytest.approx(2.0)
    assert result.position.vector.norm() == pytest.approx(Vector3(1.0, 0.0, 2.0).norm())
    assert result.position.frame is Frame.ECIC
    assert result.inertial_velocity.frame is Frame.ECIC
    assert result.inertial_acceleration.frame is Frame.ECIC
####


def test_ecic_conversion_rejects_wrong_frames_and_invalid_mass() -> None:
    vector = FrameVector3(Vector3(1.0, 0.0, 0.0), Frame.ECIC)
    ecfc = FrameVector3(Vector3(1.0, 0.0, 0.0), Frame.ECFC)
    with pytest.raises(ValueError, match="position must be expressed"):
        ecic_coords(vector, ecfc, ecfc, 1.0, 0.0, 0.0, 0.0, 0.0)
    with pytest.raises(ValueError, match="mass"):
        ecic_coords(ecfc, ecfc, ecfc, 0.0, 0.0, 0.0, 0.0, 0.0)
####


def test_derived_ecic_to_ecfc_inverse_round_trips_position_velocity_and_force() -> None:
    position = FrameVector3(Vector3(10.0, -4.0, 3.0), Frame.ECFC)
    velocity = FrameVector3(Vector3(2.0, 5.0, -1.0), Frame.ECFC)
    force = FrameVector3(Vector3(12.0, -8.0, 4.0), Frame.ECFC)
    forward = ecic_coords(position, velocity, force, 2.0, 0.25, 7.292115e-5, 200.0, 20.0)
    inverse = ecfc_coords(
        forward.position,
        forward.inertial_velocity,
        forward.inertial_acceleration,
        2.0,
        0.25,
        7.292115e-5,
        200.0,
        20.0,
    )

    assert inverse.position.vector.x == pytest.approx(position.vector.x)
    assert inverse.position.vector.y == pytest.approx(position.vector.y)
    assert inverse.earth_relative_velocity.vector.x == pytest.approx(velocity.vector.x)
    assert inverse.earth_relative_velocity.vector.y == pytest.approx(velocity.vector.y)
    assert inverse.earth_relative_velocity.vector.z == pytest.approx(velocity.vector.z)
    assert inverse.total_force.vector.x == pytest.approx(force.vector.x)
    assert inverse.total_force.vector.y == pytest.approx(force.vector.y)
    assert inverse.total_force.vector.z == pytest.approx(force.vector.z)
####


def test_geocentric_unit_vectors_return_framed_right_handed_basis() -> None:
    basis = geocentric_unit_vectors(Longitude(math.radians(18.0)), Latitude(math.radians(27.0)))

    assert basis.parent_frame is Frame.ECFC
    assert basis.child_frame is Frame.GEOCENTRIC_HORIZON
    assert basis.is_orthonormal()
    assert basis.first.cross(basis.second).x == pytest.approx(basis.third.x)
    assert basis.first.cross(basis.second).y == pytest.approx(basis.third.y)
    assert basis.first.cross(basis.second).z == pytest.approx(basis.third.z)


def test_geocentric_unit_vectors_handle_equator_and_pole_conventions() -> None:
    equator = geocentric_unit_vectors(Longitude(0.0), Latitude(0.0))
    pole = geocentric_unit_vectors(Longitude(1.2), Latitude(math.pi / 2.0))

    assert equator.first == Vector3(0.0, 0.0, 1.0)
    assert equator.second == Vector3(0.0, 1.0, 0.0)
    assert equator.third == Vector3(-1.0, 0.0, 0.0)
    assert pole.is_orthonormal()
    with pytest.raises(ValueError, match="latitude"):
        geocentric_unit_vectors(Longitude(0.0), Latitude(math.pi))
####


def test_geodetic_unit_vectors_are_framed_and_orthonormal() -> None:
    basis = geodetic_unit_vectors(Longitude(math.radians(18.0)), Latitude(math.radians(27.0)))

    assert basis.parent_frame is Frame.ECFC
    assert basis.child_frame is Frame.GEODETIC_HORIZON
    assert basis.is_orthonormal()
    assert basis.first.cross(basis.second) == basis.third
####


def test_geocentric_position_conversion_preserves_order_units_and_round_trips() -> None:
    source = GeocentricCoordinates(
        radius=Quantity(10.0, Unit.KILOMETER),
        longitude=Longitude(math.radians(90.0)),
        latitude=Latitude(math.radians(20.0)),
    )
    ecfc = geocentric_position_to_ecfc(source)
    recovered = ecfc_position_to_geocentric(ecfc)

    assert ecfc.frame is Frame.ECFC
    assert ecfc.unit is Unit.KILOMETER
    assert ecfc.vector.x == pytest.approx(0.0, abs=1e-12)
    assert ecfc.vector.y == pytest.approx(10.0 * math.cos(math.radians(20.0)))
    assert ecfc.vector.z == pytest.approx(10.0 * math.sin(math.radians(20.0)))
    assert recovered.radius.value == pytest.approx(10.0)
    assert recovered.radius.unit is Unit.KILOMETER
    assert recovered.longitude.radians == pytest.approx(source.longitude.radians)
    assert recovered.latitude.radians == pytest.approx(source.latitude.radians)
    assert recovered.as_tuple(CoordinateOrder.RADIUS_LONGITUDE_LATITUDE)[0] == recovered.radius
####


def test_ecfc_position_inverse_rejects_wrong_frame_or_dimension() -> None:
    with pytest.raises(ValueError, match="position must be expressed"):
        ecfc_position_to_geocentric(FrameQuantityVector3(Vector3(1.0, 0.0, 0.0), Frame.ECIC, Unit.KILOMETER))
    with pytest.raises(ValueError, match="length units"):
        ecfc_position_to_geocentric(FrameQuantityVector3(Vector3(1.0, 0.0, 0.0), Frame.ECFC, Unit.SECOND))
####


def test_geocentric_velocity_conversion_handles_cardinal_and_vertical_conventions() -> None:
    north = ecfc_velocity_to_geocentric(FrameVector3(Vector3(0.0, 0.0, 100.0), Frame.ECFC), Longitude(0.0), Latitude(0.0))
    east = ecfc_velocity_to_geocentric(FrameVector3(Vector3(0.0, 100.0, 0.0), Frame.ECFC), Longitude(0.0), Latitude(0.0))
    up = ecfc_velocity_to_geocentric(FrameVector3(Vector3(100.0, 0.0, 0.0), Frame.ECFC), Longitude(0.0), Latitude(0.0))
    zero = ecfc_velocity_to_geocentric(FrameVector3(Vector3(0.0, 0.0, 0.0), Frame.ECFC), Longitude(0.0), Latitude(0.0))

    assert north.speed == pytest.approx(100.0)
    assert north.flight_path_angle.radians == pytest.approx(0.0)
    assert north.heading.radians == pytest.approx(0.0)
    assert east.heading.radians == pytest.approx(math.pi / 2.0)
    assert up.flight_path_angle.radians == pytest.approx(math.pi / 2.0)
    assert up.heading.radians == pytest.approx(0.0)
    assert zero.speed == 0.0
    assert zero.flight_path_angle.radians == 0.0
    assert zero.heading.radians == 0.0
####


def test_derived_geocentric_velocity_inverse_round_trips_at_nontrivial_location() -> None:
    longitude = Longitude(math.radians(123.0))
    latitude = Latitude(math.radians(-38.0))
    source = geocentric_velocity_to_ecfc(7500.0, FlightPathAngle(0.2), Heading(-0.7), longitude, latitude)
    recovered = ecfc_velocity_to_geocentric(source, longitude, latitude)

    assert recovered.speed == pytest.approx(7500.0)
    assert recovered.flight_path_angle.radians == pytest.approx(0.2)
    assert recovered.heading.radians == pytest.approx(-0.7)
####


def test_geocentric_velocity_conversion_rejects_invalid_inputs() -> None:
    with pytest.raises(ValueError, match="velocity must be expressed"):
        ecfc_velocity_to_geocentric(FrameVector3(Vector3(1.0, 0.0, 0.0), Frame.ECIC), Longitude(0.0), Latitude(0.0))
    with pytest.raises(ValueError, match="speed"):
        geocentric_velocity_to_ecfc(-1.0, FlightPathAngle(0.0), Heading(0.0), Longitude(0.0), Latitude(0.0))
    with pytest.raises(ValueError, match="flight-path angle"):
        geocentric_velocity_to_ecfc(1.0, FlightPathAngle(math.pi), Heading(0.0), Longitude(0.0), Latitude(0.0))
####


def test_geodetic_position_conversion_round_trips_with_explicit_units() -> None:
    from taoryx.earth import resolve_ellipsoid_parameters

    parameters = resolve_ellipsoid_parameters(Quantity(6378.137, Unit.KILOMETER), flattening=1.0 / 298.257223563)
    source = GeodeticCoordinates(
        longitude=Longitude(math.radians(123.0)),
        latitude=Latitude(math.radians(-38.0)),
        altitude=Quantity(12.5, Unit.KILOMETER),
    )
    ecfc = geodetic_position_to_ecfc(source, parameters)
    recovered = ecfc_position_to_geodetic(ecfc, parameters)

    assert ecfc.frame is Frame.ECFC
    assert ecfc.unit is Unit.KILOMETER
    assert recovered.longitude.radians == pytest.approx(source.longitude.radians, abs=1e-12)
    assert recovered.latitude.radians == pytest.approx(source.latitude.radians, abs=1e-12)
    assert recovered.altitude.value == pytest.approx(source.altitude.value, abs=1e-7)
    assert recovered.altitude.unit is Unit.KILOMETER
####


def test_geodetic_velocity_conversion_round_trips_and_uses_geodetic_frame() -> None:
    longitude = Longitude(math.radians(123.0))
    latitude = Latitude(math.radians(-38.0))
    source = geodetic_velocity_to_ecfc(7500.0, FlightPathAngle(0.2), Heading(-0.7), longitude, latitude)
    recovered = ecfc_velocity_to_geodetic(source, longitude, latitude)

    assert recovered.speed == pytest.approx(7500.0)
    assert recovered.flight_path_angle.radians == pytest.approx(0.2)
    assert recovered.heading.radians == pytest.approx(-0.7)
####


def test_geodetic_position_inverse_rejects_origin_and_nonconvergence() -> None:
    from taoryx.earth import resolve_ellipsoid_parameters

    parameters = resolve_ellipsoid_parameters(Quantity(6378.137, Unit.KILOMETER), flattening=1.0 / 298.257223563)
    with pytest.raises(ValueError, match="origin"):
        ecfc_position_to_geodetic(FrameQuantityVector3(Vector3(0.0, 0.0, 0.0), Frame.ECFC, Unit.KILOMETER), parameters)
    position = geodetic_position_to_ecfc(
        GeodeticCoordinates(Longitude(0.0), Latitude(0.3), Quantity(1.0, Unit.KILOMETER)), parameters
    )
    with pytest.raises(RuntimeError, match="did not converge"):
        ecfc_position_to_geodetic(position, parameters, max_iterations=1)
####


def test_velocity_frames_build_earth_and_wind_bases_with_explicit_frames() -> None:
    reference = geodetic_unit_vectors(Longitude(0.4), Latitude(0.3))
    earth_velocity = FrameVector3(Vector3(3000.0, 1000.0, 500.0), Frame.ECFC)
    wind_velocity = FrameVector3(Vector3(30.0, -20.0, 10.0), Frame.ECFC)
    frames = velocity_frames(earth_velocity, wind_velocity, reference, Angle(0.2))

    assert frames.earth_relative.child_frame is Frame.VELOCITY_EARTH
    assert frames.wind_relative.child_frame is Frame.WIND
    assert frames.earth_relative.parent_frame is Frame.ECFC
    assert frames.wind_relative.is_orthonormal()
    assert frames.wind_relative.first.dot(earth_velocity.vector - wind_velocity.vector) > 0.0
####


def test_velocity_frames_reject_zero_or_vertical_relative_velocity() -> None:
    reference = geodetic_unit_vectors(Longitude(0.0), Latitude(0.0))
    zero = FrameVector3(Vector3(0.0, 0.0, 0.0), Frame.ECFC)
    vertical = FrameVector3(reference.third.scaled(10.0), Frame.ECFC)
    with pytest.raises(ValueError, match="zero velocity"):
        velocity_frames(zero, zero, reference, Angle(0.0))
    with pytest.raises(ValueError, match="vertical velocity"):
        velocity_frames(vertical, zero, reference, Angle(0.0))
####


def test_inertial_platform_projection_and_inverse_round_trip() -> None:
    platform_basis = Basis3(
        Vector3(0.0, 1.0, 0.0),
        Vector3(-1.0, 0.0, 0.0),
        Vector3(0.0, 0.0, 1.0),
        Frame.ECIC,
        Frame.INERTIAL_PLATFORM,
    )
    origin = FrameVector3(Vector3(100.0, 200.0, -50.0), Frame.ECIC)
    position = FrameVector3(Vector3(110.0, 180.0, -47.0), Frame.ECIC)
    velocity = FrameVector3(Vector3(2.0, 3.0, 4.0), Frame.ECIC)
    acceleration = FrameVector3(Vector3(-1.0, 5.0, 0.5), Frame.ECIC)
    projected = inertial_platform_coordinates(position, velocity, acceleration, origin, platform_basis)
    recovered = inertial_platform_to_ecic(projected, origin, platform_basis)

    assert projected.position.vector == Vector3(-20.0, -10.0, 3.0)
    assert recovered[0].vector == position.vector
    assert recovered[1].vector.x == pytest.approx(velocity.vector.x)
    assert recovered[1].vector.y == pytest.approx(velocity.vector.y)
    assert recovered[1].vector.z == pytest.approx(velocity.vector.z)
    assert recovered[2].vector.x == pytest.approx(acceleration.vector.x)
    assert recovered[2].vector.y == pytest.approx(acceleration.vector.y)
    assert recovered[2].vector.z == pytest.approx(acceleration.vector.z)
####


def test_tangent_plane_basis_and_state_projection_round_trip_preserve_units() -> None:
    basis = tangent_plane_unit_vectors(Longitude(0.4), Latitude(0.3), Angle(0.2))
    origin = FrameQuantityVector3(Vector3(1000.0, -2000.0, 3000.0), Frame.ECFC, Unit.KILOMETER)
    position = FrameQuantityVector3(Vector3(1010.0, -1980.0, 2995.0), Frame.ECFC, Unit.KILOMETER)
    velocity = FrameQuantityVector3(Vector3(2.0, 3.0, -1.0), Frame.ECFC, Unit.METER_PER_SECOND)
    acceleration = FrameQuantityVector3(Vector3(0.1, -0.2, 0.3), Frame.ECFC, Unit.METER_PER_SECOND_SQUARED)
    projected = tangent_plane_coordinates(position, velocity, acceleration, origin, basis)
    recovered = tangent_plane_to_ecfc(projected, origin, basis)

    assert basis.child_frame is Frame.TANGENT_PLANE
    assert basis.is_orthonormal()
    assert recovered[0].vector.x == pytest.approx(position.vector.x)
    assert recovered[0].vector.y == pytest.approx(position.vector.y)
    assert recovered[0].vector.z == pytest.approx(position.vector.z)
    assert recovered[0].unit is Unit.KILOMETER
    assert recovered[1].vector.x == pytest.approx(velocity.vector.x)
    assert recovered[1].vector.y == pytest.approx(velocity.vector.y)
    assert recovered[1].vector.z == pytest.approx(velocity.vector.z)
    assert recovered[1].unit is Unit.METER_PER_SECOND
    assert recovered[2].vector.x == pytest.approx(acceleration.vector.x)
    assert recovered[2].vector.y == pytest.approx(acceleration.vector.y)
    assert recovered[2].vector.z == pytest.approx(acceleration.vector.z)
####
