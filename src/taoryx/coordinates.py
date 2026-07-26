"""Typed coordinate-conversion algorithms from the TAOS catalog."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .contracts import (
    Angle,
    Basis3,
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
    Vector3,
)
from .earth import EllipsoidParameters, ellipsoidal_surface_geometry
from .equations.frames import ecfc_to_ecic_acceleration, ecfc_to_ecic_position, ecfc_to_ecic_velocity, ecic_rotation_angle
from .equations.geodesy import CartesianVector3
from .equations.geodesy import ecfc_to_geocentric_position as _legacy_ecfc_to_geocentric_position
from .equations.geodesy import geocentric_position_to_ecfc as _legacy_geocentric_position_to_ecfc
from .equations.geodesy import geocentric_unit_vectors as _legacy_geocentric_unit_vectors
from .equations.geodesy import tangent_plane_unit_vectors as _legacy_tangent_plane_unit_vectors


@dataclass(frozen=True, slots=True)
class EcicCoordinates:
    """ECIC position, inertial velocity, and inertial acceleration outputs."""

    rotation_angle_radians: float
    position: FrameVector3
    inertial_velocity: FrameVector3
    inertial_acceleration: FrameVector3


@dataclass(frozen=True, slots=True)
class EcfcCoordinates:
    """ECFC position, earth-relative velocity, and total force outputs."""

    rotation_angle_radians: float
    position: FrameVector3
    earth_relative_velocity: FrameVector3
    total_force: FrameVector3


@dataclass(frozen=True, slots=True)
class GeocentricVelocityCoordinates:
    """Speed, flight-path angle, and heading in the geocentric horizon frame."""

    speed: float
    flight_path_angle: FlightPathAngle
    heading: Heading

    def __post_init__(self) -> None:
        if not math.isfinite(self.speed) or self.speed < 0.0:
            raise ValueError("speed must be finite and non-negative")
        ####
    ####


@dataclass(frozen=True, slots=True)
class GeodeticVelocityCoordinates:
    """Speed, flight-path angle, and heading in the geodetic horizon frame."""

    speed: float
    flight_path_angle: FlightPathAngle
    heading: Heading

    def __post_init__(self) -> None:
        if not math.isfinite(self.speed) or self.speed < 0.0:
            raise ValueError("speed must be finite and non-negative")
        ####
    ####


@dataclass(frozen=True, slots=True)
class VelocityFrames:
    """Earth-relative and wind-relative velocity bases."""

    earth_relative: Basis3
    wind_relative: Basis3


@dataclass(frozen=True, slots=True)
class InertialPlatformCoordinates:
    """Position, inertial velocity, and acceleration in platform components."""

    position: FrameVector3
    inertial_velocity: FrameVector3
    inertial_acceleration: FrameVector3


@dataclass(frozen=True, slots=True)
class TangentPlaneCoordinates:
    """Position, velocity, and acceleration relative to a tangent origin."""

    position: FrameQuantityVector3
    earth_relative_velocity: FrameQuantityVector3
    earth_relative_acceleration: FrameQuantityVector3


def geocentric_unit_vectors(longitude: Longitude, latitude: Latitude) -> Basis3:
    """Build the local geocentric north/east/down basis from equation 2-6.

    This implements ``TAOS-ALG-COORD-002``. The returned basis expresses the
    geocentric horizon child frame in ECFC components.
    """

    legacy_basis = _legacy_geocentric_unit_vectors(longitude.radians, latitude.radians)
    return Basis3(
        _as_vector3(legacy_basis[0]),
        _as_vector3(legacy_basis[1]),
        _as_vector3(legacy_basis[2]),
        parent_frame=Frame.ECFC,
        child_frame=Frame.GEOCENTRIC_HORIZON,
    )
####


def geodetic_unit_vectors(longitude: Longitude, latitude: Latitude) -> Basis3:
    """Build the local geodetic north/east/down basis from equations 2-19--2-21.

    This implements ``TAOS-ALG-COORD-008``. The basis is expressed in ECFC
    components and differs from the geocentric basis when the latitudes differ.
    """

    cosine_latitude = math.cos(latitude.radians)
    sine_latitude = math.sin(latitude.radians)
    cosine_longitude = math.cos(longitude.radians)
    sine_longitude = math.sin(longitude.radians)
    first = Vector3(-sine_latitude * cosine_longitude, -sine_latitude * sine_longitude, cosine_latitude)
    second = Vector3(-sine_longitude, cosine_longitude, 0.0)
    return Basis3(
        first,
        second,
        first.cross(second),
        parent_frame=Frame.ECFC,
        child_frame=Frame.GEODETIC_HORIZON,
    )
####


def geocentric_position_to_ecfc(position: GeocentricCoordinates) -> FrameQuantityVector3:
    """Convert radius/longitude/latitude to a unit-preserving ECFC vector.

    This implements ``TAOS-ALG-COORD-003`` and equation 2-7. Cartesian
    components retain the input radius unit and are labeled as ECFC.
    """

    legacy = _legacy_geocentric_position_to_ecfc(
        position.radius.value,
        position.longitude.radians,
        position.latitude.radians,
    )
    return FrameQuantityVector3(_as_vector3(legacy), Frame.ECFC, position.radius.unit)
####


def ecfc_position_to_geocentric(position: FrameQuantityVector3) -> GeocentricCoordinates:
    """Convert a unit-preserving ECFC position to radius/longitude/latitude.

    This is the derived inverse paired with ``TAOS-ALG-COORD-003``. The
    longitude/latitude fields remain named values rather than an ambiguous
    tuple, and the radius retains the Cartesian input unit.
    """

    _require_quantity_frame(position, Frame.ECFC, "position")
    legacy = _legacy_ecfc_to_geocentric_position(position.vector.x, position.vector.y, position.vector.z)
    return GeocentricCoordinates(
        Quantity(legacy.radius, position.unit),
        Longitude(legacy.longitude_radians),
        Latitude(legacy.latitude_radians),
    )
####


def geodetic_position_to_ecfc(
    position: GeodeticCoordinates,
    parameters: EllipsoidParameters,
) -> FrameQuantityVector3:
    """Convert geodetic longitude/latitude/altitude to ECFC coordinates.

    This implements ``TAOS-ALG-COORD-010`` and equations 2-30 through 2-33,
    using the surface geometry from ``TAOS-ALG-COORD-009``.
    """

    altitude = position.altitude.to(parameters.equatorial_radius.unit)
    geometry = ellipsoidal_surface_geometry(parameters, position.latitude)
    radius = parameters.equatorial_radius.unit
    cosine_longitude = math.cos(position.longitude.radians)
    sine_longitude = math.sin(position.longitude.radians)
    equatorial_projection = geometry.surface_equatorial_radius.value + altitude.value * math.cos(position.latitude.radians)
    vector = Vector3(
        equatorial_projection * cosine_longitude,
        equatorial_projection * sine_longitude,
        geometry.surface_polar_coordinate.value + altitude.value * math.sin(position.latitude.radians),
    )
    return FrameQuantityVector3(vector, Frame.ECFC, radius)
####


def ecfc_position_to_geodetic(
    position: FrameQuantityVector3,
    parameters: EllipsoidParameters,
    *,
    tolerance: float = 1e-8,
    max_iterations: int = 25,
) -> GeodeticCoordinates:
    """Invert geodetic position using the manual's bounded fixed-point iteration.

    This implements ``TAOS-ALG-COORD-011`` and equations 2-34 through 2-41.
    ``tolerance`` is expressed in the input position unit.
    """

    _require_quantity_frame(position, Frame.ECFC, "position")
    if not math.isfinite(tolerance) or tolerance <= 0.0:
        raise ValueError("tolerance must be positive and finite")
    if not isinstance(max_iterations, int) or max_iterations <= 0:
        raise ValueError("max_iterations must be positive")
    normalized = position.to(parameters.equatorial_radius.unit)
    convergence_tolerance = tolerance * position.unit.scale_to_si / parameters.equatorial_radius.unit.scale_to_si
    x, y, z = normalized.vector.x, normalized.vector.y, normalized.vector.z
    radius = math.sqrt(x * x + y * y + z * z)
    if radius == 0.0:
        raise ValueError("geodetic position is undefined at the origin")
    longitude = Longitude(math.atan2(y, x))
    projected_radius = math.hypot(x, y)
    eccentricity_squared = parameters.eccentricity * parameters.eccentricity
    geocentric_sine = z / radius
    intercept = parameters.equatorial_radius.value * eccentricity_squared * geocentric_sine
    last_intercept = math.inf
    latitude = math.asin(max(-1.0, min(1.0, geocentric_sine)))
    altitude = 0.0
    iterations = 0
    while abs(intercept - last_intercept) > convergence_tolerance:
        if iterations >= max_iterations:
            raise RuntimeError("geodetic inverse iteration did not converge")
        last_intercept = intercept
        displaced_z = z + intercept
        radius_plus_altitude = math.hypot(projected_radius, displaced_z)
        latitude = math.asin(max(-1.0, min(1.0, displaced_z / radius_plus_altitude)))
        geometry = ellipsoidal_surface_geometry(parameters, Latitude(latitude))
        altitude = radius_plus_altitude - geometry.normal_distance.value
        intercept = geometry.axis_intercept.value
        iterations += 1
    ####
    output_altitude = Quantity(
        altitude * parameters.equatorial_radius.unit.scale_to_si / position.unit.scale_to_si,
        position.unit,
    )
    return GeodeticCoordinates(longitude, Latitude(latitude), output_altitude)
####


def ecfc_velocity_to_geocentric(
    velocity: FrameVector3,
    longitude: Longitude,
    latitude: Latitude,
) -> GeocentricVelocityCoordinates:
    """Resolve an ECFC velocity into geocentric horizon angles.

    This implements ``TAOS-ALG-COORD-005`` and equations 2-10 through 2-14.
    A zero-speed or purely vertical vector uses heading zero because its
    horizontal direction is undefined, matching the manual's convention.
    """

    _require_frame(velocity, Frame.ECFC, "velocity")
    basis = geocentric_unit_vectors(longitude, latitude)
    components = velocity.vector
    north = components.dot(basis.first)
    east = components.dot(basis.second)
    down = components.dot(basis.third)
    speed = math.sqrt(north * north + east * east + down * down)
    if speed == 0.0:
        return GeocentricVelocityCoordinates(0.0, FlightPathAngle(0.0), Heading(0.0))
    flight_path_angle = math.asin(max(-1.0, min(1.0, -down / speed)))
    horizontal_speed = math.hypot(north, east)
    heading = 0.0 if horizontal_speed <= 1e-12 * max(1.0, speed) else math.atan2(east, north)
    return GeocentricVelocityCoordinates(speed, FlightPathAngle(flight_path_angle), Heading(heading))
####


def geocentric_velocity_to_ecfc(
    speed: float,
    flight_path_angle: FlightPathAngle,
    heading: Heading,
    longitude: Longitude,
    latitude: Latitude,
) -> FrameVector3:
    """Construct an ECFC velocity from geocentric horizon components.

    This is the derived inverse paired with ``TAOS-ALG-COORD-005`` and
    implements equations 2-15 through 2-17 from ``TAOS-ALG-COORD-006``.
    The down component is negative for a positive flight-path angle.
    """

    if not math.isfinite(speed) or speed < 0.0:
        raise ValueError("speed must be finite and non-negative")
    if not -math.pi / 2.0 <= flight_path_angle.radians <= math.pi / 2.0:
        raise ValueError("flight-path angle must be within [-pi/2, pi/2]")
    basis = geocentric_unit_vectors(longitude, latitude)
    horizontal_speed = speed * math.cos(flight_path_angle.radians)
    north = horizontal_speed * math.cos(heading.radians)
    east = horizontal_speed * math.sin(heading.radians)
    down = -speed * math.sin(flight_path_angle.radians)
    vector = basis.first.scaled(north) + basis.second.scaled(east) + basis.third.scaled(down)
    return FrameVector3(vector, Frame.ECFC)
####


def ecfc_velocity_to_geodetic(
    velocity: FrameVector3,
    longitude: Longitude,
    latitude: Latitude,
) -> GeodeticVelocityCoordinates:
    """Resolve an ECFC velocity into geodetic horizon angles.

    This implements ``TAOS-ALG-COORD-012`` and equations 2-42 through 2-46.
    Zero-speed and purely vertical vectors use heading zero because their
    horizontal direction is undefined.
    """

    _require_frame(velocity, Frame.ECFC, "velocity")
    basis = geodetic_unit_vectors(longitude, latitude)
    north = velocity.vector.dot(basis.first)
    east = velocity.vector.dot(basis.second)
    down = velocity.vector.dot(basis.third)
    speed = math.sqrt(north * north + east * east + down * down)
    if speed == 0.0:
        return GeodeticVelocityCoordinates(0.0, FlightPathAngle(0.0), Heading(0.0))
    flight_path_angle = math.asin(max(-1.0, min(1.0, -down / speed)))
    horizontal_speed = math.hypot(north, east)
    heading = 0.0 if horizontal_speed <= 1e-12 * max(1.0, speed) else math.atan2(east, north)
    return GeodeticVelocityCoordinates(speed, FlightPathAngle(flight_path_angle), Heading(heading))
####


def geodetic_velocity_to_ecfc(
    speed: float,
    flight_path_angle: FlightPathAngle,
    heading: Heading,
    longitude: Longitude,
    latitude: Latitude,
) -> FrameVector3:
    """Construct an ECFC velocity from geodetic horizon components.

    This implements ``TAOS-ALG-COORD-013`` and equations 2-47 through 2-49.
    """

    if not math.isfinite(speed) or speed < 0.0:
        raise ValueError("speed must be finite and non-negative")
    if not -math.pi / 2.0 <= flight_path_angle.radians <= math.pi / 2.0:
        raise ValueError("flight-path angle must be within [-pi/2, pi/2]")
    basis = geodetic_unit_vectors(longitude, latitude)
    horizontal_speed = speed * math.cos(flight_path_angle.radians)
    north = horizontal_speed * math.cos(heading.radians)
    east = horizontal_speed * math.sin(heading.radians)
    down = -speed * math.sin(flight_path_angle.radians)
    vector = basis.first.scaled(north) + basis.second.scaled(east) + basis.third.scaled(down)
    return FrameVector3(vector, Frame.ECFC)
####


def velocity_frames(
    earth_relative_velocity: FrameVector3,
    wind_velocity: FrameVector3,
    reference_basis: Basis3,
    bank: Angle,
) -> VelocityFrames:
    """Build earth-relative and wind-relative velocity frames.

    This implements ``TAOS-ALG-COORD-016`` and equations 2-62 through 2-70.
    The wind vector is subtracted from earth-relative velocity. Both velocity
    frames use the reference horizon down axis to resolve their lateral axis;
    therefore zero or vertical relative velocity is undefined.
    """

    _require_reference_horizon(reference_basis)
    _require_frame(earth_relative_velocity, Frame.ECFC, "earth-relative velocity")
    _require_frame(wind_velocity, Frame.ECFC, "wind velocity")
    earth_basis = _velocity_basis(earth_relative_velocity.vector, reference_basis, Frame.VELOCITY_EARTH)
    air_relative = earth_relative_velocity.vector - wind_velocity.vector
    air_basis = _velocity_basis(air_relative, reference_basis, Frame.WIND)
    cosine_bank = math.cos(bank.radians)
    sine_bank = math.sin(bank.radians)
    wind_basis = Basis3(
        air_basis.first,
        air_basis.second.scaled(cosine_bank) + air_basis.third.scaled(sine_bank),
        air_basis.second.scaled(-sine_bank) + air_basis.third.scaled(cosine_bank),
        air_basis.parent_frame,
        Frame.WIND,
    )
    return VelocityFrames(earth_basis, wind_basis)
####


def _velocity_basis(vector: Vector3, reference_basis: Basis3, child_frame: Frame) -> Basis3:
    speed = vector.norm()
    if speed <= 0.0:
        raise ValueError("velocity frame is undefined for zero velocity")
    x_axis = vector.scaled(1.0 / speed)
    lateral = reference_basis.third.cross(x_axis)
    lateral_norm = lateral.norm()
    if lateral_norm <= 1e-12:
        raise ValueError("velocity frame is undefined for vertical velocity")
    y_axis = lateral.scaled(1.0 / lateral_norm)
    z_axis = x_axis.cross(y_axis)
    return Basis3(x_axis, y_axis, z_axis, reference_basis.parent_frame, child_frame)
####


def _require_reference_horizon(basis: Basis3) -> None:
    if basis.parent_frame is not Frame.ECFC or basis.child_frame not in (Frame.GEOCENTRIC_HORIZON, Frame.GEODETIC_HORIZON):
        raise ValueError("reference basis must be an ECFC local horizon basis")
    if not basis.is_orthonormal():
        raise ValueError("reference basis must be orthonormal")
    ####


def inertial_platform_coordinates(
    position: FrameVector3,
    inertial_velocity: FrameVector3,
    inertial_acceleration: FrameVector3,
    platform_origin: FrameVector3,
    platform_basis: Basis3,
) -> InertialPlatformCoordinates:
    """Project ECIC state quantities into an inertial platform.

    This implements ``TAOS-ALG-COORD-018``. The platform origin and basis are
    fixed in ECIC; position is first translated by the platform origin and all
    three quantities are then projected into platform components.
    """

    _require_frame(position, Frame.ECIC, "position")
    _require_frame(inertial_velocity, Frame.ECIC, "inertial velocity")
    _require_frame(inertial_acceleration, Frame.ECIC, "inertial acceleration")
    _require_frame(platform_origin, Frame.ECIC, "platform origin")
    _require_platform_basis(platform_basis)
    relative_position = position.vector - platform_origin.vector
    return InertialPlatformCoordinates(
        FrameVector3(_project(relative_position, platform_basis), Frame.INERTIAL_PLATFORM),
        FrameVector3(_project(inertial_velocity.vector, platform_basis), Frame.INERTIAL_PLATFORM),
        FrameVector3(_project(inertial_acceleration.vector, platform_basis), Frame.INERTIAL_PLATFORM),
    )
####


def inertial_platform_to_ecic(
    coordinates: InertialPlatformCoordinates,
    platform_origin: FrameVector3,
    platform_basis: Basis3,
) -> tuple[FrameVector3, FrameVector3, FrameVector3]:
    """Expand platform components back into ECIC state quantities.

    This is the derived inverse of ``TAOS-ALG-COORD-018`` and is valid for an
    orthonormal platform basis.
    """

    _require_frame(platform_origin, Frame.ECIC, "platform origin")
    _require_platform_basis(platform_basis)
    for vector, name in (
        (coordinates.position, "platform position"),
        (coordinates.inertial_velocity, "platform inertial velocity"),
        (coordinates.inertial_acceleration, "platform inertial acceleration"),
    ):
        if vector.frame is not Frame.INERTIAL_PLATFORM:
            raise ValueError(f"{name} must be expressed in {Frame.INERTIAL_PLATFORM}")
        ####
    return (
        FrameVector3(_expand(coordinates.position.vector, platform_basis) + platform_origin.vector, Frame.ECIC),
        FrameVector3(_expand(coordinates.inertial_velocity.vector, platform_basis), Frame.ECIC),
        FrameVector3(_expand(coordinates.inertial_acceleration.vector, platform_basis), Frame.ECIC),
    )
####


def tangent_plane_unit_vectors(longitude: Longitude, latitude: Latitude, azimuth: Angle) -> Basis3:
    """Build the tangent-plane basis from equations 2-90 through 2-92."""

    legacy_basis = _legacy_tangent_plane_unit_vectors(longitude.radians, latitude.radians, azimuth.radians)
    return Basis3(
        _as_vector3(legacy_basis[0]),
        _as_vector3(legacy_basis[1]),
        _as_vector3(legacy_basis[2]),
        Frame.ECFC,
        Frame.TANGENT_PLANE,
    )
####


def tangent_plane_coordinates(
    position: FrameQuantityVector3,
    earth_relative_velocity: FrameQuantityVector3,
    earth_relative_acceleration: FrameQuantityVector3,
    origin_position: FrameQuantityVector3,
    tangent_basis: Basis3,
) -> TangentPlaneCoordinates:
    """Project state quantities relative to a tangent-plane origin.

    This implements ``TAOS-ALG-COORD-019`` and equations 2-90 through 2-92.
    All input vectors must be ECFC and the position vectors must share units.
    """

    _require_quantity_frame(position, Frame.ECFC, "position")
    _require_quantity_frame_any(earth_relative_velocity, Frame.ECFC, "earth-relative velocity")
    _require_quantity_frame_any(earth_relative_acceleration, Frame.ECFC, "earth-relative acceleration")
    _require_quantity_frame(origin_position, Frame.ECFC, "tangent-plane origin")
    if position.unit.dimension != origin_position.unit.dimension:
        raise ValueError("position and tangent-plane origin must have compatible units")
    _require_tangent_basis(tangent_basis)
    origin = origin_position.to(position.unit)
    relative = position.vector - origin.vector
    return TangentPlaneCoordinates(
        FrameQuantityVector3(_project(relative, tangent_basis), Frame.TANGENT_PLANE, position.unit),
        FrameQuantityVector3(_project(earth_relative_velocity.vector, tangent_basis), Frame.TANGENT_PLANE, earth_relative_velocity.unit),
        FrameQuantityVector3(_project(earth_relative_acceleration.vector, tangent_basis), Frame.TANGENT_PLANE, earth_relative_acceleration.unit),
    )
####


def tangent_plane_to_ecfc(
    coordinates: TangentPlaneCoordinates,
    origin_position: FrameQuantityVector3,
    tangent_basis: Basis3,
) -> tuple[FrameQuantityVector3, FrameQuantityVector3, FrameQuantityVector3]:
    """Expand tangent-plane components back into ECFC state quantities."""

    _require_quantity_frame(origin_position, Frame.ECFC, "tangent-plane origin")
    _require_tangent_basis(tangent_basis)
    for vector, name in (
        (coordinates.position, "tangent-plane position"),
        (coordinates.earth_relative_velocity, "tangent-plane velocity"),
        (coordinates.earth_relative_acceleration, "tangent-plane acceleration"),
    ):
        if vector.frame is not Frame.TANGENT_PLANE:
            raise ValueError(f"{name} must be expressed in {Frame.TANGENT_PLANE}")
        ####
    origin = origin_position.to(coordinates.position.unit)
    return (
        FrameQuantityVector3(_expand(coordinates.position.vector, tangent_basis) + origin.vector, Frame.ECFC, coordinates.position.unit),
        FrameQuantityVector3(_expand(coordinates.earth_relative_velocity.vector, tangent_basis), Frame.ECFC, coordinates.earth_relative_velocity.unit),
        FrameQuantityVector3(_expand(coordinates.earth_relative_acceleration.vector, tangent_basis), Frame.ECFC, coordinates.earth_relative_acceleration.unit),
    )
####


def _project(vector: Vector3, basis: Basis3) -> Vector3:
    return Vector3(vector.dot(basis.first), vector.dot(basis.second), vector.dot(basis.third))
####


def _expand(vector: Vector3, basis: Basis3) -> Vector3:
    return basis.first.scaled(vector.x) + basis.second.scaled(vector.y) + basis.third.scaled(vector.z)
####


def _require_platform_basis(basis: Basis3) -> None:
    if basis.parent_frame is not Frame.ECIC or basis.child_frame is not Frame.INERTIAL_PLATFORM:
        raise ValueError("platform basis must be an ECIC to inertial-platform basis")
    if not basis.is_orthonormal():
        raise ValueError("platform basis must be orthonormal")
    ####


def _require_tangent_basis(basis: Basis3) -> None:
    if basis.parent_frame is not Frame.ECFC or basis.child_frame is not Frame.TANGENT_PLANE:
        raise ValueError("tangent basis must be an ECFC to tangent-plane basis")
    if not basis.is_orthonormal():
        raise ValueError("tangent basis must be orthonormal")
    ####


def ecic_coords(
    position: FrameVector3,
    earth_relative_velocity: FrameVector3,
    total_force: FrameVector3,
    mass: float,
    initial_angle_radians: float,
    rotation_rate_radians_per_second: float,
    time_seconds: float,
    reference_time_seconds: float,
) -> EcicCoordinates:
    """Convert ECFC state quantities to ECIC quantities.

    This implements ``TAOS-ALG-COORD-001`` and equations 2-1 through 2-5.
    Inputs are explicitly ECFC vectors; outputs are explicitly ECIC vectors.
    The velocity conversion includes the transport term from earth rotation,
    while acceleration is obtained from total force divided by mass.
    """

    _require_frame(position, Frame.ECFC, "position")
    _require_frame(earth_relative_velocity, Frame.ECFC, "earth-relative velocity")
    _require_frame(total_force, Frame.ECFC, "total force")
    if not math.isfinite(mass) or mass <= 0.0:
        raise ValueError("mass must be positive and finite")
    angle = ecic_rotation_angle(
        initial_angle_radians,
        rotation_rate_radians_per_second,
        time_seconds,
        reference_time_seconds,
    )
    ecfc_position = _as_cartesian(position.vector)
    ecfc_velocity = _as_cartesian(earth_relative_velocity.vector)
    ecfc_force = _as_cartesian(total_force.vector)
    return EcicCoordinates(
        angle,
        FrameVector3(_as_vector3(ecfc_to_ecic_position(ecfc_position, angle)), Frame.ECIC),
        FrameVector3(_as_vector3(ecfc_to_ecic_velocity(ecfc_position, ecfc_velocity, angle, rotation_rate_radians_per_second)), Frame.ECIC),
        FrameVector3(_as_vector3(ecfc_to_ecic_acceleration(ecfc_force, mass, angle)), Frame.ECIC),
    )
####


def ecfc_coords(
    position: FrameVector3,
    inertial_velocity: FrameVector3,
    inertial_acceleration: FrameVector3,
    mass: float,
    initial_angle_radians: float,
    rotation_rate_radians_per_second: float,
    time_seconds: float,
    reference_time_seconds: float,
) -> EcfcCoordinates:
    """Apply the derived inverse of the ECFC-to-ECIC conversion.

    The manual prints the forward equations 2-1 through 2-5. This inverse is
    derived from that documented rotation: position and acceleration rotate by
    ``-theta``; the ECIC rotational transport term is removed from inertial
    velocity before the inverse rotation; acceleration is multiplied by mass
    to recover total force.
    """

    _require_frame(position, Frame.ECIC, "position")
    _require_frame(inertial_velocity, Frame.ECIC, "inertial velocity")
    _require_frame(inertial_acceleration, Frame.ECIC, "inertial acceleration")
    if not math.isfinite(mass) or mass <= 0.0:
        raise ValueError("mass must be positive and finite")
    angle = ecic_rotation_angle(
        initial_angle_radians,
        rotation_rate_radians_per_second,
        time_seconds,
        reference_time_seconds,
    )
    cosine = math.cos(angle)
    sine = math.sin(angle)
    ecic_position = position.vector
    ecic_velocity = inertial_velocity.vector
    ecic_acceleration = inertial_acceleration.vector
    ecfc_position = Vector3(
        ecic_position.x * cosine + ecic_position.y * sine,
        -ecic_position.x * sine + ecic_position.y * cosine,
        ecic_position.z,
    )
    corrected_velocity = Vector3(
        ecic_velocity.x + rotation_rate_radians_per_second * ecic_position.y,
        ecic_velocity.y - rotation_rate_radians_per_second * ecic_position.x,
        ecic_velocity.z,
    )
    ecfc_velocity = Vector3(
        corrected_velocity.x * cosine + corrected_velocity.y * sine,
        -corrected_velocity.x * sine + corrected_velocity.y * cosine,
        corrected_velocity.z,
    )
    ecfc_acceleration = Vector3(
        ecic_acceleration.x * cosine + ecic_acceleration.y * sine,
        -ecic_acceleration.x * sine + ecic_acceleration.y * cosine,
        ecic_acceleration.z,
    )
    return EcfcCoordinates(
        angle,
        FrameVector3(ecfc_position, Frame.ECFC),
        FrameVector3(ecfc_velocity, Frame.ECFC),
        FrameVector3(ecfc_acceleration.scaled(mass), Frame.ECFC),
    )
####


def _require_frame(vector: FrameVector3, frame: Frame, name: str) -> None:
    if vector.frame is not frame:
        raise ValueError(f"{name} must be expressed in {frame}")
    ####


def _require_quantity_frame(vector: FrameQuantityVector3, frame: Frame, name: str) -> None:
    if vector.frame is not frame:
        raise ValueError(f"{name} must be expressed in {frame}")
    if vector.unit.dimension != "length":
        raise ValueError(f"{name} must have length units")
    ####


def _require_quantity_frame_any(vector: FrameQuantityVector3, frame: Frame, name: str) -> None:
    if vector.frame is not frame:
        raise ValueError(f"{name} must be expressed in {frame}")
    ####


def _as_vector3(vector: CartesianVector3) -> Vector3:
    return Vector3(vector.x, vector.y, vector.z)
####


def _as_cartesian(vector: Vector3) -> CartesianVector3:
    return CartesianVector3(vector.x, vector.y, vector.z)
####
