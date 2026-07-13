"""Small executable helpers for the easiest TAOS coordinate equations."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class CartesianVector3:
    """Simple 3D Cartesian tuple with named fields for readability."""

    x: float
    y: float
    z: float
####


@dataclass(frozen=True, slots=True)
class GeocentricPosition:
    """Geocentric position in the documented TAOS order: radius, longitude, latitude."""

    radius: float
    longitude_radians: float
    latitude_radians: float
####


@dataclass(frozen=True, slots=True)
class GeodeticPosition:
    """Geodetic position in the documented TAOS order: longitude, latitude, altitude."""

    longitude_radians: float
    latitude_radians: float
    altitude: float
####


@dataclass(frozen=True, slots=True)
class VelocityAngles:
    """Velocity magnitude plus TAOS flight-path and heading angles."""

    speed: float
    gamma_radians: float
    psi_radians: float
####


def _dot(left: CartesianVector3, right: CartesianVector3) -> float:
    return left.x * right.x + left.y * right.y + left.z * right.z
####


def _compose_from_basis(
    basis: tuple[CartesianVector3, CartesianVector3, CartesianVector3],
    components: CartesianVector3,
) -> CartesianVector3:
    north, east, down = basis
    return CartesianVector3(
        components.x * north.x + components.y * east.x + components.z * down.x,
        components.x * north.y + components.y * east.y + components.z * down.y,
        components.x * north.z + components.y * east.z + components.z * down.z,
    )
####


def _decompose_to_basis(
    basis: tuple[CartesianVector3, CartesianVector3, CartesianVector3],
    vector: CartesianVector3,
) -> CartesianVector3:
    north, east, down = basis
    return CartesianVector3(
        _dot(vector, north),
        _dot(vector, east),
        _dot(vector, down),
    )
####


def polar_radius_from_equatorial_radius(
    equatorial_radius: float,
    *,
    eccentricity: float | None = None,
    flattening: float | None = None,
) -> float:
    """Return the polar radius from TAOS equation 2-18.

    The manual gives two equivalent forms:

    - ``R_p = R_⊕ sqrt(1 - e^2)``
    - ``R_p = R_⊕ (1 - f)``
    """

    if eccentricity is None and flattening is None:
        raise ValueError("either eccentricity or flattening must be provided")
    ####
    if eccentricity is not None and flattening is not None:
        expected_flattening = 1.0 - math.sqrt(1.0 - eccentricity * eccentricity)
        if not math.isclose(flattening, expected_flattening, rel_tol=0.0, abs_tol=1e-12):
            raise ValueError("eccentricity and flattening are inconsistent")
        ####
    ####
    if eccentricity is not None:
        return equatorial_radius * math.sqrt(1.0 - eccentricity * eccentricity)
    ####
    assert flattening is not None
    return equatorial_radius * (1.0 - flattening)
####


def geocentric_position_to_ecfc(
    radius: float,
    longitude_radians: float,
    latitude_radians: float,
) -> CartesianVector3:
    """Convert geocentric spherical coordinates to ECFC Cartesian coordinates."""

    cos_latitude = math.cos(latitude_radians)
    return CartesianVector3(
        radius * cos_latitude * math.cos(longitude_radians),
        radius * cos_latitude * math.sin(longitude_radians),
        radius * math.sin(latitude_radians),
    )
####


def ecfc_to_geocentric_position(x: float, y: float, z: float) -> GeocentricPosition:
    """Invert the geocentric position transform from TAOS equations 2-8 and 2-9.

    Returns ``GeocentricPosition(radius, longitude, latitude)``.
    """

    radius = math.sqrt(x * x + y * y + z * z)
    if radius == 0.0:
        raise ValueError("geocentric position is undefined at the origin")
    ####
    longitude = math.atan2(y, x)
    latitude = math.asin(z / radius)
    return GeocentricPosition(radius, longitude, latitude)
####


def geocentric_unit_vectors(longitude_radians: float, latitude_radians: float) -> tuple[CartesianVector3, CartesianVector3, CartesianVector3]:
    """Return the local geocentric north, east, and down unit vectors."""

    cos_latitude = math.cos(latitude_radians)
    sin_latitude = math.sin(latitude_radians)
    cos_longitude = math.cos(longitude_radians)
    sin_longitude = math.sin(longitude_radians)

    north = CartesianVector3(
        -sin_latitude * cos_longitude,
        -sin_latitude * sin_longitude,
        cos_latitude,
    )
    east = CartesianVector3(
        -sin_longitude,
        cos_longitude,
        0.0,
    )
    down = CartesianVector3(
        -cos_latitude * cos_longitude,
        -cos_latitude * sin_longitude,
        -sin_latitude,
    )
    return north, east, down
####


def geocentric_velocity_components_from_angles(
    speed: float,
    gamma_radians: float,
    psi_radians: float,
) -> CartesianVector3:
    """Return local geocentric velocity components from TAOS equations 2-62 to 2-64."""

    cosine_gamma = math.cos(gamma_radians)
    return CartesianVector3(
        speed * cosine_gamma * math.cos(psi_radians),
        speed * cosine_gamma * math.sin(psi_radians),
        -speed * math.sin(gamma_radians),
    )
####


def geocentric_velocity_angles_from_components(components: CartesianVector3) -> VelocityAngles:
    """Return speed, flight-path angle, and heading from local geocentric components."""

    speed = math.sqrt(components.x * components.x + components.y * components.y + components.z * components.z)
    if speed == 0.0:
        return VelocityAngles(0.0, 0.0, 0.0)
    ####
    gamma = math.asin(-components.z / speed)
    horizontal_speed = math.hypot(components.x, components.y)
    psi = 0.0 if math.isclose(horizontal_speed, 0.0, abs_tol=1e-12 * max(1.0, speed)) else math.atan2(components.y, components.x)
    return VelocityAngles(speed, gamma, psi)
####


def geocentric_velocity_to_ecfc(
    longitude_radians: float,
    latitude_radians: float,
    components: CartesianVector3,
) -> CartesianVector3:
    """Transform local geocentric velocity components into ECFC coordinates."""

    return _compose_from_basis(geocentric_unit_vectors(longitude_radians, latitude_radians), components)
####


def ecfc_to_geocentric_velocity(
    longitude_radians: float,
    latitude_radians: float,
    vector: CartesianVector3,
) -> CartesianVector3:
    """Transform ECFC velocity components into local geocentric coordinates."""

    return _decompose_to_basis(geocentric_unit_vectors(longitude_radians, latitude_radians), vector)
####


def geodetic_surface_normal_distance(
    equatorial_radius: float,
    eccentricity: float,
    latitude_radians: float,
) -> float:
    """Return the normal distance ``N`` from TAOS equation 2-30."""

    sine = math.sin(latitude_radians)
    return equatorial_radius / math.sqrt(1.0 - eccentricity * eccentricity * sine * sine)
####


def geodetic_position_to_ecfc(
    equatorial_radius: float,
    eccentricity: float,
    longitude_radians: float,
    latitude_radians: float,
    altitude: float,
) -> CartesianVector3:
    """Convert geodetic coordinates to ECFC Cartesian coordinates."""

    normal_distance = geodetic_surface_normal_distance(equatorial_radius, eccentricity, latitude_radians)
    cos_latitude = math.cos(latitude_radians)
    sin_latitude = math.sin(latitude_radians)
    cos_longitude = math.cos(longitude_radians)
    sin_longitude = math.sin(longitude_radians)
    equatorial_projection = (normal_distance + altitude) * cos_latitude
    return CartesianVector3(
        equatorial_projection * cos_longitude,
        equatorial_projection * sin_longitude,
        (normal_distance * (1.0 - eccentricity * eccentricity) + altitude) * sin_latitude,
    )
####


def ecfc_to_geodetic_position(
    equatorial_radius: float,
    eccentricity: float,
    x: float,
    y: float,
    z: float,
    *,
    tolerance: float = 1e-8,
    max_iterations: int = 25,
) -> GeodeticPosition:
    """Invert the geodetic position transform using the TAOS fixed-point iteration.

    Returns ``GeodeticPosition(longitude, latitude, altitude)``.
    """

    geocentric_position = ecfc_to_geocentric_position(x, y, z)
    if geocentric_position.radius == 0.0:
        raise ValueError("geodetic position is undefined at the origin")
    ####
    projected_radius = math.hypot(x, y)
    intercept = equatorial_radius * eccentricity * eccentricity * math.sin(geocentric_position.latitude_radians)
    last_intercept = float("inf")
    iterations = 0
    geodetic_latitude = geocentric_position.latitude_radians
    altitude = 0.0

    while abs(intercept - last_intercept) > tolerance:
        if iterations >= max_iterations:
            raise RuntimeError("geodetic inverse iteration did not converge")
        ####
        last_intercept = intercept
        z_d = z + intercept
        radius_plus_altitude = math.hypot(projected_radius, z_d)
        if radius_plus_altitude == 0.0:
            raise ValueError("geodetic position is undefined at the origin")
        ####
        geodetic_latitude = math.asin(z_d / radius_plus_altitude)
        normal_distance = geodetic_surface_normal_distance(equatorial_radius, eccentricity, geodetic_latitude)
        altitude = radius_plus_altitude - normal_distance
        intercept = normal_distance * eccentricity * eccentricity * math.sin(geodetic_latitude)
        iterations += 1
    ####
    return GeodeticPosition(geocentric_position.longitude_radians, geodetic_latitude, altitude)
####


def geodetic_unit_vectors(longitude_radians: float, latitude_radians: float) -> tuple[CartesianVector3, CartesianVector3, CartesianVector3]:
    """Return the local geodetic north, east, and down unit vectors."""

    cos_latitude = math.cos(latitude_radians)
    sin_latitude = math.sin(latitude_radians)
    cos_longitude = math.cos(longitude_radians)
    sin_longitude = math.sin(longitude_radians)

    north = CartesianVector3(
        -sin_latitude * cos_longitude,
        -sin_latitude * sin_longitude,
        cos_latitude,
    )
    east = CartesianVector3(
        -sin_longitude,
        cos_longitude,
        0.0,
    )
    down = CartesianVector3(
        -cos_latitude * cos_longitude,
        -cos_latitude * sin_longitude,
        -sin_latitude,
    )
    return north, east, down
####


def geodetic_velocity_components_from_angles(
    speed: float,
    gamma_radians: float,
    psi_radians: float,
) -> CartesianVector3:
    """Return local geodetic velocity components from TAOS equations 2-135 to 2-137."""

    cosine_gamma = math.cos(gamma_radians)
    return CartesianVector3(
        speed * cosine_gamma * math.cos(psi_radians),
        speed * cosine_gamma * math.sin(psi_radians),
        -speed * math.sin(gamma_radians),
    )
####


def geodetic_velocity_angles_from_components(components: CartesianVector3) -> VelocityAngles:
    """Return speed, flight-path angle, and heading from local geodetic components."""

    speed = math.sqrt(components.x * components.x + components.y * components.y + components.z * components.z)
    if speed == 0.0:
        return VelocityAngles(0.0, 0.0, 0.0)
    ####
    gamma = math.asin(-components.z / speed)
    horizontal_speed = math.hypot(components.x, components.y)
    psi = 0.0 if math.isclose(horizontal_speed, 0.0, abs_tol=1e-12 * max(1.0, speed)) else math.atan2(components.y, components.x)
    return VelocityAngles(speed, gamma, psi)
####


def geodetic_velocity_to_ecfc(
    longitude_radians: float,
    latitude_radians: float,
    components: CartesianVector3,
) -> CartesianVector3:
    """Transform local geodetic velocity components into ECFC coordinates."""

    return _compose_from_basis(geodetic_unit_vectors(longitude_radians, latitude_radians), components)
####


def ecfc_to_geodetic_velocity(
    longitude_radians: float,
    latitude_radians: float,
    vector: CartesianVector3,
) -> CartesianVector3:
    """Transform ECFC velocity components into local geodetic coordinates."""

    return _decompose_to_basis(geodetic_unit_vectors(longitude_radians, latitude_radians), vector)
####
