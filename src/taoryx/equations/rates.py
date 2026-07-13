"""Longitude, latitude, altitude, and ground-speed rate helpers."""

from __future__ import annotations

import math

from .geodesy import (
    CartesianVector3,
    ecfc_to_geocentric_position,
    ecfc_to_geodetic_position,
    geocentric_unit_vectors,
    geodetic_surface_normal_distance,
    geodetic_unit_vectors,
)


def longitude_from_ecfc_position(position: CartesianVector3) -> float:
    """Return longitude from ECFC position components."""

    return math.atan2(position.y, position.x)
####


def longitude_rate_from_ecfc(position: CartesianVector3, velocity: CartesianVector3) -> float:
    """Return the longitude rate from ECFC position and velocity components."""

    denominator = position.x * position.x + position.y * position.y
    if denominator == 0.0:
        return 0.0
    ####
    return (velocity.y * position.x - velocity.x * position.y) / denominator
####


def geocentric_latitude_rate_from_ecfc(position: CartesianVector3, velocity: CartesianVector3) -> float:
    """Return the geocentric latitude rate from ECFC position and velocity."""

    geocentric_position = ecfc_to_geocentric_position(position.x, position.y, position.z)
    if geocentric_position.radius == 0.0:
        return 0.0
    ####
    north, _, _ = geocentric_unit_vectors(
        geocentric_position.longitude_radians,
        geocentric_position.latitude_radians,
    )
    return (velocity.x * north.x + velocity.y * north.y + velocity.z * north.z) / geocentric_position.radius
####


def geodetic_latitude_rate_from_ecfc(
    equatorial_radius: float,
    eccentricity: float,
    position: CartesianVector3,
    velocity: CartesianVector3,
) -> float:
    """Return the geodetic latitude rate from ECFC position and velocity."""

    geodetic_position = ecfc_to_geodetic_position(
        equatorial_radius,
        eccentricity,
        position.x,
        position.y,
        position.z,
    )
    north, _, _ = geodetic_unit_vectors(geodetic_position.longitude_radians, geodetic_position.latitude_radians)
    normal_distance = geodetic_surface_normal_distance(
        equatorial_radius,
        eccentricity,
        geodetic_position.latitude_radians,
    )
    sine_latitude = math.sin(geodetic_position.latitude_radians)
    denominator = geodetic_position.altitude + normal_distance * (1.0 - eccentricity * eccentricity) / (
        1.0 - eccentricity * eccentricity * sine_latitude * sine_latitude
    )
    if denominator == 0.0:
        return geocentric_latitude_rate_from_ecfc(position, velocity)
    ####
    return (velocity.x * north.x + velocity.y * north.y + velocity.z * north.z) / denominator
####


def altitude_rate_from_ecfc(
    equatorial_radius: float,
    eccentricity: float,
    position: CartesianVector3,
    velocity: CartesianVector3,
) -> float:
    """Return the altitude rate from ECFC position and velocity."""

    geodetic_position = ecfc_to_geodetic_position(
        equatorial_radius,
        eccentricity,
        position.x,
        position.y,
        position.z,
    )
    _, _, down = geodetic_unit_vectors(geodetic_position.longitude_radians, geodetic_position.latitude_radians)
    return -(velocity.x * down.x + velocity.y * down.y + velocity.z * down.z)
####


def geodetic_down_unit_vector_derivative(
    longitude_radians: float,
    latitude_radians: float,
    longitude_rate_radians_per_second: float,
    latitude_rate_radians_per_second: float,
) -> CartesianVector3:
    """Return the time derivative of the geodetic down unit vector."""

    cos_latitude = math.cos(latitude_radians)
    sin_latitude = math.sin(latitude_radians)
    cos_longitude = math.cos(longitude_radians)
    sin_longitude = math.sin(longitude_radians)

    north = CartesianVector3(
        -sin_latitude * cos_longitude,
        -sin_latitude * sin_longitude,
        cos_latitude,
    )
    east = CartesianVector3(-sin_longitude, cos_longitude, 0.0)
    down = CartesianVector3(-cos_latitude * cos_longitude, -cos_latitude * sin_longitude, -sin_latitude)
    return CartesianVector3(
        (sin_latitude * (cos_longitude * north.x + sin_longitude * east.x) - cos_latitude * down.x)
        * latitude_rate_radians_per_second
        + cos_latitude * (sin_longitude * north.x - cos_longitude * east.x) * longitude_rate_radians_per_second,
        (sin_latitude * (cos_longitude * north.y + sin_longitude * east.y) - cos_latitude * down.y)
        * latitude_rate_radians_per_second
        + cos_latitude * (sin_longitude * north.y - cos_longitude * east.y) * longitude_rate_radians_per_second,
        (sin_latitude * (cos_longitude * north.z + sin_longitude * east.z) - cos_latitude * down.z)
        * latitude_rate_radians_per_second
        + cos_latitude * (sin_longitude * north.z - cos_longitude * east.z) * longitude_rate_radians_per_second,
    )
####


def altitude_acceleration_from_ecfc(
    equatorial_radius: float,
    eccentricity: float,
    position: CartesianVector3,
    velocity: CartesianVector3,
    acceleration: CartesianVector3,
) -> float:
    """Return the altitude acceleration from ECFC state variables."""

    geodetic_position = ecfc_to_geodetic_position(
        equatorial_radius,
        eccentricity,
        position.x,
        position.y,
        position.z,
    )
    longitude_rate = longitude_rate_from_ecfc(position, velocity)
    latitude_rate = geodetic_latitude_rate_from_ecfc(equatorial_radius, eccentricity, position, velocity)
    down = geodetic_unit_vectors(geodetic_position.longitude_radians, geodetic_position.latitude_radians)[2]
    down_derivative = geodetic_down_unit_vector_derivative(
        geodetic_position.longitude_radians,
        geodetic_position.latitude_radians,
        longitude_rate,
        latitude_rate,
    )
    return -(
        acceleration.x * down.x
        + acceleration.y * down.y
        + acceleration.z * down.z
    ) - (
        velocity.x * down_derivative.x
        + velocity.y * down_derivative.y
        + velocity.z * down_derivative.z
    )
####


def ground_speed_vector_from_ecfc(
    equatorial_radius: float,
    eccentricity: float,
    position: CartesianVector3,
    velocity: CartesianVector3,
) -> CartesianVector3:
    """Return the ground-speed vector from ECFC state variables."""

    geodetic_position = ecfc_to_geodetic_position(
        equatorial_radius,
        eccentricity,
        position.x,
        position.y,
        position.z,
    )
    longitude = geodetic_position.longitude_radians
    latitude = geodetic_position.latitude_radians
    altitude = geodetic_position.altitude
    longitude_rate = longitude_rate_from_ecfc(position, velocity)
    latitude_rate = geodetic_latitude_rate_from_ecfc(equatorial_radius, eccentricity, position, velocity)
    altitude_rate = altitude_rate_from_ecfc(equatorial_radius, eccentricity, position, velocity)
    down = geodetic_unit_vectors(longitude, latitude)[2]
    down_derivative = geodetic_down_unit_vector_derivative(longitude, latitude, longitude_rate, latitude_rate)
    return CartesianVector3(
        velocity.x + altitude_rate * down.x + altitude * down_derivative.x,
        velocity.y + altitude_rate * down.y + altitude * down_derivative.y,
        velocity.z + altitude_rate * down.z + altitude * down_derivative.z,
    )
####


def ground_speed_magnitude_from_ecfc(
    equatorial_radius: float,
    eccentricity: float,
    position: CartesianVector3,
    velocity: CartesianVector3,
) -> float:
    """Return the ground-speed magnitude from ECFC state variables."""

    ground_speed = ground_speed_vector_from_ecfc(equatorial_radius, eccentricity, position, velocity)
    return math.sqrt(
        ground_speed.x * ground_speed.x
        + ground_speed.y * ground_speed.y
        + ground_speed.z * ground_speed.z
    )
####
