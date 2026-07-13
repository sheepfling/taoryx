"""Longitude, latitude, altitude, and ground-speed rate helpers."""

from __future__ import annotations

import math

from .geodesy import (
    CartesianVector3,
    ecfc_to_geocentric_position,
    ecfc_to_geocentric_velocity,
    ecfc_to_geodetic_position,
    ecfc_to_geodetic_velocity,
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


def _dot(left: CartesianVector3, right: CartesianVector3) -> float:
    return left.x * right.x + left.y * right.y + left.z * right.z
####


def _compose_from_basis(
    basis: tuple[CartesianVector3, CartesianVector3, CartesianVector3],
    components: CartesianVector3,
) -> CartesianVector3:
    x_axis, y_axis, z_axis = basis
    return CartesianVector3(
        components.x * x_axis.x + components.y * y_axis.x + components.z * z_axis.x,
        components.x * x_axis.y + components.y * y_axis.y + components.z * z_axis.y,
        components.x * x_axis.z + components.y * y_axis.z + components.z * z_axis.z,
    )
####


def _velocity_axes_from_angles(
    reference_basis: tuple[CartesianVector3, CartesianVector3, CartesianVector3],
    gamma_radians: float,
    psi_radians: float,
) -> tuple[CartesianVector3, CartesianVector3, CartesianVector3]:
    cosine_gamma = math.cos(gamma_radians)
    sine_gamma = math.sin(gamma_radians)
    cosine_psi = math.cos(psi_radians)
    sine_psi = math.sin(psi_radians)
    return (
        _compose_from_basis(
            reference_basis,
            CartesianVector3(
                cosine_gamma * cosine_psi,
                cosine_gamma * sine_psi,
                -sine_gamma,
            ),
        ),
        _compose_from_basis(
            reference_basis,
            CartesianVector3(-sine_psi, cosine_psi, 0.0),
        ),
        _compose_from_basis(
            reference_basis,
            CartesianVector3(
                sine_gamma * cosine_psi,
                sine_gamma * sine_psi,
                cosine_gamma,
            ),
        ),
    )
####


def _velocity_angles_from_components(components: CartesianVector3) -> tuple[float, float, float]:
    speed = math.sqrt(components.x * components.x + components.y * components.y + components.z * components.z)
    if speed == 0.0:
        return 0.0, 0.0, 0.0
    ####
    gamma = math.asin(-components.z / speed)
    horizontal_speed = math.hypot(components.x, components.y)
    psi = 0.0 if math.isclose(horizontal_speed, 0.0, abs_tol=1e-12 * max(1.0, speed)) else math.atan2(components.y, components.x)
    return speed, gamma, psi
####


def _flight_path_angle_rates(
    reference_basis: tuple[CartesianVector3, CartesianVector3, CartesianVector3],
    longitude_rate: float,
    latitude_rate: float,
    velocity_components: CartesianVector3,
    acceleration_components: CartesianVector3,
) -> tuple[float, float]:
    speed, gamma, psi = _velocity_angles_from_components(velocity_components)
    if speed == 0.0:
        return 0.0, 0.0
    ####
    _, y_v, z_v = _velocity_axes_from_angles(reference_basis, gamma, psi)
    cosine_gamma = math.cos(gamma)
    gamma_rate = -_dot(acceleration_components, z_v) / speed - longitude_rate * y_v.z + latitude_rate * math.cos(psi)
    if math.isclose(cosine_gamma, 0.0, abs_tol=1e-12):
        psi_rate = 0.0
    else:
        psi_rate = (
            _dot(acceleration_components, y_v) / speed
            - longitude_rate * z_v.z
            + latitude_rate * math.sin(gamma) * math.sin(psi)
        ) / cosine_gamma
    return gamma_rate, psi_rate
####


def geocentric_flight_path_rates_from_ecfc(
    position: CartesianVector3,
    velocity: CartesianVector3,
    acceleration: CartesianVector3,
) -> tuple[float, float]:
    """Return the geocentric flight-path-angle rates."""

    geocentric_position = ecfc_to_geocentric_position(position.x, position.y, position.z)
    velocity_components = ecfc_to_geocentric_velocity(
        geocentric_position.longitude_radians,
        geocentric_position.latitude_radians,
        velocity,
    )
    acceleration_components = ecfc_to_geocentric_velocity(
        geocentric_position.longitude_radians,
        geocentric_position.latitude_radians,
        acceleration,
    )
    return _flight_path_angle_rates(
        geocentric_unit_vectors(geocentric_position.longitude_radians, geocentric_position.latitude_radians),
        longitude_rate_from_ecfc(position, velocity),
        geocentric_latitude_rate_from_ecfc(position, velocity),
        velocity_components,
        acceleration_components,
    )
####


def geocentric_vertical_flight_path_angle_rate_from_ecfc(
    position: CartesianVector3,
    velocity: CartesianVector3,
    acceleration: CartesianVector3,
) -> float:
    """Return the geocentric vertical flight-path-angle rate."""

    return geocentric_flight_path_rates_from_ecfc(position, velocity, acceleration)[0]
####


def geocentric_heading_rate_from_ecfc(
    position: CartesianVector3,
    velocity: CartesianVector3,
    acceleration: CartesianVector3,
) -> float:
    """Return the geocentric heading rate."""

    return geocentric_flight_path_rates_from_ecfc(position, velocity, acceleration)[1]
####


def geodetic_flight_path_rates_from_ecfc(
    equatorial_radius: float,
    eccentricity: float,
    position: CartesianVector3,
    velocity: CartesianVector3,
    acceleration: CartesianVector3,
) -> tuple[float, float]:
    """Return the geodetic flight-path-angle rates."""

    geodetic_position = ecfc_to_geodetic_position(
        equatorial_radius,
        eccentricity,
        position.x,
        position.y,
        position.z,
    )
    velocity_components = ecfc_to_geodetic_velocity(
        geodetic_position.longitude_radians,
        geodetic_position.latitude_radians,
        velocity,
    )
    acceleration_components = ecfc_to_geodetic_velocity(
        geodetic_position.longitude_radians,
        geodetic_position.latitude_radians,
        acceleration,
    )
    return _flight_path_angle_rates(
        geodetic_unit_vectors(geodetic_position.longitude_radians, geodetic_position.latitude_radians),
        longitude_rate_from_ecfc(position, velocity),
        geodetic_latitude_rate_from_ecfc(equatorial_radius, eccentricity, position, velocity),
        velocity_components,
        acceleration_components,
    )
####


def geodetic_vertical_flight_path_angle_rate_from_ecfc(
    equatorial_radius: float,
    eccentricity: float,
    position: CartesianVector3,
    velocity: CartesianVector3,
    acceleration: CartesianVector3,
) -> float:
    """Return the geodetic vertical flight-path-angle rate."""

    return geodetic_flight_path_rates_from_ecfc(
        equatorial_radius,
        eccentricity,
        position,
        velocity,
        acceleration,
    )[0]
####


def geodetic_heading_rate_from_ecfc(
    equatorial_radius: float,
    eccentricity: float,
    position: CartesianVector3,
    velocity: CartesianVector3,
    acceleration: CartesianVector3,
) -> float:
    """Return the geodetic heading rate."""

    return geodetic_flight_path_rates_from_ecfc(
        equatorial_radius,
        eccentricity,
        position,
        velocity,
        acceleration,
    )[1]
####
