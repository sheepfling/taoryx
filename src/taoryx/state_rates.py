"""Typed longitude, latitude, and altitude rate algorithms from TAOS."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .contracts import Frame, FrameQuantityVector3, FrameVector3, Vector3
from .coordinates import ecfc_position_to_geocentric, ecfc_position_to_geodetic, geocentric_unit_vectors, geodetic_unit_vectors
from .earth import EllipsoidParameters, ellipsoidal_surface_geometry
from .equations import CartesianVector3, geocentric_flight_path_rates_from_ecfc, geodetic_flight_path_rates_from_ecfc


@dataclass(frozen=True, slots=True)
class GeocentricStateRates:
    """Longitude and geocentric-latitude rates in radians per time unit."""

    longitude_rate: float
    latitude_rate: float


@dataclass(frozen=True, slots=True)
class GeodeticStateRates:
    """Geodetic-latitude and altitude rates in coherent input units."""

    latitude_rate: float
    altitude_rate: float
####


@dataclass(frozen=True, slots=True)
class DynamicPressureDerivatives:
    """Dynamic pressure and its first two time derivatives."""

    pressure: float
    pressure_rate: float
    pressure_acceleration: float
####


@dataclass(frozen=True, slots=True)
class GroundSpeedResult:
    """Surface-relative ground-speed vector and its scalar integration rate."""

    vector: FrameVector3
    magnitude: float
####


@dataclass(frozen=True, slots=True)
class FlightPathRateResult:
    """Geocentric and geodetic vertical/heading rate pairs."""

    geocentric_vertical_rate: float
    geocentric_heading_rate: float
    geodetic_vertical_rate: float
    geodetic_heading_rate: float
####


def dynamic_pressure_derivatives(
    air_density: float,
    air_density_rate: float,
    air_density_second_rate: float,
    air_relative_speed: float,
    air_relative_speed_rate: float,
    air_acceleration_magnitude: float,
) -> DynamicPressureDerivatives:
    """Evaluate TAOS-ALG-DYN-008 and equations 2-141 through 2-144.

    Inputs are coherent scalar values. The second derivative follows the
    manual's approximation: second derivatives of air-relative velocity and
    atmospheric density are reduced to the supplied scalar terms, and the
    air-relative acceleration magnitude is used for the quadratic velocity
    contribution.
    """

    values = (
        air_density,
        air_density_rate,
        air_density_second_rate,
        air_relative_speed,
        air_relative_speed_rate,
        air_acceleration_magnitude,
    )
    if any(not math.isfinite(value) for value in values):
        raise ValueError("dynamic-pressure inputs must be finite")
    pressure = 0.5 * air_density * air_relative_speed * air_relative_speed
    pressure_rate = 0.5 * air_density_rate * air_relative_speed * air_relative_speed + air_density * air_relative_speed * air_relative_speed_rate
    pressure_acceleration = (
        0.5 * air_density_second_rate * air_relative_speed * air_relative_speed
        + 2.0 * air_density_rate * air_relative_speed * air_relative_speed_rate
        + air_density * air_acceleration_magnitude * air_acceleration_magnitude
    )
    return DynamicPressureDerivatives(pressure, pressure_rate, pressure_acceleration)
####


def ground_speed(
    position: FrameQuantityVector3,
    earth_relative_velocity: FrameVector3,
    parameters: EllipsoidParameters,
) -> GroundSpeedResult:
    """Evaluate TAOS-ALG-DYN-010 and equations 2-148 through 2-152.

    The returned vector is tangent to the geodetic surface below the vehicle;
    its magnitude is the scalar rate used to integrate ground range. Numeric
    velocity and position units must be coherent at the call boundary.
    """

    _require_position_and_velocity(position, earth_relative_velocity)
    normalized = position.to(parameters.equatorial_radius.unit)
    geodetic = ecfc_position_to_geodetic(normalized, parameters)
    basis = geodetic_unit_vectors(geodetic.longitude, geodetic.latitude)
    rates = geodetic_latitude_and_altitude_rates(normalized, earth_relative_velocity, parameters)
    equatorial_projection = math.hypot(normalized.vector.x, normalized.vector.y)
    if equatorial_projection == 0.0:
        longitude_rate = 0.0
    else:
        longitude_rate = (
            earth_relative_velocity.vector.y * normalized.vector.x
            - earth_relative_velocity.vector.x * normalized.vector.y
        ) / (equatorial_projection * equatorial_projection)
    latitude = geodetic.latitude.radians
    longitude = geodetic.longitude.radians
    sine = math.sin(latitude)
    cosine = math.cos(latitude)
    down_derivative = Vector3(
        sine * math.cos(longitude) * rates.latitude_rate + cosine * math.sin(longitude) * longitude_rate,
        sine * math.sin(longitude) * rates.latitude_rate - cosine * math.cos(longitude) * longitude_rate,
        -cosine * rates.latitude_rate,
    )
    ground_vector = (
        earth_relative_velocity.vector
        + basis.third.scaled(rates.altitude_rate)
        + down_derivative.scaled(geodetic.altitude.value)
    )
    return GroundSpeedResult(FrameVector3(ground_vector, Frame.ECFC), ground_vector.norm())
####


def flight_path_angle_rates(
    position: FrameQuantityVector3,
    earth_relative_velocity: FrameVector3,
    earth_fixed_acceleration: FrameVector3,
    parameters: EllipsoidParameters,
) -> FlightPathRateResult:
    """Evaluate TAOS-ALG-DYN-011 and equations 2-153 through 2-171.

    The equation registry supplies the scalar implementation. This binding
    makes the ECFC frame and ellipsoid requirements explicit and returns named
    rate components instead of an ambiguous tuple.
    """

    _require_position_and_velocity(position, earth_relative_velocity)
    if earth_fixed_acceleration.frame is not Frame.ECFC:
        raise ValueError("earth-fixed acceleration must be expressed in ECFC")
    normalized = position.to(parameters.equatorial_radius.unit)
    legacy_position = CartesianVector3(normalized.vector.x, normalized.vector.y, normalized.vector.z)
    legacy_velocity = CartesianVector3(earth_relative_velocity.vector.x, earth_relative_velocity.vector.y, earth_relative_velocity.vector.z)
    legacy_acceleration = CartesianVector3(earth_fixed_acceleration.vector.x, earth_fixed_acceleration.vector.y, earth_fixed_acceleration.vector.z)
    geocentric = geocentric_flight_path_rates_from_ecfc(
        legacy_position,
        legacy_velocity,
        legacy_acceleration,
    )
    geodetic = geodetic_flight_path_rates_from_ecfc(
        parameters.equatorial_radius.value,
        parameters.eccentricity,
        legacy_position,
        legacy_velocity,
        legacy_acceleration,
    )
    return FlightPathRateResult(geocentric[0], geocentric[1], geodetic[0], geodetic[1])
####


def mach_rate(
    air_relative_speed: float,
    air_relative_speed_rate: float,
    speed_of_sound: float,
    speed_of_sound_rate: float,
) -> float:
    """Evaluate TAOS-ALG-DYN-009 and equations 2-145 through 2-147."""

    values = (air_relative_speed, air_relative_speed_rate, speed_of_sound, speed_of_sound_rate)
    if any(not math.isfinite(value) for value in values):
        raise ValueError("Mach-rate inputs must be finite")
    if speed_of_sound == 0.0:
        raise ValueError("Mach rate is undefined when speed of sound is zero")
    return (air_relative_speed_rate * speed_of_sound - air_relative_speed * speed_of_sound_rate) / (speed_of_sound * speed_of_sound)
####


def altitude_acceleration(
    position: FrameQuantityVector3,
    earth_relative_velocity: FrameVector3,
    earth_fixed_acceleration: FrameVector3,
    parameters: EllipsoidParameters,
) -> float:
    """Evaluate TAOS-ALG-DYN-007 and equations 2-138 through 2-140.

    The result uses the coherent numeric units supplied for velocity and
    acceleration. Geodetic longitude and latitude rates differentiate the
    local down unit vector used by the altitude equation.
    """

    _require_position_and_velocity(position, earth_relative_velocity)
    if earth_fixed_acceleration.frame is not Frame.ECFC:
        raise ValueError("earth-fixed acceleration must be expressed in ECFC")
    normalized = position.to(parameters.equatorial_radius.unit)
    geodetic = ecfc_position_to_geodetic(normalized, parameters)
    basis = geodetic_unit_vectors(geodetic.longitude, geodetic.latitude)
    rates = geodetic_latitude_and_altitude_rates(normalized, earth_relative_velocity, parameters)
    equatorial_projection = math.hypot(normalized.vector.x, normalized.vector.y)
    if equatorial_projection == 0.0:
        longitude_rate = 0.0
    else:
        longitude_rate = (
            earth_relative_velocity.vector.y * normalized.vector.x
            - earth_relative_velocity.vector.x * normalized.vector.y
        ) / (equatorial_projection * equatorial_projection)
    sine = math.sin(geodetic.latitude.radians)
    cosine = math.cos(geodetic.latitude.radians)
    longitude = geodetic.longitude.radians
    down_derivative = Vector3(
        sine * math.cos(longitude) * rates.latitude_rate + cosine * math.sin(longitude) * longitude_rate,
        sine * math.sin(longitude) * rates.latitude_rate - cosine * math.cos(longitude) * longitude_rate,
        -cosine * rates.latitude_rate,
    )
    return -earth_fixed_acceleration.vector.dot(basis.third) - earth_relative_velocity.vector.dot(down_derivative)
####


def longitude_and_geocentric_latitude_rates(
    position: FrameQuantityVector3,
    earth_relative_velocity: FrameVector3,
) -> GeocentricStateRates:
    """Evaluate TAOS-ALG-DYN-005 and equations 2-123 through 2-128."""

    _require_position_and_velocity(position, earth_relative_velocity)
    coordinates = ecfc_position_to_geocentric(position)
    radius = coordinates.radius.value
    if radius == 0.0:
        raise ValueError("geocentric rates are undefined at the origin")
    equatorial_projection = math.hypot(position.vector.x, position.vector.y)
    if equatorial_projection == 0.0:
        longitude_rate = 0.0
    else:
        longitude_rate = (earth_relative_velocity.vector.y * position.vector.x - earth_relative_velocity.vector.x * position.vector.y) / (equatorial_projection * equatorial_projection)
    north = geocentric_unit_vectors(coordinates.longitude, coordinates.latitude).first
    latitude_rate = earth_relative_velocity.vector.dot(north) / radius
    return GeocentricStateRates(longitude_rate, latitude_rate)
####


def geodetic_latitude_and_altitude_rates(
    position: FrameQuantityVector3,
    earth_relative_velocity: FrameVector3,
    parameters: EllipsoidParameters,
) -> GeodeticStateRates:
    """Evaluate TAOS-ALG-DYN-006 and equations 2-129 through 2-137.

    If the final geodetic denominator is singular, the geocentric latitude rate
    is used, matching the manual's stated fallback convention.
    """

    _require_position_and_velocity(position, earth_relative_velocity)
    normalized = position.to(parameters.equatorial_radius.unit)
    geodetic = ecfc_position_to_geodetic(normalized, parameters)
    basis = geodetic_unit_vectors(geodetic.longitude, geodetic.latitude)
    geometry = ellipsoidal_surface_geometry(parameters, geodetic.latitude)
    sine = math.sin(geodetic.latitude.radians)
    eccentricity_squared = parameters.eccentricity * parameters.eccentricity
    altitude = geodetic.altitude.value
    denominator = altitude + geometry.normal_distance.value * (1.0 - eccentricity_squared) / (1.0 - eccentricity_squared * sine * sine)
    north_velocity = earth_relative_velocity.vector.dot(basis.first)
    if denominator == 0.0:
        geocentric = ecfc_position_to_geocentric(normalized)
        radius = geocentric.radius.value
        north = geocentric_unit_vectors(geocentric.longitude, geocentric.latitude).first
        latitude_rate = earth_relative_velocity.vector.dot(north) / radius
    else:
        latitude_rate = north_velocity / denominator
    altitude_rate = -earth_relative_velocity.vector.dot(basis.third)
    return GeodeticStateRates(latitude_rate, altitude_rate)
####


def _require_position_and_velocity(position: FrameQuantityVector3, velocity: FrameVector3) -> None:
    if position.frame is not Frame.ECFC:
        raise ValueError("position must be expressed in ECFC")
    if position.unit.dimension != "length":
        raise ValueError("position must have length units")
    if velocity.frame is not Frame.ECFC:
        raise ValueError("earth-relative velocity must be expressed in ECFC")
    ####
