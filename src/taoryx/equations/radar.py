"""Radar observation helpers derived from the TAOS Chapter 2 formulas."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .frames import BodyAxes
from .geodesy import CartesianVector3, geodetic_unit_vectors


@dataclass(frozen=True, slots=True)
class RadarObservation:
    """Basic radar observation geometry."""

    relative_position: CartesianVector3
    line_of_sight: CartesianVector3
    range: float
    azimuth_radians: float
    elevation_radians: float
####


@dataclass(frozen=True, slots=True)
class RadarAngles:
    """Radar azimuth and elevation angles in radians."""

    azimuth_radians: float
    elevation_radians: float
####


def vector_subtract(left: CartesianVector3, right: CartesianVector3) -> CartesianVector3:
    return CartesianVector3(left.x - right.x, left.y - right.y, left.z - right.z)
####


def vector_norm(vector: CartesianVector3) -> float:
    return math.sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z)
####


def vector_scale(vector: CartesianVector3, scale: float) -> CartesianVector3:
    return CartesianVector3(vector.x * scale, vector.y * scale, vector.z * scale)
####


def radar_relative_position(vehicle_position: CartesianVector3, radar_position: CartesianVector3) -> CartesianVector3:
    """Return the vehicle position relative to the radar station."""

    return vector_subtract(vehicle_position, radar_position)
####


def radar_line_of_sight_unit_vector(relative_position: CartesianVector3) -> CartesianVector3:
    """Return the line-of-sight unit vector from the station to the vehicle."""

    distance = vector_norm(relative_position)
    if distance == 0.0:
        raise ValueError("radar line of sight is undefined at zero range")
    ####
    return vector_scale(relative_position, 1.0 / distance)
####


def radar_azimuth_elevation(
    station_longitude_radians: float,
    station_latitude_radians: float,
    relative_position: CartesianVector3,
) -> RadarAngles:
    """Compute radar azimuth and elevation from the station-local basis."""

    north, east, down = geodetic_unit_vectors(station_longitude_radians, station_latitude_radians)
    x_component = relative_position.x * north.x + relative_position.y * north.y + relative_position.z * north.z
    y_component = relative_position.x * east.x + relative_position.y * east.y + relative_position.z * east.z
    z_component = relative_position.x * down.x + relative_position.y * down.y + relative_position.z * down.z
    range_distance = vector_norm(relative_position)
    if range_distance == 0.0:
        raise ValueError("radar azimuth and elevation are undefined at zero range")
    ####
    azimuth = math.atan2(y_component, x_component)
    elevation = math.asin(-z_component / range_distance)
    return RadarAngles(azimuth, elevation)
####


def radar_observation(
    vehicle_position: CartesianVector3,
    radar_position: CartesianVector3,
    station_longitude_radians: float,
    station_latitude_radians: float,
) -> RadarObservation:
    """Compute the core radar observation quantities."""

    relative_position = radar_relative_position(vehicle_position, radar_position)
    line_of_sight = radar_line_of_sight_unit_vector(relative_position)
    range_distance = vector_norm(relative_position)
    angles = radar_azimuth_elevation(station_longitude_radians, station_latitude_radians, relative_position)
    return RadarObservation(relative_position, line_of_sight, range_distance, angles.azimuth_radians, angles.elevation_radians)
####


def radar_range_rate(relative_position: CartesianVector3, velocity: CartesianVector3) -> float:
    """Return the line-of-sight range rate from TAOS equation 2-248."""

    return (
        relative_position.x * velocity.x
        + relative_position.y * velocity.y
        + relative_position.z * velocity.z
    ) / vector_norm(relative_position)
####


def radar_range_acceleration(
    relative_position: CartesianVector3,
    velocity: CartesianVector3,
    acceleration: CartesianVector3,
) -> float:
    """Return the line-of-sight range acceleration."""

    range_distance = vector_norm(relative_position)
    if range_distance == 0.0:
        raise ValueError("radar range acceleration is undefined at zero range")
    ####
    unit_vector = radar_line_of_sight_unit_vector(relative_position)
    unit_vector_rate = radar_unit_vector_rate(relative_position, velocity)
    return acceleration.x * unit_vector.x + acceleration.y * unit_vector.y + acceleration.z * unit_vector.z + (
        velocity.x * unit_vector_rate.x
        + velocity.y * unit_vector_rate.y
        + velocity.z * unit_vector_rate.z
    )
####


def radar_unit_vector_rate(
    relative_position: CartesianVector3,
    velocity: CartesianVector3,
) -> CartesianVector3:
    """Return the line-of-sight unit-vector rate."""

    range_distance = vector_norm(relative_position)
    if range_distance == 0.0:
        raise ValueError("radar unit-vector rate is undefined at zero range")
    ####
    unit_vector = radar_line_of_sight_unit_vector(relative_position)
    range_rate = radar_range_rate(relative_position, velocity)
    return CartesianVector3(
        (velocity.x - range_rate * unit_vector.x) / range_distance,
        (velocity.y - range_rate * unit_vector.y) / range_distance,
        (velocity.z - range_rate * unit_vector.z) / range_distance,
    )
####


def radar_unit_vector_acceleration(
    relative_position: CartesianVector3,
    velocity: CartesianVector3,
    acceleration: CartesianVector3,
) -> CartesianVector3:
    """Return the line-of-sight unit-vector acceleration."""

    range_distance = vector_norm(relative_position)
    if range_distance == 0.0:
        raise ValueError("radar unit-vector acceleration is undefined at zero range")
    ####
    unit_vector = radar_line_of_sight_unit_vector(relative_position)
    range_rate = radar_range_rate(relative_position, velocity)
    unit_vector_rate = radar_unit_vector_rate(relative_position, velocity)
    range_acceleration = radar_range_acceleration(relative_position, velocity, acceleration)
    return CartesianVector3(
        (acceleration.x - range_acceleration * unit_vector.x - 2.0 * range_rate * unit_vector_rate.x) / range_distance,
        (acceleration.y - range_acceleration * unit_vector.y - 2.0 * range_rate * unit_vector_rate.y) / range_distance,
        (acceleration.z - range_acceleration * unit_vector.z - 2.0 * range_rate * unit_vector_rate.z) / range_distance,
    )
####


def radar_azimuth_rate(
    station_longitude_radians: float,
    station_latitude_radians: float,
    relative_position: CartesianVector3,
    velocity: CartesianVector3,
) -> float:
    """Return the radar azimuth rate."""

    north, east, _ = geodetic_unit_vectors(station_longitude_radians, station_latitude_radians)
    x_component = relative_position.x * north.x + relative_position.y * north.y + relative_position.z * north.z
    y_component = relative_position.x * east.x + relative_position.y * east.y + relative_position.z * east.z
    vx = velocity.x * north.x + velocity.y * north.y + velocity.z * north.z
    vy = velocity.x * east.x + velocity.y * east.y + velocity.z * east.z
    denominator = x_component * x_component + y_component * y_component
    if denominator == 0.0:
        return 0.0
    ####
    return (vy * x_component - vx * y_component) / denominator
####


def radar_azimuth_acceleration(
    station_longitude_radians: float,
    station_latitude_radians: float,
    relative_position: CartesianVector3,
    velocity: CartesianVector3,
    acceleration: CartesianVector3,
) -> float:
    """Return the radar azimuth acceleration."""

    north, east, _ = geodetic_unit_vectors(station_longitude_radians, station_latitude_radians)
    x_component = relative_position.x * north.x + relative_position.y * north.y + relative_position.z * north.z
    y_component = relative_position.x * east.x + relative_position.y * east.y + relative_position.z * east.z
    vx = velocity.x * north.x + velocity.y * north.y + velocity.z * north.z
    vy = velocity.x * east.x + velocity.y * east.y + velocity.z * east.z
    ax = acceleration.x * north.x + acceleration.y * north.y + acceleration.z * north.z
    ay = acceleration.x * east.x + acceleration.y * east.y + acceleration.z * east.z
    denominator = x_component * x_component + y_component * y_component
    if denominator == 0.0:
        return 0.0
    ####
    azimuth_rate = radar_azimuth_rate(station_longitude_radians, station_latitude_radians, relative_position, velocity)
    return (
        (ay * x_component - ax * y_component) / denominator
        - 2.0 * azimuth_rate * (vx * x_component + vy * y_component) / denominator
    )
####


def radar_elevation_rate(
    station_longitude_radians: float,
    station_latitude_radians: float,
    relative_position: CartesianVector3,
    velocity: CartesianVector3,
) -> float:
    """Return the radar elevation rate."""

    _, _, down = geodetic_unit_vectors(station_longitude_radians, station_latitude_radians)
    z_component = relative_position.x * down.x + relative_position.y * down.y + relative_position.z * down.z
    range_distance = vector_norm(relative_position)
    if range_distance == 0.0:
        return 0.0
    ####
    unit_vector_rate = radar_unit_vector_rate(relative_position, velocity)
    sine_elevation = -z_component / range_distance
    cosine_elevation = math.sqrt(max(0.0, 1.0 - sine_elevation * sine_elevation))
    if cosine_elevation == 0.0:
        return 0.0
    ####
    return -(
        unit_vector_rate.x * down.x
        + unit_vector_rate.y * down.y
        + unit_vector_rate.z * down.z
    ) / cosine_elevation
####


def radar_elevation_acceleration(
    station_longitude_radians: float,
    station_latitude_radians: float,
    relative_position: CartesianVector3,
    velocity: CartesianVector3,
    acceleration: CartesianVector3,
) -> float:
    """Return the radar elevation acceleration."""

    _, _, down = geodetic_unit_vectors(station_longitude_radians, station_latitude_radians)
    range_distance = vector_norm(relative_position)
    if range_distance == 0.0:
        return 0.0
    ####
    unit_vector = radar_line_of_sight_unit_vector(relative_position)
    unit_vector_acceleration = radar_unit_vector_acceleration(relative_position, velocity, acceleration)
    sine_elevation = -(unit_vector.x * down.x + unit_vector.y * down.y + unit_vector.z * down.z)
    cosine_elevation = math.sqrt(max(0.0, 1.0 - sine_elevation * sine_elevation))
    if cosine_elevation == 0.0:
        return 0.0
    ####
    rate = radar_elevation_rate(station_longitude_radians, station_latitude_radians, relative_position, velocity)
    return -(
        unit_vector_acceleration.x * down.x
        + unit_vector_acceleration.y * down.y
        + unit_vector_acceleration.z * down.z
    ) / cosine_elevation + rate * rate * sine_elevation / cosine_elevation
####


def radar_aspect_sine(line_of_sight: CartesianVector3, body_axes: BodyAxes) -> float:
    """Return the radar aspect-angle sine."""

    return math.sqrt(
        (line_of_sight.x * body_axes.y.x + line_of_sight.y * body_axes.y.y + line_of_sight.z * body_axes.y.z) ** 2
        + (line_of_sight.x * body_axes.z.x + line_of_sight.y * body_axes.z.y + line_of_sight.z * body_axes.z.z) ** 2
    )
####


def radar_aspect_cosine(line_of_sight: CartesianVector3, body_axes: BodyAxes) -> float:
    """Return the radar aspect-angle cosine."""

    return -(
        line_of_sight.x * body_axes.x.x
        + line_of_sight.y * body_axes.x.y
        + line_of_sight.z * body_axes.x.z
    )
####


def radar_meridional_sine(line_of_sight: CartesianVector3, body_axes: BodyAxes) -> float:
    """Return the radar meridional-angle sine."""

    denominator = math.sqrt(
        (line_of_sight.x * body_axes.y.x + line_of_sight.y * body_axes.y.y + line_of_sight.z * body_axes.y.z) ** 2
        + (line_of_sight.x * body_axes.z.x + line_of_sight.y * body_axes.z.y + line_of_sight.z * body_axes.z.z) ** 2
    )
    if denominator == 0.0:
        return 0.0
    ####
    return -(
        line_of_sight.x * body_axes.z.x
        + line_of_sight.y * body_axes.z.y
        + line_of_sight.z * body_axes.z.z
    ) / denominator
####


def radar_meridional_cosine(line_of_sight: CartesianVector3, body_axes: BodyAxes) -> float:
    """Return the radar meridional-angle cosine."""

    denominator = math.sqrt(
        (line_of_sight.x * body_axes.y.x + line_of_sight.y * body_axes.y.y + line_of_sight.z * body_axes.y.z) ** 2
        + (line_of_sight.x * body_axes.z.x + line_of_sight.y * body_axes.z.y + line_of_sight.z * body_axes.z.z) ** 2
    )
    if denominator == 0.0:
        return 0.0
    ####
    return -(
        line_of_sight.x * body_axes.y.x
        + line_of_sight.y * body_axes.y.y
        + line_of_sight.z * body_axes.y.z
    ) / denominator
####


def relative_vehicle_position(
    vehicle_position: CartesianVector3,
    current_vehicle_position: CartesianVector3,
) -> CartesianVector3:
    """Return the position of the i-th vehicle relative to the current vehicle."""

    return radar_relative_position(vehicle_position, current_vehicle_position)
####


def relative_vehicle_unit_vector(relative_position: CartesianVector3) -> CartesianVector3:
    """Return the relative line-of-sight unit vector."""

    return radar_line_of_sight_unit_vector(relative_position)
####


def relative_vehicle_azimuth(body_axes: BodyAxes, relative_position: CartesianVector3) -> float:
    """Return the relative vehicle azimuth."""

    x_component = relative_position.x * body_axes.x.x + relative_position.y * body_axes.x.y + relative_position.z * body_axes.x.z
    y_component = relative_position.x * body_axes.y.x + relative_position.y * body_axes.y.y + relative_position.z * body_axes.y.z
    return math.atan2(y_component, x_component)
####


def relative_vehicle_elevation(body_axes: BodyAxes, relative_position: CartesianVector3) -> float:
    """Return the relative vehicle elevation."""

    unit_vector = radar_line_of_sight_unit_vector(relative_position)
    return math.asin(
        -(
            unit_vector.x * body_axes.z.x
            + unit_vector.y * body_axes.z.y
            + unit_vector.z * body_axes.z.z
        )
    )
####
