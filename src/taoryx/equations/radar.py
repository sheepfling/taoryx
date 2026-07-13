"""Radar observation helpers derived from the TAOS Chapter 2 formulas."""

from __future__ import annotations

import math
from dataclasses import dataclass

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
