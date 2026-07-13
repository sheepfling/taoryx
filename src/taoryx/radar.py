"""Typed radar and relative-vehicle observation bindings."""

from __future__ import annotations

from dataclasses import dataclass

from .equations.frames import BodyAxes
from .equations.geodesy import CartesianVector3
from .equations.radar import (
    radar_aspect_cosine,
    radar_aspect_sine,
    radar_azimuth_acceleration,
    radar_azimuth_elevation,
    radar_azimuth_rate,
    radar_elevation_acceleration,
    radar_elevation_rate,
    radar_line_of_sight_unit_vector,
    radar_meridional_cosine,
    radar_meridional_sine,
    radar_range_acceleration,
    radar_range_rate,
    radar_relative_position,
    relative_vehicle_azimuth,
    relative_vehicle_elevation,
)


@dataclass(frozen=True, slots=True)
class RadarObservationResult:
    """Radar range, angle derivatives, and body aspect diagnostics."""

    relative_position: CartesianVector3
    line_of_sight: CartesianVector3
    range: float
    range_rate: float
    range_acceleration: float
    azimuth_radians: float
    azimuth_rate_radians_per_second: float
    azimuth_acceleration_radians_per_second2: float
    elevation_radians: float
    elevation_rate_radians_per_second: float
    elevation_acceleration_radians_per_second2: float
    aspect_sine: float
    aspect_cosine: float
    meridional_sine: float
    meridional_cosine: float
####


def radar_observations(
    radar_position: CartesianVector3,
    station_longitude_radians: float,
    station_latitude_radians: float,
    vehicle_position: CartesianVector3,
    vehicle_velocity: CartesianVector3,
    vehicle_acceleration: CartesianVector3,
    body_axes: BodyAxes,
) -> RadarObservationResult:
    """Evaluate TAOS-ALG-RADAR-001 and equations 2-246 through 2-261."""

    relative_position = radar_relative_position(vehicle_position, radar_position)
    line_of_sight = radar_line_of_sight_unit_vector(relative_position)
    angles = radar_azimuth_elevation(station_longitude_radians, station_latitude_radians, relative_position)
    return RadarObservationResult(
        relative_position,
        line_of_sight,
        _norm(relative_position),
        radar_range_rate(relative_position, vehicle_velocity),
        radar_range_acceleration(relative_position, vehicle_velocity, vehicle_acceleration),
        angles.azimuth_radians,
        radar_azimuth_rate(station_longitude_radians, station_latitude_radians, relative_position, vehicle_velocity),
        radar_azimuth_acceleration(
            station_longitude_radians,
            station_latitude_radians,
            relative_position,
            vehicle_velocity,
            vehicle_acceleration,
        ),
        angles.elevation_radians,
        radar_elevation_rate(station_longitude_radians, station_latitude_radians, relative_position, vehicle_velocity),
        radar_elevation_acceleration(
            station_longitude_radians,
            station_latitude_radians,
            relative_position,
            vehicle_velocity,
            vehicle_acceleration,
        ),
        radar_aspect_sine(line_of_sight, body_axes),
        radar_aspect_cosine(line_of_sight, body_axes),
        radar_meridional_sine(line_of_sight, body_axes),
        radar_meridional_cosine(line_of_sight, body_axes),
    )
####


@dataclass(frozen=True, slots=True)
class RelativeVehicleObservation:
    """Relative range, body-frame angles, offsets, and closure velocity."""

    relative_position: CartesianVector3
    range: float
    azimuth_radians: float
    elevation_radians: float
    closure_velocity: float
####


def relative_vehicle_observations(
    current_position: CartesianVector3,
    current_velocity: CartesianVector3,
    target_position: CartesianVector3,
    target_velocity: CartesianVector3,
    body_axes: BodyAxes,
) -> RelativeVehicleObservation:
    """Evaluate TAOS-ALG-REL-001 and equations 2-262 through 2-265."""

    relative_position = radar_relative_position(target_position, current_position)
    relative_velocity = radar_relative_position(target_velocity, current_velocity)
    distance = _norm(relative_position)
    if distance == 0.0:
        raise ValueError("relative vehicle observation is undefined at zero range")
    return RelativeVehicleObservation(
        relative_position,
        distance,
        relative_vehicle_azimuth(body_axes, relative_position),
        relative_vehicle_elevation(body_axes, relative_position),
        -(
            relative_position.x * relative_velocity.x
            + relative_position.y * relative_velocity.y
            + relative_position.z * relative_velocity.z
        )
        / distance,
    )
####


def _norm(vector: CartesianVector3) -> float:
    return (vector.x * vector.x + vector.y * vector.y + vector.z * vector.z) ** 0.5
####
