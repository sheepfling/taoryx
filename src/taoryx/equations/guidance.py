"""Guidance, initial-impact-point, and intercept equation helpers."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .geodesy import CartesianVector3


@dataclass(frozen=True, slots=True)
class Matrix3x3:
    """Small 3x3 matrix with row vectors stored explicitly."""

    row0: CartesianVector3
    row1: CartesianVector3
    row2: CartesianVector3
####


def _dot(left: CartesianVector3, right: CartesianVector3) -> float:
    return left.x * right.x + left.y * right.y + left.z * right.z
####


def _add(left: CartesianVector3, right: CartesianVector3) -> CartesianVector3:
    return CartesianVector3(left.x + right.x, left.y + right.y, left.z + right.z)
####


def _subtract(left: CartesianVector3, right: CartesianVector3) -> CartesianVector3:
    return CartesianVector3(left.x - right.x, left.y - right.y, left.z - right.z)
####


def _scale(vector: CartesianVector3, factor: float) -> CartesianVector3:
    return CartesianVector3(vector.x * factor, vector.y * factor, vector.z * factor)
####


def _norm(vector: CartesianVector3) -> float:
    return math.sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z)
####


def _matrix_vector_product(matrix: Matrix3x3, vector: CartesianVector3) -> CartesianVector3:
    return CartesianVector3(_dot(matrix.row0, vector), _dot(matrix.row1, vector), _dot(matrix.row2, vector))
####


def ballistic_coefficient(weight: float, drag_coefficient: float, reference_area: float) -> float:
    """Return the ballistic coefficient from TAOS equation 2-271."""

    return weight / (drag_coefficient * reference_area)
####


def lift_to_drag_ratio(lift_coefficient: float, drag_coefficient: float) -> float:
    """Return the lift-to-drag ratio from TAOS equation 2-272."""

    return lift_coefficient / drag_coefficient
####


def iip_wind_velocity_unit_vector(air_relative_velocity: CartesianVector3) -> CartesianVector3:
    """Return the unit vector along the air-relative velocity."""

    speed = _norm(air_relative_velocity)
    if speed == 0.0:
        raise ValueError("wind velocity unit vector is undefined at zero speed")
    ####
    return _scale(air_relative_velocity, 1.0 / speed)
####


def iip_aerodynamic_acceleration_from_drag_coefficient(
    dynamic_pressure: float,
    gravity_acceleration: float,
    drag_coefficient: float,
    reference_area: float,
    weight: float,
    wind_velocity_unit_vector: CartesianVector3,
) -> CartesianVector3:
    """Return the IIP aerodynamic acceleration from drag coefficient terms."""

    scale = drag_coefficient * dynamic_pressure * reference_area * gravity_acceleration / weight
    return _scale(wind_velocity_unit_vector, scale)
####


def iip_aerodynamic_acceleration_from_ballistic_coefficient(
    dynamic_pressure: float,
    gravity_acceleration: float,
    ballistic_coefficient_value: float,
    wind_velocity_unit_vector: CartesianVector3,
) -> CartesianVector3:
    """Return the IIP aerodynamic acceleration from the ballistic coefficient."""

    scale = dynamic_pressure * gravity_acceleration / ballistic_coefficient_value
    return _scale(wind_velocity_unit_vector, scale)
####


def iip_aerodynamic_acceleration_from_dynamic_pressure(
    gravity_acceleration: float,
    air_density: float,
    ballistic_coefficient_value: float,
    air_relative_velocity: CartesianVector3,
) -> CartesianVector3:
    """Return the IIP aerodynamic acceleration using dynamic pressure."""

    speed = _norm(air_relative_velocity)
    if speed == 0.0:
        raise ValueError("IIP aerodynamic acceleration is undefined at zero air-relative speed")
    ####
    scale = 0.5 * gravity_acceleration * air_density * speed / ballistic_coefficient_value
    return _scale(air_relative_velocity, scale)
####


def iip_aerodynamic_acceleration_from_density(
    gravity_acceleration: float,
    air_density: float,
    ballistic_coefficient_value: float,
    air_relative_velocity: CartesianVector3,
) -> CartesianVector3:
    """Return the IIP aerodynamic acceleration using air density."""

    return iip_aerodynamic_acceleration_from_dynamic_pressure(
        gravity_acceleration,
        air_density,
        ballistic_coefficient_value,
        air_relative_velocity,
    )
####


def iip_aerodynamic_acceleration_vector(
    gravity_acceleration: float,
    air_density: float,
    ballistic_coefficient_value: float,
    air_relative_velocity: CartesianVector3,
) -> CartesianVector3:
    """Return the vector form of the IIP aerodynamic acceleration."""

    return iip_aerodynamic_acceleration_from_dynamic_pressure(
        gravity_acceleration,
        air_density,
        ballistic_coefficient_value,
        air_relative_velocity,
    )
####


def predictive_intercept_position_equality(
    interceptor_position: CartesianVector3,
    interceptor_velocity: CartesianVector3,
    target_position: CartesianVector3,
    target_velocity: CartesianVector3,
    intercept_time_seconds: float,
    current_time_seconds: float,
) -> tuple[CartesianVector3, CartesianVector3]:
    """Return both sides of the nonaccelerating intercept equality."""

    delta_t = intercept_time_seconds - current_time_seconds
    left = _add(interceptor_position, _scale(interceptor_velocity, delta_t))
    right = _add(target_position, _scale(target_velocity, delta_t))
    return left, right
####


def predictive_time_to_intercept(
    interceptor_position: CartesianVector3,
    interceptor_velocity: CartesianVector3,
    target_position: CartesianVector3,
    target_velocity: CartesianVector3,
) -> float:
    """Return the estimated time to intercept."""

    relative_position = _subtract(target_position, interceptor_position)
    relative_velocity = _subtract(interceptor_velocity, target_velocity)
    range_distance = _norm(relative_position)
    speed = _norm(relative_velocity)
    if speed == 0.0:
        raise ValueError("time to intercept is undefined when the relative speed is zero")
    ####
    return range_distance / speed
####


def predictive_intercept_point(
    interceptor_position: CartesianVector3,
    interceptor_velocity: CartesianVector3,
    target_position: CartesianVector3,
    target_velocity: CartesianVector3,
) -> CartesianVector3:
    """Return the predicted intercept point."""

    delta_t = predictive_time_to_intercept(
        interceptor_position,
        interceptor_velocity,
        target_position,
        target_velocity,
    )
    return _add(target_position, _scale(target_velocity, delta_t))
####


def line_of_sight_yaw(relative_position: CartesianVector3) -> float:
    """Return the line-of-sight yaw angle."""

    return math.atan2(relative_position.y, relative_position.x)
####


def line_of_sight_pitch(relative_position: CartesianVector3) -> float:
    """Return the line-of-sight pitch angle."""

    return math.atan2(relative_position.z, math.hypot(relative_position.x, relative_position.y))
####


def line_of_sight_yaw_rate(relative_position: CartesianVector3, relative_velocity: CartesianVector3) -> float:
    """Return the line-of-sight yaw rate."""

    denominator = relative_position.x * relative_position.x + relative_position.y * relative_position.y
    if denominator == 0.0:
        return 0.0
    ####
    return (
        relative_position.x * relative_velocity.y - relative_position.y * relative_velocity.x
    ) / denominator
####


def line_of_sight_pitch_rate(relative_position: CartesianVector3, relative_velocity: CartesianVector3) -> float:
    """Return the line-of-sight pitch rate."""

    horizontal_sq = relative_position.x * relative_position.x + relative_position.y * relative_position.y
    if horizontal_sq == 0.0:
        return 0.0
    ####
    horizontal = math.sqrt(horizontal_sq)
    range_sq = horizontal_sq + relative_position.z * relative_position.z
    return (
        horizontal_sq * relative_velocity.z
        - relative_position.z * (relative_position.x * relative_velocity.x + relative_position.y * relative_velocity.y)
    ) / (horizontal * range_sq)
####


def proportional_navigation_yaw_acceleration(
    navigation_constant: float,
    closure_velocity: float,
    yaw_rate: float,
) -> float:
    """Return the proportional-navigation yaw acceleration."""

    return navigation_constant * closure_velocity * yaw_rate
####


def proportional_navigation_pitch_acceleration(
    navigation_constant: float,
    closure_velocity: float,
    pitch_rate: float,
) -> float:
    """Return the proportional-navigation pitch acceleration."""

    return navigation_constant * closure_velocity * pitch_rate
####


def proportional_navigation_closure_velocity(
    relative_position: CartesianVector3,
    relative_velocity: CartesianVector3,
) -> float:
    """Return the proportional-navigation closure velocity."""

    range_distance = _norm(relative_position)
    if range_distance == 0.0:
        raise ValueError("closure velocity is undefined at zero range")
    ####
    return -_dot(relative_velocity, relative_position) / range_distance
####


def source_los_pitch_rotation_matrix(pitch_radians: float) -> Matrix3x3:
    """Return the first printed line-of-sight rotation matrix."""

    cosine_pitch = math.cos(pitch_radians)
    sine_pitch = math.sin(pitch_radians)
    return Matrix3x3(
        CartesianVector3(cosine_pitch, sine_pitch, 0.0),
        CartesianVector3(-sine_pitch, cosine_pitch, 0.0),
        CartesianVector3(0.0, 0.0, 1.0),
    )
####


def source_los_yaw_rotation_matrix(yaw_radians: float) -> Matrix3x3:
    """Return the second printed line-of-sight rotation matrix."""

    cosine_yaw = math.cos(yaw_radians)
    sine_yaw = math.sin(yaw_radians)
    return Matrix3x3(
        CartesianVector3(cosine_yaw, 0.0, sine_yaw),
        CartesianVector3(0.0, 1.0, 0.0),
        CartesianVector3(-sine_yaw, 0.0, cosine_yaw),
    )
####


def los_to_ecfc_acceleration_matrix(pitch_radians: float, yaw_radians: float) -> Matrix3x3:
    """Return the transposed LOS-to-ECFC acceleration matrix."""

    cosine_pitch = math.cos(pitch_radians)
    sine_pitch = math.sin(pitch_radians)
    cosine_yaw = math.cos(yaw_radians)
    sine_yaw = math.sin(yaw_radians)
    return Matrix3x3(
        CartesianVector3(cosine_pitch * cosine_yaw, -sine_pitch * cosine_yaw, -sine_yaw),
        CartesianVector3(sine_pitch, cosine_pitch, 0.0),
        CartesianVector3(cosine_pitch * sine_yaw, -sine_pitch * sine_yaw, cosine_yaw),
    )
####


def proportional_navigation_ecfc_acceleration(
    pitch_radians: float,
    yaw_radians: float,
    pitch_acceleration: float,
    yaw_acceleration: float,
) -> CartesianVector3:
    """Return the commanded ECFC acceleration components."""

    matrix = los_to_ecfc_acceleration_matrix(pitch_radians, yaw_radians)
    return _matrix_vector_product(matrix, CartesianVector3(0.0, pitch_acceleration, yaw_acceleration))
####
