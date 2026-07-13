"""Typed guidance-transition bindings."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .equations import (
    CartesianVector3,
    cubic_guidance_acceleration,
    cubic_guidance_coefficient_a,
    cubic_guidance_coefficient_b,
    line_of_sight_pitch,
    line_of_sight_pitch_rate,
    line_of_sight_yaw,
    line_of_sight_yaw_rate,
    parabolic_guidance_coefficient_a,
    parabolic_guidance_coefficient_b,
    parabolic_guidance_rate,
    proportional_navigation_closure_velocity,
    proportional_navigation_ecfc_acceleration,
    proportional_navigation_pitch_acceleration,
    proportional_navigation_yaw_acceleration,
)


def parabolic_guidance_correction(
    current_value: float,
    desired_value: float,
    desired_rate: float,
    current_time: float,
    guidance_interval: float,
) -> float:
    """Evaluate TAOS-ALG-GUID-003 and equations 2-273 through 2-276."""

    coefficient_a = parabolic_guidance_coefficient_a(current_value, desired_value, desired_rate, guidance_interval)
    coefficient_b = parabolic_guidance_coefficient_b(current_time, coefficient_a, desired_rate, guidance_interval)
    return parabolic_guidance_rate(current_time, coefficient_a, coefficient_b)
####


@dataclass(frozen=True, slots=True)
class PredictiveInterceptResult:
    """Predicted intercept state and ECFC line-of-sight guidance angles."""

    time_to_intercept_seconds: float
    intercept_point: CartesianVector3
    heading_radians: float
    flight_path_angle_radians: float
####


@dataclass(frozen=True, slots=True)
class ProportionalNavigationResult:
    """Line-of-sight state and ECFC acceleration command."""

    yaw_radians: float
    pitch_radians: float
    yaw_rate_radians_per_second: float
    pitch_rate_radians_per_second: float
    closure_velocity: float
    yaw_acceleration: float
    pitch_acceleration: float
    ecfc_acceleration: CartesianVector3
####


def proportional_navigation(
    interceptor_position: CartesianVector3,
    interceptor_velocity: CartesianVector3,
    target_position: CartesianVector3,
    target_velocity: CartesianVector3,
    navigation_constant: float,
) -> ProportionalNavigationResult:
    """Evaluate TAOS-ALG-GUID-007 and equations 2-287 through 2-297."""

    relative_position = CartesianVector3(
        target_position.x - interceptor_position.x,
        target_position.y - interceptor_position.y,
        target_position.z - interceptor_position.z,
    )
    relative_velocity = CartesianVector3(
        target_velocity.x - interceptor_velocity.x,
        target_velocity.y - interceptor_velocity.y,
        target_velocity.z - interceptor_velocity.z,
    )
    yaw = line_of_sight_yaw(relative_position)
    pitch = line_of_sight_pitch(relative_position)
    yaw_rate = line_of_sight_yaw_rate(relative_position, relative_velocity)
    pitch_rate = line_of_sight_pitch_rate(relative_position, relative_velocity)
    closure_velocity = proportional_navigation_closure_velocity(relative_position, relative_velocity)
    yaw_acceleration = proportional_navigation_yaw_acceleration(navigation_constant, closure_velocity, yaw_rate)
    pitch_acceleration = proportional_navigation_pitch_acceleration(navigation_constant, closure_velocity, pitch_rate)
    return ProportionalNavigationResult(
        yaw,
        pitch,
        yaw_rate,
        pitch_rate,
        closure_velocity,
        yaw_acceleration,
        pitch_acceleration,
        proportional_navigation_ecfc_acceleration(pitch, yaw, pitch_acceleration, yaw_acceleration),
    )
####


def predictive_intercept(
    interceptor_position: CartesianVector3,
    interceptor_velocity: CartesianVector3,
    target_position: CartesianVector3,
    target_velocity: CartesianVector3,
) -> PredictiveInterceptResult:
    """Evaluate TAOS-ALG-GUID-006 and equations 2-284 through 2-286."""

    from .equations import predictive_intercept_point, predictive_time_to_intercept

    time_to_intercept = predictive_time_to_intercept(
        interceptor_position,
        interceptor_velocity,
        target_position,
        target_velocity,
    )
    intercept_point = predictive_intercept_point(
        interceptor_position,
        interceptor_velocity,
        target_position,
        target_velocity,
    )
    relative = CartesianVector3(
        intercept_point.x - interceptor_position.x,
        intercept_point.y - interceptor_position.y,
        intercept_point.z - interceptor_position.z,
    )
    horizontal_distance = math.hypot(relative.x, relative.y)
    return PredictiveInterceptResult(
        time_to_intercept,
        intercept_point,
        math.atan2(relative.y, relative.x),
        math.atan2(relative.z, horizontal_distance),
    )
####


def cubic_guidance_correction(
    current_time: float,
    guidance_interval: float,
    current_value: float,
    desired_value: float,
    current_rate: float,
    desired_rate: float,
) -> float:
    """Evaluate TAOS-ALG-GUID-004 and equations 2-277 through 2-280."""

    coefficient_b = cubic_guidance_coefficient_b(
        current_time,
        guidance_interval,
        current_value,
        desired_value,
        current_rate,
        desired_rate,
    )
    coefficient_a = cubic_guidance_coefficient_a(
        current_time,
        guidance_interval,
        current_rate,
        desired_rate,
        coefficient_b,
    )
    return cubic_guidance_acceleration(current_time, coefficient_a, coefficient_b)
####
