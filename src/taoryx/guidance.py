"""Typed guidance-transition bindings."""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
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
from .numeric import Function, NewtonSystemResult


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


@dataclass(frozen=True, slots=True)
class RangeInsensitiveAxisResult:
    """Body-axis delta-v direction selected from IIP sensitivities."""

    yaw_radians: float
    pitch_radians: float
    delta_v: float
    position_sensitivity_norm: float
    time_sensitivity: float
####


@dataclass(frozen=True, slots=True)
class GuidanceControlClassification:
    """Selected direct-control set and remaining free controls."""

    selected_set: tuple[str, ...]
    direct_controls: tuple[str, ...]
    free_controls: tuple[str, ...]
####


def classify_guidance_rules(
    direct_controls: Sequence[str],
    control_sets: Sequence[Sequence[str]],
) -> GuidanceControlClassification:
    """Evaluate TAOS-ALG-GUID-001's lowest-numbered compatible control set."""

    direct = tuple(dict.fromkeys(name.casefold() for name in direct_controls))
    normalized_sets = tuple(tuple(name.casefold() for name in control_set) for control_set in control_sets)
    selected = next((control_set for control_set in normalized_sets if set(direct).issubset(control_set)), None)
    if selected is None:
        raise ValueError("direct guidance controls are inconsistent with all control sets")
    return GuidanceControlClassification(selected, direct, tuple(name for name in selected if name not in direct))
####


def solve_guidance(
    residual_function: Function,
    initial_controls: Sequence[float],
    control_bounds: Sequence[tuple[float, float]],
    increments: Sequence[float] | float,
    residual_tolerance: float,
    *,
    max_iterations: int = 100,
    ) -> NewtonSystemResult:
    """Evaluate TAOS-ALG-GUID-002 through the typed Newton guidance contract."""

    from .numeric import newton_system

    return newton_system(
        residual_function,
        initial_controls,
        control_bounds,
        increments,
        residual_tolerance,
        max_iterations=max_iterations,
    )
####


def range_insensitive_axis(
    position_sensitivity: Sequence[tuple[float, float]],
    time_sensitivity: tuple[float, float],
    *,
    delta_v: float = 10.0,
) -> RangeInsensitiveAxisResult:
    """Evaluate TAOS-ALG-GUID-008 from IIP position/time sensitivities."""

    raw_jacobian = tuple(tuple(float(value) for value in row) for row in position_sensitivity)
    time_gradient = tuple(float(value) for value in time_sensitivity)
    if len(time_gradient) != 2 or not raw_jacobian or any(len(row) != 2 for row in raw_jacobian):
        raise ValueError("position and time sensitivities must have two control columns")
    jacobian = tuple((row[0], row[1]) for row in raw_jacobian)
    if delta_v <= 0.0 or not math.isfinite(delta_v):
        raise ValueError("delta_v must be positive and finite")
    candidates = tuple(2.0 * math.pi * index / 720.0 for index in range(720))
    best = min(
        candidates,
        key=lambda angle: (
            _sensitivity_norm(jacobian, (math.cos(angle), math.sin(angle)))
            / max(abs(time_gradient[0] * math.cos(angle) + time_gradient[1] * math.sin(angle)), 1e-15),
            -abs(time_gradient[0] * math.cos(angle) + time_gradient[1] * math.sin(angle)),
        ),
    )
    direction = (math.cos(best), math.sin(best))
    return RangeInsensitiveAxisResult(
        math.atan2(direction[1], direction[0]),
        0.0,
        delta_v,
        _sensitivity_norm(jacobian, direction),
        time_gradient[0] * direction[0] + time_gradient[1] * direction[1],
    )
####


@dataclass(frozen=True, slots=True)
class FlightPathLimit:
    """One named flight-condition limit and optional replacement rule."""

    variable: str
    lower: float | None = None
    upper: float | None = None
    replacement_rule: str | None = None
####


@dataclass(frozen=True, slots=True)
class FlightPathLimitResult:
    """Effective bounds and rules after applying active flight limits."""

    bounds: tuple[tuple[str, tuple[float | None, float | None]], ...]
    rules: tuple[str, ...]
####


def apply_flight_path_limits(
    active_rules: Sequence[str],
    free_control_bounds: Mapping[str, tuple[float | None, float | None]],
    limits: Sequence[FlightPathLimit],
    current_values: Mapping[str, float],
) -> FlightPathLimitResult:
    """Evaluate TAOS-ALG-GUID-009's bound/replacement-rule contract."""

    effective = {name.casefold(): bounds for name, bounds in free_control_bounds.items()}
    rules = list(active_rules)
    for limit in limits:
        value = current_values.get(limit.variable)
        if value is None:
            raise KeyError(f"missing current value for flight limit {limit.variable}")
        outside = (limit.lower is not None and value < limit.lower) or (limit.upper is not None and value > limit.upper)
        if outside and limit.replacement_rule is not None:
            rules.append(limit.replacement_rule)
        if limit.variable.casefold() in effective and not outside:
            lower, upper = effective[limit.variable.casefold()]
            effective[limit.variable.casefold()] = (
                limit.lower if lower is None else max(lower, limit.lower) if limit.lower is not None else lower,
                limit.upper if upper is None else min(upper, limit.upper) if limit.upper is not None else upper,
            )
    return FlightPathLimitResult(tuple(sorted(effective.items())), tuple(rules))
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


def _sensitivity_norm(jacobian: tuple[tuple[float, float], ...], direction: tuple[float, float]) -> float:
    return math.sqrt(sum((row[0] * direction[0] + row[1] * direction[1]) ** 2 for row in jacobian))
####
