"""Shared reduced-order waypoint guidance laws for interceptor runtime tiers."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Literal

GuidanceArchetype = Literal["waypoint_pursuit", "proportional_navigation"]
GuidanceRuntimeLaw = Literal[
    "waypoint_pursuit",
    "proportional_navigation",
    "external_lateral_acceleration",
]
GuidanceMode = Literal[
    "waypoint_pursuit",
    "proportional_navigation",
    "capture_fallback",
    "target_track_unavailable",
    "direct_lateral_acceleration",
]


@dataclass(frozen=True, slots=True)
class WaypointGuidanceEvaluation:
    """One unbounded lateral-acceleration demand plus guidance diagnostics."""

    archetype: GuidanceRuntimeLaw
    mode: GuidanceMode
    acceleration_mps2: tuple[float, float, float]
    commanded_acceleration_mps2: float
    closing_speed_mps: float
    line_of_sight_rate_rad_s: float
    navigation_constant: float
    relative_velocity_mps: tuple[float, float, float]
    time_to_closest_approach_s: float
    predicted_miss_distance_m: float

    ####


@dataclass(frozen=True, slots=True)
class LateralAccelerationTracking:
    """Normalized magnitude and direction tracking for one lateral command."""

    command_magnitude_mps2: float
    achieved_magnitude_mps2: float
    achievement_fraction: float
    direction_error_valid: bool
    direction_error_rad: float

    ####


def evaluate_lateral_acceleration_tracking(
    command_vector_mps2: tuple[float, float, float],
    achieved_vector_mps2: tuple[float, float, float],
) -> LateralAccelerationTracking:
    """Compare achieved lateral acceleration with its guidance-law request.

    A zero command has no direction to track and reports achievement one only
    when achieved acceleration is also zero. A nonzero command with zero
    achievement reports fraction zero and an invalid, zero-valued direction
    error. The explicit validity flag prevents zero from being mistaken for
    perfect angular alignment.
    """

    if any(not math.isfinite(value) for value in (*command_vector_mps2, *achieved_vector_mps2)):
        raise ValueError("lateral-acceleration tracking vectors must be finite")
    command = _norm(command_vector_mps2)
    achieved = _norm(achieved_vector_mps2)
    command_valid = command > 1.0e-12
    achieved_valid = achieved > 1.0e-12
    direction_valid = command_valid and achieved_valid
    if direction_valid:
        cosine = _dot(command_vector_mps2, achieved_vector_mps2) / (command * achieved)
        direction_error = math.acos(min(max(cosine, -1.0), 1.0))
    else:
        direction_error = 0.0
    if command_valid:
        achievement_fraction = min(achieved / command, 1.0)
    else:
        achievement_fraction = 1.0 if not achieved_valid else 0.0
    return LateralAccelerationTracking(
        command_magnitude_mps2=command,
        achieved_magnitude_mps2=achieved,
        achievement_fraction=achievement_fraction,
        direction_error_valid=direction_valid,
        direction_error_rad=direction_error,
    )
    ####


def evaluate_waypoint_guidance(
    position_m: tuple[float, float, float],
    velocity_mps: tuple[float, float, float],
    waypoint_m: tuple[float, float, float],
    *,
    waypoint_velocity_mps: tuple[float, float, float] = (0.0, 0.0, 0.0),
    archetype: GuidanceArchetype | str,
    pursuit_time_constant_s: float,
    navigation_constant: float,
) -> WaypointGuidanceEvaluation:
    """Evaluate pursuit or target-relative proportional-navigation steering.

    Proportional navigation uses the inertial line-of-sight rate and positive
    relative closing speed. The default zero target velocity exactly preserves
    stationary-waypoint behavior. A non-closing geometry uses the pursuit law
    as an explicit capture fallback so live retargeting cannot strand the
    surrogate with zero steering demand.
    """

    if archetype == "waypoint_pursuit":
        selected_archetype: GuidanceArchetype = "waypoint_pursuit"
    elif archetype == "proportional_navigation":
        selected_archetype = "proportional_navigation"
    else:
        raise ValueError(f"unsupported guidance archetype {archetype!r}")
    if not math.isfinite(pursuit_time_constant_s) or pursuit_time_constant_s <= 0.0:
        raise ValueError("pursuit time constant must be finite and positive")
    if not math.isfinite(navigation_constant) or navigation_constant <= 0.0:
        raise ValueError("navigation constant must be finite and positive")
    if any(not math.isfinite(value) for value in (*position_m, *velocity_mps, *waypoint_m, *waypoint_velocity_mps)):
        raise ValueError("guidance position, velocity, waypoint, and target-velocity values must be finite")

    displacement = _subtract(waypoint_m, position_m)
    range_m = _norm(displacement)
    speed_mps = _norm(velocity_mps)
    line_of_sight = _unit(displacement, fallback=(1.0, 0.0, 0.0))
    direction = _unit(velocity_mps, fallback=line_of_sight)
    pursuit = direction_response_acceleration(
        direction,
        line_of_sight,
        speed_mps=speed_mps,
        time_constant_s=pursuit_time_constant_s,
    )
    relative_velocity = _subtract(waypoint_velocity_mps, velocity_mps)
    closing_speed = -_dot(relative_velocity, line_of_sight)
    if range_m <= 1.0e-12:
        line_of_sight_rate = (0.0, 0.0, 0.0)
    else:
        line_of_sight_rate = _scaled(
            _cross(displacement, relative_velocity),
            1.0 / range_m**2,
        )
    relative_speed_squared = _dot(relative_velocity, relative_velocity)
    time_to_closest_approach = 0.0 if relative_speed_squared <= 1.0e-12 else max(-_dot(displacement, relative_velocity) / relative_speed_squared, 0.0)
    closest_displacement = (
        displacement[0] + relative_velocity[0] * time_to_closest_approach,
        displacement[1] + relative_velocity[1] * time_to_closest_approach,
        displacement[2] + relative_velocity[2] * time_to_closest_approach,
    )

    selected: tuple[float, float, float]
    mode: GuidanceMode
    if selected_archetype == "waypoint_pursuit":
        selected = pursuit
        mode = "waypoint_pursuit"
    else:
        proportional = _scaled(
            _cross(line_of_sight_rate, direction),
            navigation_constant * max(closing_speed, 0.0),
        )
        proportional = _lateral_component(proportional, direction)
        if closing_speed <= 1.0e-9 and _norm(pursuit) > 1.0e-12:
            selected = pursuit
            mode = "capture_fallback"
        else:
            selected = proportional
            mode = "proportional_navigation"
    return WaypointGuidanceEvaluation(
        archetype=selected_archetype,
        mode=mode,
        acceleration_mps2=selected,
        commanded_acceleration_mps2=_norm(selected),
        closing_speed_mps=closing_speed,
        line_of_sight_rate_rad_s=_norm(line_of_sight_rate),
        navigation_constant=navigation_constant,
        relative_velocity_mps=relative_velocity,
        time_to_closest_approach_s=time_to_closest_approach,
        predicted_miss_distance_m=_norm(closest_displacement),
    )
    ####


def evaluate_direct_lateral_acceleration(
    command_vector_mps2: tuple[float, float, float],
    *,
    reference_direction: tuple[float, float, float],
) -> WaypointGuidanceEvaluation:
    """Project one external local-NEU request onto the transverse plane.

    The direct-control grammar commands the reduced-order lateral-force input,
    not thrust or longitudinal acceleration. Projection is evaluated against
    the current velocity-aligned direction, so an obsolete held command cannot
    silently become an axial force as the vehicle turns.
    """

    if any(not math.isfinite(value) for value in (*command_vector_mps2, *reference_direction)):
        raise ValueError("direct lateral-acceleration command and reference direction must be finite")
    direction = _unit(reference_direction, fallback=(1.0, 0.0, 0.0))
    lateral = _lateral_component(command_vector_mps2, direction)
    return WaypointGuidanceEvaluation(
        archetype="external_lateral_acceleration",
        mode="direct_lateral_acceleration",
        acceleration_mps2=lateral,
        commanded_acceleration_mps2=_norm(lateral),
        closing_speed_mps=0.0,
        line_of_sight_rate_rad_s=0.0,
        navigation_constant=0.0,
        relative_velocity_mps=(0.0, 0.0, 0.0),
        time_to_closest_approach_s=0.0,
        predicted_miss_distance_m=0.0,
    )
    ####


def direction_response_acceleration(
    direction: tuple[float, float, float],
    requested_direction: tuple[float, float, float],
    *,
    speed_mps: float,
    time_constant_s: float,
) -> tuple[float, float, float]:
    """Return the pursuit-style lateral response between two unit directions."""

    raw = (
        (requested_direction[0] - direction[0]) * speed_mps / time_constant_s,
        (requested_direction[1] - direction[1]) * speed_mps / time_constant_s,
        (requested_direction[2] - direction[2]) * speed_mps / time_constant_s,
    )
    return _lateral_component(raw, direction)
    ####


def unavailable_target_track_guidance(
    *,
    archetype: GuidanceArchetype,
    navigation_constant: float,
) -> WaypointGuidanceEvaluation:
    """Return an explicit zero-demand state when no target packet is valid."""

    return WaypointGuidanceEvaluation(
        archetype=archetype,
        mode="target_track_unavailable",
        acceleration_mps2=(0.0, 0.0, 0.0),
        commanded_acceleration_mps2=0.0,
        closing_speed_mps=0.0,
        line_of_sight_rate_rad_s=0.0,
        navigation_constant=navigation_constant,
        relative_velocity_mps=(0.0, 0.0, 0.0),
        time_to_closest_approach_s=0.0,
        predicted_miss_distance_m=0.0,
    )
    ####


def _lateral_component(
    value: tuple[float, float, float],
    direction: tuple[float, float, float],
) -> tuple[float, float, float]:
    longitudinal = _dot(value, direction)
    return tuple(value[index] - longitudinal * direction[index] for index in range(3))  # type: ignore[return-value]
    ####


def _subtract(
    left: tuple[float, float, float],
    right: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (left[0] - right[0], left[1] - right[1], left[2] - right[2])
    ####


def _dot(left: tuple[float, float, float], right: tuple[float, float, float]) -> float:
    return sum(a * b for a, b in zip(left, right, strict=True))
    ####


def _cross(
    left: tuple[float, float, float],
    right: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )
    ####


def _norm(value: tuple[float, float, float]) -> float:
    return math.sqrt(sum(item * item for item in value))
    ####


def _unit(
    value: tuple[float, float, float],
    *,
    fallback: tuple[float, float, float],
) -> tuple[float, float, float]:
    magnitude = _norm(value)
    return fallback if magnitude <= 1.0e-12 else tuple(item / magnitude for item in value)  # type: ignore[return-value]
    ####


def _scaled(value: tuple[float, float, float], factor: float) -> tuple[float, float, float]:
    return (value[0] * factor, value[1] * factor, value[2] * factor)
    ####


__all__ = [
    "GuidanceArchetype",
    "GuidanceMode",
    "LateralAccelerationTracking",
    "WaypointGuidanceEvaluation",
    "direction_response_acceleration",
    "evaluate_direct_lateral_acceleration",
    "evaluate_lateral_acceleration_tracking",
    "evaluate_waypoint_guidance",
    "unavailable_target_track_guidance",
]
####
