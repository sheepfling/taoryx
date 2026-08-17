"""Focused math witnesses for shared interceptor waypoint guidance laws."""

from __future__ import annotations

import math

import pytest
from taoryx_parametric_interceptors import (
    evaluate_lateral_acceleration_tracking,
    evaluate_waypoint_guidance,
)


def test_lateral_tracking_normalizes_magnitude_and_requires_two_directions() -> None:
    aligned = evaluate_lateral_acceleration_tracking((3.0, 4.0, 0.0), (1.5, 2.0, 0.0))
    orthogonal = evaluate_lateral_acceleration_tracking((1.0, 0.0, 0.0), (0.0, 2.0, 0.0))
    opposite = evaluate_lateral_acceleration_tracking((1.0, 0.0, 0.0), (-1.0, 0.0, 0.0))
    unsupported = evaluate_lateral_acceleration_tracking((1.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    no_demand = evaluate_lateral_acceleration_tracking((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))

    assert aligned.command_magnitude_mps2 == 5.0
    assert aligned.achieved_magnitude_mps2 == 2.5
    assert aligned.achievement_fraction == 0.5
    assert aligned.direction_error_valid
    assert aligned.direction_error_rad == pytest.approx(0.0)
    assert orthogonal.achievement_fraction == 1.0
    assert orthogonal.direction_error_rad == pytest.approx(math.pi / 2.0)
    assert opposite.direction_error_rad == pytest.approx(math.pi)
    assert unsupported.achievement_fraction == 0.0
    assert not unsupported.direction_error_valid
    assert unsupported.direction_error_rad == 0.0
    assert no_demand.achievement_fraction == 1.0
    assert not no_demand.direction_error_valid
    assert no_demand.direction_error_rad == 0.0
    with pytest.raises(ValueError, match="must be finite"):
        evaluate_lateral_acceleration_tracking((math.nan, 0.0, 0.0), (0.0, 0.0, 0.0))
    ####


def test_pursuit_and_proportional_navigation_share_geometry_but_not_hidden_behavior() -> None:
    inputs = {
        "position_m": (0.0, 0.0, 0.0),
        "velocity_mps": (100.0, 0.0, 0.0),
        "waypoint_m": (1_000.0, 1_000.0, 0.0),
        "pursuit_time_constant_s": 0.5,
        "navigation_constant": 3.0,
    }
    pursuit = evaluate_waypoint_guidance(**inputs, archetype="waypoint_pursuit")
    proportional = evaluate_waypoint_guidance(**inputs, archetype="proportional_navigation")

    assert pursuit.mode == "waypoint_pursuit"
    assert proportional.mode == "proportional_navigation"
    assert pursuit.closing_speed_mps == pytest.approx(70.71067811865476)
    assert proportional.closing_speed_mps == pytest.approx(pursuit.closing_speed_mps)
    assert pursuit.line_of_sight_rate_rad_s == pytest.approx(0.05)
    assert proportional.line_of_sight_rate_rad_s == pytest.approx(0.05)
    assert pursuit.acceleration_mps2 == pytest.approx((0.0, 141.4213562373095, 0.0))
    assert proportional.acceleration_mps2 == pytest.approx((0.0, 10.606601717798213, 0.0))
    assert proportional.navigation_constant == 3.0
    ####


def test_proportional_navigation_reports_capture_fallback_for_opening_retarget() -> None:
    evaluation = evaluate_waypoint_guidance(
        (0.0, 0.0, 0.0),
        (-100.0, 0.0, 0.0),
        (1_000.0, 1_000.0, 0.0),
        archetype="proportional_navigation",
        pursuit_time_constant_s=0.5,
        navigation_constant=4.0,
    )

    assert evaluation.archetype == "proportional_navigation"
    assert evaluation.mode == "capture_fallback"
    assert evaluation.closing_speed_mps < 0.0
    assert evaluation.commanded_acceleration_mps2 == pytest.approx(141.4213562373095)
    ####


def test_moving_target_guidance_reports_relative_kinematics_and_linear_miss_prediction() -> None:
    evaluation = evaluate_waypoint_guidance(
        (0.0, 0.0, 0.0),
        (100.0, 0.0, 0.0),
        (1_000.0, 100.0, 0.0),
        waypoint_velocity_mps=(20.0, 10.0, 0.0),
        archetype="proportional_navigation",
        pursuit_time_constant_s=0.5,
        navigation_constant=3.0,
    )

    relative_velocity = (-80.0, 10.0, 0.0)
    displacement = (1_000.0, 100.0, 0.0)
    relative_speed_squared = sum(value**2 for value in relative_velocity)
    time_to_closest = -sum(a * b for a, b in zip(displacement, relative_velocity, strict=True)) / relative_speed_squared
    closest = tuple(displacement[index] + relative_velocity[index] * time_to_closest for index in range(3))

    assert evaluation.relative_velocity_mps == relative_velocity
    assert evaluation.closing_speed_mps == pytest.approx(79_000.0 / math.sqrt(1_010_000.0))
    assert evaluation.time_to_closest_approach_s == pytest.approx(time_to_closest)
    assert evaluation.predicted_miss_distance_m == pytest.approx(math.sqrt(sum(value**2 for value in closest)))
    assert evaluation.line_of_sight_rate_rad_s > 0.0
    ####


def test_zero_target_velocity_preserves_stationary_waypoint_results() -> None:
    common = {
        "position_m": (10.0, -20.0, 30.0),
        "velocity_mps": (100.0, 15.0, -5.0),
        "waypoint_m": (1_000.0, 200.0, 100.0),
        "archetype": "proportional_navigation",
        "pursuit_time_constant_s": 0.5,
        "navigation_constant": 3.0,
    }
    omitted = evaluate_waypoint_guidance(**common)
    explicit = evaluate_waypoint_guidance(**common, waypoint_velocity_mps=(0.0, 0.0, 0.0))

    assert explicit == omitted
    ####


def test_guidance_validation_rejects_unknown_law_and_nonpositive_tuning() -> None:
    common = ((0.0, 0.0, 0.0), (100.0, 0.0, 0.0), (1_000.0, 0.0, 0.0))
    with pytest.raises(ValueError, match="unsupported guidance archetype"):
        evaluate_waypoint_guidance(
            *common,
            archetype="magic",
            pursuit_time_constant_s=0.5,
            navigation_constant=3.0,
        )
    with pytest.raises(ValueError, match="navigation constant"):
        evaluate_waypoint_guidance(
            *common,
            archetype="proportional_navigation",
            pursuit_time_constant_s=0.5,
            navigation_constant=0.0,
        )
    ####
